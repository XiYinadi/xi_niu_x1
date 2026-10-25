#!/usr/bin/env python3
"""laser_odom 的**纯函数层** —— 不 import rclpy,因此能 headless 单测。

分层理由与 `motor_control.py` / `motor_hw.py` 同一套路（本项目的既定纪律）:

    laser_odom_core.py  ← 纯数学。不碰 ROS,可单测。
    laser_odom.py       ← ROS 节点。订阅 /scan,发 /odom + odom→base_link TF。
    laser_odom_test.py  ← 离线走廊仿真。喂**合成**扫描进去,断言位姿误差。

⚠️⚠️ `icp_align`(点对点)当前**逐点等价于 laser_odom.py 的旧行为**
     (2026-09-27 抽取时如此)。刻意保留:**改行为的是 `icp_align_plane`**,
     而 `icp_align` 留着当**对照基线**,让 `laser_odom_test.py` 能同时跑两条路。

--------------------------------------------------------------------------
为什么**点对点 ICP 在走廊里是瞎的**,以及点对面为什么能修
--------------------------------------------------------------------------
2026-09-27 单帧实测(`icp_diag.py`):车在 15m×2m 走廊中段**前进 5.0cm**,
量程 5.0m,点对点 ICP 报出来 **0.37cm** —— 丢掉 93%。量程提到 8.0m 也只到 1.27cm。
**只有侧墙时是 0.00cm,且与量程完全无关。**

机理(两步,缺一不可):

  ① **对应关系沿墙滑**。侧墙上的点很密(1m 处相邻束间距 ≈ 8.7mm)。
     一个移动了 5cm 的点去找最近邻,找到的是**墙上另一个物理点**,
     距离只有几毫米 —— 比真对应(5cm)还近。于是几百个侧墙点
     一致投出「几乎没动」的票。
  ② **票数淹没**。端墙在 7.5m 处、视张角只有 0.267 rad ⇒ 约 34 个点;
     侧墙几百个点。真实信息被 **15:1 投掉**。

⇒ 所以提高 `max_range` 只是**多加几张真票**,改不动票数结构。
   **这是它不解决问题的原因**(与最初「端墙超出量程」的猜测不同 —— 那条已被实测否掉)。

**点对面 ICP 的结构性差别**:它的代价函数是「沿目标**法向**的距离」,
在**面内是平的** ⇒ 平行侧墙**主动弃权**,而不是投假票。于是沿走廊方向
只剩端墙/门框(它们的法向沿走廊轴)有话语权 ⇒ 位移被正确解出来。

⚠️ **别改成「逐级收紧对应点门限」** —— 那是反的:滑动的假对应(4mm)比
   真对应(50mm)**更近**,收紧门限**先扔掉真对应**。实测过,别回头。

⚠️ `max_range` 仍值得留在 8.0:端墙点更多 ⇒ 点对外的方程条件更好。
   但它是**辅助**,不是主因。别把功劳记在它头上。
"""

import math

import numpy as np
from scipy.spatial import cKDTree


def scan_to_points(scan_msg, max_range=5.0, range_min=None):
    """把一帧扫描转成激光坐标系下的 2D 点 (N,2) numpy 数组。

    参数
    ----
    scan_msg : 任何带 `.ranges` / `.angle_min` / `.angle_increment` / `.range_min`
               的对象。ROS 的 `sensor_msgs/LaserScan` 满足它,测试里构造的
               假对象也满足 —— 这就是本模块不需要 rclpy 的原因。
    max_range : **承重参数**,见模块 docstring。超出它的点被丢弃。
    range_min : 下限。默认取 `scan_msg.range_min`(A1 驱动给的是 0.15,
                因为更近处是雷达盲区)。

    返回 None 表示有效点不足 20 个(帧不可用)。
    """
    if range_min is None:
        range_min = scan_msg.range_min

    points = []
    angle = scan_msg.angle_min
    for r in scan_msg.ranges:
        if range_min < r < max_range and math.isfinite(r):
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            points.append([x, y])
        angle += scan_msg.angle_increment

    if len(points) < 20:
        return None
    return np.array(points, dtype=np.float64)


def icp_step(source, target, max_dist=1.0):
    """一次 ICP 迭代:把 source 对齐到 target。

    用 SVD/Kabsch 求**点到点**对应下的刚体变换(含反射保护)。

    返回 (R, t, mean_err);失败时 (None, None, inf)。

    ⚠️ `max_dist` 是**对应点门限**:只在 target 里找这个距离以内的最近点。
       它偏大时,平行墙上的对应点能**沿墙滑动** —— 走廊里的危险就在这。
       `icp_align` 支持让这个门限**逐级收紧**(由粗到细)。
    """
    if source is None or target is None or len(source) < 10 or len(target) < 10:
        return None, None, float('inf')

    tree = cKDTree(target)
    dists, idxs = tree.query(source)

    mask = dists < max_dist
    if mask.sum() < 10:
        return None, None, float('inf')

    src_matched = source[mask]
    tgt_matched = target[idxs[mask]]

    src_mean = src_matched.mean(axis=0)
    tgt_mean = tgt_matched.mean(axis=0)

    src_centered = src_matched - src_mean
    tgt_centered = tgt_matched - tgt_mean

    W = src_centered.T @ tgt_centered
    U, _, Vt = np.linalg.svd(W)
    R = Vt.T @ U.T

    # 反射保护:SVD 有时会给出 det = -1 的「镜像」解,那不是刚体变换。
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    t = tgt_mean - R @ src_mean
    mean_err = dists[mask].mean()

    return R, t, mean_err


def icp_align(source, target, iters=10, max_dist=1.0, min_error=0.005):
    """迭代 ICP,返回 (R_total, t_total, converged, last_err)。

    参数
    ----
    max_dist : **标量**   → 所有迭代用同一个门限(= 旧行为)
               **序列**   → 第 i 次迭代用 max_dist[i],即**由粗到细**。
                            序列比 iters 短时,后续迭代沿用最后一个值。

    返回的 (R_total, t_total) 是**激光坐标系下**的相对变换:
    把「上一帧的点」变到「当前帧的点」所需的那一步。
    """
    if source is None or target is None or len(source) < 10 or len(target) < 10:
        return None, None, False, float('inf')

    try:
        gates = list(max_dist)
    except TypeError:
        gates = None  # 标量

    R_total = np.eye(2)
    t_total = np.zeros(2)
    prev_err = float('inf')
    last_err = float('inf')
    converged = False

    src = source.copy()
    for i in range(iters):
        gate = gates[min(i, len(gates) - 1)] if gates else max_dist

        R, t, err = icp_step(src, target, gate)
        if R is None:
            break

        src = (R @ src.T).T + t
        t_total = R @ t_total + t
        R_total = R @ R_total
        last_err = err

        if abs(prev_err - err) < min_error:
            converged = True
            break
        prev_err = err

    return R_total, t_total, converged, last_err


def scan_to_points_normals(scan_msg, max_range=5.0, range_min=None):
    """把一帧扫描转成 **点 + 法向**。返回 `(points, normals)`,无效为 nan。

    ⚠️ 法向是点对面 ICP 的**全部依据** —— 见模块 docstring。没有它,
       平行墙就会继续投假票。
    ⚠️ 法向由**相邻束的切线转 90°** 得到(2D 激光的标准做法)。所以
       **束序相邻**才算数:被 `max_range`/`range_min` 过滤掉或丢点的位置
       没有邻居,该点法向记 nan,后续会被剔除 —— 这是对的,硬凑邻居
       会在墙角处造出**假法向**,比没有更坏。
    """
    if range_min is None:
        range_min = scan_msg.range_min

    pts, beam_idx = [], []
    angle = scan_msg.angle_min
    for k, r in enumerate(scan_msg.ranges):
        if range_min < r < max_range and math.isfinite(r):
            pts.append([r * math.cos(angle), r * math.sin(angle)])
            beam_idx.append(k)
        angle += scan_msg.angle_increment

    if len(pts) < 20:
        return None, None

    points = np.array(pts, dtype=np.float64)
    normals = np.full_like(points, np.nan)
    pos = {k: i for i, k in enumerate(beam_idx)}      # 束号 → 输出下标

    for i, k in enumerate(beam_idx):
        a, b = pos.get(k - 1), pos.get(k + 1)
        if a is None or b is None:
            continue                                   # 缺邻居 ⇒ 无正常法向
        tx, ty = points[b] - points[a]
        n = math.hypot(tx, ty)
        if n < 1e-9:
            continue
        normals[i] = (-ty / n, tx / n)                 # 切线转 90° 即法向

    return points, normals


def icp_step_plane(source, target, target_normals, max_dist=1.0):
    """一次**点对面** ICP 迭代(小角线性化)。返回 `(R, t, mean_err)`。

    解 `n·(R·p + t − q) = 0` 的最小二乘,其中 `n` 是 q 处(即目标点处)的法向。
    雅可比为 `[n_x, n_y, n_y·p_x − n_x·p_y]`,最后一项是 θ 的导数。

    ⚠️ `target_normals` 为 nan 的行会被剔除 —— 它们给不出沿面约束。
    ⚠️ 这是**线性化**步:单步只在小角度下准确。靠 `icp_align_plane` 迭代收敛,
       **不要**指望一次调用就拿到大转角。
    """
    if source is None or target is None or len(source) < 10 or len(target) < 10:
        return None, None, float('inf')

    dists, idxs = cKDTree(target).query(source)
    ok = (dists < max_dist) & np.isfinite(target_normals[idxs]).all(axis=1)
    if ok.sum() < 10:
        return None, None, float('inf')

    p = source[ok]
    q = target[idxs[ok]]
    n = target_normals[idxs[ok]]

    A = np.column_stack([n[:, 0], n[:, 1],
                         n[:, 1] * p[:, 0] - n[:, 0] * p[:, 1]])
    b = (n * (q - p)).sum(axis=1)
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    dx, dy, dth = float(sol[0]), float(sol[1]), float(sol[2])

    c, s = math.cos(dth), math.sin(dth)
    R = np.array([[c, -s], [s, c]])
    t = np.array([dx, dy])
    return R, t, float(dists[ok].mean())


def icp_align_plane(source, target, target_normals,
                    iters=10, max_dist=1.0, min_error=0.005):
    """迭代**点对面** ICP。返回 `(R_total, t_total, converged, last_err)`。

    返回形状与 `icp_align` **完全一致**,所以 `integrate()` 可以原样复用。

    ⚠️ 与点对点版的关键差别不在迭代次数,而在**代价函数的形状**:
       平行面内代价恒为 0 ⇒ 侧墙弃权,不投票。见模块 docstring。
    """
    if source is None or target is None or len(source) < 10 or len(target) < 10:
        return None, None, False, float('inf')

    R_total = np.eye(2)
    t_total = np.zeros(2)
    prev_err = float('inf')
    last_err = float('inf')
    converged = False

    for _ in range(iters):
        # 把累计变换作用到源点上,再在**当前估计位置**求对应
        cur = (R_total @ source.T).T + t_total
        R, t, err = icp_step_plane(cur, target, target_normals, max_dist)
        if R is None:
            break

        t_total = R @ t_total + t
        R_total = R @ R_total
        last_err = err

        if abs(prev_err - err) < min_error:
            converged = True
            break
        prev_err = err

    return R_total, t_total, converged, last_err


def compose_pose(pose, dx, dy, dtheta):
    """2D 位姿复合:`新位姿 = 旧位姿 ⊕ 增量`。

    (dx, dy) 是**激光坐标系下**的位移,先用当前朝向转到世界系再叠加 ——
    这就是旧代码里那个 `dx*cos - dy*sin` / `dx*sin + dy*cos` 的含义。
    角度归一化到 (-pi, pi]。
    """
    x, y, th = pose
    nx = x + dx * math.cos(th) - dy * math.sin(th)
    ny = y + dx * math.sin(th) + dy * math.cos(th)
    nth = math.atan2(math.sin(th + dtheta), math.cos(th + dtheta))
    return nx, ny, nth


def integrate(current_pose, R_total, t_total,
              max_step_dist=0.3, max_step_theta=0.3):
    """把一次 ICP 结果并进位姿。返回 `(new_pose, accepted, dist, dtheta)`。

    ⚠️ `max_step_dist` / `max_step_theta` 是**合理性闸**:一帧之内不可能
       移动这么远/转这么多,超过了就认为 ICP 匹配错了。雷达 10 Hz 下
       0.3 m 相当于 3 m/s,机器人绝无可能 —— 所以触发基本都意味着坏匹配。

    ⚠️⚠️ 被拒绝时,**调用方不得推进参考帧**(`prev_points`)。
       旧代码在拒绝时仍然推进了参考帧,后果有二:
         ① 那 0.3+ m 的真实位移**永久丢失** —— 下一帧只测得到 N+1→N+2;
         ② 下一帧拿 N+1 当参考、位姿却还停在 N ⇒ **一帧的系统性偏差**。
       正确做法是在外面保持参考帧不动、下一帧重试,见 `laser_odom.py`。
    """
    if R_total is None or t_total is None:
        return current_pose, False, 0.0, 0.0

    dtheta = math.atan2(R_total[1, 0], R_total[0, 0])
    dx = float(t_total[0])
    dy = float(t_total[1])
    dist = math.hypot(dx, dy)

    if dist < max_step_dist and abs(dtheta) < max_step_theta:
        return compose_pose(current_pose, dx, dy, dtheta), True, dist, dtheta

    return current_pose, False, dist, dtheta
