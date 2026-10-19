#!/bin/bash
# mapping_ctl.sh — 建图管线的最小启停开关
# ============================================================
#   bash ~/mapping_ctl.sh start | stop | status
#
# 给网页地图窗口那个「启动建图」按钮用,也可以直接 ssh 跑。
#
# 只碰这 4 个进程,别的都不碰:
#   rplidar_node / wheel_odom.py / static_transform_publisher / async_slam_toolbox_node
#
# ⚠️ 为什么不复用 start_slam.sh:
#   ① 它 `pkill -f slam_viz` —— 会杀掉正在提供这个按钮的网页服务;
#   ② 它默认 USE_WHEEL_ODOM=1 ⇒ 起 motor_driver.py,而 motor_driver 和 pwm_proc
#      **申请同一批 GPIO line**(libgpiod 独占),后启动的直接 line busy;
#   ③ 它无条件拉 rviz2(上次运行撞 GLSL link error)。
#   按钮的爆炸半径必须**恰好是那 4 个 SLAM 进程**,不能建在一个「默认动作是危险」
#   的脚本上。所以这里是独立一份,不是 start_slam.sh 的开关。
#
# ⚠️⚠️ 本脚本自己绝不能出现在任何 pkill -f 的模式里 —— 命令行里带那个字符串的 shell
#   会把自己杀掉(exit 255)。所以 wheel_odom 用带括号的 `wheel_odom[.]py` 写法,
#   和本仓库既有的 `slam_viz[.]py` 约定一致。加新模式前先想一遍这件事。
#
# 退出码:
#   0  成功
#   2  雷达设备不存在(重启后需重新插拔 USB)
#   3  motor_driver 在跑(会和 wheel_odom 抢 odom→base_link TF)
#   4  用法错
#   5  source ROS 环境失败
#   6  定位导航正在运行，必须先停止
#   7  编码器未标定或采样器缺失

set -u

LIDAR_DEV="${LIDAR_DEV:-/dev/ttyUSB0}"    # 可覆盖:为了能在不拔硬件的情况下故意测失败路径
LIDAR_LOG=/tmp/lidar.log
ODOM_LOG=/tmp/laser_odom.log  # Legacy log filename retained for the deployed web tailer
TF_LOG=/tmp/tf_laser.log
SLAM_LOG=/tmp/slam.log

# 这 4 个文件必须和 slam_viz.py 的 LOG_FILES 一致 —— 网页日志窗口 tail 的就是它们,
# 换名字不会报错,只会让日志窗口安静下来。
PROCS=(rplidar_node 'wheel_odom[.]py' static_transform_publisher async_slam_toolbox_node)

usage() {
    echo "用法: bash $0 start|stop|status" >&2
    exit 4
}

# 有没有在跑(供预检用)。用 pgrep 而不是 pgrep -f 全量匹配,避免匹配到自己的参数。
process_pattern() {
    case "$1" in
        *odom*py*|*motor_driver*) printf '^[^ ]*python3 [^ ]*/%s([ ]|$)' "$1" ;;
        rplidar_node|static_transform_publisher|async_slam_toolbox_node)
            printf '^[^ ]*/%s([ ]|$)' "$1" ;;
        *) printf '^[^ ]*/%s' "$1" ;;
    esac
}
running() { pgrep -f "$(process_pattern "$1")" >/dev/null 2>&1; }

# 杀干净:先 SIGTERM,复核,残余再 SIGKILL。返回剩余数量。
kill_wait() {
    local pat n
    pat=$(process_pattern "$1")
    pkill -f "$pat" 2>/dev/null
    for _ in 1 2 3 4 5; do
        sleep 0.2
        pgrep -f "$pat" >/dev/null 2>&1 || return 0
    done
    pkill -9 -f "$pat" 2>/dev/null
    sleep 0.3
    n=$(pgrep -f "$pat" 2>/dev/null | wc -l)
    return "$(( n > 0 ? 1 : 0 ))"
}

do_stop() {
    # 顺序:先断**产出**(SLAM),再断输入 —— 反过来会让 SLAM 在最后几秒刷
    # "Failed to compute odom pose" 之类的噪声,看着像出错其实只是顺序问题。
    local leftover=0
    for pat in async_slam_toolbox_node static_transform_publisher 'wheel_odom[.]py' 'laser_odom[.]py' rplidar_node; do
        if running "$pat"; then
            kill_wait "$pat" || leftover=$((leftover + 1))
            echo "  已停 $pat"
        fi
    done
    # ros2 launch 的父进程(它自己会在子节点死后退出,但可能赖着)
    pkill -f '^[^ ]*python3 [^ ]*/ros2 launch rplidar_ros rplidar_a1_launch[.]py' 2>/dev/null

    if [ "$leftover" -gt 0 ]; then
        echo "⚠️ 停止不干净:有进程 SIGKILL 后仍在"
        for pat in "${PROCS[@]}"; do pgrep -af "$(process_pattern "$pat")" 2>/dev/null; done
        return 1
    fi
    echo "已停止"
    return 0
}

do_start() {
    # Localization owns the same scanner/TF chain. Refuse a second publisher.
    if pgrep -f '^[^ ]*python3 [^ ]*/ros2 launch .*/robot/navigation/navigation_launch[.]py' >/dev/null; then
        echo "请先在网页停止定位，再启动建图"
        return 6
    fi
    # ── 预检:全部快速失败,文案直接指出下一步 ──────────────────
    if [ ! -e "$LIDAR_DEV" ]; then
        echo "❌ 雷达设备 $LIDAR_DEV 不存在 —— 设备重启后需要重新插拔 RPLidar 的 USB"
        return 2
    fi
    # motor_driver 与 wheel_odom **都**发 odom→base_link TF。两个发布者同时存在时
    # slam_toolbox 的 TF 查询会随机取到其中一个,地图会莫名其妙地抖/飘,而日志上
    # 只是偶发的 TF 警告 —— 所以这里宁可不起,也不要起一个「看着在跑」的坏组合。
    if running 'motor_driver[.]py'; then
        echo "❌ motor_driver.py 正在跑,它和 wheel_odom 会同时发布 odom→base_link TF"
        echo "   两者不能并存。先 bash ~/start_pwm.sh(它会停掉 motor_driver 把 GPIO 让给 pwm_proc)"
        return 3
    fi

    # Fail before touching the existing map when encoder calibration is missing.
    /usr/bin/python3 "$HOME/robot/slam/wheel_odom.py" --check || return 7
    [ -x "$HOME/robot/slam/encoder_sampler" ] || { echo "编码器采样器未安装"; return 7; }

    # ── 清残留(幂等)────────────────────────────────────────
    do_stop >/dev/null 2>&1 || return 1

    # ── 环境:自己 source,不依赖调用者 ───────────────────────
    # ⚠️⚠️ 必须临时关掉 `set -u`:ROS 的 setup.bash 会引用**未定义的**变量
    #    (COLCON_TRACE 等)。开着 nounset 去 source,shell 会在那一行**直接退出** ——
    #    表现是「脚本返回 1、什么都没起、日志一行没动、也没有任何 ❌」,
    #    比报错还难查。
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    # shellcheck disable=SC1091
    source "$HOME/lidar_ws/install/setup.bash"
    set -u

    # source 之后立刻核实 —— 上面那种失败如果没被抓住,后面 4 个 setsid 会把
    # "ros2: command not found" 写进各自的日志文件里,看起来像 4 个独立故障。
    if ! command -v ros2 >/dev/null 2>&1; then
        echo "❌ source ROS 环境后仍找不到 ros2 —— 检查 /opt/ros/humble 与 ~/lidar_ws/install"
        return 5
    fi

    # ── 起 4 个 ────────────────────────────────────────────
    # setsid:脱离调用者的会话/进程组。**重启网页不能杀掉进行中的建图** ——
    #   start_slam_viz.sh 每次都会 pkill slam_viz,那是日常操作。
    # `>` 重定向会原地截断日志 —— log_tail_loop 现在能识别并重开(见 slam_viz.py)。
    setsid nohup ros2 launch rplidar_ros rplidar_a1_launch.py > "$LIDAR_LOG" 2>&1 < /dev/null &
    echo "  → 雷达     $LIDAR_DEV            日志 $LIDAR_LOG"

    # Input-only encoder node publishes odom -> base_link; motor GPIO untouched.
    setsid nohup python3 "$HOME/robot/slam/wheel_odom.py" --params "$HOME/robot/slam/wheel_odom.yaml" \
        > "$ODOM_LOG" 2>&1 < /dev/null &
    echo "  → 编码器里程计                日志 $ODOM_LOG"

    setsid nohup ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser \
        > "$TF_LOG" 2>&1 < /dev/null &
    echo "  → 静态 TF   base_link→laser       日志 $TF_LOG"

    # ⚠️ 这个 YAML 名字里的 "no_odom" 指的是**没有轮式编码器里程计**,不是「不需要
    #    /odom」。它写着 odom_frame: odom,而 async 节点拿不到 odom→base_link 就
    #    直接丢弃每一帧扫描。所以上面那两路(里程计 + TF)是它的前提,不是陪衬。
    setsid nohup ros2 run slam_toolbox async_slam_toolbox_node \
        --ros-args --params-file "$HOME/mapping_no_odom.yaml" > "$SLAM_LOG" 2>&1 < /dev/null &
    echo "  → SLAM     slam_toolbox(async)   日志 $SLAM_LOG"

    # ── 只报「起来了没」,不报「建图成没成」────────────────────
    # 起完就退出这件事,和建图有没有真的在跑,是两回事。脚本不对后者表态 ——
    # 状态一律由 slam_viz 按 /scan 和 /map 的**实际数据流**判定。一个会说谎的
    # 「已启动」正是本项目反复吃过的那类静默失效。
    #
    # 数进程本身也只是「报个数」,不是判据。但要**等**它们起来再数:`ros2 launch`
    # 和 `ros2 run` 拉起真正的子进程要一两秒,固定 sleep 0.5 会数出「3/4」这种
    # 假失败(一秒后其实全都在),而它会出现在网页日志窗口里吓人。
    local alive=0 tries=0
    while [ "$tries" -lt 12 ]; do
        alive=0
        for pat in rplidar_node 'wheel_odom[.]py' static_transform_publisher async_slam_toolbox_node; do
            if running "$pat"; then alive=$((alive + 1)); fi
        done
        [ "$alive" -ge 4 ] && break
        tries=$((tries + 1))
        sleep 0.5
    done
    echo "已拉起 $alive/4 个进程(是否真的在建图,看地图窗口有没有在画)"
    # 缺任意节点都清理本次启动；四个进程齐全仍不等于数据流健康。
    if [ "$alive" -lt 4 ]; then
        tail -n 8 "$ODOM_LOG"
        do_stop
        return 1
    fi
    return 0
}

do_status() {
    local found=0
    for pat in "${PROCS[@]}"; do
        local out
        out=$(pgrep -af "$(process_pattern "$pat")" 2>/dev/null)
        if [ -n "$out" ]; then
            found=1
            echo "$out"
        fi
    done
    [ "$found" -eq 1 ] || echo "(建图管线未运行)"
    return 0
}

case "${1:-}" in
    start)  do_start  ;;
    stop)   do_stop   ;;
    status) do_status ;;
    *)      usage     ;;
esac
