# 电机驱动接线表 — L298N ↔ 犀牛派 X1 40-Pin

> 引脚映射已对照犀牛派官方文档逐条核验（2026-09-15，10/10 匹配）。
> 来源: https://rhinopi.docs.aidlux.com/rhino-x1-aidlux/basic-dev/40-pin/pin_intro
>
> ⚠️ 排针为 **3.3V 逻辑**。所有输入到犀牛派的信号都不得超过 3.3V。
>
> ⚠️ **「10/10 匹配」说的是排针 PIN ↔ GPIO 编号那一层,它是对的。**
> **「GPIO 信号到底驱动哪个轮子」那一层曾经写反**(文档写 PWMA=左轮,实测是右轮),
> 已于 2026-09-15 按实车驾驶结果更正,详见下文「通道 ≠ 左右轮」。**照本表重接线之前
> 请先读完那一节** —— 这是本文件唯一一处曾经把结论写反的地方。

---

## ⚠️ 三条硬性警告

1. **编码器 VCC 必须接 3.3V（pin1 / pin17），严禁接 5V。**
   A/B 输出电平跟随 VCC，接 5V 会向 GPIO 灌 5V，打坏 pin7/8/10/11。

2. **L298N 的 5V 脚与排针 5V 只能二选一。**
   - 方案 A（推荐）：**拔掉 L298N 的 5V 跳线帽**，5V 脚当输入，从排针 pin2/pin4 取 5V。
   - 方案 B：保留跳线帽，L298N 自己从 VS 稳压出 5V，此时 **5V 脚悬空，不要接排针**。
   - 两个 5V 源直接对接会互相倒灌。

3. **必须共地。** 犀牛派 GND ↔ L298N GND ↔ 编码器 GND，全部连通。
   不共地则 PWM 无参考电平，表现为电机乱转或不转。

---

## 一、信号线：L298N → 犀牛派

| L298N 端子 | 犀牛派 PIN | GPIO | gpiochip0 line | 建议线色 | 说明 |
|---|---|---|---|---|---|
| ENA / PWMA | **32** | GPIO_91 | 91 | 橙 | **通道 L** — 调速 (软件 PWM) |
| IN1 / AIN1 | **18** | GPIO_61 | 61 | 黄 | **通道 L** — 方向 1 |
| IN2 / AIN2 | **31** | GPIO_62 | 62 | 绿 | **通道 L** — 方向 2 |
| ENB / PWMB | **36** | GPIO_63 | 63 | 蓝 | **通道 R** — 调速 (软件 PWM) |
| IN3 / BIN1 | **37** | GPIO_60 | 60 | 紫 | **通道 R** — 方向 1 |
| IN4 / BIN2 | **33** | GPIO_124 | 124 | 灰 | **通道 R** — 方向 2 |

> ⚠️ **本表刻意只写「通道 L / 通道 R」,不写「左轮 / 右轮」。** 原因见下节。

**电机输出端**（L298N → 直流电机）：

| L298N | 接到 | 依据 |
|---|---|---|
| OUT1 / OUT2 | **右电机** | 实测通道 L 驱动右轮;而 OUT1/OUT2 正是由 ENA/IN1/IN2 控制的那一对 |
| OUT3 / OUT4 | **左电机** | 同上,通道 R 驱动左轮 |

> 轮子转向反了**不要重接线**,但先分清是哪一种反 —— 这两种的修法完全不同:
> * **单侧反**(直行变成原地自转)→ 改 `flip_left` / `flip_right`。
> * **整体转向反**(前进后退正常,只有左右转相反)→ 改 `swap_lr`。改 `flip_*` **修不回来**。
>   且通道交叉时 `flip_left` 实际作用在**右**轮、`flip_right` 在**左**轮,别照着名字调。
> 详见下节。

**电平逻辑**（已由 `motor_hw.py` 的 `GpioMotorHW.set_direction` 实现）：

```
IN1=1, IN2=0  →  正转
IN1=0, IN2=1  →  反转
IN1=0, IN2=0  →  停机
ENA 高 → 电机使能；PWM 占空比调速；ENA 低 → 停机
```

---

## ⚠️ 通道 ≠ 左右轮：实测定案（2026-09-15）

**通道 L(GPIO 91/61/62,即 ENA/IN1/IN2)物理上驱动的是【右】轮;
通道 R(GPIO 63/60/124,即 ENB/IN3/IN4)驱动的是【左】轮。**

也就是说,**接线与本文档最初的「PWMA=左电机」设计相反**。这不是推测:

- `motor_params.yaml` 曾按「PWMA=左电机」置 `swap_lr: false`,实际驾驶时**前进/后退正常
  但左右转整体反向**;置 `swap_lr: true` 后转向正确。
- 判据的力学依据:只有 `swap_lr` 能翻转「转向」而保持前进/后退不变
  (`map_channels` 交换通道 → `v_r-v_l` 变成 `v_l-v_r`);翻 `flip_*` 是反过来 ——
  单独翻一侧会把直行变成原地自转。所以现象与成因是唯一对应的。

**两个后果,踩之前务必知道:**

1. **转向整体反向,只能改 `swap_lr`。** 改 `flip_left`/`flip_right` **修不回来** ——
   通道交叉时两轮的极性被一起取反,前进/后退也会跟着翻。
   当前定案值:`swap_lr: true`(`motor_params.yaml`),别翻。
2. **通道被交叉后,`flip_left`/`flip_right` 的名字与物理轮是反的:**
   `flip_left` 实际作用于**右**轮,`flip_right` 实际作用于**左**轮。
   要调某个轮的极性,先按 `swap_lr` 折算,别照着名字调。

若将来重新接线并确认与文档一致,把 `swap_lr` 改回 `false` 即可 —— 但**改线之后要重测转向**,
不能只改参数。

---

## 二、编码器 → 犀牛派

| 编码器信号 | 犀牛派 PIN | GPIO | line | 建议线色 |
|---|---|---|---|---|
| 左编码器 A | **7** | GPIO_00 | 0 | 棕 |
| 左编码器 B | **8** | GPIO_12 | 12 | 红 |
| 右编码器 A | **11** | GPIO_05 | 5 | 棕 |
| 右编码器 B | **10** | GPIO_01 | 1 | 红 |
| 编码器 VCC | **1** 或 **17** (3.3V OUT) | — | — | 橙 |
| 编码器 GND | **9** / **14** / **25** / **34** / **39** | — | — | 黑 |

> 编码器 A/B 接反只会让计数方向变负，改 `enc_sign_left` / `enc_sign_right` 为 `-1` 即可。

⚠️ **编码器目前未接线**(`motor_params.yaml` 的 `use_encoders: false`)。
本节与下表是**待接线的规划**,不是现状。不要照着它去 `ros2 topic echo /odom`
验证 —— 直连 PWM 路径下根本没有 `/odom`,开环也没有里程计可用
(SLAM 请改用 `USE_WHEEL_ODOM=0` 的激光 ICP)。

⚠️ 上表的「左/右编码器」是**按设计**的命名,与电机一样**未经接线核对**。
真接上以后,判断哪一路是哪个轮,以「手转某个轮、看哪一路计数在动」为准,
不要相信标签。

---

## 三、舵机（转向）→ 犀牛派

**现状：舵机独占转向，两轮不再差速。**

摇杆的左右轴（`a`）**只**送给舵机；轮子只吃油门（`l`），两轮等速。
由 `steer_mode` 决定，见下表 —— 它是**互斥三选一，不是叠加开关**：

| `steer_mode` | 轮子吃 `a`？ | 舵机吃 `a`？ | 说明 |
|---|---|---|---|
| **`servo`**（当前） | ❌（喂 0） | ✅ | 差速项彻底为 0，`s_l == s_r` |
| `diff` | ✅ 差速 | ❌ 恒 park | 2026-09-15 之前的老行为，是回退路径 |
| `both` | ✅ | ✅ | 2026-09-15 那版「保留差速 + 舵机做辅助」的设计 |

**为什么不做成可叠加的开关**：两者同时转向会**互相打架** —— 舵机把轮子掰向
一边、差速又把车身转向另一边，车会又转又拧。

⚠️ **`steer_mode: servo`/`both` 要求 `servo_enable: true`。** 没有舵机就**完全没有
转向**，所以 `pwm_proc` 在那种组合下**直接退出**（退出码 1，文案点名 `steer_mode`），
不做「优雅降级」—— 一辆不能转向的车在日志上和正常的一模一样，正是最该避免的那种故障。

⚠️ 选 `servo` 后，**「转向整体反向只能改 `swap_lr`」这条第一节的判据不再适用**：
那时 `a` 根本到不了轮子，`swap_lr` 对左右转没有任何影响（两通道恒等，交换等于没交换）。
转向方向改判 `servo_invert`。上面第一节的 `swap_lr` 结论只在 `diff` 模式下有效。

⚠️ 同理，`joy_inject.py --wheel L|R` 的单轮隔离在 `servo` 模式下**无解** ——
任何 `(l, a)` 都只能让两通道相等。该工具会**拒绝执行**并指向 `motor_test.py`，
而不是按老公式算出一个把两轮一起驱动的 `(l, a)` 却仍打印「仅左轮」。

| 舵机线 | 犀牛派 PIN | GPIO / line | 线色 | 说明 |
|---|---|---|---|---|
| 信号 | **28** | GPIO_33 / line 33 | 橙或黄 | 3.3V 逻辑 |
| VCC | **2** 或 **4** | 5V OUT | 红 | ⚠️ 见下方红字 |
| GND | **6/9/14/20/25/30/34/39** 任一 | — | 棕或黑 | **必须共地**，不共地 PWM 没有参考电平 |

**为什么是 pin 28**：`gpioinfo` 在板子上实测该 line 为 `unused`；它属于
**同一块 `gpiochip0`**，所以本项目一直用的「`GPIO_XX` == `gpiochip0` line 号」这条
规则对它成立；且 pin 30 / pin 34 都是 GND，跳线好走。
备选：**pin 26**（GPIO_22，同行 pin 25 就是 GND）/ **pin 27**（GPIO_32）。

⚠️ 上板前自己核一遍：`gpioinfo | grep -w 33` 应是 `unused`。
本表的 pin↔GPIO 映射，**只有电机/编码器那 10 条做过核验**（本文件开头那句
「10/10 匹配」说的就是这个范围），pin 28 这条是 2026-09-15 新查的。

⛔ **别拿去当信号的那些脚**：pin 1/17（3.3V OUT）、pin 2/4（5V OUT，只供电）、
各 GND、pin 3/5/13/15（I2C）、pin 19/21/23/24（SPI3）、pin 16/29（UART）。
⚠️ 其中 **pin 29 自 2026-09-27 起已被二维云台占用**（UART TX）—— 它不再是「空着
别动」，而是**已经在用**，别再把它规划给别的东西，见 `gimbal_wiring.md`。
pin 16 是同一条 UART 的 RX；我们**单向只发**所以没接它，但也**别**把云台的 TX
接到上面 —— 那才是真会把 5V 电平打进 3.3V 引脚的那条路径（第二节）。
**pin 12/35/38/40（LPI_I2S）属于 `gpiochip9`，`GPIO_XX == line XX` 在那里不成立**。
也**别动** pin 7/8/11/10（line 0/12/5/1）—— 今天空着，但那是编码器规划占用的，
`use_encoders: true` 那天会被一次 `LineBulk` 申请走。

### ⚠️ 供电：这条是本次唯一由你承担的风险

排针 5V（pin 2/4）现在正按**方案 A** 给 L298N 逻辑供电，相机也在同一条轨上，
而这条轨**没有任何文档给出电流预算** —— 本文件从头到尾没有一个 mA 数字。

软件侧做的「非驾驶态不发脉冲（park）」只能保证**停着的时候**舵机几乎不吃电
（无脉冲 = 舵机松掉 = 几乎 0 电流）。**一旦 arm 起来驾驶，舵机就是实打实在这条轨上
拉 50~150mA，堵转可到 ~700mA。**

而且真被拉低压了，**软件检测不到** —— 症状会先在相机 / SLAM 上出现，很难联想到舵机。
真出问题时的正解是**独立 5V BEC**（接电池，与犀牛派共地），不是改软件。

### 安装要求（不是可选项）

**舵机臂要装在「命令 1500µs」对应的位置上。**
没有反馈线，就无从得知舵机的静止角，所以每次 park → arm 都是「从它此刻在哪、
以全速转到目标角」。装在 1500µs 处能让这个动作最小。

### 两个必须**实车**定、不能照名字猜的参数

| 参数 | 怎么定 |
|---|---|
| `servo_invert` | 看「车头往左转时舵机是不是也往左偏」。⚠️ **与 `swap_lr` 无关** —— `swap_lr` 翻的是差速转向，这个翻的是舵机自己的机械安装方向，是**两个独立的未知量** |
| `servo_center_us` | 给 `a=0`，让舵机臂对准机械中位。**中位偏了改这个，不是改摇杆** |

`flip_left` / `flip_right` 已经因为「照着参数名猜物理方向」栽过一次，别再来一遍。

---

## 四、电源

| L298N | 接到 | 说明 |
|---|---|---|
| GND | 排针 GND (6/9/14/20/25/30/34/39) | **必接**，共地 |
| 5V (逻辑) | 排针 pin2 或 pin4 (5V OUT) | 仅方案 A；方案 B 悬空 |
| 12V / VS | **独立电池/电源正极** | 严禁从排针取电机电流 |
| — | 电源负极 ↔ 犀牛派 GND | 共地 |

**当前走直连 PWM 路径**(`bash ~/start_pwm.sh` → `pwm_proc.py`),它**绕开了 ROS**,
所以 `start_slam.sh` / `motor_params.yaml` 里的 `cmd_vel_timeout: 0.5s` 看门狗
**在这条路上不生效**。这条路的停机机制是 `pwm_proc` 本地的
`pwm_deadman_ms: 300`(300ms 没收到摇杆包 → duty 0)+ `armed` 门控(上电默认禁)。

`motor_driver.py` 那条 ROS 路径(用 `/cmd_vel`、看门狗、轮式里程计)**仍然可用**,
但**不能与 `pwm_proc` 同时跑** —— 两者申请同一批 GPIO line,libgpiod 独占,
后启动的会直接报 line busy。

---

## 五、排针占用图（接线时照此数脚）

```
        左列 (奇)                     右列 (偶)
 pin  1  3.3V OUT    ┃   5V OUT          pin  2  ← L298N 逻辑 5V (方案A) + 舵机VCC
 pin  3  I2C1_SDA    ┃   5V OUT          pin  4  ← L298N 逻辑 5V (方案A) + 舵机VCC
 pin  5  I2C1_SCL    ┃   GND             pin  6  ← 共地
 pin  7 ○GPIO_00     ┃ ○ GPIO_12         pin  8    左编码器A / 左编码器B (未接线)
 pin  9  GND         ┃ ○ GPIO_01         pin 10  ← 共地      / 右编码器B (未接线)
 pin 11 ○GPIO_05     ┃   LPI_I2S2_CLK    pin 12    右编码器A (未接线)
 pin 13  I2C4_SDA    ┃   GND             pin 14  ← 共地
 pin 15  I2C4_SCL    ┃   UART_RX         pin 16
 pin 17  3.3V OUT  ←─┃   GPIO_61       ● pin 18    IN1  / 编码器VCC
 pin 19  SPI3_MOSI   ┃   GND             pin 20  ← 共地
 pin 21  SPI3_MISO   ┃   GPIO_118        pin 22
 pin 23  SPI3_CLK    ┃   SPI3_CS0        pin 24
 pin 25  GND         ┃   GPIO_22         pin 26    舵机备用脚
 pin 27  GPIO_32     ┃ ◇ GPIO_33         pin 28  ← 舵机信号 (未接线)
 pin 29  UART_TX     ┃   GND             pin 30  ← 共地
 pin 31 ●GPIO_62     ┃ ● GPIO_91         pin 32    IN2 / ENA
 pin 33 ●GPIO_124    ┃   GND             pin 34  ← 共地
 pin 35  LPI_I2S2_WS ┃ ● GPIO_63         pin 36    ENB
 pin 37 ●GPIO_60     ┃   LPI_I2S2_DATA0  pin 38    IN3
 pin 39  GND         ┃   LPI_I2S2_DATA1  pin 40
```

● = 本项目占用   ○ = 已规划但**尚未接线**(编码器四路)
◇ = 舵机信号，**尚未接线**；接之前先 `gpioinfo | grep -w 33` 核实是 `unused`

---

## 六、接完线后的验证顺序

```bash
# 0) 先解 GPIO 权限 (root:700 会挡住 aidlux 用户，重启后失效需重跑)
echo aidlux | sudo -S chmod 666 /dev/gpiochip*

# 1) 单脚通断自检 —— 逐脚置位，万用表量排针
python3 ~/gpio_smoke.py

# 2) 起直连 PWM 进程 (会先停掉 motor_driver 让出 GPIO)
bash ~/start_pwm.sh

# 3) 单轮隔离 / 阈值扫描 —— 逐段停、由人盯着轮子确认
#    ⚠️ 驱动轮必须已架空
python3 ~/joy_inject.py --arm --wheel L      # 只驱动通道 L
python3 ~/joy_inject.py --arm --wheel R      # 只驱动通道 R
python3 ~/joy_inject.py --arm --sweep --wheel L   # 扫启动阈值

# 4) 舵机 (仅在接好线、servo_enable 置 true 之后)
gpioinfo | grep -w 33                        # 应先看到 [used] 且 consumer=servo_steer
tail -f /tmp/pwm_proc.log                    # 启动横幅应打印舵机 line/center/range/invert
python3 ~/joy_inject.py --arm --a  0         # a=0    → 对机械中位 (调 servo_center_us)
python3 ~/joy_inject.py --arm --a  1         # a=+1   → 一端
python3 ~/joy_inject.py --arm --a -1         # a=-1   → 另一端 (定 servo_invert)
#    ⚠️ 舵机消费的就是**已经到达 pwm_proc 的那个 a**,前端/UDP 契约一个字都没改,
#       所以 --a 直接就能驱动它,不需要新参数。
#    ✅ steer_mode=servo 下这三条**不会驱动电机**:l 默认 0 → 两轮 duty 恒 0,
#       只有舵机在动。想更保险就显式写 --l 0,并在状态行确认 duty L=0.00 R=0.00。
#       (旧版这里写「a 会同时喂给差速、两轮原地自转」—— 那是 diff 模式的行为,已不适用。)

# 4b) 舵机抖 / 行程不对 —— 先量脉冲串,再怀疑接线,最后才怀疑舵机
python3 ~/servo_pulse_probe.py 30            # 量我们实际发出的脉宽(只测量,不驱动)
python3 ~/joy_inject.py --arm --servo-sweep  # a 从 -1 扫到 +1 逐档停,最后松掉
#    ⚠️ servo_pulse_probe 申请 line 32/22/118(当前仅有的三个空闲脚),用 Proxy 包住
#       真实的 _line,量的就是生产路径。**实测结论见文件 docstring**:静止时脉宽中位
#       1502µs、p1/p99 = 1498/1625、>150µs 的坏帧只有 0.4 次/秒
#       ⇒ **信号是干净的,「一直在抖」不是软件造成的**,别再改代码。
#    ⚠️ --servo-sweep 每档停 6 秒(必须 > 状态行的 5 秒间隔,否则一行都落不进段内),
#       全程 l=0 ⇒ 电机不励磁。最后一段 armed=false 是**刻意**的对照:松掉之后
#       **照样**抖 ⇒ 抖与我们的信号、与舵机的驱动电流都无关,只能往供电/共地/机械上找。

# 5) 编码器 (仅在接好线、use_encoders 置 true 之后)
#    手转轮子看计数是否变化 (motor_probe.py)
```

> 不用浏览器也能验证整条电机链路 —— `joy_inject.py` 直接向 `pwm_proc` 的 UDP 口灌值,
> 绕开 SocketIO。它是唯一能区分「网页链路坏」和「GPIO/电机坏」的工具。
> 它的 payload `{l, a, armed}` 与 `slam_viz._send_drive` **完全同构**,所以它也绕开了
> 网页那一层的驾驶权仲裁 —— 两个浏览器抢控制权时,它是可靠的那条路。

### 常见故障对照

| 现象 | 原因 | 解法 |
|---|---|---|
| `Permission denied` | gpiochip 是 root:700 | 跑上面的 chmod |
| 电机不转，但 gpio_smoke 正常 | 未共地 / VS 没供电 | 检查 GND 和电池 |
| **单侧**轮子反向(直行变成原地自转) | 该侧 IN 线序反 | 改 `flip_left` / `flip_right`(注意名字与物理轮相反) |
| **整体**转向反向(前进后退正常,左右转相反) | 左右通道交叉 | 改 `swap_lr`——**改 `flip_*` 无效**。⚠️ **仅在 `steer_mode: diff` 下成立**;`servo` 模式下转向不走轮子,该改 `servo_invert` |
| 电机一顿一顿、和摇杆不同步 | 软 PWM 线程 GIL 饿死控制环 | 见 `motor_hw.DualSoftPwm._busy_until`(已修:粗睡+精等) |
| 编码器计数为负 | A/B 反 | 改 `enc_sign_left/right` 为 `-1` |
| 电机啸叫 | 软 PWM 1kHz 落在共振点 | 降 `pwm_freq` 到 500 |
| `line busy` 启动失败 | `motor_driver` 正占着 GPIO | 先停它;两者互斥 |
| 直连路径下没有 `/odom` | 设计如此(开环,编码器未接) | SLAM 用 `USE_WHEEL_ODOM=0` 激光 ICP |
| 状态行一直是 `servo=off` | `servo_enable: false` | 那是默认值,开了才会申请 line |
| 状态行一直是 `servo=PARK` | 没点「启用电机」,或摇杆断流触发 deadman | park 是设计行为;`--arm` 灌值即可验证 |
| **舵机完全不动、日志看着一切正常** | `servo_enable:true` 但**没接线** —— 没有 libgpiod 报错、别的什么都不一样 | 看 `servo=` 那一列:它在发脉冲就说明软件侧是好的,问题在接线上 |
| `pwm_proc` 起不来,错误里提到舵机 line | `servo_line` 撞了别的 line / 被别的进程占着 | 注意这**不是** motor_driver 占用(那批 line 本进程已拿到);只跑电机就先 `servo_enable: false` |
| 舵机转向与车头相反 | 机械安装方向 | 改 `servo_invert`——**与 `swap_lr` 无关**,别改那个 |
| 舵机不在中位 | 机械中位偏 | 改 `servo_center_us` |
| **舵机静止时一直在轻微抖** | 实测(2026-09-15)**不是我们的信号**:`servo_pulse_probe.py` 量到静止时脉宽中位 1502µs、p1/p99 = 1498/1625、坏帧仅 0.4 次/秒 —— 舵机听不见 | 按可能性排:① 独立电源**没和犀牛派共地**(不共地没有参考电平,症状恰好就是「不转/抖/行程不全」);② 舵机臂没装在 1500µs 处 ⇒ 一直顶在机械限位堵转;③ 空载的廉价模拟舵机在中位做极限环振荡(装上连杆通常会安静)。先用 `--servo-sweep` 逐档停,一次定出中位/限位/`servo_invert` |
| 舵机嗡嗡响 / 发烫 | 顶在机械限位堵转(中位偏了) | 调 **`servo_center_us`**,不是 `servo_deadband` —— a=0 时指令本来就是 0,死区无事可做 |
| **相机/SLAM 先出问题、舵机也跟着乱** | 舵机把排针 5V 轨拉低压了 | 换**独立 5V BEC**(与犀牛派共地)。**软件检测不到低压**,别指望改参数能修 |
| **打方向时两轮 `duty` 纹丝不动** | `steer_mode: servo` 的**正常现象** | 不是故障:`a` 只去舵机。看状态行的 `steer=` 与 `servo=` 两列确认 |
| **打方向有舵机声,车却不转弯** | `steer_mode: servo` 下转向完全靠舵机机械连杆 | 轮子等速 ⇒ 没有差速贡献的偏航。检查连杆是否真的带动转向、舵机扭矩够不够 |
| `pwm_proc` 启动即退出,提示 `steer_mode=… 需要舵机` | `steer_mode` 选了 `servo`/`both` 但 `servo_enable: false` | 故意的:轮子已不吃 `a`,没舵机就是整车无法转向。二选一:开 `servo_enable`,或把 `steer_mode` 改回 `diff` |
