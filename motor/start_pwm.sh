#!/bin/bash
# 犀牛派 直连 PWM 进程 启动脚本
# 用法: bash ~/start_pwm.sh
#
# 数据通路(整条不经过 ROS):
#     网页摇杆 60Hz → slam_viz → UDP 127.0.0.1:5813 → pwm_proc → GPIO
#
# 为什么必须独立进程:软 PWM 是 1kHz 纯 Python 忙等线程,留在 motor_driver 里
# 会和 rclpy 的单线程执行器抢同一个 GIL,把控制回调间歇性饿死 ——
# 实测 /wheel_speed 间隔中位 20.3ms 但 p90 116.6ms / p99 466.9ms,
# 松手后 duty 要连降数步、步步卡住,叠加成 931ms 的松手延迟。
#
# ⚠️ 独立进程只解决了「和 rclpy 抢」这一半 —— **不要**以为分完进程这类问题就没了。
#    同一个忙等线程在 pwm_proc 自己进程里又饿死了它自己的 200Hz 主循环(实测降到
#    1~2Hz,并撑爆 UDP 接收缓冲:发 60 包/s 只收到 24)。真凶是 `_busy_until` 对整个
#    PWM 周期纯自旋,motor_hw.py 里已改用「粗睡 + 末段精等」修掉。
#    详见 pwm_proc.py 模块注释与 motor_hw.DualSoftPwm._busy_until 的 docstring。

# ⚠️ pkill -f 匹配的是**整条命令行**,而调用本脚本的外层 shell(尤其是 ssh 的
#    `bash -c "... motor_driver.py ..."`,命令行里往往就含这些文件名)会被一起
#    命中 —— 2026-09-15 实测把自己的 ssh 会话杀断(exit 255)。
#    `^[^ ]*python3` 锚定到行首,只命中真正以 python3 起头的进程;
#    `[^ ]*` 同时覆盖 `python3` 和 `/usr/bin/python3` 两种写法。
MD_RE='^[^ ]*python3 .*motor_driver\.py'
PWM_RE='^[^ ]*python3 .*pwm_proc\.py'

# ⚠️⚠️ 预检必须在**动 motor_driver 之前**。
#    否则坏状态(无权开 gpiochip)下会先杀掉好的那个、再起不来新的 ⇒
#    留下「电机没驱动、车还能滑」的空窗,而且下面的报错只会说
#    「pwm_proc 没起来」,不会说「是我先把好的那个杀了」。判据看不出这个因果。
#
# 判据是**真的按 pwm_proc 的方式 open 一次** —— 不是 `[ ! -r ]`/`test -r`,
# 后者只看**权限位**,而这里真正要问的是「**内核让不让我开**」。用 python3 是因为
# 它一定在(下面的 pwm_proc 就用它),不引入 ldd/ioctl 那些额外的依赖。
# ⚠️ 这段的前身(2026-09-26~27,板上版本)就是 `[ ! -r ] || [ ! -w ]` —— 对已知的
#    「root:root 0600」那个坏状态它够用,但问的是**代理指标**,不是真问题。
if ! /usr/bin/python3 -c \
    'import os; fd = os.open("/dev/gpiochip0", os.O_RDWR); os.close(fd)' 2>/dev/null; then
  echo "!! /dev/gpiochip0 打不开(需要读写)—— 在**停 motor_driver 之前**退出,"
  echo "   以免留下「旧驱动已停、新驱动没起」的空窗。"
  echo "   当前状态(需要 root:gpio 660;权限位不对只是一个**可能**的原因):"
  # ⚠️ 两件事必须分开:节点**根本不存在** vs 节点在但**权限不对**。它们的修法完全不同
  #    (前者改权限毫无用处)。判据用 `[ -e ]` 直接问节点,而不是去 grep `ls` 的输出 ——
  #    实测 `ls` 失败时的报错文本里**就含 "gpiochip" 这个字符串**(`/dev/gpiochip*`),
  #    任何 `case *gpiochip*` / `grep gpiochip` 都会把"不存在"误判成"存在"。
  if [ -e /dev/gpiochip0 ]; then
    ls -l /dev/gpiochip* 2>&1 | sed 's/^/     /'
  else
    echo "     ↑ /dev/gpiochip0 **根本不存在** —— 那就不是权限问题:AidLux 的 /dev 是"
    echo "       tmpfs,容器/内核没导出这个节点,改权限改不出来(下面的修法对它无效)。"
  fi
  echo "   systemctl is-active fix-gpio-perms → $(systemctl is-active fix-gpio-perms 2>&1)"
  echo "   修法(正常情况下开机自愈服务会处理,这里是被绕过了):"
  echo "     sudo systemctl restart fix-gpio-perms"
  echo "   手动兜底(走 gpio 组 660,**不要** chmod 666):"
  echo "     sudo chgrp gpio /dev/gpiochip* && sudo chmod 660 /dev/gpiochip*"
  exit 6
fi

# ⚠️ GPIO 是独占的:必须先停掉 motor_driver,否则 pwm_proc 报 line busy 起不来。
# 用 SIGINT 而不是 SIGTERM,让 motor_driver 走 finally 里的干净关闭路径
# (duty 归零 + 方向脚 park),否则进程直接死掉会留下浮空的 ENA。
if pgrep -f "$MD_RE" > /dev/null; then
  echo "停掉 motor_driver(它占着 GPIO line)"
  pkill -INT -f "$MD_RE" 2>/dev/null
  for _ in $(seq 1 10); do pgrep -f "$MD_RE" > /dev/null || break; sleep 0.5; done
  if pgrep -f "$MD_RE" > /dev/null; then
    echo "  SIGINT 未退出,补 SIGTERM"
    pkill -f "$MD_RE" 2>/dev/null
    sleep 1
  fi
fi
pkill -f "$PWM_RE" 2>/dev/null
sleep 1

# 系统 python3:不导入 rclpy,不需要 ROS 环境
# (和 fire_detect.py / cam_stream.py 同一套路;ROS env 会带来别的坑)
setsid nohup /usr/bin/python3 ~/pwm_proc.py --params ~/motor_params.yaml \
  > /tmp/pwm_proc.log 2>&1 </dev/null &
echo "pwm_proc PID=$!"

sleep 2
if ! pgrep -f "$PWM_RE" > /dev/null; then
  echo "!! pwm_proc 没起来 —— 日志:"
  tail -20 /tmp/pwm_proc.log
  exit 1
fi

# ⚠️ 别用 `hostname -I | awk '{print $1}'` 只印第一个地址就当"就是这个"。本机同时有
#    多个地址(实测 `hostname -I` → "<LAN-IP> <LAN-IP>":前者是 wlan0,
#    后者是 AidLux 的 br-lan 网桥),而页面只在其中一个上可达;顺序也不保证。
#    只印第一个会给出打不开的 URL,且看起来完全正常。所以全列出来。
IPS=$(hostname -I 2>/dev/null)
echo "摇杆 UDP 127.0.0.1:5813   网页 http://<下面任一地址>:5800   点「启用电机」后才驱动"
echo "本机候选地址: ${IPS:-取不到}"
echo "  ⚠️ 多个地址时顺序不保证,哪个通要试 —— 或直接用你 SSH 连的那个地址"
echo "日志     tail -f /tmp/pwm_proc.log   (每 5 秒一行:包率/龄期/duty)"
