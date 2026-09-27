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

# ⚠️ GPIO 权限预检 —— 必须在停 motor_driver 之前做。
#    本机 /dev/gpiochip* 每次重启都会退回 root:root 0600(AidLux 容器内 /dev 是
#    tmpfs,gpiochip 没有 uevent ⇒ /etc/udev/rules.d 里那条规则永远不会触发)。
#    正常由开机服务 fix-gpio-perms.service 修正;这里兜底。
#    放在最前面是因为:如果权限不对,先停掉 motor_driver 再失败,车会处于
#    「旧驱动没了、新驱动没起」的空窗 —— 宁可什么都不动。
if [ ! -r /dev/gpiochip0 ] || [ ! -w /dev/gpiochip0 ]; then
  echo "❌ /dev/gpiochip0 无读写权限(当前 $(stat -c %U:%G /dev/gpiochip0) $(stat -c %a /dev/gpiochip0))"
  echo "   修法: sudo chgrp gpio /dev/gpiochip* && sudo chmod 660 /dev/gpiochip*"
  echo "   或:   sudo systemctl restart fix-gpio-perms"
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
#    多个地址(实测 `hostname -I` → "192.168.31.220 192.168.1.1":前者是 wlan0,
#    后者是 AidLux 的 br-lan 网桥),而页面只在其中一个上可达;顺序也不保证。
#    只印第一个会给出打不开的 URL,且看起来完全正常。所以全列出来。
IPS=$(hostname -I 2>/dev/null)
echo "摇杆 UDP 127.0.0.1:5813   网页 http://<下面任一地址>:5800   点「启用电机」后才驱动"
echo "本机候选地址: ${IPS:-取不到}"
echo "  ⚠️ 多个地址时顺序不保证,哪个通要试 —— 或直接用你 SSH 连的那个地址"
echo "日志     tail -f /tmp/pwm_proc.log   (每 5 秒一行:包率/龄期/duty)"
