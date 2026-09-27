#!/bin/bash
# 犀牛派 二维云台 串口进程 启动脚本
# 用法: bash ~/start_gimbal.sh [start|stop|status]      (默认 start)
#
# 数据通路(全在本机,不经过 ROS):
#     网页云台摇杆 62.5Hz → slam_viz → UDP 127.0.0.1:5814 → gimbal_proc → /dev/ttyHS1(PIN_29) → 云台
#
# ⚠️ 爆炸半径恰好是 gimbal_proc **一个**进程(CLAUDE.md 的纪律)。
#    本脚本**不碰** GPIO、不碰 pwm_proc/motor_driver、不碰 slam_viz ——
#    串口和 GPIO 是两条独立资源,没有任何理由互相影响。
#
# ⚠️⚠️ 尤其**不要**在这里加 `pkill -f gimbal_proc` 之类的「顺手清理」去想当然地
#    对齐 `start_slam_viz.sh` 的样子 —— 那会让**以后每次重启网页都顺手杀掉云台**。
#    云台进程的重启是**本脚本**的事,不是网页的事。

set -u

GP_RE='^[^ ]*python3 .*gimbal_proc\.py'   # ⚠️ 必须锚定行首:见下方说明
PARAMS="$HOME/gimbal_params.yaml"
LOG=/tmp/gimbal_proc.log

# ⚠️ pkill -f 匹配的是**整条命令行**,调用本脚本的外层 shell(尤其是 ssh 的
#    `bash -c "... gimbal_proc.py ..."`)会被一起命中 —— 2026-09-15 实测把
#    自己的 ssh 会话杀断(exit 255)。`^[^ ]*python3` 锚定行首只命中真正的进程;
#    `[^ ]*` 同时覆盖 `python3` 与 `/usr/bin/python3` 两种写法。
# ⚠️ 同理:**判断「它跑没跑」也绝不能用 `pgrep -f "gimbal_proc"`** ——
#    在 ssh 里那条命令行本身就含这个字符串,会命中自己,得到假警报。

port_of() {   # 从 YAML 里取出 serial_port(容忍行尾注释)
  sed -n 's/^[[:space:]]*serial_port:[[:space:]]*\([^[:space:]#]*\).*/\1/p' \
    "$PARAMS" 2>/dev/null | head -1
}

case "${1:-start}" in

stop)
  if pgrep -f "$GP_RE" > /dev/null; then
    # ⚠️ 用 SIGTERM 不用 SIGKILL:gimbal_proc 的 shutdown() 要在退出前
    #    **发最后一帧 (0,0)**。SIGKILL 跳过它 ⇒ 云台保持最后的位置。
    #    (单向无 ACK 的协议里这个洞堵不上,只能靠云台自己的看门狗 —— 未知。)
    echo "停 gimbal_proc(SIGTERM,让它走完发 (0,0) 的关闭路径)"
    pkill -TERM -f "$GP_RE" 2>/dev/null
    for _ in $(seq 1 10); do pgrep -f "$GP_RE" > /dev/null || break; sleep 0.5; done
    if pgrep -f "$GP_RE" > /dev/null; then
      echo "  未退出,补 SIGKILL —— ⚠️ 这一路**不会**发最后的 (0,0),云台停在原位"
      pkill -KILL -f "$GP_RE" 2>/dev/null
    fi
  else
    echo "gimbal_proc 没在跑"
  fi
  ;;

status)
  if pgrep -f "$GP_RE" > /dev/null; then
    echo "gimbal_proc 在跑 (PID $(pgrep -f "$GP_RE" | tr '\n' ' '))"
  else
    echo "gimbal_proc 没在跑"
  fi
  P=$(port_of); echo "串口(PIN_29/PIN_30): ${P:-未读到}"
  echo "日志: $LOG"
  tail -3 "$LOG" 2>/dev/null || echo "  (还没有日志)"
  ;;

start|*)
  [ -f "$PARAMS" ] || { echo "!! 找不到 $PARAMS"; exit 3; }

  PORT=$(port_of)
  [ -n "$PORT" ] || { echo "!! $PARAMS 里没读到 serial_port"; exit 4; }

  # ── 预检:节点到底叫什么 ──────────────────────────────────────────────
  # ⚠️ 这一步**不能省**。板上同时存在 ttyHS1 和 ttyHS3,对应**两块不同的排针**,
  #    而选错的那个是「open 成功、write 返回 6、日志全绿、云台一动不动」——
  #    本项目最痛恨的失败形状。所以宁可在这里停下,把候选节点摆出来。
  # ⚠️ 别写成 `ls ... | sed ... || echo "没有"` —— 管道的退出码是 **sed** 的,
  #    恒为 0 ⇒ 那个 `||` **永远不触发**,「候选节点:」后面**空着什么都不印**。
  #    而「一个都没有」正是本预检最该说清楚的那一种情形(沉默的空输出 = 本项目最恨的形状)。
  echo "串口候选节点:"
  _HS=$(ls -l /dev/ttyHS* 2>/dev/null)
  if [ -n "$_HS" ]; then
    printf '%s\n' "$_HS" | sed 's/^/     /'
  else
    echo "     ⚠️ 一个 /dev/ttyHS* 都没有 —— **这本身就是结论**:节点没建,或板子不走这条 UART。"
    echo "        内核认得哪些 tty: ls /sys/class/tty/ | grep -i hs"
  fi
  if [ ! -e "$PORT" ]; then
    echo "!! $PARAMS 里写的 $PORT **不存在**。"
    echo "   板上真正的节点看上面那行;内核认得哪些: ls /sys/class/tty/ | grep -i hs"
    echo "   节点缺失(不是名字错)时可以靠 major:minor 建:mknod <节点> c <maj> <min>"
    echo "   ⚠️ 改完 $PARAMS 再跑本脚本。别猜 —— 选错的那个表现是「一切正常、云台不动」。"
    exit 7
  fi

  # 上一个实例必须先走掉:TIOCEXCL 是**独占**打开,第二个 opener 会 EBUSY。
  # 这也正是我们要的 —— 「别人偷偷往我的线上写字节」变成响亮失败。
  if pgrep -f "$GP_RE" > /dev/null; then
    echo "已有 gimbal_proc 在跑,先按 stop 的方式停掉(让它发完最后的 (0,0))"
    pkill -TERM -f "$GP_RE" 2>/dev/null
    for _ in $(seq 1 10); do pgrep -f "$GP_RE" > /dev/null || break; sleep 0.5; done
    pgrep -f "$GP_RE" > /dev/null && { echo "!! 旧的没退干净"; exit 1; }
  fi

  # 系统 python3:不导入 rclpy。
  # ⚠️ 这里**没有**「必须独立进程」那种理由(不像 pwm_proc 要躲 GIL 忙等),
  #    纯粹是分层:串口坏了只坏云台,不连累网页和地图;且这个进程要能单独重启。
  setsid nohup /usr/bin/python3 "$HOME/gimbal_proc.py" --params "$PARAMS" \
    > "$LOG" 2>&1 </dev/null &
  echo "gimbal_proc PID=$!"

  sleep 2
  if ! pgrep -f "$GP_RE" > /dev/null; then
    echo "!! gimbal_proc 没起来 —— 日志:"
    tail -20 "$LOG" 2>/dev/null | sed 's/^/     /'
    echo "   ⚠️ 直接跑一遍能看到它自己的完整报错(退出码:2 串口/3 参数文件/4 参数值):"
    echo "      /usr/bin/python3 ~/gimbal_proc.py --params ~/gimbal_params.yaml"
    exit 1
  fi

  echo "云台 UDP 127.0.0.1:5814   串口 $PORT   网页 http://<本机地址>:5800"
  echo "  ⚠️ 云台**没有 armed 门** —— 网页上拨动摇杆就直接动,不必点「启用电机」。"
  echo "  ⚠️ 云台的供电来源见 ~/gimbal_wiring.md(若取自排针 5V,会拖累相机/SLAM)"
  echo "日志     tail -f $LOG   (每 5 秒一行:帧率/包率/龄期/dx/dy/短写/端口状态)"
  echo "免浏览器 python3 ~/gimbal_inject.py --axis-sweep   (定方向/行程,先跑这个)"
  echo "串口诊断 /usr/bin/python3 ~/gimbal_proc.py --loopback      (需先 stop:串口独占)"
  echo "         ⚠️ 回环要短接 PIN_29↔PIN_16(**不是** 29↔30),测完必须拔掉跳线"
  ;;

esac
