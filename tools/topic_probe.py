#!/usr/bin/env python3
"""从话题取一条消息,按字段路径打印(空格分隔);超时打印 TIMEOUT 并以 1 退出。

    python3 topic_probe.py /wheel_speed linear.z angular.x --timeout 6
    python3 topic_probe.py /odom pose.pose.position.x --timeout 6

为什么不用 `ros2 topic echo --once`:在 AidLux 上它会在 choose_qos 里调用
ros2 daemon 的 xmlrpc 接口并抛异常退出(与本节点无关),结果就是「话题明明在
发布、echo 却什么都拿不到」。用在测试脚本里会导致**静默假通过** —— 解析失败
返回 0,恰好等于「占空比已归零」的期望值。所以这里直连 rclpy,不碰 daemon。
"""

import sys
import time


def _msg_type(topic):
    if 'odom' in topic:
        from nav_msgs.msg import Odometry
        return Odometry
    if 'speed' in topic or 'cmd_vel' in topic:
        from geometry_msgs.msg import Twist
        return Twist
    from geometry_msgs.msg import Twist
    return Twist


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 2
    topic = argv[0]
    fields, timeout = [], 6.0
    i = 1
    while i < len(argv):
        if argv[i] == '--timeout':
            timeout = float(argv[i + 1])
            i += 2
        else:
            fields.append(argv[i])
            i += 1
    if not fields:
        fields = ['linear.x']

    import rclpy
    rclpy.init()
    node = rclpy.create_node('topic_probe')
    try:
        got = []
        node.create_subscription(
            _msg_type(topic), topic, lambda m: got.append(m), 10)
        deadline = time.monotonic() + timeout
        while not got and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not got:
            print('TIMEOUT')
            return 1
        vals = []
        for f in fields:
            v = got[0]
            for part in f.split('.'):
                v = getattr(v, part)
            vals.append('%g' % v)
        print(' '.join(vals))
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
