"""Static board acceptance; subscribes only, never sends drive commands."""
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from tf2_ros import Buffer, TransformListener


def main():
    rclpy.init()
    node = Node('gyro_mapping_acceptance')
    counts, last, detail = {}, {}, {}
    def receive(name):
        def callback(msg):
            counts[name] = counts.get(name, 0)+1
            last[name] = time.monotonic()
            if name == 'map':
                detail['map_size'] = [msg.info.width, msg.info.height]
                detail['occupied_cells'] = sum(v > 50 for v in msg.data)
            if name == 'imu':
                detail['gyro_rad_s'] = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
            if name == 'odom':
                detail['odom_xy'] = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        return callback
    subscriptions = []
    for name, topic, typ in [('imu','/imu/data_raw',Imu), ('scan','/scan',LaserScan), ('odom','/odom',Odometry)]:
        subscriptions.append(node.create_subscription(typ, topic, receive(name), qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(OccupancyGrid, '/map', receive('map'),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    start = time.monotonic()
    while time.monotonic()-start < 20.:
        rclpy.spin_once(node, timeout_sec=.1)
    now = time.monotonic()
    ages = {key: round(now-value, 3) for key, value in last.items()}
    publishers = [p.node_name for p in node.get_publishers_info_by_topic('/odom')]
    transform = buffer.can_transform('map', 'base_link', rclpy.time.Time())
    result = {'counts_20s':counts, 'ages_s':ages, 'odom_publishers':publishers,
              'map_to_base_link':transform, **detail}
    healthy = (all(counts.get(k,0)>1 for k in ('imu','scan','odom','map')) and
               ages.get('odom',999)<1 and ages.get('scan',999)<2 and ages.get('map',999)<10 and
               ages.get('imu',999)<.2 and publishers == ['laser_imu_odom'] and
               transform and detail.get('occupied_cells',0)>0)
    result['healthy'] = healthy
    print(json.dumps(result))
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if healthy else 1)


if __name__ == '__main__':
    main()
