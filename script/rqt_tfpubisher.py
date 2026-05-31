import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('tf_publisher')
    broadcaster = tf2_ros.TransformBroadcaster(node)

    timer_period = 0.1  # seconds
    def timer_callback():
        t = TransformStamped()
        t.header.stamp = node.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'baselink'
        t.transform.translation.x = 1.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        broadcaster.sendTransform(t)
        print('Published transform from map to baselink')

    timer = node.create_timer(timer_period, timer_callback)

    rclpy.spin(node)

    node.destroy_timer(timer)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
