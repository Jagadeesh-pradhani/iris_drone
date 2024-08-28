import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math

class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)

        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

    def odom_callback(self, msg):
        # Extract odometry data
        odom_position = msg.pose.pose.position
        odom_orientation = msg.pose.pose.orientation

        # Publish transform from odom to base_link
        self.publish_transform(msg.header.stamp, odom_position, odom_orientation, 'odom', 'base_link')

        # Define the offset for base_scan relative to base_link
        base_scan_offset = TransformStamped()
        base_scan_offset.header.stamp = msg.header.stamp
        base_scan_offset.header.frame_id = 'base_link'
        base_scan_offset.child_frame_id = 'base_scan'

        # Applying the base_scan offset (0, 0, 0.075077) as defined in the SDF file
        base_scan_offset.transform.translation.x = 0.0
        base_scan_offset.transform.translation.y = 0.0
        base_scan_offset.transform.translation.z = 0.075077

        # No rotation between base_link and base_scan
        base_scan_offset.transform.rotation.x = 0.0
        base_scan_offset.transform.rotation.y = 0.0
        base_scan_offset.transform.rotation.z = 0.0
        base_scan_offset.transform.rotation.w = 1.0

        # Publish the transform
        self.tf_broadcaster.sendTransform(base_scan_offset)

    def publish_transform(self, stamp, position, orientation, parent_frame, child_frame):
        t = TransformStamped()

        t.header.stamp = stamp
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame

        t.transform.translation.x = position.x
        t.transform.translation.y = position.y
        t.transform.translation.z = position.z
        t.transform.rotation = orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)

    node = OdomToTF()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
