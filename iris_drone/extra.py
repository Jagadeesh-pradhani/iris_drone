import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetEntityState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, TransformStamped
from tf2_ros import TransformBroadcaster

class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')

        # Create a service client for the /get_entity_state service
        self.client = self.create_client(GetEntityState, '/get_entity_state')

        # Create a publisher for the /odom topic
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)

        # Create a TransformBroadcaster for publishing the transforms
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create a timer that triggers the update method at a specified rate
        self.timer = self.create_timer(1.0 / 20.0, self.timer_callback)  # 20 Hz

    def timer_callback(self):
        if not self.client.service_is_ready():
            self.get_logger().warning('/get_entity_state service is not ready')
            return

        # Create a request object
        request = GetEntityState.Request()
        request.name = 'iris_demo'

        # Send the request and get the response
        future = self.client.call_async(request)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error('Service call failed: %s', response)
                return

            state = response.state

            # Extract position and orientation
            position = state.pose.position
            orientation = state.pose.orientation

            # Get the current simulation time
            current_time = self.get_clock().now().to_msg()

            # Create and publish Odometry message
            odom_msg = Odometry()
            odom_msg.header.stamp = current_time
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'

            odom_msg.pose.pose = Pose(
                position=position,
                orientation=orientation
            )

            # Populate odom_msg.twist with the Twist information from the service response
            odom_msg.twist.twist = Twist(
                linear=state.twist.linear,
                angular=state.twist.angular
            )

            self.odom_publisher.publish(odom_msg)

            # Broadcast transform from odom to base_link
            transform_odom_to_base = TransformStamped()
            transform_odom_to_base.header.stamp = current_time
            transform_odom_to_base.header.frame_id = 'odom'
            transform_odom_to_base.child_frame_id = 'base_link'

            transform_odom_to_base.transform.translation.x = position.x
            transform_odom_to_base.transform.translation.y = position.y
            transform_odom_to_base.transform.translation.z = position.z
            transform_odom_to_base.transform.rotation = orientation

            self.tf_broadcaster.sendTransform(transform_odom_to_base)

            # Broadcast transform from base_link to base_scan
            transform_base_to_scan = TransformStamped()
            transform_base_to_scan.header.stamp = current_time
            transform_base_to_scan.header.frame_id = 'base_link'
            transform_base_to_scan.child_frame_id = 'base_scan'

            # Assuming base_scan is offset by some fixed values from base_link
            transform_base_to_scan.transform.translation.x = 0.0
            transform_base_to_scan.transform.translation.y = 0.0
            transform_base_to_scan.transform.translation.z = 0.075077
            transform_base_to_scan.transform.rotation.x = 0.0
            transform_base_to_scan.transform.rotation.y = 0.0
            transform_base_to_scan.transform.rotation.z = 0.0
            transform_base_to_scan.transform.rotation.w = 1.0

            self.tf_broadcaster.sendTransform(transform_base_to_scan)

            self.get_logger().info('Published odometry and transforms.')

        except Exception as e:
            self.get_logger().error(f'Failed to get robot state: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = RobotStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
