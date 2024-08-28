import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Twist, TransformStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile

class OdomTfPublisher(Node):

    def __init__(self):
        super().__init__('odom_tf_publisher')

        # Initialize subscriber to gazebo model states
        self.subscription = self.create_subscription(
            ModelStates,
            '/model_states',
            self.model_states_callback,
            QoSProfile(depth=10))

        # Initialize odom publisher
        self.odom_publisher = self.create_publisher(Odometry, '/odom', QoSProfile(depth=10))

        # Initialize TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.drone_name = 'iris_demo'  # Replace with your drone model name in Gazebo

    def model_states_callback(self, msg):
        try:
            # Find the index of the drone in the model states
            idx = msg.name.index(self.drone_name)

            # Extract pose and twist
            pose = msg.pose[idx]
            twist = msg.twist[idx]

            # Publish odometry
            # self.publish_odom(pose, twist)                                                                                    

            # Publish TF
            self.publish_tf(pose)

        except ValueError:
            self.get_logger().error(f'Model {self.drone_name} not found in /gazebo/model_states')

    def publish_odom(self, pose, twist):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Set position
        odom_msg.pose.pose = pose

        # Set velocity
        odom_msg.twist.twist = twist

        # Publish odom
        self.odom_publisher.publish(odom_msg)

    def publish_tf(self, pose):
        t = TransformStamped()

        # Set the header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # Fill in the transform data
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation = pose.orientation

        # Send the transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)

    node = OdomTfPublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()