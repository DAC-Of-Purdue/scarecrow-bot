import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance


class NavigationNode(Node):

    def __init__(self):
        super().__init__("scarecrow_nav")
        self.subscription = self.create_subscription(
            Odometry,
            "odom",
            self.listener_callback,
            qos_profile=qos_profile_sensor_data,
        )

    def listener_callback(self, msg: Odometry):
        self.extract_msgs(msg.pose, msg.twist)

    def extract_msgs(self, pose: PoseWithCovariance, speed: TwistWithCovariance):
        print(f"{pose.pose.position.x:4.2f} {pose.pose.position.y:4.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
