import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Point


class NavigationNode(Node):

    def __init__(self):
        self.SCALE = (100, 100)  # (x, y) scale factor from meter to pixel
        super().__init__("scarecrow_nav")
        self.odom_sub = self.create_subscription(
            Odometry,
            "odom",
            self.odom_callback,
            qos_profile=qos_profile_sensor_data,
        )
        self.rabbit_location_sub = self.create_subscription(
            Point,
            "rabbit/location",
            self.rabbit_location_callback,
            qos_profile=qos_profile_sensor_data,
        )
        self.odom = PoseWithCovariance()  # current location

    def odom_callback(self, msg: Odometry):
        self.extract_msgs(msg.pose, msg.twist)

    def extract_msgs(self, pose: PoseWithCovariance, speed: TwistWithCovariance):
        print(f"{pose.pose.position.x:4.2f} {pose.pose.position.y:4.2f}")
        self.odom = pose.pose

    def rabbit_location_callback(self, msg: Point):
        dist = self.calc_distance(msg)
        print(dist)

    def calc_distance(self, rabbit_location: Point) -> float:
        return math.sqrt(
            (self.odom.pose.position.x - rabbit_location.x) ** 2
            + (self.odom.pose.position.y - rabbit_location.y) ** 2
        )


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
