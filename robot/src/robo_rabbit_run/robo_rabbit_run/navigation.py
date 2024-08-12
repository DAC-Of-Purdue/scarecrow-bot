import math
import rclpy
import numpy as np
import cv2 as cv
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Point, Twist


class NavigationNode(Node):

    def __init__(self):
        self.ARENA_SIZE = 500
        super().__init__("scarecrow_nav")
        self.rabbit_location_sub = self.create_subscription(
            Point,
            "rabbit/location",
            self.rabbit_location_callback,
            qos_profile=qos_profile_sensor_data,
        )

        self.scarecrow_location_sub = self.create_subscription(
            Point,
            "scarecrow/location",
            self.scarecrow_location_callback,
            qos_profile=qos_profile_sensor_data,
        )
        # self.odom_sub = self.create_subscription(
        #     Odometry,
        #     "odom",
        #     self.odom_callback,
        #     qos_profile=qos_profile_sensor_data,
        # )
        self.control_publisher = self.create_publisher(
            Twist, "cmd_vel", qos_profile=qos_profile_system_default
        )
        self.prev_location = None
        self.current_heading = None
        self.find_transformation()

    def odom_callback(self, msg: Odometry):
        self.extract_msgs(msg.pose, msg.twist)

    def extract_msgs(self, pose: PoseWithCovariance, speed: TwistWithCovariance):
        # print(
        #     f"raw location ({pose.pose.position.x:5.3f}, {pose.pose.position.y:5.3f})"
        # )
        location = self.plane_transformation(
            Point(x=pose.pose.position.x, y=pose.pose.position.y, z=1.0)
        )
        if self.prev_location:
            self.current_heading = self.calc_heading(self.prev_location, location)
            self.robot_vec = self.calc_vector(self.prev_location, location)
        self.prev_location = location  # save previous location

    def calc_vector(self, loc_a: Point, loc_b: Point):
        return np.array([loc_b.x - loc_a.x, loc_b.y - loc_a.y])

    def calc_heading(self, loc_a: Point, loc_b: Point):
        diff_x = math.radians(loc_b.x - loc_a.x)
        diff_y = math.radians(loc_b.y - loc_a.y)

        heading = math.degrees(math.atan2(diff_y, diff_x))

        if heading < 0:
            heading += 360

        return heading

    def find_transformation(self):
        self.H = cv.getPerspectiveTransform(
            np.array(
                (  # arena 4 corner's coordinates in ROS domain in mm (time 1000)
                    (-4071, -3036),  # upper left
                    (-3504, -20),  # upper right
                    (-1184, -453),  # lower right
                    (-1591, -3301),  # lower left
                ),
                dtype="float32",
            ),
            np.array(
                (  # target coordinates
                    (0, 0),  # upper left
                    (self.ARENA_SIZE, 0),  # upper right
                    (self.ARENA_SIZE, self.ARENA_SIZE - 100),  # lower right
                    (0, self.ARENA_SIZE - 100),  # lower left
                ),
                dtype="float32",
            ),
        )

    def plane_transformation(self, location: Point) -> Point:
        """
        Apply perspective transformation
        """

        location = np.array(
            [location.x * 1000, location.y * 1000, location.z], dtype=np.float32
        )
        transformed_location = np.matmul(self.H, location)
        scale = transformed_location[2]
        return Point(
            x=transformed_location[0] / scale, y=transformed_location[1] / scale, z=1.0
        )

    def rabbit_location_callback(self, msg: Point):
        dist = self.calc_distance(msg)
        self.chasing(dist, msg)

    def calc_distance(self, rabbit_location: Point) -> float:
        if self.prev_location is None:
            return 0

        return math.sqrt(
            (self.prev_location.x - rabbit_location.x) ** 2
            + (self.prev_location.y - rabbit_location.y) ** 2
        )

    def calc_angle_two_vectors(self, v1, v2):
        def unit_vector(vector):
            """Returns the unit vector of the vector."""
            return vector / np.linalg.norm(vector)

        """ Returns the angle in radians between vectors 'v1' and 'v2'::

                >>> angle_between((1, 0, 0), (0, 1, 0))
                1.5707963267948966
                >>> angle_between((1, 0, 0), (1, 0, 0))
                0.0
                >>> angle_between((1, 0, 0), (-1, 0, 0))
                3.141592653589793
        """
        v1_u = unit_vector(v1)
        v2_u = unit_vector(v2)
        return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

    def chasing(self, dist: float, location: Point):
        cmd = Twist()
        if self.prev_location is not None and self.current_heading is not None:
            cmd.angular.z = 0.0
            speed = 0.0 if dist < 30 else 1.0
            cmd.linear.x = speed / 2
            target_heading = self.calc_heading(self.prev_location, location)
            diff_heading = target_heading - self.current_heading
            print(
                f" target heading = {target_heading:3.0f} current heading = {self.current_heading:3.0f} diff = {diff_heading:4.0f} {dist:4.0f}"
            )
            print(self.prev_location)

            cmd.angular.z = np.clip((diff_heading / 50) * -1 * speed, -1.0, 1.0)
            print(cmd.angular)
        else:
            cmd.linear.x = 0.2
        self.control_publisher.publish(cmd)

    def scarecrow_location_callback(self, msg: Point):
        if self.prev_location and self.calc_distance(msg) > 3:
            self.current_heading = self.calc_heading(self.prev_location, msg)
            self.prev_location = msg
        if self.prev_location is None:
            self.prev_location = msg  # save previous location


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
