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
        self.ARENA_SIZE = 400
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

        self.timer = self.create_timer(0.25, self.chasing)
        # self.odom_sub = self.create_subscription(
        #     Odometry,
        #     "odom",
        #     self.odom_callback,
        #     qos_profile=qos_profile_sensor_data,
        # )
        self.control_publisher = self.create_publisher(
            Twist, "cmd_vel", qos_profile=qos_profile_system_default
        )
        self.rabbit_location_current = None
        self.turtle_location_current = None
        self.turtle_location_prev = None
        # self.turtle_heading_current = None
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
        if self.turtle_location_prev:
            self.turtle_heading_current = self.calc_heading(
                self.turtle_location_prev, location
            )
            self.robot_vec = self.calc_vector(self.turtle_location_prev, location)
        self.turtle_location_prev = location  # save previous location

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
        self.rabbit_location_current = msg

    def calc_distance(self, location_a: Point, location_b: Point) -> float:

        return math.sqrt(
            (location_a.x - location_b.x) ** 2 + (location_a.y - location_b.y) ** 2
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

    def chasing(self):
        cmd = Twist()
        if self.turtle_location_prev is not None:
            if (
                self.calc_distance(
                    self.turtle_location_current, self.turtle_location_prev
                )
                > 16
            ):
                self.turtle_location_prev = self.turtle_location_current

            dist = self.calc_distance(
                self.rabbit_location_current, self.turtle_location_current
            )
            cmd.angular.z = 0.0
            speed = 0.0 if dist < 30 else 1.0
            cmd.linear.x = speed
            heading_current = self.calc_heading(
                self.turtle_location_prev, self.turtle_location_current
            )
            heading_target = self.calc_heading(
                self.turtle_location_current, self.rabbit_location_current
            )
            diff_heading = heading_target - heading_current
            print(
                f" target heading = {heading_target:3.0f} current heading = {heading_current:3.0f} diff = {diff_heading:4.0f} {dist:4.0f}"
            )
            # print(self.turtle_location_prev)
            if np.abs(diff_heading) > 90:
                # only turn
                cmd.linear.x = 0.3
                cmd.angular.z = -1.0 if heading_target > heading_current else 1.0

            else:
                if np.abs(diff_heading) > 180:
                    cmd.angular.z = np.clip((diff_heading / 30) * speed, -1.0, 1.0)
                else:
                    cmd.angular.z = np.clip((diff_heading / 30) * -1 * speed, -1.0, 1.0)
            print(cmd.linear.x, cmd.angular.z)

        else:
            # first event. Then it will have turtle_location_prev
            cmd.linear.x = 0.5
            self.turtle_location_prev = self.turtle_location_current

        self.control_publisher.publish(cmd)

    def scarecrow_location_callback(self, msg: Point):
        self.turtle_location_current = msg
        # if self.turtle_location_prev and self.calc_distance(self.turtle_location_prev, msg) > 3:
        #     self.turtle_heading_current = self.calc_heading(
        #         self.turtle_location_prev, msg
        #     )
        #     self.turtle_location_prev = msg
        # if self.turtle_location_prev is None:
        #     self.turtle_location_prev = msg  # save previous location


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
