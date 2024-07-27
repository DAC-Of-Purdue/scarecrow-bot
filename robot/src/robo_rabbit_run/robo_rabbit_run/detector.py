import rclpy
import cv2 as cv
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError


class RabbitDetectionNode(Node):

    def __init__(self):
        self.ARENA_SIZE = 500
        super().__init__("rabbit_detector")
        self.subscription = self.create_subscription(
            Image,
            "arena",
            self.listener_callback,
            qos_profile=qos_profile_sensor_data,
        )
        self.publisher = self.create_publisher(
            Image, "rabbit/location_image", qos_profile=qos_profile_sensor_data
        )
        self.debug_publisher = self.create_publisher(
            Image, "rabbit/debug", qos_profile=qos_profile_sensor_data
        )
        self.location_publisher = self.create_publisher(
            Point, "rabbit/location", qos_profile=qos_profile_sensor_data
        )
        self.bridge = CvBridge()
        self.find_transformation()

    def listener_callback(self, msg: Image):
        try:
            image_raw = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_top_view = self.top_view_transformation(image_raw)
            self.detect_rabbit()
        except CvBridgeError as e:
            self.get_logger().error("Failed to convert image: %s" % str(e))
            return

        try:
            detection_image = self.bridge.cv2_to_imgmsg(self.image_top_view, "bgr8")
            self.publisher.publish(detection_image)
        except CvBridgeError as e:
            self.get_logger().error("Failed to convert image: %s" % str(e))

    def find_transformation(self):
        self.H = cv.getPerspectiveTransform(
            # np.array(
            #     (  # arena 4 corner's coordinates
            #         (96, 114),  # upper left
            #         (688, 22),  # upper right
            #         (1210, 272),  # lower right
            #         (366, 682),  # lower left
            #     ),
            #     dtype="float32",
            # ),
            # np.array(
            #     (  # target coordinates
            #         (0, 0),  # upper left
            #         (self.ARENA_SIZE, 0),  # upper right
            #         (self.ARENA_SIZE, self.ARENA_SIZE),  # lower right
            #         (0, self.ARENA_SIZE),  # lower left
            #     ),
            #     dtype="float32",
            # ),
            np.array(
                (  # arena 4 corner's coordinates
                    (315, 308),  # upper left
                    (1059, 298),  # upper right
                    (1234, 681),  # lower right
                    (79, 695),  # lower left
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

    def top_view_transformation(self, image):
        """
        Apply perspective transformation
        """

        return cv.warpPerspective(
            image, self.H, (self.ARENA_SIZE, self.ARENA_SIZE - 100)
        )

    def locate_rabbit(self, contours: list):
        """extract moment on each contour and then average"""
        # initial locations
        x = 0
        y = 0
        for cnt in contours:
            moment = cv.moments(cnt)
            x += int(moment["m10"] / moment["m00"])
            y += int(moment["m01"] / moment["m00"])

        try:

            x = x / len(contours)
            y = y / len(contours)

            rabbit_location = Point(x=x, y=y, z=0.0)
            self.location_publisher.publish(rabbit_location)

            cv.putText(
                self.image_top_view,
                f"{x}, {y}",
                (int(x) - 20, int(y) - 20),
                cv.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 0),
                2,
            )
            cv.circle(self.image_top_view, (int(x), int(y)), 5, (255, 0, 255), -1)

        except ZeroDivisionError:
            ...

    def detect_rabbit(self):
        """Detect rabbit from top-view image then find the contours"""
        mask = cv.inRange(
            cv.cvtColor(self.image_top_view, cv.COLOR_BGR2HSV),
            np.array([0, 40, 200]),
            np.array([15, 100, 255]),
        )
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, np.ones((7, 7), np.uint8))
        self.debug_publisher.publish(self.bridge.cv2_to_imgmsg(mask, "mono8"))
        contours, _ = cv.findContours(  # find contours
            mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE
        )

        contours = [  # only keep contours that fit our criteria
            cnt for cnt in contours if cv.contourArea(cnt) > 20
        ]
        cv.drawContours(self.image_top_view, contours, -1, (0, 0, 255), 3)

        self.locate_rabbit(contours)


def main(args=None):
    rclpy.init(args=args)
    node = RabbitDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
