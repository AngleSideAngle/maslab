from cv2.gapi import BGR2RGB
import rclpy
from typing import Optional
from cv2.typing import MatLike
from numpy._typing import NDArray
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from hardware_interfaces.msg import DetectedCube
from functools import reduce

import cv2
from cv_bridge import CvBridge
import numpy as np

from dataclasses import dataclass

CUBE_LENGTH = 0.0508  # meters


@dataclass
class ColorBound:
    min: NDArray[np.float64]
    max: NDArray[np.float64]


@dataclass
class UsefulDetectedCube:
    distance: float
    x: int
    y: int
    width: int
    height: int

    def to_ros_msg(self) -> DetectedCube:
        return DetectedCube(
            x=self.x,
            y=self.y,
            width=self.width,
            height=self.height,
            distance=self.distance,
        )


def is_finite(value: float) -> bool:
    return not (value == float("inf") or value == float("-inf"))


class CubeDetect(Node):

    def __init__(self):
        super().__init__("cube_detect")
        self.FOCAL_LENGTH = 1
        self.SENSOR_WIDTH = 1
        # TODO color bounds
        self.LOW_RED_BOUND = ColorBound(
            np.array([0, 100, 100]), np.array([10, 255, 255])
        )
        self.UPPER_RED_BOUND = ColorBound(
            np.array([170, 100, 100]), np.array([180, 255, 255])
        )
        self.GREEN_BOUND = ColorBound(np.array([40, 40, 40]), np.array([90, 255, 255]))

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            CompressedImage, "image_raw/compressed", self.image_callback, 10
        )
        self.cube_image_pub = self.create_publisher(Image, "cube_image/raw", 10)
        self.green_pub = self.create_publisher(DetectedCube, "green_cube", 10)
        self.red_pub = self.create_publisher(DetectedCube, "red_cube", 10)

    def image_callback(self, msg: CompressedImage) -> None:
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        red = self.find_object(hsv_frame, self.LOW_RED_BOUND, self.UPPER_RED_BOUND)
        green = self.find_object(hsv_frame, self.GREEN_BOUND)

        def display_bounding_box(obj: UsefulDetectedCube) -> None:
            cv2.rectangle(
                frame,
                (obj.x, obj.y),
                (obj.x + obj.width, obj.y + obj.height),
                (0, 0, 255),
                2,
            )
            cv2.putText(
                frame,
                f"{round(obj.distance)} m",
                (obj.x, obj.y),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 0, 255),
                2,
            )

        if red is not None and is_finite(red.distance):
            self.red_pub.publish(red.to_ros_msg())
            display_bounding_box(red)

        if green is not None and is_finite(green.distance):
            self.green_pub.publish(green.to_ros_msg())
            display_bounding_box(green)

        # rqt is freaking out when we give it a bgr image for some reason
        # TODO pr patch to rqt (skull)
        # since ros2 cv bridge sucks and doesn't let us use a format for the
        # compressed image, we need to convert it and use the raw image
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        debug_image = self.bridge.cv2_to_imgmsg(rgb_frame, "rgb8")
        self.cube_image_pub.publish(debug_image)

    def find_object(
        self, hsv_image: MatLike, *colors: ColorBound
    ) -> Optional[UsefulDetectedCube]:
        mask = reduce(
            lambda mask1, mask2: cv2.bitwise_or(mask1, mask2),
            map(lambda bound: cv2.inRange(hsv_image, bound.min, bound.max), colors),
        )
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None

        max_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(max_contour)

        # jank distance calculation
        width = np.sqrt(cv2.contourArea(max_contour))
        frame_width = hsv_image.shape[1]
        if width == 0:
            return None
        distance = (
            CUBE_LENGTH * frame_width / width * self.FOCAL_LENGTH / self.SENSOR_WIDTH
        )

        return UsefulDetectedCube(
            x=x,
            y=y,
            width=w,
            height=h,
            distance=distance,
        )


def main(args=None):
    rclpy.init(args=args)

    cube_detect = CubeDetect()

    rclpy.spin(cube_detect)

    cube_detect.destroy_node()
    rclpy.shutdown()
