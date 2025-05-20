# !/usr/bin/env python3

# display_face.py

# Zane Meyer

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor

import cv2
import numpy as np

class FaceWriter(Node):
    def __init__(self, screen_size):
        super().__init__("face_writer")
        self.pub = self.create_publisher(Image, '/face', 10)
        self.timer = self.create_timer(0.1, self.publish_img)
        self.bridge = CvBridge()
        self.face_pose = {
            "left_eye": {
                "center": (0.33, 0.33),
                "pupil": (0.33, 0.33),
                "radius": 140,
                "thickness": 15,
                "pupil_size": 60,
            },
            "right_eye": {
                "center": (0.67, 0.33),
                "pupil": (0.67, 0.33),
                "radius": 140,
                "thickness": 15,
                "pupil_size": 60,
            },
            "mouth": {
                "left_corner": (0.40, 0.80),
                "right_corner": (0.60, 0.80),
                "upper_lip": (0.50, 0.82),
                "lower_lip": (0.50, 0.85)
            },
        }
        self.screen_size = screen_size

    def set_pupil(self, eye_dict, pupil_center=None, pupil_size=None):
        """Moves the pupil to a location"""
        if pupil_size:
            eye_dict["pupil_size"] = pupil_size
        if pupil_center:
            eye_dict["pupil"] = pupil_center

    def _ratio2pt(self, point):
        """converts point fractions into indices using the screen size"""
        pt =  np.multiply((self.screen_size[1], self.screen_size[0]), point)
        return np.int32(pt)

    
    def _pt2ratio(self, point):
        """converts indices to point ratios of the screen size"""
        return (point[0]/self.screen_size[1], point[1]/self.screen_size[0])

    def draw_eye(self, im, eye_dict, color = (255,30,0)):
        """Color is in BGR, not RGB."""
        cv2.circle(im, self._ratio2pt(eye_dict["center"]), eye_dict["radius"], color, eye_dict["thickness"])
        cv2.circle(im, self._ratio2pt(eye_dict["pupil"]), eye_dict["pupil_size"], color, -1)


    def draw_mouth(self, im, mouth_dict, color = (255,30,0)):
        """Color is in BGR, not RGB."""
        pass # TBD

    def publish_img(self):
        # create img
        im = np.zeros(self.screen_size)
        self.draw_eye(im, self.face_pose["left_eye"])
        self.draw_eye(im, self.face_pose["right_eye"])
        self.draw_mouth(im, self.face_pose["mouth"])

        img = self.bridge.cv2_to_imgmsg(im)
        self.pub.publish(img)

class FaceDisplay(Node):
    def __init__(self):
        super().__init__("face_display")
        self.sub = self.create_subscription(Image, '/face', self.display, 10)
        self.bridge = CvBridge()
        self.window = cv2.namedWindow("Face_Display", cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty('Face_Display', cv2.WND_PROP_ASPECT_RATIO, cv2.WINDOW_FREERATIO)
        cv2.setWindowProperty('Face_Display', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        cv2.waitKey(100) # sleep while letting cv2 set up
        _, _, x, y = cv2.getWindowImageRect("Face_Display")
        self.size = (y,x,3)
        self.image = np.zeros(self.size)
        self.fullscreen = True

    def display(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg)
        if self.fullscreen:
            cv2.setWindowProperty('Face_Display', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        else:
            cv2.setWindowProperty('Face_Display', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)


def generate_face(args=None):
    rclpy.init(args=args)
    node = FaceWriter()
    rclpy.spin(node)
    rclpy.shundown()


def draw_face(args=None):
    rclpy.init(args=args)
    exec = MultiThreadedExecutor()
    display = FaceDisplay()
    writer = FaceWriter(screen_size = display.size)
    exec.add_node(display)
    exec.add_node(writer)

    while rclpy.ok():
        cv2.imshow("Face_Display", display.image)
        key = cv2.waitKey(1)
        if (key & 0xFF) == ord('q') or (key & 0xFF) == ord('f'):
            display.fullscreen = not display.fullscreen
        elif key == 27:
            break
        exec.spin_once()
    rclpy.shutdown()

if __name__ == "__main__":
    draw_face()