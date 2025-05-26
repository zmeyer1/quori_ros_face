# !/usr/bin/env python3

# calibrate_face.py

# Zane Meyer

import rclpy
from rclpy.node import Node
from quori_face_msgs.msg import Eye, Face, Mouth

import cv2
import numpy as np

class FaceCalibrator(Node):
    def __init__(self):
        super().__init__("face_calibrator")

        self.pub = self.create_publisher(Face, 'quori_face/face_cmd', 10)
        self.count = 0
        self.timer = self.create_timer(0.5, self.move_eyes) # update image at 2 Hz

    def move_eyes(self):
        face = Face()
        self.count += 1
        for position in [0.33, 0.67]:
            msg = Eye()
            if (self.count // 10) % 2:
              msg.pupil = [position+0.02, 0.33]
            else:
              msg.pupil = [position-0.02, 0.33]
            face.eyes.append(msg)

        self.pub.publish(face)

def main(args=None):
  rclpy.init(args=args)
  node = FaceCalibrator()
  rclpy.spin(node)
  rclpy.shutdown()


if __name__ == "__main__":
  main()