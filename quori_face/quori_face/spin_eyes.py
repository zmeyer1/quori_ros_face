# !/usr/bin/env python3

# debug_eyes.py

# Zane Meyer

import rclpy
from rclpy.node import Node
from quori_face_msgs.msg import Eye, Face

import time
import numpy as np

class EyeSpinner(Node):
    def __init__(self):
        super().__init__("eye_spinner")

        self.pub = self.create_publisher(Face, 'quori_face/face_cmd', 10)
        self.timer = self.create_timer(0.1, self.move_eyes) # update image at 10 Hz
        self.start = time.time()

    def move_eyes(self):
        face = Face()
        for eye in range(2):
            msg = Eye()
            msg.pupil = [0.05*np.cos(0.5*2*np.pi * (time.time() - self.start)), 0.05*np.sin(0.5*2*np.pi * (time.time() - self.start))]
            face.eyes.append(msg)

        self.pub.publish(face)

def main(args=None):
  rclpy.init(args=args)
  node = EyeSpinner()
  rclpy.spin(node)
  rclpy.shutdown()


if __name__ == "__main__":
  main()