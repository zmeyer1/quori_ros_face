# !/usr/bin/env python3

# gaze_controller.py

# Zane Meyer

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from quori_face_msgs.msg import Eye, Face, Mouth
from quori_face_msgs.srv import FaceQuery

import cv2, time
import numpy as np

class GazeController(Node):
    def __init__(self):
        super().__init__("face_writer")

        self.create_subscription(PointStamped, 'quori_face/focal_point', self.update_face, 10)

        self.declare_parameter("max_blink_period", 10.0)
        self.declare_parameter("PPI", 100.0) # Roughly an average pixels per inch default for Quori's screen
    

    def update_face(self, msg):
        """Takes in a Point in 3D space and moves the eyes to look at that point"""
        # Convert point to the eye reference frame
        # Normalize the vector to size: eye_radius / PPI / 0.0254
        # Do the math to determine the current pupil position in xy
        # convert the xy location to a percentage of the screen using PPI and the screensize


        
def main(args=None):
    rclpy.init(args=args)    
    tracker = GazeController()

    rclpy.spin(tracker)
    rclpy.shutdown()

if __name__ == "__main__":
    main()