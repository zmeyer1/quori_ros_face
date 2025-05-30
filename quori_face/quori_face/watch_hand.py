# !/usr/bin/env python3
# 
# watch_hand.py
# 
# Zane Meyer

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image, PointCloud
import cv2
from cv_bridge import CvBridge
import message_filters
import numpy as np

import mediapipe as mp


mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

class HandWatcher(Node):

    def __init__(self):
        super().__init__('hand_watcher')

        self.pub = self.create_publisher(PointStamped, 'quori_face/focal_point', 10)
        self.image_pub = self.create_publisher(Image, 'hand_detection', 10)

        color_img_sub = message_filters.Subscriber(self, Image, '/astra_ros/devices/default/color/image_color')
        point_cld_sub = message_filters.Subscriber(self, PointCloud, '/astra_ros/devices/default/point_cloud')
        self.ts = message_filters.ApproximateTimeSynchronizer([color_img_sub, point_cld_sub], queue_size=10, slop=2**30)
        self.ts.registerCallback(self.image_callback)

        self.bridge = CvBridge()

        self.hands = mp_hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.7, min_tracking_confidence=0.5)

    def image_callback(self, color_msg, pt_cloud_msg):
        
        self.get_logger().error(f"Stepping into callback")
        try:
            image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            im_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        results = self.hands.process(im_rgb)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                # Get the wrist position as center pos
                wrist = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST]
                center_x = int(wrist.x * image.shape[1])
                center_y = int(wrist.y * image.shape[0])

                # point cloud stuff
                loc = pt_cloud_msg.points[center_x * image.shape[1] + center_y]

                pt = PointStamped()
                pt.header.frame_id = "quori/head_camera_optical"
                pt.header.stamp = self.get_clock().now().to_msg()
                pt.point.x = float(loc.x)
                pt.point.y = float(loc.y)
                pt.point.z = float(loc.z)

                self.get_logger().info(f"Detected hand at ({loc.x}, {loc.y}, {loc.z})")

                self.pub.publish(pt)
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, encoding='bgr8'))
        

def main(args=None):
    rclpy.init(args=args)
    node = HandWatcher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()