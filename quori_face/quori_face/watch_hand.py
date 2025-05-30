# !/usr/bin/env python3
# 
# watch_hand.py
# 
# Zane Meyer

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image, PointCloud, CameraInfo
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
        color_info_sub = message_filters.Subscriber(self, CameraInfo, '/astra_ros/devices/default/color/camera_info')
        depth_sub = message_filters.Subscriber(self, Image, '/astra_ros/devices/default/depth/image')
        self.ts = message_filters.ApproximateTimeSynchronizer([color_img_sub, color_info_sub, depth_sub], queue_size=10, slop=0.5)
        self.ts.registerCallback(self.image_callback)

        self.bridge = CvBridge()

        self.hands = mp_hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.7, min_tracking_confidence=0.5)

    def image_callback(self, color_msg, info_msg, depth_msg):
        
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

                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

                depth_value = depth_image[center_y, center_x]
                if depth_value == 0:
                    self.get_logger().warn("Depth value at wrist is zero, skipping hand detection.")
                    return                

                fx = info_msg.k[0] # focal lengths
                fy = info_msg.k[4]
                cx = info_msg.k[2] # principal pts
                cy = info_msg.k[5]

                x_per_z= (center_x - cx) / fx
                y_per_z = (center_y - cy) / fy
                z = depth_value / 1000.0  # Convert from mm to meters

                out_pt = PointStamped()
                out_pt.point.x = x_per_z * z
                out_pt.point.y = y_per_z * z
                out_pt.point.z = z
                out_pt.header.frame_id = "quori/head_camera_optical"
                out_pt.header.stamp = self.get_clock().now().to_msg()

                self.get_logger().info(f"Detected hand at ({out_pt.point.x}, {out_pt.point.y}, {out_pt.point.z})")
                self.pub.publish(out_pt)
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, encoding='bgr8'))
        

def main(args=None):
    rclpy.init(args=args)
    node = HandWatcher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()