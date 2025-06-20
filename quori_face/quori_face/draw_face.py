# !/usr/bin/env python3

# draw_face.py

# Zane Meyer

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from quori_face_msgs.msg import Eye, Face
from quori_face_msgs.srv import FaceQuery, ScreenSize
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from enum import Enum

import cv2, time
import numpy as np

class DefaultColors(Enum):
    TEAL = (255, 60, 0) # BGR format
    BLACK = (0, 0, 0)
    RED = (0, 0, 255)
    MAJENTA = (255, 0, 230)
    YELLOW = (0, 255, 255)

HEAD_DIAMETER = 0.2
MPERINCH = 0.0254

class FaceWriter(Node):
    def __init__(self, screen_size):
        super().__init__("face_writer")

        self.timer = self.create_timer(0.05, self.blink_manager) # update image at 20 Hz
        self.create_subscription(Face, "quori_face/face_cmd", self.update_face, 10)
        self.create_service(FaceQuery, "quori_face/query_face", self.get_face)
        self.create_service(ScreenSize, "quori_face/get_screensize", self.get_screensize)

        self.tf_broadcaster = StaticTransformBroadcaster(self)

        self.declare_parameter("max_blink_period", 10.0)
        self.declare_parameter("PPI", 100.0) # Roughly an average pixels per inch default for Quori's screen
    
        # coordinates are fractions of the screen size: [0,0] is the top left, [1,1] is the bottom right
        # sizes are in pixels, angles in degrees
        self.face_pose = {
            "left_eye": {
                "center": [0.23, 0.4],
                "pupil": [0.05, 0.05], # relative to center
                "radius": 70,
                "thickness": 8,
                "pupil_size": 25,
                "eyelid": [0.23, 0.1],
                "eyelid_angle": -10,
                "eyelid_size": [160,75],
            },
            "right_eye": {
                "center": [0.77, 0.4],
                "pupil": [-0.05, 0.05], # relative to center
                "radius": 70,
                "thickness": 8,
                "pupil_size": 25,
                "eyelid": [0.77, 0.1],
                "eyelid_angle": 10,
                "eyelid_size": [160,75],
            },
            "mouth": {
                "left_corner": [0.25, 0.62],
                "right_corner": [0.75, 0.62],
                "upper_lip": [0.50, 0.64],
                "lower_lip": [0.50, 0.68]
            },
        }
        self.screen_size = screen_size
        self.image = np.zeros(self.screen_size)
        self.blink_state = 0.0 # 1 is fully closed, 0 is fully open
        self.blink_direction = 1
        self.last_blink = time.time()
        self.blink_pause = 0
        self.update_face_tf()

    def get_face(self, request, response):

        for idx in ["left_eye", "right_eye"]:
            eye = Eye()
            for k,v in self.face_pose[idx].items():
                setattr(eye, k, v)
            response.face.eyes.append(eye)
        for k,v in self.face_pose["mouth"].items():
            setattr(response.face.mouth, k, v)
        return response

    def get_screensize(self, request, response):
        response.screensize = self.screen_size
        return response

    def update_face(self, msg):
        self._update_prop(self.face_pose["left_eye"], msg.eyes[0])
        self._update_prop(self.face_pose["right_eye"], msg.eyes[1])
        self.update_face_tf()
        self._update_prop(self.face_pose["mouth"], msg.mouth)

    def _update_prop(self, eye_dict, eye):
        for key in eye_dict.keys():
            if getattr(eye, key):
                eye_dict[key] = getattr(eye, key)

    def _ratio2pt(self, point):
        """converts point fractions into indices using the screen size"""
        pt =  np.multiply((self.screen_size[1], self.screen_size[0]), point)
        return np.int32(pt)
    
    def _pt2ratio(self, point):
        """converts indices to point ratios of the screen size"""
        return (point[0]/self.screen_size[1], point[1]/self.screen_size[0])

    def update_face_tf(self):
        """Broadcasts the face pose as a TF2 transform."""

        ppi = self.get_parameter("PPI").get_parameter_value().double_value

        t_left, t_right = TransformStamped(), TransformStamped()
        for t, eye in zip([t_left, t_right], ["left_eye", "right_eye"]):
            # Convert radius from pixels to meters
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "quori/head" # the base of quori's skull
            t.child_frame_id = f"quori/head_{eye}"
            t.transform.translation.x = HEAD_DIAMETER / 2
            t.transform.translation.y = (self.face_pose[eye]["center"][0] - 0.5)*self.screen_size[0] / ppi * MPERINCH
            t.transform.translation.z = HEAD_DIAMETER / 2

            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            self.tf_broadcaster.sendTransform(t)


    def draw_eye(self, im, eye_dict, color = (255,60,0)):
        """Color is in BGR, not RGB."""
        eye_center = self._ratio2pt(eye_dict["center"])
        eye_vector = self._ratio2pt(eye_dict["pupil"])
        pupil_center = self._ratio2pt(eye_dict["pupil"])

        if np.linalg.norm(eye_vector) > (eye_dict["radius"] - eye_dict["pupil_size"]- eye_dict["thickness"]):
            # handle pupils out of spec
            radius = eye_dict["radius"] - eye_dict["pupil_size"] - eye_dict["thickness"]
            pupil_angle = np.arctan2(eye_vector[1],eye_vector[0])
            pupil_center = np.int32([radius*np.cos(pupil_angle), radius*np.sin(pupil_angle)])
        cv2.circle(im, eye_center, eye_dict["radius"], color, eye_dict["thickness"])
        cv2.circle(im, eye_center+pupil_center, eye_dict["pupil_size"], color, -1)
        lid_pos = np.array(eye_dict["eyelid"]) + self.blink_state*(np.array(eye_dict["center"]) - np.array(eye_dict["eyelid"]))
        cv2.ellipse(im, self._ratio2pt(lid_pos), eye_dict["eyelid_size"], eye_dict["eyelid_angle"], 0, 360, (0,0,0), -1)
        

    def draw_mouth(self, im, mouth_dict, color = (255,60,0)):
        """Color is in BGR, not RGB."""
        upper_x, upper_y = zip(*[self._ratio2pt(mouth_dict[key]) for key in ["left_corner", "upper_lip", "right_corner"]])
        lower_x, lower_y = zip(*[self._ratio2pt(mouth_dict[key]) for key in ["left_corner", "lower_lip", "right_corner"]])
        
        upper_coefs = np.polyfit(upper_x, upper_y, 2)
        lower_coefs = np.polyfit(lower_x, lower_y, 2)
        i, j = np.meshgrid(*map(np.arange, im.shape[0:2]), indexing="ij")

        upper_mask = upper_coefs[2] + upper_coefs[1]*j + upper_coefs[0]*j**2 < i
        lower_mask = lower_coefs[2] + lower_coefs[1]*j + lower_coefs[0]*j**2 > i

        im[np.logical_and(upper_mask, lower_mask)] = color

    def set_image(self, color=(255, 60, 0)):
        # create img
        im = np.zeros(self.screen_size)
        self.draw_eye(im, self.face_pose["left_eye"], color=color)
        self.draw_eye(im, self.face_pose["right_eye"], color=color)
        self.draw_mouth(im, self.face_pose["mouth"], color=color)
        self.image = im

    def blink_manager(self):
        if time.time() - self.last_blink > self.blink_pause:
            blink_step = 0.1  # increment the blink by 10% each time
            self.blink_pause = 0.0 # keep going at refresh rate
            self.last_blink = time.time()
            blink_state = self.blink_state + blink_step * self.blink_direction
            if blink_state >= 1.0:
                blink_state = 1.0
                self.blink_direction *= -1
                self.blink_pause = 0.2 # pause a little when closed
            elif blink_state <= 0.0:
                blink_state = 0.0
                self.blink_direction *= -1
                self.blink_pause = min(np.random.random(),0.5)*self.get_parameter("max_blink_period").get_parameter_value().double_value
            self.blink_state = blink_state
        
def draw_face(args=None):
    rclpy.init(args=args)

    cv2.namedWindow("Face_Display", cv2.WND_PROP_FULLSCREEN)
    _, _, old_x, old_y = cv2.getWindowImageRect("Face_Display")
    cv2.setWindowProperty("Face_Display", cv2.WND_PROP_ASPECT_RATIO, cv2.WINDOW_FREERATIO)
    cv2.setWindowProperty("Face_Display", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    while True:
        _, _, x, y = cv2.getWindowImageRect("Face_Display")
        if x != old_x and y != old_y:
            break
        cv2.imshow("Face_Display", np.zeros((3,3)))
        cv2.waitKey(20)
    size = (y//2,x//2,3) # manually limit image size for performance reasons
    fullscreen = True

    exec = SingleThreadedExecutor()
    writer = FaceWriter(screen_size = size)
    exec.add_node(writer)

    while rclpy.ok():
        if fullscreen:
            cv2.setWindowProperty("Face_Display", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        else:
            cv2.setWindowProperty("Face_Display", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
        writer.set_image(color = DefaultColors.TEAL.value)
        cv2.imshow("Face_Display", writer.image)
        key = cv2.waitKey(1)
        if (key & 0xFF) == ord("q") or (key & 0xFF) == ord("f"):
            fullscreen = not fullscreen
        elif key == 27:
            break
        exec.spin_once()
    rclpy.shutdown()

if __name__ == "__main__":
    draw_face()