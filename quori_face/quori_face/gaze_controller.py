# !/usr/bin/env python3

# gaze_controller.py

# Zane Meyer

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from quori_face_msgs.msg import Eye, Face
from quori_face_msgs.srv import FaceQuery, ScreenSize
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point


class GazeController(Node):
    def __init__(self):
        super().__init__("face_writer")

        self.pub = self.create_publisher(Face, 'quori_face/face_cmd', 10)
        self.timer = self.create_timer(1, self.poll_face)


        self.create_subscription(PointStamped, 'quori_face/focal_point', self.update_face, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.declare_parameter("max_blink_period", 10.0)
        self.declare_parameter("PPI", 100.0) # Roughly an average pixels per inch default for Quori's screen
        self.face_query = self.create_client(FaceQuery, 'quori_face/query_face')
        self.screen_query = self.create_client(ScreenSize, 'quori_face/get_screensize')
        while not self.face_query.wait_for_service(timeout_sec=1.0) and not self.screen_query.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('quori_face services not available, waiting...')

        self.face_pose = None
        self.screen_size = None

        future = self.screen_query.call_async(ScreenSize.Request())
        future.add_done_callback(self.set_screen_size)


    def poll_face(self):
        future = self.face_query.call_async(FaceQuery.Request())
        future.add_done_callback(self.set_face_pose)

    def set_face_pose(self, future):
        if future.result() is None:
            self.get_logger().error('Failed to get face data')
            return
        self.face_pose = future.result().face
    
    def set_screen_size(self, future):
        if future.result() is None:
            self.get_logger().error('Failed to get screen size')
            return
        self.screen_size = future.result().screensize

    def update_face(self, msg):
        """Takes in a Point in 3D space and moves the eyes to look at that point"""

        self.get_logger().info(f"Received focal point: {msg.point.x}, {msg.point.y}, {msg.point.z}")

        if self.face_pose is None or self.screen_size is None:
            self.get_logger().warn("Face pose or screen size not set yet, skipping gaze update.")
            return

        face_msg = Face()

        for i,eye in enumerate(["left_eye", "right_eye"]):
            eye_size = self.face_pose.eyes[i].radius
            eye_size = eye_size / self.get_parameter("PPI").value * 0.0254  # Convert from pixels to meters
            transform = self.tf_buffer.lookup_transform(f"quori/head_{eye}",msg.header.frame_id, rclpy.time.Time())
            target_pt = do_transform_point(msg, transform)

            # We want to find the coordinates of the point on the face that passes through a line from the target point
            # to a point behind the eye, at a distance equal to the eye radius.
            t = eye_size / (target_pt.point.x + eye_size)
            y_face = (target_pt.point.y) * t
            z_face = (target_pt.point.z) * t

            # now convert the point back to a fraction of the screen size y->x, z->y
            x_coord = y_face * self.get_parameter("PPI").value / 0.0254
            y_coord = -z_face * self.get_parameter("PPI").value / 0.0254  # negative because the screen is flipped in the y direction
            x_coord /= self.screen_size[0]
            y_coord /= self.screen_size[1]
            # Publish the eye position
            eye_msg = Eye()
            eye_msg.pupil = [x_coord, y_coord]
            face_msg.eyes.append(eye_msg)

        self.get_logger().info(f"Published eye positions: {face_msg.eyes[0].pupil}, {face_msg.eyes[1].pupil}")

        self.pub.publish(face_msg)

        
def main(args=None):
    rclpy.init(args=args)    
    executor = SingleThreadedExecutor()
    
    executor.add_node(GazeController())

    executor.spin()
    rclpy.shutdown()

if __name__ == "__main__":
    main()