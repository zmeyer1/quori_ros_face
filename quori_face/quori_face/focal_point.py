# !/usr/bin/env python3
# 
# focal_point.py
# 
# Zane Meyer

import rclpy
from rclpy.node import Node
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from geometry_msgs.msg import PointStamped

class FocalPoint(Node):

    def __init__(self):
        super().__init__('focal_point')

        self.server = InteractiveMarkerServer(self, "focal_point")
        self.menu_handler = MenuHandler()
        self.pub = self.create_publisher(PointStamped, 'quori_face/focal_point', 10)

        self.create_focal_pt()


    def create_focal_pt(self):
        # Create an interactive marker
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.name = "focal_point"
        int_marker.description = "Quori Face Focal Point"

        int_marker.pose.position.x = 0.0
        int_marker.pose.position.y = 0.0
        int_marker.pose.position.z = 0.0
        int_marker.pose.orientation.x = 0.0
        int_marker.pose.orientation.y = 0.0
        int_marker.pose.orientation.z = 0.0
        int_marker.pose.orientation.w = 1.0

        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.MOVE_3D
        control.name = "move_3d"
        control.markers.append(marker)
        control.always_visible = True

        int_marker.controls.append(control)

        self.server.insert(int_marker, feedback_callback=self.update_position)
        self.menu_handler.apply(self.server, int_marker.name)
        self.server.applyChanges()


    def update_position(self, feedback):
        """Callback for handling user interaction with the marker."""
        if feedback.event_type == feedback.MOUSE_UP: # Feedback once mouse button up
            self.get_logger().info(
                f"Marker {feedback.marker_name} position: "
                f"x={feedback.pose.position.x:.2f}, "
                f"y={feedback.pose.position.y:.2f}, "
                f"z={feedback.pose.position.z:.2f}"
            )
            pt = PointStamped()
            pt.point.x = feedback.pose.position.x
            pt.point.y = feedback.pose.position.y
            pt.point.z = feedback.pose.position.z
            pt.header.frame_id = "map"
            pt.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(pt)


def main(args=None):
    rclpy.init(args=args)
    node = FocalPoint()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()