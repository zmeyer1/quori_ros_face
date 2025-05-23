# quori_ros_face
Display a face on quori that is controlled by ros

## To clone the respository
git clone https://github.com/zmeyer1/quori_ros_face.git

colcon build && source install/setup.bash

## To run the face
 ros2 run quori_face draw_face --ros-args -p blink_period:=2.0

## To move the eyes
Just manually move them for now. Theres a default JSON file you can edit, or you can publish a Face message to quori_face/face_cmd

## To create the focal point

 ros2 run quori_face focal_point
