# quori_ros_face
This project is for the Quori 1.0 robot: https://quori.org/

The robot is running this source code: https://github.com/Quori-Robot/quori_ros

Which is bridged to ROS2 with this respository: https://github.com/zmeyer1/noetic_rolling_bridge_docker

This project displays a face on quori that is controlled by ROS 2, specifically versions of Jazzy and Kilted

## Non-ros PythonDependencies (install in your venv)

### pip install mediapipe
Optional, for hand tracking

### pip install opencv-python=='4.6.0.66'
 Some other versions may have a display bug -- like the default mediapipe version


## To clone the respository

### git clone https://github.com/zmeyer1/quori_ros_face.git

### colcon build && source install/setup.bash

## To run the face
### ros2 run quori_face draw_face --ros-args -p max_blink_period:=3.0

## To move the eyes
There are several different launch files to accomplish this:

### ros2 launch quori_face eye_spin.xml 
runs the face and then writes commands to spin the eyes in a circle


## To move the eyes after you've launched a quori bag file (anything with a conversion between /map and /quori/head)

### ros2 launch quori_face quori_face_draggable.py
runs the face and the gaze tracker, where the object of interest is an interactive marker that can be moved in RVIZ [which also launches]

### ros2 launch quori_face quori_face.xml
runs the face, and the gaze tracker, can be coupled with

### ros2 run quori_face watch_hand
 in order to track gaze on a hand in the scene, but only if you have Quori's camera publishing (reach out to me if you want a bag file to test this)



