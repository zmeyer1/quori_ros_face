# quori_ros_face
Display a face on quori that is controlled by ROS, specifically Jazzy and Kilted

## Non-ros PythonDependencies (in you venv)

(pip install mediapipe) Optional, for hand tracking

(pip install opencv-python=='4.6.0.66') Some other versions have a display bug -- like the default mediapipe version


## To clone the respository
(git clone https://github.com/zmeyer1/quori_ros_face.git)

(colcon build && source install/setup.bash)

## To run the face
 (ros2 run quori_face draw_face --ros-args -p max_blink_period:=3.0)

## To move the eyes
There are several different launch files to accomplish this:

(ros2 launch quori_face eye_spin.xml) runs the face and then writes commands to spin the eyes in a circle

(ros2 launch quori_face quori_face_draggable.py) runs the face and the gaze tracker, where the object of interest is an interactive marker that can be moved in RVIZ [which also launches]

(ros2 launch quori_face quori_face.xml) runs the face, and the gaze tracker, can be coupled with (ros2 run quori_face watch_hand) in order to track gaze on a hand in the scene



