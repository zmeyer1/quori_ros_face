import launch, os
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
  cascade = [

    launch_ros.actions.Node(
      package='quori_face',
      executable='draw_face',
      name='draw_face_node',
      parameters=[{
        'max_blink_period': 12.0,
        'PPI': 100.0
      }]
    ),
    launch_ros.actions.Node(
      package='quori_face',
      executable='gaze_controller',
      name='gaze_controller_node',
      parameters=[{
        'PPI': 100.0
      }]
    ),
    launch_ros.actions.Node(
      package='quori_face',
      executable='focal_point',
      name='focal_point_node'
    ),
    launch_ros.actions.Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      arguments=['-d', os.path.join(get_package_share_directory('quori_face'), 'config', 'config.rviz')]
    ),

  ]
  return launch.LaunchDescription(cascade)