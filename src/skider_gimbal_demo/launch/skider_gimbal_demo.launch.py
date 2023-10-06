import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    return LaunchDescription(
        [
            # Node(
            #     package='skider_gimbal_demo',
            #     executable='gimbal_demo_node',
            #     name='gimbal_demo_node',
            #     output='screen',
            #     # parameters=[os.path.join(get_package_share_directory('skider_gimbal_demo'), 'config', 'params.yaml')],
            #     parameters=["/home/oem/Desktop/ros-control/skider_03/src/skider_gimbal_demo/config/params.yaml"],
                
            # ),
            Node(
                package='skider_gimbal_demo',
                executable='chassis_demo_node',
                name='chassis_demo_node',
                output='screen',
                # parameters=[os.path.join(get_package_share_directory('skider_chassis_demo'), 'config', 'params.yaml')],
                parameters=["/home/oem/Desktop/ros-control/skider_03/src/skider_gimbal_demo/config/params.yaml"],
                
            )
        ]
    )