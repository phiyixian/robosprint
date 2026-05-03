from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(package='vision_pkg', executable='camera_node'),
        Node(package='vision_pkg', executable='cube_detector'),
        Node(package='vision_pkg', executable='junction_detector'),

        Node(package='decision_pkg', executable='line_fsm_controller'),

        Node(package='navigation_pkg', executable='line_follower'),

        Node(package='control_pkg', executable='motor_driver'),
        Node(package='control_pkg', executable='manipulator'),

        Node(package='game_state_pkg', executable='game_state'),

    ])
