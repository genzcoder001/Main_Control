import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('rado_control_3')
    
    # Load twist_mux parameters
    twist_mux_params = os.path.join(pkg_share, 'config/twist_mux.yaml')
    
    return LaunchDescription([
        # # Start twist_mux for control switching
        # Node(
        #     package='twist_mux',
        #     executable='twist_mux',
        #     parameters=[twist_mux_params],
        #     remappings=[('/cmd_vel_out', '/cmd_vel')]
        # ),
        
        # Start nodes for manual joystick control
        Node(package='joy', executable='joy_node', name='joy_node'),
        Node(
            package='teleop_twist_joy', 
            executable='teleop_node', 
            name='teleop_node',
            parameters=[{'enable_button': 0}], # Example: Use button 0 as enable
            remappings=[('/cmd_vel', '/manual/cmd_vel')]
        ),
        
        # Start your custom nodes
         Node(
            package='rado_control_3',
            executable='state_manager_node.py', # Use the script name
            name='state_manager'
        ),
        Node(
            package='rado_control_3',
            executable='coordinate_follower_node.py', # Use the script name
            name='coordinate_follower'
        ),
        
    ])
