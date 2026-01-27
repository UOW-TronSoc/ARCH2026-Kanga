from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Set up paths to the necessary configuration files and packages
    robot_description_path = get_package_share_directory("kanga_arm_description")
    simulation_path = get_package_share_directory("kanga_arm_simulation")

    # Load config for leg configuration
    leg_config = os.path.join(
        robot_description_path,
        'config',
        'leg_config.yaml'
        )
    
    # Load config for simulation parameters
    simulation_params = os.path.join(
        simulation_path,
        'config',
        'simulation.yaml'
        )

    # Declare launch arguments for the configurations
    return LaunchDescription([
        # Setup raisim node
        Node(
            package='kanga_arm_simulation',
            executable='raisim_bridge',
            name='raisim_bridge',
            output='screen',
            parameters=[
                {
                "robot_description_path": robot_description_path,
                }, 
                leg_config,
                simulation_params
            ]
        )
    ])
