import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    scheduler = Node(
        package="docking_scheduler",
        executable="scheduler",
        name="scheduler",
        
    )

    ld.add_action(scheduler)

    return ld