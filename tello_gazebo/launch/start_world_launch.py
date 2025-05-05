import os, xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node



def generate_launch_description():
    simu_pkg = get_package_share_directory("tello_gazebo")
    xacro_file = os.path.join(get_package_share_directory("tello_desc"), "xacro", "tello.xacro")

    os.environ["GAZEBO_MODEL_PATH"] += os.path.join(simu_pkg, "models")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        )
    )

    spawn_robot_cmd = Node(
        package = "gazebo_ros",
        executable = "spawn_entity.py",
        arguments = [
            "-entity", "drone1", 
            "-topic", "/drone1/robot_description",
            "-x", "0.0", 
            "-y", "0.0",
            "-Y", "0.0",
            "-robot_namespace", "drone1"
        ],
    )

    robot_state_pub_cmd = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        namespace = "drone1",
        parameters = [
            {"robot_description": xacro.process_file(xacro_file, mappings={"suffix": "1", "cam_needed": "false"}).toxml()},
            {"frame_prefix" : "drone1/"}
        ]
    )


    #*************************************************************    
    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value = os.path.join(simu_pkg, "worlds", "env_1.world"),
            description = "World file to use for the simulation"
        ),

        DeclareLaunchArgument(
            "verbose",
            default_value = "true",
            description = "Increase messages written to terminal"
        ),

        gazebo_launch,

        spawn_robot_cmd,

        robot_state_pub_cmd
    ])