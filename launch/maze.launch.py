from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("maze_runner")
    world_file = os.path.join(pkg_share, "worlds", "maze_avoid.world")

    # Prefer a path inside the installed package if possible.
    # If you keep the model in the source tree, this is OK for now:
    vehicle_sdf = os.path.expanduser("~/ros2_ws/src/maze_runner/models/model.sdf")

    gazebo = ExecuteProcess(
        cmd=["gazebo", "--verbose", world_file, "-s", "libgazebo_ros_factory.so"],
        output="screen",
    )

    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-entity", "vehicle",
                    "-file", vehicle_sdf,
                    "-x", "-8.5",
                    "-y", "-8.5",
                    "-z", "0.3",
                    "-Y", "0.0",
                ],
                output="screen",
            )
        ],
    )

    wall_follower = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="maze_runner",
                executable="wall_follower",   # this must match setup.py console_scripts name
                parameters=[
                    {"scan_forward_deg": 0.0},
                    {"d_des": 1.2},
                    {"v": 0.45},
                    {"front_stop": 1.6},
                    {"front_hard": 1.1},
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription([gazebo, spawn_robot, wall_follower])
