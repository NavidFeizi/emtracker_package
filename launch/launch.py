from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    ld = LaunchDescription()

    tracker_node = ExecuteProcess(
        cmd=[
            'taskset', '-c', '2',  # This sets the affinity to CPU core 0
            # 'gnome-terminal', '--',
            'ros2', 'run', 'emtracker', 'track',
            '--ros-args',
            # '-p', 'cutoff_freq:=30.6',
            '-p', 'cutoff_freq:=14.0',
            '--remap', '__node:=emtracker_node'
        ],
        output='screen',
    )

    plotjuggler_node = ExecuteProcess(
        cmd=[
            # 'taskset', '-c', '2',  # This sets the affinity to CPU core 0
            # 'gnome-terminal', '--',
            'ros2', 'run', 'plotjuggler', 'plotjuggler',
            '--ros-args',
            '--remap', '__node:=plotjuggler_node'
        ],
        # output='screen',
    )


    # ld.add_action(plotjuggler_node)
    ld.add_action(tracker_node)
    return ld
