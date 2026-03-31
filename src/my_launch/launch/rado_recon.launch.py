from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    return LaunchDescription([

    ExecuteProcess(
        cmd=['ros2', 'run', 'py_pubsub', 'camfeed_recon'],
        name='camfeed_recon',
        output='screen'
        )
    ])
