from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    return LaunchDescription([

        ExecuteProcess(
            cmd=['ros2', 'run', 'py_pubsub', 'camfeed_auto'],
            name='camfeed_auto',
            output='screen'
        ),

        TimerAction(
            period=1.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'launch', 'openvino_cone_detection', 'openvino_cone_detection.launch.py'],
                    name='cone_launch_file',
                    output='screen'
                )
            ]
        ),
    ])
