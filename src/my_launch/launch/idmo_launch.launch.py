from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'run', 'py_pubsub', 'esp32rst' ],
            name='esp32_reset1'
        ),
        TimerAction(
            period= 0.5,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'py_pubsub', 'camfeed_idmo'],
                    name = 'camfeed_idmo'
                )
            ]
        ),
        TimerAction(
            period= 1.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'py_pubsub', 'esp32rst1'],
                    name = 'esp32_reset2'
                )
            ]
        ),
        TimerAction(
            period= 1.5,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'py_pubsub', 'esp32rst2'],
                    name = 'esp32_reset3'
                )
            ]
        ),
        TimerAction(
            period= 1.75,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'py_pubsub', 'esp32rst3'],
                    name = 'esp32_reset4'
                )
            ]
        ),
        TimerAction(
            period= 2.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'py_pubsub', 'esp32rst4'],
                    name = 'esp32_reset5'
                )
            ]
        ),
        TimerAction(
            period= 2.25,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev/esp32_1'],
                    name = 'micro_ros_agent_0'
                )
            ]
        ),
        TimerAction(
            period= 2.5,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev/esp32_2'],
                    name = 'micro_ros_agent_1'
                )
            ]
        ),
        TimerAction(
            period= 2.75,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev/esp32_3'],
                    name = 'micro_ros_agent_2'
                )
            ]
        ),
        TimerAction(
            period= 3.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev/esp32_4'],
                    name = 'micro_ros_agent_3'
                )
            ]
        ),
        TimerAction(
            period= 3.25,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev/esp32_5'],
                    name = 'micro_ros_agent_4'
                )
            ]
        )

    ])
