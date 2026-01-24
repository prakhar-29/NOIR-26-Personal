from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    return LaunchDescription([

        ExecuteProcess(
            cmd=['ros2', 'run', 'py_pubsub', 'esp32rst'],
            name='esp32_reset1',
            output='screen'
        ),

        TimerAction(
            period=0.5,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'py_pubsub', 'esp32rst1'],
                    name='esp32_reset2',
                    output='screen'
                )
            ]
        ),
        TimerAction(
            period=0.75,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'py_pubsub', 'esp32rst2'],
                    name='esp32_reset3',
                    output='screen'
                )
            ]
        ),

        TimerAction(
            period=1.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'py_pubsub', 'camfinal'],
                    name='camfeed_final',
                    output='screen'
                )
            ]
        ),

        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'launch',
                        'ldlidar_node', 'ldlidar_rviz2.launch.py'
                    ],
                    name='ldlidar_launch',
                    output='screen'
                )
            ]
        ),

        TimerAction(
            period=2.5,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run',
                        'nmea_navsat_driver', 'nmea_serial_driver',
                        '--ros-args',
                        '-p', 'port:=/dev/gps',
                        '-p', 'baud:=9600',
                        '-p', 'use_gnss:=true'
                    ],
                    name='gps_node',
                    output='screen'
                )
            ]
        ),

        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run',
                        'micro_ros_agent', 'micro_ros_agent',
                        'serial', '--dev', '/dev/esp32_1'
                    ],
                    name='micro_ros_agent_0',
                    #output='screen'
                )
            ]
        ),

        TimerAction(
            period=3.5,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run',
                        'micro_ros_agent', 'micro_ros_agent',
                        'serial', '--dev', '/dev/esp32_2'
                    ],
                    name='micro_ros_agent_1',
                    #output='screen'
                )
            ]
        ),
        TimerAction(
            period=4.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run',
                        'micro_ros_agent', 'micro_ros_agent',
                        'serial', '--dev', '/dev/esp32_3'
                    ],
                    name='micro_ros_agent_2',
                    #output='screen'
                )
            ]
        )

    ])
