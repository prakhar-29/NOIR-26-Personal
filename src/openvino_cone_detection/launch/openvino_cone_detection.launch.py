#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('openvino_cone_detection')
    
    # Declare launch arguments
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=os.path.join(pkg_dir, 'models', 'yolov8s_cone_fp32.xml'),
        description='Path to OpenVINO model XML file'
    )
    
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='CPU',
        description='OpenVINO device: CPU, GPU, or AUTO'
    )
    
    # CHANGED: Now supports both numeric indices and full device paths
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/v4l/by-path/pci-0000:00:14.0-usb-0:3:1.0-video-index0',
        description='Camera device path (e.g., /dev/video2 or /dev/v4l/by-path/...)'
    )
    
    conf_threshold_arg = DeclareLaunchArgument(
        'conf_threshold',
        default_value='0.018',
        description='Confidence threshold for detection'
    )
    
    iou_threshold_arg = DeclareLaunchArgument(
        'iou_threshold',
        default_value='0.45',
        description='IOU threshold for NMS'
    )
    
    min_box_area_arg = DeclareLaunchArgument(
        'min_box_area',
        default_value='500',
        description='Minimum bounding box area in pixels'
    )
    
    max_box_area_arg = DeclareLaunchArgument(
        'max_box_area',
        default_value='200000',
        description='Maximum bounding box area in pixels'
    )
    
    target_color_arg = DeclareLaunchArgument(
        'target_color',
        default_value='all',
        description='Target cone color: yellow, blue, orange, large_orange, red, green, others, or all'
    )
    
    frame_width_arg = DeclareLaunchArgument(
        'frame_width',
        default_value='640',
        description='Camera frame width'
    )
    
    frame_height_arg = DeclareLaunchArgument(
        'frame_height',
        default_value='640',
        description='Camera frame height'
    )
    
    use_stability_filter_arg = DeclareLaunchArgument(
        'use_stability_filter',
        default_value='true',
        description='Enable temporal stability filtering'
    )
    
    history_size_arg = DeclareLaunchArgument(
        'history_size',
        default_value='5',
        description='Number of frames for stability tracking'
    )
    
    min_detection_frames_arg = DeclareLaunchArgument(
        'min_detection_frames',
        default_value='3',
        description='Minimum frames a detection must appear to be valid'
    )
    
    show_visualization_arg = DeclareLaunchArgument(
        'show_visualization',
        default_value='false',
        description='Show OpenCV visualization window'
    )
    
    use_async_inference_arg = DeclareLaunchArgument(
        'use_async_inference',
        default_value='false',
        description='Use asynchronous inference for better performance'
    )
    
    # Node configuration
    openvino_cone_detection_node = Node(
        package='openvino_cone_detection',
        executable='openvino_cone_detection_node',
        name='openvino_cone_detection_node',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'device': LaunchConfiguration('device'),
            'camera_device': LaunchConfiguration('camera_device'),  # CHANGED
            'conf_threshold': LaunchConfiguration('conf_threshold'),
            'iou_threshold': LaunchConfiguration('iou_threshold'),
            'min_box_area': LaunchConfiguration('min_box_area'),
            'max_box_area': LaunchConfiguration('max_box_area'),
            'target_color': LaunchConfiguration('target_color'),
            'frame_width': LaunchConfiguration('frame_width'),
            'frame_height': LaunchConfiguration('frame_height'),
            'use_stability_filter': LaunchConfiguration('use_stability_filter'),
            'history_size': LaunchConfiguration('history_size'),
            'min_detection_frames': LaunchConfiguration('min_detection_frames'),
            'show_visualization': LaunchConfiguration('show_visualization'),
            'use_async_inference': LaunchConfiguration('use_async_inference'),
        }],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        model_path_arg,
        device_arg,
        camera_device_arg,  # CHANGED
        conf_threshold_arg,
        iou_threshold_arg,
        min_box_area_arg,
        max_box_area_arg,
        target_color_arg,
        frame_width_arg,
        frame_height_arg,
        use_stability_filter_arg,
        history_size_arg,
        min_detection_frames_arg,
        show_visualization_arg,
        use_async_inference_arg,
        openvino_cone_detection_node,
    ])
