# ros2 launch phase_rtabmap_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
import os

def generate_launch_description():
    
    phase_camera = Node(
        package='phase_rtabmap_foxy',
        namespace='stereo',
        executable='phase_camera',
        name='phase_pub',
        output="screen",
        remappings=[
            ('/stereo/left/image_raw', '/stereo_camera/left/image_raw'),
            ('/stereo/right/image_raw', '/stereo_camera/right/image_raw'),
            ('/stereo/left/image_rect', '/stereo_camera/left/image_rect'),
            ('/stereo/right/image_rect', '/stereo_camera/right/image_rect'),
            ('/stereo/left/camera_info', '/stereo_camera/left/camera_info'),
            ('/stereo/right/camera_info', '/stereo_camera/right/camera_info'),
        ]
    )
    phase_camera_group = GroupAction(
        actions=[
            phase_camera,
        ]
    )
    tf2 = Node(
        package='tf2_ros',
        executable="static_transform_publisher",
        name="phase_trans",
        arguments = ["0", "0", "0", "-1.5707963267948966", "0", "-1.5707963267948966", "points2", "phase", "100"]
    )
    launch_stereo_image_proc_with_ns = GroupAction(
        actions=[
            PushRosNamespace('stereo_camera'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                        'stereo_image_proc.launch.py'
                ]),
                launch_arguments={
                    'approximate_sync': 'True'
                }.items()
            )
        ]
    )
    launch_rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
                'rtabmap.launch.py'
        ]),
        launch_arguments={
            'arg2': '--delete_db_on_start',
            'frame_id': 'phase',
            'rgb_topic': '/stereo_camera/left/image_rect_color',
            'depth_topic': '/stereo/depth/image',
            'camera_info_topic': '/stereo_camera/left/camera_info'
        }.items()
    )
    return LaunchDescription([
        phase_camera_group,
        tf2,
        launch_stereo_image_proc_with_ns,
        launch_rtabmap,
    ])