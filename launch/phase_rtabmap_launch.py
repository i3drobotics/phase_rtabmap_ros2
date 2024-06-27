#!/usr/bin/env python3
# cd ~/dev_ws
# colcon build --packages-select phase_rtabmap_ros2
# . install/setup.bash
# ros2 launch phase_rtabmap_ros2 phase_rtabmap_launch.py
# ros2 launch phase_rtabmap_ros2 phase_rtabmap_launch.py left_serial:=40266661 right_serial:=40298125 camera_name:=746974616e24324 device_type:=titania interface_type:=usb exposure:=10000

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # args that can be set from the command line or a default will be used
    left_serial_launch_arg = DeclareLaunchArgument(
        "left_serial", default_value=TextSubstitution(text="23638717")
    )
    right_serial_launch_arg = DeclareLaunchArgument(
        "right_serial", default_value=TextSubstitution(text="23638711")
    )
    camera_name_launch_arg = DeclareLaunchArgument(
        "camera_name", default_value=TextSubstitution(text="Basler acA2440-35uc")
    )
    device_type_launch_arg = DeclareLaunchArgument(
        "device_type", default_value=TextSubstitution(text="phobos")
    )
    interface_type_launch_arg = DeclareLaunchArgument(
        "interface_type", default_value=TextSubstitution(text="usb")
    )
    exposure_launch_arg = DeclareLaunchArgument(
        "exposure", default_value=TextSubstitution(text="25000")
    )

    left_serial_arg = LaunchConfiguration("left_serial")
    right_serial_arg = LaunchConfiguration("right_serial")
    camera_name_arg = LaunchConfiguration("camera_name")
    device_type_arg = LaunchConfiguration("device_type")
    interface_type_arg = LaunchConfiguration("interface_type")
    exposure_arg = LaunchConfiguration("exposure")

    
    phase_camera = Node(
        package='phase_rtabmap_ros2',
        executable='phase_camera',
        name='phase_pub',
        output="screen",
        arguments=[
            "--left_serial", left_serial_arg,
            "--right_serial", right_serial_arg,
            "--camera_name", camera_name_arg,
            "--device_type", device_type_arg,
            "--interface_type", interface_type_arg, 
            "--exposure", exposure_arg
            ],
    )

    # ros2 run tf2_ros static_transform_publisher "0 0 0 1.5707963267948966 0 -1.5707963267948966 base_link camera_link
    tf2 = Node(
        package='tf2_ros',
        executable="static_transform_publisher",
        name="camera_base_link",
        arguments = ["0", "0", "0", "-0.785", "0", "-1.5707963267948966", "base_link", "camera_link"]
    )

        # ros2 run imu_publish_ros2 imu_publisher
    imu_publish = Node(
        package='imu_publish_ros2',
        executable='imu_publisher',
        output="screen",
    )

    # ros2 run imu_filter_madgwick imu_filter_madgwick_node 
    # --ros-args -p fixed_frame:='base_link'
    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        output="screen",
        parameters=[
            {"use_mag": False},
            {"_publish_tf": True},
            {"fixed_frame": 'base_link'},
            # {"reverse_tf": True}
            # {"_world_frame": 'base_link'}
        ],
    )

    # ros2 launch rtabmap_ros rtabmap.launch.py args:=--delete_db_on_start frame_id:=base_link rgb_topic:=/left/image_rect_color depth_topic:=/depth/image camera_info_topic:=/left/camera_info
    launch_rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rtabmap_launch'),
                'launch/rtabmap.launch.py'
            ])
        ]),
        launch_arguments={
            'args': '--delete_db_on_start',
            'frame_id': 'base_link',
            'rgb_topic': '/left/image_rect',
            'depth_topic': '/depth/image',
            'camera_info_topic': '/left/camera_info',
            # 'subscribe_rgbd': 'false',
            # 'wait_imu_to_init': 'true',
            # 'imu_topic': '/imu/data'
        }.items()
    )
    return LaunchDescription([
        left_serial_launch_arg,
        right_serial_launch_arg,
        camera_name_launch_arg,
        device_type_launch_arg,
        interface_type_launch_arg,
        exposure_launch_arg,
        phase_camera,
        tf2,
        imu_publish,
        imu_filter,
        launch_rtabmap,
    ])