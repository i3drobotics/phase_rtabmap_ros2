# cd ~/dev_ws/install/phase_rtabmap_foxy/share/phase_rtabmap_foxy/
# ros2 launch phase_rtabmap_foxy phase_camera_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution

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
        package='phase_rtabmap_foxy',
        executable='phase_camera',
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
    return LaunchDescription([
        left_serial_launch_arg,
        right_serial_launch_arg,
        camera_name_launch_arg,
        device_type_launch_arg,
        interface_type_launch_arg,
        exposure_launch_arg,
        phase_camera,
    ])