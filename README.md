# phase_rtabmap_ros2
Demonstration of mapping with Phase camera in ROS2 foxy.
The package is currently runs on Linux.

## Install pyphase
Install pyphase, follow Linux instruction from the link below

https://github.com/i3drobotics/pyphase

## Build ROS2 workspace
```bash
mkdir -p ~/dev_ws/src
cd ~/dev_ws
source /opt/ros/foxy/setup.bash
git clone --branch humble https://github.com/ros-perception/image_pipeline.git src/image_pipeline
git clone --branch humble https://github.com/ros-perception/image_common.git src/image_common
git clone --branch humble-devel https://github.com/introlab/rtabmap.git src/rtabmap
git clone --branch humble-devel https://github.com/introlab/rtabmap_ros.git src/rtabmap_ros
git clone https://github.com/i3drobotics/phase_rtabmap_ros2.git src/phase_rtabmap_ros2
sudo apt-get update
rosdep update && rosdep install --from-paths src --ignore-src -r -y
export MAKEFLAGS="-j6" # Can be ignored if you have a lot of RAM (>16GB)
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
. install/setup.bash
```

## Run RTabmap Scan
```bash
ros2 launch phase_rtabmap_ros2 phase_rtabmap_launch.py 
```
Optional launch arguments when use different camera or exposure value
```bash
ros2 launch phase_rtabmap_ros2 phase_rtabmap_launch.py left_serial:=23638717 right_serial:=23638711 camera_name:=Basler acA2440-35uc device_type:=phobos interface_type:=usb exposure:=25000
```

## Save video of the camera
ros2 run phase_rtabmap_ros2 phase_camera_record_launch.py  left_serial:=23638717 right_serial:=23638711 camera_name:=Basler acA2440-35uc device_type:=phobos interface_type:=usb exposure:=25000

## Update calibration yaml file
Calibration yaml files are needed to store in phase_rtabmap_ros2/cal folder before build
Need to check or rename if yaml files are in the name of "left.yaml" and "right.yaml"

## Troubleshoot for GigE network configuration
For Network Configuration, see the link below for troubleshooting

https://docs.baslerweb.com/network-configuration-(gige-cameras)
