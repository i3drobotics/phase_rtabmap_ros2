# phase_rtabmap_foxy
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
git clone --branch foxy https://github.com/ros-perception/image_pipeline.git src/image_pipeline
git clone --branch foxy https://github.com/ros-perception/image_common.git src/image_common
git clone -- branch foxy-devel https://github.com/introlab/rtabmap.git src/rtabmap
git clone --branch ros2 https://github.com/introlab/rtabmap_ros.git src/rtabmap_ros
git clone https://github.com/i3drobotics/phase_rtabmap_foxy.git src/phase_rtabmap_foxy
sudo apt-get update
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
. install/setup.bash
```

## Run RTabmap Scan
```bash
cd ~/dev_ws/install/phase_rtabmap_foxy/share/phase_rtabmap_foxy
ros2 launch phase_rtabmap_launch.py
```
Optional launch arguments when use different camera or exposure
```bash
cd ~/dev_ws/install/phase_rtabmap_foxy/share/phase_rtabmap_foxy
ros2 launch phase_rtabmap_launch.py left_serial:=23638717 right_serial:=23638711 camera_name:=Basler acA2440-35uc camera_type:=phobos exposure:=25000
```
###
