# phase_rtabmap_foxy
Demonstration of mapping with Phase camera in ROS2 foxy.

Install rtabmap_ros from the link below
https://github.com/introlab/rtabmap_ros
```bash
sudo apt install ros-noetic-rtabmap-ros
```

## Build ROS2 workspace
```bash
mkdir -p ~/dev_ws/src
cd ~/dev_ws
source /opt/ros/${ROS_DISTRO}/setup.bash
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
cd src/phase_rtabmap_foxy_launch
ros2 launch phase_rtabmap_launch.py
```
###
