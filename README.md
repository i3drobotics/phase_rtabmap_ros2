# phase_rtabmap_foxy

cd/dev_ws/src
git clone https://github.com/i3drobotics/phase_rtabmap_foxy.git
git clone --branch foxy https://github.com/ros-perception/image_pipeline.git
git clone --branch foxy https://github.com/ros-perception/image_common.git
git clone https://github.com/introlab/rtabmap.git src/rtabmap
git clone --branch ros2 https://github.com/introlab/rtabmap_ros.git src/rtabmap_ros
cd ~/dev_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
. install/setup.bash
cd src/phase_rtabmap_foxy_launch
ros2 launch phase_rtabmap_launch.py