[![ROS2 (Ubuntu)](https://github.com/IMRCLab/uav-rai-demo/actions/workflows/ci-ros2.yml/badge.svg)](https://github.com/IMRCLab/uav-rai-demo/actions/workflows/ci-ros2.yml)

# uav-rai-demo
ROS2 Package to connect RAI and Crazyswarm2

## Building and Running

```
mkdir -p ros2_ws/src
cd ros2_ws/src
git clone https://github.com/IMRCLab/crazyswarm2 --recursive
git clone --branch ros2 --recursive https://github.com/IMRCLab/motion_capture_tracking.git
git clone https://github.com/IMRCLab/uav-rai-demo.git
cd ../
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DMARC_BUILD=OFF
```
Note: symlink-install allows you to edit Python and config files without running `colcon build` every time.

In a separate terminal:
```
. install/local_setup.zsh (OR . install/local_setup.bash)
ros2 launch crazyswarm2 launch.py
```

In a separate terminal:
```
. install/local_setup.zsh (OR . install/local_setup.bash)
cd src/uav-rai-demo/uav-rai-demo/config
ros2 run uav-rai-demo demo1
(OR: ../../../../install/uav-rai-demo/lib/uav-rai-demo/demo1 to debug)
```
