ros2 launch mecanum_base ir_sensor_broadcaster.launch.py
clear
rm -rf build/mecanum_base install/mecanum_base log
ls install/mecanum_base/share/mecanum_base/ir_sensor_broadcaster.xml
ls install/mecanum_base/lib/libir_sensor_broadcaster.so
source install/setup.bash 
ros2 run controller_manager ros2_control_node --ros-args --params-file src/mecanum_base/config/ros2_control.yaml
colcon build --packages-select mecanum_base --symlink-install
source install/setup.bash 
ros2 launch mecanum_base launch.py
ros2 launch mecanum_base bringup.launch.py 
clear
colcon build --packages-select mecanum_base --symlink-install
source install/setup.bash 
ros2 launch mecanum_base bringup.launch.py 
clear
colcon build --packages-select mecanum_base --symlink-install
source install/setup.bash 
ros2 launch mecanum_base bringup.launch.py 
colcon build --packages-select mecanum_base --symlink-install
colcon build --packages-select mecanum_base --symlink-install
clear
colcon build --packages-select mecanum_base --symlink-install
clear
colcon build --packages-select mecanum_base --symlink-install
source install/setup.bash 
ros2 launch mecanum_base bringup.launch.py 
clear
colcon build --packages-select mecanum_base --symlink-install
source install/setup.bash 
ros2 launch mecanum_base bringup.launch.py 
clear
colcon build --packages-select mecanum_base --symlink-install
source install/setup.bash 
ros2 launch mecanum_base bringup.launch.py 
source install/setup.bash 
ros2 control list_controllers
ros2 topic list
ros2 topic echo /ir_front_left_broadcaster/range 
source install/setup.bash 
ros2 control list_controllers
ros2 topic list
source install/setup.bash 
ros2 control list_controllers
ros2 topic pub /servo_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5, -0.5]"
ros2 topic pub /servo_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.333, -0.333]"
ros2 topic pub /servo_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.111, -0.1111]"
sudo apt update
sudo apt install ros-jazzy-rosbridge-server
clear
colcon build --packages-select mecanum_base --symlink-install
source install/setup.bash 
ros2 launch mecanum_base bringup.launch.py 
clear
ros2 launch mecanum_base bringup.launch.py 
sudo apt install nodejs npm
cd src/mecanum_base/
ls
cd webserver/
ros2 launch mecanum_base bringup.launch.py 
clear
ros2 launch mecanum_base bringup.launch.py 
ros2 launch mecanum_base bringup.launch.py 
clear
cd
source install/setup.bash 
ls /home/ws/install/mecanum_base/share/mecanum_base/launch/../webserver
ls /home/ws/install/mecanum_base/share/mecanum_base/launch/..
ls /home/ws/install/mecanum_base/share/mecanum_base/
pwd
ls
cd src
cd mecanum_base/
ls
clear
colcon build --packages-select mecanum_base --symlink-install
source install/setup.bash 
ls install/mecanum_base/share/mecanum_base/webserver/
ls install/mecanum_base/share/mecanum_base/webserver/
ls install/mecanum_base/share/mecanum_base/webserver/
clear
ros2 launch mecanum_base bringup.launch.py 
clear
ros2 launch mecanum_base bringup.launch.py 
cd
ls
rm -rf build/ install/ log/
cd src/mecanum_base/
ls
rm -rf build/ install/ log/
ls
cd
ps aux | grep rosbridge_websocket
netstat -tuln | grep 9090
ss -tuln | grep 9090
websocat ws://localhost:9090
ps aux | grep node
ps aux | grep node | grep js
ps aux | grep node | grep serve
lsof -i :3000
netstat -an | grep 3000
ls /home/ws/install/mecanum_base/share/mecanum_base/launch/../webserver
ls /home/ws/install/mecanum_base/share/mecanum_base/src
ls /home/ws/install/mecanum_base/share/mecanum_base/launch/../../webserver
pwd
ls /home/ws/install/mecanum_base/share/mecanum_base/launch/
ls /home/ws/install/mecanum_base/share/mecanum_base/launch/..
cd mecanum_base/webserver
ls
cd src
cd mecanum_base/webserver
node server.js
npm init -y
npm install express
node server.js
history| grep  fetch
git config pull.rebase false
pwd
ls
colcon build --packages-select mecanum_base --symlink-install
sudo apt install ros-jazzy-battery-state-broadcaster
colcon build --packages-select mecanum_base --symlink-install
ros2 pkg list | grep battery_state_broadcaster
ros2 control list_controller_types | grep BatteryStateBroadcaster
ros2 pkg prefix battery_state_broadcaster
more /opt/ros/jazzy/include/battery_state_broadcaster/battery_state_broadcaster/BatteryStateBroadcaster.hpp 
more /opt/ros/jazzy/include/battery_state_broadcaster/battery_state_broadcaster/BatterySensor.hpp 
cLEAr
clear
more /opt/ros/jazzy/include/battery_state_broadcaster/battery_state_broadcaster/BatterySensor.hpp 
ros2 pkg list | grep battery_state_broadcaster
ros2 pkg prefix battery_state_broadcaster
clear
colcon build --packages-select mecanum_base --symlink-install
clear
colcon build --packages-select mecanum_base --symlink-install
history
clear
ros2 launch mecanum_base bringup.launch.py 
source install/setup.bash 
ros2 launch mecanum_base bringup.launch.py 
clear
ros2 launch mecanum_base bringup.launch.py 
ps -aux
clear
source install/setup.bash 
ros2 launch mecanum_base bringup.launch.py 
source install/setup.bash 
ros2 launch mecanum_base bringup.launch.py 
source install/setup.bash 
ros2 launch mecanum_base bringup.launch.py 
sudo lsof -i :8000
ros2 launch mecanum_base bringup.launch.py 
ros2 launch mecanum_base bringup.launch.py 
clear
colcon build --packages-select mecanum_base --symlink-install
source install/setup.bash 
ros2 launch mecanum_base bringup.launch.py 
colcon build --packages-select mecanum_base
source install/setup.bash 
ros2 launch mecanum_base bringup.launch.py 
hostory|grep rm
history|grep rm
rm -rf build/ install/ log/
colcon build --packages-select mecanum_base --symlink-install
source install/setup.bash 
ros2 launch mecanum_base bringup.launch.py 
sudo lsof -i :8000
sudo lsof -i :8000
sudo apt update
sudo apt install lsof
sudo lsof -i :8000
sudo kill -9 9960
sudo lsof -i :8000
sudo lsof -i :8000
sudo lsof -i :8000
sudo lsof -i :8000
sudo lsof -i :8000
sudo lsof -i :9090
sudo lsof -i :9090
sudo lsof -i :9090
clear
sudo lsof -i :9090
clear
sudo lsof -i :9090 |more
ps aux | grep rosbridge
sudo lsof -i :9090
sudo lsof -i :9091
sudo lsof -i :9090
ps -p 3642 -o args
colcon build --packages-select mecanum_base --symlink-install
source install/setup.bash 
ros2 launch mecanum_base bringup.launch.py 
ros2 launch mecanum_base bringup.launch.py 
colcon build --packages-select mecanum_base --symlink-install
source install/setup.bash 
clear
ros2 launch mecanum_base bringup.launch.py 
clear
colcon build --packages-select mecanum_base --symlink-install
source install/setup.bash 
ros2 launch mecanum_base bringup.launch.py 
clear
colcon build --packages-select mecanum_base --symlink-install
source install/setup.bash 
ros2 launch mecanum_base bringup.launch.py 
colcon build --packages-select mecanum_base --symlink-install
source install/setup.bash 
ros2 launch mecanum_base bringup.launch.py 
ros2 launch mecanum_base bringup.launch.py 
colcon build --packages-select mecanum_base --symlink-install
colcon build --packages-select mecanum_base --symlink-install
source install/setup.bash 
ros2 launch mecanum_base bringup.launch.py 
colcon build --packages-select mecanum_base --symlink-install
colcon build --packages-select mecanum_base --symlink-install
source install/setup.bash 
ros2 launch mecanum_base bringup.launch.py 
ros2 launch mecanum_base bringup.launch.py 
ros2 launch mecanum_base bringup.launch.py 
sudo apt install ros-jazzy-topic-tools
source install/setup.
source install/setup.bash 
ros2 run topic_tools throttle messages /input_topic 5.0 /throttled_topic
ros2 run topic_tools throttle messages /rosout 5.0 /throttled_topic
ros2 run topic_tools throttle 
ros2 launch mecanum_base bringup.launch.py 
ros2 launch mecanum_base bringup.launch.py 
colcon build --packages-select mecanum_base --symlink-install
source install/setup.bash  
ros2 launch mecanum_base bringup.launch.py 
colcon build --packages-select mecanum_base --symlink-install
source install/setup.bash  
ros2 launch mecanum_base bringup.launch.py 
ros2 launch mecanum_base bringup.launch.py 
colcon build --packages-select mecanum_base --symlink-install
source install/setup.bash  
ros2 launch mecanum_base bringup.launch.py 
clear
ros2 launch mecanum_base bringup.launch.py 
colcon build --packages-select mecanum_base --symlink-install
source install/setup.bash  
ros2 launch mecanum_base bringup.launch.py 
clear
colcon build --packages-select mecanum_base --symlink-install
source install/setup.bash  
ros2 launch mecanum_base bringup.launch.py 
clear
colcon build --packages-select mecanum_base --symlink-install
source install/setup.bash  
ros2 launch mecanum_base bringup.launch.py 
clear
ros2 launch mecanum_base bringup.launch.py 
ros2 launch mecanum_base bringup.launch.py 
colcon build --packages-select mecanum_base --symlink-install
source install/setup.bash  
ros2 launch mecanum_base bringup.launch.py 
colcon build --packages-select mecanum_base --symlink-install
source install/setup.bash  
ros2 launch mecanum_base bringup.launch.py 
colcon build --packages-select mecanum_base --symlink-install
colcon build --packages-select mecanum_base --symlink-install
source install/setup.bash  
ros2 launch mecanum_base bringup.launch.py 
colcon build --packages-select mecanum_base --symlink-install
source install/setup.bash  
ros2 launch mecanum_base bringup.launch.py 
ros2 launch mecanum_base bringup.launch.py 
colcon build --packages-select mecanum_base --symlink-install
colcon build --packages-select mecanum_base --symlink-install
source install/setup.bash  
ros2 launch mecanum_base bringup.launch.py 
ros2 launch mecanum_base bringup.launch.py 
colcon build --packages-select mecanum_base --symlink-install
source install/setup.bash  
ros2 launch mecanum_base bringup.launch.py 
colcon build --packages-select mecanum_base --symlink-install
source install/setup.bash  
ros2 launch mecanum_base bringup.launch.py 
ros2 launch mecanum_base bringup.launch.py 
colcon build --packages-select mecanum_base --symlink-install
source install/setup.bash  
ros2 launch mecanum_base bringup.launch.py 
ros2 run image_tools cam2image
[200~ros2 run image_transport republish raw compressed
~ros2 run image_transport republish raw compressed
ros2 run image_transport republish raw compressed
clear
colcon build --packages-select mecanum_base --symlink-install
source install/setup.bash  
ros2 launch mecanum_base bringup.launch.py 
clear
ros2 launch mecanum_base bringup.launch.py >> log.txt
colcon build --packages-select mecanum_base --symlink-install
colcon build --packages-select mecanum_base --symlink-install
source install/setup.bash  
ros2 launch mecanum_base bringup.launch.py > log.txt
sudo apt install ros-jazzy-compressed-image-transport
ros2 launch mecanum_base bringup.launch.py > log.txt
colcon build --packages-select mecanum_base --symlink-install
source install/setup.bash  
ros2 launch mecanum_base bringup.launch.py > log.txt
ros2 run image_tools cam2image --ros-args -p use_compressed:=true
colcon build --packages-select mecanum_base --symlink-install
source install/setup.bash  
ros2 launch mecanum_base bringup.launch.py > log.txt
clear
sudo apt install ros-jazzy-compressed-image-transport
ros2 launch mecanum_base bringup.launch.py > log.txt
ros2 run image_tools cam2image --ros-args -p use_compressed:=true -p video_device:=/dev/video0
ros2 run image_tools cam2image --ros-args -p use_compressed:=true -p video_device:=/dev/video0
B
[200~ros2 run image_tools cam2image --ros-args -p use_compressed:=true -p video_device:=/dev/video0 -p backend:=v4l2
~
ros2 run image_tools cam2image --ros-args -p use_compressed:=true -p video_device:=/dev/video0 -p backend:=v4l2
apt install -y   gstreamer1.0-tools   gstreamer1.0-plugins-base   gstreamer1.0-plugins-good   gstreamer1.0-plugins-bad   gstreamer1.0-plugins-ugly   v4l-utils
sudo apt install -y   gstreamer1.0-tools   gstreamer1.0-plugins-base   gstreamer1.0-plugins-good   gstreamer1.0-plugins-bad   gstreamer1.0-plugins-ugly   v4l-utils
ros2 run image_tools cam2image --ros-args -p use_compressed:=true -p video_device:=/dev/video0 -p backend:=v4l2
ros2 run image_tools cam2image --ros-args -p use_compressed:=true -p video_device:=/dev/video0 -p backend:=v4l2
ros2 run image_tools cam2image --ros-args -p use_compressed:=true -p video_device:=/dev/video0 -p backend:=v4l2
ros2 run image_tools cam2image --ros-args -p use_compressed:=true -p video_device:=/dev/video0 
ros2 run image_tools cam2image --ros-args -p use_compressed:=true -p video_device:=/dev/video0 -p backend:=v4l2
ros2 run image_tools cam2image   --ros-args   -p use_compressed:=true   -p video_device:=/dev/video0   -r image:=/camera/image_raw
ros2 run image_tools cam2image   --ros-args   -p use_compressed:=true   -p video_device:=/dev/video0   -r image:=/camera/image_raw
sudo apt install ros-jazzy-compressed-image-transport
sudo apt install ros-jazzy-compressed-image-transport
ros2 run image_tools cam2image   --ros-args   -p use_compressed:=true   -p video_device:=/dev/video0   -r image:=/camera/image_raw
history| grep image
ros2 run image_tools cam2image --ros-args -p use_compressed:=true
ros2 run v4l2_camera v4l2_camera_node   --ros-args   -p video_device:=/dev/video0   -r image_raw:=/camera/image_raw
clear
sudo apt update
sudo apt install ros-jazzy-v4l2-camera
ros2 run v4l2_camera v4l2_camera_node   --ros-args   -p video_device:=/dev/video0   -r image_raw:=/camera/image_raw
ros2 run v4l2_camera v4l2_camera_node   --ros-args   -p video_device:=/dev/video0   -r image_raw:=/camera/image_raw
colcon build --packages-select mecanum_base --symlink-install
source install/setup.bash  
ros2 launch mecanum_base bringup.launch.py > log.txt
colcon build --packages-select mecanum_base --symlink-install
source install/setup.bash  
ros2 launch mecanum_base bringup.launch.py > log.txt
ros2 launch mecanum_base bringup.launch.py > log.txt
ros2 topic list
ros2 topic list
ros2 topic list
ros2 topic list
ros2 topic list
ros2 topic echo /battery_state_broadcaster/battery_state_throttled
AA
ros2 topic echo /battery_state_broadcaster/battery_state_throttled
ros2 topic echo /battery_state_broadcaster/battery_state_throttled
clear
ros2 topic echo /battery_state_broadcaster/battery_state_throttled
ros2 topic echo /battery_state_broadcaster/battery_state
ros2 topic echo /battery_state_broadcaster/battery_state
clear
ros2 control list_controllers
ros2 control list_controllers
clear
clear
ros2 control list_controllers
ros2 control list_state_interfaces
clear
ros2 control list_hardware_interfaces
ros2 topic echo /battery_state_broadcaster/battery_state_throttled
ros2 topic echo /battery_state_broadcaster/battery_state_throttled
ros2 topic echo /battery_state_broadcaster/battery_state_throttled
ros2 node list
ros2 run rqt_image_view rqt_image_view
bg
ros2 topic list
ros2 run rqt_image_view rqt_image_view&
ros2 run rqt_image_view rqt_image_view&
ros2 topic list
ros2 topic list | grep compre
ros2 topic list | grep compre
ros2 run rqt_image_view rqt_image_view&
clear
ros2 topic list | grep compre
ros2 run rqt_image_view rqt_image_view&
ros2 topic list | grep camera
ros2 run rqt_image_view rqt_image_view
ros2 topic list | grep comp
ros2 topic list 
ros2 run rqt_image_view rqt_image_view
ros2 run rqt_image_view rqt_image_view&
ros2 topic list | grep comp
ros2 run rqt_image_view rqt_image_view&
ros2 run rqt_image_view rqt_image_view&
ffplay /dev/video0
ros2 run rqt_image_view rqt_image_view&
ros2 topic list | grep comp
ros2 topic list
ros2 topic list
ros2 run rqt_image_view rqt_image_view
ros2 run rqt_image_view rqt_image_view
ros2 run rqt_image_view rqt_image_view
ros2 run rqt_image_view rqt_image_view
ros2 run rqt_gui rqt_gui
ros2 topic echo /camera/image_raw/compressed
ros2 topic echo /camera/image_raw/compressed
ros2 run rqt_gui rqt_gui
rqt_image_view
ros2 run rqt_gui rqt_image_view
ros2 run rqt_gui rqt_image_view
ros2 run rqt_image_view rqt_image_view
ros2 run rqt_gui rqt_gui
ros2 topic list
ros2 topic echo image_raw/compressed
ros2 run rqt_gui rqt_gui
ros2 run rqt_image_view rqt_image_view
ros2 topic list
ros2 run rqt_image_view rqt_image_view
ros2 run rqt_image_view rqt_image_view
ros2 run rqt_image_view rqt_image_view
ros2 topic list
ros2 topic list
ros2 topic list
ros2 run rqt_image_view rqt_image_view
ros2 run rqt_image_view rqt_image_view
ros2 topic list
ros2 run rqt_image_view rqt_image_view
ros2 topic list
source install/setup.bash 
ros2 run rqt_image_view rqt_image_view
ros2 topic info /camera/image_raw/compressed
colcon build --packages-select mecanum_base --symlink-install
colcon build --packages-select mecanum_base --symlink-install
source install/setup.bash 
ros2 launch mecanum_base bringup.launch.py 
colcon build --packages-select mecanum_base --symlink-install
source install/setup.bash 
ros2 launch mecanum_base bringup.launch.py 
ros2 topic echo /rosout_string 
ros2 topic echo /rosout_string 
ros2 topic info /rosout_string 
history|grep fetch
git pull origin main
git status
git status
git cherry-pick --continue
clear
git status
git checkout --theirs src/mecanum_base/web/calibration_interface.html
git add src/mecanum_base/web/calibration_interface.html
git cherry-pick --continue
git config --global core.editor "code --wait"
git cherry-pick --continue
git status
git status
git status
source install/setup.bash 
ros2 launch mecanum_base bringup.launch.py 
ros2 launch mecanum_base bringup.launch.py 
clear
ros2 launch mecanum_base bringup.launch.py 
clear
colcon build --packages-select mecanum_base --symlink-install
clear
colcon build --packages-select mecanum_base --symlink-install
ros2 launch mecanum_base bringup.launch.py 
source install/setup.
source install/setup.bash 
ros2 launch mecanum_base bringup.launch.py 
clear
ros2 launch mecanum_base bringup.launch.py 
colcon build --packages-select mecanum_base --symlink-install
sudo apt update
sudo apt install ros-jazzy-pal-statistics
ls /opt/ros/jazzy/include/pal_statistics/pal_statistics_macros.hpp
ls /opt/ros/jazzy/include/pal_statistics/
ls /opt/ros/jazzy/include/pal_statistics/
ls /opt/ros/jazzy/include/pal_statistics/pal_statistics/
history|grep rm
rm -rf build/ install/ log/
ls
colcon build --packages-select mecanum_base --symlink-install
source install/setup.bash 
ros2 launch mecanum_base bringup.launch.py 
clear
ls /dev/ttyUSB*
ros2 launch mecanum_base bringup.launch.py 
ls /dev/ttyUSB*
ls /dev/ttyUSB*
ls /dev/ttyUSB*
clear
ros2 launch mecanum_base bringup.launch.py 
ls /dev/ttyUSB*
ls /dev/ttyUSB*
ls /dev/ttyUSB*
ls /dev/ttyUSB*
ls /dev/ttyUSB*
ros2 launch mecanum_base bringup.launch.py 
source install/setup.bash 
ros2 launch mecanum_base bringup.launch.py 
ros2 launch mecanum_base bringup.launch.py 
ros2 launch mecanum_base bringup.launch.py 
clear
ros2 launch mecanum_base bringup.launch.py 
ros2 launch mecanum_base bringup.launch.py 
