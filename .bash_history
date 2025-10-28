ls
cd
ls
ros2
git rm -r --cached install/ build/ log/
git commit -m "Ignora build/, install/ e log/"
ls
ls -al
more .gitignore 
git rm -r --cached install/ build/ log/
ls
ls install/
git rm -r --cached .vscode .vscode-server .vscode-remote
git commit -m "Ignora dati VS Code e VS Code Server"
ls
howmai
whomai
whoami
pwd
ls -al
more .bashrc 
ros2 launch mecanum_base bringup.launch.py
source install/setup.bash 
ros2 launch mecanum_base bringup.launch.py
ros2 launch mecanum_base bringup.launch.py
cd .git
ls
ps aux | grep git
cd..
cd ..
ls -al /home/ws/.git/index.lock
rm -f /home/ws/.git/index.lock
ls -ld /home/ws/.git
whoami
df -h
git config --global user.email "dave@dave.com"
git config --global user.name "Dave"
ls
pwd
ls
git --version
ros2
ros2
git --version
cat -n /home/ws/.gitconfig
git config --list
git --version
ps aux | grep git
ps aux | grep git
ps aux | grep git
ps aux | grep git
ssh -T git@github.com
git push origin -u main
git status
git remote -v
cat .git/config
git remote add origin git@github.com:hellodaveishere/mecanum.git
git push -u origin main
git fsck --full
pwd
ls
ls -al src
tee
tree
ls .vscode
ls -al
clear
pwd
ls -al
pwd
rm -rf .git
git init
git add .
rm -rf src/Archive/serial-ros2/.git
rm -rf src/examples/.git
git rm --cached src/Archive/serial-ros2
git rm --cached src/examples
git rm --cached -r -f src/Archive/serial-ros2
git rm --cached -r -f src/examples
ls src/Archive/serial-ros2
ls src/examples
rm -rf src/Archive/serial-ros2/.git
rm -rf src/examples/.git
git add src/Archive/serial-ros2
git add src/examples
clear
git status
git commit -m "Pulizia dei repository annidati e reinserimento come cartelle normali"
git remote -v
git remote add origin git@github.com:hellodaveishere/mecanum.git
git remote -v
git push -u origin main
git branch
git branch -m main
git push -u origin main
git ls-files | grep -Ei '(\.pem|\.key|\.crt|\.env|\.secret|\.token|\.p12|\.pfx|id_ed25519)'
git ls-files | grep -E '^\.ssh/|^\.gnupg/|^\.ros/|^\.rviz2/'
git log --all --pretty=format:"%H" | while read commit_hash; do   git ls-tree -r "$commit_hash" | grep -Ei '(\.pem|\.key|\.env|id_ed25519)' && echo "🔐 Found in commit $commit_hash"; done
git filter-branch --force --index-filter   'git rm --cached --ignore-unmatch .ssh/id_ed25519 .ssh/id_ed25519.pub .ros/log/latest .rviz2/persistent_settings .ssh/known_hosts .ssh/known_hosts.old'   --prune-empty --tag-name-filter cat -- --all
git add .
git commit -m "Salvataggio temporaneo prima della pulizia"
git filter-branch --force --index-filter   'git rm --cached --ignore-unmatch .ssh/id_ed25519 .ssh/id_ed25519.pub .ros/log/latest .rviz2/persistent_settings .ssh/known_hosts .ssh/known_hosts.old'   --prune-empty --tag-name-filter cat -- --all
rm -rf .git/refs/original/
git reflog expire --expire=now --all
git gc --prune=now --aggressive
git push -u origin main --force
ls ~/.ssh/id_ed25519.pub
ls ~/.ssh/
ls -la ~/.ssh/
ls -la ~/.ssh/
ssh-keygen -t ed25519 -C "dave@dave.com"
cat ~/.ssh/id_ed25519.pub
ssh -T git@github.com
git push -u origin main --force
ros2 launch mecanum_base bringup.launch.py
cat .bashrc 
more .bashrc 
colcon build --packages-select mecanum_base
source install/setup.bash 
ros2 launch mecanum_base bringup.launch.py
ros2 launch mecanum_base bringup.launch.py
ros2 launch mecanum_base bringup.launch.py
rqt&
ros2 launch mecanum_base bringup.launch.py
colcon build --packages-select mecanum_base
source install/setup.bash 
ros2 launch mecanum_base bringup.launch.py
colcon build --packages-select mecanum_base
source install/setup.bash 
ros2 launch mecanum_base bringup.launch.py
colcon build --packages-select mecanum_base
source install/setup.bash 
ros2 launch mecanum_base bringup.launch.py
ros2 topic echo /rosout | grep MecanumSystem
ros2 topic echo /rosot
ros2 topic echo /rosout
ros2 topic echo /rosout | grep MecanumSystem
rqt&
cp -r src/sllidar_ros2 /tmp/sllidar_ros2_backup
git checkout main
``>
#### 4. **Copia la directory nel branch `main`**
```bash
cp -r /tmp/sllidar_ros2_backup src/sllidar_ros2

cp -r /tmp/sllidar_ros2_backup src/sllidar_ros2
git add src/sllidar_ros2
git commit -m "Aggiungo sllidar_ros2 al branch main"
git push origin main
git pull origin main
git pull origin main --no-rebase
git commit -m "Merge branch 'origin/main' into main"
git push origin main
colcon build --packages-select mecanum_base
source install/setup.bash 
colcon build --packages-select mecanum_base
colcon build --packages-select mecanum_base
clear
colcon build --packages-select mecanum_base
clear
colcon build --packages-select mecanum_base
cd src/
ls
cd mecanum_base/
tree
cd src/
ls
ros2 pkg create --build-type ament_cmake ir_sensor_broadcaster --dependencies rclcpp controller_interface std_msgs
cd
clear
colcon build --packages-select mecanum_base
colcon build --packages-select mecanum_base
clear
colcon build --packages-select mecanum_base
ros2 pkg list | grep controller_interface
clear
ros2 pkg list | grep controller_interface
colcon build --packages-select mecanum_base
clear
colcon build --packages-select mecanum_base
clear
colcon build --packages-select mecanum_base
clear
colcon build --packages-select mecanum_base
ros2 control list_controllers
source install/setup.bash 
ros2 launch mecanum_base bringup.launch.py
ros2 launch mecanum_base bringup.launch.py
clear
ros2 control list_controllers
colcon build --packages-select mecanum_base
ros2 control list_controllers
clear
colcon build --packages-select mecanum_base
ls src/mecanum_base/mecanum_hardware_plugin.xml
ls src/mecanum_base/ir_sensor_broadcaster.xml
clear
colcon build --packages-select mecanum_base
ls src/mecanum_base/mecanum_hardware_plugin.xml
clear
colcon build --packages-select mecanum_base
clear
colcon build --packages-select mecanum_base
source install/setup.bash 
ros2 launch mecanum_base bringup.launch.py
clear
colcon build --packages-select mecanum_base
source install/setup.bash 
ros2 launch mecanum_base bringup.launch.py
clear
colcon build --packages-select mecanum_base
source install/setup.bash 
ros2 control list_controller_types
ros2 launch mecanum_base bringup.launch.py
source install/setup.bash
source install/setup.bash
clear
ros2 run pluginlib pluginlib_tester controller_interface::ControllerInterface mecanum_base/IRSensorBroadcaster
ros2 control list_controller_types
ros2 node list
ros2 node list
clear
colcon build --packages-select mecanum_base
source install/setup.bash
ros2 launch mecanum_base bringup.launch.py
clear
colcon build --packages-select mecanum_base
clear
colcon build --packages-select mecanum_base
source install/setup.bash
ros2 launch mecanum_base bringup.launch.py
clear
colcon build --packages-select mecanum_base
source install/setup.bash
ros2 launch mecanum_base bringup.launch.py
clear
colcon build --packages-select mecanum_base
src/mecanum_base/mecanum_hardware_plugin.xml
src/mecanum_base/ir_sensor_broadcaster.xml
ls src/mecanum_base/ir_sensor_broadcaster.xml
ls src/mecanum_base/mecanum_hardware_plugin.xml
clear
rm -rf build/ install/ log/
colcon build --packages-select mecanum_base
source install/setup.bash
source install/setup.bash
ros2 launch mecanum_base bringup.launch.py
clear
ros2 launch mecanum_base ir_sensor_broadcaster.launch.py
source install/setup.bash 
ros2 launch mecanum_base ir_sensor_broadcaster.launch.py
colcon build --packages-select mecanum_base
source install/setup.bash 
ros2 launch mecanum_base ir_sensor_broadcaster.launch.py
clear
colcon build --packages-select mecanum_base
source install/setup.bash 
ros2 launch mecanum_base ir_sensor_broadcaster.launch.py
clear
colcon build --packages-select mecanum_base
source install/setup.bash 
ros2 launch mecanum_base ir_sensor_broadcaster.launch.py
clear
colcon build --packages-select mecanum_base
source install/setup.bash 
ros2 launch mecanum_base ir_sensor_broadcaster.launch.py
clear
colcon build --packages-select mecanum_base
source install/setup.bash 
ros2 launch mecanum_base ir_sensor_broadcaster.launch.py
clear
colcon build --packages-select mecanum_base
source install/setup.bash 
ros2 launch mecanum_base ir_sensor_broadcaster.launch.py
colcon build --packages-select mecanum_base
source install/setup.bash 
colcon build --packages-select mecanum_base
source install/setup.bash 
ros2 launch mecanum_base ir_sensor_broadcaster.launch.py
clear
colcon build --packages-select mecanum_base
source install/setup.bash 
ros2 launch mecanum_base ir_sensor_broadcaster.launch.py
clear
colcon build --packages-select mecanum_base
source install/setup.bash 
ros2 launch mecanum_base ir_sensor_broadcaster.launch.py
clear
colcon build --packages-select mecanum_base
source install/setup.bash 
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
