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
grep B1000000 /usr/include/asm-generic/termbits.h
colcon build --packages-select mecanum_base
source install/setup.bash 
source install/setup.bash 
ros2 launch mecanum_base bringup.launch.py
colcon build --packages-select mecanum_base
source install/setup.bash 
ros2 launch mecanum_base bringup.launch.py
colcon build --packages-select mecanum_base
source install/setup.bash 
colcon build --packages-select mecanum_base
ros2 launch mecanum_base bringup.launch.py
rqt&
cd src/
cd mecanum_base/tests/
ls
./test_mecanum.sh 
./test_mecanum.sh 
colcon build --packages-select mecanum_base
source install/setup.bash 
ros2 launch mecanum_base bringup.launch.py
ros2 launch mecanum_base bringup.launch.py
colcon build --packages-select mecanum_base
source install/setup.bash 
ros2 launch mecanum_base bringup.launch.py
sudo apt install ros-noble-ros2-control ros-noble-ros2-controllers
ros2 pkg prefix ros2_controllers
ls
cd install/mecanum_base/share/
ls
cd mecanum_base/
ls
cd
ls /opt/ros/jazzy/share/ros2_controllers/
ls /opt/ros/jazzy/share/ros2_controllers/
sudo apt install ros-jazzy-position-controllers
ros2 run controller_manager spawner --help
clear
ros2 control list_controller_types
ros2 launch mecanum_base bringup.launch.py
colcon build --packages-select mecanum_base
source install/setup.bash 
ros2 launch mecanum_base bringup.launch.py
colcon build --packages-select mecanum_base
source install/setup.bash 
ros2 launch mecanum_base bringup.launch.py
clear
ros2 launch mecanum_base bringup.launch.py
ros2 launch mecanum_base bringup.launch.py
clear
ros2 launch mecanum_base bringup.launch.py
clear
ros2 launch mecanum_base bringup.launch.py
colcon build --packages-select mecanum_base
source install/setup.bash 
ros2 launch mecanum_base bringup.launch.py
clear
ros2 launch mecanum_base bringup.launch.py
ros2 launch mecanum_base bringup.launch.py
ros2 launch mecanum_base bringup.launch.py
ros2 launch mecanum_base bringup.launch.py
source install/setup.bash 
ros2 launch mecanum_base bringup.launch.py
rm -rf .git/refs/original/
git status
git reflog
history|grep rm
rm -rf build/mecanum_base install/mecanum_base log
colcon build --packages-select mecanum_base
source install/setup.bash 
ros2 launch mecanum_base bringup.launch.py
ros2 launch mecanum_base bringup.launch.py
ros2 launch mecanum_base bringup.launch.py
ros2 launch mecanum_base bringup.launch.py
ros2 control list_controller_types
ros2 control list_controllers
ros2 topic list
cd src/mecanum_base/tests/
ls
chmod +x test_ruote_servos_sonar.sh 
./test_ruote_servos_sonar.sh 
ros2 topic list
./test_ruote_servos_sonar.sh 
./test_ruote_servos_sonar.sh 2
ros2 control controllers_list
ros2 control list_controllers
ros2 control list_controllers
ros2 control list_controllers
rqt&
./test_ruote_servos_sonar.sh 2
./test_ruote_servos_sonar.sh 2
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 control list_controllers
./test_ruote_servos_sonar.sh 2
ros2 control list_controllers
