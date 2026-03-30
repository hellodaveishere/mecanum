# 🤖 Sistema di Navigazione per Robot Mecanum (ROS 2)

Questo pacchetto ROS 2 fornisce l’infrastruttura software completa per un robot mobile con ruote Mecanum, includendo:

- ros2_control con hardware_interface custom  
- odometria, calibrazione, gestione E‑Stop  
- EKF (robot_localization)  
- SLAM Toolbox (mapping/localization)  
- AMCL (localizzazione su mappa)  
- Nav2 (navigazione autonoma completa)  
- LiDAR, IMU, IR, servo, batteria  
- Interfaccia web via rosbridge + Node.js  
- Behavior Tree personalizzabile  

Il sistema è progettato per robot indoor compatti, con navigazione precisa e affidabile.

---

# 📂 Struttura del pacchetto

mecanum_base/  
├── config/  
│   ├── nav2_params.yaml  
│   ├── controllers.yaml  
│   ├── ekf.yaml  
│   └── slam_toolbox.yaml  
├── maps/  
│   ├── mappa.yaml  
│   └── mappa.pgm/png  
├── bt_xml/  
│   └── navigate.xml  
├── launch/  
│   ├── bringup_navigation.launch.py  
│   └── slam_launch.py  
├── urdf/  
│   └── mecanum_robot.xacro  
├── src/  
│   ├── mecanum_system.cpp  
│   ├── mecanum_cmd_node.cpp  
│   ├── mecanum_odom_node.cpp  
│   ├── calibration_node.cpp  
│   ├── estop_manager_node.cpp  
│   ├── rosout_relay_node.cpp  
│   └── my_nav_client.cpp  
└── webserver/

---

# 🧩 Architettura del sistema

## ros2_control
Gestisce motori, encoder, servo, IR, IMU, batteria, E‑Stop.

## EKF (robot_localization)
Fonde odometria, IMU e altri sensori per produrre `/odometry/filtered`.

## LiDAR
Usato per SLAM, AMCL e collision monitor.

## SLAM Toolbox
- mapping → crea la mappa  
- localization → usa la mappa salvata  

## Nav2
Componenti principali:
- planner_server → calcola il percorso  
- controller_server → genera cmd_vel  
- smoother_server → rende il percorso fluido  
- behavior_server → recovery automatiche  
- waypoint_follower → navigazione multi-goal  
- velocity_smoother → filtra cmd_vel  
- collision_monitor → evita collisioni  
- bt_navigator → coordina tutto tramite Behavior Tree  
- lifecycle_manager → attiva i nodi nell’ordine corretto  

---

# 🗺️ Creazione della mappa (MAPPING MODE)

## 1. Avvia SLAM Toolbox
ros2 launch mecanum_base slam_launch.py

Questo avvia:
- SLAM Toolbox in modalità mapping  
- LiDAR  
- TF  
- ros2_control  
- odometria  
- EKF  

## 2️⃣ Muovi il robot lentamente
- copri tutte le stanze  
- evita movimenti bruschi  
- chiudi loop (ripassa negli stessi punti)  

## 3️⃣ Salva la mappa
ros2 run nav2_map_server map_saver_cli -f mappa

Genera:
mappa.yaml
mappa.pgm


## 4️⃣ Copia i file nella cartella maps/
mecanum_base/maps/mappa.yaml
mecanum_base/maps/mappa.pgm


---

# 🎯 Uso della mappa (LOCALIZATION MODE)

## 1️⃣ Modifica `slam_toolbox.yaml`
mode: "localization"

## 2️⃣ Modifica `nav2_params.yaml`
map_server:
    ros__parameters:
    yaml_filename: "maps/mappa.yaml"

## 3️⃣ Avvia Nav2
ros2 launch mecanum_base bringup_navigation.launch.py rviz:=true


## 4️⃣ Ora puoi:
- inviare goal  
- seguire waypoint  
- navigare autonomamente  

---

# 🚀 Comandi rapidi

### Avvio completo robot + Nav2
ros2 launch mecanum_base bringup_navigation.launch.py

### Avvio SLAM Toolbox (mapping)
ros2 launch mecanum_base slam_launch.py

### Salvataggio mappa
ros2 run nav2_map_server map_saver_cli -f mappa

### Invio goal da terminale
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}"

---

# 🌳 Behavior Tree (BT)

Il file:
bt_xml/navigate.xml

Contiene la logica:
- calcolo percorso  
- esecuzione percorso  
- recovery automatiche  

Puoi modificarlo per:
- aggiungere recovery  
- cambiare comportamento  
- aggiungere condizioni personalizzate  

---

# 🔧 Troubleshooting

## ❌ Il robot non si muove
ros2 control list_controllers

## ❌ Nav2 non parte
ros2 lifecycle get /planner_server

## ❌ AMCL non converge
ros2 run tf2_tools view_frames

## ❌ SLAM produce una mappa distorta
Riduci:
minimum_travel_distance: 0.2
minimum_travel_heading: 0.2

## ❌ Il robot si ferma troppo presto
Modifica:
time_before_collision: 1.2

---

# 🧠 Pipeline TF
map
↑  (AMCL / SLAM)
|
odom
↑  (EKF)
|
base_footprint
↑
base_link
↑
ruote / sensori

---

# 🧠 Pipeline di navigazione
LiDAR → Costmap → Planner → Smoother → Controller → Motori
↑                                   ↓
AMCL ← EKF ← Odometria ← ros2_control

---

# 🛠️ Per sviluppatori

## Compilazione
colcon build --symlink-install
source install/setup.bash

## Test nodi
ros2 run mecanum_base mecanum_cmd_node
ros2 run mecanum_base mecanum_odom_node
ros2 run mecanum_base estop_manager_node

## Verifica TF
ros2 run tf2_tools view_frames

---

# 📜 Licenza
Apache 2.0

---

# 📬 Contatti
- Email: tuo.email@example.com
- GitHub: *(aggiungi link)*






