Comando servo motori:
ros2 topic pub /servo_position_controller/commands std_msgs/msg/Float64MultiArray "data: [1.0, -0.5]"


ros2 launch mecanum_base bringup.launch.py --ros-args -r __log_level:=debug

ros2 launch mecanum_base bringup.launch.py -- --ros-args --log-level serial_manager:=debug



* Flusso IMU ***
1. Acquisizione
Pico legge sensori IMU (accel, gyro, mag) e invia pacchetti JSON via seriale.

2. Parsing e buffer
Nodo lato host interpreta il JSON (parseImuJson() in Serial manager.cpp), crea un ImuPacket e lo mette in un ImuBuffer condiviso.

3. Hardware interface
mecanum_system.cpp legge dal buffer in read() e aggiorna le state interfaces di ROS 2 control (accelerazione, velocità angolare, campo magnetico, quaternion unitario poiché non fornito da pico).
ELIMINARE Imu state interface perché non usate. uso infatti ekf che usa Imu/data

4.  raw
imu_bridge_node legge periodicamente dal buffer e pubblica:

/imu/data_raw → sensor_msgs/Imu con accel/gyro e quaternion unitario.

/imu/mag → sensor_msgs/MagneticField con dati magnetometro.

5. Calcolo orientamento
imu_filter_madgwick fonde /imu/data_raw + /imu/mag → calcola quaternion reale → pubblica /imu/data.

6. Broadcaster parallelo  ( DA RIMUOVERE PERCHE' NON SERVE !!!)
imu_sensor_broadcaster di ROS 2 control pubblica /imu_broadcaster/imu direttamente dalle state interfaces (utile per debug o uso alternativo).

7. Fusione sensoriale
robot_localization (EKF) legge:

/imu/data per orientamento e velocità angolare.

/mecanum_velocity_controller/odom per odometria.

Produce /odometry/filtered e TF odom → base_link.


====
Pacchetto completo ROS 2 Jazzy con meccanum, ros2_control SystemInterface mock/hardware e UART JSON
Ti consegno un pacchetto C++ completo e pronto all’uso che:

Implementa un SystemInterface ros2_control con toggle mock/hardware.

Comunica via UART con Raspberry Pi Pico usando stringhe JSON line-based.

Pubblica odometria e TF.

Integra RViz con RobotModel, TF e Odometry.

Si lancia con un unico launch parametrico.

In aggiunta
1. Buffer condiviso
Classe EncoderBuffer con mutex e queue FIFO per i pacchetti encoder.

2. Nodo UART reader
Legge dalla seriale, parsa JSON, e scrive nel buffer.

3. SystemInterface modificata
Legge dal buffer invece che dalla seriale.

-----
rviz/mecanum.rviz
Questo file configura RViz per visualizzare:

✅ Il modello robotico (RobotModel)

✅ La trasformazione odom → base_link (TF)

✅ L’odometria (Odometry)

rviz2 -d ~/ws/src/mecanum_base/rviz/mecanum.rviz
----
mecanum_hardware_plugin.xml
Questo file registra il plugin hardware MecanumSystem per ros2_control. Deve essere installato nella cartella share/mecanum_base/.

Note tecniche:

path="mecanum_system" corrisponde al nome della libreria compilata (libmecanum_system.so)

type="mecanum_hardware::MecanumSystem" è il nome della classe C++ implementata

base_class_type="hardware_interface::SystemInterface" è l’interfaccia ROS 2 da cui eredita
---
urdf/mecanum.urdf.xacro
Questo file descrive il robot e configura ros2_control. È scritto in Xacro per permettere parametri dinamici come mock, wheel_radius, serial_port, ecc.

🔧 Come usarlo:

Puoi passare parametri al file Xacro direttamente nel launch file o da terminale:
xacro mecanum.urdf.xacro mock:=false serial_port:=/dev/ttyUSB0
----

Risultato atteso
Il nodo uart_reader_node legge continuamente dalla seriale.

I dati encoder vengono messi in un buffer condiviso.

Il plugin hardware legge dal buffer in tempo reale, senza blocchi.

Il sistema è fluido, reattivo e compatibile con ros2_control.
---


Come testare il sistema
1. Compila il pacchetto
bash
cd ~/ws
[rm -rf build/mecanum_base install/mecanum_base log]

colcon build --packages-select mecanum_base
source install/setup.bash
2. Avvia tutto
bash
ros2 launch mecanum_base bringup.launch.py
3. Invia comandi
bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2, y: 0.1}, angular: {z: 0.5}}" -r 10
4. Osserva in RViz
RobotModel visibile

TF odom → base_link

Odometry aggiornata

=======================================================================
=======================================================================

* Contenuti principali:
=======================

Plugin hardware (mock): src/mecanum_system.cpp, include/mecanum_hardware/mecanum_system.hpp, mecanum_hardware_plugin.xml

Nodo cmd_vel→ruote: src/mecanum_cmd_node.cpp

Nodo odometria: src/mecanum_odom_node.cpp

URDF con ros2_control: urdf/mecanum.urdf

Config ros2_control: config/controllers.yaml

Launch: launch/bringup.launch.py
----
Descrizione dei contenuti
Percorso	Contenuto
include/mecanum_hardware/	    Header del plugin hardware
src/mecanum_system.cpp	        Implementazione SystemInterface mock
src/mecanum_cmd_node.cpp	    Nodo che converte /cmd_vel in velocità ruote
src/mecanum_odom_node.cpp	    Nodo che calcola /odom da /joint_states
urdf/mecanum.urdf	            URDF del robot con <ros2_control> e plugin hardware
config/controllers.yaml	        Configurazione controller velocity + broadcaster
launch/bringup.launch.py	    Avvio di ros2_control_node, controller manager, nodi C++
mecanum_hardware_plugin.xml	    Pluginlib XML per registrare il plugin hardware
CMakeLists.txt	                Build C++ e installazione plugin/nodi
package.xml	                    Dipendenze ROS 2 e dichiarazione pacchetto
----

mecanum_base/
├─ CMakeLists.txt
├─ package.xml
├─ mecanum_hardware_plugin.xml
├─ include/
│  └─ mecanum_hardware/
│     └─ mecanum_system.hpp
├─ src/
│  ├─ mecanum_system.cpp
│  ├─ mecanum_cmd_node.cpp
│  └─ mecanum_odom_node.cpp
├─ urdf/
│  └─ mecanum.urdf
├─ config/
│  └─ controllers.yaml
└─ launch/
   └─ bringup.launch.py
----

🛠️ Compilazione
Assicurati di essere nella root del tuo workspace:

bash
cd ~/ws
colcon build --packages-select mecanum_base
Poi sorgi l’ambiente:

bash
source ~/ws/install/setup.bash
----

🧪 Test passo-passo
1. Avvio del sistema
bash
ros2 launch mecanum_base bringup.launch.py
Vedrai in console:

Avvio di ros2_control_node

Spawner dei controller

Avvio dei nodi mecanum_cmd_node e mecanum_odom_node

2. Invia comandi
bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2, y: 0.1}, angular: {z: 0.5}}" -r 10

3. Osserva i risultati
✅ /joint_states: pubblicato dal broadcaster

✅ /odom: pubblicato dal nodo odometria

✅ TF odom → base_link: visibile in RViz

✅ /mecanum_velocity_controller/commands: riceve i comandi ruota

4. Visualizza
bash
ros2 topic echo /odom
ros2 topic echo /joint_states
Oppure apri RViz e aggiungi:

Odometry (topic: /odom)

TF

RobotModel (se hai mesh nel URDF)

🎯 Cosa aspettarti
Il robot virtuale (mock) risponde ai comandi /cmd_vel con movimento fluido.

Le ruote simulano accelerazione verso il setpoint.

La posizione del robot si aggiorna correttamente in /odom.

Il sistema è pronto per essere collegato a un microcontrollore reale (es. Raspberry Pi Pico W) sostituendo il mock con lettura encoder e scrittura motori.
