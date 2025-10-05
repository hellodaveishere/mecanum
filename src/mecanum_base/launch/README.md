

🤖 Mecanum Robot ROS 2

Sistema robotico basato su ruote Mecanum, con supporto per:

- Controllo cinematico via ros2_control
- IMU integrata
- Servomotori pan/tilt
- 3 sonar HC-SR04 (frontale, sinistro, destro)
- Localizzazione con EKF
- Visualizzazione in RViz

---

📦 Struttura del pacchetto

`
mecanum_base/
├── urdf/
│   └── mecanum_robot.xacro       # Descrizione robotica
├── config/
│   ├── controllers.yaml          # Configurazione dei controller
│   └── ekf.yaml                  # Configurazione EKF
├── rviz/
│   └── mecanum.rviz              # Layout RViz
├── launch/
│   └── mecanum.launch.py         # File di lancio principale
├── src/
│   ├── mecanumcmdnode.cpp      # Nodo comandi
│   └── mecanumodomnode.cpp     # Nodo odometria
`

---

🚀 Avvio del sistema

`bash
ros2 launch mecanum_base mecanum.launch.py
`

🔧 Argomenti disponibili

| Argomento | Default | Descrizione                  |
|----------|---------|------------------------------|
| rviz   | true  | Avvia RViz alla partenza     |

---

🧠 Componenti principali

1. ros2_control
- Plugin: mecanum_hardware/MecanumSystem
- Gestisce ruote, servo e sonar via seriale

2. Controller attivi

| Nome controller               | Tipo                                      | Funzione                      |
|------------------------------|-------------------------------------------|-------------------------------|
| mecanumvelocitycontroller| JointGroupVelocityController            | Controllo ruote mecanum       |
| pan_controller             | JointPositionController                 | Controllo servo pan           |
| tilt_controller            | JointPositionController                 | Controllo servo tilt          |
| imu_broadcaster            | IMUSensorBroadcaster                    | Pubblicazione dati IMU        |
| sonarfrontbroadcaster    | RangeSensorBroadcaster                  | Sonar frontale                |
| sonarleftbroadcaster     | RangeSensorBroadcaster                  | Sonar sinistro                |
| sonarrightbroadcaster    | RangeSensorBroadcaster                  | Sonar destro                  |
| jointstatebroadcaster    | JointStateBroadcaster                   | Stato dei giunti              |

3. Nodi custom

- mecanumcmdnode: riceve comandi da /cmd_vel
- mecanumodomnode: calcola odometria

4. EKF

- Fonde IMU e odometria per stimare la posa
- Configurato in config/ekf.yaml

---

📡 Comunicazione seriale

Il microcontroller invia pacchetti CSV come:

`
ENC,<dlfl>,<dlfr>,<dlrl>,<dlrr>
IMU,<qx>,<qy>,<qz>,<qw>,<gx>,<gy>,<gz>,<ax>,<ay>,<az>
SON,<rangefront>,<rangeleft>,<range_right>
CMD,<velfl>,<velfr>,<velrl>,<velrr>,<panpos>,<tiltpos>
`

- ENC: delta encoder ruote
- IMU: orientamento e accelerazioni
- SON: distanza sonar in metri
- CMD: comandi da ROS 2 verso microcontroller

---

👀 Visualizzazione

- RViz mostra:
  - Modello robotico
  - Pose e traiettoria
  - Dati IMU e sonar
  - Stato dei giunti

---

🧪 Test rapido

`bash

Verifica pubblicazione sonar
ros2 topic echo /data/sonar_front

Comando servo pan
ros2 topic pub /pancontroller/commands stdmsgs/msg/Float64 "data: 0.5"

Comando servo tilt
ros2 topic pub /tiltcontroller/commands stdmsgs/msg/Float64 "data: -0.3"
`

---

🛠️ Requisiti

- ROS 2 Humble o successivo
- Microcontroller compatibile (Arduino, Pi Pico, ESP32)
- Porta seriale configurata in controllers.yaml

---

📄 Licenza

Questo progetto è distribuito sotto licenza MIT.
`
