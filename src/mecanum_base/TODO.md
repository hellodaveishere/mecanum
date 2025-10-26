# 📝 TODO: Limitare frequenza messaggi per rosbridge

## ✅ Obiettivo
Limitare la frequenza dei messaggi pubblicati su un topic ROS 2 per evitare saturazione del WebSocket usato da rosbridge.

---

## 🔧 Passaggi

- [ ] Installare `topic_tools`:
      sudo apt install ros-humble-topic-tools

- [ ] Avviare nodo throttle manualmente (per test):
      ros2 run topic_tools throttle messages /input_topic 5.0 /throttled_topic

- [ ] Modificare la pagina web per fare subscribe a `/throttled_topic` invece di `/input_topic`

- [ ] Verificare che `rosbridge_websocket` sia attivo:
      ros2 launch rosbridge_server rosbridge_websocket_launch.xml

- [ ] Testare ricezione messaggi a frequenza ridotta nel browser

---

## 🚀 Integrazione in launch file

- [ ] Creare un file `throttle.launch.py` nel tuo pacchetto ROS:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='topic_tools',
            executable='throttle',
            name='throttle_node',
            arguments=['messages', '/input_topic', '5.0', '/throttled_topic']
        )
    ])