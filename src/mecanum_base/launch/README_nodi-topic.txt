===============================================================================
📦 Mecanum Base – Controller e Broadcaster ROS 2
===============================================================================

| Nome controller/broadcaster     | Tipo ROS 2 Control       | Tipo messaggio ROS 2           | Topic (I/O)            | Descrizione                                                  |
|---------------------------------|---------------------------|----------------------------------|-------------------------|--------------------------------------------------------------|
| joint_state_broadcaster         | Broadcaster               | sensor_msgs/JointState          | /joint_states (output)  | Stato dei giunti: posizione, velocità, sforzo                |
| mecanum_velocity_controller     | Controller                | geometry_msgs/Twist             | /commands/velocity (input)| Comandi di velocità per le ruote Mecanum                    |
| servo_position_controller       | Controller                | std_msgs/Float64MultiArray      | /commands/position (input)| Comandi di posizione per i servomotori pan/tilt             |
| imu_broadcaster                 | Broadcaster               | sensor_msgs/Imu                 | /data/imu (output)      | Dati IMU: orientazione, velocità angolare, accelerazione    |
| ir_front_left_broadcaster      | Broadcaster               | sensor_msgs/Range               | /ir_front_left (output) | Distanza dal sensore IR frontale sinistro                   |
| ir_front_center_broadcaster    | Broadcaster               | sensor_msgs/Range               | /ir_front_center (output)| Distanza dal sensore IR frontale centrale                   |
| ir_front_right_broadcaster     | Broadcaster               | sensor_msgs/Range               | /ir_front_right (output)| Distanza dal sensore IR frontale destro                     |
| battery_state_broadcaster      | Broadcaster               | sensor_msgs/BatteryState        | /battery_state (output) | Stato batteria: tensione, percentuale, salute, tecnologia   |
