# ============================================================================
#  SystemState.msg
#  --------------------------------------------------------------------------
#  Stato logico completo del robot, aggiornato dal pacchetto STS (1 Hz).
#  Ogni campo è un intero TaskStatus (0–5):
#    0 = UNKNOWN
#    1 = RUNNING
#    2 = FAILED
#    3 = DISABLED
#    4 = TIMEOUT
#    5 = ERROR
# ============================================================================

int32 robot_status          # Stato generale del robot

int32[4] motor_status       # Stato dei 4 motori (FL, FR, RL, RR)
int32[4] encoder_status     # Stato dei 4 encoder (FL, FR, RL, RR)

int32[2] servo_status       # Stato dei 2 servo (pan, tilt)
int32[3] ir_status          # Stato dei 3 IR (front-left, front-center, front-right)

int32 vacuum_status         # Stato sensore aspirazione / vacuum
int32 imu_status            # Stato IMU
int32 battery_status        # Stato monitor batteria
