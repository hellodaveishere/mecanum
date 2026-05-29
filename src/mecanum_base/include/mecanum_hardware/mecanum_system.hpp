#pragma once

#include <string>
#include <vector>
#include <memory>
#include <optional>
#include <algorithm>
#include <mutex>
#include <cmath>
#include <unordered_map>   // necessario per mappe timeout/timestamp

// ROS 2 Control
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/hardware_component_info.hpp"

// ROS 2 Core
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"

// Messaggi standard IMU
#include "sensor_msgs/msg/imu.hpp"

// per avere hash di default negli enum class e usare unordered_map
namespace std {
template <>
struct hash<MecanumSystem::SensorType>
{
    size_t operator()(const MecanumSystem::SensorType& t) const noexcept
    {
        return static_cast<size_t>(t);
    }
};
}


namespace mecanum_hardware
{

    // 📦 Stato per ciascuna ruota (posizione, velocità e comando)
    struct JointState
    {
        double pos_rad = 0.0;   // posizione cumulativa letta dall’hardware (rad)
        double vel_rad_s = 0.0; // velocità filtrata letta dall’hardware (rad/s)
        double effort = 0.0;    // eventuale sforzo (se il microcontrollore lo invia)
        double cmd_vel = 0.0;   // comando di velocità ricevuto da ros2_control
    };

    // 📦 Stato IMU (orientamento, velocità angolare, accelerazione lineare)
    struct ImuState
    {
        double orientation[4]{0.0, 0.0, 0.0, 1.0}; // Quaternion [x,y,z,w]
        double angular_vel[3]{0.0, 0.0, 0.0};      // [rad/s]
        double linear_accel[3]{0.0, 0.0, 0.0};     // [m/s²]
    };

    // Struttura per contenere i valori dei tre sensori IR.
    // Ogni campo rappresenta la lettura di un sensore IR specifico.
    // Struttura per contenere i valori dei tre sensori IR frontali.
    struct IRState
    {
        double ir_front_left = 0.0;   // Sensore IR frontale sinistro [m]
        double ir_front_center = 0.0; // Sensore IR frontale centrale [m]
        double ir_front_right = 0.0;  // Sensore IR frontale destro [m]
    };

    // 📦 Stato dei servomotori (in radianti)
    struct ServoState
    {
        double pan_position = 90 * M_PI / 180.0; // Posizione del servo pan
        double tilt_position = 0.0;              // Posizione del servo tilt
    };

    // 📦 Comandi desiderati per i servomotori (in radianti)
    struct ServoCommand
    {
        double pan_position = 90 * M_PI / 180.0; // Comando di posizione per il servo pan
        double tilt_position = 0.0;              // Comando di posizione per il servo tilt
    };

    // Struttura per raggruppare lo stato della batteria
    struct BatteryState
    {
        double voltage = 0.0; // Tensione della batteria (V)
        // double current = 0.0;     // Corrente assorbita (A)
        // double temperature = 0.0; // Temperatura (°C)
        // double charge = 0.0;      // Carica attuale (Ah)
        // double capacity = 0.0;    // Capacità nominale (Ah)
        double percentage = 0.0; // Stato di carica in percentuale (0.0–1.0)
    };

    // 🔧 Classe principale del sistema hardware Mecanum
    //    Implementa l'interfaccia ROS 2 Control e comunica direttamente via seriale
    class MecanumSystem final : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(MecanumSystem)

        // 🔄 Inizializzazione del sistema hardware
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

        // 📤 Esportazione delle interfacce di stato (posizione, velocità, imu)
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        // 📥 Esportazione delle interfacce di comando (cmd_vel)
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        // ⚡ Attivazione del sistema
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &prev_state) override;

        // 💤 Disattivazione del sistema
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &prev_state) override;

        // 📡 Lettura da hardware (encoder + imu)
        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        // 📝 Scrittura su hardware (comandi motori)
        hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        // ℹ️ Informazioni hardware (popolate da URDF/XACRO)
        hardware_interface::HardwareInfo info_;

    private:
        // Usato per calcolare frequenza fra due messaggi ricevuti via UART
        std::unordered_map<std::string, rclcpp::Time> last_stamp_;
        std::unordered_map<std::string, bool> first_stamp_;


        // Stato E-STOP hardware (1.0 = attivo, 0.0 = non attivo)
        double estop_active_state_{0.0};

        // Flag per tcdrain: se true, forza lo svuotamento del buffer di trasmissione
        bool enable_tcdrain_{false};

        // NOTA: alvune variabile sono definite nel file URDF
        // e poi caricate con es getP("wheel_radius", wheel_radius_); in file mecanum_system.cpp

        // ⚙️ Parametri cinematici e logici
        double wheel_radius_{0.025}; // Raggio ruota [m]
        double L_{0.15};             // Metà lunghezza telaio [m]
        double W_{0.15};             // Metà larghezza telaio [m]
        bool mock_{false};           // true = simulazione, false = hardware reale
        double accel_limit_{25.0};   // Limite accelerazione [rad/s²] (solo mock)

        // ⚙️ Parametri encoder
        int ticks_per_rev_{12};           // Tick per giro encoder (lato motore)
        double gear_ratio_{90.0};         // Rapporto di trasmissione
        double ticks_per_wheel_rev_{0.0}; // Tick per giro ruota = encoder * rapporto

        // 🔁 Inversione direzione ruote (1 o -1)
        int inv_fl_{1}, inv_fr_{1}, inv_rl_{1}, inv_rr_{1};

        // 🔩 Stato dei giunti (ruote)
        std::vector<std::string> joint_names_; // Nomi dei giunti
        std::vector<JointState> joints_;       // Stato di ciascun giunto

        // 📦 Stato IMU
        ImuState imu_state_;

        // Istanza della struttura IRState per memorizzare lo stato corrente dei sensori IR.
        IRState ir_state_;

        // Istanza della struttura ServoState per memorizzare la posizione corrente dei serco pan, tile
        ServoState servo_state_;

        // 📦 Comandi desiderati per i servomotori (in radianti)
        ServoCommand servo_command_;

        // Stato della batteria
        BatteryState battery_state_; // Struttura contenente tutti i parametri della batteria

        // 🧪 Simulazione mock: dinamica di primo ordine
        void apply_mock_dynamics_(double dt);

        // 📡 Gestione seriale diretta
        //std::string serial_port_{"/dev/ttyUSB0"}; // Porta seriale tramite FTDI232
        std::string serial_port_{"/dev/ttyAMA0"}; // Porta seriale diretta su UART0 (PL011) attiva su GPIO14/15 del rpi5
        int baudrate_{1000000};                    // Baudrate
        int serial_fd_{-1};                       // File descriptor seriale
        std::mutex serial_mutex_;                 // Protezione accesso concorrente

     
        // 🔒 Mutex per la scrittura sulla seriale
        // Protegge accessi concorrenti al file descriptor durante le operazioni di write()
        std::mutex serialmutex;

        // 🧵 Thread di lettura
        std::thread readerthread;                 // Thread dedicato alla lettura dalla seriale
        std::atomic<bool> running_reader_{false}; // Flag di vita per il thread di lettura
        std::queue<std::string> rxqueue;          // Buffer FIFO dei pacchetti ricevuti
        std::mutex rxmutex;                       // Protegge la coda di ricezione
        std::condition_variable rxcv_;            // Notifica arrivo nuovo messaggio

        // 🧵 Thread di scrittura
        std::thread writerthread;                 // Thread dedicato alla scrittura sulla seriale
        std::atomic<bool> running_writer_{false}; // Flag di vita per il thread di scrittura
        std::queue<std::string> txqueue;          // Buffer FIFO dei comandi da inviare
        std::mutex txmutex;                       // Protegge la coda di trasmissione
        std::condition_variable wxcv_;            // Notifica quando c’è un nuovo comando da inviare

        /////////////////////////////////
        // enum dei sensori monitorati
        enum class SensorType {
            UART_GLOBAL,   // assenza totale di pacchetti UART
            ENC,           // encoder
            IMU,           // imu
            IRS,           // sensori IR
            SERVO,         // pan/tilt
            BAT,           // batteria
            EMR            // emergency stop
        };
        
        // timestamp ultimo aggiornamento per ogni sensore
        std::unordered_map<SensorType, rclcpp::Time> last_update_;
        
        // timeout per ogni sensore
        std::unordered_map<SensorType, double> timeout_sec_ = {
            {SensorType::UART_GLOBAL, 1.0},
            {SensorType::ENC,         0.5},
            {SensorType::IMU,         0.2},
            {SensorType::IRS,         0.5},
            {SensorType::SERVO,       0.5},
            {SensorType::BAT,         5.0},
            {SensorType::EMR,         2.0}
        };
        
        // flag anti‑spam per ogni sensore
        std::unordered_map<SensorType, bool> warned_;

        // variabili per errori di framing UART
        int framing_errors_ = 0;
        rclcpp::Time last_framing_error_{0,0,RCL_ROS_TIME};
        double framing_window_sec_ = 2.0;
        bool framing_warned_ = false;
        
        // variabili per controllo dt ros2_control
        double dt_warn_threshold_ = 0.050;   // 50 ms
        double dt_error_threshold_ = 0.200;  // 200 ms
        bool dt_warned_ = false;
        bool dt_error_warned_ = false;
        
        // dichiarazioni delle tre funzioni helper
        void update_sensor_timestamp(SensorType type, const rclcpp::Time& t);
        void register_framing_error(const rclcpp::Time& t);
        
        void check_sensor_timeouts(const rclcpp::Time& now);
        ///////////////////////

        // 🔌 Funzioni di supporto per la seriale
        bool open_serial();
        void close_serial();
        bool send_command_(const std::string &cmd);

        // 🔄 Thread di lettura/scrittura
        void readerloop(); // Funzione eseguita dal thread per leggere continuamente dalla seriale
        void writerloop(); // Funzione eseguita dal thread per scrivere continuamente dalla seriale

        // 📥 API non bloccante per ottenere un messaggio dalla coda
        std::optional<std::string> read_buffer_();

        // 🔄 Parsing dei pacchetti ricevuti
        void parse_encoder_packet_(const std::string &line);
        void parse_imu_packet_(const std::string &line);
    };

} // namespace mecanum_hardware
