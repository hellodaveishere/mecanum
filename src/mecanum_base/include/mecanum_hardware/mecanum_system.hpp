#pragma once

#include <string>
#include <vector>
#include <memory>
#include <optional>
#include <algorithm>
#include <mutex>

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

namespace mecanum_hardware
{

    // 📦 Stato per ciascuna ruota (posizione, velocità e comando)
    struct JointState
    {
        double pos_rad = 0.0;   // posizione letta dall’hardware
        double vel_rad_s = 0.0; // velocità letta dall’hardware
        double effort = 0.0;    // eventuale sforzo (se presente)
        double cmd_vel = 0.0;   // comando di velocità da ROS2 Control
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
        double pan_position = 0.0;  // Posizione del servo pan
        double tilt_position = 0.0; // Posizione del servo tilt
    };

    // 📦 Comandi desiderati per i servomotori (in radianti)
    struct ServoCommand
    {
        double pan_position = 0.0;  // Comando di posizione per il servo pan
        double tilt_position = 0.0; // Comando di posizione per il servo tilt
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
        // ⚙️ Parametri cinematici e logici
        double wheel_radius_{0.05}; // Raggio ruota [m]
        double L_{0.15};            // Metà lunghezza telaio [m]
        double W_{0.15};            // Metà larghezza telaio [m]
        bool mock_{false};          // true = simulazione, false = hardware reale
        double accel_limit_{25.0};  // Limite accelerazione [rad/s²] (solo mock)

        // ⚙️ Parametri encoder
        int ticks_per_rev_{2048};         // Tick per giro encoder (lato motore)
        double gear_ratio_{30.0};         // Rapporto di trasmissione
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

        // 🧪 Simulazione mock: dinamica di primo ordine
        void apply_mock_dynamics_(double dt);

        // 📡 Gestione seriale diretta
        std::string serial_port_{"/dev/ttyUSB0"}; // Porta seriale
        int baudrate_{115200};                    // Baudrate
        int serial_fd_{-1};                       // File descriptor seriale
        std::mutex serial_mutex_;                 // Protezione accesso concorrente

        // 🔌 Funzioni di supporto per la seriale
        bool open_serial();
        void close_serial();
        bool send_command_(const std::string &cmd);
        std::optional<std::string> read_line_();

        // 🔄 Parsing dei pacchetti ricevuti
        void parse_encoder_packet_(const std::string &line);
        void parse_imu_packet_(const std::string &line);
    };

} // namespace mecanum_hardware
