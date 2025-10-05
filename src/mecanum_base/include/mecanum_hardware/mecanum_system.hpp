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
        double pos_rad = 0.0;
        double vel_rad_s = 0.0;
        double effort = 0.0;
        double cmd_vel = 0.0;
    };

    // 📦 Stato IMU (orientamento, velocità angolare, accelerazione lineare)
    struct ImuState
    {
        double orientation[4]{0.0, 0.0, 0.0, 1.0};
        double angular_vel[3]{0.0, 0.0, 0.0};
        double linear_accel[3]{0.0, 0.0, 0.0};
    };

    // 📦 Stato dei servomotori pan/tilt
    struct ServoState
    {
        double position = 0.0;  // Posizione attuale [rad]
        double command = 0.0;   // Comando da ROS 2 Control [rad]
    };

    // 📦 Stato dei sonar HC-SR04
    struct SonarState
    {
        double range_m = 0.0;   // Distanza misurata [m]
    };

    // 🔧 Classe principale del sistema hardware Mecanum
    class MecanumSystem final : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(MecanumSystem)

        // 🔄 Inizializzazione del sistema hardware
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

        // 📤 Esportazione delle interfacce di stato (ruote, imu, servo, sonar)
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        // 📥 Esportazione delle interfacce di comando (ruote, servo)
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        // ⚡ Attivazione del sistema
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &prev_state) override;

        // 💤 Disattivazione del sistema
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &prev_state) override;

        // 📡 Lettura da hardware (encoder, imu, servo, sonar)
        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        // 📝 Scrittura su hardware (comandi ruote, servo)
        hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        // ℹ️ Informazioni hardware (popolate da URDF/XACRO)
        hardware_interface::HardwareInfo info_;

    private:
        // ⚙️ Parametri cinematici e logici
        double wheel_radius_{0.05};
        double L_{0.15};
        double W_{0.15};
        bool mock_{false};
        double accel_limit_{25.0};

        // ⚙️ Parametri encoder
        int ticks_per_rev_{2048};
        double gear_ratio_{30.0};
        double ticks_per_wheel_rev_{0.0};

        // 🔁 Inversione direzione ruote
        int inv_fl_{1}, inv_fr_{1}, inv_rl_{1}, inv_rr_{1};

        // 🔩 Stato dei giunti ruota
        std::vector<std::string> joint_names_;
        std::vector<JointState> joints_;

        // 📦 Stato IMU
        ImuState imu_state_;

        // 🔩 Stato dei servomotori pan/tilt
        std::vector<std::string> servo_names_;   // es. ["pan_joint", "tilt_joint"]
        std::vector<ServoState> servos_;

        // 📡 Stato dei sonar HC-SR04
        std::vector<std::string> sonar_names_;   // es. ["sonar_front", "sonar_left", "sonar_right"]
        std::vector<SonarState> sonars_;

        // 🧪 Simulazione mock
        void apply_mock_dynamics_(double dt);

        // 📡 Gestione seriale diretta
        std::string serial_port_{"/dev/ttyUSB0"};
        int baudrate_{115200};
        int serial_fd_{-1};
        std::mutex serial_mutex_;

        // 🔌 Funzioni di supporto per la seriale
        bool open_serial();
        void close_serial();
        bool send_command_(const std::string &cmd);
        std::optional<std::string> read_line_();

        // 🔄 Parsing dei pacchetti ricevuti
        void parse_encoder_packet_(const std::string &line);
        void parse_imu_packet_(const std::string &line);
        void parse_servo_packet_(const std::string &line);  // ← da implementare
        void parse_sonar_packet_(const std::string &line);  // ← da implementare
    };

} // namespace mecanum_hardware