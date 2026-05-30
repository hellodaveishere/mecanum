#include <string>
#include <vector>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "mecanum_base/msg/system_state.hpp"

namespace mecanum_base
{

// ============================================================================
//  SystemStateBroadcaster
//  --------------------------------------------------------------------------
//  Broadcaster ros2_control che:
//    - legge una serie di state interfaces esposte dalla HW interface
//    - costruisce un messaggio mecanum_base::msg::SystemState
//    - lo pubblica sul topic /robot/system_state
//
//  NOTA IMPORTANTE:
//    La HW interface deve esporre state interfaces con nomi coerenti, ad es.:
//      system_state/robot_status
//      system_state/motor_fl_status
//      system_state/motor_fr_status
//      system_state/motor_rl_status
//      system_state/motor_rr_status
//      system_state/encoder_fl_status
//      ...
// ============================================================================
class SystemStateBroadcaster : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override
  {
    // Nessun parametro obbligatorio per ora.
    // Potresti aggiungere in futuro un prefisso configurabile.
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    // Broadcaster: non usa command interfaces
    return {
      controller_interface::interface_configuration_type::NONE,
      {}
    };
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    // Qui dichiariamo quali state interfaces vogliamo leggere dalla HW interface.
    // I nomi devono combaciare con quelli esportati da MecanumSystem::export_state_interfaces().
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    // Robot status
    config.names.push_back("system_state/robot_status");

    // Motori
    config.names.push_back("system_state/motor_fl_status");
    config.names.push_back("system_state/motor_fr_status");
    config.names.push_back("system_state/motor_rl_status");
    config.names.push_back("system_state/motor_rr_status");

    // Encoder
    config.names.push_back("system_state/encoder_fl_status");
    config.names.push_back("system_state/encoder_fr_status");
    config.names.push_back("system_state/encoder_rl_status");
    config.names.push_back("system_state/encoder_rr_status");

    // Servo
    config.names.push_back("system_state/servo_pan_status");
    config.names.push_back("system_state/servo_tilt_status");

    // IR
    config.names.push_back("system_state/ir_fl_status");
    config.names.push_back("system_state/ir_fc_status");
    config.names.push_back("system_state/ir_fr_status");

    // Vacuum, IMU, Battery
    config.names.push_back("system_state/vacuum_status");
    config.names.push_back("system_state/imu_status");
    config.names.push_back("system_state/battery_status");

    return config;
  }

  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    // Publisher creato sul nodo interno del controller (QUI è corretto avere un nodo)
    system_state_pub_ =
      get_node()->create_publisher<mecanum_base::msg::SystemState>(
        "/robot/system_state",
        rclcpp::SystemDefaultsQoS());

    if (!system_state_pub_) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "SystemStateBroadcaster: impossibile creare il publisher /robot/system_state");
      return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    // Mappiamo i LoanedStateInterface in una tabella nome → puntatore
    state_map_.clear();
    for (auto & si : state_interfaces_) {
      state_map_[si.get_name()] = &si;
    }

    // Controllo di consistenza: tutti i nomi richiesti devono esistere
    auto check = [&](const std::string &name) {
      if (state_map_.find(name) == state_map_.end()) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "SystemStateBroadcaster: state interface mancante: %s",
                     name.c_str());
        return false;
      }
      return true;
    };

    bool ok = true;
    ok &= check("system_state/robot_status");

    ok &= check("system_state/motor_fl_status");
    ok &= check("system_state/motor_fr_status");
    ok &= check("system_state/motor_rl_status");
    ok &= check("system_state/motor_rr_status");

    ok &= check("system_state/encoder_fl_status");
    ok &= check("system_state/encoder_fr_status");
    ok &= check("system_state/encoder_rl_status");
    ok &= check("system_state/encoder_rr_status");

    ok &= check("system_state/servo_pan_status");
    ok &= check("system_state/servo_tilt_status");

    ok &= check("system_state/ir_fl_status");
    ok &= check("system_state/ir_fc_status");
    ok &= check("system_state/ir_fr_status");

    ok &= check("system_state/vacuum_status");
    ok &= check("system_state/imu_status");
    ok &= check("system_state/battery_status");

    if (!ok) {
      return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/) override
  {
    state_map_.clear();
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update(
      const rclcpp::Time & /*time*/,
      const rclcpp::Duration & /*period*/) override
  {
    if (!system_state_pub_) {
      return controller_interface::return_type::OK;
    }

    mecanum_base::msg::SystemState msg;

    // Helper per leggere un intero da una state interface
    auto get_int = [&](const std::string &name) -> int32_t {
      auto it = state_map_.find(name);
      if (it == state_map_.end()) {
        return 0;
      }
      return static_cast<int32_t>(it->second->get_value());
    };

    msg.robot_status = get_int("system_state/robot_status");

    msg.motor_status[0] = get_int("system_state/motor_fl_status");
    msg.motor_status[1] = get_int("system_state/motor_fr_status");
    msg.motor_status[2] = get_int("system_state/motor_rl_status");
    msg.motor_status[3] = get_int("system_state/motor_rr_status");

    msg.encoder_status[0] = get_int("system_state/encoder_fl_status");
    msg.encoder_status[1] = get_int("system_state/encoder_fr_status");
    msg.encoder_status[2] = get_int("system_state/encoder_rl_status");
    msg.encoder_status[3] = get_int("system_state/encoder_rr_status");

    msg.servo_status[0] = get_int("system_state/servo_pan_status");
    msg.servo_status[1] = get_int("system_state/servo_tilt_status");

    msg.ir_status[0] = get_int("system_state/ir_fl_status");
    msg.ir_status[1] = get_int("system_state/ir_fc_status");
    msg.ir_status[2] = get_int("system_state/ir_fr_status");

    msg.vacuum_status  = get_int("system_state/vacuum_status");
    msg.imu_status     = get_int("system_state/imu_status");
    msg.battery_status = get_int("system_state/battery_status");

    system_state_pub_->publish(msg);

    return controller_interface::return_type::OK;
  }

private:
  rclcpp::Publisher<mecanum_base::msg::SystemState>::SharedPtr system_state_pub_;

  // Mappa nome → state interface
  std::unordered_map<std::string, hardware_interface::LoanedStateInterface*> state_map_;
};

}  // namespace mecanum_base

// Registrazione del plugin per pluginlib
PLUGINLIB_EXPORT_CLASS(
  mecanum_base::SystemStateBroadcaster,
  controller_interface::ControllerInterface)
