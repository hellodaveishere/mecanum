#pragma once
#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace mecanum_base {

class EstopStateBroadcaster : public controller_interface::ControllerInterface {
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
  int estop_state_index_{-1};
};

} // namespace mecanum_base
