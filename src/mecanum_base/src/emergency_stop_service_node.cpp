include "rclcpp/rclcpp.hpp"

include "std_srvs/srv/trigger.hpp"

include "mecanum_system.hpp"

class EstopServiceNode : public rclcpp::Node {
public:
  EstopServiceNode(std::sharedptr<mecanumhardware::MecanumSystem> hw_interface)
  : Node("estopservicenode"), hwinterface(hw_interface) {
    service = this->createservice<std_srvs::srv::Trigger>(
      "/clearemergencystop",
      std::bind(&EstopServiceNode::handleservice, this, std::placeholders::1, std::placeholders::_2)
    );
  }

private:
  void handle_service(
    const std::sharedptr<stdsrvs::srv::Trigger::Request> request,
    std::sharedptr<stdsrvs::srv::Trigger::Response> response) {
    hwinterface->clearEmergencyStop();  // Chiamata all’interfaccia pubblica
    response->success = true;
    response->message = "Emergency Stop disattivato.";
  }

  std::sharedptr<mecanumhardware::MecanumSystem> hwinterface;
  rclcpp::Service<stdsrvs::srv::Trigger>::SharedPtr service;
};