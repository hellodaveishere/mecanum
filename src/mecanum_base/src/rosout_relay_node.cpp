#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/log.hpp"
#include "std_msgs/msg/string.hpp"

class RosoutRelay : public rclcpp::Node {
public:
  RosoutRelay() : Node("rosout_relay") {
    // Publisher su /rosout_string
    publisher_ = this->create_publisher<std_msgs::msg::String>("/rosout_string", 10);

    // Sottoscrizione al topic /rosout
    subscription_ = this->create_subscription<rcl_interfaces::msg::Log>(
      "/rosout",
      10,
      std::bind(&RosoutRelay::callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Nodo rosout_relay avviato");
  }

private:
  void callback(const rcl_interfaces::msg::Log::SharedPtr msg) {
    std::ostringstream oss;
    oss << "[" << msg->level << "] " << msg->name << ": " << msg->msg;

    auto out_msg = std_msgs::msg::String();
    out_msg.data = oss.str();

    publisher_->publish(out_msg);
    //RCLCPP_INFO(this->get_logger(), "Relay: %s", out_msg.data.c_str());
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RosoutRelay>());
  rclcpp::shutdown();
  return 0;
}
