#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <algorithm>

/*
  Nodo comando:
  - Input: /cmd_vel (vx, vy, wz)
  - Output: <controller>/commands (Float64MultiArray): [FL, FR, RL, RR] [rad/s]
  - Nota: il controller target è JointGroupVelocityController
*/
class MecanumCmdNode : public rclcpp::Node
{
public:
  MecanumCmdNode() : Node("mecanum_cmd_node")
  {
    r_ = declare_parameter<double>("wheel_radius", 0.05);
    L_ = declare_parameter<double>("L", 0.15);
    W_ = declare_parameter<double>("W", 0.15);
    max_vx_ = declare_parameter<double>("max_vx", 1.0);
    max_vy_ = declare_parameter<double>("max_vy", 1.0);
    max_wz_ = declare_parameter<double>("max_wz", 3.0);
    controller_topic_ = declare_parameter<std::string>("controller_cmd_topic", "/mecanum_velocity_controller/commands");

    pub_cmd_ = create_publisher<std_msgs::msg::Float64MultiArray>(controller_topic_, 10);
    sub_twist_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(10),
      std::bind(&MecanumCmdNode::twistCb, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "mecanum_cmd_node: pubblicazione comandi su %s", controller_topic_.c_str());
  }

private:
  void twistCb(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    const double vx = std::clamp(msg->linear.x,  -max_vx_,  max_vx_);
    const double vy = std::clamp(msg->linear.y,  -max_vy_,  max_vy_);
    const double wz = std::clamp(msg->angular.z, -max_wz_,  max_wz_);

    const double k = 1.0 / r_;
    const double a = (L_ + W_);

    // Ordine: FL, FR, RL, RR
    const double w_fl = k * ( vx - vy - a * wz );
    const double w_fr = k * ( vx + vy + a * wz );
    const double w_rl = k * ( vx + vy - a * wz );
    const double w_rr = k * ( vx - vy + a * wz );

    std_msgs::msg::Float64MultiArray arr;
    arr.data = { w_fl, w_fr, w_rl, w_rr };
    pub_cmd_->publish(arr);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cmd_;

  double r_, L_, W_;
  double max_vx_, max_vy_, max_wz_;
  std::string controller_topic_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MecanumCmdNode>());
  rclcpp::shutdown();
  return 0;
}
