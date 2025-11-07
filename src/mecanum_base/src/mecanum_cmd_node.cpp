#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <algorithm>
#include <fstream>

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

    wheel_correction_ = declare_parameter<std::vector<double>>("wheel_correction", {1.0, 1.0, 1.0, 1.0});
    wheel_offset_ = {0.0, 0.0, 0.0, 0.0};

    loadCorrectionFromFile();

    pub_cmd_ = create_publisher<std_msgs::msg::Float64MultiArray>(controller_topic_, 10);
    sub_twist_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::QoS(10),
        std::bind(&MecanumCmdNode::twistCb, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "mecanum_cmd_node: pubblicazione comandi su %s", controller_topic_.c_str());
  }

private:
  void twistCb(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    const double vx = std::clamp(msg->linear.x, -max_vx_, max_vx_);
    const double vy = std::clamp(msg->linear.y, -max_vy_, max_vy_);
    const double wz = std::clamp(msg->angular.z, -max_wz_, max_wz_);

    const double k = 1.0 / r_;
    const double a = L_ + W_;

    double w_fl = k * (vx - vy - a * wz);
    double w_fr = k * (vx + vy + a * wz);
    double w_rl = k * (vx + vy - a * wz);
    double w_rr = k * (vx - vy + a * wz);

    std_msgs::msg::Float64MultiArray arr;
    arr.data = {
        (w_fl + wheel_offset_[0]) * wheel_correction_[0],
        (w_fr + wheel_offset_[1]) * wheel_correction_[1],
        (w_rl + wheel_offset_[2]) * wheel_correction_[2],
        (w_rr + wheel_offset_[3]) * wheel_correction_[3]};

    pub_cmd_->publish(arr);
  }

  void loadCorrectionFromFile()
  {
    std::ifstream in("config/correction.yaml");
    if (in.is_open())
    {
      std::string line;
      while (std::getline(in, line))
      {
        if (line.find("wheel_correction") != std::string::npos)
        {
          size_t start = line.find("[");
          size_t end = line.find("]");
          if (start != std::string::npos && end != std::string::npos && end > start)
          {
            std::string values = line.substr(start + 1, end - start - 1);
            std::stringstream ss(values);
            std::string val;
            int i = 0;
            while (std::getline(ss, val, ',') && i < 4)
            {
              try
              {
                wheel_offset_[i] = std::stod(val);
              }
              catch (...)
              {
                wheel_offset_[i] = 0.0;
              }
              ++i;
            }
            while (i < 4)
              wheel_offset_[i++] = 0.0;
            RCLCPP_INFO(get_logger(), "📂 Correzione caricata da config/correction.yaml");
            return;
          }
        }
      }
      in.close();
      RCLCPP_WARN(get_logger(), "⚠️ File YAML trovato ma formato non valido. Correzioni impostate a zero.");
    }
    else
    {
      RCLCPP_WARN(get_logger(), "⚠️ File config/correction.yaml non trovato. Correzioni impostate a zero.");
    }

    wheel_offset_ = {0.0, 0.0, 0.0, 0.0};
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cmd_;

  double r_, L_, W_;
  double max_vx_, max_vy_, max_wz_;
  std::string controller_topic_;
  std::vector<double> wheel_correction_;
  std::vector<double> wheel_offset_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MecanumCmdNode>());
  rclcpp::shutdown();
  return 0;
}
