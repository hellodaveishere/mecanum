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

    // wheel_offset_ = {0.0, 0.0, 0.0, 0.0 };
    // loadCorrectionFromFile();
    wheel_offset_ = loadCorrectionFromFile();

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

    // Non cosidera wheel_offset in caso di stop
    auto applyOffset = [](double w, double offset)
    {
      return std::abs(w) > 1e-3 ? w + offset : 0.0;
    };

    arr.data = {
        applyOffset(w_fl, wheel_offset_[0]) * wheel_correction_[0],
        applyOffset(w_fr, wheel_offset_[1]) * wheel_correction_[1],
        applyOffset(w_rl, wheel_offset_[2]) * wheel_correction_[2],
        applyOffset(w_rr, wheel_offset_[3]) * wheel_correction_[3]};

    pub_cmd_->publish(arr);
  }

  std::vector<double> loadCorrectionFromFile()
  {
    // 🔧 Tipi di test cinematici da combinare
    const std::vector<std::string> types = {"rettilineo", "strafe", "rotazione"};

    // ⚖️ Pesi assegnati a ciascun test (equilibrati)
    const std::vector<double> weights = {0.33, 0.33, 0.34};

    // 🧮 Vettori parziali per ciascun test
    std::vector<std::vector<double>> partials(3, std::vector<double>(4, 0.0));
    bool loaded_any = false;

    // 📂 Carica ciascun file YAML separato
    for (size_t t = 0; t < types.size(); ++t)
    {
      std::string filepath = "config/correction_" + types[t] + ".yaml";
      std::ifstream in(filepath);

      if (in.is_open())
      {
        std::string line;
        while (std::getline(in, line))
        {
          if (line.find("correction_" + types[t]) != std::string::npos)
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
                  partials[t][i] = std::stod(val);
                }
                catch (...)
                {
                  partials[t][i] = 0.0;
                }
                ++i;
              }

              loaded_any = true;
              RCLCPP_INFO(get_logger(), "📂 Correzione '%s' caricata da %s", types[t].c_str(), filepath.c_str());
              break;
            }
          }
        }
        in.close();
      }
      else
      {
        RCLCPP_WARN(get_logger(), "⚠️ File %s non trovato. Correzione '%s' impostata a zero.", filepath.c_str(), types[t].c_str());
      }
    }

    // ➕ Somma pesata dei tre vettori per ottenere la correzione finale
    std::vector<double> wheel_offset(4, 0.0);
    for (int i = 0; i < 4; ++i)
      wheel_offset[i] = weights[0] * partials[0][i] + weights[1] * partials[1][i] + weights[2] * partials[2][i];

    // ✅ Log finale
    if (loaded_any)
    {
      RCLCPP_INFO(get_logger(), "✅ Correzione totale combinata con pesi: rettilineo=%.2f, strafe=%.2f, rotazione=%.2f",
                  weights[0], weights[1], weights[2]);

      RCLCPP_INFO(get_logger(), "🔎 Offset ruote finali:");
      RCLCPP_INFO(get_logger(), "FL: %.4f", wheel_offset[0]);
      RCLCPP_INFO(get_logger(), "FR: %.4f", wheel_offset[1]);
      RCLCPP_INFO(get_logger(), "RL: %.4f", wheel_offset[2]);
      RCLCPP_INFO(get_logger(), "RR: %.4f", wheel_offset[3]);
    }
    else
    {
      RCLCPP_WARN(get_logger(), "⚠️ Nessuna correzione caricata. Tutti gli offset impostati a zero.");
    }

    return wheel_offset;
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
