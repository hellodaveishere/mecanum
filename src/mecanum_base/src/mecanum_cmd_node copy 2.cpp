#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <custom_interfaces/srv/calibration_input.hpp>  // Servizio personalizzato
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <cmath>
#include <algorithm>

class MecanumCmdNode : public rclcpp::Node
{
public:
  MecanumCmdNode() : Node("mecanum_cmd_node")
  {
    // Parametri cinematici
    r_ = declare_parameter<double>("wheel_radius", 0.05);
    L_ = declare_parameter<double>("L", 0.15);
    W_ = declare_parameter<double>("W", 0.15);
    max_vx_ = declare_parameter<double>("max_vx", 1.0);
    max_vy_ = declare_parameter<double>("max_vy", 1.0);
    max_wz_ = declare_parameter<double>("max_wz", 3.0);
    controller_topic_ = declare_parameter<std::string>("controller_cmd_topic", "/mecanum_velocity_controller/commands");

    // Fattori di correzione ruote (FL, FR, RL, RR)
    wheel_correction_ = declare_parameter<std::vector<double>>("wheel_correction", {1.0, 1.0, 1.0, 1.0});

    // Publisher comandi ruote
    pub_cmd_ = create_publisher<std_msgs::msg::Float64MultiArray>(controller_topic_, 10);

    // Publisher per inviare messaggi al frontend via rosbridge
    calib_pub_ = create_publisher<std_msgs::msg::String>("/calibration_status", 10);

    // Subscriber al comando di velocità
    sub_twist_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(10),
      std::bind(&MecanumCmdNode::twistCb, this, std::placeholders::_1));

    // Servizio per ricevere input di calibrazione dal frontend
    calib_srv_ = create_service<custom_interfaces::srv::CalibrationInput>(
      "/run_calibration_test",
      std::bind(&MecanumCmdNode::handleCalibrationRequest, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "✅ Nodo mecanum_cmd_node avviato.");
  }

private:
  // Funzione per calcolare la correzione contestuale
  std::vector<double> computeCorrectionVector(double vx, double vy, double wz, double ex, double ey, double ew)
  {
    const double a = L_ + W_;
    const double k = 1.0 / r_;

    // Matrice cinematicamente inversa (4x3)
    std::vector<std::vector<double>> M = {
      {1, -1, -a},
      {1,  1,  a},
      {1,  1, -a},
      {1, -1,  a}
    };

    // Pesi dinamici in base al tipo di movimento
    double total = std::abs(vx) + std::abs(vy) + std::abs(wz);
    double wx = total > 0 ? std::abs(vx) / total : 0;
    double wy = total > 0 ? std::abs(vy) / total : 0;
    double ww = total > 0 ? std::abs(wz) / total : 0;

    // Vettore di errore pesato
    std::vector<double> e = {
      wx * ex,
      wy * ey,
      ww * ew
    };

    // Calcolo correzione ruote
    std::vector<double> correction(4, 0.0);
    for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 3; ++j)
        correction[i] += M[i][j] * e[j];

    return correction;
  }

  // Callback per comando di velocità
  void twistCb(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    const double vx = std::clamp(msg->linear.x,  -max_vx_,  max_vx_);
    const double vy = std::clamp(msg->linear.y,  -max_vy_,  max_vy_);
    const double wz = std::clamp(msg->angular.z, -max_wz_,  max_wz_);

    const double k = 1.0 / r_;
    const double a = (L_ + W_);

    // Velocità nominali ruote
    double w_fl = k * ( vx - vy - a * wz );
    double w_fr = k * ( vx + vy + a * wz );
    double w_rl = k * ( vx + vy - a * wz );
    double w_rr = k * ( vx - vy + a * wz );

    // Nessuna correzione automatica in tempo reale (solo da test)
    std::vector<double> correction = {0.0, 0.0, 0.0, 0.0};

    // Applica correzione e fattori ruota
    std_msgs::msg::Float64MultiArray arr;
    arr.data = {
      (w_fl + correction[0]) * wheel_correction_[0],
      (w_fr + correction[1]) * wheel_correction_[1],
      (w_rl + correction[2]) * wheel_correction_[2],
      (w_rr + correction[3]) * wheel_correction_[3]
    };

    pub_cmd_->publish(arr);
  }

  // Salva il vettore di correzione in un file YAML
  void saveCorrectionToFile(const std::vector<double>& correction)
  {
    std::ofstream out("config/correction.yaml");
    if (out.is_open()) {
      out << "wheel_correction: ["
          << correction[0] << ", "
          << correction[1] << ", "
          << correction[2] << ", "
          << correction[3] << "]\n";
      out.close();
      RCLCPP_INFO(get_logger(), "💾 Correzione salvata in config/correction.yaml");
    } else {
      RCLCPP_WARN(get_logger(), "⚠️ Impossibile scrivere su config/correction.yaml");
    }
  }

  // Gestione del servizio di calibrazione
  void handleCalibrationRequest(
    const std::shared_ptr<custom_interfaces::srv::CalibrationInput::Request> req,
    std::shared_ptr<custom_interfaces::srv::CalibrationInput::Response> res)
  {
    std::string tipo = req->test_type;
    double ex = 0.0, ey = 0.0, ew = 0.0;

    if (tipo == "rettilineo" || tipo == "strafe") ew = req->error_value;
    else if (tipo == "rotazione") ey = req->error_value;

    double vx = (tipo == "rettilineo") ? 0.5 : 0.0;
    double vy = (tipo == "strafe") ? 0.5 : 0.0;
    double wz = (tipo == "rotazione") ? 1.0 : 0.0;

    auto correction = computeCorrectionVector(vx, vy, wz, ex, ey, ew);

    // Salva su file YAML
    saveCorrectionToFile(correction);

    // Costruisci messaggio per frontend
    std::stringstream ss;
    ss << "🧪 Test: " << tipo << "\n";
    ss << "Errore: ex=" << ex << ", ey=" << ey << ", ew=" << ew << "\n";
    ss << "Correzione ruote:\n";
    ss << "FL: " << correction[0] << "\n";
    ss << "FR: " << correction[1] << "\n";
    ss << "RL: " << correction[2] << "\n";
    ss << "RR: " << correction[3] << "\n";

    std_msgs::msg::String status;
    status.data = ss.str();
    calib_pub_->publish(status);

    res->result = ss.str();
  }

  // Membri ROS
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cmd_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr calib_pub_;
  rclcpp::Service<custom_interfaces::srv::CalibrationInput>::SharedPtr calib_srv_;

  // Parametri
  double r_, L_, W_;
  double max_vx_, max_vy_, max_wz_;
  std::string controller_topic_;
  std::vector<double> wheel_correction_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MecanumCmdNode>());
  rclcpp::shutdown();
  return 0;
}
