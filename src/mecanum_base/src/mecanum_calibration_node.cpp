#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <custom_interfaces/srv/calibration_input.hpp>

#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <numeric>
#include <cmath>
#include <algorithm>

class CalibrationNode : public rclcpp::Node
{
public:
  CalibrationNode() : Node("calibration_node")
  {
    // === Parametri robot ===
    r_ = declare_parameter<double>("wheel_radius", 0.05);
    L_ = declare_parameter<double>("L", 0.15);
    W_ = declare_parameter<double>("W", 0.15);
    controller_topic_ = declare_parameter<std::string>("controller_cmd_topic", "/mecanum_velocity_controller/commands");

    wheel_offset_ = {0.0, 0.0, 0.0, 0.0};
    last_wheel_velocity_ = {0.0, 0.0, 0.0, 0.0};

    loadCorrectionFromFile();

    // === Publisher e Subscriber ROS ===
    pub_cmd_ = create_publisher<std_msgs::msg::Float64MultiArray>(controller_topic_, 10);
    calib_pub_ = create_publisher<std_msgs::msg::String>("/calibration_status", 10);

    sub_joint_states_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&CalibrationNode::jointStatesCb, this, std::placeholders::_1));

    // === Servizio per avviare la calibrazione ===
    calib_srv_ = create_service<custom_interfaces::srv::CalibrationInput>(
      "/run_calibration_test",
      std::bind(&CalibrationNode::handleCalibrationRequest, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "✅ Nodo di calibrazione avviato.");
  }

private:
  enum class CalibrationMode { MIN_PWM, MAX_PWM, MAP_PWM };

  // === Callback per joint_states ===
  void jointStatesCb(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::map<std::string, double> joint_vel_map;
    for (size_t i = 0; i < msg->name.size(); ++i)
      joint_vel_map[msg->name[i]] = msg->velocity[i];

    last_wheel_velocity_ = {
      joint_vel_map["wheel_fl_joint"],
      joint_vel_map["wheel_fr_joint"],
      joint_vel_map["wheel_rl_joint"],
      joint_vel_map["wheel_rr_joint"]
    };
  }

  // === Funzione per eseguire test di calibrazione PWM ===
  void runCalibrationTest(CalibrationMode mode, double step, std::vector<double>& result)
  {
    result.clear();
    std::ostringstream log;
    log << "🧪 Avvio test: ";
    if (mode == CalibrationMode::MIN_PWM) log << "Impulso minimo\n";
    else if (mode == CalibrationMode::MAX_PWM) log << "Impulso massimo\n";
    else log << "Mappatura impulso-velocità\n";

    calib_pub_->publish(std_msgs::msg::String().set__data(log.str()));

    for (int pwm = 50; pwm <= 255; pwm += static_cast<int>(step)) {
      std_msgs::msg::Float64MultiArray cmd;
      cmd.data = {pwm, pwm, pwm, pwm};
      pub_cmd_->publish(cmd);
      rclcpp::sleep_for(std::chrono::milliseconds(300));

      double avg = std::accumulate(last_wheel_velocity_.begin(), last_wheel_velocity_.end(), 0.0) / 4.0;
      result.push_back(avg);

      std::ostringstream step_log;
      step_log << "PWM: " << pwm << " → Velocità media: " << avg << " rad/s\n";
      calib_pub_->publish(std_msgs::msg::String().set__data(step_log.str()));
    }

    calib_pub_->publish(std_msgs::msg::String().set__data("✅ Test completato.\n"));
  }

  // === Funzione per calcolare vettore di correzione cinematico ===
  std::vector<double> computeCorrectionVector(double vx, double vy, double wz, double ex, double ey, double ew)
  {
    const double a = L_ + W_;
    std::vector<std::vector<double>> M = {
      {1, -1, -a},
      {1,  1,  a},
      {1,  1, -a},
      {1, -1,  a}
    };

    double total = std::abs(vx) + std::abs(vy) + std::abs(wz);
    double wx = total > 0 ? std::abs(vx) / total : 0;
    double wy = total > 0 ? std::abs(vy) / total : 0;
    double ww = total > 0 ? std::abs(wz) / total : 0;

    std::vector<double> e = { wx * ex, wy * ey, ww * ew };
    std::vector<double> correction(4, 0.0);
    for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 3; ++j)
        correction[i] += M[i][j] * e[j];

    return correction;
  }

  // === Servizio per gestire richiesta di calibrazione ===
  void handleCalibrationRequest(
    const std::shared_ptr<custom_interfaces::srv::CalibrationInput::Request> req,
    std::shared_ptr<custom_interfaces::srv::CalibrationInput::Response> res)
  {
    std::string tipo = req->test_type;
    double valore = req->error_value;
    std::vector<double> result;
    CalibrationMode mode;

    if (tipo == "min_pwm") {
      mode = CalibrationMode::MIN_PWM;
    } else if (tipo == "max_pwm") {
      mode = CalibrationMode::MAX_PWM;
    } else if (tipo == "map_pwm") {
      mode = CalibrationMode::MAP_PWM;
    } else if (tipo == "rettilineo" || tipo == "strafe" || tipo == "rotazione") {
      double ex = 0.0, ey = 0.0, ew = 0.0;

      if (tipo == "rettilineo") ex = valore;
      else if (tipo == "strafe") ey = valore;
      else if (tipo == "rotazione") ew = valore;

      double vx = (tipo == "rettilineo") ? 0.5 : 0.0;
      double vy = (tipo == "strafe") ? 0.5 : 0.0;
      double wz = (tipo == "rotazione") ? 1.0 : 0.0;

      auto correction = computeCorrectionVector(vx, vy, wz, ex, ey, ew);
      saveCorrectionToFile(correction);
      wheel_offset_ = correction;

      std::stringstream ss;
      ss << "🧪 Test cinematico: " << tipo << "\n";
      ss << "Errore: ex=" << ex << ", ey=" << ey << ", ew=" << ew << "\n";
      ss << "Correzione ruote:\n";
      ss << "FL: " << correction[0] << "\n";
      ss << "FR: " << correction[1] << "\n";
      ss << "RL: " << correction[2] << "\n";
      ss << "RR: " << correction[3] << "\n";

      calib_pub_->publish(std_msgs::msg::String().set__data(ss.str()));
      res->result = ss.str();
      return;
    } else {
      res->result = "❌ Tipo di test non riconosciuto.";
      calib_pub_->publish(std_msgs::msg::String().set__data(res->result));
      return;
    }

    runCalibrationTest(mode, valore, result);

    std::stringstream ss;
    ss << "📊 Risultati test " << tipo << ":\n";
    for (size_t i = 0; i < result.size(); ++i)
      ss << "Step " << i << ": " << result[i] << " rad/s\n";

    calib_pub_->publish(std_msgs::msg::String().set__data(ss.str()));
    res->result = ss.str();
  }

  // === Salvataggio correzione su file ===
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
RCLCPP_INFO(get_logger(), "💾 Correzione salvata in config/correction.yaml");
    } else {
      RCLCPP_WARN(get_logger(), "⚠️ Impossibile scrivere su config/correction.yaml");
    }
  }

  void loadCorrectionFromFile()
  {
    std::ifstream in("config/correction.yaml");
    if (in.is_open()) {
      std::string line;
      while (std::getline(in, line)) {
        if (line.find("wheel_correction") != std::string::npos) {
          size_t start = line.find("[");
          size_t end = line.find("]");
          if (start != std::string::npos && end != std::string::npos && end > start) {
            std::string values = line.substr(start + 1, end - start - 1);
            std::stringstream ss(values);
            std::string val;
            int i = 0;
            while (std::getline(ss, val, ',') && i < 4) {
              try {
                wheel_offset_[i] = std::stod(val);
              } catch (...) {
                wheel_offset_[i] = 0.0;
              }
              ++i;
            }
            while (i < 4) wheel_offset_[i++] = 0.0;
            RCLCPP_INFO(get_logger(), "📂 Correzione caricata da config/correction.yaml");
            return;
          }
        }
      }
      in.close();
      RCLCPP_WARN(get_logger(), "⚠️ File YAML trovato ma formato non valido. Correzioni impostate a zero.");
    } else {
      RCLCPP_WARN(get_logger(), "⚠️ File config/correction.yaml non trovato. Correzioni impostate a zero.");
    }

    wheel_offset_ = {0.0, 0.0, 0.0, 0.0};
  }

  // === Membri ROS ===
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cmd_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr calib_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
  rclcpp::Service<custom_interfaces::srv::CalibrationInput>::SharedPtr calib_srv_;

  // === Parametri robot ===
  double r_, L_, W_;
  std::string controller_topic_;
  std::vector<double> wheel_offset_;
  std::vector<double> last_wheel_velocity_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CalibrationNode>());
  rclcpp::shutdown();
  return 0;
}