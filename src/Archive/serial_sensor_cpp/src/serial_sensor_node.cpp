// colcon build --packages-select serial_sensor_cpp
// source install/setup.bash
// ros2 run serial_sensor_cpp serial_sensor_node


#include <chrono>
#include <thread>
#include <string>
#include <vector>
#include <algorithm>
#include <cctype>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <diagnostic_updater/updater.hpp>
#include <diagnostic_updater/function.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <serial/serial.h>
#include <nlohmann/json.hpp>

using namespace std::chrono_literals;
using json = nlohmann::json;

class SerialSensorNode : public rclcpp::Node
{
public:
  SerialSensorNode()
  : Node("serial_sensor_node")
  {
    // Declare parameters
    declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    declare_parameter<int>("baud_rate", 115200);
    declare_parameter<double>("loop_hz", 10.0);
    declare_parameter<double>("serial_reset_timeout", 5.0);
    declare_parameter<std::string>("encoder_topic", "/encoder");
    declare_parameter<std::string>("imu_topic", "/imu");
    declare_parameter<std::string>("sonar_topic", "/sonar");
    declare_parameter<std::vector<std::string>>(
      "sonar_frames",
      std::vector<std::string>{"front", "right", "left"});
    declare_parameter<std::string>("encoder_frame_id", "wheel_encoder");
    declare_parameter<std::string>("imu_frame_id", "base_imu");

    // Fetch parameter values
    get_parameter("serial_port", port_);
    get_parameter("baud_rate", baud_rate_);
    get_parameter("loop_hz", loop_hz_);
    get_parameter("serial_reset_timeout", reset_timeout_);
    get_parameter("encoder_topic", encoder_topic_);
    get_parameter("imu_topic", imu_topic_);
    get_parameter("sonar_topic", sonar_topic_);
    get_parameter("sonar_frames", sonar_frames_);
    get_parameter("encoder_frame_id", encoder_frame_id_);
    get_parameter("imu_frame_id", imu_frame_id_);

    // Track last read time
    last_read_time_ = now();

    // Open serial port with retry every second
    wait_and_open_serial();

    // Publishers
    enc_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>(encoder_topic_, 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 10);

    for (auto & f : sonar_frames_) {
      sonar_pubs_.push_back(
        create_publisher<sensor_msgs::msg::Range>(sonar_topic_ + "/" + f, 10));
    }

    // Diagnostics
    updater_ = std::make_shared<diagnostic_updater::Updater>(this);
    updater_->setHardwareID("serial_sensor_node");
    updater_->add(
      diagnostic_updater::FunctionDiagnosticTask(
        "Serial Sensor Status",
        std::bind(&SerialSensorNode::diagnostics_callback, this,
                  std::placeholders::_1)));

    // Timer
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / loop_hz_),
      std::bind(&SerialSensorNode::timer_callback, this));
  }

private:
  void wait_and_open_serial()
  {
    while (rclcpp::ok()) {
      try {
        serial_.setPort(port_);
        serial_.setBaudrate((uint32_t)baud_rate_);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        serial_.setTimeout(to);
        serial_.open();
        RCLCPP_INFO(get_logger(),
          "Opened serial port %s @ %d bps",
          port_.c_str(), baud_rate_);
        return;
      } catch (const serial::IOException &) {
        RCLCPP_WARN(get_logger(),
          "Serial port %s not available; retrying in 1 second",
          port_.c_str());
        std::this_thread::sleep_for(1s);
      }
    }
  }

  bool diagnostics_callback(
    diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    auto now = this->now();
    double elapsed = (now - last_read_time_).seconds();

    if (elapsed > reset_timeout_) {
      stat.summary(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "No data; serial hung");
    } else {
      stat.summary(
        diagnostic_msgs::msg::DiagnosticStatus::OK,
        "Serial port healthy");
    }
    stat.add("seconds since last read", elapsed);
    return true;
  }

  void timer_callback()
  {
    auto now = this->now();
    double elapsed = (now - last_read_time_).seconds();

    if (elapsed > reset_timeout_) {
      RCLCPP_WARN(get_logger(), "Serial timeout; reopening port");
      try { if (serial_.isOpen()) serial_.close(); } catch (...) {}
      wait_and_open_serial();
      last_read_time_ = now;
      return;
    }

    // Read one full line
    std::string raw_line;
    try {
      raw_line = serial_.readline(65536, "\n");
    } catch (const serial::SerialException & e) {
      RCLCPP_ERROR(get_logger(), "Serial read error: %s", e.what());
      updater_->force_update();
      return;
    }

    if (raw_line.empty()) {
      return;
    }

    // Clean and trim
    replace_substrings(raw_line, "/n", "");
    trim(raw_line);
    if (raw_line.empty()) {
      return;
    }

    // Validate JSON
    if (raw_line.front() != '{' || raw_line.back() != '}') {
      RCLCPP_WARN(get_logger(),
        "Non-JSON received (skipping): '%s'",
        raw_line.c_str());
      return;
    }

    // Parse JSON
    json data;
    try {
      data = json::parse(raw_line);
      last_read_time_ = now;
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(),
        "JSON parse error: %s. Raw: '%s'",
        e.what(), raw_line.c_str());
      return;
    }

    // Publish encoder
    if (data.contains("enc")) {
      try {
        std_msgs::msg::Int32MultiArray msg;
        for (auto & v : data["enc"]) {
          msg.data.push_back(v.get<int>());
        }
        enc_pub_->publish(msg);
      } catch (...) {
        RCLCPP_WARN(get_logger(), "Bad encoder data");
      }
    }

    // Publish IMU
    if (data.contains("imu")) {
      try {
        auto imu_vals = data["imu"];
        sensor_msgs::msg::Imu msg;
        msg.header.stamp = now;
        msg.header.frame_id = imu_frame_id_;
        msg.linear_acceleration.x = imu_vals.at(0).get<double>();
        msg.linear_acceleration.y = imu_vals.at(1).get<double>();
        msg.linear_acceleration.z = imu_vals.at(2).get<double>();
        imu_pub_->publish(msg);
      } catch (...) {
        RCLCPP_WARN(get_logger(), "Bad IMU data");
      }
    }

    // Publish sonar
    if (data.contains("sonar")) {
      try {
        auto ranges = data["sonar"];
        for (size_t i = 0; i < ranges.size() && i < sonar_pubs_.size(); ++i) {
          sensor_msgs::msg::Range msg;
          msg.header.stamp = now;
          msg.header.frame_id = sonar_frames_[i];
          msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
          msg.field_of_view = 0.5f;
          msg.min_range = 0.02f;
          msg.max_range = 4.0f;
          msg.range = ranges.at(i).get<double>() / 1000.0;
          sonar_pubs_[i]->publish(msg);
        }
      } catch (...) {
        RCLCPP_WARN(get_logger(), "Bad sonar data");
      }
    }

    updater_->force_update();
  }

  static void trim(std::string & s)
  {
    s.erase(s.begin(),
      std::find_if(s.begin(), s.end(), [](unsigned char ch) {
        return !std::isspace(ch);
      }));
    s.erase(
      std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) {
        return !std::isspace(ch);
      }).base(),
      s.end());
  }

  static void replace_substrings(
    std::string & s,
    const std::string & old_sub,
    const std::string & new_sub)
  {
    size_t pos = 0;
    while ((pos = s.find(old_sub, pos)) != std::string::npos) {
      s.replace(pos, old_sub.length(), new_sub);
      pos += new_sub.length();
    }
  }

  // Parameters
  std::string port_;
  int baud_rate_;
  double loop_hz_;
  double reset_timeout_;
  std::string encoder_topic_;
  std::string imu_topic_;
  std::string sonar_topic_;
  std::vector<std::string> sonar_frames_;
  std::string encoder_frame_id_;
  std::string imu_frame_id_;

  // Serial
  serial::Serial serial_;

  // Last read
  rclcpp::Time last_read_time_;

  // ROS interfaces
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr enc_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr> sonar_pubs_;

  // Diagnostics
  std::shared_ptr<diagnostic_updater::Updater> updater_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialSensorNode>());
  rclcpp::shutdown();
  return 0;
}
