#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/range.hpp"

#include <serial/serial.h>
#include <nlohmann/json.hpp>
#include <vector>
#include <string>
#include <sstream>

class SerialJsonNode : public rclcpp::Node
{
public:
  SerialJsonNode();

private:
  // 🔧 Core logic
  void load_parameters();
  void readSerial();
  void writeSerial(const std_msgs::msg::String::SharedPtr msg);
  void sendWheelPower(const std::string &wheel_name, int value);
  void sendServoCommand(const std::string &servo_name, int value);

  // 🛠️ Template helpers
  template <typename T>
  void declare(const std::string &name, const T &default_value, T &output_variable)
  {
    // this->declare_parameter<T>(name, default_value);
    output_variable = this->get_parameter_or(name, default_value);
  }

  // 📦 Parameters
  std::string port_;
  int baudrate_;
  double rx_poll_time_;
  double reset_timeout_;
  std::mutex serial_mutex_;


  // ENCODER
  // Stato interno
  int wheel_power_front_right_ = 0;
  int wheel_power_front_left_ = 0;
  int wheel_power_back_right_ = 0;
  int wheel_power_back_left_ = 0;

  int servo_command_tilt_ = 0;
  int servo_command_pan_ = 0;

  // Strings containing the full topic name got from yaml file
  std::string encoder_topic_front_left_;
  std::string encoder_topic_front_right_;
  std::string encoder_topic_back_left_;
  std::string encoder_topic_back_right_;
  std::string encoder_frame_id_front_left_;
  std::string encoder_frame_id_front_right_;
  std::string encoder_frame_id_back_left_;
  std::string encoder_frame_id_back_right_;

  std::string imu_topic_;
  std::string imu_frame_id_;

  // SONAR
  std::string sonar_topic_right_;
  std::string sonar_topic_front_;
  std::string sonar_topic_left_;

  std::string sonar_frame_id_right_;
  std::string sonar_frame_id_front_;
  std::string sonar_frame_id_left_;

  int radiation_type_;
  double field_of_view_;
  double min_range_;
  double max_range_;


  std::string servo_topic_tilt_;
  std::string servo_topic_pan_;

  std::string wheel_topic_power_front_right_;
  std::string wheel_topic_power_front_left_;
  std::string wheel_topic_power_back_right_;
  std::string wheel_topic_power_back_left_;

  // 🔌 Serial port and buffer
  serial::Serial serial_port_;
  std::string buffer_;

  // 🔗 ROS interfaces
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr enc_back_left_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr enc_back_right_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr enc_front_left_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr enc_front_right_pub_;

  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr sonar_right_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr sonar_front_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr sonar_left_pub_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr wheel_power_front_right_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr wheel_power_front_left_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr wheel_power_back_right_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr wheel_power_back_left_sub_;

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr servo_command_tilt_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr servo_command_pan_sub_;

  rclcpp::CallbackGroup::SharedPtr rx_group_;
  rclcpp::CallbackGroup::SharedPtr tx_group_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  // std::vector<rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr> sonar_pubs_;
};
