// DEVCONTAINER`
// ============
// Per ricaricarlo: CNTR+SHIFT+P -> devcontainer: reopen in container (reopen folder locally) oppure rebuild container
// In .devcontainer trovi Dockerfile e devcontainer.json se vuoi fare modifiche

// COMPILAZIONE
// ============
// rm -rf build/ install/ log/
// colcon build --packages-select serial_json_node --symlink-install
// ros2 run serial_json_node serial_json_node

// USO
// ===
// 1) LANCIARE NODO
//    =============
//    Per lanciare serial_json_node:
//    source install/setup.bash
//    ros2 launch serial_json_node serial_json.launch.py

// 2) INVIARE COMANDI AI MOTORI/SERVO
//    ===============================
// 2a) Per inviare comandi al pico usa topic /pico_tx
//     ros2 topic pub --once /pico_tx std_msgs/msg/String "{data: '{\"velocity\": {\"front_right\": 0.7, \"front_left\": 0.7, \"back_right\": 0.7, \"back_left\": 0.7}}'}"

//     Note: wheel velocity command to send to Pico in rad/sec {"velocity": {"front_right": 8.0, "front_left": 7.5, "back_right": 7.8, "back_left": 7.6}} received by topic cmd_vel_raw
//     TODO node to map cmd_vel into cmd_vel_raw

// 2b) Ci sono anche i topic come /bot/wheel/front_right/power, ma non sono riconosciuti dal pico
//     Per usarli:
//     ros2 topic pub --once /bot/wheel/front_right/power std_msgs/msg/Int32 "{data: 1}"

// 3) RICEVERE DATI DA SENSORI
//    ========================
//    ros2 topic echo /pico_rx

//    Note:
//    encoder velocity in rad/sec {"encod_velocity": {"front_right": 8.0, "front_left": 7.5, "back_right": 7.8, "back_left": 7.6}} to send to /enc_raw
//    TODO node to map /enc_raw into /bot/odom converting encoder velocity data into odom message. Use odometry.cpp from yahboom project particularly the function   updateFromVelocity(...)


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <serial/serial.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

#include "serial_json_node/serial_json_node.hpp"

#include <sstream>
#include <chrono>
#include <filesystem>

SerialJsonNode::SerialJsonNode()
    : Node("serial_json_node", rclcpp::NodeOptions()
                                   .allow_undeclared_parameters(true)
                                   .automatically_declare_parameters_from_overrides(true))
{
  using namespace std::chrono_literals;

  load_parameters();

  // Create callback groups for RX and TX
  rx_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  tx_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // Configure serial port
  serial_port_.setPort(port_);
  serial_port_.setBaudrate(baudrate_);
  serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
  serial_port_.setTimeout(timeout); // TODO get from yaml

  using namespace std::chrono_literals;
  namespace fs = std::filesystem;

  // Wait until the serial port device file exists
  while (rclcpp::ok())
  {
    if (fs::exists(port_))
    {
      try
      {
        serial_port_.open();
        RCLCPP_INFO(this->get_logger(),
                    "Opened serial port %s at %d baud",
                    port_.c_str(), baudrate_);
        break; // Exit loop once successfully opened
      }
      catch (const serial::IOException &e)
      {
        RCLCPP_WARN(this->get_logger(),
                    "Serial port %s exists but failed to open: %s. Retrying in 1s...",
                    port_.c_str(), e.what());
      }
    }
    else
    {
      RCLCPP_WARN(this->get_logger(),
                  "Serial port %s not available, retrying in 1s...", port_.c_str());
    }

    rclcpp::sleep_for(1s);
  }

  if (!rclcpp::ok())
  {
    RCLCPP_ERROR(this->get_logger(), "Interrupted before opening the serial port.");
    return;
  }

  // Publisher for incoming JSON messages
  publisher_ = this->create_publisher<std_msgs::msg::String>("pico_rx", 10);

  enc_back_left_pub_ = this->create_publisher<std_msgs::msg::Int32>(encoder_topic_back_left_, 10);
  enc_back_right_pub_ = this->create_publisher<std_msgs::msg::Int32>(encoder_topic_back_right_, 10);
  enc_front_left_pub_ = this->create_publisher<std_msgs::msg::Int32>(encoder_topic_front_left_, 10);
  enc_front_right_pub_ = this->create_publisher<std_msgs::msg::Int32>(encoder_topic_front_right_, 10);

  sonar_right_pub_ = this->create_publisher<sensor_msgs::msg::Range>(sonar_topic_right_, 10);
  sonar_front_pub_ = this->create_publisher<sensor_msgs::msg::Range>(sonar_topic_front_, 10);
  sonar_left_pub_ = this->create_publisher<sensor_msgs::msg::Range>(sonar_topic_left_, 10);

  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 10);

  // Why Use callback_group?
  // By default, all callbacks go into a single group and are executed
  // sequentially.
  // If you want parallel execution (e.g. one thread for receiving
  // messages, another for publishing), you can create multiple callback
  // groups and assign them to different executors or threads.

  // Subscriber for outgoing messages from node to rpi pico
  rclcpp::SubscriptionOptions tx_options;
  tx_options.callback_group = tx_group_;


  // == SUBSCRIPTION ==
  subscription_ = this->create_subscription<std_msgs::msg::String>(
      "pico_tx", 10,
      std::bind(&SerialJsonNode::writeSerial, this, std::placeholders::_1),
      tx_options);

  using std_msgs::msg::Int32;
  // Front right wheel
  wheel_power_front_right_sub_ = this->create_subscription<Int32>(
      wheel_topic_power_front_right_, 10,
      [this](const Int32::SharedPtr msg)
      {
        wheel_power_front_right_ = msg->data;
        sendWheelPower("front_right", msg->data);
      },
      tx_options);

  // Front left wheel
  wheel_power_front_left_sub_ = this->create_subscription<Int32>(
      wheel_topic_power_front_left_, 10,
      [this](const Int32::SharedPtr msg)
      {
        wheel_power_front_left_ = msg->data;
        sendWheelPower("front_left", msg->data);
      },
      tx_options);

  // Back right wheel
  wheel_power_back_right_sub_ = this->create_subscription<Int32>(
      wheel_topic_power_back_right_, 10,
      [this](const Int32::SharedPtr msg)
      {
        wheel_power_back_right_ = msg->data;
        sendWheelPower("back_right", msg->data);
      },
      tx_options);

  // Back left wheel
  wheel_power_back_left_sub_ = this->create_subscription<Int32>(
      wheel_topic_power_back_left_, 10,
      [this](const Int32::SharedPtr msg)
      {
        wheel_power_back_left_ = msg->data;
        sendWheelPower("back_left", msg->data);
      },
      tx_options);

  // Servo Tilt
  servo_command_tilt_sub_ = this->create_subscription<Int32>(
      servo_topic_tilt_, 10,
      [this](const Int32::SharedPtr msg)
      {
        servo_command_tilt_ = msg->data;
        sendServoCommand("tilt", msg->data);
      },
      tx_options);

  // Servo Pan
  servo_command_pan_sub_ = this->create_subscription<Int32>(
      servo_topic_pan_, 10,
      [this](const Int32::SharedPtr msg)
      {
        servo_command_pan_ = msg->data;
        sendServoCommand("pan", msg->data);
      },
      tx_options);

  // Timer to poll serial port in RX group
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(rx_poll_time_)), // 50ms
      std::bind(&SerialJsonNode::readSerial, this),
      rx_group_); // with Callback Group rx_group_ attached to the timer
}

void SerialJsonNode::readSerial()
{
  auto now = this->now();

  try
  {
    size_t available = serial_port_.available();
    if (available == 0)
    {
      return;
    }

    // Read all available bytes at once
    std::string chunk = serial_port_.read(available);
    buffer_ += chunk;

    // Split buffer by newline and parse complete lines
    size_t pos;
    while ((pos = buffer_.find('\n')) != std::string::npos)
    {
      std::string line = buffer_.substr(0, pos);
      buffer_.erase(0, pos + 1);

      try
      {
        auto j = json::parse(line);
        std_msgs::msg::String msg;
        msg.data = j.dump();
        publisher_->publish(msg);

        // Data Example:
        // data: '{"enc":{"back_left":30,"back_right":40,"front_left":10,"front_right":20}}'

        // 📤 Publish encoder data
        if (j.contains("enc") && j["enc"].is_object())
        {
          std_msgs::msg::Int32 msg_bl,msg_br, msg_fl, msg_fr;

          if (j["enc"].contains("back_left"))
          {
            msg_bl.data = j["enc"]["back_left"].get<int>();
            enc_back_left_pub_->publish(msg_bl);
          }

          if (j["enc"].contains("back_right"))
          {
            msg_br.data = j["enc"]["back_right"].get<int>();
            enc_back_right_pub_->publish(msg_br);
          }

          if (j["enc"].contains("front_left"))
          {
            msg_fl.data = j["enc"]["front_left"].get<int>();
            enc_front_left_pub_->publish(msg_fl);
          }

          if (j["enc"].contains("front_right"))
          {
            msg_fr.data = j["enc"]["front_right"].get<int>();
            enc_front_right_pub_->publish(msg_fr);
          }
        }

        // Publish IMU
        if (j.contains("imu") && j["imu"].is_object())
        {
          auto imu_vals = j["imu"];
          if (imu_vals.size() >= 3)
          {
            sensor_msgs::msg::Imu imu_msg;
            imu_msg.header.stamp = now;
            imu_msg.header.frame_id = imu_frame_id_;
            imu_msg.linear_acceleration.x = imu_vals.at(0).get<double>();
            imu_msg.linear_acceleration.y = imu_vals.at(1).get<double>();
            imu_msg.linear_acceleration.z = imu_vals.at(2).get<double>();
            imu_pub_->publish(imu_msg);
          }
          else
          {
            RCLCPP_WARN(get_logger(), "IMU data incomplete");
          }
        }

        // Publish sonar
        // data: {'sonar': {'right': 45, 'left': 23, 'front': 34}}
        // RIGHT
        if (j["sonar"].contains("right") && j["sonar"].is_object())
        {
          sensor_msgs::msg::Range msg;
          msg.header.stamp = now;
          msg.header.frame_id = sonar_frame_id_right_;
          if (radiation_type_ == 0) {
            msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
          }
          msg.field_of_view = field_of_view_;
          msg.min_range = min_range_;
          msg.max_range = max_range_;
          msg.range = j["sonar"]["right"].get<double>(); // Deve essere in metri!

          sonar_right_pub_->publish(msg);
        }

        // FRONT
        if (j["sonar"].contains("front") && j["sonar"].is_object())
        {
          sensor_msgs::msg::Range msg;
          msg.header.stamp = now;
          
          msg.header.frame_id = sonar_frame_id_front_;
          if (radiation_type_ == 0) {
            msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
          }
          msg.field_of_view = field_of_view_;
          msg.min_range = min_range_;
          msg.max_range = max_range_;
          msg.range = j["sonar"]["front"].get<double>(); // Deve essere in metri!

          sonar_front_pub_->publish(msg);
        }

        // LEFT
        if (j["sonar"].contains("left") && j["sonar"].is_object())
        {
          sensor_msgs::msg::Range msg;
          msg.header.stamp = now;
          msg.header.frame_id = sonar_frame_id_left_;
          if (radiation_type_ == 0) {
            msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
          }
          msg.field_of_view = field_of_view_;
          msg.min_range = min_range_;
          msg.max_range = max_range_;
          msg.range = j["sonar"]["left"].get<double>(); // Deve essere in metri!

          sonar_left_pub_->publish(msg);
        }
      }
      catch (const json::parse_error &e)
      {
        RCLCPP_WARN(this->get_logger(),
                    "JSON parse error on \"%s\": %s",
                    line.c_str(), e.what());
      }
      catch (const std::exception &e)
      {
        RCLCPP_WARN(this->get_logger(),
                    "Exception while parsing line \"%s\": %s",
                    line.c_str(), e.what());
      }
    }
  }
  catch (const serial::IOException &e)
  {
    RCLCPP_ERROR(this->get_logger(),
                 "Serial I/O error during read: %s", e.what());
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(this->get_logger(),
                 "Unexpected error during serial read: %s", e.what());
  }
}

void SerialJsonNode::sendWheelPower(const std::string &wheel_name, int value)
{
  if (!serial_port_.isOpen())
  {
    RCLCPP_ERROR(this->get_logger(), "Cannot write: serial port not open");
    return;
  }

  try
  {
    // il mutex garantisce che una sola scrittura avvenga alla volta,
    // anche se più callback vengono attivati quasi simultaneamente.
    std::lock_guard<std::mutex> lock(serial_mutex_);
    nlohmann::json j;
    j["wheel_power"] = {{wheel_name, value}};
    serial_port_.write(j.dump() + "\n");
  }
  catch (const serial::IOException &e)
  {
    RCLCPP_ERROR(this->get_logger(),
                 "Serial I/O error during write: %s", e.what());
  }
}

void SerialJsonNode::sendServoCommand(const std::string &servo_name, int value)
{
  if (!serial_port_.isOpen())
  {
    RCLCPP_ERROR(this->get_logger(), "Cannot write: serial port not open");
    return;
  }

  try
  {
    // il mutex garantisce che una sola scrittura avvenga alla volta,
    // anche se più callback vengono attivati quasi simultaneamente.
    std::lock_guard<std::mutex> lock(serial_mutex_);
    nlohmann::json j;
    j["servo_command"] = {{servo_name, value}};
    serial_port_.write(j.dump() + "\n");
  }
  catch (const serial::IOException &e)
  {
    RCLCPP_ERROR(this->get_logger(),
                 "Serial I/O error during write: %s", e.what());
  }
}

void SerialJsonNode::writeSerial(const std_msgs::msg::String::SharedPtr msg)
{
  if (!serial_port_.isOpen())
  {
    RCLCPP_ERROR(this->get_logger(), "Cannot write: serial port not open");
    return;
  }

  try
  {
    // il mutex garantisce che una sola scrittura avvenga alla volta,
    // anche se più callback vengono attivati quasi simultaneamente.
    std::lock_guard<std::mutex> lock(serial_mutex_);
    serial_port_.write(msg->data + "\n");
  }
  catch (const serial::IOException &e)
  {
    RCLCPP_ERROR(this->get_logger(),
                 "Serial I/O error during write: %s", e.what());
  }
}

// 🧩 Load all parameters
void SerialJsonNode::load_parameters()
{
  // 🔌 Serial parameters
  declare("serial.port", std::string("/dev/ttyUSB0"), port_);
  declare("serial.baud_rate", 115200, baudrate_);
  declare("serial.rx_poll_hz", 20.0, rx_poll_time_);
  declare("serial.reset_timeout_s", 5.0, reset_timeout_);

  // ⚙️ Wheel power topics
  declare("wheel_power.front_right.topic", std::string(""), wheel_topic_power_front_right_);
  declare("wheel_power.front_left.topic", std::string(""), wheel_topic_power_front_left_);
  declare("wheel_power.back_left.topic", std::string(""), wheel_topic_power_back_left_);
  declare("wheel_power.back_right.topic", std::string(""), wheel_topic_power_back_right_);

  // 🎯 Servo topics
  declare("servo.tilt.topic", std::string(""), servo_topic_tilt_);
  declare("servo.pan.topic", std::string(""), servo_topic_pan_);

  // 🧭 Encoder topic
  declare("encoder_front_left.topic", std::string("/bot/enc_front_left/encoder"), encoder_topic_front_left_);
  declare("encoder.front_right.topic", std::string("/bot/enc_front_right/encoder"), encoder_topic_front_right_);
  declare("encoder.back_left.topic", std::string("/bot/enc_back_left/encoder"), encoder_topic_back_left_);
  declare("encoder.back_right.topic", std::string("/bot/enc_back_right/encoder"), encoder_topic_back_right_);

  //declare("encoder_front_left.frame_id", std::string(""), encoder_frame_id_front_left_);
  //declare("encoder.front_right.frame_id", std::string(""), encoder_frame_id_front_right_);
  //declare("encoder.back_left.frame_id", std::string(""), encoder_frame_id_back_left_);
  //declare("encoder.back_right.frame_id", std::string(""), encoder_frame_id_back_right_);


  // IMU topics
  declare("imu.topic", std::string(""), imu_topic_);
  declare("imu_frame_id", std::string("base_imu"), imu_frame_id_);

  // Sonar topics
  declare("sonar.left.topic", std::string(""), sonar_topic_left_);
  declare("sonar.front.topic", std::string(""), sonar_topic_front_);
  declare("sonar.right.topic", std::string(""), sonar_topic_right_);

  // Sonar parameters
  declare("sonar.left.frame_id", std::string(""), sonar_frame_id_left_);
  declare("sonar.front.frame_id", std::string(""), sonar_frame_id_front_);
  declare("sonar.right.frame_id", std::string(""), sonar_frame_id_right_);

  declare("sonar.radiation_type", 0, radiation_type_);
  declare("sonar.field_of_view", 0.0, field_of_view_);
  declare("sonar.min_range", 0.0, min_range_);
  declare("sonar.max_range", 0.0, max_range_);


}

// Parameters used in the code:
// they are in hpp file

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SerialJsonNode>();

  // Use a multi-threaded executor to allow parallel callback execution
  // with the callback_group in SubscriptionOptions
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
