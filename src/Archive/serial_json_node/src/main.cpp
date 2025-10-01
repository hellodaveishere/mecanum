//ros2 run serial_json_node serial_json_node --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=115200

// File: src/main.cpp

#include <chrono>
#include <csignal>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <serial/serial.h>
#include <nlohmann/json.hpp>

using namespace std::chrono_literals;
using json = nlohmann::json;

class SerialJsonNode : public rclcpp::Node
{
public:
  SerialJsonNode()
  : Node("serial_json_node")
  {
    // 1) callback groups
    rx_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    tx_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // 2) serial setup
    serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
    serial_port_.setPort("/dev/serial0");
    serial_port_.setBaudrate(115200);
    serial_port_.setTimeout(timeout);

    try {
      serial_port_.open();
      RCLCPP_INFO(get_logger(), "Porta seriale aperta: %s @ %d",
                  serial_port_.getPort().c_str(),
                  serial_port_.getBaudrate());
    } catch (const serial::IOException &e) {
      RCLCPP_ERROR(get_logger(),
                   "Unable to open serial port: %s", e.what());
      throw;  // forza l’uscita se non riesce ad aprire
    }

    // 3) publisher per dati in arrivo
    publisher_ = create_publisher<std_msgs::msg::String>("pico_rx", 10);

    // 4) subscriber per invio dati
    rclcpp::SubscriptionOptions tx_opts;
    tx_opts.callback_group = tx_group_;
    subscription_ = create_subscription<std_msgs::msg::String>(
      "pico_tx", 10,
      std::bind(&SerialJsonNode::writeSerial, this, std::placeholders::_1),
      tx_opts);

    // 5) timer di polling seriale
    timer_ = create_wall_timer(
      50ms,
      std::bind(&SerialJsonNode::readSerial, this),
      rx_group_);
  }

private:
  void readSerial()
  {
    static std::string buffer;
    while (serial_port_.available() > 0) {
      auto data = serial_port_.read(1);
      char c = data.empty() ? '\0' : data[0];
      if (c == '\n') {
        try {
          auto j = json::parse(buffer);
          std_msgs::msg::String msg;
          msg.data = j.dump();
          publisher_->publish(msg);
        } catch (const json::parse_error &e) {
          RCLCPP_WARN(get_logger(),
                      "JSON parse error: %s (buffer: \"%s\")",
                      e.what(), buffer.c_str());
        }
        buffer.clear();
      } else if (c != '\r') {
        buffer += c;
      }
    }
  }

  void writeSerial(const std_msgs::msg::String::SharedPtr msg)
  {
    if (serial_port_.isOpen()) {
      serial_port_.write(msg->data + "\n");
    } else {
      RCLCPP_ERROR(get_logger(), "Serial port not open!");
    }
  }

  serial::Serial serial_port_;
  rclcpp::TimerBase::SharedPtr               timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::CallbackGroup::SharedPtr           rx_group_;
  rclcpp::CallbackGroup::SharedPtr           tx_group_;
};

static std::atomic<bool> g_shutdown_requested{false};

void signal_handler(int)
{
  RCLCPP_INFO(rclcpp::get_logger("signal"), "SIGINT received, shutting down...");
  g_shutdown_requested = true;
  rclcpp::shutdown();
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  std::signal(SIGINT, signal_handler);

  auto node = std::make_shared<SerialJsonNode>();
  RCLCPP_INFO(node->get_logger(), "SerialJsonNode started");

  // single-threaded spin è sufficiente qui
  while (rclcpp::ok() && !g_shutdown_requested) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(10ms);
  }

  RCLCPP_INFO(node->get_logger(), "SerialJsonNode stopped");
  return 0;
}
