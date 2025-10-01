// rm -rf build/ install/ log/
// colcon build --packages-select serial_json_node

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <serial/serial.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

class SerialJsonNode : public rclcpp::Node
{
public:
  SerialJsonNode() : Node("serial_json_node")
  {
    using namespace std::chrono_literals;

    // Gruppi di callback per RX e TX
    rx_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    tx_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // Configurazione porta seriale
    serial_port_.setPort("/dev/ttyACM0"); // Usa la porta appropriata sul Pi
    serial_port_.setBaudrate(115200);
    // serial_port_.setTimeout(serial::Timeout::simpleTimeout(100));
    serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
    serial_port_.setTimeout(timeout);

    try
    {
      serial_port_.open();
      RCLCPP_INFO(this->get_logger(), "Porta seriale aperta correttamente.");
    }
    catch (serial::IOException &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Errore nell'apertura della porta seriale: %s", e.what());
      return;
    }

    // Publisher per messaggi ricevuti
    publisher_ = this->create_publisher<std_msgs::msg::String>("pico_rx", 10);

    // Subscriber per messaggi da inviare
    rclcpp::SubscriptionOptions tx_options;
    tx_options.callback_group = tx_group_;
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "pico_tx", 10,
        std::bind(&SerialJsonNode::writeSerial, this, std::placeholders::_1),
        tx_options);

    // Timer per lettura seriale
    //rclcpp::TimerOptions rx_options;
    //rx_options.callback_group = rx_group_;
    //timer_ = this->create_wall_timer(
    //    50ms,
    //    std::bind(&SerialJsonNode::readSerial, this),
    //    rx_options);

    // crea un gruppo di callback se non l’hai già
    rx_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);

    // crea il timer direttamente
    timer_ = this->create_wall_timer(
        50ms, // periodo
        std::bind(&SerialJsonNode::readSerial, this),
        rx_group_ // gruppo di callback
    );
  }

private:
  void readSerial()
  {
    static std::string buffer;
    while (serial_port_.available())
    {
      char c = serial_port_.read(1)[0];
      if (c == '\n')
      {
        try
        {
          auto j = json::parse(buffer);
          std_msgs::msg::String msg;
          msg.data = j.dump(); // Rinvio come stringa JSON completa
          publisher_->publish(msg);
        }
        catch (json::parse_error &e)
        {
          RCLCPP_WARN(this->get_logger(), "Errore parsing JSON: %s", e.what());
        }
        buffer.clear();
      }
      else
      {
        buffer += c;
      }
    }
  }

  void writeSerial(const std_msgs::msg::String::SharedPtr msg)
  {
    if (serial_port_.isOpen())
    {
      serial_port_.write(msg->data + "\n");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Porta seriale non aperta!");
    }
  }

  serial::Serial serial_port_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::CallbackGroup::SharedPtr rx_group_;
  rclcpp::CallbackGroup::SharedPtr tx_group_;
};