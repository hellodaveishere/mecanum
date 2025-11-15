// Per test: ros2 service call /clear_estop std_srvs/srv/Trigger "{}"
// ros2 control list_controllers
// Vedrai che mecanum_velocity_controller passa da active a inactive.
/*
Note operative
Il servizio ora si chiama clear_estop relativo al nodo → lo vedrai come /estop_manager_node/clear_estop se il nodo gira senza namespace.

Puoi verificarlo con:

bash
ros2 service list | grep clear_estop
Per monitorare lo stato dei controller:

bash
watch -n 1 ros2 control list_controllers
Quando l’E‑STOP hardware diventa true, il controller passa a inactive.

Quando chiami:

bash
ros2 service call /estop_manager_node/clear_estop std_srvs/srv/Trigger "{}"
e l’hardware è rilasciato, il controller torna active.

*/

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/bool.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"

/**
 * Nodo EstopManager:
 *  - Si sottoscrive al topic /estop/active (pubblicato dal broadcaster)
 *  - Disattiva il controller di movimento quando l’E‑STOP hardware diventa attivo
 *  - Espone un servizio /clear_estop per riattivare il controller quando l’E‑STOP hardware è rilasciato
 */
class EstopManagerNode : public rclcpp::Node {
public:
  EstopManagerNode() : Node("estop_manager_node"), hw_estop_active_(false) {
    // Sottoscrizione al topic /estop/active
    sub_estop_ = this->create_subscription<std_msgs::msg::Bool>(
      "/estop/active", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        bool new_state = msg->data;
        RCLCPP_INFO(this->get_logger(), "Ricevuto estop_active=%s", new_state ? "true" : "false");

        // Se passa da inattivo a attivo → disattiva il controller
        if (!hw_estop_active_ && new_state) {
          deactivate_motion_controller();
        }
        hw_estop_active_ = new_state;
      });

    // Servizio per reset software dell’E‑STOP
    reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "clear_estop",   // nome del servizio (senza slash iniziale → relativo al nodo)
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr res) {
        // Se l’E‑STOP hardware è ancora attivo → reset negato
        if (hw_estop_active_) {
          res->success = false;
          res->message = "Reset negato: E‑STOP hardware ancora attivo";
          RCLCPP_WARN(this->get_logger(), "Tentativo di reset negato: hardware ancora attivo");
          return;
        }
        // Altrimenti prova a riattivare il controller
        bool ok = activate_motion_controller();
        res->success = ok;
        res->message = ok ? "Controller riattivato" : "Errore riattivazione";
      });
  }

private:
  bool hw_estop_active_;  // stato attuale dell’E‑STOP hardware
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_estop_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;

  /**
   * Disattiva il controller di movimento inviando una richiesta al controller_manager
   */
  void deactivate_motion_controller() {
    auto client = this->create_client<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/switch_controller");

    if (!client->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_ERROR(this->get_logger(), "Controller manager non disponibile per disattivazione");
      return;
    }

    auto req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    req->deactivate_controllers = {"mecanum_velocity_controller"};
    req->activate_controllers = {};
    req->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;

    auto fut = client->async_send_request(req);
    auto ret = rclcpp::spin_until_future_complete(this->get_node_base_interface(), fut,
                                                  std::chrono::seconds(2));

    if (ret != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Timeout/cancellazione richiesta di disattivazione");
      return;
    }

    bool ok = fut.get()->ok;
    if (ok) {
      RCLCPP_WARN(this->get_logger(), "mecanum_velocity_controller disattivato (E‑STOP)");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Disattivazione fallita (switch_controller ha restituito ok=false)");
    }
  }

  /**
   * Riattiva il controller di movimento inviando una richiesta al controller_manager
   */
  bool activate_motion_controller() {
    auto client = this->create_client<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/switch_controller");

    if (!client->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_ERROR(this->get_logger(), "Controller manager non disponibile per riattivazione");
      return false;
    }

    auto req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    req->activate_controllers = {"mecanum_velocity_controller"};
    req->deactivate_controllers = {};
    req->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;

    auto fut = client->async_send_request(req);
    auto ret = rclcpp::spin_until_future_complete(this->get_node_base_interface(), fut,
                                                  std::chrono::seconds(2));

    if (ret != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Timeout/cancellazione richiesta di riattivazione");
      return false;
    }

    bool ok = fut.get()->ok;
    if (ok) {
      RCLCPP_INFO(this->get_logger(), "mecanum_velocity_controller riattivato");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Riattivazione fallita (switch_controller ha restituito ok=false)");
    }
    return ok;
  }
};


int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EstopManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
