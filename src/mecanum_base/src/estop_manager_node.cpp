#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/bool.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"

/**
 * Nodo EstopManager:
 *  - Si sottoscrive al topic /estop/active (pubblicato dal broadcaster hardware)
 *  - Disattiva il controller di movimento quando l’E‑STOP hardware diventa attivo (solo via topic)
 *  - Espone un servizio ~/clear_estop per riattivare il controller quando l’E‑STOP hardware è rilasciato (solo via servizio)
 *
 * 📌 Comandi di test:
 *  Verifica servizio:
 *    ros2 service list | grep clear_estop   → /estop_manager_node/clear_estop
 *
 *  Disattivazione via topic:
 *    ros2 topic pub /estop/active std_msgs/msg/Bool "data: true"
 *
 *  Riattivazione via servizio:
 *    ros2 service call /estop_manager_node/clear_estop std_srvs/srv/Trigger "{}"
 *
 *  Stato controller:
 *    watch -n 1 ros2 control list_controllers
 */
class EstopManagerNode : public rclcpp::Node
{
public:
  EstopManagerNode()
      : Node("estop_manager_node"),
        hw_estop_active_(false)
  {
    // Callback groups
    service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    client_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // Client verso controller_manager (QoS aggiornato)
    switch_client_ = this->create_client<controller_manager_msgs::srv::SwitchController>(
        "/controller_manager/switch_controller",
        rclcpp::ServicesQoS(),
        client_cb_group_);

    list_client_ = this->create_client<controller_manager_msgs::srv::ListControllers>(
        "/controller_manager/list_controllers",
        rclcpp::ServicesQoS(),
        client_cb_group_);

    // Sottoscrizione al topic /estop/active
    sub_estop_ = this->create_subscription<std_msgs::msg::Bool>(
        "/estop/active", rclcpp::QoS(10),
        [this](const std_msgs::msg::Bool::SharedPtr msg)
        {
          bool new_state = msg->data;

          // Stampa log solo se lo stato cambia
          if (new_state != hw_estop_active_)
          {
            if (new_state)
            {
              RCLCPP_WARN(this->get_logger(), "E-STOP ATTIVO: disattivo il controller");
              bool stopped = deactivate_motion_controller();
              if (!stopped)
              {
                RCLCPP_ERROR(this->get_logger(), "Disattivazione controller non confermata");
              }
            }
            else
            {
              RCLCPP_INFO(this->get_logger(), "E-STOP DISATTIVO");
            }
          }

          // Aggiorna lo stato interno
          hw_estop_active_ = new_state;
        });

    // Servizio ~/clear_estop
    reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "~/clear_estop",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
               std_srvs::srv::Trigger::Response::SharedPtr res)
        {
          RCLCPP_INFO(this->get_logger(), "Richiesta riattivazione controller");

          if (hw_estop_active_)
          {
            res->success = false;
            res->message = "Impossibile riattivare: estop ancora attivo";
            return;
          }

          const bool requested = activate_motion_controller();

          // Poll dello stato reale fino a 2s
          const bool active = wait_until_controller_state(
              "mecanum_velocity_controller", "active",
              std::chrono::milliseconds(2000), std::chrono::milliseconds(100));

          if (active)
          {
            res->success = true;
            res->message = "Controller riattivato";
          }
          else
          {
            res->success = false;
            res->message = requested
                               ? "Attivazione non confermata entro timeout"
                               : "Errore riattivazione (richiesta non inviata/timeout)";
          }
        },
        rclcpp::ServicesQoS(),
        service_cb_group_);
  }

private:
  bool hw_estop_active_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_estop_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  rclcpp::CallbackGroup::SharedPtr service_cb_group_;
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_client_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_client_;

  bool wait_service(const rclcpp::ClientBase::SharedPtr &client, std::chrono::seconds timeout)
  {
    if (!client->wait_for_service(timeout))
    {
      RCLCPP_ERROR(this->get_logger(), "Servizio %s non disponibile",
                   client->get_service_name());
      return false;
    }
    return true;
  }

  bool deactivate_motion_controller()
  {
    if (!wait_service(switch_client_, std::chrono::seconds(3)))
      return false;

    auto req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    req->deactivate_controllers.push_back("mecanum_velocity_controller");
    req->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;

    auto fut = switch_client_->async_send_request(req);
    auto status = fut.wait_for(std::chrono::seconds(5));
    if (status != std::future_status::ready)
    {
      RCLCPP_ERROR(this->get_logger(), "Timeout nella chiamata a switch_controller (deactivate)");
      return false;
    }

    auto res = fut.get();
    RCLCPP_INFO(this->get_logger(), "deactivate_controllers -> ok=%s", res->ok ? "true" : "false");

    return res->ok;
  }

  bool activate_motion_controller()
  {
    if (!wait_service(switch_client_, std::chrono::seconds(3)))
      return false;

    auto req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    req->activate_controllers.push_back("mecanum_velocity_controller");
    req->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;

    auto fut = switch_client_->async_send_request(req);
    auto status = fut.wait_for(std::chrono::seconds(5));
    if (status != std::future_status::ready)
    {
      RCLCPP_ERROR(this->get_logger(), "Timeout nella chiamata a switch_controller (activate)");
      return false;
    }

    auto res = fut.get();
    RCLCPP_INFO(this->get_logger(), "activate_controllers -> ok=%s", res->ok ? "true" : "false");

    return res->ok;
  }

  bool is_controller_state(const std::string &name, const std::string &state)
  {
    if (!wait_service(list_client_, std::chrono::seconds(2)))
      return false;

    auto req = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
    auto fut = list_client_->async_send_request(req);
    auto status = fut.wait_for(std::chrono::seconds(2));
    if (status != std::future_status::ready)
    {
      RCLCPP_ERROR(this->get_logger(), "Timeout nella chiamata a list_controllers");
      return false;
    }

    auto res = fut.get();
    for (const auto &c : res->controller)
    {
      if (c.name == name)
      {
        return c.state == state;
      }
    }
    return false;
  }

  bool wait_until_controller_state(const std::string &name,
                                   const std::string &target_state,
                                   std::chrono::milliseconds timeout,
                                   std::chrono::milliseconds step)
  {
    const auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < timeout)
    {
      if (is_controller_state(name, target_state))
      {
        return true;
      }
      rclcpp::sleep_for(step);
    }
    return false;
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EstopManagerNode>();

  // Executor multithread: consente ai client di ricevere risposte mentre il servizio è in attesa
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
