#include "mecanum_base/estop_state_broadcaster.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace mecanum_base
{

    controller_interface::CallbackReturn EstopStateBroadcaster::on_init()
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration EstopStateBroadcaster::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration cfg;
        cfg.type = controller_interface::interface_configuration_type::NONE;
        return cfg;
    }

    controller_interface::InterfaceConfiguration EstopStateBroadcaster::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration cfg;
        cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        // Legge la state interface esportata dall'hardware interface
        cfg.names = {"estop_sensor/estop_active"};
        return cfg;
    }

    controller_interface::CallbackReturn EstopStateBroadcaster::on_activate(const rclcpp_lifecycle::State &)
    {
        pub_ = get_node()->create_publisher<std_msgs::msg::Bool>("/estop/active", 10);
        estop_state_index_ = 0; // unico stato richiesto
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn EstopStateBroadcaster::on_deactivate(const rclcpp_lifecycle::State &)
    {
        pub_.reset();
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type EstopStateBroadcaster::update(const rclcpp::Time &, const rclcpp::Duration &)
    {
        if (!pub_)
            return controller_interface::return_type::OK;

        auto opt_val = state_interfaces_[estop_state_index_].get_optional();
        bool active = opt_val.has_value() && opt_val.value() > 0.5;

        std_msgs::msg::Bool msg;
        msg.data = active;
        pub_->publish(msg);
        return controller_interface::return_type::OK;
    }

} // namespace mecanum_base


PLUGINLIB_EXPORT_CLASS(mecanum_base::EstopStateBroadcaster,
                       controller_interface::ControllerInterface)
