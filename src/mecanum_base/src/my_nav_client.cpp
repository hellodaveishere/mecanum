#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class NavClient : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  NavClient() : Node("my_nav_client")
  {
    client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    timer_ = this->create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&NavClient::send_goal, this));
  }

private:
  void send_goal()
  {
    if (!client_->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_WARN(get_logger(), "Nav2 action server not available");
      return;
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = now();
    goal_msg.pose.pose.position.x = 1.0;
    goal_msg.pose.pose.position.y = 0.0;
    goal_msg.pose.pose.orientation.w = 1.0;

    client_->async_send_goal(goal_msg);
    RCLCPP_INFO(get_logger(), "Goal sent");
  }

  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavClient>());
  rclcpp::shutdown();
  return 0;
}
