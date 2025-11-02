#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <unordered_map>
#include <string>
#include <cmath>

class MecanumOdomNode : public rclcpp::Node
{
public:
  MecanumOdomNode() : Node("mecanum_odom_node")
  {
    r_ = declare_parameter<double>("wheel_radius", 0.05);
    L_ = declare_parameter<double>("L", 0.15);
    W_ = declare_parameter<double>("W", 0.15);
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    // 👇 frame base impostato su base_footprint per allinearsi all'URDF
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");

    fl_ = declare_parameter<std::string>("front_left_joint",  "front_left_wheel_joint");
    fr_ = declare_parameter<std::string>("front_right_joint", "front_right_wheel_joint");
    rl_ = declare_parameter<std::string>("rear_left_joint",   "rear_left_wheel_joint");
    rr_ = declare_parameter<std::string>("rear_right_joint",  "rear_right_wheel_joint");

    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    /joint_statessub_js_ = create_subscription<sensor_msgs::msg::JointState>(
      "", rclcpp::QoS(50),
      std::bind(&MecanumOdomNode::jsCb, this, std::placeholders::_1));

    last_time_ = now();
    RCLCPP_INFO(get_logger(), "mecanum_odom_node avviato");
  }

private:
  void jsCb(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::unordered_map<std::string, double> vel;
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (i < msg->velocity.size()) vel[msg->name[i]] = msg->velocity[i];
    }
    if (!vel.count(fl_) || !vel.count(fr_) || !vel.count(rl_) || !vel.count(rr_)) return;

    const double w_fl = vel[fl_];
    const double w_fr = vel[fr_];
    const double w_rl = vel[rl_];
    const double w_rr = vel[rr_];

    const double a = (L_ + W_);
    const double vx = (r_/4.0) * ( w_fl + w_fr + w_rl + w_rr );
    const double vy = (r_/4.0) * ( -w_fl + w_fr + w_rl - w_rr );
    const double wz = (r_/(4.0*a)) * ( -w_fl + w_fr - w_rl + w_rr );

    const rclcpp::Time t = now();
    const double dt = (t - last_time_).seconds();
    last_time_ = t;

    const double dth = wz * dt;
    const double sth = std::sin(theta_ + 0.5 * dth);
    const double cth = std::cos(theta_ + 0.5 * dth);

    x_ += (vx * cth - vy * sth) * dt;
    y_ += (vx * sth + vy * cth) * dt;
    theta_ += dth;

    // Pubblica /odom
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = t;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    const double qz = std::sin(theta_ * 0.5);
    const double qw = std::cos(theta_ * 0.5);
    odom.pose.pose.orientation.z = qz;
    odom.pose.pose.orientation.w = qw;
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = wz;

    odom.pose.covariance[0] = 1e-3;
    odom.pose.covariance[7] = 1e-3;
    odom.pose.covariance[35] = 1e-2;
    odom.twist.covariance[0] = 5e-3;
    odom.twist.covariance[7] = 5e-3;
    odom.twist.covariance[35] = 5e-3;
    pub_odom_->publish(odom);

    // Pubblica TF odom -> base_footprint
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = t;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.z = qz;
    tf.transform.rotation.w = qw;
    tf_broadcaster_->sendTransform(tf);
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  double r_, L_, W_;
  std::string odom_frame_, base_frame_;
  std::string fl_, fr_, rl_, rr_;

  double x_{0.0}, y_{0.0}, theta_{0.0};
  rclcpp::Time last_time_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MecanumOdomNode>());
  rclcpp::shutdown();
  return 0;
}
