#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("suspension_node");

  auto pub = node->create_publisher<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::QoS(10));

  sensor_msgs::msg::JointState msg;
  msg.header.stamp = node->now();
  msg.header.frame_id = "";
  msg.name = {"left_suspension_joint", "right_suspension_joint"};
  msg.position = {0.0, 0.0};
  msg.velocity = {0.0, 0.0};
  msg.effort = {0.0, 0.0};

  rclcpp::sleep_for(std::chrono::milliseconds(500));
  pub->publish(msg);

  RCLCPP_INFO(node->get_logger(),
    "Published suspension joint states (left=0.0, right=0.0)");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
