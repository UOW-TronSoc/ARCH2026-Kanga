#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include <can_msgs/msg/frame.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class ArmEndEffectorMapper : public rclcpp::Node
{
public:
  ArmEndEffectorMapper()
  : Node("arm_end_effector_mapper")
  {
    joint_command_topic_ = this->declare_parameter<std::string>(
      "joint_command_topic", "/kanga_arm/joint_control");
    can_interface_ = this->declare_parameter<std::string>("can_interface", "can0");
    j5_scale_to_255_ = this->declare_parameter<double>("j5_scale_to_255", 1.0);
    j6_min_angle_ = this->declare_parameter<double>("j6_min_angle", 0.0);
    j6_max_angle_ = this->declare_parameter<double>("j6_max_angle", 180.0);

    if (can_interface_.empty()) {
      RCLCPP_WARN(this->get_logger(), "Parameter 'can_interface' is empty; defaulting to 'can0'");
      can_interface_ = "can0";
    }
    if (j6_min_angle_ > j6_max_angle_) {
      std::swap(j6_min_angle_, j6_max_angle_);
    }

    const std::string can_tx_topic = "CAN/" + can_interface_ + "/transmit";
    can_pub_ = this->create_publisher<can_msgs::msg::Frame>(can_tx_topic, 10);
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_command_topic_, 10,
      std::bind(&ArmEndEffectorMapper::joint_state_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "arm_end_effector_mapper active: %s -> %s",
      joint_command_topic_.c_str(),
      can_tx_topic.c_str());
  }

private:
  static bool extract_joint_value(
    const sensor_msgs::msg::JointState & msg,
    const std::vector<double> & values,
    const std::vector<std::string> & candidate_names,
    size_t fallback_index,
    double & out_value)
  {
    if (!msg.name.empty() && msg.name.size() == values.size()) {
      for (const auto & candidate_name : candidate_names) {
        const auto it = std::find(msg.name.begin(), msg.name.end(), candidate_name);
        if (it != msg.name.end()) {
          const size_t idx = static_cast<size_t>(std::distance(msg.name.begin(), it));
          out_value = values[idx];
          return true;
        }
      }
    }

    if (fallback_index < values.size()) {
      out_value = values[fallback_index];
      return true;
    }

    return false;
  }

  void publish_j5_frame(double j5_value)
  {
    const double full_scale = std::max(std::abs(j5_scale_to_255_), 1.0e-6);
    const double normalized = std::clamp(std::abs(j5_value) / full_scale, 0.0, 1.0);
    const auto magnitude = static_cast<uint8_t>(std::lround(normalized * 255.0));
    uint8_t direction = 0U;
    if (magnitude > 0U) {
      direction = static_cast<uint8_t>((j5_value < 0.0) ? 0 : 1);
    }

    if (has_last_j5_ && last_j5_direction_ == direction && last_j5_magnitude_ == magnitude) {
      return;
    }

    can_msgs::msg::Frame msg;
    msg.header.stamp = this->now();
    msg.id = 802;
    msg.is_rtr = false;
    msg.is_extended = false;
    msg.is_error = false;
    msg.dlc = 2;
    msg.data[0] = direction;
    msg.data[1] = magnitude;

    can_pub_->publish(msg);
    last_j5_direction_ = direction;
    last_j5_magnitude_ = magnitude;
    has_last_j5_ = true;
  }

  void publish_j6_frame(double j6_value)
  {
    const double bounded = std::clamp(j6_value, j6_min_angle_, j6_max_angle_);
    const auto shifted = static_cast<uint8_t>(std::clamp(std::lround(bounded), 0L, 255L));

    if (has_last_j6_ && last_j6_value_ == shifted) {
      return;
    }

    can_msgs::msg::Frame msg;
    msg.header.stamp = this->now();
    msg.id = 803;
    msg.is_rtr = false;
    msg.is_extended = false;
    msg.is_error = false;
    msg.dlc = 1;
    msg.data[0] = shifted;

    can_pub_->publish(msg);
    last_j6_value_ = shifted;
    has_last_j6_ = true;
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    double j5_value = 0.0;
    if (extract_joint_value(*msg, msg->velocity, {"j5", "arm_j5"}, 4U, j5_value)) {
      publish_j5_frame(j5_value);
    } else {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Could not extract j5 command from joint message");
    }

    double j6_value = 0.0;
    const bool found_j6 = extract_joint_value(*msg, msg->position, {"j6", "arm_j6"}, 5U, j6_value);

    if (found_j6) {
      publish_j6_frame(j6_value);
    } else {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Could not extract j6 position command from joint message");
    }
  }

  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

  std::string joint_command_topic_;
  std::string can_interface_;
  double j5_scale_to_255_{1.0};
  double j6_min_angle_{0.0};
  double j6_max_angle_{180.0};
  bool has_last_j5_{false};
  uint8_t last_j5_direction_{0U};
  uint8_t last_j5_magnitude_{0U};
  bool has_last_j6_{false};
  uint8_t last_j6_value_{0U};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmEndEffectorMapper>());
  rclcpp::shutdown();
  return 0;
}
