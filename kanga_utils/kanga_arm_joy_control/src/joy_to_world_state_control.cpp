#include <algorithm>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class JoyToWorldStateControl : public rclcpp::Node
{
public:
  JoyToWorldStateControl()
  : Node("joy_to_world_state_control")
  {
    axis_indices_ = this->declare_parameter<std::vector<int64_t>>(
      "axis_indices", std::vector<int64_t>{0, 1, 2, 3});
    joint_axis0_index_ = this->declare_parameter<int64_t>("joint_axis0_index", 0);
    axis_negative_j6_index_ = this->declare_parameter<int64_t>("axis_negative_j6", 4);
    axis_positive_j6_index_ = this->declare_parameter<int64_t>("axis_positive_j6", 5);
    j6_axis_pressed_threshold_ = this->declare_parameter<double>("j6_axis_pressed_threshold", 1.0);
    j6_min_angle_ = this->declare_parameter<double>("j6_min_angle", 0.0);
    j6_max_angle_ = this->declare_parameter<double>("j6_max_angle", 180.0);
    j6_start_angle_ = this->declare_parameter<double>("j6_start_angle", 90.0);
    j6_pwm_increment_speed_ = this->declare_parameter<double>("j6_pwm_increment_speed", 10.0);
    button_negative_roll_index_ = this->declare_parameter<int64_t>("button_negative_roll", 10);
    button_positive_roll_index_ = this->declare_parameter<int64_t>("button_positive_roll", 9);

    linear_scale_ = this->declare_parameter<double>("linear_scale", 1.0);
    pitch_scale_ = this->declare_parameter<double>("pitch_scale", 1.0);
    roll_scale_ = this->declare_parameter<double>("roll_scale", 1.0);

    world_control_topic_ = this->declare_parameter<std::string>(
      "world_control_topic", "kanga_arm/world_state_control");
    joint_control_topic_ = this->declare_parameter<std::string>(
      "joint_control_topic", "/kanga_arm/joint_control");

    if (axis_indices_.size() < 4) {
      RCLCPP_WARN(
        this->get_logger(),
        "axis_indices has fewer than 4 entries; missing commands will be zero");
      axis_indices_.resize(4, -1);
    } else if (axis_indices_.size() > 4) {
      axis_indices_.resize(4);
    }

    if (j6_min_angle_ > j6_max_angle_) {
      std::swap(j6_min_angle_, j6_max_angle_);
    }
    j6_position_ = std::clamp(j6_start_angle_, j6_min_angle_, j6_max_angle_);

    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(world_control_topic_, 10);
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(joint_control_topic_, 10);
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&JoyToWorldStateControl::joyCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "joy_to_world_state_control active: /joy -> (%s and %s)",
      world_control_topic_.c_str(),
      joint_control_topic_.c_str());
  }

private:
  static double readAxis(const sensor_msgs::msg::Joy & msg, int64_t index)
  {
    if (index < 0) {
      return 0.0;
    }
    const auto axis_index = static_cast<size_t>(index);
    if (axis_index >= msg.axes.size()) {
      return 0.0;
    }
    return msg.axes[axis_index];
  }

  static double readButton(const sensor_msgs::msg::Joy & msg, int64_t index)
  {
    if (index < 0) {
      return 0.0;
    }
    const auto button_index = static_cast<size_t>(index);
    if (button_index >= msg.buttons.size()) {
      return 0.0;
    }
    return static_cast<double>(msg.buttons[button_index]);
  }

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    const rclcpp::Time now = steady_clock_.now();
    if (last_update_time_.nanoseconds() == 0) {
      last_update_time_ = now;
    }
    const double dt = (now - last_update_time_).seconds();
    last_update_time_ = now;

    geometry_msgs::msg::Twist twist_msg;
    sensor_msgs::msg::JointState joint_msg;

    // Axis mapping: 0->X, 1->Y, 2->Z, 3->pitch.
    twist_msg.linear.x = linear_scale_ * readAxis(*msg, axis_indices_[0]);
    twist_msg.linear.y = linear_scale_ * readAxis(*msg, axis_indices_[1]);
    twist_msg.linear.z = linear_scale_ * readAxis(*msg, axis_indices_[2]);
    twist_msg.angular.y = pitch_scale_ * readAxis(*msg, axis_indices_[3]);

    // Reuse roll buttons as requested: positive - negative.
    const double roll_negative = readButton(*msg, button_negative_roll_index_);
    const double roll_positive = readButton(*msg, button_positive_roll_index_);
    twist_msg.angular.x = roll_scale_ * (roll_positive - roll_negative);
    twist_msg.angular.z = 0.0;

    joint_msg.header.stamp = this->get_clock()->now();
    joint_msg.name = {"j1", "j2", "j3", "j4", "j5", "j6"};
    joint_msg.velocity.resize(6, 0.0);
    joint_msg.position.resize(6, 0.0);
    joint_msg.velocity[0] = readAxis(*msg, joint_axis0_index_);
    joint_msg.velocity[4] = roll_positive - roll_negative;

    const bool j6_positive_pressed = readTriggerPressed(
      *msg, axis_positive_j6_index_, positive_trigger_initialized_);
    const bool j6_negative_pressed = readTriggerPressed(
      *msg, axis_negative_j6_index_, negative_trigger_initialized_);
    if (j6_positive_pressed == j6_negative_pressed) {
      joint_msg.velocity[5] = 0.0;
    } else {
      joint_msg.velocity[5] = j6_positive_pressed ? 1.0 : -1.0;
    }
    j6_position_ += joint_msg.velocity[5] * j6_pwm_increment_speed_ * std::max(dt, 0.0);
    j6_position_ = std::clamp(j6_position_, j6_min_angle_, j6_max_angle_);
    joint_msg.position[5] = j6_position_;

    twist_pub_->publish(twist_msg);
    joint_pub_->publish(joint_msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  bool readTriggerPressed(const sensor_msgs::msg::Joy & msg, int64_t axis_index, bool & initialized)
  {
    const double axis_value = readAxis(msg, axis_index);
    if (!initialized) {
      // Some drivers report untouched triggers as 0 until first movement.
      // Treat that as "not pressed" until we observe the released state.
      if (axis_value >= j6_axis_pressed_threshold_) {
        initialized = true;
      } else {
        return false;
      }
    }
    return axis_value < j6_axis_pressed_threshold_;
  }

  std::vector<int64_t> axis_indices_;
  int64_t joint_axis0_index_{0};
  int64_t axis_negative_j6_index_{4};
  int64_t axis_positive_j6_index_{5};
  double j6_axis_pressed_threshold_{1.0};
  double j6_min_angle_{0.0};
  double j6_max_angle_{180.0};
  double j6_start_angle_{90.0};
  double j6_pwm_increment_speed_{10.0};  // 1 unit per 100 ms
  double j6_position_{0.0};
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Time last_update_time_{0, 0, RCL_STEADY_TIME};
  bool negative_trigger_initialized_{false};
  bool positive_trigger_initialized_{false};
  int64_t button_negative_roll_index_{10};
  int64_t button_positive_roll_index_{9};
  double linear_scale_{1.0};
  double pitch_scale_{1.0};
  double roll_scale_{1.0};
  std::string world_control_topic_;
  std::string joint_control_topic_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyToWorldStateControl>());
  rclcpp::shutdown();
  return 0;
}
