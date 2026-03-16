#include <algorithm>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

/**
 * @brief Hybrid joy control: toggle between joint and end-effector modes.
 *
 * Button 1 toggles between:
 * - Joint mode: J1-J6 from joystick axes/buttons -> joint_control
 * - EE mode: J1,J5,J6 -> joint_control; X,Z,pitch,roll -> ee_state_control
 *
 * J1, J5, J6 are always sent to joint_control (for arm_end_effector_mapper).
 * J2, J3, J4 and X,Z,pitch switch based on mode.
 */
class JoyToHybridControl : public rclcpp::Node
{
public:
  JoyToHybridControl()
  : Node("joy_to_hybrid_control")
  {
    // Joint mode: axes for j1,j2,j3,j4
    joint_axis_indices_ = this->declare_parameter<std::vector<int64_t>>(
      "joint_axis_indices", std::vector<int64_t>{0, 1, 2, 3});
    // EE mode: axes for X, Z, pitch (axis 1->X, 0->Y zeroed, 3->Z, 2->pitch)
    ee_axis_indices_ = this->declare_parameter<std::vector<int64_t>>(
      "ee_axis_indices", std::vector<int64_t>{1, 0, 3, 2});

    axis_negative_j6_index_ = this->declare_parameter<int64_t>("axis_negative_j6", 4);
    axis_positive_j6_index_ = this->declare_parameter<int64_t>("axis_positive_j6", 5);
    j6_axis_pressed_threshold_ = this->declare_parameter<double>("j6_axis_pressed_threshold", 1.0);
    j6_min_angle_ = this->declare_parameter<double>("j6_min_angle", 0.0);
    j6_max_angle_ = this->declare_parameter<double>("j6_max_angle", 180.0);
    j6_start_angle_ = this->declare_parameter<double>("j6_start_angle", 90.0);
    j6_pwm_increment_speed_ = this->declare_parameter<double>("j6_pwm_increment_speed", 30.0);
    button_negative_roll_index_ = this->declare_parameter<int64_t>("button_negative_roll", 10);
    button_positive_roll_index_ = this->declare_parameter<int64_t>("button_positive_roll", 9);
    mode_toggle_button_index_ = this->declare_parameter<int64_t>("mode_toggle_button", 0);

    linear_scale_ = this->declare_parameter<double>("linear_scale", 1.0);
    pitch_scale_ = this->declare_parameter<double>("pitch_scale", 1.0);
    roll_scale_ = this->declare_parameter<double>("roll_scale", 1.0);
    joint_velocity_scale_ = this->declare_parameter<double>("joint_velocity_scale", 1.0);

    ee_control_topic_ = this->declare_parameter<std::string>(
      "ee_control_topic", "kanga_arm/ee_state_control");
    joint_control_topic_ = this->declare_parameter<std::string>(
      "joint_control_topic", "/kanga_arm/joint_control");
    mode_topic_ = this->declare_parameter<std::string>(
      "mode_topic", "kanga_arm/control_mode_joint");
    const std::string start_mode = this->declare_parameter<std::string>("start_mode", "joint");
    mode_ = (start_mode == "ee" || start_mode == "end_effector") ? ControlMode::kEE : ControlMode::kJoint;

    resizeAxisVector(joint_axis_indices_, "joint_axis_indices", 4);
    resizeAxisVector(ee_axis_indices_, "ee_axis_indices", 4);

    if (j6_min_angle_ > j6_max_angle_) {
      std::swap(j6_min_angle_, j6_max_angle_);
    }
    j6_position_ = std::clamp(j6_start_angle_, j6_min_angle_, j6_max_angle_);

    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(joint_control_topic_, 10);
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(ee_control_topic_, 10);
    mode_pub_ = this->create_publisher<std_msgs::msg::Bool>(mode_topic_, 10);
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&JoyToHybridControl::joyCallback, this, std::placeholders::_1));

    publishMode();

    RCLCPP_INFO(
      this->get_logger(),
      "joy_to_hybrid_control active: mode=%s, toggle=button[%ld], /joy -> (%s | %s)",
      modeString().c_str(),
      mode_toggle_button_index_,
      joint_control_topic_.c_str(),
      ee_control_topic_.c_str());
  }

private:
  enum class ControlMode
  {
    kJoint,
    kEE
  };

  static double readAxis(const sensor_msgs::msg::Joy & msg, int64_t index)
  {
    if (index < 0) return 0.0;
    const auto axis_index = static_cast<size_t>(index);
    if (axis_index >= msg.axes.size()) return 0.0;
    return msg.axes[axis_index];
  }

  static double readButton(const sensor_msgs::msg::Joy & msg, int64_t index)
  {
    if (index < 0) return 0.0;
    const auto button_index = static_cast<size_t>(index);
    if (button_index >= msg.buttons.size()) return 0.0;
    return static_cast<double>(msg.buttons[button_index]);
  }

  static bool readButtonPressed(const sensor_msgs::msg::Joy & msg, int64_t index)
  {
    return readButton(msg, index) != 0.0;
  }

  void resizeAxisVector(std::vector<int64_t> & indices, const std::string & param_name, size_t expected)
  {
    if (indices.size() < expected) {
      RCLCPP_WARN(
        this->get_logger(),
        "%s has fewer than %zu entries; padding with -1",
        param_name.c_str(), expected);
      indices.resize(expected, -1);
    } else if (indices.size() > expected) {
      indices.resize(expected);
    }
  }

  bool readTriggerPressed(const sensor_msgs::msg::Joy & msg, int64_t axis_index, bool & initialized)
  {
    const double axis_value = readAxis(msg, axis_index);
    if (!initialized) {
      if (axis_value >= j6_axis_pressed_threshold_) {
        initialized = true;
      } else {
        return false;
      }
    }
    return axis_value < j6_axis_pressed_threshold_;
  }

  std::string modeString() const
  {
    return mode_ == ControlMode::kJoint ? "joint" : "ee";
  }

  void toggleMode()
  {
    if (mode_ == ControlMode::kJoint) {
      mode_ = ControlMode::kEE;
      publishZeroTwist();
    } else {
      mode_ = ControlMode::kJoint;
      publishZeroTwist();
    }
    publishMode();
    RCLCPP_INFO(this->get_logger(), "Mode switched to: %s", modeString().c_str());
  }

  void publishMode()
  {
    std_msgs::msg::Bool msg;
    msg.data = (mode_ == ControlMode::kJoint);
    mode_pub_->publish(msg);
  }

  void publishZeroTwist()
  {
    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = twist_msg.linear.y = twist_msg.linear.z = 0.0;
    twist_msg.angular.x = twist_msg.angular.y = twist_msg.angular.z = 0.0;
    twist_pub_->publish(twist_msg);
  }

  void publishJointCommand(const sensor_msgs::msg::Joy & joy_msg)
  {
    const rclcpp::Time now = steady_clock_.now();
    if (last_update_time_.nanoseconds() == 0) {
      last_update_time_ = now;
    }
    const double dt = (now - last_update_time_).seconds();
    last_update_time_ = now;

    sensor_msgs::msg::JointState joint_msg;
    joint_msg.header.stamp = this->get_clock()->now();
    joint_msg.name = {"j1", "j2", "j3", "j4", "j5", "j6"};
    joint_msg.velocity.resize(6, 0.0);
    joint_msg.position.resize(6, 0.0);

    joint_msg.velocity[0] = joint_velocity_scale_ * readAxis(joy_msg, joint_axis_indices_[0]);
    joint_msg.velocity[1] = joint_velocity_scale_ * readAxis(joy_msg, joint_axis_indices_[1]);
    joint_msg.velocity[2] = joint_velocity_scale_ * readAxis(joy_msg, joint_axis_indices_[2]);
    joint_msg.velocity[3] = -joint_velocity_scale_ * readAxis(joy_msg, joint_axis_indices_[3]);

    const double j5_negative = readButton(joy_msg, button_negative_roll_index_);
    const double j5_positive = readButton(joy_msg, button_positive_roll_index_);
    joint_msg.velocity[4] = joint_velocity_scale_ * (j5_positive - j5_negative);

    const bool j6_positive_pressed = readTriggerPressed(
      joy_msg, axis_positive_j6_index_, positive_trigger_initialized_);
    const bool j6_negative_pressed = readTriggerPressed(
      joy_msg, axis_negative_j6_index_, negative_trigger_initialized_);
    if (j6_positive_pressed == j6_negative_pressed) {
      joint_msg.velocity[5] = 0.0;
    } else {
      joint_msg.velocity[5] = j6_positive_pressed ? 1.0 : -1.0;
    }
    j6_position_ += joint_msg.velocity[5] * j6_pwm_increment_speed_ * std::max(dt, 0.0);
    j6_position_ = std::clamp(j6_position_, j6_min_angle_, j6_max_angle_);
    joint_msg.position[5] = j6_position_;

    joint_pub_->publish(joint_msg);
  }

  void publishEECommand(const sensor_msgs::msg::Joy & joy_msg)
  {
    const rclcpp::Time now = steady_clock_.now();
    if (last_update_time_.nanoseconds() == 0) {
      last_update_time_ = now;
    }
    const double dt = (now - last_update_time_).seconds();
    last_update_time_ = now;

    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = -linear_scale_ * readAxis(joy_msg, ee_axis_indices_[0]);
    twist_msg.linear.y = 0.0;  // j1 bypasses kinematics
    twist_msg.linear.z = linear_scale_ * readAxis(joy_msg, ee_axis_indices_[2]);
    twist_msg.angular.y = -pitch_scale_ * readAxis(joy_msg, ee_axis_indices_[3]);
    const double roll_negative = readButton(joy_msg, button_negative_roll_index_);
    const double roll_positive = readButton(joy_msg, button_positive_roll_index_);
    twist_msg.angular.x = -roll_scale_ * (roll_positive - roll_negative);
    twist_msg.angular.z = 0.0;
    twist_pub_->publish(twist_msg);

    sensor_msgs::msg::JointState joint_msg;
    joint_msg.header.stamp = this->get_clock()->now();
    joint_msg.name = {"j1", "j2", "j3", "j4", "j5", "j6"};
    joint_msg.velocity.resize(6, 0.0);
    joint_msg.position.resize(6, 0.0);
    joint_msg.velocity[0] = readAxis(joy_msg, joint_axis_indices_[0]);
    joint_msg.velocity[4] = roll_positive - roll_negative;

    const bool j6_positive_pressed = readTriggerPressed(
      joy_msg, axis_positive_j6_index_, positive_trigger_initialized_);
    const bool j6_negative_pressed = readTriggerPressed(
      joy_msg, axis_negative_j6_index_, negative_trigger_initialized_);
    if (j6_positive_pressed == j6_negative_pressed) {
      joint_msg.velocity[5] = 0.0;
    } else {
      joint_msg.velocity[5] = j6_positive_pressed ? 1.0 : -1.0;
    }
    j6_position_ += joint_msg.velocity[5] * j6_pwm_increment_speed_ * std::max(dt, 0.0);
    j6_position_ = std::clamp(j6_position_, j6_min_angle_, j6_max_angle_);
    joint_msg.position[5] = j6_position_;

    joint_pub_->publish(joint_msg);
  }

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    const bool toggle_pressed = readButtonPressed(*msg, mode_toggle_button_index_);
    if (toggle_pressed && !toggle_button_was_pressed_) {
      toggleMode();
    }
    toggle_button_was_pressed_ = toggle_pressed;

    if (mode_ == ControlMode::kJoint) {
      publishJointCommand(*msg);
    } else {
      publishEECommand(*msg);
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mode_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  std::vector<int64_t> joint_axis_indices_;
  std::vector<int64_t> ee_axis_indices_;
  int64_t axis_negative_j6_index_{4};
  int64_t axis_positive_j6_index_{5};
  double j6_axis_pressed_threshold_{1.0};
  double j6_min_angle_{0.0};
  double j6_max_angle_{180.0};
  double j6_start_angle_{90.0};
  double j6_pwm_increment_speed_{30.0};
  double j6_position_{0.0};
  int64_t button_negative_roll_index_{10};
  int64_t button_positive_roll_index_{9};
  int64_t mode_toggle_button_index_{0};
  double linear_scale_{1.0};
  double pitch_scale_{1.0};
  double roll_scale_{1.0};
  double joint_velocity_scale_{1.0};
  std::string ee_control_topic_;
  std::string joint_control_topic_;
  std::string mode_topic_;
  ControlMode mode_{ControlMode::kJoint};
  bool toggle_button_was_pressed_{false};
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Time last_update_time_{0, 0, RCL_STEADY_TIME};
  bool negative_trigger_initialized_{false};
  bool positive_trigger_initialized_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyToHybridControl>());
  rclcpp::shutdown();
  return 0;
}
