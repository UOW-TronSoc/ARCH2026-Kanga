#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class JoyToDualControl : public rclcpp::Node
{
public:
  JoyToDualControl()
  : Node("joy_to_dual_control")
  {
    joint_axis_indices_ = this->declare_parameter<std::vector<int64_t>>(
      "joint_axis_indices", std::vector<int64_t>{0, 1, 3, 2});
    world_axis_indices_ = this->declare_parameter<std::vector<int64_t>>(
      "world_axis_indices", std::vector<int64_t>{0, 1, 2, 3});

    button_negative_j5_index_ = this->declare_parameter<int64_t>("button_negative_j5", 10);
    button_positive_j5_index_ = this->declare_parameter<int64_t>("button_positive_j5", 9);
    button_negative_roll_index_ = this->declare_parameter<int64_t>("button_negative_roll", 10);
    button_positive_roll_index_ = this->declare_parameter<int64_t>("button_positive_roll", 9);
    mode_toggle_button_index_ = this->declare_parameter<int64_t>("mode_toggle_button", 0);

    joint_velocity_scale_ = this->declare_parameter<double>("joint_velocity_scale", 1.0);
    linear_scale_ = this->declare_parameter<double>("linear_scale", 1.0);
    pitch_scale_ = this->declare_parameter<double>("pitch_scale", 1.0);
    roll_scale_ = this->declare_parameter<double>("roll_scale", 1.0);

    joint_topic_ = this->declare_parameter<std::string>(
      "joint_control_topic", "/kanga_arm/joint_control");
    world_control_topic_ = this->declare_parameter<std::string>(
      "world_control_topic", "kanga_arm/world_state_control");
    const std::string start_mode = this->declare_parameter<std::string>("start_mode", "joint");
    mode_ = (start_mode == "world") ? ControlMode::kWorld : ControlMode::kJoint;

    resizeAxisVector(joint_axis_indices_, "joint_axis_indices");
    resizeAxisVector(world_axis_indices_, "world_axis_indices");

    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(joint_topic_, 10);
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(world_control_topic_, 10);
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&JoyToDualControl::joyCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "joy_to_dual_control active: mode=%s, toggle_button=%ld, /joy -> (%s | %s)",
      modeString().c_str(),
      mode_toggle_button_index_,
      joint_topic_.c_str(),
      world_control_topic_.c_str());
  }

private:
  enum class ControlMode
  {
    kJoint,
    kWorld
  };

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

  static bool readButtonPressed(const sensor_msgs::msg::Joy & msg, int64_t index)
  {
    if (index < 0) {
      return false;
    }
    const auto button_index = static_cast<size_t>(index);
    if (button_index >= msg.buttons.size()) {
      return false;
    }
    return msg.buttons[button_index] != 0;
  }

  static double readButtonValue(const sensor_msgs::msg::Joy & msg, int64_t index)
  {
    return readButtonPressed(msg, index) ? 1.0 : 0.0;
  }

  void resizeAxisVector(std::vector<int64_t> & axis_indices, const std::string & param_name)
  {
    if (axis_indices.size() < 4) {
      RCLCPP_WARN(
        this->get_logger(),
        "%s has fewer than 4 entries; missing commands will be zero",
        param_name.c_str());
      axis_indices.resize(4, -1);
    } else if (axis_indices.size() > 4) {
      axis_indices.resize(4);
    }
  }

  std::string modeString() const
  {
    return mode_ == ControlMode::kJoint ? "joint" : "world";
  }

  void publishZeroJoint()
  {
    sensor_msgs::msg::JointState joint_msg;
    joint_msg.header.stamp = this->get_clock()->now();
    joint_msg.name = {"j1", "j2", "j3", "j4", "j5"};
    joint_msg.velocity.resize(5, 0.0);
    joint_pub_->publish(joint_msg);
  }

  void publishZeroWorld()
  {
    geometry_msgs::msg::Twist twist_msg;
    twist_pub_->publish(twist_msg);
  }

  void toggleMode()
  {
    if (mode_ == ControlMode::kJoint) {
      mode_ = ControlMode::kWorld;
      publishZeroJoint();
    } else {
      mode_ = ControlMode::kJoint;
      publishZeroWorld();
    }

    RCLCPP_INFO(this->get_logger(), "Joystick mode switched to: %s", modeString().c_str());
  }

  void publishJointCommand(const sensor_msgs::msg::Joy & joy_msg)
  {
    sensor_msgs::msg::JointState joint_msg;
    joint_msg.header.stamp = this->get_clock()->now();
    joint_msg.name = {"j1", "j2", "j3", "j4", "j5"};
    joint_msg.velocity.resize(5, 0.0);

    joint_msg.velocity[0] = joint_velocity_scale_ * readAxis(joy_msg, joint_axis_indices_[0]);
    joint_msg.velocity[1] = joint_velocity_scale_ * readAxis(joy_msg, joint_axis_indices_[1]);
    joint_msg.velocity[2] = joint_velocity_scale_ * readAxis(joy_msg, joint_axis_indices_[2]);
    joint_msg.velocity[3] = joint_velocity_scale_ * readAxis(joy_msg, joint_axis_indices_[3]);

    const double j5_negative = readButtonValue(joy_msg, button_negative_j5_index_);
    const double j5_positive = readButtonValue(joy_msg, button_positive_j5_index_);
    joint_msg.velocity[4] = joint_velocity_scale_ * (j5_positive - j5_negative);

    joint_pub_->publish(joint_msg);
  }

  void publishWorldCommand(const sensor_msgs::msg::Joy & joy_msg)
  {
    geometry_msgs::msg::Twist twist_msg;

    twist_msg.linear.x = linear_scale_ * readAxis(joy_msg, world_axis_indices_[0]);
    twist_msg.linear.y = linear_scale_ * readAxis(joy_msg, world_axis_indices_[1]);
    twist_msg.linear.z = linear_scale_ * readAxis(joy_msg, world_axis_indices_[2]);
    twist_msg.angular.y = pitch_scale_ * readAxis(joy_msg, world_axis_indices_[3]);

    const double roll_negative = readButtonValue(joy_msg, button_negative_roll_index_);
    const double roll_positive = readButtonValue(joy_msg, button_positive_roll_index_);
    twist_msg.angular.x = roll_scale_ * (roll_positive - roll_negative);
    twist_msg.angular.z = 0.0;

    twist_pub_->publish(twist_msg);
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
      publishWorldCommand(*msg);
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  std::vector<int64_t> joint_axis_indices_;
  std::vector<int64_t> world_axis_indices_;
  int64_t button_negative_j5_index_{10};
  int64_t button_positive_j5_index_{9};
  int64_t button_negative_roll_index_{10};
  int64_t button_positive_roll_index_{9};
  int64_t mode_toggle_button_index_{0};

  double joint_velocity_scale_{1.0};
  double linear_scale_{1.0};
  double pitch_scale_{1.0};
  double roll_scale_{1.0};

  std::string joint_topic_;
  std::string world_control_topic_;
  ControlMode mode_{ControlMode::kJoint};
  bool toggle_button_was_pressed_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyToDualControl>());
  rclcpp::shutdown();
  return 0;
}

