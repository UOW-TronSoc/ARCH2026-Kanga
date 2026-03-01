#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

class JoyToWorldStateControl : public rclcpp::Node
{
public:
  JoyToWorldStateControl()
  : Node("joy_to_world_state_control")
  {
    axis_indices_ = this->declare_parameter<std::vector<int64_t>>(
      "axis_indices", std::vector<int64_t>{0, 1, 2, 3});
    button_negative_roll_index_ = this->declare_parameter<int64_t>("button_negative_roll", 10);
    button_positive_roll_index_ = this->declare_parameter<int64_t>("button_positive_roll", 9);

    linear_scale_ = this->declare_parameter<double>("linear_scale", 1.0);
    pitch_scale_ = this->declare_parameter<double>("pitch_scale", 1.0);
    roll_scale_ = this->declare_parameter<double>("roll_scale", 1.0);

    world_control_topic_ = this->declare_parameter<std::string>(
      "world_control_topic", "kanga_arm/world_state_control");

    if (axis_indices_.size() < 4) {
      RCLCPP_WARN(
        this->get_logger(),
        "axis_indices has fewer than 4 entries; missing commands will be zero");
      axis_indices_.resize(4, -1);
    } else if (axis_indices_.size() > 4) {
      axis_indices_.resize(4);
    }

    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(world_control_topic_, 10);
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&JoyToWorldStateControl::joyCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "joy_to_world_state_control active: /joy -> %s", world_control_topic_.c_str());
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
    geometry_msgs::msg::Twist twist_msg;

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

    twist_pub_->publish(twist_msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  std::vector<int64_t> axis_indices_;
  int64_t button_negative_roll_index_{10};
  int64_t button_positive_roll_index_{9};
  double linear_scale_{1.0};
  double pitch_scale_{1.0};
  double roll_scale_{1.0};
  std::string world_control_topic_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyToWorldStateControl>());
  rclcpp::shutdown();
  return 0;
}

