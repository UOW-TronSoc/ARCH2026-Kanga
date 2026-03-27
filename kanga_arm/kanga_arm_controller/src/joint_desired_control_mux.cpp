#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

/**
 * @brief MUX node that selects joint_desired_control from joint or EE controller.
 *
 * Subscribes to:
 * - joint_desired_control_from_joint (joint_control_relay output)
 * - joint_desired_control_from_ee (control_node output)
 * - kanga_arm/control_mode_joint (true = joint mode, false = EE mode)
 *
 * Publishes to joint_desired_control based on the mode.
 */
class JointDesiredControlMux : public rclcpp::Node
{
public:
  JointDesiredControlMux()
  : Node("joint_desired_control_mux")
  {
    joint_input_topic_ = this->declare_parameter<std::string>(
      "joint_desired_control_from_joint", "joint_desired_control_from_joint");
    ee_input_topic_ = this->declare_parameter<std::string>(
      "joint_desired_control_from_ee", "joint_desired_control_from_ee");
    mode_topic_ = this->declare_parameter<std::string>(
      "mode_topic", "kanga_arm/control_mode_joint");
    output_topic_ = this->declare_parameter<std::string>(
      "output_topic", "joint_desired_control");
    default_joint_mode_ = this->declare_parameter<bool>("default_joint_mode", true);

    use_joint_mode_ = default_joint_mode_;

    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_input_topic_, 10,
      std::bind(&JointDesiredControlMux::jointCallback, this, std::placeholders::_1));
    ee_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      ee_input_topic_, 10,
      std::bind(&JointDesiredControlMux::eeCallback, this, std::placeholders::_1));
    mode_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      mode_topic_, 10,
      std::bind(&JointDesiredControlMux::modeCallback, this, std::placeholders::_1));

    output_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(output_topic_, 10);

    RCLCPP_INFO(
      this->get_logger(),
      "joint_desired_control_mux active: mode=%s, %s | %s -> %s",
      use_joint_mode_ ? "joint" : "ee",
      joint_input_topic_.c_str(),
      ee_input_topic_.c_str(),
      output_topic_.c_str());
  }

private:
  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    latest_joint_msg_ = msg;
    if (use_joint_mode_) {
      output_pub_->publish(*msg);
    }
  }

  void eeCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    latest_ee_msg_ = msg;
    if (!use_joint_mode_) {
      output_pub_->publish(*msg);
    }
  }

  void modeCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    const bool was_joint = use_joint_mode_;
    use_joint_mode_ = msg->data;
    if (was_joint != use_joint_mode_) {
      publish_zero_hold();
      RCLCPP_INFO(
        this->get_logger(),
        "Mode switched to: %s",
        use_joint_mode_ ? "joint" : "ee");
    }
  }

  void publish_zero_hold()
  {
    sensor_msgs::msg::JointState zero_msg;
    zero_msg.header.stamp = this->get_clock()->now();
    zero_msg.name = {"arm_j1", "arm_j2", "arm_j3", "arm_j4", "arm_j5"};
    zero_msg.position.resize(5, 0.0);
    zero_msg.velocity.resize(5, 0.0);
    zero_msg.effort.resize(5, 0.0);
    output_pub_->publish(zero_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr ee_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mode_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr output_pub_;

  std::string joint_input_topic_;
  std::string ee_input_topic_;
  std::string mode_topic_;
  std::string output_topic_;
  bool default_joint_mode_;
  bool use_joint_mode_;

  sensor_msgs::msg::JointState::SharedPtr latest_joint_msg_;
  sensor_msgs::msg::JointState::SharedPtr latest_ee_msg_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointDesiredControlMux>());
  rclcpp::shutdown();
  return 0;
}
