#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <array>

#include "kanga_interfaces/msg/control_message.hpp"

class WheelCommandMapper : public rclcpp::Node
{
public:
    WheelCommandMapper() : Node("wheel_command_mapper")
    {
        wheel_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/core/wheel_motor_cmds", 10);
        test_pub_  = this->create_publisher<kanga_interfaces::msg::ControlMessage>("/odrive_axis0/control_message", 10);

        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&WheelCommandMapper::twistCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "wheel_command_mapper active");
    }

private:
    // Placeholder: turn Twist into 4 wheel commands [FL, FR, RL, RR]
    std::array<float, 4> computeWheelCommands(const geometry_msgs::msg::Twist &tw)
    {
        // TODO: replace with your real kinematics later
        // For now just pass through linear.x to all four
        float v = static_cast<float>(tw.linear.x);
        return {v, v, v, v};
    }

    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        auto cmds = computeWheelCommands(*msg);

        std_msgs::msg::Float32MultiArray out;
        out.data.resize(4);
        out.data[0] = cmds[0]; // front left
        out.data[1] = cmds[1]; // front right
        out.data[2] = cmds[2]; // rear left
        out.data[3] = cmds[3]; // rear right

        wheel_pub_->publish(out);

        // Also publish a temporary ControlMessage
        kanga_interfaces::msg::ControlMessage ctrl;
        ctrl.control_mode = 2;
        ctrl.input_mode   = 1;
        ctrl.input_pos    = 0.0;
        ctrl.input_vel    = 20.0 * cmds[0];
        ctrl.input_torque = 0.0;

        test_pub_->publish(ctrl);
    }

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_pub_;
    rclcpp::Publisher<kanga_interfaces::msg::ControlMessage>::SharedPtr test_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelCommandMapper>());
    rclcpp::shutdown();
    return 0;
}
