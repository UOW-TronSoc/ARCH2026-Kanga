#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>

class JoyToTwist : public rclcpp::Node
{
public:
    JoyToTwist() : Node("joy_to_twist")
    {
        // Publisher: Twist command
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Subscriber: Joystick
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&JoyToTwist::joyCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "joy_to_twist active");
    }

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "converting");

        if (msg->axes.size() < 3) {
            RCLCPP_WARN(this->get_logger(), "Not enough axes in /joy message");
            return;
        }

        geometry_msgs::msg::Twist twist;
        twist.linear.x  = msg->axes[0]; // axis 0 -> vx
        twist.linear.y  = msg->axes[2]; // axis 1 -> vy
        twist.angular.z = msg->axes[1]; // axis 2 -> wz

        cmd_pub_->publish(twist);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyToTwist>());
    rclcpp::shutdown();
    return 0;
}
