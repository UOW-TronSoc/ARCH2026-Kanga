#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <algorithm>
#include <cmath>

class TwistToPercent : public rclcpp::Node
{
public:
    TwistToPercent() : Node("twist_to_percent")
    {
        max_linear_x_  = this->declare_parameter<double>("max_linear_x", 0.4);
        max_linear_y_  = this->declare_parameter<double>("max_linear_y", 0.4);
        max_angular_z_ = this->declare_parameter<double>("max_angular_z", 2.2);

        sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&TwistToPercent::on_twist, this, std::placeholders::_1));

        pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/kanga_drive/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(),
            "twist_to_percent active  max_linear_x=%.3f  max_linear_y=%.3f  max_angular_z=%.3f",
            max_linear_x_, max_linear_y_, max_angular_z_);
    }

private:
    static double to_percent(double value, double max_abs)
    {
        if (max_abs <= 0.0) {
            return 0.0;
        }
        return std::clamp(value / max_abs * 100.0, -100.0, 100.0);
    }

    void on_twist(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        geometry_msgs::msg::Twist out;
        out.linear.x  = to_percent(msg->linear.x,  max_linear_x_);
        out.linear.y  = to_percent(msg->linear.y,  max_linear_y_);
        out.linear.z  = msg->linear.z;
        out.angular.x = msg->angular.x;
        out.angular.y = msg->angular.y;
        out.angular.z = to_percent(msg->angular.z, max_angular_z_);
        pub_->publish(out);
    }

    double max_linear_x_{0.4};
    double max_linear_y_{0.4};
    double max_angular_z_{2.2};

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwistToPercent>());
    rclcpp::shutdown();
    return 0;
}
