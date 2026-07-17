#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <kanga_interfaces/srv/axis_state.hpp>

static constexpr uint32_t AXIS_STATE_IDLE               = 1;
static constexpr uint32_t AXIS_STATE_CLOSED_LOOP_CONTROL = 8;

static constexpr int BUTTON_A = 0;  // disable drive
static constexpr int BUTTON_B = 1;  // enable drive

class JoyDriveEnable : public rclcpp::Node
{
public:
    JoyDriveEnable() : Node("joy_drive_enable")
    {
        const std::vector<std::string> wheel_namespaces = {
            "wheel_fl", "wheel_bl", "wheel_br", "wheel_fr"
        };

        for (const auto & ns : wheel_namespaces) {
            auto client = this->create_client<kanga_interfaces::srv::AxisState>(
                "/" + ns + "/request_axis_state");
            axis_clients_.push_back(client);
        }

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&JoyDriveEnable::joyCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "joy_drive_enable active — A: disable drive, B: enable drive");
    }

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (static_cast<int>(msg->buttons.size()) <= BUTTON_B) {
            return;
        }

        bool a_pressed = (msg->buttons[BUTTON_A] == 1) && (prev_buttons_.size() > BUTTON_A) && (prev_buttons_[BUTTON_A] == 0);
        bool b_pressed = (msg->buttons[BUTTON_B] == 1) && (prev_buttons_.size() > BUTTON_B) && (prev_buttons_[BUTTON_B] == 0);

        if (a_pressed) {
            RCLCPP_INFO(this->get_logger(), "A pressed — setting all drives to IDLE");
            setAllAxes(AXIS_STATE_IDLE);
        } else if (b_pressed) {
            RCLCPP_INFO(this->get_logger(), "B pressed — setting all drives to CLOSED_LOOP_CONTROL");
            setAllAxes(AXIS_STATE_CLOSED_LOOP_CONTROL);
        }

        prev_buttons_ = msg->buttons;
    }

    void setAllAxes(uint32_t state)
    {
        for (auto & client : axis_clients_) {
            if (!client->service_is_ready()) {
                RCLCPP_WARN(this->get_logger(), "Axis state service not ready, skipping");
                continue;
            }
            auto request = std::make_shared<kanga_interfaces::srv::AxisState::Request>();
            request->axis_requested_state = state;
            client->async_send_request(request);
        }
    }

    std::vector<rclcpp::Client<kanga_interfaces::srv::AxisState>::SharedPtr> axis_clients_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    std::vector<int32_t> prev_buttons_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyDriveEnable>());
    rclcpp::shutdown();
    return 0;
}
