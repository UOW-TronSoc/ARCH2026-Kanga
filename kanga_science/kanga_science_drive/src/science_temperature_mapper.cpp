#include <algorithm>
#include <cstdint>
#include <functional>
#include <string>

#include <can_msgs/msg/frame.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>

// CAN IDs must match Science_RTOS.ino
static constexpr uint32_t CAN_ID_AUGER_ACTUATOR  = 821U;
static constexpr uint32_t CAN_ID_HEATING_CONTROL = 824U;
static constexpr uint32_t CAN_ID_COOLING_CONTROL = 829U;

class ScienceTemperatureMapper : public rclcpp::Node
{
public:
  ScienceTemperatureMapper()
  : Node("science_temperature_mapper")
  {
    can_interface_ = this->declare_parameter<std::string>("can_interface", "can2");

    if (can_interface_.empty()) {
      RCLCPP_WARN(this->get_logger(), "Parameter 'can_interface' is empty; defaulting to 'can2'");
      can_interface_ = "can2";

    }

    const std::string can_tx_topic = "CAN/" + can_interface_ + "/transmit";
    can_pub_ = this->create_publisher<can_msgs::msg::Frame>(can_tx_topic, 10);

    heating_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/kanga_science/heating", 10,
      std::bind(&ScienceTemperatureMapper::heating_callback, this, std::placeholders::_1));

    cooling_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/kanga_science/cooling", 10,
      std::bind(&ScienceTemperatureMapper::cooling_callback, this, std::placeholders::_1));

    actuator_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/kanga_science/linear_actuator_speed", 10,
      std::bind(&ScienceTemperatureMapper::actuator_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "science_temperature_mapper active -> %s  (heating: ID %u, cooling: ID %u, actuator: ID %u)",
      can_tx_topic.c_str(),
      CAN_ID_HEATING_CONTROL,
      CAN_ID_COOLING_CONTROL,
      CAN_ID_AUGER_ACTUATOR);
  }

private:
  void actuator_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    // Positive = forward (dir 0), negative = reverse (dir 1), magnitude = speed 0-255
    const uint8_t direction = (msg->data < 0) ? 1U : 0U;
    const uint8_t speed = static_cast<uint8_t>(std::clamp(std::abs(msg->data), 0, 255));

    if (last_actuator_speed_ == speed && last_actuator_dir_ == direction) {
      return;
    }
    last_actuator_speed_ = speed;
    last_actuator_dir_ = direction;

    can_msgs::msg::Frame frame;
    frame.header.stamp = this->now();
    frame.id = CAN_ID_AUGER_ACTUATOR;
    frame.is_rtr = false;
    frame.is_extended = false;
    frame.is_error = false;
    frame.dlc = 2;
    frame.data[0] = direction;
    frame.data[1] = speed;
    can_pub_->publish(frame);

    RCLCPP_INFO(this->get_logger(), "Actuator dir: %u  speed: %u", direction, speed);
  }

  void publish_frame(uint32_t can_id, bool state)
  {
    can_msgs::msg::Frame msg;
    msg.header.stamp = this->now();
    msg.id = can_id;
    msg.is_rtr = false;
    msg.is_extended = false;
    msg.is_error = false;
    msg.dlc = 1;
    msg.data[0] = state ? 1U : 0U;
    can_pub_->publish(msg);
  }

  void heating_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (last_heating_state_ == msg->data) {
      return;
    }
    last_heating_state_ = msg->data;
    publish_frame(CAN_ID_HEATING_CONTROL, msg->data);
    RCLCPP_INFO(this->get_logger(), "Heating -> %s", msg->data ? "ON" : "OFF");
  }

  void cooling_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (last_cooling_state_ == msg->data) {
      return;
    }
    last_cooling_state_ = msg->data;
    publish_frame(CAN_ID_COOLING_CONTROL, msg->data);
    RCLCPP_INFO(this->get_logger(), "Cooling -> %s", msg->data ? "ON" : "OFF");
  }

  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr heating_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cooling_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr actuator_sub_;

  std::string can_interface_;
  bool last_heating_state_{false};
  bool last_cooling_state_{false};
  uint8_t last_actuator_speed_{255U};  // sentinel so first message always sends
  uint8_t last_actuator_dir_{255U};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScienceTemperatureMapper>());
  rclcpp::shutdown();
  return 0;
}
