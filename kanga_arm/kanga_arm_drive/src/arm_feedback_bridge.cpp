#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "kanga_interfaces/msg/controller_status.hpp"

namespace
{

struct OdriveConfig
{
  std::string ns;
  bool invert{false};
  double reduction{1.0};
};

std::string trim(const std::string & input)
{
  const auto first = std::find_if_not(
    input.begin(), input.end(), [](unsigned char ch) { return std::isspace(ch); });
  if (first == input.end()) {
    return "";
  }

  const auto last = std::find_if_not(
    input.rbegin(), input.rend(), [](unsigned char ch) { return std::isspace(ch); }).base();

  return std::string(first, last);
}

bool starts_with(const std::string & text, const std::string & prefix)
{
  return text.size() >= prefix.size() && text.compare(0, prefix.size(), prefix) == 0;
}

std::string strip_quotes(std::string value)
{
  if (value.size() >= 2U && value.front() == '"' && value.back() == '"') {
    value = value.substr(1, value.size() - 2U);
  }
  return value;
}

bool parse_bool(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

  return value == "true" || value == "1" || value == "yes";
}

bool parse_double(const std::string & value, double & out)
{
  try {
    size_t parsed_chars = 0;
    out = std::stod(value, &parsed_chars);
    return parsed_chars == value.size();
  } catch (...) {
    return false;
  }
}

std::vector<OdriveConfig> load_configs_from_yaml(const rclcpp::Logger & logger)
{
  std::vector<OdriveConfig> configs;

  std::string config_path;
  try {
    config_path = ament_index_cpp::get_package_share_directory("kanga_arm_drive") +
      "/config/odrive_node_ids_arm.yaml";
  } catch (const std::exception & ex) {
    RCLCPP_WARN(logger, "Unable to locate arm odrive configuration: %s", ex.what());
    return configs;
  }

  std::ifstream config_stream(config_path);
  if (!config_stream.is_open()) {
    RCLCPP_WARN(logger, "Unable to open arm odrive configuration file: %s", config_path.c_str());
    return configs;
  }

  std::string line;
  bool in_nodes = false;
  int nodes_indent = -1;
  OdriveConfig current;
  bool current_valid = false;

  auto push_current = [&]() {
      if (!current.ns.empty()) {
        if (current.reduction <= 0.0) {
          RCLCPP_WARN(
            logger,
            "Invalid reduction %.3f for '%s'; using 1.0",
            current.reduction,
            current.ns.c_str());
          current.reduction = 1.0;
        }
        configs.push_back(current);
      }
      current = OdriveConfig();
      current_valid = false;
    };

  while (std::getline(config_stream, line)) {
    const auto first_non_space = line.find_first_not_of(' ');
    if (first_non_space == std::string::npos) {
      continue;
    }

    const int indent = static_cast<int>(first_non_space);
    const auto trimmed = trim(line);
    if (trimmed.empty()) {
      continue;
    }

    if (!in_nodes) {
      if (trimmed == "odrive_nodes:") {
        in_nodes = true;
        nodes_indent = indent;
      }
      continue;
    }

    if (indent <= nodes_indent && trimmed.rfind('-', 0) != 0) {
      push_current();
      break;
    }

    if (trimmed == "-") {
      push_current();
      current_valid = true;
      continue;
    }

    if (starts_with(trimmed, "- namespace:")) {
      push_current();
      current_valid = true;
      current.ns = strip_quotes(trim(trimmed.substr(std::string("- namespace:").size())));
      continue;
    }

    if (!current_valid) {
      continue;
    }

    if (starts_with(trimmed, "namespace:")) {
      current.ns = strip_quotes(trim(trimmed.substr(std::string("namespace:").size())));
      continue;
    }

    if (starts_with(trimmed, "invert:")) {
      current.invert = parse_bool(trim(trimmed.substr(std::string("invert:").size())));
      continue;
    }

    if (starts_with(trimmed, "reduction:")) {
      const auto reduction_text = trim(trimmed.substr(std::string("reduction:").size()));
      double parsed = 1.0;
      if (parse_double(reduction_text, parsed)) {
        current.reduction = parsed;
      } else {
        RCLCPP_WARN(
          logger,
          "Could not parse reduction value '%s'; using 1.0",
          reduction_text.c_str());
        current.reduction = 1.0;
      }
      continue;
    }
  }

  push_current();

  return configs;
}

}  // namespace

class ArmFeedbackBridge : public rclcpp::Node
{
public:
  ArmFeedbackBridge()
  : Node("arm_feedback_bridge")
  {
    const auto configs = load_configs_from_yaml(this->get_logger());
    if (configs.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No odrive nodes configured; feedback bridge inactive");
      return;
    }

    const double publish_rate_hz = this->declare_parameter<double>("publish_rate_hz", 500.0);
    frame_id_ = this->declare_parameter<std::string>("frame_id", "");

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    axes_.reserve(configs.size());
    subs_.reserve(configs.size());

    for (const auto & cfg : configs) {
      AxisData axis;
      axis.name = cfg.ns;
      axis.reduction = static_cast<float>(std::max(1.0e-6, cfg.reduction));
      axis.sign = cfg.invert ? -1.0F : 1.0F;
      axes_.push_back(axis);

      const auto topic = "/" + cfg.ns + "/controller_status";
      subs_.push_back(this->create_subscription<kanga_interfaces::msg::ControllerStatus>(
        topic, 10,
        [this, index = axes_.size() - 1](
          const kanga_interfaces::msg::ControllerStatus::SharedPtr msg) {
          this->status_callback(index, msg);
        }));
    }

    const double safe_rate_hz = publish_rate_hz > 0.0 ? publish_rate_hz : 500.0;
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / safe_rate_hz)),
      std::bind(&ArmFeedbackBridge::publish_joint_states, this));

    RCLCPP_INFO(
      this->get_logger(),
      "arm_feedback_bridge active: %zu axes -> joint_states at %.1f Hz",
      axes_.size(), safe_rate_hz);
  }

private:
  struct AxisData
  {
    std::string name;
    float reduction{1.0F};
    float sign{1.0F};
    float position{0.0F};
    float velocity{0.0F};
    float effort{0.0F};
    bool seen{false};
  };

  void status_callback(size_t index, const kanga_interfaces::msg::ControllerStatus::SharedPtr msg)
  {
    if (index >= axes_.size() || !msg) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    auto & axis = axes_[index];
    const float inv_reduction = 1.0F / axis.reduction;
    axis.position = axis.sign * static_cast<float>(msg->pos_estimate) * inv_reduction;
    axis.velocity = axis.sign * static_cast<float>(msg->vel_estimate) * inv_reduction;
    axis.effort = axis.sign * static_cast<float>(msg->torque_estimate);
    axis.seen = true;
  }

  void publish_joint_states()
  {
    if (!joint_state_pub_ || axes_.empty()) {
      return;
    }

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = frame_id_;
    msg.name.reserve(axes_.size());
    msg.position.reserve(axes_.size());
    msg.velocity.reserve(axes_.size());
    msg.effort.reserve(axes_.size());

    size_t unseen_count = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto & axis : axes_) {
        if (!axis.seen) {
          ++unseen_count;
        }
        msg.name.push_back(axis.name);
        msg.position.push_back(axis.position);
        msg.velocity.push_back(axis.velocity);
        msg.effort.push_back(axis.effort);
      }
    }

    if (unseen_count > 0) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Waiting for controller_status from %zu/%zu axes",
        unseen_count, axes_.size());
    }

    joint_state_pub_->publish(msg);
  }

  std::mutex mutex_;
  std::string frame_id_;
  std::vector<AxisData> axes_;
  std::vector<rclcpp::Subscription<kanga_interfaces::msg::ControllerStatus>::SharedPtr> subs_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmFeedbackBridge>());
  rclcpp::shutdown();
  return 0;
}
