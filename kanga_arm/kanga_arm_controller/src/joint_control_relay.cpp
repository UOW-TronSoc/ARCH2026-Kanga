#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>
#include <mutex>
#include <algorithm>
#include <cmath>
#include <limits>

class JointControlRelay : public rclcpp::Node
{
public:
  JointControlRelay() : Node("kanga_arm_joint_control_relay")
  {
    init_pos_ = this->declare_parameter<std::vector<double>>(
      "joint_initial_positions",
      std::vector<double>(dof, 0.0));

    if (init_pos_.size() < dof) {
      RCLCPP_WARN(
        this->get_logger(),
        "joint_initial_positions size is less than %zu; padding with zeros",
        dof);
      init_pos_.resize(dof, 0.0);
    } else if (init_pos_.size() > dof) {
      init_pos_.resize(dof);
    }

    // The relay tracks its own desired position state by integrating velocity commands.
    current_position_ = init_pos_;
    feedback_position_ = init_pos_;
    latest_velocity_ = std::vector<double>(dof, 0.0);
    last_velocity_time_ = steady_clock_.now();

    joint_control_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/kanga_arm/joint_control", 10,
      std::bind(&JointControlRelay::jointControlCallback, this, std::placeholders::_1));

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      std::bind(&JointControlRelay::jointStateCallback, this, std::placeholders::_1));

    // Per-joint safety and command-shaping parameters.
    joint_min_limits_ = this->declare_parameter<std::vector<double>>(
      "joint_min_limits", std::vector<double>(dof, -std::numeric_limits<double>::infinity()));
    joint_max_limits_ = this->declare_parameter<std::vector<double>>(
      "joint_max_limits", std::vector<double>(dof, std::numeric_limits<double>::infinity()));
    joint_velocity_invert_ = this->declare_parameter<std::vector<bool>>(
      "joint_velocity_invert", std::vector<bool>(dof, false));
    joint_max_velocity_ = this->declare_parameter<std::vector<double>>(
      "joint_max_velocity", std::vector<double>(dof, std::numeric_limits<double>::infinity()));
    joint_max_velocity_direct_ = this->declare_parameter<std::vector<double>>(
      "joint_max_velocity_direct", std::vector<double>{});

    if (joint_min_limits_.size() < dof) {
      RCLCPP_WARN(this->get_logger(), "joint_min_limits has fewer than %zu entries; padding with -inf", dof);
      joint_min_limits_.resize(dof, -std::numeric_limits<double>::infinity());
    } else if (joint_min_limits_.size() > dof) {
      joint_min_limits_.resize(dof);
    }

    if (joint_max_limits_.size() < dof) {
      RCLCPP_WARN(this->get_logger(), "joint_max_limits has fewer than %zu entries; padding with +inf", dof);
      joint_max_limits_.resize(dof, std::numeric_limits<double>::infinity());
    } else if (joint_max_limits_.size() > dof) {
      joint_max_limits_.resize(dof);
    }

    if (joint_velocity_invert_.size() < dof) {
      RCLCPP_WARN(this->get_logger(), "joint_velocity_invert has fewer than %zu entries; padding with false", dof);
      joint_velocity_invert_.resize(dof, false);
    } else if (joint_velocity_invert_.size() > dof) {
      joint_velocity_invert_.resize(dof);
    }

    if (joint_max_velocity_.size() < dof) {
      RCLCPP_WARN(this->get_logger(), "joint_max_velocity has fewer than %zu entries; padding with +inf", dof);
      joint_max_velocity_.resize(dof, std::numeric_limits<double>::infinity());
    } else if (joint_max_velocity_.size() > dof) {
      joint_max_velocity_.resize(dof);
    }

    if (joint_max_velocity_direct_.empty()) {
      joint_max_velocity_direct_ = joint_max_velocity_;
    } else if (joint_max_velocity_direct_.size() < dof) {
      joint_max_velocity_direct_.resize(dof, std::numeric_limits<double>::infinity());
    } else if (joint_max_velocity_direct_.size() > dof) {
      joint_max_velocity_direct_.resize(dof);
    }

    for (size_t i = 0; i < dof; ++i) {
      if (joint_min_limits_[i] > joint_max_limits_[i]) {
        RCLCPP_WARN(
          this->get_logger(),
          "Invalid limits on joint %zu (min > max). Swapping values.",
          i);
        std::swap(joint_min_limits_[i], joint_max_limits_[i]);
      }
      if (joint_max_velocity_direct_[i] < 0.0) {
        RCLCPP_WARN(
          this->get_logger(),
          "joint_max_velocity_direct[%zu] is negative. Using absolute value.",
          i);
        joint_max_velocity_direct_[i] = std::abs(joint_max_velocity_direct_[i]);
      }
      current_position_[i] = std::clamp(current_position_[i], joint_min_limits_[i], joint_max_limits_[i]);
    }

    desired_control_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "joint_desired_control", 10);

    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 100.0);
    const double control_time_step_ms = this->declare_parameter<double>("control_time_step_ms", 10.0);
    max_velocity_slew_rate_ = this->declare_parameter<double>("max_velocity_slew_rate", 8.0);
    hold_position_on_stale_ = this->declare_parameter<bool>("hold_position_on_stale", true);
    command_stale_timeout_s_ = this->declare_parameter<double>("command_stale_timeout_s", 0.5);
    const double safe_rate_hz = (publish_rate_hz_ > 0.0) ? publish_rate_hz_ : 100.0;
    if (publish_rate_hz_ > 0.0) {
      publish_period_seconds_ = 1.0 / safe_rate_hz;
    } else {
      // Fallback path keeps compatibility with older config that specifies control_time_step_ms.
      const double safe_control_time_step_ms = std::max(control_time_step_ms, 1.0);
      publish_period_seconds_ = safe_control_time_step_ms * 1e-3;
      publish_rate_hz_ = 1.0 / publish_period_seconds_;
    }

    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(publish_period_seconds_)),
      std::bind(&JointControlRelay::publishDesiredControl, this));

    RCLCPP_INFO(this->get_logger(), "Kanga Arm joint control relay started");
  }

private:
  // Stores latest commanded velocity for each joint.
  void jointControlCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    const size_t controlled_dof = active_dof_;
    if (msg->velocity.size() < controlled_dof) {
      RCLCPP_WARN(
        this->get_logger(),
        "Joint control message has fewer than %zu velocities; missing entries treated as zero",
        controlled_dof);
    }

    for (size_t i = 0; i < controlled_dof; ++i) {
      double velocity = (msg->velocity.size() > i) ? msg->velocity[i] : 0.0;
      if (joint_velocity_invert_[i]) {
        velocity = -velocity;
      }
      velocity = std::clamp(velocity, -joint_max_velocity_direct_[i], joint_max_velocity_direct_[i]);
      latest_velocity_[i] = velocity;
    }
    for (size_t i = controlled_dof; i < dof; ++i) {
      latest_velocity_[i] = 0.0;
    }
    last_velocity_time_ = steady_clock_.now();
  }

  // Updates measured joint feedback used for limit checks and command anchoring.
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (!msg || msg->position.empty()) {
      return;
    }

    const size_t reported_dof = std::min(dof, msg->position.size());
    std::lock_guard<std::mutex> lock(command_mutex_);
    active_dof_ = std::max<size_t>(1, reported_dof);
    have_feedback_ = true;

    for (size_t i = 0; i < reported_dof; ++i) {
      feedback_position_[i] = msg->position[i];
    }
  }

  // Periodic control loop: filter command, integrate desired position, clamp limits, publish.
  void publishDesiredControl()
  {
    const rclcpp::Time now = steady_clock_.now();
    const double dt = publish_period_seconds_;

    size_t controlled_dof = dof;
    std::vector<double> raw_velocity;
    bool is_stale = true;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      controlled_dof = active_dof_;
      raw_velocity.assign(controlled_dof, 0.0);
      const double stale_seconds = (now - last_velocity_time_).seconds();
      is_stale = stale_seconds > command_stale_timeout_s_;
      if (is_stale) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          1000,
          "Watchdog triggered: command stream stale for %.3f s (timeout=%.3f s). Holding position=%s",
          stale_seconds,
          command_stale_timeout_s_,
          hold_position_on_stale_ ? "true" : "false");
      }
      if (!is_stale) {
        raw_velocity.assign(latest_velocity_.begin(), latest_velocity_.begin() + controlled_dof);
      }
    }

    if (is_stale && hold_position_on_stale_) {
      // Immediate hold: stop target motion when upstream commands disappear.
      std::fill(filtered_velocity_.begin(), filtered_velocity_.begin() + controlled_dof, 0.0);
    }

    std::vector<double> velocity(controlled_dof, 0.0);
    const double max_step = std::max(0.0, max_velocity_slew_rate_) * dt;
    for (size_t i = 0; i < controlled_dof; ++i) {
      if (is_stale && hold_position_on_stale_) {
        velocity[i] = 0.0;
        continue;
      }

      // Slew-rate limiting smooths step-like joystick inputs.
      const double dv = raw_velocity[i] - filtered_velocity_[i];
      if (dv > max_step) {
        filtered_velocity_[i] += max_step;
      } else if (dv < -max_step) {
        filtered_velocity_[i] -= max_step;
      } else {
        filtered_velocity_[i] = raw_velocity[i];
      }

      filtered_velocity_[i] = std::clamp(
        filtered_velocity_[i], -joint_max_velocity_direct_[i], joint_max_velocity_direct_[i]);
      velocity[i] = filtered_velocity_[i];
      const double measured_position_limit_frame = have_feedback_
        ? feedback_position_[i]
        : (joint_velocity_invert_[i] ? -current_position_[i] : current_position_[i]);
      current_position_[i] += velocity[i] * dt;

      // Keep desired position inside configured hard limits.
      current_position_[i] = std::clamp(current_position_[i], joint_min_limits_[i], joint_max_limits_[i]);

      // Evaluate anti-windup in joint-limit frame (before velocity sign inversion).
      const double velocity_limit_frame = joint_velocity_invert_[i] ? -velocity[i] : velocity[i];
      if ((measured_position_limit_frame <= joint_min_limits_[i] && velocity_limit_frame < 0.0) ||
          (measured_position_limit_frame >= joint_max_limits_[i] && velocity_limit_frame > 0.0)) {
        velocity[i] = 0.0;
        filtered_velocity_[i] = 0.0;
      }
    }

    sensor_msgs::msg::JointState out_msg;
    out_msg.header.stamp = this->get_clock()->now();
    out_msg.position.resize(controlled_dof);
    out_msg.velocity.resize(controlled_dof);
    out_msg.effort.resize(controlled_dof, 0.0);

    for (size_t i = 0; i < controlled_dof; ++i) {
      out_msg.position[i] = current_position_[i];
      out_msg.velocity[i] = velocity[i];
    }

    desired_control_pub_->publish(out_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_control_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr desired_control_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  const size_t dof = 5;
  std::vector<double> init_pos_;
  std::vector<double> current_position_;
  std::vector<double> feedback_position_;
  std::vector<double> latest_velocity_;
  std::vector<double> filtered_velocity_{std::vector<double>(dof, 0.0)};
  std::vector<double> joint_min_limits_;
  std::vector<double> joint_max_limits_;
  std::vector<bool> joint_velocity_invert_;
  std::vector<double> joint_max_velocity_;
  std::vector<double> joint_max_velocity_direct_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Time last_velocity_time_;
  std::mutex command_mutex_;
  double publish_rate_hz_{100.0};
  double publish_period_seconds_{0.01};
  double max_velocity_slew_rate_{8.0};
  bool hold_position_on_stale_{true};
  double command_stale_timeout_s_{0.5};
  size_t active_dof_{dof};
  bool have_feedback_{false};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointControlRelay>());
  rclcpp::shutdown();
  return 0;
}
