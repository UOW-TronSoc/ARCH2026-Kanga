#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <memory>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "kanga_interfaces/msg/control_message.hpp"
#include "kanga_interfaces/srv/axis_state.hpp"

namespace
{

struct OdriveConfig
{
  std::string ns;
  bool invert{false};
  double reduction{1.0};
};

/**
 * @brief Remove leading and trailing whitespace from a string.
 * Inputs:
 * - input: Source string that may include surrounding whitespace.
 * Output:
 * - Returns a trimmed copy of input.
 */
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

/**
 * @brief Check if a string starts with a given prefix.
 * Inputs:
 * - text: Full string to inspect.
 * - prefix: Prefix to compare against the start of text.
 * Output:
 * - Returns true when text begins with prefix; otherwise false.
 */
bool starts_with(const std::string & text, const std::string & prefix)
{
  return text.size() >= prefix.size() && text.compare(0, prefix.size(), prefix) == 0;
}

/**
 * @brief Strip surrounding double quotes from a string value.
 * Inputs:
 * - value: String that may be wrapped in quotes.
 * Output:
 * - Returns value without outer quotes when present; otherwise unchanged.
 */
std::string strip_quotes(std::string value)
{
  if (value.size() >= 2U && value.front() == '"' && value.back() == '"') {
    value = value.substr(1, value.size() - 2U);
  }
  return value;
}

/**
 * @brief Parse a boolean string in a permissive way.
 * Inputs:
 * - value: Text such as "true", "1", or "yes" (case-insensitive).
 * Output:
 * - Returns true for recognized true values; otherwise false.
 */
bool parse_bool(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

  return value == "true" || value == "1" || value == "yes";
}

/**
 * @brief Parse a double value from text.
 * Inputs:
 * - value: String expected to contain a floating-point number.
 * - out: Reference where the parsed value is written on success.
 * Output:
 * - Returns true if parsing succeeds and consumes the full string; otherwise false.
 */
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

/**
 * @brief Load arm ODrive configuration entries from YAML.
 * Inputs:
 * - logger: ROS logger for warnings during parsing/validation.
 * Output:
 * - Returns parsed OdriveConfig entries, or fallback defaults if none are found.
 */
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

  if (configs.empty()) {
    RCLCPP_WARN(logger, "No ODrive namespaces found in %s; falling back to defaults", config_path.c_str());
    configs.push_back({"arm_j1", false, 1.0});
    configs.push_back({"arm_j2", false, 1.0});
    configs.push_back({"arm_j3", false, 1.0});
    configs.push_back({"arm_j4", false, 1.0});
  }

  return configs;
}

}  // namespace

class ArmCommandMapper : public rclcpp::Node
{
public:
  /**
   * @brief Construct and initialize the arm command mapper node.
   * Inputs:
   * - ROS parameters: axis_requested_state, max_joint_velocity, controlled_joint_names.
   * - YAML configuration loaded by load_configs_from_yaml().
   * Output:
   * - Creates publishers, service clients, timer, and subscriber needed for mapping commands.
   */
  ArmCommandMapper()
  : Node("arm_command_mapper")
  {
    const int axis_state_param = this->declare_parameter<int>("axis_requested_state", 8);
    if (axis_state_param < 0) {
      RCLCPP_WARN(
        this->get_logger(),
        "axis_requested_state parameter must be non-negative; clamping to 0");
      requested_axis_state_ = 0U;
    } else {
      requested_axis_state_ = static_cast<uint32_t>(axis_state_param);
    }

    max_joint_velocity_ = this->declare_parameter<double>("max_joint_velocity", 10.0);

    controlled_joint_names_ = this->declare_parameter<std::vector<std::string>>(
      "controlled_joint_names", std::vector<std::string>{"arm_j1", "arm_j2", "arm_j3", "arm_j4"});

    auto configs = load_configs_from_yaml(this->get_logger());

    for (const auto & cfg : configs) {
      if (cfg.ns.empty()) {
        continue;
      }

      ctrl_publishers_.push_back(
        this->create_publisher<kanga_interfaces::msg::ControlMessage>(
          "/" + cfg.ns + "/control_message", 10));
      invert_flags_.push_back(cfg.invert);
      reduction_factors_.push_back(static_cast<float>(cfg.reduction));
      axis_names_.push_back(cfg.ns);

      const auto service_name = "/" + cfg.ns + "/request_axis_state";
      axis_state_service_names_.push_back(service_name);
      axis_state_clients_.push_back(
        this->create_client<kanga_interfaces::srv::AxisState>(service_name));
      axis_state_ready_.push_back(false);
      axis_state_request_sent_.push_back(false);
    }

    if (ctrl_publishers_.empty()) {
      RCLCPP_WARN(
        this->get_logger(),
        "No valid arm ODrive configurations found; control publishers will remain inactive");
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "Configured %zu arm ODrive control publishers", ctrl_publishers_.size());
    }

    axis_state_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&ArmCommandMapper::tick_axis_state_requests, this));

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_desired_control", 10,
      std::bind(&ArmCommandMapper::joint_state_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "arm_command_mapper active");
  }

  /**
   * @brief Register a shutdown hook that requests idle state on all axes.
   * Inputs:
   * - None directly; uses internal service configuration.
   * Output:
   * - Installs a one-time shutdown callback.
   */
  void register_shutdown_hook()
  {
    if (shutdown_hook_registered_) {
      return;
    }

    auto shared_self = std::static_pointer_cast<ArmCommandMapper>(this->shared_from_this());
    if (!shared_self) {
      RCLCPP_WARN(this->get_logger(), "Failed to register shutdown hook: shared_from_this cast failed");
      return;
    }

    std::weak_ptr<ArmCommandMapper> weak_self = shared_self;
    rclcpp::on_shutdown([weak_self]() {
      if (auto self = weak_self.lock()) {
        std::thread([self]() {
            try {
              self->command_axes_state(1U, "shutdown");
            } catch (const std::exception & ex) {
              RCLCPP_WARN(self->get_logger(), "Shutdown handler exception: %s", ex.what());
            }
          }).detach();
      }
    });

    shutdown_hook_registered_ = true;
  }

  /**
   * @brief Immediately request a target state for all configured axes.
   * Inputs:
   * - target_state: Axis state value to request.
   * - context: Log context string describing why the request was sent.
   * Output:
   * - Triggers command_axes_state() for all axes.
   */
  void request_axes_state_now(uint32_t target_state, const char * context)
  {
    command_axes_state(target_state, context);
  }

private:
  using AxisState = kanga_interfaces::srv::AxisState;

  /**
   * @brief Extract one joint velocity from a JointState message.
   * Inputs:
   * - msg: Incoming JointState message.
   * - joint_name: Joint name to search for.
   * - fallback_index: Velocity index used if name lookup fails.
   * - out_vel: Reference set to extracted velocity on success.
   * Output:
   * - Returns true if velocity is found and written to out_vel; otherwise false.
   */
  bool extract_joint_velocity(
    const sensor_msgs::msg::JointState & msg,
    const std::string & joint_name,
    size_t fallback_index,
    float & out_vel) const
  {
    if (!msg.name.empty() && msg.name.size() == msg.velocity.size()) {
      const auto it = std::find(msg.name.begin(), msg.name.end(), joint_name);
      if (it != msg.name.end()) {
        const size_t idx = static_cast<size_t>(std::distance(msg.name.begin(), it));
        out_vel = static_cast<float>(msg.velocity[idx]);
        return true;
      }
    }

    if (fallback_index < msg.velocity.size()) {
      out_vel = static_cast<float>(msg.velocity[fallback_index]);
      return true;
    }

    return false;
  }

  /**
   * @brief Timer callback that requests target axis state for axes not yet ready.
   * Inputs:
   * - None directly; reads internal client and ready/request flags.
   * Output:
   * - Sends async service requests and cancels timer when all axes are ready.
   */
  void tick_axis_state_requests()
  {
    if (axis_state_clients_.empty()) {
      return;
    }

    bool all_ready = true;

    for (size_t i = 0; i < axis_state_clients_.size(); ++i) {
      if (axis_state_ready_[i]) {
        continue;
      }

      all_ready = false;

      auto & client = axis_state_clients_[i];
      if (!client->service_is_ready()) {
        continue;
      }

      if (!axis_state_request_sent_[i]) {
        auto request = std::make_shared<AxisState::Request>();
        request->axis_requested_state = requested_axis_state_;

        auto future = client->async_send_request(
          request,
          [this, index = i](rclcpp::Client<AxisState>::SharedFuture future_response) {
            this->handle_axis_state_response(index, future_response);
          });
        (void)future;

        axis_state_request_sent_[i] = true;
        RCLCPP_INFO(
          this->get_logger(),
          "Requested state %u for %s",
          requested_axis_state_,
          axis_state_service_names_[i].c_str());
      }
    }

    if (all_ready && axis_state_timer_) {
      axis_state_timer_->cancel();
    }
  }

  /**
   * @brief Handle asynchronous response from one axis-state service call.
   * Inputs:
   * - index: Axis index associated with this response.
   * - future_response: Future containing the AxisState service response.
   * Output:
   * - Updates ready/request flags and logs success/retry/error status.
   */
  void handle_axis_state_response(
    size_t index, rclcpp::Client<AxisState>::SharedFuture future_response)
  {
    try {
      auto response = future_response.get();
      if (!response) {
        throw std::runtime_error("Received empty response");
      }

      if (response->active_errors != 0U) {
        axis_state_ready_[index] = false;
        axis_state_request_sent_[index] = false;
        RCLCPP_WARN(
          this->get_logger(),
          "%s reported active_errors=%u; will retry state request",
          axis_state_service_names_[index].c_str(),
          response->active_errors);
        return;
      }

      axis_state_ready_[index] = true;
      RCLCPP_INFO(
        this->get_logger(),
        "%s acknowledged state request (axis_state=%u, result=%u)",
        axis_state_service_names_[index].c_str(),
        response->axis_state,
        response->procedure_result);

      if (all_axes_ready()) {
        waiting_to_publish_logged_ = false;
        if (axis_state_timer_) {
          axis_state_timer_->cancel();
        }
        RCLCPP_INFO(this->get_logger(), "All arm axes ready; control commands will resume");
      }
    } catch (const std::exception & ex) {
      axis_state_ready_[index] = false;
      axis_state_request_sent_[index] = false;
      RCLCPP_WARN(
        this->get_logger(),
        "Failed to process response from %s: %s",
        axis_state_service_names_[index].c_str(), ex.what());
    }
  }

  /**
   * @brief Check whether all configured axes are ready for command publishing.
   * Inputs:
   * - None directly; inspects axis_state_ready_ flags.
   * Output:
   * - Returns true only when all axis readiness flags are true and non-empty.
   */
  bool all_axes_ready() const
  {
    return !axis_state_ready_.empty() &&
           std::all_of(axis_state_ready_.begin(), axis_state_ready_.end(),
                       [](bool ready) { return ready; });
  }

  /**
   * @brief Convert desired joint velocities into ODrive velocity control commands.
   * Inputs:
   * - msg: JointState command message from joint_desired_control.
   * Output:
   * - Publishes velocity-mode ControlMessage commands for configured controlled joints.
   */
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    constexpr size_t kControlledDofs = 4U;

    if (!all_axes_ready()) {
      if (!waiting_to_publish_logged_) {
        RCLCPP_WARN(
          this->get_logger(),
          "Arm control commands paused; waiting for all axes to reach state %u",
          requested_axis_state_);
        waiting_to_publish_logged_ = true;
      }
      return;
    }
    waiting_to_publish_logged_ = false;

    if (msg->velocity.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Received joint_desired_control without velocity data; skipping");
      return;
    }

    kanga_interfaces::msg::ControlMessage ctrl;
    ctrl.control_mode = 2;  // velocity control
    ctrl.input_mode = 2;
    ctrl.input_pos = 0.0F;
    ctrl.input_torque = 0.0F;

    const size_t count = std::min({ctrl_publishers_.size(), kControlledDofs, controlled_joint_names_.size()});
    const float max_joint_vel = static_cast<float>(std::max(0.0, max_joint_velocity_));

    for (size_t i = 0; i < count; ++i) {
      float joint_vel = 0.0F;
      if (!extract_joint_velocity(*msg, controlled_joint_names_[i], i, joint_vel)) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "Missing velocity for %s; skipping this cycle",
          controlled_joint_names_[i].c_str());
        return;
      }

      const float reduction = std::max(reduction_factors_[i], 1.0e-6F);
      float limited_joint_vel = joint_vel;
      if (max_joint_vel > 0.0F) {
        limited_joint_vel = std::clamp(limited_joint_vel, -max_joint_vel, max_joint_vel);
      } else {
        limited_joint_vel = 0.0F;
      }
      const float sign = invert_flags_[i] ? -1.0F : 1.0F;
      const float motor_vel = reduction * limited_joint_vel;
      ctrl.input_vel = sign * motor_vel;
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 500,
        "Publishing ODrive command axis=%s joint=%s joint_vel=%.5f rad/s motor_input_vel=%.5f rad/s (raw_joint_vel=%.5f rad/s, reduction=%.3f, joint_vel_limit=%.5f rad/s)",
        axis_names_[i].c_str(),
        controlled_joint_names_[i].c_str(),
        limited_joint_vel,
        ctrl.input_vel,
        joint_vel,
        reduction,
        max_joint_vel);
      ctrl_publishers_[i]->publish(ctrl);
    }
  }

  /**
   * @brief Send a target axis state request to every configured ODrive axis.
   * Inputs:
   * - target_state: Desired axis state to request.
   * - context: Log context string for traceability.
   * Output:
   * - Performs service calls with bounded wait and updates shutdown_command_sent_.
   */
  void command_axes_state(uint32_t target_state, const char * context)
  {
    if (axis_state_clients_.empty() || shutdown_command_sent_) {
      return;
    }

    using namespace std::chrono_literals;

    auto helper_context = std::make_shared<rclcpp::Context>();
    rclcpp::InitOptions init_options;
    init_options.auto_initialize_logging(false);
    helper_context->init(0, nullptr, init_options);

    rclcpp::NodeOptions options;
    options.context(helper_context);
    options.use_global_arguments(false);
    options.start_parameter_services(false);
    options.start_parameter_event_publisher(false);

    auto helper_node = rclcpp::Node::make_shared("arm_command_mapper_shutdown_helper", options);
    auto helper_logger = helper_node->get_logger();

    rclcpp::ExecutorOptions exec_options;
    exec_options.context = helper_context;
    rclcpp::executors::SingleThreadedExecutor executor(exec_options);
    executor.add_node(helper_node);

    struct PendingCall
    {
      std::string service_name;
      rclcpp::Client<AxisState>::SharedPtr client;
      rclcpp::Client<AxisState>::SharedFuture future;
    };

    std::vector<PendingCall> pending_calls;
    pending_calls.reserve(axis_state_clients_.size());

    for (const auto & service_name : axis_state_service_names_) {
      auto client = helper_node->create_client<AxisState>(service_name);

      if (!client->wait_for_service(5s)) {
        RCLCPP_WARN(
          helper_logger,
          "%s service unavailable during %s handling",
          service_name.c_str(),
          context);
        continue;
      }

      auto request = std::make_shared<AxisState::Request>();
      request->axis_requested_state = target_state;

      PendingCall call;
      call.service_name = service_name;
      call.client = client;
      auto future_and_request = client->async_send_request(request);
      call.future = future_and_request.future.share();
      pending_calls.push_back(std::move(call));
    }

    const auto deadline = std::chrono::steady_clock::now() + 5s;

    for (auto & call : pending_calls) {
      auto remaining = deadline - std::chrono::steady_clock::now();
      if (remaining <= std::chrono::steady_clock::duration::zero()) {
        remaining = std::chrono::steady_clock::duration::zero();
      }

      auto status = executor.spin_until_future_complete(call.future, remaining);

      if (status == rclcpp::FutureReturnCode::SUCCESS) {
        try {
          auto response = call.future.get();
          if (response && response->active_errors == 0U) {
            RCLCPP_INFO(
              helper_logger,
              "%s set to state %u successfully during %s",
              call.service_name.c_str(),
              target_state,
              context);
          } else {
            RCLCPP_WARN(
              helper_logger,
              "%s returned errors while setting state %u during %s",
              call.service_name.c_str(),
              target_state,
              context);
          }
        } catch (const std::exception & ex) {
          RCLCPP_WARN(
            helper_logger,
            "Exception while handling response from %s: %s",
            call.service_name.c_str(),
            ex.what());
        }
      } else if (status == rclcpp::FutureReturnCode::TIMEOUT) {
        RCLCPP_WARN(
          helper_logger,
          "Timed out waiting for %s response during %s",
          call.service_name.c_str(),
          context);
      } else {
        RCLCPP_WARN(
          helper_logger,
          "Service call to %s interrupted while handling %s",
          call.service_name.c_str(),
          context);
      }
    }

    executor.remove_node(helper_node);
    helper_context->shutdown("arm_command_mapper shutdown helper completed");
    shutdown_command_sent_ = true;
  }

  std::vector<rclcpp::Publisher<kanga_interfaces::msg::ControlMessage>::SharedPtr> ctrl_publishers_;
  std::vector<bool> invert_flags_;
  std::vector<float> reduction_factors_;
  std::vector<std::string> axis_names_;
  std::vector<std::string> controlled_joint_names_;
  double max_joint_velocity_{10.0};

  std::vector<rclcpp::Client<AxisState>::SharedPtr> axis_state_clients_;
  std::vector<std::string> axis_state_service_names_;
  std::vector<bool> axis_state_ready_;
  std::vector<bool> axis_state_request_sent_;
  rclcpp::TimerBase::SharedPtr axis_state_timer_;
  uint32_t requested_axis_state_{};
  bool waiting_to_publish_logged_{false};
  bool shutdown_hook_registered_{false};
  bool shutdown_command_sent_{false};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
};

/**
 * @brief Entry point for the arm command mapper executable.
 * Inputs:
 * - argc: Argument count.
 * - argv: Argument vector.
 * Output:
 * - Returns process exit code (0 on normal completion).
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmCommandMapper>();
  node->register_shutdown_hook();
  rclcpp::spin(node);
  node->request_axes_state_now(1U, "post-spin");
  rclcpp::shutdown();
  return 0;
}
