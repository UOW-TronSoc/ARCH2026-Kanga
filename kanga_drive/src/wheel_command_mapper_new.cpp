#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <rclcpp/executors/single_threaded_executor.hpp>
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

std::vector<OdriveConfig> load_configs_from_yaml(const rclcpp::Logger & logger)
{
    std::vector<OdriveConfig> configs;

    std::string config_path;
    try {
        config_path = ament_index_cpp::get_package_share_directory("kanga_drive") +
                      "/config/odrive_node_ids_drive.yaml";
    } catch (const std::exception & ex) {
        RCLCPP_WARN(logger, "Unable to locate odrive configuration: %s", ex.what());
        return configs;
    }

    std::ifstream config_stream(config_path);
    if (!config_stream.is_open()) {
        RCLCPP_WARN(logger, "Unable to open odrive configuration file: %s", config_path.c_str());
        return configs;
    }

    std::string line;
    bool in_nodes = false;
    int nodes_indent = -1;
    OdriveConfig current;
    bool current_valid = false;

    auto push_current = [&]() {
        if (!current.ns.empty()) {
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
    }

    push_current();

    if (configs.empty()) {
        RCLCPP_WARN(logger, "No ODrive namespaces found in %s; falling back to defaults",
            config_path.c_str());
        configs.push_back({"wheel_fl", false});
        configs.push_back({"wheel_bl", false});
        configs.push_back({"wheel_br", false});
        configs.push_back({"wheel_fr", false});
    }

    return configs;
}

}  // namespace

class WheelCommandMapperNew : public rclcpp::Node
{
public:
    WheelCommandMapperNew() : Node("wheel_command_mapper_new")
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

        max_wheel_velocity_ = this->declare_parameter<double>("max_wheel_velocity", 20.0);

        initialize_wheels(load_configs_from_yaml(this->get_logger()));

        axis_state_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&WheelCommandMapperNew::tick_axis_state_requests, this));

        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&WheelCommandMapperNew::twist_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "wheel_command_mapper_new active");
    }

    void register_shutdown_hook()
    {
        if (shutdown_hook_registered_) {
            return;
        }

        auto shared_self = std::static_pointer_cast<WheelCommandMapperNew>(this->shared_from_this());
        if (!shared_self) {
            RCLCPP_WARN(
                this->get_logger(),
                "Failed to register shutdown hook: shared_from_this cast failed");
            return;
        }

        std::weak_ptr<WheelCommandMapperNew> weak_self = shared_self;
        rclcpp::on_shutdown([weak_self]() {
            if (auto self = weak_self.lock()) {
                std::thread([self]() {
                    try {
                        self->request_axes_state_now(1U, "shutdown");
                    } catch (const std::exception & ex) {
                        RCLCPP_WARN(self->get_logger(), "Shutdown handler exception: %s", ex.what());
                    }
                }).detach();
            }
        });

        shutdown_hook_registered_ = true;
    }

    void request_axes_state_now(uint32_t target_state, const char * context)
    {
        command_axes_state(target_state, context);
    }

private:
    using AxisState = kanga_interfaces::srv::AxisState;

    struct WheelEndpoint
    {
        std::string ns;
        bool invert{false};
        rclcpp::Publisher<kanga_interfaces::msg::ControlMessage>::SharedPtr ctrl_pub;
        std::string axis_state_service_name;
        rclcpp::Client<AxisState>::SharedPtr axis_state_client;
        bool axis_state_ready{false};
        bool axis_state_request_sent{false};
    };

    void initialize_wheels(const std::vector<OdriveConfig> & configs)
    {
        for (const auto & cfg : configs) {
            if (cfg.ns.empty()) {
                continue;
            }

            WheelEndpoint wheel;
            wheel.ns = cfg.ns;
            wheel.invert = cfg.invert;
            wheel.ctrl_pub = this->create_publisher<kanga_interfaces::msg::ControlMessage>(
                "/" + cfg.ns + "/control_message", 10);
            wheel.axis_state_service_name = "/" + cfg.ns + "/request_axis_state";
            wheel.axis_state_client =
                this->create_client<AxisState>(wheel.axis_state_service_name);
            wheels_.push_back(std::move(wheel));
        }

        if (wheels_.empty()) {
            RCLCPP_WARN(
                this->get_logger(),
                "No valid ODrive configurations found; control publishers will remain inactive");
        } else {
            RCLCPP_INFO(
                this->get_logger(),
                "Configured %zu ODrive control publishers", wheels_.size());
        }
    }

    std::array<float, 4> compute_wheel_commands(const geometry_msgs::msg::Twist & tw) const
    {
        constexpr double deg2rad = M_PI / 180.0;
        constexpr double theta = 51.0 * deg2rad;
        const double s = std::sin(theta);
        const double c = std::cos(theta);
        const double alpha = 1.0 / c;
        constexpr double half_length = 0.435;
        constexpr double half_width = 0.3535;
        const double r = half_length + half_width;

        const double vx = static_cast<double>(tw.linear.x);
        const double vy = static_cast<double>(tw.linear.y);
        const double omega = static_cast<double>(tw.angular.z);

        double v_fl = alpha * (s * vx + c * vy - r * omega);
        double v_fr = alpha * (s * vx - c * vy + r * omega);
        double v_rl = alpha * (s * vx - c * vy - r * omega);
        double v_rr = alpha * (s * vx + c * vy + r * omega);

        const double max_vel = std::max(0.0, max_wheel_velocity_);
        const auto clamp = [max_vel](double value) {
            if (max_vel == 0.0) {
                return 0.0;
            }
            return std::clamp(value, -max_vel, max_vel);
        };

        v_fl = clamp(v_fl);
        v_fr = clamp(v_fr);
        v_rl = clamp(v_rl);
        v_rr = clamp(v_rr);

        return {
            static_cast<float>(v_fl),
            static_cast<float>(v_rl),
            static_cast<float>(v_rr),
            static_cast<float>(v_fr)};
    }

    void tick_axis_state_requests()
    {
        if (wheels_.empty()) {
            return;
        }

        bool all_ready = true;

        for (size_t i = 0; i < wheels_.size(); ++i) {
            auto & wheel = wheels_[i];
            if (wheel.axis_state_ready) {
                continue;
            }

            all_ready = false;

            if (!wheel.axis_state_client->service_is_ready()) {
                continue;
            }

            if (wheel.axis_state_request_sent) {
                continue;
            }

            auto request = std::make_shared<AxisState::Request>();
            request->axis_requested_state = requested_axis_state_;

            auto future = wheel.axis_state_client->async_send_request(
                request,
                [this, index = i](rclcpp::Client<AxisState>::SharedFuture future_response) {
                    this->handle_axis_state_response(index, future_response);
                });
            (void)future;

            wheel.axis_state_request_sent = true;
            RCLCPP_INFO(
                this->get_logger(),
                "Requested state %u for %s",
                requested_axis_state_,
                wheel.axis_state_service_name.c_str());
        }

        if (all_ready && axis_state_timer_) {
            axis_state_timer_->cancel();
        }
    }

    void handle_axis_state_response(
        size_t index, rclcpp::Client<AxisState>::SharedFuture future_response)
    {
        if (index >= wheels_.size()) {
            return;
        }

        auto & wheel = wheels_[index];

        try {
            auto response = future_response.get();
            if (!response) {
                throw std::runtime_error("Received empty response");
            }

            if (response->active_errors != 0U) {
                wheel.axis_state_ready = false;
                wheel.axis_state_request_sent = false;
                RCLCPP_WARN(
                    this->get_logger(),
                    "%s reported active_errors=%u; will retry state request",
                    wheel.axis_state_service_name.c_str(),
                    response->active_errors);
                return;
            }

            wheel.axis_state_ready = true;
            RCLCPP_INFO(
                this->get_logger(),
                "%s acknowledged state request (axis_state=%u, result=%u)",
                wheel.axis_state_service_name.c_str(),
                response->axis_state,
                response->procedure_result);

            if (all_axes_ready()) {
                waiting_to_publish_logged_ = false;
                if (axis_state_timer_) {
                    axis_state_timer_->cancel();
                }
                RCLCPP_INFO(
                    this->get_logger(),
                    "All axes ready; control commands will resume");
            }
        } catch (const std::exception & ex) {
            wheel.axis_state_ready = false;
            wheel.axis_state_request_sent = false;
            RCLCPP_WARN(
                this->get_logger(),
                "Failed to process response from %s: %s",
                wheel.axis_state_service_name.c_str(),
                ex.what());
        }
    }

    bool all_axes_ready() const
    {
        return !wheels_.empty() &&
               std::all_of(wheels_.begin(), wheels_.end(),
                   [](const WheelEndpoint & wheel) { return wheel.axis_state_ready; });
    }

    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        const auto commands = compute_wheel_commands(*msg);

        if (!all_axes_ready()) {
            if (!waiting_to_publish_logged_) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Control commands paused; waiting for all axes to reach state %u",
                    requested_axis_state_);
                waiting_to_publish_logged_ = true;
            }
            return;
        }

        waiting_to_publish_logged_ = false;

        kanga_interfaces::msg::ControlMessage ctrl;
        ctrl.control_mode = 2;
        ctrl.input_mode = 2;
        ctrl.input_pos = 0.0F;

        const size_t count = std::min(wheels_.size(), commands.size());
        for (size_t i = 0; i < count; ++i) {
            const float sign = wheels_[i].invert ? -1.0F : 1.0F;
            ctrl.input_vel = sign * commands[i];
            wheels_[i].ctrl_pub->publish(ctrl);
        }
    }

    void command_axes_state(uint32_t target_state, const char * context)
    {
        if (wheels_.empty() || shutdown_command_sent_) {
            return;
        }

        using namespace std::chrono_literals;

        auto helper_context = std::make_shared<rclcpp::Context>();
        rclcpp::InitOptions init_options;
        init_options.auto_initialize_logging(false);
        helper_context->init(0, nullptr, init_options);

        rclcpp::NodeOptions node_options;
        node_options.context(helper_context);
        node_options.use_global_arguments(false);
        node_options.start_parameter_services(false);
        node_options.start_parameter_event_publisher(false);

        auto helper_node = rclcpp::Node::make_shared(
            "wheel_command_mapper_new_shutdown_helper",
            node_options);
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
        pending_calls.reserve(wheels_.size());

        for (const auto & wheel : wheels_) {
            auto client = helper_node->create_client<AxisState>(wheel.axis_state_service_name);

            if (!client->wait_for_service(5s)) {
                RCLCPP_WARN(
                    helper_logger,
                    "%s service unavailable during %s handling",
                    wheel.axis_state_service_name.c_str(),
                    context);
                continue;
            }

            auto request = std::make_shared<AxisState::Request>();
            request->axis_requested_state = target_state;

            PendingCall call;
            call.service_name = wheel.axis_state_service_name;
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
        helper_context->shutdown("wheel_command_mapper_new shutdown helper completed");
        shutdown_command_sent_ = true;
    }

    std::vector<WheelEndpoint> wheels_;
    double max_wheel_velocity_{20.0};
    uint32_t requested_axis_state_{8U};

    bool waiting_to_publish_logged_{false};
    bool shutdown_hook_registered_{false};
    bool shutdown_command_sent_{false};

    rclcpp::TimerBase::SharedPtr axis_state_timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WheelCommandMapperNew>();
    node->register_shutdown_hook();
    rclcpp::spin(node);
    node->request_axes_state_now(1U, "post-spin");
    rclcpp::shutdown();
    return 0;
}
