#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <chrono>
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

std::vector<OdriveConfig> load_configs_from_yaml(const rclcpp::Logger & logger)
{
    std::vector<OdriveConfig> configs;

    std::string config_path;
    try {
        config_path = ament_index_cpp::get_package_share_directory("kanga_drive") +
                      "/config/odrive_node_ids.yaml";
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
            auto value = trim(trimmed.substr(std::string("- namespace:").size()));
            if (value.size() >= 2 && value.front() == '"' && value.back() == '"') {
                value = value.substr(1, value.size() - 2);
            }
            current.ns = value;
            continue;
        }

        if (!current_valid) {
            continue;
        }

        if (starts_with(trimmed, "namespace:")) {
            auto value = trim(trimmed.substr(std::string("namespace:").size()));
            if (value.size() >= 2 && value.front() == '"' && value.back() == '"') {
                value = value.substr(1, value.size() - 2);
            }
            current.ns = value;
            continue;
        }

        if (starts_with(trimmed, "invert:")) {
            auto value = trim(trimmed.substr(std::string("invert:").size()));
            std::transform(
                value.begin(), value.end(), value.begin(),
                [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            current.invert = (value == "true" || value == "1" || value == "yes");
            continue;
        }
    }

    push_current();

    if (configs.empty()) {
        RCLCPP_WARN(logger, "No ODrive namespaces found in %s; falling back to defaults",
            config_path.c_str());
        configs.push_back({"odrive_axis0", false});
        configs.push_back({"odrive_axis1", false});
        configs.push_back({"odrive_axis2", false});
        configs.push_back({"odrive_axis3", false});
    }

    return configs;
}

}  // namespace

class WheelCommandMapper : public rclcpp::Node
{
public:
    WheelCommandMapper() : Node("wheel_command_mapper")
    {
        const int axis_state_param = this->declare_parameter<int>(
            "axis_requested_state", 8);
        if (axis_state_param < 0) {
            RCLCPP_WARN(
                this->get_logger(),
                "axis_requested_state parameter must be non-negative; clamping to 0");
            requested_axis_state_ = 0U;
        } else {
            requested_axis_state_ = static_cast<uint32_t>(axis_state_param);
        }

        max_wheel_velocity_ = this->declare_parameter<double>(
            "max_wheel_velocity", 20.0);

        auto configs = load_configs_from_yaml(this->get_logger());

        for (const auto & cfg : configs) {
            if (cfg.ns.empty()) {
                continue;
            }

            ctrl_publishers_.push_back(
                this->create_publisher<kanga_interfaces::msg::ControlMessage>(
                    "/" + cfg.ns + "/control_message", 10));
            invert_flags_.push_back(cfg.invert);
            wheel_names_.push_back(cfg.ns);

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
                "No valid ODrive configurations found; control publishers will remain inactive");
        } else {
            RCLCPP_INFO(
                this->get_logger(),
                "Configured %zu ODrive control publishers", ctrl_publishers_.size());
        }

        axis_state_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&WheelCommandMapper::tick_axis_state_requests, this));

        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&WheelCommandMapper::twistCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "wheel_command_mapper active");
    }

    void register_shutdown_hook()
    {
        if (shutdown_hook_registered_) {
            return;
        }

        auto shared_self = std::static_pointer_cast<WheelCommandMapper>(this->shared_from_this());
        if (!shared_self) {
            RCLCPP_WARN(this->get_logger(), "Failed to register shutdown hook: shared_from_this cast failed");
            return;
        }

        std::weak_ptr<WheelCommandMapper> weak_self = shared_self;
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

    void request_axes_state_now(uint32_t target_state, const char * context)
    {
        command_axes_state(target_state, context);
    }

private:
    using AxisState = kanga_interfaces::srv::AxisState;

    std::array<float, 4> computeWheelCommands(const geometry_msgs::msg::Twist & tw)
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
        
        auto clamp = [max_vel](double value) {
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
                RCLCPP_INFO(
                    this->get_logger(),
                    "All axes ready; control commands will resume");
            }
        } catch (const std::exception & ex) {
            axis_state_ready_[index] = false;
            axis_state_request_sent_[index] = false;
            RCLCPP_WARN(
                this->get_logger(),
                "Failed to process response from %s: %s",
                axis_state_service_names_[index].c_str(),
                ex.what());
        }
    }

    bool all_axes_ready() const
    {
        return !axis_state_ready_.empty() &&
               std::all_of(axis_state_ready_.begin(), axis_state_ready_.end(),
                   [](bool ready) { return ready; });
    }

    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        auto cmds = computeWheelCommands(*msg);

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
        ctrl.input_mode   = 2;
        ctrl.input_pos    = 0.0F;

        const size_t count = std::min(ctrl_publishers_.size(), cmds.size());
        if (count > 0U) {
            std::ostringstream oss;
            const size_t named = std::min(count, wheel_names_.size());
            for (size_t i = 0; i < count; ++i) {
                if (i > 0U) {
                    oss << ", ";
                }
                if (i < named) {
                    oss << wheel_names_[i];
                } else {
                    oss << "wheel" << i;
                }
                oss << '=' << cmds[i];
            }

            // RCLCPP_INFO_THROTTLE(
            //     this->get_logger(), *this->get_clock(), 500,
            //     "Wheel velocities: %s", oss.str().c_str());
        }

        for (size_t i = 0; i < count; ++i) {
            const float sign = invert_flags_[i] ? -1.0F : 1.0F;
            ctrl.input_vel = sign * cmds[i];
            ctrl_publishers_[i]->publish(ctrl);
        }
    }

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

        auto helper_node = rclcpp::Node::make_shared(
            "wheel_command_mapper_shutdown_helper",
            options);
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
        helper_context->shutdown("wheel_command_mapper shutdown helper completed");
        shutdown_command_sent_ = true;
    }

    std::vector<rclcpp::Publisher<kanga_interfaces::msg::ControlMessage>::SharedPtr> ctrl_publishers_;
    std::vector<bool> invert_flags_;
    std::vector<std::string> wheel_names_;
    double max_wheel_velocity_{20.0};

    std::vector<rclcpp::Client<AxisState>::SharedPtr> axis_state_clients_;
    std::vector<std::string> axis_state_service_names_;
    std::vector<bool> axis_state_ready_;
    std::vector<bool> axis_state_request_sent_;
    rclcpp::TimerBase::SharedPtr axis_state_timer_;
    uint32_t requested_axis_state_{};
    bool waiting_to_publish_logged_{false};
    bool shutdown_hook_registered_{false};
    bool shutdown_command_sent_{false};

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WheelCommandMapper>();
    node->register_shutdown_hook();
    rclcpp::spin(node);
    node->request_axes_state_now(1U, "post-spin");
    rclcpp::shutdown();
    return 0;
}
