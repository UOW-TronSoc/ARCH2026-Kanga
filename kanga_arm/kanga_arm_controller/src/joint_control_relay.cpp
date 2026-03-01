#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>
#include <mutex>
#include <algorithm>

class JointControlRelay : public rclcpp::Node
{
public:
	JointControlRelay() : Node("kanga_arm_joint_control_relay")
	{
		init_pos_ = this->declare_parameter<std::vector<double>>(
			"joint_initial_positions",
			std::vector<double>(dof, 0.0));

		if (init_pos_.size() < dof)
		{
			RCLCPP_WARN(this->get_logger(), "joint_initial_positions size is less than %zu; padding with zeros", dof);
			init_pos_.resize(dof, 0.0);
		}
		else if (init_pos_.size() > dof)
		{
			init_pos_.resize(dof);
		}

		current_position_ = init_pos_;
		latest_velocity_ = std::vector<double>(dof, 0.0);
		last_velocity_time_ = steady_clock_.now();

		joint_control_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
			"/kanga_arm/joint_control", 10,
			std::bind(&JointControlRelay::jointControlCallback, this, std::placeholders::_1));

			desired_control_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
				"joint_desired_control", 10);

			publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 100.0);
			const double control_time_step_ms = this->declare_parameter<double>("control_time_step_ms", 10.0);
			max_velocity_slew_rate_ = this->declare_parameter<double>("max_velocity_slew_rate", 8.0);
			const double safe_rate_hz = (publish_rate_hz_ > 0.0) ? publish_rate_hz_ : 100.0;
		if (publish_rate_hz_ > 0.0)
		{
			publish_period_seconds_ = 1.0 / safe_rate_hz;
		}
		else
		{
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
	void jointControlCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
	{
		if (msg->velocity.size() < dof)
		{
			RCLCPP_WARN(this->get_logger(), "Joint control message has fewer than %zu velocities; missing entries treated as zero", dof);
		}

		std::lock_guard<std::mutex> lock(command_mutex_);
		for (size_t i = 0; i < dof; ++i)
		{
			latest_velocity_[i] = (msg->velocity.size() > i) ? msg->velocity[i] : 0.0;
		}
		last_velocity_time_ = steady_clock_.now();
	}

		void publishDesiredControl()
		{
			const rclcpp::Time now = steady_clock_.now();
			const double dt = publish_period_seconds_;

		std::vector<double> raw_velocity(dof, 0.0);
		{
			std::lock_guard<std::mutex> lock(command_mutex_);
			const double stale_seconds = (now - last_velocity_time_).seconds();
			if (stale_seconds <= 0.5)
			{
				raw_velocity = latest_velocity_;
			}
		}

		std::vector<double> velocity(dof, 0.0);
		const double max_step = std::max(0.0, max_velocity_slew_rate_) * dt;
		for (size_t i = 0; i < dof; ++i)
		{
			const double dv = raw_velocity[i] - filtered_velocity_[i];
			if (dv > max_step)
			{
				filtered_velocity_[i] += max_step;
			}
			else if (dv < -max_step)
			{
				filtered_velocity_[i] -= max_step;
			}
			else
			{
				filtered_velocity_[i] = raw_velocity[i];
			}

			velocity[i] = filtered_velocity_[i];
			current_position_[i] += velocity[i] * dt;
		}

		sensor_msgs::msg::JointState out_msg;
		out_msg.header.stamp = this->get_clock()->now();
		out_msg.position.resize(dof);
		out_msg.velocity.resize(dof);
		out_msg.effort.resize(dof, 0.0);

		for (size_t i = 0; i < dof; ++i)
		{
			out_msg.position[i] = current_position_[i];
			out_msg.velocity[i] = velocity[i];
		}

		desired_control_pub_->publish(out_msg);
	}

	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_control_sub_;
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr desired_control_pub_;
	rclcpp::TimerBase::SharedPtr timer_;

	const size_t dof = 5;
	std::vector<double> init_pos_;
	std::vector<double> current_position_;
	std::vector<double> latest_velocity_;
	std::vector<double> filtered_velocity_{std::vector<double>(dof, 0.0)};
	rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
	rclcpp::Time last_velocity_time_;
	std::mutex command_mutex_;
	double publish_rate_hz_{100.0};
	double publish_period_seconds_{0.01};
	double max_velocity_slew_rate_{8.0};
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<JointControlRelay>());
	rclcpp::shutdown();
	return 0;
}
