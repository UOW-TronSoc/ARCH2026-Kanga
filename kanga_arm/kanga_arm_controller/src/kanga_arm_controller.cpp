#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include <mutex>
#include <chrono>
#include <string>

#include "kanga_arm_controller/arm_kinematics.hpp"

/**
 * @brief Main arm control node.
 *
 * Data flow:
 * 1) Reads measured joint state from `joint_states`.
 * 2) Reads desired world-space command from `kanga_arm/world_state_control`.
 * 3) Runs `controlCommands()` at 1 kHz (configurable).
 * 4) Publishes joint command output and end-effector pose estimate.
 */
class KangaArmController : public rclcpp::Node
{
public:
	KangaArmController() : Node("kanga_arm_controller")
	{
		// Configure timing and model parameters.
		control_time_step_ms = this->declare_parameter<float>("control_time_step_ms", 1.0f);
		const auto init_pos_param = this->declare_parameter<std::vector<double>>(
			"joint_initial_positions",
			std::vector<double>(dof, 0.0));
		joint_min_limits_ = this->declare_parameter<std::vector<double>>(
			"joint_min_limits", std::vector<double>(dof, -std::numeric_limits<double>::infinity()));
		joint_max_limits_ = this->declare_parameter<std::vector<double>>(
			"joint_max_limits", std::vector<double>(dof, std::numeric_limits<double>::infinity()));
		joint_max_velocity_ = this->declare_parameter<std::vector<double>>(
			"joint_max_velocity", std::vector<double>(dof, std::numeric_limits<double>::infinity()));
		hold_position_on_stale_ = this->declare_parameter<bool>("hold_position_on_stale", true);
		command_stale_timeout_s_ = this->declare_parameter<double>("command_stale_timeout_s", 0.5);
		const auto link_lengths_param = this->declare_parameter<std::vector<double>>(
			"link_lengths", std::vector<double>{});
		end_effector_config_ = this->declare_parameter<std::string>("end_effector_config", "roll_tool");

		const double default_terminal_length = (link_lengths_param.size() > 4) ? link_lengths_param[4] : 0.0;
		const auto roll_tool_tf_param = this->declare_parameter<std::vector<double>>(
			"roll_tool_transform_matrix", std::vector<double>{});
		const auto scoop_tool_tf_param = this->declare_parameter<std::vector<double>>(
			"scoop_tool_transform_matrix", std::vector<double>{});
		const auto roll_tool_offset_xyz_param = this->declare_parameter<std::vector<double>>(
			"roll_tool_offset_xyz", std::vector<double>{default_terminal_length, 0.0, 0.0});
		const auto roll_tool_offset_rpy_param = this->declare_parameter<std::vector<double>>(
			"roll_tool_offset_rpy", std::vector<double>{0.0, 0.0, 0.0});
		const auto scoop_tool_offset_xyz_param = this->declare_parameter<std::vector<double>>(
			"scoop_tool_offset_xyz", std::vector<double>{default_terminal_length, 0.0, 0.0});
		const auto scoop_tool_offset_rpy_param = this->declare_parameter<std::vector<double>>(
			"scoop_tool_offset_rpy", std::vector<double>{0.0, 0.0, 0.0});

		// Initialize tracked joint state vectors.
		init_pos = Eigen::VectorXd::Zero(dof);
		reference_world_velocity_ = Eigen::VectorXd::Zero(6);
		joint_position = Eigen::VectorXd::Zero(dof);
		joint_velocity = Eigen::VectorXd::Zero(dof);
		desired_joint_position_ = Eigen::VectorXd::Zero(dof);

		if (init_pos_param.size() >= dof)
		{
			for (size_t i = 0; i < dof; ++i)
			{
				init_pos(static_cast<Eigen::Index>(i)) = init_pos_param[i];
				joint_position(static_cast<Eigen::Index>(i)) = init_pos_param[i];
				desired_joint_position_(static_cast<Eigen::Index>(i)) = init_pos_param[i];
			}
		}
		else if (!init_pos_param.empty())
		{
			// Partial parameter vectors are allowed, but this node currently expects all joints.
			RCLCPP_WARN(this->get_logger(), "joint_initial_positions size is less than %zu; using zeros", dof);
		}

		link_lengths = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(link_lengths_param.size()));
		for (size_t i = 0; i < link_lengths_param.size(); ++i)
		{
			link_lengths(static_cast<Eigen::Index>(i)) = link_lengths_param[i];
		}

		if (joint_min_limits_.size() < dof)
		{
			RCLCPP_WARN(this->get_logger(), "joint_min_limits has fewer than %zu entries; padding with -inf", dof);
			joint_min_limits_.resize(dof, -std::numeric_limits<double>::infinity());
		}
		else if (joint_min_limits_.size() > dof)
		{
			joint_min_limits_.resize(dof);
		}

		if (joint_max_limits_.size() < dof)
		{
			RCLCPP_WARN(this->get_logger(), "joint_max_limits has fewer than %zu entries; padding with +inf", dof);
			joint_max_limits_.resize(dof, std::numeric_limits<double>::infinity());
		}
		else if (joint_max_limits_.size() > dof)
		{
			joint_max_limits_.resize(dof);
		}

		if (joint_max_velocity_.size() < dof)
		{
			RCLCPP_WARN(this->get_logger(), "joint_max_velocity has fewer than %zu entries; padding with +inf", dof);
			joint_max_velocity_.resize(dof, std::numeric_limits<double>::infinity());
		}
		else if (joint_max_velocity_.size() > dof)
		{
			joint_max_velocity_.resize(dof);
		}

		for (size_t i = 0; i < dof; ++i)
		{
			if (joint_min_limits_[i] > joint_max_limits_[i])
			{
				std::swap(joint_min_limits_[i], joint_max_limits_[i]);
			}
			if (joint_max_velocity_[i] < 0.0)
			{
				joint_max_velocity_[i] = std::abs(joint_max_velocity_[i]);
			}

			const auto idx = static_cast<Eigen::Index>(i);
			desired_joint_position_(idx) = std::clamp(
				desired_joint_position_(idx), joint_min_limits_[i], joint_max_limits_[i]);
		}

		last_velocity_time_ = steady_clock_.now();

		kinematics_ = kanga_arm_controller::ArmKinematics(dof);
		kinematics_.setLinkLengths(link_lengths_param);

		Eigen::Matrix4d roll_tool_tf = Eigen::Matrix4d::Identity();
		Eigen::Matrix4d scoop_tool_tf = Eigen::Matrix4d::Identity();

		if (!vectorToMatrix4(
				roll_tool_tf_param, roll_tool_tf, "roll_tool_transform_matrix"))
		{
			roll_tool_tf = toolTransformFromXyzRpy(
				vectorToEigen3(
					roll_tool_offset_xyz_param,
					Eigen::Vector3d(default_terminal_length, 0.0, 0.0),
					"roll_tool_offset_xyz"),
				vectorToEigen3(
					roll_tool_offset_rpy_param,
					Eigen::Vector3d::Zero(),
					"roll_tool_offset_rpy"));
		}

		if (!vectorToMatrix4(
				scoop_tool_tf_param, scoop_tool_tf, "scoop_tool_transform_matrix"))
		{
			scoop_tool_tf = toolTransformFromXyzRpy(
				vectorToEigen3(
					scoop_tool_offset_xyz_param,
					Eigen::Vector3d(default_terminal_length, 0.0, 0.0),
					"scoop_tool_offset_xyz"),
				vectorToEigen3(
					scoop_tool_offset_rpy_param,
					Eigen::Vector3d::Zero(),
					"scoop_tool_offset_rpy"));
		}

		kinematics_.setToolTransforms(roll_tool_tf, scoop_tool_tf);
		if (!kinematics_.setEndEffectorMode(end_effector_config_))
		{
			RCLCPP_WARN(
				this->get_logger(),
				"Invalid end_effector_config='%s'. Falling back to roll_tool.",
				end_effector_config_.c_str());
			(void)kinematics_.setEndEffectorMode("roll_tool");
			end_effector_config_ = "roll_tool";
		}

		// Initialize published endpoint with the best available estimate.
		endpoint_position = kinematics_.forwardPosition(joint_position);
		desired_world_position_ = endpoint_position;

		// Encoder/state feedback subscription.
		joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
			"joint_states", 10,
			std::bind(&KangaArmController::jointStateCallback, this, std::placeholders::_1));

		// Desired world velocity command subscription.
		world_velocity_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
			"kanga_arm/world_state_control", 10,
			std::bind(&KangaArmController::worldVelocityCallback, this, std::placeholders::_1));

		// Output command publisher for the downstream actuator/control bridge.
		desired_control_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_desired_control", 10);

		// Diagnostic/telemetry publisher for Cartesian end-effector pose.
		endpoint_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("endpoint", 10);

		// Main control loop timer (default 1 kHz from 1.0 ms period).
		timer_ = rclcpp::create_timer(
			this->get_node_base_interface(),
			this->get_node_timers_interface(),
			this->get_clock(),
			std::chrono::microseconds((int)(control_time_step_ms * 1000)),
			std::bind(&KangaArmController::controlCommands, this));

		// Startup log for operator visibility.
		RCLCPP_INFO(
			this->get_logger(),
			"Kanga Arm Controller Node started (end_effector_config=%s)",
			end_effector_config_.c_str());
	}

private:
	static Eigen::Matrix4d toolTransformFromXyzRpy(
		const Eigen::Vector3d &xyz,
		const Eigen::Vector3d &rpy)
	{
		const Eigen::AngleAxisd rx(rpy.x(), Eigen::Vector3d::UnitX());
		const Eigen::AngleAxisd ry(rpy.y(), Eigen::Vector3d::UnitY());
		const Eigen::AngleAxisd rz(rpy.z(), Eigen::Vector3d::UnitZ());
		Eigen::Matrix4d t = Eigen::Matrix4d::Identity();
		t.block<3, 3>(0, 0) = (rz * ry * rx).toRotationMatrix();
		t.block<3, 1>(0, 3) = xyz;
		return t;
	}

	bool vectorToMatrix4(
		const std::vector<double> &values,
		Eigen::Matrix4d &out,
		const std::string &param_name)
	{
		if (values.empty())
		{
			return false;
		}

		if (values.size() != 16U)
		{
			RCLCPP_WARN(
				this->get_logger(),
				"%s must have exactly 16 values (row-major 4x4). Ignoring it.",
				param_name.c_str());
			return false;
		}

		Eigen::Matrix4d matrix = Eigen::Matrix4d::Zero();
		for (int r = 0; r < 4; ++r)
		{
			for (int c = 0; c < 4; ++c)
			{
				matrix(r, c) = values[static_cast<size_t>(r * 4 + c)];
			}
		}

		const Eigen::Matrix4d expected_last_row(
			(Eigen::Matrix4d() << 0.0, 0.0, 0.0, 0.0,
			 0.0, 0.0, 0.0, 0.0,
			 0.0, 0.0, 0.0, 0.0,
			 0.0, 0.0, 0.0, 1.0)
				.finished());

		if (!matrix.row(3).isApprox(expected_last_row.row(3), 1e-9))
		{
			RCLCPP_WARN(
				this->get_logger(),
				"%s has invalid homogeneous bottom row. Forcing [0 0 0 1].",
				param_name.c_str());
			matrix.row(3) = Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0);
		}

		out = matrix;
		return true;
	}

	Eigen::Vector3d vectorToEigen3(
		const std::vector<double> &values,
		const Eigen::Vector3d &fallback,
		const std::string &param_name)
	{
		if (values.size() != 3U)
		{
			RCLCPP_WARN(
				this->get_logger(),
				"%s must have exactly 3 values. Using fallback [%.3f, %.3f, %.3f].",
				param_name.c_str(),
				fallback.x(), fallback.y(), fallback.z());
			return fallback;
		}
		return Eigen::Vector3d(values[0], values[1], values[2]);
	}

	/**
	 * @brief Periodic control loop callback.
	 *
	 * Current behavior:
	 * - maps world-space twist command to joint velocity via damped Jacobian pseudoinverse,
	 * - publishes velocity-focused joint command output,
	 * - publishes estimated endpoint pose from the configured kinematics model.
	 */
	void controlCommands()
	{
		// Ignore cycle zero when simulated time has not started.
		const auto now_ros = this->get_clock()->now();
		if (now_ros.seconds() == 0.0)
			return;

		const auto stamp = now_ros;

		// Create control effort message
		sensor_msgs::msg::JointState control_effort;
		control_effort.header.stamp = stamp;
		control_effort.position.resize(dof);
		control_effort.velocity.resize(dof);
		control_effort.effort.resize(dof);

		Eigen::VectorXd reference_world_velocity(6);
		double stale_seconds = 0.0;
		{
			std::lock_guard<std::mutex> lock(command_mutex_);
			reference_world_velocity = reference_world_velocity_;
			stale_seconds = (steady_clock_.now() - last_velocity_time_).seconds();
		}

		const bool is_stale = stale_seconds > command_stale_timeout_s_;
		if (is_stale && hold_position_on_stale_)
		{
			RCLCPP_WARN_THROTTLE(
				this->get_logger(),
				*this->get_clock(),
				1000,
				"World command watchdog triggered: stale for %.3f s (timeout=%.3f s). Holding q_ref.",
				stale_seconds,
				command_stale_timeout_s_);
			reference_world_velocity.setZero();
		}

		const double dt = static_cast<double>(control_time_step_ms) * 1e-3;
		desired_world_position_ += reference_world_velocity.head<3>() * dt;

		const Eigen::MatrixXd jacobian = kinematics_.computeJacobian(joint_position);
		const Eigen::MatrixXd jacobian_pinv = pseudoInverse(jacobian, 1e-4);
		const Eigen::VectorXd joint_velocity_cmd = jacobian_pinv * reference_world_velocity;

		for (size_t i = 0; i < dof; ++i)
		{
			const auto idx = static_cast<Eigen::Index>(i);
			double cmd_vel = (joint_velocity_cmd.size() > idx) ? joint_velocity_cmd(idx) : 0.0;
			cmd_vel = std::clamp(cmd_vel, -joint_max_velocity_[i], joint_max_velocity_[i]);

			desired_joint_position_(idx) += cmd_vel * dt;
			desired_joint_position_(idx) = std::clamp(
				desired_joint_position_(idx), joint_min_limits_[i], joint_max_limits_[i]);

			// If saturated, don't continue commanding into the limit.
			if ((desired_joint_position_(idx) <= joint_min_limits_[i] && cmd_vel < 0.0) ||
				(desired_joint_position_(idx) >= joint_max_limits_[i] && cmd_vel > 0.0))
			{
				cmd_vel = 0.0;
			}

			control_effort.position[i] = desired_joint_position_(idx);
			control_effort.velocity[i] = cmd_vel;
			control_effort.effort[i] = 0.0;
		}

		desired_control_pub_->publish(control_effort);

		// Publish latest endpoint estimate from current measured joint state.
		const Eigen::Isometry3d endpoint_tf = kinematics_.forwardTransform(joint_position);
		endpoint_position = endpoint_tf.translation();
		geometry_msgs::msg::Pose endpoint_msg;
		endpoint_msg.position.x = endpoint_position.x();
		endpoint_msg.position.y = endpoint_position.y();
		endpoint_msg.position.z = endpoint_position.z();
		const Eigen::Quaterniond q(endpoint_tf.rotation());
		endpoint_msg.orientation.x = q.x();
		endpoint_msg.orientation.y = q.y();
		endpoint_msg.orientation.z = q.z();
		endpoint_msg.orientation.w = q.w();
		endpoint_publisher_->publish(endpoint_msg);
	}

	/**
	 * @brief Stores most recent measured encoder/joint state.
	 *
	 * Expected message layout:
	 * - `position`: at least `dof` entries (required)
	 * - `velocity`: optional; missing entries default to zero
	 */
	void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
	{
		if (msg->position.size() < dof)
		{
			RCLCPP_ERROR(this->get_logger(), "Received joint state message with incorrect size");
			return;
		}

		for (size_t i = 0; i < dof; ++i)
		{
			joint_position(static_cast<int>(i)) = msg->position[i];
			joint_velocity(static_cast<int>(i)) = (msg->velocity.size() > i) ? msg->velocity[i] : 0.0;
		}
	}

	/**
	 * @brief Stores most recent desired world (Cartesian) velocity command.
	 *
	 * Command is cached with a timestamp for use by the periodic control loop.
	 */
	void worldVelocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
	{
		std::lock_guard<std::mutex> lock(command_mutex_);
		reference_world_velocity_(0) = msg->linear.x;
		reference_world_velocity_(1) = msg->linear.y;
		reference_world_velocity_(2) = msg->linear.z;
		reference_world_velocity_(3) = msg->angular.x;
		reference_world_velocity_(4) = msg->angular.y;
		reference_world_velocity_(5) = msg->angular.z;
		last_velocity_time_ = steady_clock_.now();
	}

	/**
	 * @brief Damped least-squares pseudoinverse for a possibly non-square matrix.
	 */
	Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd &matrix, double damping) const
	{
		const Eigen::Index rows = matrix.rows();
		const Eigen::Index cols = matrix.cols();
		const double lambda2 = damping * damping;

		if (rows >= cols)
		{
			const Eigen::MatrixXd lhs = matrix.transpose() * matrix + lambda2 * Eigen::MatrixXd::Identity(cols, cols);
			return lhs.ldlt().solve(matrix.transpose());
		}

		const Eigen::MatrixXd lhs = matrix * matrix.transpose() + lambda2 * Eigen::MatrixXd::Identity(rows, rows);
		return matrix.transpose() * lhs.ldlt().solve(Eigen::MatrixXd::Identity(rows, rows));
	}

	// ROS interfaces.
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr world_velocity_sub_;
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr desired_control_pub_;
	rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr endpoint_publisher_;
	rclcpp::TimerBase::SharedPtr timer_;

	// Parameters/model constants.
	Eigen::VectorXd init_pos;	  // Initial joint positions [rad], size `dof`.
	Eigen::VectorXd link_lengths; // Link lengths [m], size equals configured chain length.
	kanga_arm_controller::ArmKinematics kinematics_{};
	std::string end_effector_config_{"roll_tool"};

	// Runtime state (measured/estimated).
	Eigen::VectorXd joint_position;			 // Joint positions [rad], size `dof`.
	Eigen::VectorXd joint_velocity;			 // Joint velocities [rad/s], size `dof`.
	Eigen::VectorXd desired_joint_position_; // Integrated desired joint positions (q_ref).
	Eigen::Vector3d endpoint_position;		 // End-effector Cartesian position [m].
	Eigen::Vector3d desired_world_position_; // Integrated desired Cartesian position [m].

	// Cached desired command from kanga_arm/world_state_control.
	Eigen::VectorXd reference_world_velocity_; // Reference world velocity [m/s, rad/s], size 6.
	rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
	rclcpp::Time last_velocity_time_;
	std::mutex command_mutex_;

	// Control loop configuration.
	std::vector<double> joint_min_limits_;
	std::vector<double> joint_max_limits_;
	std::vector<double> joint_max_velocity_;
	bool hold_position_on_stale_{true};
	double command_stale_timeout_s_{0.5};
	float control_time_step_ms;
	const size_t dof = 5;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<KangaArmController>());
	rclcpp::shutdown();
	return 0;
}
