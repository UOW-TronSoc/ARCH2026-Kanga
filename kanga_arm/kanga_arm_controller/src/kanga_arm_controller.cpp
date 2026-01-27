#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Dense>
#include <vector>

class KangaArmController : public rclcpp::Node
{
public:
	KangaArmController() : Node("kanga_arm_controller")
	{
		// Setup ROS2 parameter time step for simulation, timers and models
		control_time_step_ms = this->declare_parameter<float>("control_time_step_ms", 1.0f);
		init_pos = this->declare_parameter<std::vector<double>>(
			"joint_initial_positions",
			std::vector<double>(dof, 0.0));
		link_lengths = this->declare_parameter<std::vector<double>>("link_lengths", std::vector<double>{});

		// Initialise variables for joint positions and velocities
		joint_position = Eigen::VectorXd::Zero(dof);
		joint_velocity = Eigen::VectorXd::Zero(dof);

		if (init_pos.size() >= dof)
		{
			for (size_t i = 0; i < dof; ++i)
			{
				joint_position(static_cast<int>(i)) = init_pos[i];
			}
		}
		else if (!init_pos.empty())
		{
			RCLCPP_WARN(this->get_logger(), "joint_initial_positions size is less than %zu; using zeros", dof);
		}

		endpoint_position = forwardKinematics(joint_position);

		// Set up subscription to encoder feedback for joint states
		joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
			"joint_states", 10,
			std::bind(&KangaArmController::jointStateCallback, this, std::placeholders::_1));

		// Set up publishers for desired control effort
		desired_control_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_desired_control", 10);

		// Set up publisher for endpoint pose
		endpoint_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("endpoint", 10);

		// Create timer to update the control commands
		timer_ = rclcpp::create_timer(
			this->get_node_base_interface(),
			this->get_node_timers_interface(),
			this->get_clock(),
			std::chrono::microseconds((int)(control_time_step_ms * 1000)),
			std::bind(&KangaArmController::controlCommands, this));

		// Feedback for controller start
		RCLCPP_INFO(this->get_logger(), "Kanga Arm Controller Node started");
	}

private:
	/*
	 * Callback that calculated the desired joint states based on the current joint states and desired trajectory.
	 * Current implementation holds the current state for a 5-DOF arm.
	 */
	void controlCommands()
	{
		// If sim time has not started yet, do nothing
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

		for (size_t i = 0; i < dof; ++i)
		{
			control_effort.position[i] = joint_position(static_cast<int>(i));
			control_effort.velocity[i] = 0.0;
			control_effort.effort[i] = 0.0;
		}

		desired_control_pub_->publish(control_effort);

		// Publish a single endpoint pose
		endpoint_position = forwardKinematics(joint_position);
		geometry_msgs::msg::Pose endpoint_msg;
		endpoint_msg.position.x = endpoint_position.x();
		endpoint_msg.position.y = endpoint_position.y();
		endpoint_msg.position.z = endpoint_position.z();
		endpoint_msg.orientation.x = 0.0;
		endpoint_msg.orientation.y = 0.0;
		endpoint_msg.orientation.z = 0.0;
		endpoint_msg.orientation.w = 1.0;
		endpoint_publisher_->publish(endpoint_msg);
	}

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

	/*
	 * Computes the Jacobian matrix for a 5-DOF arm based on the joint angles.
	 * Stub implementation until the control model is finalized.
	 */
	Eigen::MatrixXd computeJacobian(const Eigen::VectorXd &q)
	{
		(void)q;
		// TODO: Implement the 5-DOF Jacobian.
		return Eigen::MatrixXd::Zero(6, dof);
	}

	/*
	 * Computes the end-effector position for a 5-DOF arm based on the joint angles.
	 * Stub implementation until the control model is finalized.
	 */
	Eigen::Vector3d forwardKinematics(const Eigen::VectorXd &q)
	{
		(void)q;
		// TODO: Implement the 5-DOF forward kinematics.
		return Eigen::Vector3d::Zero();
	}

	/*
	 * Computes the inverse kinematics for a 5-DOF arm based on the desired pose.
	 */
	Eigen::VectorXd inverseKinematics(const Eigen::Vector3d &pos)
	{
		(void)pos;
		// NOTE: 5-DOF IK is near impossible to solve; stub for now.
		return Eigen::VectorXd::Zero(dof);
	}

	// Declaration for ROS2 subscriptions and publishers
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr desired_control_pub_;
	rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr endpoint_publisher_;
	rclcpp::TimerBase::SharedPtr timer_;

	// Declaration for model parameters and variables
	std::vector<double> init_pos;
	std::vector<double> link_lengths;

	Eigen::VectorXd joint_position;  // size 5, joint positions
	Eigen::VectorXd joint_velocity;  // size 5, joint velocities
	Eigen::Vector3d endpoint_position; // end-effector position

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
