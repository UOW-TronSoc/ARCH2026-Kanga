#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Dense>
#include <vector>
#include <mutex>
#include <chrono>

/**
 * @brief Main arm control node.
 *
 * Data flow:
 * 1) Reads measured joint state from `joint_states`.
 * 2) Reads desired joint-space command from `/kanga_arm/joint_control`.
 * 3) Runs `controlCommands()` at 1 kHz (configurable).
 * 4) Publishes joint command output and end-effector pose estimate.
 *
 * Kinematic methods are currently stubs and are intentionally left
 * as placeholders until the final arm model is available.
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
		const auto link_lengths_param = this->declare_parameter<std::vector<double>>(
			"link_lengths", std::vector<double>{});

		// Initialize tracked joint state vectors.
		init_pos = Eigen::VectorXd::Zero(dof);
		latest_velocity_ = Eigen::VectorXd::Zero(dof);
		joint_position = Eigen::VectorXd::Zero(dof);
		joint_velocity = Eigen::VectorXd::Zero(dof);

		if (init_pos_param.size() >= dof)
		{
			for (size_t i = 0; i < dof; ++i)
			{
				init_pos(static_cast<Eigen::Index>[i]) = init_pos_param[i];
				joint_position(static_cast<Eigen::Index>[i]) = init_pos_param[i];
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
			link_lengths(static_cast<Eigen::Index>[i]) = link_lengths_param[i];
		}

		// Initialize published endpoint with the best available estimate.
		endpoint_position = forwardKinematics(joint_position);

		// Encoder/state feedback subscription.
		joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
			"joint_states", 10,
			std::bind(&KangaArmController::jointStateCallback, this, std::placeholders::_1));

		// Desired joint command subscription.
		joint_control_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
			"/kanga_arm/joint_control", 10,
			std::bind(&KangaArmController::jointControlCallback, this, std::placeholders::_1));


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
		RCLCPP_INFO(this->get_logger(), "Kanga Arm Controller Node started");
	}

private:
	/**
	 * @brief Periodic control loop callback.
	 *
	 * Current behavior is conservative by design while kinematics are under development:
	 * - passes through measured joint velocities as placeholder command velocity,
	 * - publishes zero position/effort commands,
	 * - computes and publishes an endpoint pose using forward kinematics stub.
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

		for (size_t i = 0; i < dof; ++i)
		{
			control_effort.position[i] = 0.0;
			control_effort.velocity[i] = joint_velocity(static_cast<int>[i]);
			control_effort.effort[i] = 0.0;
		}

		desired_control_pub_->publish(control_effort);

		// Publish latest endpoint estimate from current measured joint state.
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
			joint_position(static_cast<int>[i]) = msg->position[i];
			joint_velocity(static_cast<int>[i]) = (msg->velocity.size() > i) ? msg->velocity[i] : 0.0;
		}
	}

	/**
	 * @brief Stores most recent desired joint velocity command.
	 *
	 * Command is cached with a timestamp for use by the periodic control loop.
	 * Missing velocity entries are treated as zero.
	 */
	void jointControlCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
	{
		if (msg->velocity.size() < dof)
		{
			RCLCPP_WARN(this->get_logger(), "Joint control message has fewer than %zu velocities; missing entries treated as zero", dof);
		}

		std::lock_guard<std::mutex> lock(command_mutex_);
		for (size_t i = 0; i < dof; ++i)
		{
			latest_velocity_(static_cast<Eigen::Index>[i]) = (msg->velocity.size() > i) ? msg->velocity[i] : 0.0;
		}
		last_velocity_time_ = steady_clock_.now();
	}

	/**
	 * @brief Computes 6xDOF geometric Jacobian at joint configuration `q`.
	 * @note Stub implementation until the arm model is finalized.
	 */
	Eigen::MatrixXd computeJacobian(const Eigen::VectorXd &q)
	{
		(void)q;
		// TODO: Implement Jacobian for the final chain definition.
		
		Eigen::MatrixXd Jacobian(4,6); // Do Jacobian Calcs for EE Mode

		Jacobian = [
									390.0*sin(q(0))*sin(q(1))*sin(q(2)) - 450.0*sin(q(0))*sin(q(1)) - 390.0*cos(q(1))*cos(q(2))*sin(q(0)) - 88.94*cos(q(0)) + 241.72*cos(q(1) + q(2))*sin(q(0))*sin(q(3)) + 241.72*sin(q(1) + q(2))*cos(q(3))*sin(q(0)), 
									450.0*cos(q(0))*cos(q(1)) - 390.0*cos(q(0))*cos(q(1))*sin(q(2)) - 390.0*cos(q(0))*cos(q(2))*sin(q(1)) - 241.72*cos(q(1) + q(2))*cos(q(0))*cos(q(3)) + 241.72*sin(q(1) + q(2))*cos(q(0))*sin(q(3)), 
									241.72*sin(q(1) + q(2))*cos(q(0))*sin(q(3)) - 390.0*cos(q(0))*cos(q(2))*sin(q(1)) - 241.72*cos(q(1) + q(2))*cos(q(0))*cos(q(3)) - 390.0*cos(q(0))*cos(q(1))*sin(q(2)), 
									241.72*sin(q(1) + q(2))*cos(q(0))*sin(q(3)) - 241.72*cos(q(1) + q(2))*cos(q(0))*cos(q(3));

									450.0*cos(q(0))*sin(q(1)) - 88.94*sin(q(0)) + 390.0*cos(q(0))*cos(q(1))*cos(q(2)) - 390.0*cos(q(0))*sin(q(1))*sin(q(2)) - 241.72*cos(q(1) + q(2))*cos(q(0))*sin(q(3)) - 241.72*sin(q(1) + q(2))*cos(q(0))*cos(q(3)), 
									450.0*cos(q(1))*sin(q(0)) - 390.0*cos(q(1))*sin(q(0))*sin(q(2)) - 390.0*cos(q(2))*sin(q(0))*sin(q(1)) - 241.72*cos(q(1) + q(2))*cos(q(3))*sin(q(0)) + 241.72*sin(q(1) + q(2))*sin(q(0))*sin(q(3)), 
									241.72*sin(q(1) + q(2))*sin(q(0))*sin(q(3)) - 390.0*cos(q(2))*sin(q(0))*sin(q(1)) - 241.72*cos(q(1) + q(2))*cos(q(3))*sin(q(0)) - 390.0*cos(q(1))*sin(q(0))*sin(q(2)), 
									241.72*sin(q(1) + q(2))*sin(q(0))*sin(q(3)) - 241.72*cos(q(1) + q(2))*cos(q(3))*sin(q(0));

									0,  
									241.72*sin(q(1) + q(2) + q(3)) - 390.0*cos(q(1) + q(2)) - 450.0*sin((1)), 
									241.72*sin(q(1) + q(2) + q(3)) - 390.0*cos(q(1) + q(2)),  
									241.72*sin(q(1) + q(2) + q(3));

	  							-0.5*cos(q(1) - q(0) + q(2) + q(3)) - 0.5*cos(q(0) + q(1) + q(2) + q(3)),          
									0.5*cos(q(1) - q(0) + q(2) + q(3)) - 0.5*cos(q(0) + q(1) + q(2) + q(3)),                                          
									0.5*cos(q(1) - q(0) + q(2) + q(3)) - 0.5*cos(q(0) + q(1) + q(2) + q(3)),                   
									0.5*cos(q(1) - q(0) + q(2) + q(3)) - 0.5*cos(q(0) + q(1) + q(2) + q(3));

									-1.0*sin(q(0)),    
									0,          
									0,                          
									0;

									- 0.5*sin(q(1) - q(0) + q(2) + q(3)) - 0.5*sin(q(0) + q(1) + q(2) + q(3)), 
									0.5*sin(q(1) - q(0) + q(2) + q(3)) - 0.5*sin(q(0) + q(1) + q(2) + q(3)),  
									0.5*sin(q(1) - q(0) + q(2) + q(3)) - 0.5*sin(q(0) + q(1) + q(2) + q(3)),                   
									0.5*sin(q(1) - q(0) + q(2) + q(3)) - 0.5*sin(q(0) + q(1) + q(2) + q(3))];
 
		return Jacobian;
	}

	/**
	 * @brief Computes Cartesian end-effector position from joint configuration `q`.
	 * @note Stub implementation until the arm model is finalized.
	 */
	Eigen::VectorXd forwardKinematics(const Eigen::VectorXd &q, bool isScoop)
	{
		(void)q;
		// TODO: Implement forward kinematics for the final chain definition.

		//Establish variable theta	
		double theta[5] = {q[0], q[1] + 0.5f*M_PI, q[2] + 0.5f*M_PI, q[3]}; // Removed the roll joint

		std::vector<Eigen::Matrix4d> T(q.size(), Eigen::Matrix4d::Identity());
		int n = q.size();


		for (int i=0; i<= (n-2); i++){

			T[i] << cos(theta[i]), 	 -cos(alpha[i]) * sin(theta[i]), sin(alpha[i]) * sin(theta[i]),  a[i] * cos(theta[i]),
							sin(theta[i]),    cos(alpha[i]) * cos(theta[i]), -sin(alpha[i]) * cos(theta[i]), a[i] * sin(theta[i]),
            	0,                sin(alpha[i]),                 cos(alpha[i]),                  d[i],
            	0,               0,                              0,                              1   

		}
		if (isScoop){
			
			double EndEffDH[4] = {0, 230, 0, 0}; // Add D-H parameters for scoop 

		}else{ //Normal E-E operation, includes roll joint

			double EndEffDH[4] = {0,241.725, 0, 0};

		}
		
		T[n-1] << cos(EndEfDH[0]), 	 -cos(EndEffDH[3]) * sin(EndEfDH[0]), sin(EndEffDH[3]) * sin(EndEfDH[0]),  EndEffDH[2] * cos(EndEfDH[0]),
							sin(EndEfDH[0]),    cos(EndEffDH[3]) * cos(EndEfDH[0]), -sin(EndEffDH[3]) * cos(EndEfDH[0]), EndEffDH[2] * sin(EndEfDH[0]),
							0,                	sin(EndEffDH[3]),                 cos(EndEffDH[3]),                  EndEffDH[1],
							0,               		0,                              0,                              1                      
		
		Eigen::Matrix4d T_solution = T[0];
		for (int i=0; i<= (n-1); i++){
			T_solution = T_solution * T[i+1];
		}
		double X = T_solution(3,0);
		double Y = T_solution(3,1);
		double Z = T_solution(3,2);
		double Rx = T_solution(2,0);
		double Ry = T_solution(2,1); 
		double Rz = T_solution(2,2);

		Eigen::VectorXd ToolConfigVector[6] = {X, Y, Z, Rx, Ry, Rz};
		return ToolConfigVector;
	}

	/**
	 * @brief Solves inverse kinematics for desired Cartesian position.
	 * @note Placeholder: currently returns zeros.
	 */
	Eigen::VectorXd inverseKinematics(const Eigen::Vector3d &pos)
	{
		(void)pos;
		// NOTE: Real implementation will likely require constraints and numerical solve.
		return Eigen::VectorXd::Zero(dof);
	}

	// ROS interfaces.
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_control_sub_;
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr desired_control_pub_;
	rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr endpoint_publisher_;
	rclcpp::TimerBase::SharedPtr timer_;

	// Parameters/model constants.
	Eigen::VectorXd init_pos;      // Initial joint positions [rad], size `dof`.
	Eigen::VectorXd link_lengths;  // Link lengths [m], size equals configured chain length.

	// Runtime state (measured/estimated).
	Eigen::VectorXd joint_position;   // Joint positions [rad], size `dof`.
	Eigen::VectorXd joint_velocity;   // Joint velocities [rad/s], size `dof`.
	Eigen::VectorXd endpoint_position; // End-effector Cartesian position [m].

	// Cached desired command from /kanga_arm/joint_control.
	Eigen::VectorXd latest_velocity_;  // Desired joint velocities [rad/s], size `dof`.
	rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
	rclcpp::Time last_velocity_time_;
	std::mutex command_mutex_;

	// Control loop configuration.
	float control_time_step_ms;
	const size_t dof = 5;

	
	double d[5] = {84, 111, -90.5, 68.44};
	double a[5] = {0, -449.997, -390, 0};
	double alpha[5] = {-0.5*M_PI, 0, 0, 0.5*M_PI};
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<KangaArmController>());
	rclcpp::shutdown();
	return 0;
}
double d[5] = {84, 111, -90.5, 68.44};
	double a[5] = {0, -449.997, -390, 0};
	double alpha[5] = {-0.5*M_PI, 0, 0, 0.5*M_PI};