#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <raisim/World.hpp>
#include <raisim/RaisimServer.hpp>

#include <chrono>
#include <thread>
#include <string>
#include <cmath>
#include <mutex>
#include <algorithm>
#include <Eigen/Dense>

class RaisimBridge : public rclcpp::Node
{
public:
	RaisimBridge() : Node("raisim_bridge")
	{
		// Setup ROS2 parameter time step for simulation, timers and models
			pd_time_step_ms = this->declare_parameter<float>("pd_time_step_ms", 1.0);
			stop_hold_velocity_epsilon_ = this->declare_parameter<double>("sim_stop_hold_velocity_epsilon", 0.02);

		// Logging info
		RCLCPP_INFO(this->get_logger(), "Time step for simulation: %f ms", pd_time_step_ms);

		// Setup initial joint positions
		this->declare_parameter<std::vector<double>>("joint_initial_positions", std::vector<double>{});
		std::vector<double> joint_initial_positions;
		this->get_parameter("joint_initial_positions", joint_initial_positions);
		Eigen::VectorXd joint_pos = Eigen::Map<Eigen::VectorXd>(joint_initial_positions.data(), joint_initial_positions.size());
		const int N_joints = joint_pos.size();

		// Only joint positions are needed for a fixed-base arm
		init_state = joint_pos;

		// Set world timestep for simulation
		dt_ = pd_time_step_ms * 1e-3; // seconds
		world.setTimeStep(dt_);

		// Set default material properties (restitution, friction, adhesion)
		world.setDefaultMaterial(1.0, 0.2, 0.0);

		clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>(
			"/clock", rclcpp::QoS(10).best_effort());

		auto ground = world.addGround(0);
		ground->setAppearance("hidden");

		// Variable Gravity option
		// world.setGravity(Eigen::Vector3d(0, 0, 0));

		// Raisim Activation Key
		raisim::World::setActivationKey("$ENV{HOME}/.raisim");

		// Get robot URDF file path from description package
		std::string urdf_path_base = this->declare_parameter<std::string>("robot_description_path", "/default/path");
		std::string urdf_file = urdf_path_base + "/urdf/kanga_arm.urdf";

		// Load robot into RaiSim and give name
		robot = world.addArticulatedSystem(urdf_file);
		robot->setName("Kanga Arm");


		world.setGravity({0, 0, 0});

		// Remove collision between successive links only (adjacent joints)
		robot->ignoreCollisionBetween(0, 1);
		robot->ignoreCollisionBetween(1, 2);
		robot->ignoreCollisionBetween(2, 3);
		robot->ignoreCollisionBetween(3, 4);
		robot->ignoreCollisionBetween(4, 5);
		robot->ignoreCollisionBetween(5, 6);
		robot->ignoreCollisionBetween(6, 7);

		// Setup parameter sizes for generalised position, velocity, acceleration, force and damping and set to zero
		gc = Eigen::VectorXd::Zero(robot->getGeneralizedCoordinateDim());
		gv = Eigen::VectorXd::Zero(robot->getDOF());
		gf = Eigen::VectorXd::Zero(robot->getDOF());

		// Joint names used for ROS JointState publishing.
		const auto joint_names_param = this->declare_parameter<std::vector<std::string>>(
			"joint_names", std::vector<std::string>{});
		if (joint_names_param.size() == static_cast<size_t>(robot->getDOF()))
		{
			joint_names_ = joint_names_param;
		}
		else
		{
			if (!joint_names_param.empty())
			{
				RCLCPP_WARN(this->get_logger(),
							"Ignoring joint_names: expected %zu names, got %zu",
							static_cast<size_t>(robot->getDOF()),
							joint_names_param.size());
			}

			joint_names_.clear();
			joint_names_.reserve(static_cast<size_t>(robot->getDOF()));
			for (size_t i = 0; i < static_cast<size_t>(robot->getDOF()); ++i)
			{
				joint_names_.push_back("arm_j" + std::to_string(i + 1));
			}
			RCLCPP_WARN(this->get_logger(),
						"Using default joint names arm_j1..arm_j%zu. Set joint_names in simulation.yaml to override.",
						static_cast<size_t>(robot->getDOF()));
		}

		// PD gains from params so they can be tuned via YAML without recompiling.
		const double kp_default = this->declare_parameter<double>("kp", 5000.0);
		const double kd_default = this->declare_parameter<double>("kd", 500.0);
		const auto kp_gains_param = this->declare_parameter<std::vector<double>>("kp_gains", std::vector<double>{});
		const auto kd_gains_param = this->declare_parameter<std::vector<double>>("kd_gains", std::vector<double>{});
		const size_t expected_kp_size = static_cast<size_t>(robot->getGeneralizedCoordinateDim());
		const size_t expected_kd_size = static_cast<size_t>(robot->getDOF());

		kp_ = Eigen::VectorXd::Constant(robot->getGeneralizedCoordinateDim(), kp_default);
		kd_ = Eigen::VectorXd::Constant(robot->getDOF(), kd_default);

		if (!kp_gains_param.empty())
		{
			if (kp_gains_param.size() == expected_kp_size)
			{
				kp_ = Eigen::Map<const Eigen::VectorXd>(kp_gains_param.data(), kp_gains_param.size());
			}
			else
			{
				RCLCPP_WARN(this->get_logger(),
							"Ignoring kp_gains: expected %zu elements, got %zu",
							expected_kp_size,
							kp_gains_param.size());
			}
		}

		if (!kd_gains_param.empty())
		{
			if (kd_gains_param.size() == expected_kd_size)
			{
				kd_ = Eigen::Map<const Eigen::VectorXd>(kd_gains_param.data(), kd_gains_param.size());
			}
			else
			{
				RCLCPP_WARN(this->get_logger(),
							"Ignoring kd_gains: expected %zu elements, got %zu",
							expected_kd_size,
							kd_gains_param.size());
			}
		}

		RCLCPP_INFO(this->get_logger(), "PD gains configured (kp size=%ld, kd size=%ld)", kp_.size(), kd_.size());

		q_ref = joint_pos;
		qd_ref = Eigen::VectorXd::Zero(N_joints);

		// Set siulation position and velocity
		robot->setGeneralizedCoordinate(init_state);
		robot->setGeneralizedVelocity(gv);
		robot->setGeneralizedForce(gf);

		// CoM Ball Display
		comSphere = server.addVisualSphere("viz_sphere", 0.01, 1, 0, 0, 1);

		server.setMap("dune");

		// Setup raisim server
		server.launchServer(8080);

		// Wait for server connection
		RCLCPP_INFO(this->get_logger(), "Awaiting Connection to raisim server");
		while (!server.isConnected())
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		};

		RCLCPP_INFO(this->get_logger(), "Server Connected");

		std::this_thread::sleep_for(std::chrono::milliseconds(2000));

		RCLCPP_INFO(this->get_logger(), "RaisimBridge Node Initialised");

		// Focus on the robot
		server.focusOn(robot);

		// Create Publisher for robot joint states
		joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

		// Create subscription to control node topic for joint effort commands
		desired_cmd_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
			"joint_desired_control", 10,
			std::bind(&RaisimBridge::effortCommandCallback, this, std::placeholders::_1));

		// Apply PD gains to the articulated system
		robot->setPdGains(kp_, kd_);

		// Create timer to update the simulation
		timer_ = this->create_wall_timer(
			std::chrono::duration<double>(dt_),
			std::bind(&RaisimBridge::update, this));

		// set_gc_srv_ = this->create_service<quadruped_interfaces::srv::SetGeneralizedCoordinate>(
		// 	"set_generalized_coordinate",
		// 	std::bind(&RaisimBridge::setGcCallback, this, std::placeholders::_1, std::placeholders::_2));

		// Set start time checking dimulation time displacement
		startTime = std::chrono::high_resolution_clock::now();
	}

	/*
	 * Destructor to clean up resources and safely shut down the Raisim server
	 */
	~RaisimBridge() override
	{
		// Call cleanup function
		cleanup();
	}

	/*
	 * Cleanup function to ensure the Raisim server is properly shut down
	 * This function is called in the destructor and can also be called manually
	 * It calculates the total simulation time and logs it before shutting down the server
	 */
	void cleanup()
	{
		// If shutdown hasnt occured already
		if (!shutdown_called_)
		{
			// Calculate simulation time displacement
			auto endTime = std::chrono::high_resolution_clock::now();
			auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
			RCLCPP_INFO(
				this->get_logger(),
				"\nRan for %.3f s\nSimulated time: %.3f s\nShutting down RaisimBridge",
				duration * 1e-3,
				sim_time_ns_ * 1e-9);

			// Kill the raisim server
			server.killServer();

			// Ensure shutdown not triggered again
			shutdown_called_ = true;
		}
	}

private:
	/*
	 * Function to update the simulation and publish joint states
	 * This function is called at a fixed time interval defined by the timer
	 */
	void update()
	{
		// server.integrateWorldThreadSafe();
		// return;
		// RCLCPP_DEBUG(this->get_logger(), "Received joint effort command");
		// server.integrateWorldThreadSafe();
		// return;

		// Update internal state vectors
		gc = robot->getGeneralizedCoordinate().e();
		gv = robot->getGeneralizedVelocity().e();
		gf = robot->getGeneralizedForce().e();

		// Place visualization sphere at the end-effector with a fixed offset
		static constexpr int kEndLinkBodyIndex = 5; // link_5 is the final body
		const Eigen::Vector3d tool_offset(-0.0265, 0.0, -0.21657356); // meters
		raisim::Vec<3> ee_pos_rs;
		raisim::Mat<3, 3> ee_rot_rs;
		robot->getBodyPosition(kEndLinkBodyIndex, ee_pos_rs);
		robot->getBodyOrientation(kEndLinkBodyIndex, ee_rot_rs); // world_R_link
		Eigen::Vector3d sphere_pos = ee_pos_rs.e() + ee_rot_rs.e() * tool_offset;
		comSphere->setPosition(sphere_pos[0], sphere_pos[1], sphere_pos[2]);

		// Build a single time stamp for this step in sim time
		builtin_interfaces::msg::Time stamp;
		stamp.sec = static_cast<int32_t>(sim_time_ns_ / 1000000000LL);
		stamp.nanosec = static_cast<uint32_t>(sim_time_ns_ % 1000000000LL);

		// Send stamp to /clock topic for sim time
		rosgraph_msgs::msg::Clock clk;
		clk.clock = stamp;
		clock_pub_->publish(clk);

		// Setup the joinstate message
		sensor_msgs::msg::JointState js;
		int dof = robot->getDOF();
		js.header.stamp = stamp;
		js.name = joint_names_;
		js.position.resize(dof);
		js.velocity.resize(dof);
		js.effort.resize(dof);

			// Push PD targets into Raisim's internal controller
			Eigen::VectorXd q_ref_local;
			Eigen::VectorXd qd_ref_local;
		{
			std::lock_guard<std::mutex> lock(target_mutex_);
				q_ref_local = q_ref;
				qd_ref_local = qd_ref;
			}

			// In velocity-command mode, when target velocity is near zero, hold current position
			// in sim to avoid spring-back to an integrated reference.
			const size_t q_dim = static_cast<size_t>(q_ref_local.size());
			const size_t qd_dim = static_cast<size_t>(qd_ref_local.size());
			const size_t gc_dim = static_cast<size_t>(gc.size());
			const size_t common_dim = std::min({q_dim, qd_dim, gc_dim});
			for (size_t i = 0; i < common_dim; ++i)
			{
				if (std::abs(qd_ref_local[static_cast<Eigen::Index>(i)]) <= stop_hold_velocity_epsilon_)
				{
					q_ref_local[static_cast<Eigen::Index>(i)] = gc[static_cast<Eigen::Index>(i)];
				}
			}
			robot->setPdTarget(q_ref_local, qd_ref_local);

		for (int i = 0; i < dof; ++i)
		{
			double pos = gc[i];
			double vel = gv[i];
			double eff = gf[i];
			if (!std::isfinite(pos) || !std::isfinite(vel) || !std::isfinite(eff))
			{
				RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
									 "Non-finite joint state detected at index %d. Clamping to zero.", i);
				pos = 0.0;
				vel = 0.0;
				eff = 0.0;
			}

			// Add each joint to the jointstate message
			js.position[i] = pos;
			js.velocity[i] = vel;
			js.effort[i] = eff;
		}

		// Publish joint states
		joint_state_pub->publish(js);

		// Step simulation
		server.integrateWorldThreadSafe();

		// Update simulated time
		sim_time_ns_ += static_cast<int64_t>(dt_ * 1e9);
	}

	/*
	 * Callback function to handle incoming joint effort commands
	 * This function is triggered when a new message is received on the "joint_desired_control" topic
	 * It saves the control reference commands to memory for the next simulation step
	 *
	 * @param msg The incoming message containing joint positions, velocities, and efforts
	 */
	void effortCommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
	{

		// RCLCPP_INFO(this->get_logger(), "test");

		// Make sure control commands match the robot dof
		if (msg->position.size() != robot->getDOF())
		{
			RCLCPP_WARN(this->get_logger(), "Received effort command of wrong size: %zu (expected %zu)", msg->position.size(), robot->getDOF());
			return;
		}

		std::lock_guard<std::mutex> lock(target_mutex_);

		// Save desired positions
		for (size_t i = 0; i < msg->position.size(); ++i)
		{
			q_ref[i] = msg->position[i];
		}

		// Save desired velocities if provided; otherwise zero
		if (msg->velocity.size() == msg->position.size())
		{
			for (size_t i = 0; i < msg->velocity.size(); ++i)
				qd_ref[i] = msg->velocity[i];
		}
		else
		{
			qd_ref.setZero();
		}

		return;
	}


	// Raisim control variables
	bool shutdown_called_ = false;
	raisim::World world;
	raisim::RaisimServer server{&world};
	raisim::ArticulatedSystem *robot;
	raisim::Visuals *comSphere;
	Eigen::VectorXd gc, gv, gf, init_state;
	Eigen::VectorXd q_ref, qd_ref;
	Eigen::VectorXd kp_, kd_;
	std::vector<std::string> joint_names_;
	std::mutex target_mutex_;

	// Declare ROS2 publishers, sibscribers and timers
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr desired_cmd_sub_;
	rclcpp::TimerBase::SharedPtr timer_;

	// Declare internal timer variables
	std::chrono::_V2::system_clock::time_point startTime;

	// Declare parameters for simulation and control
	float pd_time_step_ms;
	rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
	int64_t sim_time_ns_ = 0; // simulated time in nanoseconds
		double dt_ = 0.0;		  // seconds, equals world timestep
		double stop_hold_velocity_epsilon_{0.02};

};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);

	auto node = std::make_shared<RaisimBridge>();

	// Trigger node cleanup on shutdown (e.g., Ctrl+C)
	rclcpp::on_shutdown([node]()
						{ node->cleanup(); });

	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
