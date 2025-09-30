#include "daly_can_node.hpp"
#include "kanga_canbus/epoll_event_loop.hpp"
#include "kanga_canbus/socket_can.hpp"
#include <thread>

int main(int argc, char* argv[]) {
    // Init Ros2
    rclcpp::init(argc, argv);

    // Create the epoll-based event loop for handling CAN socket events.
    EpollEventLoop event_loop;

    // Create node
    auto can_node = std::make_shared<DalyCanNode>("DalyCanNode");

    // Initialize the CAN interface inside the node with the event loop.
    if (!can_node->init(&event_loop)) return -1;

    // Launch a background thread that runs the epoll event loop until no more events.
    // This thread will continuously service CAN frames and invoke recv_callback().
    std::thread can_event_loop([&event_loop]() { event_loop.run_until_empty(); });

    // ROS2 executor loop
    rclcpp::spin(can_node);

    // Clean up CAN resources
    can_node->deinit();

    // Shutdown ROS2
    rclcpp::shutdown();
    return 0;
}
