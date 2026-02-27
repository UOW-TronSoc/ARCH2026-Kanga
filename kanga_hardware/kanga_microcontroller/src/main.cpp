#include "microcontroller_can_node.hpp"
#include "kanga_canbus/epoll_event_loop.hpp"

#include <thread>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    EpollEventLoop event_loop;
    auto node = std::make_shared<MicrocontrollerCanNode>("microcontroller_can_node");

    if (!node->init(&event_loop)) {
        RCLCPP_FATAL(node->get_logger(), "Failed to initialise microcontroller CAN node");
        return -1;
    }

    std::thread can_thread([&event_loop]() { event_loop.run_until_empty(); });

    rclcpp::spin(node);

    node->deinit();
    rclcpp::shutdown();
    can_thread.join();
    return 0;
}
