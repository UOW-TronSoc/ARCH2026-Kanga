#include "microcontroller_can_node.hpp"
#include "kanga_canbus/epoll_event_loop.hpp"

#include <thread>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    EpollEventLoop event_loop;
    auto node = std::make_shared<MicrocontrollerCanNode>("core_can_node");

    const int64_t current_node_id = node->get_parameter("node_id").as_int();
    if (current_node_id == 0) {
        node->set_parameter(rclcpp::Parameter("node_id", static_cast<int64_t>(0x115)));
    }

    const int64_t current_mask = node->get_parameter("node_id_mask").as_int();
    if (current_mask == 0) {
        node->set_parameter(rclcpp::Parameter("node_id_mask", static_cast<int64_t>(0x7FF)));
    }

    const int64_t current_shift = node->get_parameter("node_id_shift").as_int();
    if (current_shift != 0) {
        node->set_parameter(rclcpp::Parameter("node_id_shift", static_cast<int64_t>(0)));
    }

    if (node->get_parameter("tx_base_id").as_int() == 0) {
        node->set_parameter(rclcpp::Parameter("tx_base_id", static_cast<int64_t>(0)));
    }

    if (node->get_parameter("expect_extended_id").as_bool()) {
        node->set_parameter(rclcpp::Parameter("expect_extended_id", false));
    }

    if (node->get_parameter("force_extended_tx").as_bool()) {
        node->set_parameter(rclcpp::Parameter("force_extended_tx", false));
    }

    if (!node->init(&event_loop)) {
        RCLCPP_FATAL(node->get_logger(), "Failed to initialise core CAN node");
        rclcpp::shutdown();
        return -1;
    }

    std::thread can_thread([&event_loop]() { event_loop.run_until_empty(); });

    rclcpp::spin(node);

    node->deinit();
    rclcpp::shutdown();
    can_thread.join();
    return 0;
}
