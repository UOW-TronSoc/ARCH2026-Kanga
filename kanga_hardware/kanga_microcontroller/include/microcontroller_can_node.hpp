#ifndef MICROCONTROLLER_CAN_NODE_HPP
#define MICROCONTROLLER_CAN_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include "kanga_microcontroller/msg/microcontroller_frame.hpp"
#include "kanga_canbus/socket_can.hpp"

#include <mutex>
#include <string>
#include <linux/can.h>
#include <linux/can/raw.h>

using std::placeholders::_1;

using MicrocontrollerFrame = kanga_microcontroller::msg::MicrocontrollerFrame;

class MicrocontrollerCanNode : public rclcpp::Node {
public:
    explicit MicrocontrollerCanNode(const std::string &node_name);

    bool init(EpollEventLoop *event_loop);
    void deinit();

private:
    void recv_callback(const can_frame &frame);
    void tx_callback(const MicrocontrollerFrame::SharedPtr msg);

    uint32_t extract_node_id(uint32_t can_id) const;
    uint32_t sanitize_node_id(uint32_t node_id) const;

    std::string interface_;
    uint32_t node_id_ = 0;
    uint32_t node_id_mask_ = 0;
    uint8_t node_id_shift_ = 0;
    bool filter_by_node_ = true;
    bool expect_extended_id_ = true;
    bool auto_header_stamp_ = true;
    bool drop_remote_frames_ = true;
    bool allow_tx_id_override_ = true;
    bool force_extended_tx_ = true;
    uint32_t tx_base_id_ = 0;
    bool debug_logging_ = false;

    SocketCanIntf can_intf_ = SocketCanIntf();

    rclcpp::Publisher<MicrocontrollerFrame>::SharedPtr rx_publisher_;
    rclcpp::Subscription<MicrocontrollerFrame>::SharedPtr tx_subscription_;

    mutable std::mutex tx_mutex_;

    inline void debug_log(const std::string &message) const {
        if (debug_logging_) {
            RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
        }
    }
};

#endif // MICROCONTROLLER_CAN_NODE_HPP
