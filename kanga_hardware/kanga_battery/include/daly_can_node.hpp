#ifndef DALY_CAN_NODE_HPP
#define DALY_CAN_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include "kanga_battery/msg/battery_info.hpp"
#include "kanga_battery/msg/bms_status.hpp"
#include "kanga_canbus/socket_can.hpp"

#include <mutex>
#include <condition_variable>
#include <array>
#include <algorithm>
#include <linux/can.h>
#include <linux/can/raw.h>

// Bring placeholders into scope for std::bind callback bindings.
using std::placeholders::_1;
using std::placeholders::_2;

// Aliases for the custom message types to make code shorter and clearer.
using BatteryInfo = kanga_battery::msg::BatteryInfo;
using BMSStatus   = kanga_battery::msg::BmsStatus;

// -----------------------------------------------------------------------------
// DalyCanNode
// A ROS2 node that communicates with a Daly BMS over SocketCAN.
//
// Responsibilities:
//  - Initialize and manage the CAN socket interface
//  - Periodically send polling requests to the Daly BMS
//  - Decode received CAN frames into BatteryInfo and BMSStatus messages
//  - Publish these messages onto ROS topics
// -----------------------------------------------------------------------------
class DalyCanNode : public rclcpp::Node {
public:
    // Constructor: accepts a node name string.
    DalyCanNode(const std::string& node_name);

    // Initialize CAN interface with an Epoll event loop for asynchronous operation.
    // Returns true on success, false on failure.
    bool init(EpollEventLoop* event_loop);

    // De-initialize and release any CAN resources.
    void deinit();

private:
    // Callback for received CAN frames. Filters by node ID, decodes frames,
    // updates internal data structures, and triggers publishing when complete.
    void recv_callback(const can_frame& frame);

    // Timer callback: periodically sends request frames for all Daly command IDs.
    void request_daly_data(void);

    // Utility to build the extended CAN identifier used by Daly requests.
    // Format is (0x18 << 24) | (data_id << 16) | node_id.
    uint32_t build_extended_id(uint8_t data_id, uint16_t node_id);

    // Helper to verify that the frame length (DLC) matches expectation.
    // Logs a warning if mismatch is detected.
    inline bool verify_length(const std::string& name, uint8_t expected, uint8_t length);

    // ---------------- Member variables ----------------

    // CAN addressing
    uint16_t local_node_id_;   // This ROS node’s CAN node ID
    uint16_t daly_node_id_;    // Daly BMS’s fixed CAN node ID
    std::string interface_;    // CAN interface name (e.g. "can0" or "can1")

    // CAN interface object (SocketCAN abstraction).
    SocketCanIntf can_intf_ = SocketCanIntf();

    // BatteryInfo publisher state
    short int info_pub_flag_ = 0;                                // Bitmask tracking which fields are ready
    std::mutex bat_info_mutex_;                                  // Protects concurrent access to bat_info_
    BatteryInfo bat_info_ = BatteryInfo();                       // Aggregated battery info
    rclcpp::Publisher<BatteryInfo>::SharedPtr info_publisher_;   // ROS publisher for BatteryInfo

    // BMSStatus publisher state
    short int status_pub_flag_ = 0;                              // Bitmask tracking which fields are ready
    std::mutex daly_stat_mutex_;                                 // Protects concurrent access to daly_stat_
    BMSStatus daly_stat_ = BMSStatus();                          // Aggregated status info
    rclcpp::Publisher<BMSStatus>::SharedPtr stat_publisher_;     // ROS publisher for BMSStatus

    // Request timer parameters
    uint16_t req_period_ = 1;                                    // Period between request cycles (seconds)
    rclcpp::TimerBase::SharedPtr request_timer_;                 // ROS timer for periodic polling
};

#endif // DALY_CAN_NODE_HPP
