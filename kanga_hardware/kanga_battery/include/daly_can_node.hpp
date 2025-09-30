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

using std::placeholders::_1;
using std::placeholders::_2;

using BatteryInfo = kanga_battery::msg::BatteryInfo;
using BMSStatus = kanga_battery::msg::BmsStatus;


class DalyCanNode : public rclcpp::Node {
public:
    DalyCanNode(const std::string& node_name);
    bool init(EpollEventLoop* event_loop); 
    void deinit();
private:
    void recv_callback(const can_frame& frame);
    void request_daly_data(void);
    uint32_t build_extended_id(uint8_t data_id, uint16_t node_id);
    inline bool verify_length(const std::string&name, uint8_t expected, uint8_t length);
    
    uint16_t local_node_id_;
    uint16_t daly_node_id_;
    std::string interface_;
    SocketCanIntf can_intf_ = SocketCanIntf();
    
    short int info_pub_flag_ = 0;
    std::mutex bat_info_mutex_;
    BatteryInfo bat_info_ = BatteryInfo();
    rclcpp::Publisher<BatteryInfo>::SharedPtr info_publisher_;
    
    short int status_pub_flag_ = 0;
    std::mutex daly_stat_mutex_;
    BMSStatus daly_stat_ = BMSStatus();
    rclcpp::Publisher<BMSStatus>::SharedPtr stat_publisher_;

    uint16_t req_period_ = 1;
    rclcpp::TimerBase::SharedPtr request_timer_;

};

#endif // DALY_CAN_NODE_HPP