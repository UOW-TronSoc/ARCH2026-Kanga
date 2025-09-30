#include "daly_can_node.hpp"
#include "kanga_canbus/epoll_event_loop.hpp"
#include "kanga_canbus/byte_swap.hpp"
#include <sys/eventfd.h>
#include <chrono>
#include <bitset>

// Command mapping for each canbus ID
enum class DalyCmdId : uint32_t
{
    kTotalVoltageCurrentSOC = 0x90, // BatteryInfo    - publisher
    kCellVoltageFrame1 = 0x95,      // BatteryInfo    - publisher (multi-frame)
    kMaxMinCellVoltage = 0x91,      // BatteryInfo    - publisher
    kMaxMinTemperature = 0x92,      // BatteryInfo    - publisher
    kChargeDischargeState = 0x93,   // BmsStatus      - publisher
    kStatusInfo1 = 0x94,            // BmsStatus      - publisher
    kCellBalanceState = 0x97,       // BmsStatus      - publisher
    kBatteryFaultStatus = 0x98,     // BmsStatus      - publisher
    kCellTemperatureFrame0 = 0x96,  // (optional)     - future use
};

DalyCanNode::DalyCanNode(const std::string &node_name) : rclcpp::Node(node_name)
{
     // Declare ROS parameters
    this->declare_parameter<std::string>("interface", "can1");
    this->declare_parameter<uint16_t>("local_node_id", 0x0140); // your ROS node ID
    this->declare_parameter<uint16_t>("daly_node_id", 0x0001);  // Daly's fixed node ID
    this->declare_parameter<uint16_t>("req_period", 1.0);

    // Read parameters into member variables.
    this->get_parameter("interface", interface_);
    this->get_parameter("local_node_id", local_node_id_);
    this->get_parameter("daly_node_id", daly_node_id_);
    this->get_parameter("req_period", req_period_);

    // Publisher for battery information
    rclcpp::QoS bat_info_qos(rclcpp::KeepAll{});
    info_publisher_ = rclcpp::Node::create_publisher<BatteryInfo>("battery_info", bat_info_qos);

    // Publisher for Daly BMS Status
    rclcpp::QoS daly_stat_qos(rclcpp::KeepAll{});
    stat_publisher_ = rclcpp::Node::create_publisher<BMSStatus>("bms_status", daly_stat_qos);

    // Timer to trigger command request to BMS
    request_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(req_period_),
        std::bind(&DalyCanNode::request_daly_data, this));
}

// Clean up can interface resources on shutdown.
void DalyCanNode::deinit()
{
    can_intf_.deinit();
}

// Initialize the CAN interface with the given epoll event loop.
bool DalyCanNode::init(EpollEventLoop *event_loop)
{
    // Initialize SocketCAN through the wrapper. Bind the receive callback.
    if (!can_intf_.init(interface_, event_loop, std::bind(&DalyCanNode::recv_callback, this, _1)))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize socket can interface: %s", interface_.c_str());
        return false;
    }

    // RCLCPP_INFO(this->get_logger(), "local_node_id: %d", local_node_id_);
    // RCLCPP_INFO(this->get_logger(), "daly_node_id: %d", daly_node_id_);
    // RCLCPP_INFO(this->get_logger(), "interface: %s", interface_.c_str());
    return true;
}

// Receive callback for every CAN frame delivered by the event loop.
void DalyCanNode::recv_callback(const can_frame &frame)
{
    // Only process extended CAN frames. Daly uses extended CAN identifiers.
    if (!(frame.can_id & CAN_EFF_FLAG))
        return; // ensure it's extended frame

    // Check CAN Node Id for BMS, otherwise ignore
    uint16_t extracted_node_id = static_cast<uint16_t>(frame.can_id & 0xFFFF);
    if (extracted_node_id != daly_node_id_)
        return;

    RCLCPP_INFO(this->get_logger(), "Got Data");

    // Decode CAN ID to command
    switch ((frame.can_id >> 16) & 0xFF)
    {
        case static_cast<uint8_t>(DalyCmdId::kTotalVoltageCurrentSOC): {
            // Check CAN frame length 
            if (!verify_length("kTotalVoltageCurrentSOC", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(bat_info_mutex_);
            
            // Save and scale battery info
            bat_info_.total_voltage    = read_be<uint16_t>(frame.data + 0) * 0.1f;
            bat_info_.measured_voltage = read_be<uint16_t>(frame.data + 2) * 0.1f;
            bat_info_.current          = (read_be<uint16_t>(frame.data + 4) - 30000) * 0.1f;
            bat_info_.soc              = read_be<uint16_t>(frame.data + 6) * 0.1f;
        
            // Update publish flag
            info_pub_flag_ |= 0b0001;
            break;
        }
        
        case static_cast<uint8_t>(DalyCmdId::kChargeDischargeState): {
            // Check CAN frame length 
            if (!verify_length("kChargeDischargeState", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(daly_stat_mutex_);
            std::lock_guard<std::mutex> guard2(bat_info_mutex_);
        
            // Save BMS charge state and battery capacity
            daly_stat_.charge_state = frame.data[0];
            bat_info_.capacity = (uint32_t(frame.data[4]) << 24) |
                                 (uint32_t(frame.data[5]) << 16) |
                                 (uint32_t(frame.data[6]) << 8)  |
                                 (uint32_t(frame.data[7]));
            
            // Update publish flags
            info_pub_flag_   |= 0b0010; // capacity for BatteryInfo
            status_pub_flag_ |= 0b1000; // charge_state for BMSStatus
            break;
        }

        case static_cast<uint8_t>(DalyCmdId::kMaxMinTemperature): {
            // Check CAN frame length 
            if (!verify_length("kMaxMinTemperature", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(daly_stat_mutex_);
        
            // Save temperatures
            daly_stat_.temps[0] = frame.data[0] - 40;
            daly_stat_.temps[1] = frame.data[2] - 40;
        
            // Update publish flag
            status_pub_flag_ |= 0b0001;
            break;
        }
        
        case static_cast<uint8_t>(DalyCmdId::kCellVoltageFrame1): {
            // Check CAN frame length 
            if (!verify_length("kCellVoltageFrame1", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(daly_stat_mutex_);
        
            uint8_t frame_index = frame.data[0]; // 0x01, 0x02, 0x03, etc.
        
            // Save all cell voltages that are distributed over multiple frames
            for (int i = 0; i < 3; ++i) {
                int idx = (frame_index - 1) * 3 + i;
                if (idx >= 7) break;  // avoid overflow
                daly_stat_.cell_voltages[idx] = (uint32_t(frame.data[1 + 2 * i]) << 8) |
                                                 uint32_t(frame.data[2 + 2 * i]);
            }
        
            // Update publish flag
            status_pub_flag_ |= 0b0010;
            break;
        }
        
        case static_cast<uint8_t>(DalyCmdId::kBatteryFaultStatus): {
            // Check CAN frame length 
            if (!verify_length("kBatteryFaultStatus", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(daly_stat_mutex_);
        
            // Save each fault bit
            for (int i = 0; i < 8; ++i) {
                daly_stat_.fault_bits[i] = frame.data[i];
            }
        
            // Update publish flag
            status_pub_flag_ |= 0b0100;
            break;
        }
        

    }

    // Flag Loggin to check data status
    // RCLCPP_INFO(rclcpp::Node::get_logger(), "info_pub_flag_: %s", std::bitset<4>(info_pub_flag_).to_string().c_str());
    // RCLCPP_INFO(rclcpp::Node::get_logger(), "status_pub_flag_: %s", std::bitset<4>(status_pub_flag_).to_string().c_str());

    // Check if all data has been updated based on publish flag
    if ((info_pub_flag_ & 0b0011) == 0b0011)
    {
        RCLCPP_INFO(rclcpp::Node::get_logger(), "Publishing bat info");
        bat_info_.header.stamp = this->now();
        info_publisher_->publish(bat_info_);
        info_pub_flag_ = 0;
    }

    // Check if all data has been updated based on publish flag
    if ((status_pub_flag_ & 0b1111) == 0b1111)
    {
        RCLCPP_INFO(rclcpp::Node::get_logger(), "Publishing bms status");
        daly_stat_.header.stamp = this->now();
        stat_publisher_->publish(daly_stat_);
        status_pub_flag_ = 0;
    }
}

// Function to send request frames for each Daly command.
void DalyCanNode::request_daly_data()
{
    // Desired Daly commands to send
    static constexpr std::array<DalyCmdId, 6> requests = {
        DalyCmdId::kTotalVoltageCurrentSOC,
        DalyCmdId::kMaxMinTemperature,
        DalyCmdId::kChargeDischargeState,
        DalyCmdId::kStatusInfo1,
        DalyCmdId::kCellVoltageFrame1,
        DalyCmdId::kBatteryFaultStatus,
    };

    for (const auto &cmd : requests)
    {
        // Create and populate canbus frame with node id and command
        can_frame frame{};
        frame.can_id = build_extended_id(static_cast<uint8_t>(cmd), local_node_id_);
        frame.can_id |= CAN_EFF_FLAG;
        frame.can_dlc = 8;
        std::fill(std::begin(frame.data), std::end(frame.data), 0);

        // Send the CAN rfame
        if (!can_intf_.send_can_frame(frame))
        {
            RCLCPP_WARN(this->get_logger(), "Failed to send request 0x%02X", static_cast<uint8_t>(cmd));
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(), "Sent request 0x%02X", static_cast<uint8_t>(cmd));
        }

        // Slight delay to allow device processing time
        std::this_thread::sleep_for(std::chrono::milliseconds(50)); 
    }
}

// Compose the 29 bit extended CAN identifier used by Daly.
// Format: 0x18 in the top byte, command in the next byte, node ID in the low two bytes.
uint32_t DalyCanNode::build_extended_id(uint8_t data_id, uint16_t node_id)
{
    return (0x18 << 24) | (data_id << 16) | node_id;
}

// Basic DLC guard with debug logging.
// Returns true when the actual DLC matches the expected value.
inline bool DalyCanNode::verify_length(const std::string &name, uint8_t expected, uint8_t length)
{
    bool valid = expected == length;
    RCLCPP_DEBUG(this->get_logger(), "received %s", name.c_str());
    if (!valid)
        RCLCPP_WARN(this->get_logger(), "Incorrect %s frame length: %d != %d", name.c_str(), length, expected);
    return valid;
}
