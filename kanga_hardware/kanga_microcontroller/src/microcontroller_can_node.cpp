#include "microcontroller_can_node.hpp"

#include <algorithm>
#include <bit>
#include <cstring>

MicrocontrollerCanNode::MicrocontrollerCanNode(const std::string &node_name)
    : rclcpp::Node(node_name) {
    this->declare_parameter<std::string>("interface", "can1");
    this->declare_parameter<int64_t>("node_id", 0);
    this->declare_parameter<int64_t>("node_id_mask", 0xFFFF);
    this->declare_parameter<int64_t>("node_id_shift", 0);
    this->declare_parameter<bool>("filter_by_node", true);
    this->declare_parameter<bool>("expect_extended_id", true);
    this->declare_parameter<bool>("force_extended_tx", true);
    this->declare_parameter<bool>("allow_tx_id_override", true);
    this->declare_parameter<int64_t>("tx_base_id", 0);
    this->declare_parameter<bool>("auto_header_stamp", true);
    this->declare_parameter<bool>("drop_remote_frames", true);

    rclcpp::QoS qos(rclcpp::KeepLast(64));
    rx_publisher_ = this->create_publisher<MicrocontrollerFrame>("microcontroller_rx", qos);
    tx_subscription_ = this->create_subscription<MicrocontrollerFrame>(
        "microcontroller_tx", qos, std::bind(&MicrocontrollerCanNode::tx_callback, this, _1));
}

bool MicrocontrollerCanNode::init(EpollEventLoop *event_loop) {
    interface_ = this->get_parameter("interface").as_string();
    node_id_ = static_cast<uint32_t>(this->get_parameter("node_id").as_int());
    node_id_mask_ = static_cast<uint32_t>(this->get_parameter("node_id_mask").as_int());
    node_id_shift_ = static_cast<uint8_t>(this->get_parameter("node_id_shift").as_int());
    filter_by_node_ = this->get_parameter("filter_by_node").as_bool();
    expect_extended_id_ = this->get_parameter("expect_extended_id").as_bool();
    force_extended_tx_ = this->get_parameter("force_extended_tx").as_bool();
    allow_tx_id_override_ = this->get_parameter("allow_tx_id_override").as_bool();
    tx_base_id_ = static_cast<uint32_t>(this->get_parameter("tx_base_id").as_int());
    auto_header_stamp_ = this->get_parameter("auto_header_stamp").as_bool();
    drop_remote_frames_ = this->get_parameter("drop_remote_frames").as_bool();

    if (!can_intf_.init(interface_, event_loop, std::bind(&MicrocontrollerCanNode::recv_callback, this, _1))) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize socket CAN interface: %s", interface_.c_str());
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Microcontroller CAN node ready on %s (node id %u)", interface_.c_str(), node_id_);
    return true;
}

void MicrocontrollerCanNode::deinit() {
    can_intf_.deinit();
}

void MicrocontrollerCanNode::recv_callback(const can_frame &frame) {
    const bool is_extended = (frame.can_id & CAN_EFF_FLAG) != 0;
    if (expect_extended_id_ && !is_extended) {
        RCLCPP_DEBUG(this->get_logger(), "Ignoring standard frame 0x%X on extended-only interface", frame.can_id);
        return;
    }

    if (drop_remote_frames_ && (frame.can_id & CAN_RTR_FLAG)) {
        RCLCPP_DEBUG(this->get_logger(), "Ignoring remote frame 0x%X", frame.can_id);
        return;
    }

    const uint32_t mask = is_extended ? CAN_EFF_MASK : CAN_SFF_MASK;
    const uint32_t raw_id = frame.can_id & mask;
    const uint32_t detected_node_id = extract_node_id(raw_id);

    if (filter_by_node_ && detected_node_id != node_id_) {
        return;
    }

    MicrocontrollerFrame msg;
    if (auto_header_stamp_) {
        msg.header.stamp = this->now();
    }
    msg.header.frame_id = interface_;
    msg.node_id = static_cast<uint16_t>(detected_node_id);
    msg.can_id = raw_id;
    msg.dlc = std::min<uint8_t>(frame.can_dlc, 8U);
    std::copy(frame.data, frame.data + msg.dlc, msg.data.begin());
    if (msg.dlc < 8U) {
        std::fill(msg.data.begin() + msg.dlc, msg.data.end(), 0U);
    }
    msg.is_extended = is_extended;
    msg.is_remote = (frame.can_id & CAN_RTR_FLAG) != 0;

    rx_publisher_->publish(msg);
}

void MicrocontrollerCanNode::tx_callback(const MicrocontrollerFrame::SharedPtr msg) {
    if (!msg) {
        return;
    }

    struct can_frame frame {
    };

    frame.can_dlc = std::min<uint8_t>(msg->dlc, 8U);
    std::copy(msg->data.begin(), msg->data.begin() + frame.can_dlc, frame.data);

    uint32_t requested_id = msg->can_id;
    bool requested_extended = msg->is_extended;
    const bool requested_remote = msg->is_remote;

    if (requested_id != 0 && !allow_tx_id_override_) {
        requested_id = 0;
    }

    const uint32_t node_value = sanitize_node_id(msg->node_id ? msg->node_id : node_id_);

    if (requested_id == 0) {
        if (node_id_mask_ != 0) {
            uint32_t masked_node = 0U;
            if (node_id_shift_ < 32U) {
                masked_node = (node_value << node_id_shift_) & node_id_mask_;
            }
            requested_id = (tx_base_id_ & ~node_id_mask_) | masked_node;
        } else if (tx_base_id_ != 0) {
            requested_id = tx_base_id_ | node_value;
        } else {
            requested_id = node_value;
        }
    }

    bool use_extended = force_extended_tx_ || requested_extended;
    if (!use_extended && expect_extended_id_) {
        use_extended = true;
    }

    if (use_extended) {
        if (requested_id > CAN_EFF_MASK) {
            RCLCPP_WARN(this->get_logger(), "TX frame id 0x%X exceeds extended range", requested_id);
            return;
        }
        frame.can_id = requested_id | CAN_EFF_FLAG;
    } else {
        if (requested_id > CAN_SFF_MASK) {
            RCLCPP_WARN(this->get_logger(), "TX frame id 0x%X exceeds standard range", requested_id);
            return;
        }
        frame.can_id = requested_id & CAN_SFF_MASK;
    }

    if (requested_remote) {
        frame.can_id |= CAN_RTR_FLAG;
    }

    {
        std::lock_guard<std::mutex> guard(tx_mutex_);
        if (!can_intf_.send_can_frame(frame)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to transmit CAN frame (id=0x%X)", requested_id);
        }
    }
}

uint32_t MicrocontrollerCanNode::extract_node_id(uint32_t can_id) const {
    if (node_id_mask_ == 0) {
        return can_id;
    }
    return (can_id & node_id_mask_) >> node_id_shift_;
}

uint32_t MicrocontrollerCanNode::sanitize_node_id(uint32_t node_id) const {
    if (node_id_mask_ == 0) {
        return node_id;
    }

    const uint32_t shifted_mask = (node_id_shift_ < 32U) ? (node_id_mask_ >> node_id_shift_) : 0U;
    if (shifted_mask == 0U) {
        return 0U;
    }

    const unsigned width = std::popcount(shifted_mask);
    if (width >= 32U) {
        return node_id;
    }

    const uint32_t limit_mask = (width >= 32U) ? 0xFFFFFFFFU : ((1U << width) - 1U);
    return node_id & limit_mask;
}
