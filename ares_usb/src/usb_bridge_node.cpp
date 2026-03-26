#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "ares_protocol.hpp" // 来自 ares_comm/ARES_bulk_library
#include "ares_usb_comm/helpers.hpp"

#include <vector>
#include <string>
#include <unordered_map>
#include <algorithm>
#include <cctype>
#include <chrono>

using namespace std::chrono_literals;

namespace ares_usb_comm
{
/**
 * @class UsbPassthroughNode
 * @brief ROS2 节点：纯净版动态透传节点。
 * 发送端监听: t0x[ID]_xxx (例如: /t0x01A3_cmd)
 * 接收端发布: r0x[ID]     (例如: /r0x01A3)
 */
class UsbPassthroughNode : public rclcpp::Node
{
public:
    UsbPassthroughNode() : Node("usb_passthrough_node")
    {
        RCLCPP_INFO(this->get_logger(), "UsbPassthroughNode starting...");

        // 尝试连接 USB 设备 ----------------------------------------------
        if (!protocol_.connect()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect USB device, node will keep running but inactive.");
        } else {
            RCLCPP_INFO(this->get_logger(), "USB device connected successfully.");
        }

        // 注册协议回调 (处理所有来自下位机的数据) --------------------------
        protocol_.register_sync_callback(
            [this](uint16_t data_id, const uint8_t *data, size_t len) {
                handle_passthrough_rx(data_id, data, len);
            });

        // 动态话题发现定时器 (1Hz)，用于自动扫描并建立下发通道 --------------
        topic_discovery_timer_ = this->create_wall_timer(
            1s, std::bind(&UsbPassthroughNode::discover_passthrough_topics, this));
            
        RCLCPP_INFO(this->get_logger(), "Passthrough bridge initialized. Waiting for topics 't0x...' or USB data...");
    }

    ~UsbPassthroughNode() override
    {
        RCLCPP_INFO(this->get_logger(), "Disconnecting USB device...");
        protocol_.disconnect();
    }

private:
    // ---------------- 动态透传功能 (ROS2 -> 下位机) ------------------------
    /**
     * @brief 自动发现前缀为 t0x 的话题并创建订阅 (下发通道建立)
     */
    void discover_passthrough_topics()
    {
        auto topic_names_and_types = this->get_topic_names_and_types();
        
        for (const auto& [topic_name, types] : topic_names_and_types)
        {
            std::string clean_name = topic_name;
            if (!clean_name.empty() && clean_name[0] == '/') {
                clean_name = clean_name.substr(1);
            }

            // 检查是否以 "t0x" 开头
            if (clean_name.length() > 3 && clean_name.substr(0, 3) == "t0x")
            {
                // 提取 ID 字符串：查找 '_' 的位置，提取 t0x 之后的 16 进制字符
                size_t underscore_pos = clean_name.find('_', 3);
                std::string hex_str;
                if (underscore_pos != std::string::npos) {
                    hex_str = clean_name.substr(3, underscore_pos - 3);
                } else {
                    hex_str = clean_name.substr(3);
                }

                // 校验提取出的是否是合法的 16 进制字符串
                bool is_hex = !hex_str.empty() && std::all_of(hex_str.begin(), hex_str.end(), ::isxdigit);
                if (!is_hex) continue;

                // 检查话题类型是否为 Float32MultiArray
                bool is_valid_type = false;
                for (const auto& type : types) {
                    if (type == "std_msgs/msg/Float32MultiArray") {
                        is_valid_type = true;
                        break;
                    }
                }
                if (!is_valid_type) continue;

                // 如果尚未订阅，则动态创建订阅通道
                if (passthrough_subs_.find(clean_name) == passthrough_subs_.end())
                {
                    uint16_t data_id = static_cast<uint16_t>(std::stoul(hex_str, nullptr, 16));
                    
                    auto sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
                        topic_name, 10,
                        [this, data_id, topic_name](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                            this->passthrough_tx_callback(data_id, topic_name, msg);
                        });
                    
                    passthrough_subs_[clean_name] = sub;
                    
                    // 仅提示通道发现
                    RCLCPP_INFO(this->get_logger(), 
                        "🔗 [TX Channel Discovered] ROS Topic: '%s' ---> USB ID: 0x%04X", 
                        topic_name.c_str(), data_id);
                }
            }
        }
    }

    /**
     * @brief ROS 话题接收回调，将不定长 Float 数组打包发往 USB
     */
    void passthrough_tx_callback(uint16_t id, const std::string& topic_name, const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        size_t float_count = msg->data.size();
        if (float_count == 0) return;

        size_t byte_len = float_count * 4;

        // 【长度监控 Log】如果这个 ID 是第一次发送，或者长度发生了动态改变，则打印 Log
        if (tx_known_lengths_.find(id) == tx_known_lengths_.end() || tx_known_lengths_[id] != byte_len) {
            RCLCPP_INFO(this->get_logger(), 
                "✅ [TX Data Transmitting] ROS Topic: '%s' ---> USB ID: 0x%04X | Length: %zu Bytes (%zu Floats)", 
                topic_name.c_str(), id, byte_len, float_count);
            tx_known_lengths_[id] = byte_len; // 更新记录的长度
        }

        // 动态分配内存以适配不固定长度 (4 字节/float)
        std::vector<uint8_t> payload(byte_len);
        
        for (size_t i = 0; i < float_count; ++i)
        {
            const uint8_t *float_bytes = reinterpret_cast<const uint8_t *>(&msg->data[i]);
            payload[i * 4 + 0] = float_bytes[0];
            payload[i * 4 + 1] = float_bytes[1];
            payload[i * 4 + 2] = float_bytes[2];
            payload[i * 4 + 3] = float_bytes[3];
        }

        if (!protocol_.send_sync(id, payload.data(), payload.size()))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to TX passthrough data. ID: 0x%04X", id);
        }
    }

    // ---------------- 动态透传功能 (下位机 -> ROS2) ------------------------
    /**
     * @brief 处理来自下位机的协议帧，自动创建 Publisher 并发布 (上报通道建立)
     */
    void handle_passthrough_rx(uint16_t data_id, const uint8_t *data, size_t len)
    {
        if (len % 4 != 0) {
            RCLCPP_WARN(this->get_logger(), "RX length %zu not a multiple of 4 (ID: 0x%04X)", len, data_id);
            return;
        }

        size_t float_count = len / 4;
        
        // 生成对应的话题名称，格式为 r0x + 16进制大写ID (例如: r0x01A3)
        char topic_name_buf[32];
        snprintf(topic_name_buf, sizeof(topic_name_buf), "r0x%04X", data_id); 
        std::string clean_name = topic_name_buf; 

        // 如果对应的 Publisher 不存在，则自动创建通道
        if (passthrough_pubs_.find(data_id) == passthrough_pubs_.end())
        {
            passthrough_pubs_[data_id] = this->create_publisher<std_msgs::msg::Float32MultiArray>(clean_name, 10);
            
            // 首次建立通道，打印建立状态
            RCLCPP_INFO(this->get_logger(), 
                "🔗 [RX Channel Discovered] USB ID: 0x%04X ---> ROS Topic: '/%s'", 
                data_id, clean_name.c_str());
        }

        // 【长度监控 Log】如果这个 ID 是第一次接收，或者长度发生了动态改变，则打印 Log
        if (rx_known_lengths_.find(data_id) == rx_known_lengths_.end() || rx_known_lengths_[data_id] != len) {
            RCLCPP_INFO(this->get_logger(), 
                "✅ [RX Data Receiving] USB ID: 0x%04X ---> ROS Topic: '/%s' | Length: %zu Bytes (%zu Floats)", 
                data_id, clean_name.c_str(), len, float_count);
            rx_known_lengths_[data_id] = len; // 更新记录的长度
        }

        // 解包并发布
        auto msg = std_msgs::msg::Float32MultiArray();
        msg.data.resize(float_count);

        for (size_t i = 0; i < float_count; ++i)
        {
            uint32_t temp = (uint32_t)data[i * 4 + 3] << 24 |
                            (uint32_t)data[i * 4 + 2] << 16 |
                            (uint32_t)data[i * 4 + 1] << 8 |
                            (uint32_t)data[i * 4 + 0];
            msg.data[i] = *reinterpret_cast<float *>(&temp);
        }

        passthrough_pubs_[data_id]->publish(msg);
    }

    // ---------------- 成员变量 ------------------------------------------
    ares::Protocol protocol_{};

    // 动态透传定时器与记录容器
    rclcpp::TimerBase::SharedPtr topic_discovery_timer_; 
    std::unordered_map<std::string, rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr> passthrough_subs_;
    std::unordered_map<uint16_t, rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr> passthrough_pubs_;

    // 用于记录上次收发的长度，防止终端被 Log 刷屏
    std::unordered_map<uint16_t, size_t> tx_known_lengths_;
    std::unordered_map<uint16_t, size_t> rx_known_lengths_;
};

} // namespace ares_usb_comm

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    {
        auto node = std::make_shared<ares_usb_comm::UsbPassthroughNode>();
        rclcpp::spin(node);
    }
    rclcpp::shutdown();
    return 0;
}