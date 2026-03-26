#include "ares_protocol.hpp"
#include <iostream>
#include <vector>
#include <chrono> // for sleep and timing
#include <thread> // for sleep and threads
#include <limits> // Required for numeric_limits
#include <ios>    // Required for streamsize
#include <atomic> // Required for atomic counters
#include <iomanip> // For std::fixed and std::setprecision

// {{ edit_1: 添加原子计数器 }}
std::atomic<uint64_t> received_sync_count{0};
std::atomic<uint64_t> sent_packets_count{0};
std::atomic<bool> keep_running_stats{true}; // 控制统计线程

// --- 回调函数示例 ---

// 处理同步帧的回调 (假设这是上位机接收)
void sync_handler(uint16_t data_id, const uint8_t* data, size_t len) {
    received_sync_count++;
    // std::cout << "[Callback] Sync received: DataID=" << data_id << ", Len=" << len << ", Data=";
    // for(size_t i = 0; i < len && i < 8; ++i) { // 最多打印前8字节
    //      printf("%02X ", data[i]);
    // }
    // if (len > 8) std::cout << "...";
    // std::cout << std::endl;
}

// 处理执行帧的回调 (假设这是下位机接收并处理)
uint32_t exec_handler(uint16_t func_id, uint32_t arg1, uint32_t arg2, uint32_t arg3, uint8_t request_id) {
    // std::cout << "[Callback] Exec received: FuncID=" << func_id
    //           << ", Arg1=" << arg1 << ", Arg2=" << arg2 << ", Arg3=" << arg3
    //           << ", ReqID=" << (int)request_id << std::endl;
    // 模拟处理并返回结果
    return arg1 + arg2 + arg3;
}

// {{ edit_3: 添加统计线程函数 }}
// {{ edit_1: 修改统计线程函数 }}
void statistics_thread() {
    uint64_t last_received_count = 0;
    uint64_t last_sent_count = 0;
    auto last_time = std::chrono::steady_clock::now();
    std::string clear_line(80, ' '); // 用于清除行的空格字符串

    while (keep_running_stats) {
        std::this_thread::sleep_for(std::chrono::seconds(1)); // 每秒统计一次

        if (!keep_running_stats) break; // 退出前检查

        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(now - last_time);
        double interval = duration.count();
        last_time = now;

        uint64_t current_received = received_sync_count.load();
        uint64_t current_sent = sent_packets_count.load();

        uint64_t received_this_interval = current_received - last_received_count;
        uint64_t sent_this_interval = current_sent - last_sent_count;

        double receive_freq = (interval > 0) ? (received_this_interval / interval) : 0.0;
        double send_freq = (interval > 0) ? (sent_this_interval / interval) : 0.0;

        // 使用 stringstream 构建输出字符串
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2); // 设置输出格式
        ss << "[Stats] RX Sync: " << receive_freq << " Hz (" << received_this_interval << "/s), "
           << "TX Total: " << send_freq << " Hz (" << sent_this_interval << "/s)";
        std::string stats_str = ss.str();


        // 输出：回车符 + 统计信息 + 清除空格 + flush
        std::cout << "\r" << stats_str << clear_line.substr(0, std::max(0, (int)(clear_line.length() - stats_str.length()))) << std::flush;


        last_received_count = current_received;
        last_sent_count = current_sent;
    }
    // 线程结束前，用空格清除最后一行统计信息并换行
    std::cout << "\r" << clear_line << "\r" << std::flush;
    std::cout << "Statistics thread finished." << std::endl; // 换行打印结束信息
}

// {{ edit_4: 添加发送线程函数 (示例) }}
void sending_thread(ares::Protocol& proto) {
    uint8_t sync_data[16]; // 示例数据
    uint16_t data_id_counter = 0;
    uint8_t request_id_counter = 0;
    int send_counter = 0;

    while (keep_running_stats && proto.is_connected()) {
        // 每发送 10 个 sync 包，发送一个 exec 包 (示例)
        if (send_counter % 10 == 0) {
            if (proto.send_exec(0x01, send_counter, send_counter + 1, send_counter + 2, request_id_counter++)) {
                sent_packets_count++;
            } else if (proto.is_connected()) { // 只有在连接时发送失败才打印错误
                 std::cerr << "Failed to send Exec command." << std::endl;
            }
        } else {
             // 填充示例数据
             for(size_t i=0; i<sizeof(sync_data); ++i) sync_data[i] = (uint8_t)(send_counter + i);
             if (proto.send_sync(data_id_counter++, sync_data, sizeof(sync_data))) {
                 sent_packets_count++;
             } else if (proto.is_connected()) {
                 std::cerr << "Failed to send Sync data." << std::endl;
             }
        }
        send_counter++;

        // 控制发送速率，例如大约 100Hz (10ms 间隔)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << "Sending thread finished." << std::endl;
}


int main() {
    ares::Protocol proto;

    // 注册回调
    proto.register_sync_callback(sync_handler);
    // proto.register_exec_callback(exec_handler); // 如果需要处理执行帧，取消注释

    // 连接设备
    std::cout << "Connecting to device..." << std::endl;
    if (!proto.connect()) {
        std::cerr << "Failed to connect." << std::endl;
        return 1;
    }
    std::cout << "Device connected: " << proto.is_connected() << std::endl;

    // {{ edit_5: 启动统计和发送线程 }}
    keep_running_stats = true;
    std::thread stats_th(statistics_thread);
    std::thread sender_th(sending_thread, std::ref(proto)); // 启动发送线程

    std::cout << "Running... Press Enter to disconnect and exit." << std::endl; // 初始提示信息

    // 持续运行，等待用户输入退出
    // std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // 这行可能不需要，取决于之前的输出
    std::cin.get();

    // {{ edit_6: 停止线程并断开连接 }}
    std::cout << "\nStopping threads..." << std::endl; // 在停止信息前加换行，避免覆盖最后一行统计
    keep_running_stats = false; // 通知线程停止

    if (sender_th.joinable()) {
        sender_th.join(); // 等待发送线程结束
    }
    if (stats_th.joinable()) {
        stats_th.join(); // 等待统计线程结束
    }

    // 断开连接
    std::cout << "Disconnecting..." << std::endl;
    proto.disconnect();

    std::cout << "Program finished." << std::endl;
    return 0;
}