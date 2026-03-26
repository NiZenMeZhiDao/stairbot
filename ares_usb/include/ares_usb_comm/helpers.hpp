#pragma once
/**
 * @file helpers.hpp
 * @brief 协议辅助函数与常量定义。
 */
#include <cstdint>
#include <cstring>
#include <array>
#include <arpa/inet.h>

namespace ares_usb_comm {

// -------------------- DataID 常量 --------------------
constexpr uint16_t DATAID_MOTORCMD_ANGLE   = 0x0101; ///< 上位机 -> 下位机：电机角度指令
constexpr uint16_t DATAID_MOTORCMD_TORQUE  = 0x0102; ///< 上位机 -> 下位机：前馈力矩指令
// constexpr uint16_t DATAID_MOTORCMD_SPEED   = 0x0102; ///< 已废弃：电机速度指令
constexpr uint16_t DATAID_MOTORSTATE_ANGLE = 0x0201; ///< 下位机 -> 上位机：角度反馈
constexpr uint16_t DATAID_MOTORSTATE_SPEED = 0x0202; ///< 下位机 -> 上位机：速度反馈
constexpr uint16_t DATAID_MOTORSTATE_TORQUE = 0x0203; ///< 下位机 -> 上位机：实际力矩反馈
constexpr uint16_t DATAID_IMU6             = 0x0301; ///< 下位机 -> 上位机：IMU 六轴

// -------------------- 浮点与网络序转换 --------------------
/**
 * @brief 将 float 转换为大端序 32 位整数。
 * @param value 主机端浮点值
 * @return uint32_t 大端序表示
 */
inline uint32_t float_to_be32(float value)
{
    uint32_t tmp;
    std::memcpy(&tmp, &value, sizeof(float));
    tmp = htonl(tmp);
    return tmp;
}

/**
 * @brief 将大端序 32 位整数转换回 float。
 * @param net 大端序整数
 * @return float 本机端浮点值
 */
inline float be32_to_float(uint32_t net)
{
    net = ntohl(net);
    float value;
    std::memcpy(&value, &net, sizeof(float));
    return value;
}

/**
 * @brief 将浮点数组打包为大端序字节流。
 * @tparam N 元素数量
 * @param src 源 float 数组
 * @param dst 目标字节缓存，需保证 `N*4` 字节
 */
template <size_t N>
inline void pack_float_array_be(const std::array<float, N>& src, uint8_t* dst)
{
    for (size_t i = 0; i < N; ++i)
    {
        uint32_t net = float_to_be32(src[i]);
        std::memcpy(dst + i * 4, &net, 4);
    }
}

/**
 * @brief 从大端序字节流解析浮点数组。
 * @tparam N 元素数量
 * @param src 源字节流，长度至少 `N*4`
 * @param dst 目标 float 数组
 */
template <size_t N>
inline void unpack_float_array_be(const uint8_t* src, std::array<float, N>& dst)
{
    for (size_t i = 0; i < N; ++i)
    {
        uint32_t net;
        std::memcpy(&net, src + i * 4, 4);
        dst[i] = be32_to_float(net);
    }
}

} // namespace ares_usb_comm 