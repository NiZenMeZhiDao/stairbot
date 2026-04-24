#!/usr/bin/env python3
from __future__ import annotations

import argparse
import struct
import sys
import time
from collections import Counter

import serial


UNLOCK_CMD = bytes([0xFF, 0xAA, 0x69, 0x88, 0xB5])


def checksum_ok(frame: bytes) -> bool:
    return len(frame) == 11 and (sum(frame[:10]) & 0xFF) == frame[10]


def format_u16(value: int) -> str:
    return f"0x{value:04X} ({value})"


def format_i16(value: int) -> str:
    signed = value if value < 0x8000 else value - 0x10000
    return f"0x{value:04X} ({signed})"


def read_frames(ser: serial.Serial, duration: float) -> list[bytes]:
    deadline = time.time() + duration
    buf = bytearray()
    frames: list[bytes] = []

    while time.time() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf.extend(chunk)

        while len(buf) >= 11:
            if buf[0] != 0x55:
                del buf[0]
                continue

            frame = bytes(buf[:11])
            if checksum_ok(frame):
                frames.append(frame)
                del buf[:11]
            else:
                del buf[0]

    return frames


def read_register_block(ser: serial.Serial, start_addr: int, wait_s: float = 0.4) -> bytes | None:
    ser.reset_input_buffer()
    ser.write(UNLOCK_CMD)
    time.sleep(0.03)
    ser.write(bytes([0xFF, 0xAA, 0x27, start_addr & 0xFF, 0x00]))

    deadline = time.time() + wait_s
    buf = bytearray()

    while time.time() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf.extend(chunk)

        while len(buf) >= 11:
            if buf[0] != 0x55:
                del buf[0]
                continue

            frame = bytes(buf[:11])
            if checksum_ok(frame):
                del buf[:11]
                if frame[1] == 0x5F:
                    return frame
            else:
                del buf[0]

    return None


def decode_u16_regs(frame: bytes) -> list[int]:
    return list(struct.unpack("<4H", frame[2:10]))


def decode_stream_summary(frames: list[bytes]) -> tuple[Counter, bool]:
    types = Counter(frame[1] for frame in frames)
    angle_frames = [frame for frame in frames if frame[1] == 0x53]
    high_precision_like = False

    if angle_frames:
        axis_markers = [frame[2] for frame in angle_frames]
        ratio = sum(v in (0x01, 0x02, 0x03) for v in axis_markers) / len(axis_markers)
        if ratio > 0.7:
            high_precision_like = True

    return types, high_precision_like


def decode_rsw(value: int) -> dict[str, bool]:
    names = [
        "time",
        "acc",
        "gyro",
        "angle",
        "mag",
        "port",
        "press",
        "gps",
        "velocity",
        "quaternion",
        "gsa",
    ]
    return {name: bool(value & (1 << idx)) for idx, name in enumerate(names)}


def print_frame_types(counter: Counter) -> None:
    if not counter:
        print("  未抓到任何 0x55 数据帧")
        return

    print("  流数据帧统计:")
    for frame_type, count in sorted(counter.items()):
        print(f"    0x{frame_type:02X}: {count}")


def analyze(findings: list[str], regs: dict[int, list[int]], stream_types: Counter, high_precision_like: bool) -> None:
    rsw_block = regs.get(0x02)
    orient_block = regs.get(0x23)
    mag_offset_block = regs.get(0x0B)
    mag_range_block = regs.get(0x1C)
    accfilt_block = regs.get(0x2A)

    if rsw_block:
        rsw = rsw_block[0]
        rsw_flags = decode_rsw(rsw)
        if not rsw_flags["angle"]:
            findings.append("RSW 未开启 angle 输出，正常流里可能拿不到 0x53 角度帧。")
        if not rsw_flags["mag"]:
            findings.append("RSW 未开启 mag 输出，正常流里可能拿不到 0x54 磁场帧。")
        if rsw_flags["quaternion"]:
            findings.append("设备开启了 quaternion 输出，但当前驱动没有解析 0x59，只是把欧拉角自行转四元数。")

    if orient_block:
        orient, axis6, filtk, gpsbaud = orient_block
        if axis6 == 1:
            findings.append("AXIS6=1，当前是 6 轴算法，yaw 为相对航向，容易漂，不适合当绝对朝向。")
        elif axis6 != 0:
            findings.append(f"AXIS6 读到异常值 {axis6}。")

        if orient == 1:
            findings.append("ORIENT=1，当前是垂直安装模式；如果你的 IMU 实际是水平安装，姿态角会整体不准。")
        elif orient not in (0, 1):
            findings.append(f"ORIENT 读到异常值 {orient}。")

        if filtk not in (30, 0x001E):
            findings.append(f"FILTK={filtk}，偏离默认值 30，滤波配置可能影响动态姿态表现。")

        if gpsbaud not in (0, 1, 2, 3, 4, 5, 6, 7):
            findings.append(f"GPSBAUD 读到异常值 {gpsbaud}。")

    if mag_offset_block:
        hx, hy, hz, _ = [v if v < 0x8000 else v - 0x10000 for v in mag_offset_block]
        if max(abs(hx), abs(hy), abs(hz)) == 0:
            findings.append("HXOFFSET/HYOFFSET/HZOFFSET 全为 0，看起来没有做过磁偏校准。")
        elif max(abs(hx), abs(hy), abs(hz)) > 20000:
            findings.append("磁偏置绝对值非常大，磁校准结果可疑。")

    if mag_range_block:
        mx, my, mz, bandwidth = mag_range_block
        if 0 in (mx, my, mz):
            findings.append("MAGRANGX/Y/Z 有 0 值，磁校准量程异常。")
        if max(mx, my, mz) - min(mx, my, mz) > 2000:
            findings.append("MAGRANGX/Y/Z 差异过大，磁场量程校准可能不一致。")
        if bandwidth not in (0, 1, 2, 3, 4, 5, 6):
            findings.append(f"BANDWIDTH={bandwidth} 超出协议常见范围。")

    if accfilt_block:
        accfilt = accfilt_block[0]
        if accfilt != 500:
            findings.append(f"ACCFILT={accfilt}，偏离默认 500，震动环境下会影响姿态稳定性。")

    if 0x54 not in stream_types:
        findings.append("抓到的流数据里没有 0x54 磁场帧，驱动看到的磁场可能一直是旧值或 0。")
    if 0x53 not in stream_types:
        findings.append("抓到的流数据里没有 0x53 角度帧，驱动看到的姿态角可能一直是旧值或 0。")
    if high_precision_like:
        findings.append("流数据里的 0x53 很像高精度按轴输出格式；如果节点仍按 TTL_STD 运行，角度会解析错。")


def main() -> int:
    parser = argparse.ArgumentParser(description="Read WIT registers without changing device settings.")
    parser.add_argument("--port", default="/dev/imu_usb")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--sniff-seconds", type=float, default=0.6)
    args = parser.parse_args()

    query_map = {
        0x02: "RSW/RRATE/BAUD/AXOFFSET",
        0x0B: "HXOFFSET/HYOFFSET/HZOFFSET/D0MODE",
        0x1C: "MAGRANGX/MAGRANGY/MAGRANGZ/BANDWIDTH",
        0x23: "ORIENT/AXIS6/FILTK/GPSBAUD",
        0x2A: "ACCFILT/RESERVED/RESERVED/POWONSEND",
        0x2E: "VERSION/YYMM/DDHH/MMSS",
        0x34: "AX/AY/AZ/TEMP",
        0x37: "GX/GY/GZ/VOLTAGE",
        0x3A: "HX/HY/HZ/TEMP",
        0x3D: "ROLL/PITCH/YAW/VERSION2",
    }

    try:
        ser = serial.Serial(args.port, args.baudrate, timeout=0.02, write_timeout=0.2)
    except Exception as exc:
        print(f"无法打开串口 {args.port}: {exc}", file=sys.stderr)
        return 1

    with ser:
        print(f"串口已打开: {args.port} @ {args.baudrate}")
        time.sleep(0.2)

        ser.reset_input_buffer()
        stream_frames = read_frames(ser, args.sniff_seconds)
        stream_types, high_precision_like = decode_stream_summary(stream_frames)
        print_frame_types(stream_types)
        print(f"  角度帧形态判断: {'疑似高精度按轴输出' if high_precision_like else '更像标准三轴角度帧'}")
        print()

        regs: dict[int, list[int]] = {}
        for addr, label in query_map.items():
            frame = read_register_block(ser, addr)
            if frame is None:
                print(f"[0x{addr:02X}] {label}: 未收到返回")
                continue

            regs[addr] = decode_u16_regs(frame)
            values = regs[addr]
            print(f"[0x{addr:02X}] {label}")
            print(
                "  "
                + ", ".join(
                    format_u16(v) if addr in (0x02, 0x1C, 0x23, 0x2A, 0x2E) else format_i16(v)
                    for v in values
                )
            )

            if addr == 0x02:
                print(f"  RSW flags: {decode_rsw(values[0])}")

        print()
        findings: list[str] = []
        analyze(findings, regs, stream_types, high_precision_like)

        if findings:
            print("问题判断:")
            for idx, finding in enumerate(findings, start=1):
                print(f"{idx}. {finding}")
        else:
            print("问题判断:")
            print("1. 关键寄存器没有读出明显异常，下一步更像是坐标系/安装方向与车体定义不一致，或者驱动单位与 ROS 约定不一致。")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
