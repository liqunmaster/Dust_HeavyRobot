#pragma once

#include <errno.h>
#include <string.h>
#include "crc.hpp"
#include "remote_types.hpp"

// 大疆 VT02 遥控器数据样本，含鼠标、键盘与滚轮
struct vt02_sample
{
    int16_t mouse_x, mouse_y, mouse_z;
    bool mouse_left, mouse_right;
    remote_keyboard keyboard;
    int16_t pulley_wheel;
};

// VT02 协议解析类，提供帧长判定与数据帧解码
class vt02
{
    public:
    static constexpr size_t header_size = 5;

    static constexpr size_t frame_size = 21;

    static constexpr uint16_t command_id = 0x0304;

    static int frame_length_from_header(const uint8_t *frame, size_t length, size_t &frame_length);

    static int decode_frame(const uint8_t *frame, size_t length, vt02_sample &sample);
};
