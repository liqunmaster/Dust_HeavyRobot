#pragma once

#include <errno.h>
#include <stddef.h>
#include <stdint.h>

// 大疆 DT7 遥控器数据样本，含四路摇杆通道与左右拨杆开关
struct dt7_sample
{
    uint16_t channel[4];
    uint8_t switch_left;
    uint8_t switch_right;
};

// DT7 协议解析类，提供数据帧解码功能
class dt7
{
    public:
    static constexpr size_t frame_size = 18;

    static int decode_frame(const uint8_t *frame, size_t length, dt7_sample &sample);
};
