# 板内消息数据；禁止将结构体内存直接序列化为 CAN 帧。
uint64 timestamp
uint64 timestamp_sample
float32[3] acceleration_m_s2
float32[3] angular_velocity_rad_s
float32 temperature_c
