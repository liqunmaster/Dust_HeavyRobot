# 板内消息数据；禁止将结构体内存直接序列化为 CAN 帧。
uint64 timestamp
bool command_fresh
bool[4] feedback_fresh
float32[4] wheel_speed_rad_s
float32[4] command_current_a
