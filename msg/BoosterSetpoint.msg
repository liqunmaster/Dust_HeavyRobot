# 板内消息数据；禁止将结构体内存直接序列化为 CAN 帧。
uint8 switch_position # 1/3 表示释放，2 表示触发；保留现有边沿触发和重新使能语义
bool valid
uint64 timestamp
