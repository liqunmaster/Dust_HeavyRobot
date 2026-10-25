# 板内消息数据；禁止将结构体内存直接序列化为 CAN 帧。
float32 forward
float32 lateral
float32 rotation
uint8 protocol
uint8 switch_left
uint8 switch_right
bool valid
uint64 timestamp # 本地源时间，单位为微秒
