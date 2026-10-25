# 板内消息数据；禁止将结构体内存直接序列化为 CAN 帧。
uint8 ORB_MAX_INSTANCES = 32
uint64 timestamp
uint32 device_id # 应用层绑定标识，并非操作系统句柄
uint8 bus_index
uint8 protocol_kind
float32 angle_rad
float32 speed_rad_s
float32 current_a
float32 torque_nm
uint32 valid_count
uint8 motor_status
