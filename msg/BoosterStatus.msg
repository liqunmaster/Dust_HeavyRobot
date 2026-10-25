# 板内消息数据；禁止将结构体内存直接序列化为 CAN 帧。
uint8 state
bool feedback_fresh
uint8 motor_status
uint32 feedback_age_ms
float32 angle_rad
float32 target_angle_rad
float32 speed_rad_s
float32 measured_torque_nm
float32 command_torque_nm
uint8 jam_retries
int32 last_error
uint64 timestamp
