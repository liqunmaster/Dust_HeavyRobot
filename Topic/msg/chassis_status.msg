# @struct ChassisStatus

uint32 timestamp_ms
bool command_fresh

bool[4] feedback_fresh
float32[4] wheel_speed_rad_s

float32[4] command_current_a
