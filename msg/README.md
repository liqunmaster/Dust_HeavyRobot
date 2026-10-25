# 消息定义与使用

根 .msg 是跨模块消息字段的唯一来源。构建扫描此目录，调用 Tools/msg/generate_messages.py，在各自 build/build_*/generated 生成类型、topic 元数据和静态存储。生成文件不提交、不手改。

ChassisSetpoint.msg 示例：

```text
float32 vx_m_s
float32 vy_m_s
float32 yaw_rad_s
uint64 timestamp
```

使用：

```cpp
#include "Publication.hpp"
#include "Subscription.hpp"
#include <uORB/topics/chassis_setpoint.h>

uORB::Publication<chassis_setpoint_s> _pub{ORB_ID(chassis_setpoint)};
uORB::Subscription _sub{ORB_ID(chassis_setpoint)};
chassis_setpoint_s target{};
if (_sub.update(&target)) {
        // 有未读取的更新，消费者仍需判断时效。
}
```

支持数值/bool、定长数组、嵌套消息、标量默认值、大写常量、# TOPICS。文件 PascalCase，字段 snake_case，生成类型 snake_case_s；每个定义要求 uint64 timestamp。暂不支持字符串、动态数组、数组默认值或完整 ROS/EmPy 语法。重复名称、未知/循环依赖和越界默认值在生成前拒绝。

## Topic 契约

| Topic | 发布者 | 主要消费者 |
| --- | --- | --- |
| manual_control_setpoint | manual_control | 输入诊断/后续编排 |
| chassis_setpoint | manual_control、显式速度设置接口 | chassis_control |
| yaw_setpoint | manual_control | yaw_control |
| lift_setpoint（4 实例） | manual_control、显式升降设置接口 | lift_control |
| booster_setpoint | manual_control | booster_control |
| sensor_imu | attitude_estimator 采样适配 | 诊断/后续消费者 |
| vehicle_attitude | attitude_estimator | yaw_control、H mcu_bridge |
| motor_feedback（32 个有界槽） | 板级电机服务适配对象 | 诊断/后续编排；闭环使用注入服务的同源物理快照 |
| chassis/lift/booster_status | 对应机构 | 诊断/后续编排 |

H 反馈实例 0 对应 yaw；L 实例 0..3 为底盘四轮，4..7 为升降，8 为 booster。实例由板级显式绑定，重启不自动重排。多发布者不代表自动仲裁控制权；公共设置接口与遥控继续遵循原有最新发布值语义。

timestamp 是本板启动后的微秒，timestamp_sample 是源采样时刻。毫秒快照通过 source_time_us 按源年龄换算，处理 uint32 回绕；重复读取不续期。两板启动时钟不共享，线上协议保留原布局与本地接收时效，不新增未经同步的远端 uptime 相减。

## 后端与上游兼容范围

Publication.hpp、Subscription.hpp、Subscription.cpp 来源于 PX4 v1.16.0，保留许可证，原文件 SHA-256 及提交记录在 platforms/common/uORB/upstream.json。适配移除了 PX4 平台 include，增加发布类型大小检查、禁止发布句柄复制并修正 unadvertise 后句柄复用。

采用“上游 C++ 封装 + 自有静态 Zephyr 后端”，没有移植完整 PX4 DeviceNode/Manager。上游字符设备、poll、callback、自动多实例分配、远端桥接、日志器和 ROS 2 集成未实现，不能直接链接上游二进制插件。生成器也是本工程独立实现。

- 已接入 Publication.publish、Subscription.updated/update/copy、按实例订阅和 ChangeInstance。
- 默认深度 1；uint8 ORB_QUEUE_LENGTH = 4 可声明 1..32 的二次幂队列。
- 每个订阅者独立维护进度。update 读未消费数据，copy 读最新数据并推进本订阅进度。首次订阅可读已发布的最新消息。
- 溢出覆盖最旧数据，读者跳到仍保留的最早消息。Node.overwritten 是各读者发现的跳过数量之和，不是唯一丢帧数。
- 项目扩展 ORB_MAX_INSTANCES 定义静态容量，PublicationInstance 明确指定实例；不等同于上游 PublicationMulti 自动分配。
- 节点与数据由构建静态生成，短自旋锁保护，无消息堆分配。事件发布生命周期沿用上游封装惯例。
- 每个 Subscription 由自己的任务使用，跨任务共享同一订阅对象需要调用者同步。

生成结构体的填充与字段表示只用于板内通信。CAN/USB/遥控继续显式编解码，不得把整个生成结构体 memcpy 成线上帧。
