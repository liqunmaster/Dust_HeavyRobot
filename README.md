# Dust HeavyRobot

Zephyr/HPM5361 的 H/L 双固件。目录和自有 C/C++ 风格参考 PX4 v1.16.0，机构控制保留在 src/modules，通过消息与物理设备服务协作。

| 目录 | 职责 |
| --- | --- |
| boards/Dust_HeavyRobot_H、Dust_HeavyRobot_L | 两端入口、固件配置与启动装配 |
| boards/common | 参数、设备绑定、共用 overlay/conf |
| boards/hpm5361 | Zephyr 板支持、DTS 和调试配置 |
| src/modules | 机构控制、输入映射、姿态任务、板间链路和健康监测 |
| src/device | 设备驱动与协议解析 |
| src/lib | 通用算法、容器、总线服务与物理电机接口 |
| platforms | Zephyr/HPM 适配、上游发布订阅封装与静态消息后端 |
| msg | 消息字段定义，构建生成 uORB/topics/*.h |
| ThirdParty、sim | 第三方依赖与独立仿真子模块 |
| Tools、test、docs | 工具、回归与当前文档 |

构建与烧录（PowerShell）：

```powershell
& '.\Tools\build&flash.cmd' H Build
& '.\Tools\build&flash.cmd' L Build
& '.\Tools\build&flash.cmd' H Flash
& '.\Tools\build&flash.cmd' L Flash Vllink
```

工具链继续使用本机 D:/MCU。产物为 build/build_h/zephyr/Dust_HeavyRobot.elf、build/build_l/zephyr/Dust_HeavyRobot.elf 及同名 .bin。VS Code 任务已更新，clangd 默认使用 H 编译数据库。

检查：

```powershell
python Tools/check_architecture.py
python Tools/run_tests.py --cxx D:/mingw64/bin/g++.exe
python Tools/format_code.py --check
```

格式工具安装见 [AStyle](Tools/astyle/README.md)。新增功能和依赖规则见[当前架构](docs/architecture.md)及[消息说明](msg/README.md)。迁移内容、验证和兼容范围见[实施记录](docs/PX4迁移实施记录_20261025.md)。

本轮 CPU 实板对比、编译优化级别对照、中文注释、短头文件引用和分目录构建验证见[CPU 对比与源码整理记录](docs/CPU对比与源码整理_20261025.md)。

本地 docs/history 中部分描述对应其他版本，当前结构以以上文档和源码为准。
