# 第三方依赖

本目录使用的第三方库归各自作者所有，遵循各自许可证。

| 目录 | 上游仓库 | 本项目固定版本 | 管理方式 |
| --- | --- | --- | --- |
| `SEGGER` | [SEGGERMicro/RTT](https://github.com/SEGGERMicro/RTT) | [`4d8feab`](https://github.com/SEGGERMicro/RTT/commit/4d8feab3150f86f37a9d323ddc88d6cdf5673072) | Git 子模块 |
| `eigen` | [libeigen/eigen](https://gitlab.com/libeigen/eigen) | [`ae47f25`](https://gitlab.com/libeigen/eigen/-/commit/ae47f25fb2330ccb0b94644e97834f5977803888) | Git 子模块 |
| `tinyusb` | [hathach/tinyusb](https://github.com/hathach/tinyusb) | [`011591a`](https://github.com/hathach/tinyusb/commit/011591a25891ee50d8565399fab5331d3041b0f7) | Git 子模块 |
| `Andes_DSP` | [HPMicro SDK / nds_dsp](https://github.com/hpmicro/hpm_sdk/tree/ccc5ec467a047f3ea85cd2fa4565e91db7135a38/middleware/hpm_math/nds_dsp) | 见下方来源说明 | 随项目保存的文件副本 |

## 下载依赖

首次克隆时同时下载子模块：

```sh
git clone --recurse-submodules https://github.com/liqunmaster/Dust_HeavyRobot.git
```

已有本项目时，在仓库根目录执行：

```sh
git submodule update --init --recursive
```

GitHub 中的 SEGGER 和 TinyUSB 子模块目录可跳转到对应上游的固定提交。
Eigen 的官方仓库位于 GitLab；如果 GitHub 未提供该子模块目录的跳转链接，请使用上表中的仓库或固定版本链接。

## Andes DSP 来源

`Andes_DSP` 的 16 个头文件及 `lib/gcc/libdspf.a` 均与 HPMicro SDK 提交
[`ccc5ec4`](https://github.com/hpmicro/hpm_sdk/commit/ccc5ec467a047f3ea85cd2fa4565e91db7135a38)
中 `middleware/hpm_math/nds_dsp` 下的对应文件具有相同 Git blob 哈希。
这是已核对内容一致的上游版本，不代表已确认最初复制时使用的 SDK 版本。

本项目将上游根目录的头文件放入 `include/`，将上游 `gcc/libdspf.a` 放入
`lib/gcc/`。为保留现有构建路径，这部分继续保存为文件副本，通过上表链接访问上游。

## 许可证与权限

引用公开上游仓库、在自己的仓库配置子模块，不需要上游仓库的写入权限。
子模块不会赋予向上游推送修改的权限，也不会替代代码使用和分发的许可证要求。

- SEGGER RTT：参见上游及子模块内的 `LICENSE.md`。
- Eigen：主要采用 MPL-2.0；部分文件使用其他许可证，参见 `COPYING.README` 和各 `COPYING.*` 文件。
- TinyUSB：主要采用 MIT，参见子模块内的 `LICENSE`，各依赖仍以其自身许可证为准。
- Andes DSP：多数 DSP 头文件标注 Apache-2.0，`riscv_simd_convert.h` 标注 BSD-3-Clause；
  内建函数头文件及预编译库的分发条件还需结合上游 SDK 的许可文件确认，不能仅按头文件许可推定。

请保留各依赖的原始版权和许可证声明。
