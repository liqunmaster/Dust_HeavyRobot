#pragma once

#include <errno.h>
#include <zephyr/kernel.h>
#include "hpm_clock_drv.h"
#include "hpm_soc.h"

// 与 clock_add_to_group() 不同，此处不忙等资源唤醒。
// 硬件仍在切换状态时，由调用方稍后重试。
inline int hpm_enable_resource_nowait(sysctl_resource_t resource)
{
	const uint32_t offset = static_cast<uint32_t>(resource) - sysctl_resource_linkable_start;
	HPM_SYSCTL->GROUP0[offset / 32U].SET = 1U << (offset % 32U);
	return sysctl_resource_target_is_busy(HPM_SYSCTL, resource) ? -EAGAIN : 0;
}
