#include <errno.h>
#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/sys/util.h>
#include <hpm_common.h>
#include <hpm_clock_drv.h>
#include <hpm_iomux.h>
#include <hpm_pllctlv2_drv.h>
#include <hpm_soc.h>

    /**
     * @brief 初始化板级 CPU 时钟 配置 PLL0 频率与 CPU 分频
     *
     * @return 成功返回 0 配置失败返回 -EIO
    */
    static int hpm5361_board_clock_init(void)
{
    if (pllctlv2_init_pll_with_freq(HPM_PLLCTLV2, PLLCTLV2_PLL_PLL0, 960000000) != status_success) {
        return -EIO;
    }
    pllctlv2_set_postdiv(HPM_PLLCTLV2, PLLCTLV2_PLL_PLL0, pllctlv2_clk0, pllctlv2_div_1p0);
    return clock_set_source_divider(clock_cpu0, clk_src_pll0_clk0, 2) == status_success ? 0 : -EIO;
}

SYS_INIT(hpm5361_board_clock_init, PRE_KERNEL_2, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

#if defined(CONFIG_HPM_BOOT_HEADER)

__attribute__((section(".nor_cfg_option"), used, aligned(4)))
const uint32_t hpm5361_nor_cfg_option[4] = {
    DT_PROP(DT_CHOSEN(zephyr_flash), nor_cfg_opt_hdr),
    DT_PROP(DT_CHOSEN(zephyr_flash), nor_cfg_opt_opt0),
    DT_PROP(DT_CHOSEN(zephyr_flash), nor_cfg_opt_opt1),
    0,
};
#endif

    /**
     * @brief C 启动阶段复用 PA04~PA08 引脚为 JTAG 调试接口
     *
    */
    void c_startup(void)
{
    HPM_IOC->PAD[IOC_PAD_PA04].FUNC_CTL = IOC_PA04_FUNC_CTL_JTAG_TDO;
    HPM_IOC->PAD[IOC_PAD_PA05].FUNC_CTL = IOC_PA05_FUNC_CTL_JTAG_TDI;
    HPM_IOC->PAD[IOC_PAD_PA06].FUNC_CTL = IOC_PA06_FUNC_CTL_JTAG_TCK;
    HPM_IOC->PAD[IOC_PAD_PA07].FUNC_CTL = IOC_PA07_FUNC_CTL_JTAG_TMS;
    HPM_IOC->PAD[IOC_PAD_PA08].FUNC_CTL = IOC_PA08_FUNC_CTL_JTAG_TRST;
}

    /**
     * @brief 执行软件复位 触发 POR 复位并进入死循环等待
     *
     * @param type 复位类型
    */
    void sys_arch_reboot(int type)
{
    ARG_UNUSED(type);

    HPM_PPOR->RESET_ENABLE = (1L << 31);
    HPM_PPOR->SOFTWARE_RESET = 1000;
    while (1) {
    }
}