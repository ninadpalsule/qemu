/*
 * ASPEED SoC 2700 family
 *
 * Copyright (c) 2023, IBM Corporation.
 *
 * This code is licensed under the GPL version 2 or later.  See
 * the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/misc/unimp.h"
#include "hw/arm/aspeed_soc.h"
#include "qemu/module.h"
#include "qemu/error-report.h"
#include "hw/i2c/aspeed_i2c.h"
#include "net/net.h"
#include "sysemu/sysemu.h"

#define ASPEED_SOC_IOMEM_SIZE       0x02c3e000
#define ASPEED_SOC_DPMCU_SIZE       0x00040000

static const hwaddr aspeed_soc_ast2700_memmap[] = {
    [ASPEED_DEV_SRAM]    = 0x10000000, /* ECC SRAM Memory Buffer */
    [ASPEED_DEV_DPMCU]   = 0x11000000, /* DisplayPort Micro Controller Unit Memory */
    [ASPEED_DEV_AHB]     = 0x12000000, /* AHB Bus Controller */
    [ASPEED_DEV_USB3A]   = 0x12010000, /* USB Hub3A Controller */
    [ASPEED_DEV_USB1A]   = 0x12011000, /* USB PortA Hub1 Controller */
    [ASPEED_DEV_USB3B]   = 0x12020000, /* USB Hub3B Controller */
    [ASPEED_DEV_USB1B]   = 0x12021000, /* USB PortB Hub1 Controller */
    [ASPEED_DEV_XHCIA]   = 0x12030000, /* USB xHCI Host Controller A */
    [ASPEED_DEV_XHCIB]   = 0x12050000, /* USB xHCI Host Controller B */
    [ASPEED_DEV_HUB0A]   = 0x12060000, /* USB PortA Hub0 Controller */
    [ASPEED_DEV_ECHIA]   = 0x12061000, /* USB EHCI Host Controller A */
    [ASPEED_DEV_HUB0B]   = 0x12062000, /* USB PortB Hub0 Controller */
    [ASPEED_DEV_ECHIB]   = 0x12063000, /* USB EHCI Host Controller B */
    [ASPEED_DEV_HACE]    = 0x12070000, /* Hash & Crypto Engine (HACE) */
    [ASPEED_DEV_RSA]     = 0x12080000, /* RSA Engine (RSA) */
    [ASPEED_DEV_EMMC]    = 0x12090000, /* eMMC Controller */
    [ASPEED_DEV_VIDEO]   = 0x120A0000, /* Video Engine */
    [ASPEED_DEV_RVAS0]   = 0x120B8000, /* RVAS VGA Snoop #0 Controller */
    [ASPEED_DEV_RVAS1]   = 0x120BC000, /* RVAS VGA Snoop #1 Controller */
    [ASPEED_DEV_2D]      = 0x120D0000, /* 2D Graphics Engine */
    [ASPEED_DEV_H2X0]    = 0x120E0000, /* AHB to PCIe Bus Bridge (H2X0) */
    [ASPEED_DEV_IRQ]     = 0x12100000, /* Software Interrupt Controller */
    [ASPEED_DEV_PCI2BMC] = 0x12110000, /* PCIe Host to BMC Controller */
    [ASPEED_DEV_PRIV0]   = 0x12140000, /* Privilege Control #0 */
    [ASPEED_DEV_DRAMC]   = 0x12C00000, /* SDRAM Memory Controller (DRAMC) */
    [ASPEED_DEV_DRAB]    = 0x12C01000, /* SDRAM Memory Arbiter Controller (DARB) */
    [ASPEED_DEV_SCU0]    = 0x12C02000, /* System Control Unit #0 (SCU0) */
    [ASPEED_DEV_XDMA]    = 0x12C04000, /* X-DMA Controller (XDMA) */
    [ASPEED_DEV_MCTP]    = 0x12C06000, /* MCTP Controller (MCTP) */
    [ASPEED_DEV_UFS]     = 0x12C08000, /* UFS Controller (UFS) */
    [ASPEED_DEV_GFX]     = 0x12C09000, /* SOC Display Controller (GFX) */
    [ASPEED_DEV_DP]      = 0x12C0A000, /* DisplayPort Interface */
    [ASPEED_DEV_EMMCBOOT] = 0x12C0B000, /* eMMC Boot Controller */
    [ASPEED_DEV_MSI]     = 0x12C0C000, /* MSI Controller (MSI) */
    [ASPEED_DEV_SRAMREG] = 0x12C0E000, /* ECC SRAM Memory Register */
    [ASPEED_DEV_RTC]     = 0x12C0F000, /* Real Time Clock (RTC)  */
    [ASPEED_DEV_TIMER0]  = 0x12C10000, /* Timer Controller #0 */
    [ASPEED_DEV_TIMER1]  = 0x12C10040, /* Timer Controller #1 */
    [ASPEED_DEV_TIMER2]  = 0x12C10080, /* Timer Controller #2 */
    [ASPEED_DEV_TIMER3]  = 0x12C100C0, /* Timer Controller #3 */
    [ASPEED_DEV_TIMER4]  = 0x12C10100, /* Timer Controller #4 */
    [ASPEED_DEV_TIMER5]  = 0x12C10140, /* Timer Controller #5 */
    [ASPEED_DEV_TIMER6]  = 0x12C10180, /* Timer Controller #6 */
    [ASPEED_DEV_TIMER7]  = 0x12C101C0, /* Timer Controller #7 */
    [ASPEED_DEV_GPIO0]   = 0x12C11000, /* GPIO Controller #0 */
    [ASPEED_DEV_UARTDMA] = 0x12C12000, /* UART DMA #0 */
    [ASPEED_DEV_UART4]   = 0x12C13000, /* UART4 - debug */
    [ASPEED_DEV_APB2PCIRC] = 0x12C16000, /* APB to PCIe RC Bridge */
    [ASPEED_DEV_BRIDGE0] = 0x12C17000, /* Internal Bridge Controller #0 */
    [ASPEED_DEV_IPC0]    = 0x12C1C000, /* InterProcessor Controller #0 */
    [ASPEED_DEV_VGA0]    = 0x12C1D000, /* VGA Link Controller #0 */
    [ASPEED_DEV_ECDSA]   = 0x12C1E000, /* ECDSA Controller */
    [ASPEED_DEV_SEC]     = 0x12C1F000, /* Secure Extension Controller (SEC) */
    [ASPEED_DEV_JTAG]    = 0x12C20000, /* JTAG Master Controller #0 */
    [ASPEED_DEV_PCI2MBUS]= 0x12C21000, /* PCIe to MBus Bridge */
    [ASPEED_DEV_FMC]     = 0x14000000, /* Firmware SPI Memory Controller */
    [ASPEED_DEV_SPI0]    = 0x14010000, /* SPI0 Memory Controller */
    [ASPEED_DEV_SPI1]    = 0x14020000, /* SPI1 Memory Controller */
    [ASPEED_DEV_SPI2]    = 0x14030000, /* SPI2 Memory Controller */
    [ASPEED_DEV_MDIO]    = 0x14040000, /* Ethernet MDC/MDIO Bus Controller */
    [ASPEED_DEV_MAC0]    = 0x14050000, /* 10/100/1G Ethernet MAC Controller (MAC0) */
    [ASPEED_DEV_MAC1]    = 0x14060000, /* 10/100/1G Ethernet MAC Controller (MAC1) */
    [ASPEED_DEV_MAC2]    = 0x14070000, /* 10/100/1G Ethernet MAC Controller (MAC2) */
    [ASPEED_DEV_SD]      = 0x14080000, /* SD/SDIO Host Controller */
    [ASPEED_DEV_TACH]    = 0x140C0000, /* PWM & Fan Tacho Controller */
    [ASPEED_DEV_AHB2PCIRC2] = 0x140D0000, /* AHB to PCIe RC bridge controller #2 */
    [ASPEED_DEV_PCI2BMC2] = 0x140F0000, /* PCIe Host to BMC Controller #2 */
    [ASPEED_DEV_PRIV1]   = 0x14100000, /* Privilege Control #1 */
    [ASPEED_DEV_SRAM]    = 0x14BC0000, /* SRAM Memory Buffer */
    [ASPEED_DEV_ADC]     = 0x14C00000, /* ADC Controller */
    [ASPEED_DEV_SGMII]   = 0x14C01000, /* SGMII Controller */
    [ASPEED_DEV_SCU1]    = 0x14C02000, /* System Control Unit #1 (SCU1) */
    [ASPEED_DEV_BBSRAM]  = 0x14C04000, /* Battery Backed SRAM */
    [ASPEED_DEV_ESPI]    = 0x14C05000, /* eSPI Controller */
    [ASPEED_DEV_OTP]     = 0x14C07000, /* OTP Controller */
    [ASPEED_DEV_JTAG1]   = 0x14C09000, /* JTAG Master Controller #1 */
    [ASPEED_DEV_SRAMMC]  = 0x14C0A000, /* SRAM Memory Controller */
    [ASPEED_DEV_GPIO1]   = 0x14C0B000, /* GPIO Controller #1 */
    [ASPEED_DEV_SGPIO0]  = 0x14C0C000, /* SGPIO Master Controller #0 */
    [ASPEED_DEV_SGPIO1]  = 0x14C0D000, /* SGPIO Master Controller #1 */
    [ASPEED_DEV_SPGIOM0] = 0x14C0E000, /* SGPIO Monitor Controller #0 */
    [ASPEED_DEV_SGPIOM1] = 0x14C0E800, /* SGPIO Monitor Controller #1 */
    [ASPEED_DEV_I2C]     = 0x14C0F000, /* I2C/SMBus Controller */
    [ASPEED_DEV_UARTDMA1]= 0x14C12000, /* UART DMA #1 */
    [ASPEED_DEV_BMCUART] = 0x14C13000, /* BMC UART - debug */
    [ASPEED_DEV_SMBUSF]  = 0x14C14000, /* SMBus Filter Controller */
    [ASPEED_DEV_QSPIF0]  = 0x14C15000, /* QSPI Filter Controller #0 */
    [ASPEED_DEV_QSPIF1]  = 0x14C16000, /* QSPI Filter Controller #1 */
    [ASPEED_DEV_QSPIF2]  = 0x14C17000, /* QSPI Filter Controller #2 */
    [ASPEED_DEV_QSPIF3]  = 0x14C18000, /* QSPI Filter Controller #3 */
    [ASPEED_DEV_MCTP2]   = 0x14C1A000, /* MCTP Controller #2 (MCTP2) */
    [ASPEED_DEV_MSI2]    = 0x14C1B000, /* MSI Controller #2 (MSI2) */
    [ASPEED_DEV_APB2PCIRC2] = 0x14C1D800, /* APB to PCIe RC Bridge #2 */
    [ASPEED_DEV_BRIDGE1] = 0x14C1E000, /* Internal Bridge Controller #1 */
    [ASPEED_DEV_PECI]    = 0x14C1F000, /* PECI Controller */
    [ASPEED_DEV_I2C]     = 0x14C20000, /* I3C Controller */
    [ASPEED_DEV_VUART0]  = 0x14C30000, /* Virtual UART0 (VUART0) */
    [ASPEED_DEV_VUART1]  = 0x14C30100, /* Virtual UART1 (VUART1) */
    [ASPEED_DEV_VUART2]  = 0x14C30200, /* Virtual UART2 (VUART2) */
    [ASPEED_DEV_VUART3]  = 0x14C30300, /* Virtual UART3 (VUART3) */
    [ASPEED_DEV_LPC]     = 0x14C31000, /* LPC Controller */
    [ASPEED_DEV_UART0]   = 0x14C33000, /* UART Controller (UART0) */
    [ASPEED_DEV_UART1]   = 0x14C33100, /* UART Controller (UART1) */
    [ASPEED_DEV_UART2]   = 0x14C33200, /* UART Controller (UART2) */
    [ASPEED_DEV_UART3]   = 0x14C33300, /* UART Controller (UART3) */
    [ASPEED_DEV_UART5]   = 0x14C33400, /* UART Controller (UART5) */
    [ASPEED_DEV_UART6]   = 0x14C33500, /* UART Controller (UART6) */
    [ASPEED_DEV_UART7]   = 0x14C33600, /* UART Controller (UART7) */
    [ASPEED_DEV_UART8]   = 0x14C33700, /* UART Controller (UART8) */
    [ASPEED_DEV_UART9]   = 0x14C33800, /* UART Controller (UART9) */
    [ASPEED_DEV_UART10]  = 0x14C33900, /* UART Controller (UART10) */
    [ASPEED_DEV_UART11]  = 0x14C33A00, /* UART Controller (UART11) */
    [ASPEED_DEV_BUCUART] = 0x14C33B00, /* UART Controller (BMC UART) */
    [ASPEED_DEV_LTPI0]   = 0x14C34000, /* LTPI Controller #0 */
    [ASPEED_DEV_LTPI1]   = 0x14C35000, /* LTPI Controller #1 */
    [ASPEED_DEV_WDT]     = 0x14C37000, /* Watchdog Timer (WDT) */
    [ASPEED_DEV_EDAF]    = 0x14C38000, /* eDAF Bridge Controller */
    [ASPEED_DEV_IPC1]    = 0x14C39000, /* Inter-Processor Controller #1 */
    [ASPEED_DEV_VGA1]    = 0x14C3A000, /* VGA Link Controller #1 */
    [ASPEED_DEV_TRNG]    = 0x14C3B000, /* TRNG Controller  */
    [ASPEED_DEV_SPGIO]   = 0x14C3C000, /* SGPIO Slave Controller */
    [ASPEED_DEV_CANBUS]  = 0x14C3E000, /* CANBUS Controller */
    [ASPEED_DEV_FSIBUF]  = 0x20000000, /* FSI memory buffer */
    [ASPEED_DEV_FMCMAP]  = 0x100000000, /* BMC SPI Flash Memory */
    [ASPEED_DEV_SPI0MAP] = 0x180000000, /* SPI0 Flash Memory */
    [ASPEED_DEV_SPI1MAP] = 0x200000000, /* SPI1 Flash Memory */
    [ASPEED_DEV_SPI2MAP] = 0x280000000, /* SPI2 Flash Memory */
    [ASPEED_DEV_SDRAM]   = 0x400000000, /* SDRAM */
};

#define ASPEED_A7MPCORE_ADDR 0x40460000
#define AST2700_MAX_IRQ 132

static const int aspeed_soc_ast2700_irqmap[] = {
    [ASPEED_DEV_UART5]     = 8,
    [ASPEED_DEV_TIMER1]    = 16,
    [ASPEED_DEV_TIMER2]    = 17,
    [ASPEED_DEV_TIMER3]    = 18,
    [ASPEED_DEV_TIMER4]    = 19,
    [ASPEED_DEV_TIMER5]    = 20,
    [ASPEED_DEV_TIMER6]    = 21,
    [ASPEED_DEV_TIMER7]    = 22,
    [ASPEED_DEV_TIMER8]    = 23,
    [ASPEED_DEV_I2C]       = 130,
    [ASPEED_DEV_I3C]       = 131,
};

static qemu_irq aspeed_soc_ast2700_get_irq(AspeedSoCState *s, int dev)
{
    Aspeed2700SoCState *a = ASPEED2700_SOC(s);
    AspeedSoCClass *sc = ASPEED_SOC_GET_CLASS(s);

    return qdev_get_gpio_in(DEVICE(&a->a35mpcore), sc->irqmap[dev]);
}

static void aspeed_soc_ast2700_init(Object *obj)
{
    AspeedSoCState *s = ASPEED_SOC(obj);
    AspeedSoCClass *sc = ASPEED_SOC_GET_CLASS(s);
    Aspeed2700SoCState *a = ASPEED2700_SOC(s);
    int i;
    char socname[8];
    char typename[64];

    if (sscanf(sc->name, "%7s", socname) != 1) {
        g_assert_not_reached();
    }

    for (i = 0; i < sc->num_cpus; i++) {
        object_initialize_child(obj, "cpu[*]", &a->cpu[i], sc->cpu_type);
    }
    snprintf(typename, sizeof(typename), "aspeed.scu-%s", socname);
    object_initialize_child(obj, "scu", &s->scu, typename);
    qdev_prop_set_uint32(DEVICE(&s->scu), "silicon-rev",
                         sc->silicon_rev);
    object_property_add_alias(obj, "hw-strap1", OBJECT(&s->scu),
                              "hw-strap1");
    object_property_add_alias(obj, "hw-strap2", OBJECT(&s->scu),
                              "hw-strap2");
    object_property_add_alias(obj, "hw-prot-key", OBJECT(&s->scu),
                              "hw-prot-key");

    object_initialize_child(obj, "a35mpcore", &a->a35mpcore,
                            TYPE_A15MPCORE_PRIV);

    object_initialize_child(obj, "rtc", &s->rtc, TYPE_ASPEED_RTC);

    snprintf(typename, sizeof(typename), "aspeed.timer-%s", socname);
    object_initialize_child(obj, "timerctrl", &s->timerctrl, typename);

    snprintf(typename, sizeof(typename), "aspeed.adc-%s", socname);
    object_initialize_child(obj, "adc", &s->adc, typename);

    snprintf(typename, sizeof(typename), "aspeed.i2c-%s", socname);
    object_initialize_child(obj, "i2c", &s->i2c, typename);

    object_initialize_child(obj, "peci", &s->peci, TYPE_ASPEED_PECI);

    snprintf(typename, sizeof(typename), "aspeed.fmc-%s", socname);
    object_initialize_child(obj, "fmc", &s->fmc, typename);

    for (i = 0; i < sc->spis_num; i++) {
        snprintf(typename, sizeof(typename), "aspeed.spi%d-%s", i + 1, socname);
        object_initialize_child(obj, "spi[*]", &s->spi[i], typename);
    }

    for (i = 0; i < sc->ehcis_num; i++) {
        object_initialize_child(obj, "ehci[*]", &s->ehci[i],
                                TYPE_PLATFORM_EHCI);
    }

    snprintf(typename, sizeof(typename), "aspeed.sdmc-%s", socname);
    object_initialize_child(obj, "sdmc", &s->sdmc, typename);
    object_property_add_alias(obj, "ram-size", OBJECT(&s->sdmc),
                              "ram-size");

    for (i = 0; i < sc->wdts_num; i++) {
        snprintf(typename, sizeof(typename), "aspeed.wdt-%s", socname);
        object_initialize_child(obj, "wdt[*]", &s->wdt[i], typename);
    }

    for (i = 0; i < sc->macs_num; i++) {
        object_initialize_child(obj, "ftgmac100[*]", &s->ftgmac100[i],
                                TYPE_FTGMAC100);

        object_initialize_child(obj, "mii[*]", &s->mii[i], TYPE_ASPEED_MII);
    }
    for (i = 0; i < sc->uarts_num; i++) {
        object_initialize_child(obj, "uart[*]", &s->uart[i], TYPE_SERIAL_MM);
    }

    snprintf(typename, sizeof(typename), TYPE_ASPEED_XDMA "-%s", socname);
    object_initialize_child(obj, "xdma", &s->xdma, typename);

    snprintf(typename, sizeof(typename), "aspeed.gpio-%s", socname);
    object_initialize_child(obj, "gpio", &s->gpio, typename);

    snprintf(typename, sizeof(typename), "aspeed.gpio-%s-1_8v", socname);
    object_initialize_child(obj, "gpio_1_8v", &s->gpio_1_8v, typename);

    object_initialize_child(obj, "sd-controller", &s->sdhci,
                            TYPE_ASPEED_SDHCI);

    object_property_set_int(OBJECT(&s->sdhci), "num-slots", 2, &error_abort);


    /* Init sd card slot class here so that they're under the correct parent */
    for (i = 0; i < ASPEED_SDHCI_NUM_SLOTS; ++i) {
        object_initialize_child(obj, "sd-controller.sdhci[*]",
                                &s->sdhci.slots[i], TYPE_SYSBUS_SDHCI);
    }

    object_initialize_child(obj, "emmc-controller", &s->emmc,
                            TYPE_ASPEED_SDHCI);

    object_property_set_int(OBJECT(&s->emmc), "num-slots", 1, &error_abort);

    object_initialize_child(obj, "emmc-controller.sdhci", &s->emmc.slots[0],
                            TYPE_SYSBUS_SDHCI);

    object_initialize_child(obj, "lpc", &s->lpc, TYPE_ASPEED_LPC);

    snprintf(typename, sizeof(typename), "aspeed.hace-%s", socname);
    object_initialize_child(obj, "hace", &s->hace, typename);

    object_initialize_child(obj, "i3c", &s->i3c, TYPE_ASPEED_I3C);
    object_initialize_child(obj, "sbc", &s->sbc, TYPE_ASPEED_SBC);

    object_initialize_child(obj, "iomem", &s->iomem, TYPE_UNIMPLEMENTED_DEVICE);
    object_initialize_child(obj, "video", &s->video, TYPE_UNIMPLEMENTED_DEVICE);
    object_initialize_child(obj, "dpmcu", &s->dpmcu, TYPE_UNIMPLEMENTED_DEVICE);
    object_initialize_child(obj, "emmc-boot-controller",
                            &s->emmc_boot_controller,
                            TYPE_UNIMPLEMENTED_DEVICE);
}

/*
 * ASPEED ast2700 has 0x0 as cluster ID
 *
 * https://developer.arm.com/documentation/ddi0388/e/the-system-control-coprocessors/summary-of-system-control-coprocessor-registers/multiprocessor-affinity-register
 */
static uint64_t aspeed_calc_affinity(int cpu)
{
    return (0x0 << ARM_AFF1_SHIFT) | cpu;
}

static void aspeed_soc_ast2700_realize(DeviceState *dev, Error **errp)
{
    int i;
    Aspeed2700SoCState *a = ASPEED2700_SOC(dev);
    AspeedSoCState *s = ASPEED_SOC(dev);
    AspeedSoCClass *sc = ASPEED_SOC_GET_CLASS(s);
    Error *err = NULL;
    qemu_irq irq;
    g_autofree char *sram_name = NULL;

    /* Default boot region (SPI memory or ROMs) */
    memory_region_init(&s->spi_boot_container, OBJECT(s),
                       "aspeed.spi_boot_container", 0x10000000);
    memory_region_add_subregion(s->memory, sc->memmap[ASPEED_DEV_SPI_BOOT],
                                &s->spi_boot_container);

    /* IO space */
    aspeed_mmio_map_unimplemented(s, SYS_BUS_DEVICE(&s->iomem), "aspeed.io",
                                  sc->memmap[ASPEED_DEV_IOMEM],
                                  ASPEED_SOC_IOMEM_SIZE);

    /* Video engine stub */
    aspeed_mmio_map_unimplemented(s, SYS_BUS_DEVICE(&s->video), "aspeed.video",
                                  sc->memmap[ASPEED_DEV_VIDEO], 0x1000);

    /* eMMC Boot Controller stub */
    aspeed_mmio_map_unimplemented(s, SYS_BUS_DEVICE(&s->emmc_boot_controller),
                                  "aspeed.emmc-boot-controller",
                                  sc->memmap[ASPEED_DEV_EMMC_BC], 0x1000);

    /* CPU */
    for (i = 0; i < sc->num_cpus; i++) {
        if (sc->num_cpus > 1) {
            object_property_set_int(OBJECT(&a->cpu[i]), "reset-cbar",
                                    ASPEED_A7MPCORE_ADDR, &error_abort);
        }
        object_property_set_int(OBJECT(&a->cpu[i]), "mp-affinity",
                                aspeed_calc_affinity(i), &error_abort);

        object_property_set_int(OBJECT(&a->cpu[i]), "cntfrq", 1125000000,
                                &error_abort);
        object_property_set_link(OBJECT(&a->cpu[i]), "memory",
                                 OBJECT(s->memory), &error_abort);

        if (!qdev_realize(DEVICE(&a->cpu[i]), NULL, errp)) {
            return;
        }
    }

    /* A7MPCORE */
    object_property_set_int(OBJECT(&a->a35mpcore), "num-cpu", sc->num_cpus,
                            &error_abort);
    object_property_set_int(OBJECT(&a->a35mpcore), "num-irq",
                            ROUND_UP(AST2700_MAX_IRQ + GIC_INTERNAL, 32),
                            &error_abort);

    sysbus_realize(SYS_BUS_DEVICE(&a->a35mpcore), &error_abort);
    aspeed_mmio_map(s, SYS_BUS_DEVICE(&a->a35mpcore), 0, ASPEED_A7MPCORE_ADDR);

    for (i = 0; i < sc->num_cpus; i++) {
        SysBusDevice *sbd = SYS_BUS_DEVICE(&a->a35mpcore);
        DeviceState  *d   = DEVICE(&a->cpu[i]);

        irq = qdev_get_gpio_in(d, ARM_CPU_IRQ);
        sysbus_connect_irq(sbd, i, irq);
        irq = qdev_get_gpio_in(d, ARM_CPU_FIQ);
        sysbus_connect_irq(sbd, i + sc->num_cpus, irq);
        irq = qdev_get_gpio_in(d, ARM_CPU_VIRQ);
        sysbus_connect_irq(sbd, i + 2 * sc->num_cpus, irq);
        irq = qdev_get_gpio_in(d, ARM_CPU_VFIQ);
        sysbus_connect_irq(sbd, i + 3 * sc->num_cpus, irq);
    }

    /* SRAM */
    sram_name = g_strdup_printf("aspeed.sram.%d", CPU(&a->cpu[0])->cpu_index);
    memory_region_init_ram(&s->sram, OBJECT(s), sram_name, sc->sram_size, &err);
    if (err) {
        error_propagate(errp, err);
        return;
    }
    memory_region_add_subregion(s->memory,
                                sc->memmap[ASPEED_DEV_SRAM], &s->sram);

    /* DPMCU */
    aspeed_mmio_map_unimplemented(s, SYS_BUS_DEVICE(&s->dpmcu), "aspeed.dpmcu",
                                  sc->memmap[ASPEED_DEV_DPMCU],
                                  ASPEED_SOC_DPMCU_SIZE);

    /* SCU */
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->scu), errp)) {
        return;
    }
    aspeed_mmio_map(s, SYS_BUS_DEVICE(&s->scu), 0, sc->memmap[ASPEED_DEV_SCU]);

    /* RTC */
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->rtc), errp)) {
        return;
    }
    aspeed_mmio_map(s, SYS_BUS_DEVICE(&s->rtc), 0, sc->memmap[ASPEED_DEV_RTC]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->rtc), 0,
                       aspeed_soc_get_irq(s, ASPEED_DEV_RTC));

    /* Timer */
    object_property_set_link(OBJECT(&s->timerctrl), "scu", OBJECT(&s->scu),
                             &error_abort);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->timerctrl), errp)) {
        return;
    }
    aspeed_mmio_map(s, SYS_BUS_DEVICE(&s->timerctrl), 0,
                    sc->memmap[ASPEED_DEV_TIMER1]);
    for (i = 0; i < ASPEED_TIMER_NR_TIMERS; i++) {
        qemu_irq irq = aspeed_soc_get_irq(s, ASPEED_DEV_TIMER1 + i);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->timerctrl), i, irq);
    }

    /* Watch dog */
    for (i = 0; i < sc->wdts_num; i++) {
        AspeedWDTClass *awc = ASPEED_WDT_GET_CLASS(&s->wdt[i]);

        object_property_set_link(OBJECT(&s->wdt[i]), "scu", OBJECT(&s->scu),
                                 &error_abort);
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->wdt[i]), errp)) {
            return;
        }
        aspeed_mmio_map(s, SYS_BUS_DEVICE(&s->wdt[i]), 0,
                        sc->memmap[ASPEED_DEV_WDT] + i * awc->iosize);
    }

    /* ADC */
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->adc), errp)) {
        return;
    }
    aspeed_mmio_map(s, SYS_BUS_DEVICE(&s->adc), 0, sc->memmap[ASPEED_DEV_ADC]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->adc), 0,
                       aspeed_soc_get_irq(s, ASPEED_DEV_ADC));

    /* UART */
    if (!aspeed_soc_uart_realize(s, errp)) {
        return;
    }

    /* I2C */
    object_property_set_link(OBJECT(&s->i2c), "dram", OBJECT(s->dram_mr),
                             &error_abort);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->i2c), errp)) {
        return;
    }
    aspeed_mmio_map(s, SYS_BUS_DEVICE(&s->i2c), 0, sc->memmap[ASPEED_DEV_I2C]);
    for (i = 0; i < ASPEED_I2C_GET_CLASS(&s->i2c)->num_busses; i++) {
        qemu_irq irq = qdev_get_gpio_in(DEVICE(&a->a35mpcore),
                                        sc->irqmap[ASPEED_DEV_I2C] + i);
        /* The AST2700 I2C controller has one IRQ per bus. */
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->i2c.busses[i]), 0, irq);
    }

    /* PECI */
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->peci), errp)) {
        return;
    }
    aspeed_mmio_map(s, SYS_BUS_DEVICE(&s->peci), 0,
                    sc->memmap[ASPEED_DEV_PECI]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->peci), 0,
                       aspeed_soc_get_irq(s, ASPEED_DEV_PECI));

    /* FMC, The number of CS is set at the board level */
    object_property_set_link(OBJECT(&s->fmc), "dram", OBJECT(s->dram_mr),
                             &error_abort);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->fmc), errp)) {
        return;
    }
    aspeed_mmio_map(s, SYS_BUS_DEVICE(&s->fmc), 0, sc->memmap[ASPEED_DEV_FMC]);
    aspeed_mmio_map(s, SYS_BUS_DEVICE(&s->fmc), 1,
                    ASPEED_SMC_GET_CLASS(&s->fmc)->flash_window_base);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->fmc), 0,
                       aspeed_soc_get_irq(s, ASPEED_DEV_FMC));

    /* Set up an alias on the FMC CE0 region (boot default) */
    MemoryRegion *fmc0_mmio = &s->fmc.flashes[0].mmio;
    memory_region_init_alias(&s->spi_boot, OBJECT(s), "aspeed.spi_boot",
                             fmc0_mmio, 0, memory_region_size(fmc0_mmio));
    memory_region_add_subregion(&s->spi_boot_container, 0x0, &s->spi_boot);

    /* SPI */
    for (i = 0; i < sc->spis_num; i++) {
        object_property_set_link(OBJECT(&s->spi[i]), "dram",
                                 OBJECT(s->dram_mr), &error_abort);
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->spi[i]), errp)) {
            return;
        }
        aspeed_mmio_map(s, SYS_BUS_DEVICE(&s->spi[i]), 0,
                        sc->memmap[ASPEED_DEV_SPI1 + i]);
        aspeed_mmio_map(s, SYS_BUS_DEVICE(&s->spi[i]), 1,
                        ASPEED_SMC_GET_CLASS(&s->spi[i])->flash_window_base);
    }

    /* EHCI */
    for (i = 0; i < sc->ehcis_num; i++) {
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->ehci[i]), errp)) {
            return;
        }
        aspeed_mmio_map(s, SYS_BUS_DEVICE(&s->ehci[i]), 0,
                        sc->memmap[ASPEED_DEV_EHCI1 + i]);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->ehci[i]), 0,
                           aspeed_soc_get_irq(s, ASPEED_DEV_EHCI1 + i));
    }

    /* SDMC - SDRAM Memory Controller */
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->sdmc), errp)) {
        return;
    }
    aspeed_mmio_map(s, SYS_BUS_DEVICE(&s->sdmc), 0,
                    sc->memmap[ASPEED_DEV_SDMC]);

    /* RAM */
    if (!aspeed_soc_dram_init(s, errp)) {
        return;
    }

    /* Net */
    for (i = 0; i < sc->macs_num; i++) {
        object_property_set_bool(OBJECT(&s->ftgmac100[i]), "aspeed", true,
                                 &error_abort);
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->ftgmac100[i]), errp)) {
            return;
        }
        aspeed_mmio_map(s, SYS_BUS_DEVICE(&s->ftgmac100[i]), 0,
                        sc->memmap[ASPEED_DEV_ETH1 + i]);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->ftgmac100[i]), 0,
                           aspeed_soc_get_irq(s, ASPEED_DEV_ETH1 + i));

        object_property_set_link(OBJECT(&s->mii[i]), "nic",
                                 OBJECT(&s->ftgmac100[i]), &error_abort);
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->mii[i]), errp)) {
            return;
        }

        aspeed_mmio_map(s, SYS_BUS_DEVICE(&s->mii[i]), 0,
                        sc->memmap[ASPEED_DEV_MII1 + i]);
    }

    /* XDMA */
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->xdma), errp)) {
        return;
    }
    aspeed_mmio_map(s, SYS_BUS_DEVICE(&s->xdma), 0,
                    sc->memmap[ASPEED_DEV_XDMA]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->xdma), 0,
                       aspeed_soc_get_irq(s, ASPEED_DEV_XDMA));

    /* GPIO */
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->gpio), errp)) {
        return;
    }
    aspeed_mmio_map(s, SYS_BUS_DEVICE(&s->gpio), 0, sc->memmap[ASPEED_DEV_GPIO]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->gpio), 0,
                       aspeed_soc_get_irq(s, ASPEED_DEV_GPIO));

    if (!sysbus_realize(SYS_BUS_DEVICE(&s->gpio_1_8v), errp)) {
        return;
    }
    aspeed_mmio_map(s, SYS_BUS_DEVICE(&s->gpio_1_8v), 0,
                    sc->memmap[ASPEED_DEV_GPIO_1_8V]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->gpio_1_8v), 0,
                       aspeed_soc_get_irq(s, ASPEED_DEV_GPIO_1_8V));

    /* SDHCI */
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->sdhci), errp)) {
        return;
    }
    aspeed_mmio_map(s, SYS_BUS_DEVICE(&s->sdhci), 0,
                    sc->memmap[ASPEED_DEV_SDHCI]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->sdhci), 0,
                       aspeed_soc_get_irq(s, ASPEED_DEV_SDHCI));

    /* eMMC */
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->emmc), errp)) {
        return;
    }
    aspeed_mmio_map(s, SYS_BUS_DEVICE(&s->emmc), 0,
                    sc->memmap[ASPEED_DEV_EMMC]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->emmc), 0,
                       aspeed_soc_get_irq(s, ASPEED_DEV_EMMC));

    /* LPC */
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->lpc), errp)) {
        return;
    }
    aspeed_mmio_map(s, SYS_BUS_DEVICE(&s->lpc), 0, sc->memmap[ASPEED_DEV_LPC]);

    /* Connect the LPC IRQ to the GIC. It is otherwise unused. */
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->lpc), 0,
                       aspeed_soc_get_irq(s, ASPEED_DEV_LPC));

    /*
     * On the AST2600 LPC subdevice IRQs are connected straight to the GIC.
     *
     * LPC subdevice IRQ sources are offset from 1 because the LPC model caters
     * to the AST2400 and AST2500. SoCs before the AST2600 have one LPC IRQ
     * shared across the subdevices, and the shared IRQ output to the VIC is at
     * offset 0.
     */
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->lpc), 1 + aspeed_lpc_kcs_1,
                       qdev_get_gpio_in(DEVICE(&a->a35mpcore),
                                sc->irqmap[ASPEED_DEV_KCS] + aspeed_lpc_kcs_1));

    sysbus_connect_irq(SYS_BUS_DEVICE(&s->lpc), 1 + aspeed_lpc_kcs_2,
                       qdev_get_gpio_in(DEVICE(&a->a35mpcore),
                                sc->irqmap[ASPEED_DEV_KCS] + aspeed_lpc_kcs_2));

    sysbus_connect_irq(SYS_BUS_DEVICE(&s->lpc), 1 + aspeed_lpc_kcs_3,
                       qdev_get_gpio_in(DEVICE(&a->a35mpcore),
                                sc->irqmap[ASPEED_DEV_KCS] + aspeed_lpc_kcs_3));

    sysbus_connect_irq(SYS_BUS_DEVICE(&s->lpc), 1 + aspeed_lpc_kcs_4,
                       qdev_get_gpio_in(DEVICE(&a->a35mpcore),
                                sc->irqmap[ASPEED_DEV_KCS] + aspeed_lpc_kcs_4));

    /* HACE */
    object_property_set_link(OBJECT(&s->hace), "dram", OBJECT(s->dram_mr),
                             &error_abort);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->hace), errp)) {
        return;
    }
    aspeed_mmio_map(s, SYS_BUS_DEVICE(&s->hace), 0,
                    sc->memmap[ASPEED_DEV_HACE]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->hace), 0,
                       aspeed_soc_get_irq(s, ASPEED_DEV_HACE));

    /* I3C */
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->i3c), errp)) {
        return;
    }
    aspeed_mmio_map(s, SYS_BUS_DEVICE(&s->i3c), 0, sc->memmap[ASPEED_DEV_I3C]);
    for (i = 0; i < ASPEED_I3C_NR_DEVICES; i++) {
        qemu_irq irq = qdev_get_gpio_in(DEVICE(&a->a35mpcore),
                                        sc->irqmap[ASPEED_DEV_I3C] + i);
        /* The AST2700 I3C controller has one IRQ per bus. */
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->i3c.devices[i]), 0, irq);
    }

    /* Secure Boot Controller */
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->sbc), errp)) {
        return;
    }
    aspeed_mmio_map(s, SYS_BUS_DEVICE(&s->sbc), 0, sc->memmap[ASPEED_DEV_SBC]);
}

static void aspeed_soc_ast2700_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    AspeedSoCClass *sc = ASPEED_SOC_CLASS(oc);

    dc->realize      = aspeed_soc_ast2700_realize;

    sc->name         = "ast2700-a0";
    sc->cpu_type     = ARM_CPU_TYPE_NAME("cortex-a35");
    sc->silicon_rev  = AST2700_A0_SILICON_REV;
    sc->sram_size    = 0x16400;
    sc->spis_num     = 2;
    sc->ehcis_num    = 2;
    sc->wdts_num     = 4;
    sc->macs_num     = 4;
    sc->uarts_num    = 13;
    sc->irqmap       = aspeed_soc_ast2700_irqmap;
    sc->memmap       = aspeed_soc_ast2700_memmap;
    sc->num_cpus     = 2;
    sc->get_irq      = aspeed_soc_ast2700_get_irq;
}

static const TypeInfo aspeed_soc_ast2700_types[] = {
    {
        .name           = TYPE_ASPEED2700_SOC,
        .parent         = TYPE_ASPEED_SOC,
        .instance_size  = sizeof(Aspeed2700SoCState),
        .abstract       = true,
    }, {
        .name           = "ast2700-a0",
        .parent         = TYPE_ASPEED2700_SOC,
        .instance_init  = aspeed_soc_ast2700_init,
        .class_init     = aspeed_soc_ast2700_class_init,
    },
};

DEFINE_TYPES(aspeed_soc_ast2700_types)
