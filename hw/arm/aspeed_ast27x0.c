/*
 * ASPEED SoC 27x0 family
 *
 * Copyright (C) 2023 ASPEED Technology Inc.
 *
 * This code is licensed under the GPL version 2 or later.  See
 * the COPYING file in the top-level directory.
 *
 * Implementation extracted from the AST2600 and adapted for Asth27x0.
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
#include "hw/intc/arm_gicv3.h"

#define ASPEED_SOC_IOMEM_SIZE       0x04000000
#define ASPEED_SOC_DPMCU_SIZE       0x00040000

static const hwaddr aspeed_soc_ast2750_memmap[] = {
        // [ASPEED_DEV_SPI_BOOT]  = ASPEED_SOC_SPI_BOOT_ADDR,
        [ASPEED_DEV_SPI_BOOT]  =  0x400000000,
        [ASPEED_DEV_SRAM]      =  0x10000000,
      //[ASPEED_DEV_IOMEM]     =  0x10000000,
        [ASPEED_DEV_SDMC]      =  0x12c00000,
        [ASPEED_DEV_SCU]       =  0x12c02000,
        [ASPEED_DEV_SCU1]      =  0x14c02000,
        [ASPEED_DEV_UART0]     =  0X14C33000,
        [ASPEED_DEV_UART1]     =  0X14C33100,
        [ASPEED_DEV_UART2]     =  0X14C33200,
        [ASPEED_DEV_UART3]     =  0X14C33300,
        [ASPEED_DEV_UART4]     =  0X12C1A000,
        [ASPEED_DEV_UART5]     =  0X14C33400,
        [ASPEED_DEV_UART6]     =  0X14C33500,
        [ASPEED_DEV_UART7]     =  0X14C33600,
        [ASPEED_DEV_UART8]     =  0X14C33700,
        [ASPEED_DEV_UART9]     =  0X14C33800,
        [ASPEED_DEV_UART10]    =  0X14C33900,
        [ASPEED_DEV_UART11]    =  0X14C33A00,
        [ASPEED_DEV_UART12]    =  0X14C33B00,
        [ASPEED_DEV_WDT]       =  0x14c37000,
        [ASPEED_DEV_VUART1]    =  0X12C18000,
        [ASPEED_DEV_VUART2]    =  0X12C18100,
        [ASPEED_DEV_VUART3]    =  0X12C18200,
        [ASPEED_DEV_VUART4]    =  0X12C18300,
        [ASPEED_DEV_FMC]       =  0x14000000,
        [ASPEED_DEV_SPI0]      =  0x14010000,
        [ASPEED_DEV_SPI1]      =  0x14020000,
        [ASPEED_DEV_SPI2]      =  0x14030000,
        [ASPEED_DEV_SDRAM]     = 0x400000000,
	[ASPEED_DEV_MII1]      =  0x14040000,
	[ASPEED_DEV_MII2]      =  0x14040008,
	[ASPEED_DEV_MII3]      =  0x14040010,
	[ASPEED_DEV_ETH1]      =  0x14050000,
	[ASPEED_DEV_ETH2]      =  0x14060000,
	[ASPEED_DEV_ETH3]      =  0x14070000,
	[ASPEED_DEV_EMMC]      =  0x12090000,
	[ASPEED_DEV_VIC]       =  0x12100000,
};

#define AST2750_MAX_IRQ 197

/* Shared Peripheral Interrupt values below are offset by -32 from datasheet */
static const int aspeed_soc_ast2750_irqmap[] = {
    [ASPEED_DEV_UART0]     = 132,
    [ASPEED_DEV_UART1]     = 132,
    [ASPEED_DEV_UART2]     = 132,
    [ASPEED_DEV_UART3]     = 132,
    [ASPEED_DEV_UART4]     = 8,
    [ASPEED_DEV_UART5]     = 132,
    [ASPEED_DEV_UART6]     = 132,
    [ASPEED_DEV_UART7]     = 132,
    [ASPEED_DEV_UART8]     = 132,
    [ASPEED_DEV_UART9]     = 132,
    [ASPEED_DEV_UART10]    = 132,
    [ASPEED_DEV_UART11]    = 132,
    [ASPEED_DEV_UART12]    = 132,
    [ASPEED_DEV_UART13]    = 132,
    [ASPEED_DEV_FMC]       = 39,
    [ASPEED_DEV_SDMC]      = 0,
    [ASPEED_DEV_SCU]       = 12,
    [ASPEED_DEV_ADC]       = 78,
    [ASPEED_DEV_XDMA]      = 6,
    [ASPEED_DEV_SDHCI]     = 43,
    [ASPEED_DEV_EHCI1]     = 5,
    [ASPEED_DEV_EHCI2]     = 9,
    [ASPEED_DEV_EMMC]      = 15,
    [ASPEED_DEV_GPIO]      = 40,
    [ASPEED_DEV_GPIO_1_8V] = 11,
    [ASPEED_DEV_RTC]       = 13,
    [ASPEED_DEV_TIMER1]    = 16,
    [ASPEED_DEV_TIMER2]    = 17,
    [ASPEED_DEV_TIMER3]    = 18,
    [ASPEED_DEV_TIMER4]    = 19,
    [ASPEED_DEV_TIMER5]    = 20,
    [ASPEED_DEV_TIMER6]    = 21,
    [ASPEED_DEV_TIMER7]    = 22,
    [ASPEED_DEV_TIMER8]    = 23,
    [ASPEED_DEV_WDT]       = 24,
    [ASPEED_DEV_PWM]       = 44,
    [ASPEED_DEV_LPC]       = 35,
    [ASPEED_DEV_IBT]       = 143,
    [ASPEED_DEV_I2C]       = 110,   /* 110 -> 125 */
    [ASPEED_DEV_PECI]      = 38,
    [ASPEED_DEV_ETH1]      = 132,
    [ASPEED_DEV_ETH2]      = 132,
    [ASPEED_DEV_HACE]      = 4,
    [ASPEED_DEV_ETH3]      = 32,
    [ASPEED_DEV_ETH4]      = 33,
    [ASPEED_DEV_KCS]       = 138,   /* 138 -> 142 */
    [ASPEED_DEV_DP]        = 62,
    [ASPEED_DEV_I3C]       = 102,   /* 102 -> 107 */
};

static qemu_irq aspeed_soc_ast2750_get_irq(AspeedSoCState *s, int dev)
{
    AspeedSoCClass *sc = ASPEED_SOC_GET_CLASS(s);

    return qdev_get_gpio_in(s->gic, sc->irqmap[dev]);
}

static void aspeed_soc_ast2750_init(Object *obj)
{
    AspeedSoCState *s = ASPEED_SOC(obj);
    AspeedSoCClass *sc = ASPEED_SOC_GET_CLASS(s);
    int i;
    char socname[8];
    char typename[64];

    if (sscanf(sc->name, "%7s", socname) != 1) {
        g_assert_not_reached();
    }

    for (i = 0; i < sc->num_cpus; i++) {
        object_initialize_child(obj, "cpu[*]", &s->cpu[i], sc->cpu_type);
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

    snprintf(typename, sizeof(typename), "aspeed.scu1-%s", socname);
    object_initialize_child(obj, "scu1", &s->scu1, typename);
    qdev_prop_set_uint32(DEVICE(&s->scu1), "silicon-rev",
                         sc->silicon_rev);

    snprintf(typename, sizeof(typename), "aspeed.fmc-%s", socname);
    object_initialize_child(obj, "fmc", &s->fmc, typename);

    for (i = 0; i < sc->spis_num; i++) {
        snprintf(typename, sizeof(typename), "aspeed.spi%d-%s", i, socname);
        object_initialize_child(obj, "spi[*]", &s->spi[i], typename);
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

    object_initialize_child(obj, "sli", &s->sli, TYPE_ASPEED_SLI);

    object_initialize_child(obj, "iomem", &s->iomem, TYPE_UNIMPLEMENTED_DEVICE);

    object_initialize_child(obj, "intc", &s->intc, TYPE_ASPEED_INTC);

}

/*
 * ASPEED ast2750 has 0xf as cluster ID
 *
 * https://developer.arm.com/documentation/ddi0388/e/the-system-control-coprocessors/summary-of-system-control-coprocessor-registers/multiprocessor-affinity-register
 */
static uint64_t aspeed_calc_affinity(int cpu)
{
    return (0x0 << ARM_AFF1_SHIFT) | cpu;
}

static void aspeed_soc_ast2750_gic(DeviceState *dev, Error **errp)
{
    AspeedSoCState *s = ASPEED_SOC(dev);
    AspeedSoCClass *sc = ASPEED_SOC_GET_CLASS(s);
    int i;

    SysBusDevice *gicbusdev;
    s->gic = qdev_new(gicv3_class_name());
    qdev_prop_set_uint32(s->gic, "revision", 3);
    qdev_prop_set_uint32(s->gic, "num-cpu", sc->num_cpus);
    qdev_prop_set_uint32(s->gic, "num-irq", 288);

    uint32_t redist0_capacity = 6;
    uint32_t redist0_count = MIN(1, redist0_capacity);
    uint32_t nb_redist_regions = 8;
    qdev_prop_set_uint32(s->gic, "len-redist-region-count", nb_redist_regions);
    qdev_prop_set_uint32(s->gic, "redist-region-count[0]", redist0_count);

    gicbusdev = SYS_BUS_DEVICE(s->gic);
    sysbus_realize_and_unref(gicbusdev, errp);
    sysbus_mmio_map(gicbusdev, 0, 0x12200000);
    sysbus_mmio_map(gicbusdev, 1, 0x12280000);
    sysbus_mmio_map(gicbusdev, 2, 0x40440000);

    for (i = 0; i < sc->num_cpus; i++) {
        DeviceState *cpudev = DEVICE(qemu_get_cpu(i));
	int NUM_IRQS = 256, ARCH_GIC_MAINT_IRQ = 9, VIRTUAL_PMU_IRQ = 7;
	int ppibase = NUM_IRQS + i * GIC_INTERNAL + GIC_NR_SGIS;

	const int timer_irq[] = {
		[GTIMER_PHYS] = 14,
		[GTIMER_VIRT] = 11,
		[GTIMER_HYP]  = 10,
		[GTIMER_SEC]  = 13,
	};
	int j;

	for (j = 0; j < ARRAY_SIZE(timer_irq); j++) {
		qdev_connect_gpio_out(cpudev, j,
				qdev_get_gpio_in(s->gic,
					ppibase + timer_irq[j]));
	}

	qemu_irq irq = qdev_get_gpio_in(s->gic, ppibase + ARCH_GIC_MAINT_IRQ);
	qdev_connect_gpio_out_named(cpudev, "gicv3-maintenance-interrupt", 0, irq);
	qdev_connect_gpio_out_named(cpudev, "pmu-interrupt", 0, qdev_get_gpio_in(s->gic, ppibase + VIRTUAL_PMU_IRQ));

	sysbus_connect_irq(gicbusdev, i, qdev_get_gpio_in(cpudev, ARM_CPU_IRQ));
	sysbus_connect_irq(gicbusdev, i + sc->num_cpus, qdev_get_gpio_in(cpudev, ARM_CPU_FIQ));
	sysbus_connect_irq(gicbusdev, i + 2 * sc->num_cpus, qdev_get_gpio_in(cpudev, ARM_CPU_VIRQ));
	sysbus_connect_irq(gicbusdev, i + 3 * sc->num_cpus, qdev_get_gpio_in(cpudev, ARM_CPU_VFIQ));
    }

}

#if 1
static bool aspeed_ast2750_uart_realize(AspeedSoCState *s, Error **errp)
{
    AspeedSoCClass *sc = ASPEED_SOC_GET_CLASS(s);
    SerialMM *smm;

    for (int i=0, uart = ASPEED_DEV_UART0; i < sc->uarts_num; i++, uart++) {
    	smm = &s->uart[i];
	/* Chardev property is set by the machine. */
	qdev_prop_set_uint8(DEVICE(smm), "regshift", 2);
	qdev_prop_set_uint32(DEVICE(smm), "baudbase", 115200);
	qdev_set_legacy_instance_id(DEVICE(smm), sc->memmap[uart], 2);
	qdev_prop_set_uint8(DEVICE(smm), "endianness", DEVICE_LITTLE_ENDIAN);
	if (!sysbus_realize(SYS_BUS_DEVICE(smm), errp)) {
		    return false;
	}

	// sysbus_connect_irq(SYS_BUS_DEVICE(smm), 0, aspeed_soc_get_irq(s, uart));
	sysbus_connect_irq(SYS_BUS_DEVICE(smm), 0, qdev_get_gpio_in(s->gic, sc->irqmap[uart]));
	aspeed_mmio_map(s, SYS_BUS_DEVICE(smm), 0, sc->memmap[uart]);
    }
    return true;
}
#endif

static void aspeed_soc_ast2750_realize(DeviceState *dev, Error **errp)
{
    int i;
    AspeedSoCState *s = ASPEED_SOC(dev);
    AspeedSoCClass *sc = ASPEED_SOC_GET_CLASS(s);
    Error *err = NULL;
    g_autofree char *sram_name = NULL;

    /* Default boot region (SPI memory or ROMs) */
    memory_region_init(&s->spi_boot_container, OBJECT(s),
                       "aspeed.spi_boot_container", 0x400000000);
    memory_region_add_subregion(s->memory, sc->memmap[ASPEED_DEV_SPI_BOOT],
                                &s->spi_boot_container);

    /* IO space */
    aspeed_mmio_map_unimplemented(s, SYS_BUS_DEVICE(&s->iomem), "aspeed.io",
                                  sc->memmap[ASPEED_DEV_IOMEM],
                                  ASPEED_SOC_IOMEM_SIZE);

    /* CPU */
    for (i = 0; i < sc->num_cpus; i++) {
        object_property_set_int(OBJECT(&s->cpu[i]), "mp-affinity",
                                aspeed_calc_affinity(i), &error_abort);

        object_property_set_int(OBJECT(&s->cpu[i]), "cntfrq", 1125000000,
                                &error_abort);
        object_property_set_link(OBJECT(&s->cpu[i]), "memory",
                                 OBJECT(s->memory), &error_abort);

        if (!qdev_realize(DEVICE(&s->cpu[i]), NULL, errp)) {
            return;
        }
    }

    /* GIC */
    aspeed_soc_ast2750_gic(dev, errp);
    s->intc.gic = s->gic;

    /* INTC */
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->intc), errp)) {
    }
    aspeed_mmio_map(s, SYS_BUS_DEVICE(&s->intc), 0, sc->memmap[ASPEED_DEV_VIC]);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->intc), 0,
		    qdev_get_gpio_in(DEVICE(&s->cpu), ARM_CPU_IRQ));
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->intc), 1,
		    qdev_get_gpio_in(DEVICE(&s->cpu), ARM_CPU_FIQ));

    /* SRAM */
    sram_name = g_strdup_printf("aspeed.sram.%d", CPU(&s->cpu[0])->cpu_index);
    memory_region_init_ram(&s->sram, OBJECT(s), sram_name, sc->sram_size, &err);
    if (err) {
        error_propagate(errp, err);
        return;
    }
    memory_region_add_subregion(s->memory,
                                sc->memmap[ASPEED_DEV_SRAM], &s->sram);

    /* SCU */
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->scu), errp)) {
        return;
    }
    aspeed_mmio_map(s, SYS_BUS_DEVICE(&s->scu), 0, sc->memmap[ASPEED_DEV_SCU]);

    /* SCU1 */
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->scu1), errp)) {
        return;
    }
    aspeed_mmio_map(s, SYS_BUS_DEVICE(&s->scu1), 0, sc->memmap[ASPEED_DEV_SCU1]);

    /* UART */
    if (!aspeed_ast2750_uart_realize(s, errp)) {
        return;
    }

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
                        sc->memmap[ASPEED_DEV_SPI0 + i]);
        aspeed_mmio_map(s, SYS_BUS_DEVICE(&s->spi[i]), 1,
                        ASPEED_SMC_GET_CLASS(&s->spi[i])->flash_window_base);
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

    /* Watch dog */
#if 0
    create_unimplemented_device("ast27x0.wdt", sc->memmap[ASPEED_DEV_WDT], 0x800);
#else
    for (i = 0; i < sc->wdts_num; i++) {
        AspeedWDTClass *awc = ASPEED_WDT_GET_CLASS(&s->wdt[i]);
        hwaddr wdt_offset = sc->memmap[ASPEED_DEV_WDT] + i * awc->iosize;

        object_property_set_link(OBJECT(&s->wdt[i]), "scu", OBJECT(&s->scu),
                                 &error_abort);
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->wdt[i]), errp)) {
            return;
        }
        aspeed_mmio_map(s, SYS_BUS_DEVICE(&s->wdt[i]), 0, wdt_offset);
    }
#endif


    // object_property_set_link(OBJECT(&s->sli), "sli", OBJECT(&s->scu), &error_abort);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->sli), errp)) {
        return;
    }
    aspeed_mmio_map(s, SYS_BUS_DEVICE(&s->sli), 0, 0x12c17000);

    create_unimplemented_device("ast27x0.oooo", 0x12380000, 0x10000);
    create_unimplemented_device("ast27x0.ahb", 0x12000000, 0x1000);
    create_unimplemented_device("ast27x0.idk", 0x140b0000, 0x1000);
    create_unimplemented_device("ast27x0.emmc", sc->memmap[ASPEED_DEV_EMMC], 0x10000);
    create_unimplemented_device("ast27x0.ufs", 0x12c08000, 0x400);
    create_unimplemented_device("ast27x0.mctp0", 0x12c06000, 0x100);
    create_unimplemented_device("ast27x0.mctp1", 0x12c07000, 0x100);
    create_unimplemented_device("ast27x0.otp", 0x14c07000, 0x800);
    create_unimplemented_device("ast27x0.sdio", 0x14080000, 0x10000);
    create_unimplemented_device("ast27x0.vga0", 0x12c1d000, 0x1000);
    create_unimplemented_device("ast27x0.vga1", 0x14c3a000, 0x2000); // Size mismatch with datasheet
    create_unimplemented_device("ast1700.ltpi", 0x30000000, 0x1000000);
    create_unimplemented_device("ast27x0.gpio0", 0x12c11000, 0x800);
    create_unimplemented_device("ast27x0.rtc", 0x12c0f000, 0x20);
    create_unimplemented_device("ast27x0.gpio1", 0x14c0b000, 0x1000);
    create_unimplemented_device("ast27x0.sgpio_master0", 0x14c0c000, 0x1000);
    create_unimplemented_device("ast27x0.sgpio_master1", 0x14c0d000, 0x1000);
    create_unimplemented_device("ast27x0.sgpio_monitor0", 0x14c0e000, 0x100);
    create_unimplemented_device("ast27x0.sgpio_monitor1", 0x14c0e800, 0x100);
    create_unimplemented_device("ast27x0.sgpio_slave", 0x14c0e800, 0x800);
    create_unimplemented_device("ast27x0.i2c", 0x14c0f000, 0x2000);
    // create_unimplemented_device("ast27x0.intc", 0x12100000, 0x4000);
    create_unimplemented_device("ast27x0.inter_bridge1", 0x14c1e000, 0x1000);
    // create_unimplemented_device("ast27x0.inter_bridge0", 0x12c17000, 0x1000);
    create_unimplemented_device("ast27x0.dp", 0x12c0a000, 0x400);
    create_unimplemented_device("ast27x0.dpmcu", 0x11000000, 0x40000);
    create_unimplemented_device("ast27x0.pciembus", 0x12c21000, 0x2000);
    create_unimplemented_device("ast27x0.mdio", 0x14040000, 0x200);
    create_unimplemented_device("ast27x0.pcie_lpc0", 0x12c19000, 0x800);
    create_unimplemented_device("ast27x0.pcie_lpc1", 0x12c19800, 0x800);
    create_unimplemented_device("ast27x0.lpc0", 0x14c31000, 0x2000);
    create_unimplemented_device("ast27x0.lpc1", 0x14c32000, 0x1000);
    create_unimplemented_device("ast27x0.udma", 0x14c12000, 0x1000);
    create_unimplemented_device("ast27x0.can", 0x14c3e000, 0x2000);
    create_unimplemented_device("ast27x0.spi2-txrx", 0x14030100, 0x100);
    create_unimplemented_device("ast27x0.bmc-dev0", 0x12110000, 0x10000);
    create_unimplemented_device("ast27x0.bmc-dev1", 0x12120000, 0x10000);
    create_unimplemented_device("ast27x0.espi0", 0x14c05000, 0x1000);
    create_unimplemented_device("ast27x0.espi1", 0x14c06000, 0x1000);
    create_unimplemented_device("ast27x0.usb.phy3a", 0x12010000, 0x1000);
    create_unimplemented_device("ast27x0.usb.vhuba1", 0x12011000, 0x1000);
    create_unimplemented_device("ast27x0.usb.phy2a1", 0x12011800, 0x1000);
    create_unimplemented_device("ast27x0.usb.phy3b", 0x12020000, 0x1000);
    create_unimplemented_device("ast27x0.usb.vhubb1", 0x12021000, 0x1000);
    create_unimplemented_device("ast27x0.usb.phy2b1", 0x12021800, 0x1000);
    create_unimplemented_device("ast27x0.pcie_vuart0", 0x12c18000, 0x100);
    create_unimplemented_device("ast27x0.pcie_vuart1", 0x12c18100, 0x100);
    create_unimplemented_device("ast27x0.pcie_vuart2", 0x12c18200, 0x100);
    create_unimplemented_device("ast27x0.pcie_vuart3", 0x12c18300, 0x100);

    create_unimplemented_device("ast27x0.vuart0", 0x14c18000, 0x100);
    create_unimplemented_device("ast27x0.vuart1", 0x14c18100, 0x100);
    create_unimplemented_device("ast27x0.vuart2", 0x14c18200, 0x100);
    create_unimplemented_device("ast27x0.vuart3", 0x14c18300, 0x100);
    
    create_unimplemented_device("ast27x0.mac0", 0x14050000, 0x10000);
    create_unimplemented_device("ast27x0.mac1", 0x14060000, 0x10000);
    create_unimplemented_device("ast27x0.mac2", 0x14070000, 0x10000);

    create_unimplemented_device("ast27x0.i3c0",  0x14c20000, 0x1000);
    create_unimplemented_device("ast27x0.i3c1",  0x14c21000, 0x1000);
    create_unimplemented_device("ast27x0.i3c2",  0x14c22000, 0x1000);
    create_unimplemented_device("ast27x0.i3c3",  0x14c23000, 0x1000);
    create_unimplemented_device("ast27x0.i3c4",  0x14c24000, 0x1000);
    create_unimplemented_device("ast27x0.i3c5",  0x14c25000, 0x1000);
    create_unimplemented_device("ast27x0.i3c6",  0x14c26000, 0x1000);
    create_unimplemented_device("ast27x0.i3c7",  0x14c27000, 0x1000);
    create_unimplemented_device("ast27x0.i3c8",  0x14c28000, 0x1000);
    create_unimplemented_device("ast27x0.i3c9",  0x14c29000, 0x1000);
    create_unimplemented_device("ast27x0.i3c10", 0x14c2a000, 0x1000);
    create_unimplemented_device("ast27x0.i3c11", 0x14c2b000, 0x1000);
    create_unimplemented_device("ast27x0.i3c12", 0x14c2c000, 0x1000);
    create_unimplemented_device("ast27x0.i3c13", 0x14c2d000, 0x1000);
    create_unimplemented_device("ast27x0.i3c14", 0x14c2e000, 0x1000);
    create_unimplemented_device("ast27x0.i3c15", 0x14c2f000, 0x1000);

    create_unimplemented_device("ast27x0.pwm_tach", 0x140c0000, 0x200);
    create_unimplemented_device("ast27x0.rsss", 0x12080000, 0x1000);
    create_unimplemented_device("ast27x0.adc", 0x14c00000, 0x200);
    create_unimplemented_device("ast27x0.video0", 0x120a0000, 0x1000);
    create_unimplemented_device("ast27x0.video1", 0x120a1000, 0x1000);
    //create_unimplemented_device("ast27x0.", 0x, 0x);
}

static void aspeed_soc_ast2750_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    AspeedSoCClass *sc = ASPEED_SOC_CLASS(oc);

    dc->realize      = aspeed_soc_ast2750_realize;

    sc->name         = "ast2750-a0";
    sc->cpu_type     = ARM_CPU_TYPE_NAME("cortex-a35");
    sc->silicon_rev  = AST2750_A0_SILICON_REV;
    sc->sram_size    = 0x20000;
    sc->spis_num     = 3;
    sc->wdts_num     = 8;
    sc->macs_num     = 1;
    sc->uarts_num    = 13;
    sc->irqmap       = aspeed_soc_ast2750_irqmap;
    sc->memmap       = aspeed_soc_ast2750_memmap;
    sc->num_cpus     = 1;
    sc->get_irq      = aspeed_soc_ast2750_get_irq;
}

static const TypeInfo aspeed_soc_ast2750_type_info = {
    .name           = "ast2750-a0",
    .parent         = TYPE_ASPEED_SOC,
    .instance_size  = sizeof(AspeedSoCState),
    .instance_init  = aspeed_soc_ast2750_init,
    .class_init     = aspeed_soc_ast2750_class_init,
    .class_size     = sizeof(AspeedSoCClass),
};

static void aspeed_soc_register_types(void)
{
    type_register_static(&aspeed_soc_ast2750_type_info);
};

type_init(aspeed_soc_register_types)
