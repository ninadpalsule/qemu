#include "qemu/osdep.h"
#include "hw/intc/aspeed_vic.h"
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/intc/arm_gicv3.h"
#include "trace.h"

#define ASPEED_INTC_NR_IRQS 128
#define ASPEED_INTC_SIZE 0x4000
#define TO_REG(N) (N >> 2)

uint64_t regs[ASPEED_INTC_SIZE];

static void aspeed_intc_set_irq(void *opaque, int irq, int level)
{
    // printf("aspeed_intc_set_irq %d %d\n", irq, level);
}

static uint64_t aspeed_intc_read(void *opaque, hwaddr offset, unsigned size)
{
    AspeedINTCState *s = ASPEED_INTC(opaque);
    GICv3State *gic = ARM_GICV3(s->gic);
    // printf("TTTT s=%p gic=%p\n", s, gic);
    uint64_t value = 0;
    switch(TO_REG(offset)) {
    case TO_REG(0x1404):
	// IRQ132
	if (gic && gicv3_gicd_level_test(gic, 164))
	{
	    value = BIT(18);
	}
	break;
    default:
	value = regs[TO_REG(offset)];
	break;
    }

    // printf("aspeed_intc_read(0x%04x) = %08x\n", (uint32_t)offset, (uint32_t)value);
    return value;
}

static void aspeed_intc_write(void *opaque, hwaddr offset, uint64_t data, unsigned size)
{
    AspeedINTCState *s = ASPEED_INTC(opaque);
    GICv3State *gic = ARM_GICV3(s->gic);
    // printf("aspeed_intc_write(0x%04x) = %08x to %08x\n", (uint32_t)offset, (uint32_t)regs[TO_REG(offset)], (uint32_t)data);
    switch(TO_REG(offset)) {
    case TO_REG(0x1400):
	regs[TO_REG(offset)] = data;
        if (regs[TO_REG(offset)])
            gicv3_gicd_enabled_set(gic, 164);
        else
            gicv3_gicd_enabled_clear(gic, 164);
        break;
    case TO_REG(0x1404): // W1C
	regs[TO_REG(offset)] &= ~(data);
	gicv3_gicd_level_clear(gic, 164);
	break;
    default:
	regs[TO_REG(offset)] = data;
	break;
    }

}

static const MemoryRegionOps aspeed_intc_ops = {
    .read = aspeed_intc_read,
    .write = aspeed_intc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .valid.unaligned = false,
};

static void aspeed_intc_realize(DeviceState *dev, Error **errp)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    AspeedINTCState *s = ASPEED_INTC(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &aspeed_intc_ops, s,
                          TYPE_ASPEED_INTC, 0x4000);

    sysbus_init_mmio(sbd, &s->iomem);

    qdev_init_gpio_in(dev, aspeed_intc_set_irq, ASPEED_INTC_NR_IRQS);
    sysbus_init_irq(sbd, &s->irq);
    sysbus_init_irq(sbd, &s->fiq);
}

static void aspeed_intc_reset(DeviceState *dev)
{
    AspeedINTCState *s = ASPEED_INTC(dev);

    s->level = 0;
    s->raw = 0;
    s->select = 0;
    s->enable = 0;
    s->trigger = 0;
    s->sense = 0x1F07FFF8FFFFULL;
    s->dual_edge = 0xF800070000ULL;
    s->event = 0x5F07FFF8FFFFULL;
    // s->gic = NULL;
}

static void aspeed_intc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = aspeed_intc_realize;
    dc->reset = aspeed_intc_reset;
    dc->desc = "ASPEED Interrupt Controller for AST27x0";
    dc->vmsd = NULL;
}

static const TypeInfo aspeed_intc_info = {
    .name = TYPE_ASPEED_INTC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AspeedINTCState),
    .class_init = aspeed_intc_class_init,
};

static void aspeed_intc_register_types(void)
{
    type_register_static(&aspeed_intc_info);
}

type_init(aspeed_intc_register_types);
