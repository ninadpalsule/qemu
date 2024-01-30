
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "hw/qdev-properties.h"
#include "hw/misc/aspeed_sli.h"
#include "qapi/error.h"
#include "migration/vmstate.h"


static uint64_t aspeed_sli_read(void *opaque, hwaddr addr, unsigned int size)
{
    AspeedSLIState *s = ASPEED_SLI(opaque);

    addr >>= 2;

    if (addr >= ASPEED_SLI_NR_REGS) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Out-of-bounds read at offset 0x%" HWADDR_PRIx "\n",
                      __func__, addr << 2);
        return 0;
    }

    return s->regs[addr];
}

static void aspeed_sli_write(void *opaque, hwaddr addr, uint64_t data,
                              unsigned int size)
{
    AspeedSLIState *s = ASPEED_SLI(opaque);

    addr >>= 2;

    if (addr >= ASPEED_SLI_NR_REGS) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Out-of-bounds write at offset 0x%" HWADDR_PRIx "\n",
                      __func__, addr << 2);
        return;
    }

    switch (addr) {
    default:
        break;
    }

    s->regs[addr] = data;
}

static const MemoryRegionOps aspeed_sli_ops = {
    .read = aspeed_sli_read,
    .write = aspeed_sli_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static void aspeed_ast2700_sli_realize(DeviceState *dev, Error **errp)
{
    AspeedSLIState *s = ASPEED_SLI(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &aspeed_sli_ops, s, TYPE_ASPEED_SLI, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
}

static void aspeed_ast2700_sli_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->desc = "AST2700 SLI Controller";
    dc->realize = aspeed_ast2700_sli_realize;
}

static const TypeInfo aspeed_ast2700_sli_info = {
    .name = TYPE_ASPEED_SLI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .class_init = aspeed_ast2700_sli_class_init,
};

static void aspeed_sli_register_types(void)
{
    type_register_static(&aspeed_ast2700_sli_info);
}

type_init(aspeed_sli_register_types);
