#ifndef ASPEED_SLI_H
#define ASPEED_SLI_H

#include "hw/sysbus.h"

#define TYPE_ASPEED_SLI "aspeed.sli"
// #define TYPE_ASPEED_AST2700_SLI TYPE_ASPEED_SLI "-ast2700"
OBJECT_DECLARE_TYPE(AspeedSLIState, AspeedSLIClass, ASPEED_SLI)

#define ASPEED_SLI_NR_REGS  (4096 >> 2)

struct AspeedSLIState {
    SysBusDevice parent;
    MemoryRegion iomem;

    uint32_t regs[ASPEED_SLI_NR_REGS];
};

struct AspeedSLIClass {
    SysBusDeviceClass parent_class;
};

#endif /* ASPEED_SLI_H */
