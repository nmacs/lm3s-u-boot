#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

#if defined(CONFIG_LM3S)
#include <asm/arch-lm3s/chip.h>
#else
#error No hardware file defined for this configuration
#endif

#endif /* __ASM_ARCH_HARDWARE_H */
