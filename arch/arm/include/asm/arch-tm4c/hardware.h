#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

#if defined(CONFIG_TM4C)
#include <asm/arch-tm4c/chip.h>
#include <asm/arch-tm4c/internal.h>
#else
#error No hardware file defined for this configuration
#endif

#endif /* __ASM_ARCH_HARDWARE_H */
