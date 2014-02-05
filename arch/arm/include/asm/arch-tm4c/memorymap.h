/************************************************************************************
 * arch/arm/src/lm3s/lm3s_memorymap.h
 *
 *   Copyright (C) 2009-2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_LM3S_LM3S_MEMORYMAP_H
#define __ARCH_ARM_SRC_LM3S_LM3S_MEMORYMAP_H

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Memory map ***********************************************************************/

#if defined(CONFIG_ARCH_TM4C12)
#  define LM3S_FLASH_BASE     0x00000000 /* -0x0003ffff: On-chip FLASH */
                                         /* -0x1fffffff: Reserved */
#  define LM3S_SRAM_BASE      0x20000000 /* -0x2000ffff: Bit-banded on-chip SRAM */
                                         /* -0x21ffffff: Reserved */
#  define LM3S_ASRAM_BASE     0x22000000 /* -0x221fffff: Bit-band alias of 20000000- */
                                         /* -0x3fffffff: Reserved */
#  define LM3S_PERIPH_BASE    0x40000000 /* -0x4001ffff: FiRM Peripherals */
                                         /* -0x41ffffff: Peripherals */
#  define LM3S_APERIPH_BASE   0x42000000 /* -0x43ffffff: Bit-band alise of 40000000- */
                                         /* -0xdfffffff: Reserved */
#  define LM3S_EPI0RAM_BASE   0x60000000 /* -0xDfffffff: EPI0 mapped peripheral and RAM */
#  define LM3S_ITM_BASE       0xe0000000 /* -0xe0000fff: Instrumentation Trace Macrocell */
#  define LM3S_DWT_BASE       0xe0001000 /* -0xe0001fff: Data Watchpoint and Trace */
#  define LM3S_FPB_BASE       0xe0002000 /* -0xe0002fff: Flash Patch and Breakpoint */
                                         /* -0xe000dfff: Reserved */
#  define LM3S_CM3P_BASE      0xE000E000 /*            : Cortex-M3 Peripherals */
#  define LM3S_NVIC_BASE      0xe000e000 /* -0xe000efff: Nested Vectored Interrupt Controller */
                                         /* -0xe003ffff: Reserved */
#  define LM3S_TPIU_BASE      0xe0040000 /* -0xe0040fff: Trace Port Interface Unit */
                                         /* -0xffffffff: Reserved */
#else
#  error "Memory map not specified for this TM4C chip"
#endif

/* Peripheral base addresses ********************************************************/

#if defined(CONFIG_ARCH_TM4C12)

#  define LM3S_GPIOA_BASE     (LM3S_PERIPH_BASE + 0x58000) /* GPIO Port A */
#  define LM3S_GPIOB_BASE     (LM3S_PERIPH_BASE + 0x59000) /* GPIO Port B */
#  define LM3S_GPIOC_BASE     (LM3S_PERIPH_BASE + 0x5A000) /* GPIO Port C */
#  define LM3S_GPIOD_BASE     (LM3S_PERIPH_BASE + 0x5B000) /* GPIO Port D */
#  define LM3S_GPIOE_BASE     (LM3S_PERIPH_BASE + 0x5C000) /* GPIO Port E */
#  define LM3S_GPIOF_BASE     (LM3S_PERIPH_BASE + 0x5D000) /* GPIO Port F */
#  define LM3S_GPIOG_BASE     (LM3S_PERIPH_BASE + 0x5E000) /* GPIO Port G */
#  define LM3S_GPIOH_BASE     (LM3S_PERIPH_BASE + 0x5F000) /* GPIO Port H */
#  define LM3S_GPIOK_BASE     (LM3S_PERIPH_BASE + 0x61000) /* GPIO Port K */
#  define LM3S_GPIOL_BASE     (LM3S_PERIPH_BASE + 0x62000) /* GPIO Port L */
#  define LM3S_GPIOM_BASE     (LM3S_PERIPH_BASE + 0x63000) /* GPIO Port M */
#  define LM3S_GPIOP_BASE     (LM3S_PERIPH_BASE + 0x65000) /* GPIO Port P */

#  define LM3S_WDOG0_BASE     (LM3S_PERIPH_BASE + 0x00000) /* Watchdog Timer 0 */
#  define LM3S_WDOG1_BASE     (LM3S_PERIPH_BASE + 0x01000) /* Watchdog Timer 1 */
#  define LM3S_WDOG_BASE(n)   (LM3S_WDOG0_BASE + (n) * 0x1000)

#  define LM3S_SSI0_BASE      (LM3S_PERIPH_BASE + 0x08000) /* SSI0 */
#  define LM3S_SSI1_BASE      (LM3S_PERIPH_BASE + 0x09000) /* SSI1 */
#  define LM3S_SSI2_BASE      (LM3S_PERIPH_BASE + 0x0A000) /* SSI2 */
#  define LM3S_SSI3_BASE      (LM3S_PERIPH_BASE + 0x0B000) /* SSI3 */

#  define LM3S_UART0_BASE     (LM3S_PERIPH_BASE + 0x0C000) /* UART0 */
#  define LM3S_UART1_BASE     (LM3S_PERIPH_BASE + 0x0D000) /* UART1 */
#  define LM3S_UART2_BASE     (LM3S_PERIPH_BASE + 0x0E000) /* UART2 */
#  define LM3S_UART3_BASE     (LM3S_PERIPH_BASE + 0x0F000) /* UART3 */
#  define LM3S_UART4_BASE     (LM3S_PERIPH_BASE + 0x10000) /* UART4 */
#  define LM3S_UART5_BASE     (LM3S_PERIPH_BASE + 0x11000) /* UART5 */
#  define LM3S_UART6_BASE     (LM3S_PERIPH_BASE + 0x12000) /* UART6 */
#  define LM3S_UART7_BASE     (LM3S_PERIPH_BASE + 0x13000) /* UART7 */

#  define LM3S_TIMER0_BASE    (LM3S_PERIPH_BASE + 0x30000)  /* Timer 0 */
#  define LM3S_TIMER1_BASE    (LM3S_PERIPH_BASE + 0x31000)  /* Timer 1 */
#  define LM3S_TIMER2_BASE    (LM3S_PERIPH_BASE + 0x32000)  /* Timer 2 */
#  define LM3S_TIMER3_BASE    (LM3S_PERIPH_BASE + 0x33000)  /* Timer 3 */
#  define LM3S_TIMER4_BASE    (LM3S_PERIPH_BASE + 0x34000)  /* Timer 4 */
#  define LM3S_TIMER5_BASE    (LM3S_PERIPH_BASE + 0x35000)  /* Timer 5 */
#  define LM3S_TIMER6_BASE    (LM3S_PERIPH_BASE + 0xE0000)  /* Timer 6 */
#  define LM3S_TIMER7_BASE    (LM3S_PERIPH_BASE + 0xE1000)  /* Timer 7 */

#  define LM3S_ADC0_BASE      (LM3S_PERIPH_BASE + 0x38000)  /* ADC 0 */
#  define LM3S_ADC1_BASE      (LM3S_PERIPH_BASE + 0x39000)  /* ADC 1 */

#  define LM3S_EPI0_BASE      (LM3S_PERIPH_BASE + 0xD0000)  /* EPI 0 */

#  define LM3S_FLASHCON_BASE  (LM3S_PERIPH_BASE + 0xFD000)  /* FLASH Control */
#  define LM3S_EEPROMCON_BASE (LM3S_PERIPH_BASE + 0xAF000)  /* EEPROM Control */
#  define LM3S_ROMCON_BASE    (LM3S_PERIPH_BASE + 0xFE000)  /* ROM Control */
#  define LM3S_SYSCON_BASE    (LM3S_PERIPH_BASE + 0xFE000)  /* System Control */

#  define STLR_UDMA_BASE      (LM3S_PERIPH_BASE + 0xFF000)  /* uDMA */

#else
#  error "Peripheral base addresses not specified for this LM3S chip"
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LM3S_LM3S_MEMORYMAP_H */
