/************************************************************************************
 * arch/arm/cpu/cortex-m4/tm4c/syscontrol.h
 *
 *   Copyright (C) 20013 Max Nekludov. All rights reserved.
 *   Author: Max Nekludov <Max.Nekludov@us.elster.com>
 * 
 *   Work based on:
 *   arch/arm/src/lm3s/lm3s_syscontrol.h
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

#ifndef __ARCH_ARM_SRC_TM4C12_SYSCONTROL_H
#define __ARCH_ARM_SRC_TM4C12_SYSCONTROL_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* System Control Register Offsets **************************************************/

#define LM3S_SYSCON_DID0_OFFSET       0x000 /* Device Identification 0 */
#define LM3S_SYSCON_DID1_OFFSET       0x004 /* Device Identification 1 */
#define LM3S_SYSCON_PBORCTL_OFFSET    0x038 /* Brown-Out Reset Control */
#define LM3S_SYSCON_RIS_OFFSET        0x050 /* Raw Interrupt Status */
#define LM3S_SYSCON_IMC_OFFSET        0x054 /* Interrupt Mask Control */
#define LM3S_SYSCON_MISC_OFFSET       0x058 /* Masked Interrupt Status and Clear */
#define LM3S_SYSCON_RESC_OFFSET       0x05C /* Reset Cause */
#define LM3S_SYSCON_PWRTC_OFFSET      0x060 /* Power-Temperature Cause */
#define LM3S_SYSCON_NMIC_OFFSET       0x064 /* NMI Cause Register */
#define LM3S_SYSCON_MOSCCTL_OFFSET    0x07C /* Main Oscillator Control */
#define LM3S_SYSCON_RSCLKCFG_OFFSET   0x0B0 /* Run and Sleep Mode Configuration Register */
#define LM3S_SYSCON_MEMTIM0_OFFSET    0x0C0 /* Memory Timing Parameter Register 0 for Main Flash and EEPROM */
#define LM3S_SYSCON_DSMEMTIM0_OFFSET  0x0C8 /* Deep Sleep Mode Memory Timing Register 0 for Main Flash and EEPROM */
#define LM3S_SYSCON_ALTCLKCFG_OFFSET  0x138 /* Alternate Clock Configuration */
#define LM3S_SYSCON_DSLPCLKCFG_OFFSET 0x144 /* Deep Sleep Clock Configuration Register */
#define LM3S_SYSCON_DIVSCLK_OFFSET    0x148 /* Divisor and Source Clock Configuration */
#define LM3S_SYSCON_PLLFREQ0_OFFSET   0x160 /* PLL Frequency 0 */
#define LM3S_SYSCON_PLLFREQ1_OFFSET   0x164 /* PLL Frequency 1 */
#define LM3S_SYSCON_PLLSTAT_OFFSET    0x168 /* PLL Status */
#define LM3S_SYSCON_SLPPWRCFG_OFFSET  0x188 /* Sleep Power Configuration */
#define LM3S_SYSCON_DSLPPWRCFG_OFFSET 0x18C /* Deep-Sleep Power Configuration */

/* Run Mode Clock Gating Control Register Offsets */
#define LM3S_SYSCON_RCGCWD_OFFSET     0x600 /* Watchdog Timer Run Mode Clock Gating Control */
#define LM3S_SYSCON_RCGCTIMER_OFFSET  0x604 /* 16/32-Bit General-Purpose Timer Run Mode Clock Gating Control */
#define LM3S_SYSCON_RCGCGPIO_OFFSET   0x608 /* General-Purpose Input/Output Run Mode Clock Gating Control */
#define LM3S_SYSCON_RCGCDMA_OFFSET    0x60C /* Micro Direct Memory Access Run Mode Clock Gating */
#define LM3S_SYSCON_RCGCEPI_OFFSET    0x610 /* EPI Run Mode Clock Gating Control */
#define LM3S_SYSCON_RCGCUART_OFFSET   0x618 /* Universal Asynchronous Receiver/Transmitter Run Mode Clock Gating Control */
#define LM3S_SYSCON_RCGCSSI_OFFSET    0x61C /* Synchronous Serial Interface Run Mode Clock Gating Control */

/* Peripheral Ready Register Offsets */
#define LM3S_SYSCON_PRWD_OFFSET       0xA00 /* Watchdog Timer Peripheral Ready */
#define LM3S_SYSCON_PRTIMER_OFFSET    0xA04 /* 16/32-Bit General-Purpose Timer Peripheral Ready */
#define LM3S_SYSCON_PRGPIO_OFFSET     0xA08 /* General-Purpose Input/Output Peripheral Ready */
#define LM3S_SYSCON_PRDMA_OFFSET      0xA0C /* Micro Direct Memory Access Peripheral Ready */
#define LM3S_SYSCON_PREPI_OFFSET      0xA10 /* EPI Peripheral Ready */
#define LM3S_SYSCON_PRUART_OFFSET     0xA18 /* Universal Asynchronous Receiver/Transmitter Peripheral Ready */
#define LM3S_SYSCON_PRSSI_OFFSET      0xA1C /* Synchronous Serial Interface Peripheral Ready */

/* System Control Register Addresses ************************************************/

#define LM3S_SYSCON_DID0              (LM3S_SYSCON_BASE + LM3S_SYSCON_DID0_OFFSET)
#define LM3S_SYSCON_DID1              (LM3S_SYSCON_BASE + LM3S_SYSCON_DID1_OFFSET)

#define LM3S_SYSCON_PBORCTL           (LM3S_SYSCON_BASE + LM3S_SYSCON_PBORCTL_OFFSET)
#define LM3S_SYSCON_LDOPCTL           (LM3S_SYSCON_BASE + LM3S_SYSCON_LDOPCTL_OFFSET)
#define LM3S_SYSCON_SRCR0             (LM3S_SYSCON_BASE + LM3S_SYSCON_SRCR0_OFFSET)
#define LM3S_SYSCON_SRCR1             (LM3S_SYSCON_BASE + LM3S_SYSCON_SRCR1_OFFSET)
#define LM3S_SYSCON_SRCR2             (LM3S_SYSCON_BASE + LM3S_SYSCON_SRCR2_OFFSET)
#define LM3S_SYSCON_RIS               (LM3S_SYSCON_BASE + LM3S_SYSCON_RIS_OFFSET)
#define LM3S_SYSCON_IMC               (LM3S_SYSCON_BASE + LM3S_SYSCON_IMC_OFFSET)
#define LM3S_SYSCON_MISC              (LM3S_SYSCON_BASE + LM3S_SYSCON_MISC_OFFSET)
#define LM3S_SYSCON_RESC              (LM3S_SYSCON_BASE + LM3S_SYSCON_RESC_OFFSET)
#define LM3S_SYSCON_MOSCCTL           (LM3S_SYSCON_BASE + LM3S_SYSCON_MOSCCTL_OFFSET)
#define LM3S_SYSCON_RSCLKCFG          (LM3S_SYSCON_BASE + LM3S_SYSCON_RSCLKCFG_OFFSET)
#define LM3S_SYSCON_MEMTIM0           (LM3S_SYSCON_BASE + LM3S_SYSCON_MEMTIM0_OFFSET)
#define LM3S_SYSCON_ALTCLKCFG         (LM3S_SYSCON_BASE + LM3S_SYSCON_ALTCLKCFG_OFFSET)
#define LM3S_SYSCON_DSLPCLKCFG        (LM3S_SYSCON_BASE + LM3S_SYSCON_DSLPCLKCFG_OFFSET)
#define LM3S_SYSCON_PLLFREQ0          (LM3S_SYSCON_BASE + LM3S_SYSCON_PLLFREQ0_OFFSET)
#define LM3S_SYSCON_PLLFREQ1          (LM3S_SYSCON_BASE + LM3S_SYSCON_PLLFREQ1_OFFSET)
#define LM3S_SYSCON_PLLSTAT           (LM3S_SYSCON_BASE + LM3S_SYSCON_PLLSTAT_OFFSET)

/* Run Mode Clock Gating Control Registers */
#define LM3S_SYSCON_RCGCWD            (LM3S_SYSCON_BASE + LM3S_SYSCON_RCGCWD_OFFSET)
#define LM3S_SYSCON_RCGCTIMER         (LM3S_SYSCON_BASE + LM3S_SYSCON_RCGCTIMER_OFFSET)
#define LM3S_SYSCON_RCGCGPIO          (LM3S_SYSCON_BASE + LM3S_SYSCON_RCGCGPIO_OFFSET)
#define LM3S_SYSCON_RCGCDMA           (LM3S_SYSCON_BASE + LM3S_SYSCON_RCGCDMA_OFFSET)
#define LM3S_SYSCON_RCGCEPI           (LM3S_SYSCON_BASE + LM3S_SYSCON_RCGCEPI_OFFSET)
#define LM3S_SYSCON_RCGCUART          (LM3S_SYSCON_BASE + LM3S_SYSCON_RCGCUART_OFFSET)
#define LM3S_SYSCON_RCGCSSI           (LM3S_SYSCON_BASE + LM3S_SYSCON_RCGCSSI_OFFSET)

/* Peripheral Ready Registers */
#define LM3S_SYSCON_PRWD              (LM3S_SYSCON_BASE + LM3S_SYSCON_PRWD_OFFSET)
#define LM3S_SYSCON_PRTIMER           (LM3S_SYSCON_BASE + LM3S_SYSCON_PRTIMER_OFFSET)
#define LM3S_SYSCON_PRGPIO            (LM3S_SYSCON_BASE + LM3S_SYSCON_PRGPIO_OFFSET)
#define LM3S_SYSCON_PRDMA             (LM3S_SYSCON_BASE + LM3S_SYSCON_PRDMA_OFFSET)
#define LM3S_SYSCON_PREPI             (LM3S_SYSCON_BASE + LM3S_SYSCON_PREPI_OFFSET)
#define LM3S_SYSCON_PRUART            (LM3S_SYSCON_BASE + LM3S_SYSCON_PRUART_OFFSET)
#define LM3S_SYSCON_PRSSI             (LM3S_SYSCON_BASE + LM3S_SYSCON_PRSSI_OFFSET)

/* System Control Register Bit Definitions ******************************************/

/* Device Identification 0 (DID0), offset 0x000 */

#define SYSCON_DID0_MINOR_SHIFT       0         /* Bits 7-0: Minor Revision of the device */
#define SYSCON_DID0_MINOR_MASK        (0xff << SYSCON_DID0_MINOR_SHIFT)
#define SYSCON_DID0_MAJOR_SHIFT       8         /* Bits 15-8: Major Revision of the device */
#define SYSCON_DID0_MAJOR_MASK        (0xff << SYSCON_DID0_MAJOR_SHIFT)
#define SYSCON_DID0_CLASS_SHIFT       16        /* Bits 23-16: Device Class */
#define SYSCON_DID0_CLASS_MASK        (0xff << SYSCON_DID0_CLASS_SHIFT)
#define SYSCON_DID0_VER_SHIFT         28        /* Bits 30-28: DID0 Version */
#define SYSCON_DID0_VER_MASK          (7 << SYSCON_DID0_VER_SHIFT)

/* Device Identification 1 (DID1), offset 0x004 */

#define SYSCON_DID1_QUAL_SHIFT        0         /* Bits 1-0: Qualification Status */
#define SYSCON_DID1_QUAL_MASK         (0x03 << SYSCON_DID1_QUAL_SHIFT)
#define SYSCON_DID1_ROHS              (1 << 2)  /* Bit 2: RoHS-Compliance */
#define SYSCON_DID1_PKG_SHIFT         3 /* Bits 4-3: Package Type */
#define SYSCON_DID1_PKG_MASK          (0x03 << SYSCON_DID1_PKG_SHIFT)
#define SYSCON_DID1_TEMP_SHIFT        5         /* Bits 7-5: Temperature Range */
#define SYSCON_DID1_TEMP_MASK         (0x07 << SYSCON_DID1_TEMP_SHIFT)
#define SYSCON_DID1_PINCOUNT_SHIFT    13        /* Bits 15-13: Package Pin Count */
#define SYSCON_DID1_PINCOUNT_MASK     (0x07 << SYSCON_DID1_PINCOUNT_SHIFT)
#define SYSCON_DID1_PARTNO_SHIFT      16        /* Bits 23-16: Part Number */
#define SYSCON_DID1_PARTNO_MASK       (0xff << SYSCON_DID1_PARTNO_SHIFT)
#define SYSCON_DID1_FAM_SHIFT         24        /* Bits 27-24: Family */
#define SYSCON_DID1_FAM_MASK          (0x0f << SYSCON_DID1_FAM_SHIFT)
#define SYSCON_DID1_VER_SHIFT         28        /* Bits 31-28:  DID1 Version */
#define SYSCON_DID1_VER_MASK          (0x0f << SYSCON_DID1_VER_SHIFT)

/* Brown-Out Reset Control (PBORCTL), offset 0x038 */

#define SYSCON_PBORCTL_BORIOR         (1 << 1)  /* Bit 1: BOR Interrupt or Reset */

/* Raw Interrupt Status (RIS), offset 0x050 */

#define SYSCON_RIS_BORRIS             (1 << 1)  /* Bit 1: Brown-Out Reset Raw Interrupt Status */
#define SYSCON_RIS_PLLLRIS            (1 << 6)  /* Bit 6: PLL Lock Raw Interrupt Status */

/* Interrupt Mask Control (IMC), offset 0x054 */

#define SYSCON_IMC_BORIM              (1 << 1)  /* Bit 1: Brown-Out Reset Interrupt Mask */
#define SYSCON_IMC_PLLLIM             (1 << 6)  /* Bit 6: PLL Lock Interrupt Mask */

/* Masked Interrupt Status and Clear (MISC), offset 0x058 */

#define SYSCON_MISC_BORMIS            (1 << 1)  /* Bit 1: BOR Masked Interrupt Status */
#define SYSCON_MISC_PLLLMIS           (1 << 6)  /* Bit 6: PLL Lock Masked Interrupt Status */

/* Reset Cause (RESC), offset 0x05C */

#define SYSCON_RESC_EXT               (1 << 0)  /* Bit 0: External Reset */
#define SYSCON_RESC_POR               (1 << 1)  /* Bit 1: Power-On Reset */
#define SYSCON_RESC_BOR               (1 << 2)  /* Bit 2: Brown-Out Reset */
#define SYSCON_RESC_WDT               (1 << 3)  /* Bit 3: Watchdog Timer Reset */
#define SYSCON_RESC_SW                (1 << 4)  /* Bit 4: Software Reset */

/* Main Oscillator Control (MOSCCTL), offset 0x07C */

#define SYSCON_MOSCCTL_CVAL           (1 << 0) /* Bit 0: Clock Validation for MOSC */
#define SYSCON_MOSCCTL_MOSCIM         (1 << 1) /* Bit 1: MOSC Failure Action */
#define SYSCON_MOSCCTL_NOXTAL         (1 << 2) /* Bit 2: No Crystal Connected */
#define SYSCON_MOSCCTL_PWRDN          (1 << 3) /* Bit 3: Power Down */
#define SYSCON_MOSCCTL_OSCRNG         (1 << 4) /* Bit 4: Oscillator Range */
#define SYSCON_MOSCCTL_SESRC          (1 << 5) /* Bit 5: Single-Ended Source */

/* Run and Sleep Mode Configuration Register (RSCLKCFG), offset 0x0B0 */

#define SYSCON_RSCLKCFG_PSYSDIV_OFFSET     0 /* PLL System Clock Divisor */
#define SYSCON_RSCLKCFG_PSYSDIV_MASK       (3FF << SYSCON_RSCLKCFG_PSYSDIV_OFFSET)
#define SYSCON_RSCLKCFG_PSYSDIV_SET(value) ((value) << SYSCON_RSCLKCFG_PSYSDIV_OFFSET)
#define SYSCON_RSCLKCFG_OSYSDIV_OFFSET     10 /* Oscillator System Clock Divisor */
#define SYSCON_RSCLKCFG_OSYSDIV_MASK       (3FF << SYSCON_RSCLKCFG_OSYSDIV_OFFSET)
#define SYSCON_RSCLKCFG_OSYSDIV_SET(value) ((value) << SYSCON_RSCLKCFG_OSYSDIV_OFFSET)
#define SYSCON_RSCLKCFG_OSCSRC_OFFSET      20 /* Oscillator Source */
# define SYSCON_RSCLKCFG_OSCSRC_PIOSC      (0x0 << SYSCON_RSCLKCFG_OSCSRC_OFFSET)
# define SYSCON_RSCLKCFG_OSCSRC_LFIOSC     (0x2 << SYSCON_RSCLKCFG_OSCSRC_OFFSET)
# define SYSCON_RSCLKCFG_OSCSRC_MOSC       (0x3 << SYSCON_RSCLKCFG_OSCSRC_OFFSET)
# define SYSCON_RSCLKCFG_OSCSRC_RTCOSC     (0x4 << SYSCON_RSCLKCFG_OSCSRC_OFFSET)
#define SYSCON_RSCLKCFG_PLLSRC_OFFSET      24 /* PLL Source */
# define SYSCON_RSCLKCFG_PLLSRC_PIOSC      (0x0 << SYSCON_RSCLKCFG_PLLSRC_OFFSET)
# define SYSCON_RSCLKCFG_PLLSRC_MOSC       (0x3 << SYSCON_RSCLKCFG_PLLSRC_OFFSET)
#define SYSCON_RSCLKCFG_USEPLL             (1 << 28) /* Use PLL */
#define SYSCON_RSCLKCFG_ACG                (1 << 29) /* Auto Clock Gating */
#define SYSCON_RSCLKCFG_NEWFREQ            (1 << 30) /* New PLLFREQ Accept */
#define SYSCON_RSCLKCFG_MEMTIMU            (1 << 31) /* Memory Timing Register Update */

/* Memory Timing Parameter Register 0 for Main Flash and EEPROM (MEMTIM0), offset 0x0C0 */

#define SYSCON_MEMTIM0_FWS_OFFSET          0 /* Flash Wait State */
#define SYSCON_MEMTIM0_FWS_MASK            (0x0F << SYSCON_MEMTIM0_FWS_OFFSET)
#define SYSCON_MEMTIM0_FWS_SET(value)      ((value) << SYSCON_MEMTIM0_FWS_OFFSET)
#define SYSCON_MEMTIM0_FBCE_OFFSET         5 /* Flash Bank Clock Edge */
#define SYSCON_MEMTIM0_FBCE_MASK           (0x01 << SYSCON_MEMTIM0_FBCE_OFFSET)
#define SYSCON_MEMTIM0_FBCE_RISE           (1 << SYSCON_MEMTIM0_FBCE_OFFSET)
#define SYSCON_MEMTIM0_FBCE_FALL           (0 << SYSCON_MEMTIM0_FBCE_OFFSET)
#define SYSCON_MEMTIM0_FBCHT_OFFSET        6 /* Flash Bank Clock High Time */
#define SYSCON_MEMTIM0_FBCHT_MASK          (0x0F << SYSCON_MEMTIM0_FBCHT_OFFSET)
# define SYSCON_MEMTIM0_FBCHT_HIGH         (0 << SYSCON_MEMTIM0_FBCHT_OFFSET) /* System clock high */
# define SYSCON_MEMTIM0_FBCHT_1_0          (1 << SYSCON_MEMTIM0_FBCHT_OFFSET) /* 1 system clock period */
# define SYSCON_MEMTIM0_FBCHT_1_5          (2 << SYSCON_MEMTIM0_FBCHT_OFFSET) /* 1.5 system clock period */
# define SYSCON_MEMTIM0_FBCHT_2_0          (3 << SYSCON_MEMTIM0_FBCHT_OFFSET) /* 2 system clock period */
# define SYSCON_MEMTIM0_FBCHT_2_5          (4 << SYSCON_MEMTIM0_FBCHT_OFFSET) /* 2.5 system clock period */
# define SYSCON_MEMTIM0_FBCHT_3_0          (5 << SYSCON_MEMTIM0_FBCHT_OFFSET) /* 3 system clock period */
# define SYSCON_MEMTIM0_FBCHT_3_5          (6 << SYSCON_MEMTIM0_FBCHT_OFFSET) /* 3.5 system clock period */
# define SYSCON_MEMTIM0_FBCHT_4_0          (7 << SYSCON_MEMTIM0_FBCHT_OFFSET) /* 4 system clock period */
# define SYSCON_MEMTIM0_FBCHT_4_5          (8 << SYSCON_MEMTIM0_FBCHT_OFFSET) /* 4.5 system clock period */
#define SYSCON_MEMTIM0_EWS_OFFSET          16 /* EEPROM Wait State */
#define SYSCON_MEMTIM0_EWS_MASK            (0x0F << SYSCON_MEMTIM0_EWS_OFFSET)
#define SYSCON_MEMTIM0_EWS_SET(value)      ((value) << SYSCON_MEMTIM0_EWS_OFFSET)
#define SYSCON_MEMTIM0_EBCE_OFFSET         21 /* EEPROM Clock Edge */
#define SYSCON_MEMTIM0_EBCE_MASK           (0x01 << SYSCON_MEMTIM0_EBCE_OFFSET)
#define SYSCON_MEMTIM0_EBCE_RISE           (1 << SYSCON_MEMTIM0_EBCE_OFFSET)
#define SYSCON_MEMTIM0_EBCE_FALL           (0 << SYSCON_MEMTIM0_EBCE_OFFSET)
#define SYSCON_MEMTIM0_EBCHT_OFFSET        22 /* EEPROM Clock High Time */
#define SYSCON_MEMTIM0_EBCHT_MASK          (0x0F << SYSCON_MEMTIM0_EBCHT_OFFSET)
# define SYSCON_MEMTIM0_EBCHT_HIGH         (0 << SYSCON_MEMTIM0_EBCHT_OFFSET) /* System clock high */
# define SYSCON_MEMTIM0_EBCHT_1_0          (1 << SYSCON_MEMTIM0_EBCHT_OFFSET) /* 1 system clock period */
# define SYSCON_MEMTIM0_EBCHT_1_5          (2 << SYSCON_MEMTIM0_EBCHT_OFFSET) /* 1.5 system clock period */
# define SYSCON_MEMTIM0_EBCHT_2_0          (3 << SYSCON_MEMTIM0_EBCHT_OFFSET) /* 2 system clock period */
# define SYSCON_MEMTIM0_EBCHT_2_5          (4 << SYSCON_MEMTIM0_EBCHT_OFFSET) /* 2.5 system clock period */
# define SYSCON_MEMTIM0_EBCHT_3_0          (5 << SYSCON_MEMTIM0_EBCHT_OFFSET) /* 3 system clock period */
# define SYSCON_MEMTIM0_EBCHT_3_5          (6 << SYSCON_MEMTIM0_EBCHT_OFFSET) /* 3.5 system clock period */
# define SYSCON_MEMTIM0_EBCHT_4_0          (7 << SYSCON_MEMTIM0_EBCHT_OFFSET) /* 4 system clock period */
# define SYSCON_MEMTIM0_EBCHT_4_5          (8 << SYSCON_MEMTIM0_EBCHT_OFFSET) /* 4.5 system clock period */

/* Deep Sleep Clock Configuration (DSLPCLKCFG), offset 0x144 */

#define SYSCON_DSLPCLKCFG_DSDIVORIDE_SHIFT 23 /* Bits 28-23: Divider Field Override */
#define SYSCON_DSLPCLKCFG_DSDIVORIDE_MASK  (0x3f << SYSCON_DSLPCLKCFG_DSDIVORIDE_SHIFT)
#  define SYSCON_DSLPCLKCFG_DSDIVORIDE(n)  ((n) << SYSCON_DSLPCLKCFG_DSDIVORIDE_SHIFT)
#define SYSCON_DSLPCLKCFG_DSOSCSRC_SHIFT   4 /* Bits 6-4: Clock Source */
#define SYSCON_DSLPCLKCFG_DSOSCSRC_MASK    (0x07 << SYSCON_DSLPCLKCFG_DSOSCSRC_SHIFT)
#  define SYSCON_DSLPCLKCFG_DSOSCSRC_MOSC  (0 << SYSCON_DSLPCLKCFG_DSOSCSRC_SHIFT)
#  define SYSCON_DSLPCLKCFG_DSOSCSRC_PIOSC (1 << SYSCON_DSLPCLKCFG_DSOSCSRC_SHIFT)
#  define SYSCON_DSLPCLKCFG_DSOSCSRC_30KHZ (3 << SYSCON_DSLPCLKCFG_DSOSCSRC_SHIFT)
#  define SYSCON_DSLPCLKCFG_DSOSCSRC_32KHZ (7 << SYSCON_DSLPCLKCFG_DSOSCSRC_SHIFT)

/* PLL Frequency 0 (PLLFREQ0), offset 0x160 */

#define SYSCON_PLLFREQ0_MINT_OFFSET      0 /* PLL M Integer Value */
#define SYSCON_PLLFREQ0_MINT_MASK        (3FF << SYSCON_PLLFREQ0_MINT_OFFSET)
#define SYSCON_PLLFREQ0_MINT_SET(value)  ((value) << SYSCON_PLLFREQ0_MINT_OFFSET)
#define SYSCON_PLLFREQ0_MFRAC_OFFSET     10 /* PLL M Fractional Value */
#define SYSCON_PLLFREQ0_MFRAC_MASK       (3FF << SYSCON_PLLFREQ0_MFRAC_OFFSET)
#define SYSCON_PLLFREQ0_MFRAC_SET(value) ((value) << SYSCON_PLLFREQ0_MFRAC_OFFSET)
#define SYSCON_PLLFREQ0_PLLPWR           (1 << 23) /* PLL Power */

/* PLL Frequency 1 (PLLFREQ1), offset 0x164 */

#define SYSCON_PLLFREQ1_N_OFFSET         0 /* PLL N Value */
#define SYSCON_PLLFREQ1_N_MASK           (1F << SYSCON_PLLFREQ1_N_OFFSET)
#define SYSCON_PLLFREQ1_N_SET(value)     ((value) << SYSCON_PLLFREQ1_N_OFFSET)
#define SYSCON_PLLFREQ1_Q_OFFSET         8 /* PLL Q Value */
#define SYSCON_PLLFREQ1_Q_MASK           (1F << SYSCON_PLLFREQ1_Q_OFFSET)
#define SYSCON_PLLFREQ1_Q_SET(value)     ((value) << SYSCON_PLLFREQ1_Q_OFFSET)

/* EPI Run Mode Clock Gating Control (RCGCEPI), offset 0x610 */

#define SYSCON_RCGCEPI_ENABLE            1
#define SYSCON_RCGCEPI_DISABLE           0

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Types
 ************************************************************************************/

#define SYS_ENABLE_CLOCK  1
#define SYS_DISABLE_CLOCK 0

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

void gpio_clock_ctrl(int port, int ctrl);
void uart_clock_ctrl(int port, int ctrl);
void dma_clock_ctrl(int ctrl);
void ssi_clock_ctrl(int module, int ctrl);
void epi_clock_ctrl(int ctrl);
void watchdog_clock_ctrl(int module, int ctrl);
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_LM3S_TM4C12_SYSCONTROL_H */
