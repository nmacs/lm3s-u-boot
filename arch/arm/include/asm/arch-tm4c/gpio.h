/************************************************************************************
 * arch/arm/src/lm3s/lm3s_gpio.h
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

#ifndef __ARCH_ARM_TM4C_GPIO_H
#define __ARCH_ARM_TM4C_GPIO_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* GPIO Register Offsets ************************************************************/

#define LM3S_GPIO_DATA_OFFSET         0x000 /* GPIO Data */
#define LM3S_GPIO_DIR_OFFSET          0x400 /* GPIO Direction */
#define LM3S_GPIO_IS_OFFSET           0x404 /* GPIO Interrupt Sense */
#define LM3S_GPIO_IBE_OFFSET          0x408 /* GPIO Interrupt Both Edges */
#define LM3S_GPIO_IEV_OFFSET          0x40c /* GPIO Interrupt Event */
#define LM3S_GPIO_IM_OFFSET           0x410 /* GPIO Interrupt Mask */
#define LM3S_GPIO_RIS_OFFSET          0x414 /* GPIO Raw Interrupt Status */
#define LM3S_GPIO_MIS_OFFSET          0x418 /* GPIO Masked Interrupt Status */
#define LM3S_GPIO_ICR_OFFSET          0x41c /* GPIO Interrupt Clear */
#define LM3S_GPIO_AFSEL_OFFSET        0x420 /* GPIO Alternate Function */
#define LM3S_GPIO_DR2R_OFFSET         0x500 /* Select GPIO 2-mA Drive Select */
#define LM3S_GPIO_DR4R_OFFSET         0x504 /* GPIO 4-mA Drive Select */
#define LM3S_GPIO_DR8R_OFFSET         0x508 /* GPIO 8-mA Drive Select */
#define LM3S_GPIO_ODR_OFFSET          0x50c /* GPIO Open Drain Select */
#define LM3S_GPIO_PUR_OFFSET          0x510 /* GPIO Pull-Up Select */
#define LM3S_GPIO_PDR_OFFSET          0x514 /* GPIO Pull-Down Select */
#define LM3S_GPIO_SLR_OFFSET          0x518 /* GPIO Slew Rate Control Select */
#define LM3S_GPIO_DEN_OFFSET          0x51C /* GPIO Digital Enable */
#define LM3S_GPIO_LOCK_OFFSET         0x520 /* GPIO Lock */
#define LM3S_GPIO_CR_OFFSET           0x524 /* GPIO Commit */
#define LM3S_GPIO_AMSEL_OFFSET        0x528 /* GPIO Analog Mode Select */
#define LM3S_GPIO_PCTL_OFFSET         0X52C /* GPIO Port Control */
#define LM3S_GPIO_PERIPHID4_OFFSET    0xfd0 /* GPIO Peripheral Identification 4 */
#define LM3S_GPIO_PERIPHID5_OFFSET    0xfd4 /* GPIO Peripheral Identification 5 */
#define LM3S_GPIO_PERIPHID6_OFFSET    0xfd8 /* GPIO Peripheral Identification 6 */
#define LM3S_GPIO_PERIPHID7_OFFSET    0xfdc /* GPIO Peripheral Identification 7 */
#define LM3S_GPIO_PERIPHID0_OFFSET    0xfe0 /* GPIO Peripheral Identification 0 */
#define LM3S_GPIO_PERIPHID1_OFFSET    0xfe4 /* GPIO Peripheral Identification 1 */
#define LM3S_GPIO_PERIPHID2_OFFSET    0xfe8 /* GPIO Peripheral Identification 2 */
#define LM3S_GPIO_PERIPHID3_OFFSET    0xfec /* GPIO Peripheral Identification 3 */
#define LM3S_GPIO_PCELLID0_OFFSET     0xff0 /* GPIO PrimeCell Identification 0 */
#define LM3S_GPIO_PCELLID1_OFFSET     0xff4 /* GPIO PrimeCell Identification 1 */
#define LM3S_GPIO_PCELLID2_OFFSET     0xff8 /* GPIO PrimeCell Identification 2 */
#define LM3S_GPIO_PCELLID3_OFFSET     0xffc /* GPIO PrimeCell Identification 3*/

/* GPIO Register Addresses **********************************************************/

#define LM3S_GPIOA_DATA               (LM3S_GPIOA_BASE + LM3S_GPIO_DATA_OFFSET)
#define LM3S_GPIOA_DIR                (LM3S_GPIOA_BASE + LM3S_GPIO_DIR_OFFSET)
#define LM3S_GPIOA_IS                 (LM3S_GPIOA_BASE + LM3S_GPIO_IS_OFFSET)
#define LM3S_GPIOA_IBE                (LM3S_GPIOA_BASE + LM3S_GPIO_IBE_OFFSET)
#define LM3S_GPIOA_IEV                (LM3S_GPIOA_BASE + LM3S_GPIO_IEV_OFFSET)
#define LM3S_GPIOA_IM                 (LM3S_GPIOA_BASE + LM3S_GPIO_IM_OFFSET)
#define LM3S_GPIOA_RIS                (LM3S_GPIOA_BASE + LM3S_GPIO_RIS_OFFSET)
#define LM3S_GPIOA_MIS                (LM3S_GPIOA_BASE + LM3S_GPIO_MIS_OFFSET)
#define LM3S_GPIOA_ICR                (LM3S_GPIOA_BASE + LM3S_GPIO_ICR_OFFSET)
#define LM3S_GPIOA_AFSEL              (LM3S_GPIOA_BASE + LM3S_GPIO_AFSEL_OFFSET)
#define LM3S_GPIOA_DR2R               (LM3S_GPIOA_BASE + LM3S_GPIO_DR2R_OFFSET)
#define LM3S_GPIOA_DR4R               (LM3S_GPIOA_BASE + LM3S_GPIO_DR4R_OFFSET)
#define LM3S_GPIOA_DR8R               (LM3S_GPIOA_BASE + LM3S_GPIO_DR8R_OFFSET)
#define LM3S_GPIOA_ODR                (LM3S_GPIOA_BASE + LM3S_GPIO_ODR_OFFSET)
#define LM3S_GPIOA_PUR                (LM3S_GPIOA_BASE + LM3S_GPIO_PUR_OFFSET)
#define LM3S_GPIOA_PDR                (LM3S_GPIOA_BASE + LM3S_GPIO_PDR_OFFSET)
#define LM3S_GPIOA_SLR                (LM3S_GPIOA_BASE + LM3S_GPIO_SLR_OFFSET)
#define LM3S_GPIOA_DEN                (LM3S_GPIOA_BASE + LM3S_GPIO_DEN_OFFSET)
#define LM3S_GPIOA_LOCK               (LM3S_GPIOA_BASE + LM3S_GPIO_LOCK_OFFSET)
#define LM3S_GPIOA_CR                 (LM3S_GPIOA_BASE + LM3S_GPIO_CR_OFFSET)
#define LM3S_GPIOA_PERIPHID4          (LM3S_GPIOA_BASE + LM3S_GPIO_PERIPHID4_OFFSET)
#define LM3S_GPIOA_PERIPHID5          (LM3S_GPIOA_BASE + LM3S_GPIO_PERIPHID5_OFFSET)
#define LM3S_GPIOA_PERIPHID6          (LM3S_GPIOA_BASE + LM3S_GPIO_PERIPHID6_OFFSET)
#define LM3S_GPIOA_PERIPHID7          (LM3S_GPIOA_BASE + LM3S_GPIO_PERIPHID7_OFFSET)
#define LM3S_GPIOA_PERIPHID0          (LM3S_GPIOA_BASE + LM3S_GPIO_PERIPHID0_OFFSET)
#define LM3S_GPIOA_PERIPHID1          (LM3S_GPIOA_BASE + LM3S_GPIO_PERIPHID1_OFFSET)
#define LM3S_GPIOA_PERIPHID2          (LM3S_GPIOA_BASE + LM3S_GPIO_PERIPHID2_OFFSET)
#define LM3S_GPIOA_PERIPHID3          (LM3S_GPIOA_BASE + LM3S_GPIO_PERIPHID3_OFFSET)
#define LM3S_GPIOA_PCELLID0           (LM3S_GPIOA_BASE + LM3S_GPIO_PCELLID0_OFFSET)
#define LM3S_GPIOA_PCELLID1           (LM3S_GPIOA_BASE + LM3S_GPIO_PCELLID1_OFFSET)
#define LM3S_GPIOA_PCELLID2           (LM3S_GPIOA_BASE + LM3S_GPIO_PCELLID2_OFFSET)
#define LM3S_GPIOA_PCELLID3           (LM3S_GPIOA_BASE + LM3S_GPIO_PCELLID3_OFFSET)

#define LM3S_GPIOB_DATA               (LM3S_GPIOB_BASE + LM3S_GPIO_DATA_OFFSET)
#define LM3S_GPIOB_DIR                (LM3S_GPIOB_BASE + LM3S_GPIO_DIR_OFFSET)
#define LM3S_GPIOB_IS                 (LM3S_GPIOB_BASE + LM3S_GPIO_IS_OFFSET)
#define LM3S_GPIOB_IBE                (LM3S_GPIOB_BASE + LM3S_GPIO_IBE_OFFSET)
#define LM3S_GPIOB_IEV                (LM3S_GPIOB_BASE + LM3S_GPIO_IEV_OFFSET)
#define LM3S_GPIOB_IM                 (LM3S_GPIOB_BASE + LM3S_GPIO_IM_OFFSET)
#define LM3S_GPIOB_RIS                (LM3S_GPIOB_BASE + LM3S_GPIO_RIS_OFFSET)
#define LM3S_GPIOB_MIS                (LM3S_GPIOB_BASE + LM3S_GPIO_MIS_OFFSET)
#define LM3S_GPIOB_ICR                (LM3S_GPIOB_BASE + LM3S_GPIO_ICR_OFFSET)
#define LM3S_GPIOB_AFSEL              (LM3S_GPIOB_BASE + LM3S_GPIO_AFSEL_OFFSET)
#define LM3S_GPIOB_DR2R               (LM3S_GPIOB_BASE + LM3S_GPIO_DR2R_OFFSET)
#define LM3S_GPIOB_DR4R               (LM3S_GPIOB_BASE + LM3S_GPIO_DR4R_OFFSET)
#define LM3S_GPIOB_DR8R               (LM3S_GPIOB_BASE + LM3S_GPIO_DR8R_OFFSET)
#define LM3S_GPIOB_ODR                (LM3S_GPIOB_BASE + LM3S_GPIO_ODR_OFFSET)
#define LM3S_GPIOB_PUR                (LM3S_GPIOB_BASE + LM3S_GPIO_PUR_OFFSET)
#define LM3S_GPIOB_PDR                (LM3S_GPIOB_BASE + LM3S_GPIO_PDR_OFFSET)
#define LM3S_GPIOB_SLR                (LM3S_GPIOB_BASE + LM3S_GPIO_SLR_OFFSET)
#define LM3S_GPIOB_DEN                (LM3S_GPIOB_BASE + LM3S_GPIO_DEN_OFFSET)
#define LM3S_GPIOB_LOCK               (LM3S_GPIOB_BASE + LM3S_GPIO_LOCK_OFFSET)
#define LM3S_GPIOB_CR                 (LM3S_GPIOB_BASE + LM3S_GPIO_CR_OFFSET)
#define LM3S_GPIOB_PERIPHID4          (LM3S_GPIOB_BASE + LM3S_GPIO_PERIPHID4_OFFSET)
#define LM3S_GPIOB_PERIPHID5          (LM3S_GPIOB_BASE + LM3S_GPIO_PERIPHID5_OFFSET)
#define LM3S_GPIOB_PERIPHID6          (LM3S_GPIOB_BASE + LM3S_GPIO_PERIPHID6_OFFSET)
#define LM3S_GPIOB_PERIPHID7          (LM3S_GPIOB_BASE + LM3S_GPIO_PERIPHID7_OFFSET)
#define LM3S_GPIOB_PERIPHID0          (LM3S_GPIOB_BASE + LM3S_GPIO_PERIPHID0_OFFSET)
#define LM3S_GPIOB_PERIPHID1          (LM3S_GPIOB_BASE + LM3S_GPIO_PERIPHID1_OFFSET)
#define LM3S_GPIOB_PERIPHID2          (LM3S_GPIOB_BASE + LM3S_GPIO_PERIPHID2_OFFSET)
#define LM3S_GPIOB_PERIPHID3          (LM3S_GPIOB_BASE + LM3S_GPIO_PERIPHID3_OFFSET)
#define LM3S_GPIOB_PCELLID0           (LM3S_GPIOB_BASE + LM3S_GPIO_PCELLID0_OFFSET)
#define LM3S_GPIOB_PCELLID1           (LM3S_GPIOB_BASE + LM3S_GPIO_PCELLID1_OFFSET)
#define LM3S_GPIOB_PCELLID2           (LM3S_GPIOB_BASE + LM3S_GPIO_PCELLID2_OFFSET)
#define LM3S_GPIOB_PCELLID3           (LM3S_GPIOB_BASE + LM3S_GPIO_PCELLID3_OFFSET)

#define LM3S_GPIOC_DATA               (LM3S_GPIOC_BASE + LM3S_GPIO_DATA_OFFSET)
#define LM3S_GPIOC_DIR                (LM3S_GPIOC_BASE + LM3S_GPIO_DIR_OFFSET)
#define LM3S_GPIOC_IS                 (LM3S_GPIOC_BASE + LM3S_GPIO_IS_OFFSET)
#define LM3S_GPIOC_IBE                (LM3S_GPIOC_BASE + LM3S_GPIO_IBE_OFFSET)
#define LM3S_GPIOC_IEV                (LM3S_GPIOC_BASE + LM3S_GPIO_IEV_OFFSET)
#define LM3S_GPIOC_IM                 (LM3S_GPIOC_BASE + LM3S_GPIO_IM_OFFSET)
#define LM3S_GPIOC_RIS                (LM3S_GPIOC_BASE + LM3S_GPIO_RIS_OFFSET)
#define LM3S_GPIOC_MIS                (LM3S_GPIOC_BASE + LM3S_GPIO_MIS_OFFSET)
#define LM3S_GPIOC_ICR                (LM3S_GPIOC_BASE + LM3S_GPIO_ICR_OFFSET)
#define LM3S_GPIOC_AFSEL              (LM3S_GPIOC_BASE + LM3S_GPIO_AFSEL_OFFSET)
#define LM3S_GPIOC_DR2R               (LM3S_GPIOC_BASE + LM3S_GPIO_DR2R_OFFSET)
#define LM3S_GPIOC_DR4R               (LM3S_GPIOC_BASE + LM3S_GPIO_DR4R_OFFSET)
#define LM3S_GPIOC_DR8R               (LM3S_GPIOC_BASE + LM3S_GPIO_DR8R_OFFSET)
#define LM3S_GPIOC_ODR                (LM3S_GPIOC_BASE + LM3S_GPIO_ODR_OFFSET)
#define LM3S_GPIOC_PUR                (LM3S_GPIOC_BASE + LM3S_GPIO_PUR_OFFSET)
#define LM3S_GPIOC_PDR                (LM3S_GPIOC_BASE + LM3S_GPIO_PDR_OFFSET)
#define LM3S_GPIOC_SLR                (LM3S_GPIOC_BASE + LM3S_GPIO_SLR_OFFSET)
#define LM3S_GPIOC_DEN                (LM3S_GPIOC_BASE + LM3S_GPIO_DEN_OFFSET)
#define LM3S_GPIOC_LOCK               (LM3S_GPIOC_BASE + LM3S_GPIO_LOCK_OFFSET)
#define LM3S_GPIOC_CR                 (LM3S_GPIOC_BASE + LM3S_GPIO_CR_OFFSET)
#define LM3S_GPIOC_PERIPHID4          (LM3S_GPIOC_BASE + LM3S_GPIO_PERIPHID4_OFFSET)
#define LM3S_GPIOC_PERIPHID5          (LM3S_GPIOC_BASE + LM3S_GPIO_PERIPHID5_OFFSET)
#define LM3S_GPIOC_PERIPHID6          (LM3S_GPIOC_BASE + LM3S_GPIO_PERIPHID6_OFFSET)
#define LM3S_GPIOC_PERIPHID7          (LM3S_GPIOC_BASE + LM3S_GPIO_PERIPHID7_OFFSET)
#define LM3S_GPIOC_PERIPHID0          (LM3S_GPIOC_BASE + LM3S_GPIO_PERIPHID0_OFFSET)
#define LM3S_GPIOC_PERIPHID1          (LM3S_GPIOC_BASE + LM3S_GPIO_PERIPHID1_OFFSET)
#define LM3S_GPIOC_PERIPHID2          (LM3S_GPIOC_BASE + LM3S_GPIO_PERIPHID2_OFFSET)
#define LM3S_GPIOC_PERIPHID3          (LM3S_GPIOC_BASE + LM3S_GPIO_PERIPHID3_OFFSET)
#define LM3S_GPIOC_PCELLID0           (LM3S_GPIOC_BASE + LM3S_GPIO_PCELLID0_OFFSET)
#define LM3S_GPIOC_PCELLID1           (LM3S_GPIOC_BASE + LM3S_GPIO_PCELLID1_OFFSET)
#define LM3S_GPIOC_PCELLID2           (LM3S_GPIOC_BASE + LM3S_GPIO_PCELLID2_OFFSET)
#define LM3S_GPIOC_PCELLID3           (LM3S_GPIOC_BASE + LM3S_GPIO_PCELLID3_OFFSET)

#define LM3S_GPIOD_DATA               (LM3S_GPIOD_BASE + LM3S_GPIO_DATA_OFFSET)
#define LM3S_GPIOD_DIR                (LM3S_GPIOD_BASE + LM3S_GPIO_DIR_OFFSET)
#define LM3S_GPIOD_IS                 (LM3S_GPIOD_BASE + LM3S_GPIO_IS_OFFSET)
#define LM3S_GPIOD_IBE                (LM3S_GPIOD_BASE + LM3S_GPIO_IBE_OFFSET)
#define LM3S_GPIOD_IEV                (LM3S_GPIOD_BASE + LM3S_GPIO_IEV_OFFSET)
#define LM3S_GPIOD_IM                 (LM3S_GPIOD_BASE + LM3S_GPIO_IM_OFFSET)
#define LM3S_GPIOD_RIS                (LM3S_GPIOD_BASE + LM3S_GPIO_RIS_OFFSET)
#define LM3S_GPIOD_MIS                (LM3S_GPIOD_BASE + LM3S_GPIO_MIS_OFFSET)
#define LM3S_GPIOD_ICR                (LM3S_GPIOD_BASE + LM3S_GPIO_ICR_OFFSET)
#define LM3S_GPIOD_AFSEL              (LM3S_GPIOD_BASE + LM3S_GPIO_AFSEL_OFFSET)
#define LM3S_GPIOD_DR2R               (LM3S_GPIOD_BASE + LM3S_GPIO_DR2R_OFFSET)
#define LM3S_GPIOD_DR4R               (LM3S_GPIOD_BASE + LM3S_GPIO_DR4R_OFFSET)
#define LM3S_GPIOD_DR8R               (LM3S_GPIOD_BASE + LM3S_GPIO_DR8R_OFFSET)
#define LM3S_GPIOD_ODR                (LM3S_GPIOD_BASE + LM3S_GPIO_ODR_OFFSET)
#define LM3S_GPIOD_PUR                (LM3S_GPIOD_BASE + LM3S_GPIO_PUR_OFFSET)
#define LM3S_GPIOD_PDR                (LM3S_GPIOD_BASE + LM3S_GPIO_PDR_OFFSET)
#define LM3S_GPIOD_SLR                (LM3S_GPIOD_BASE + LM3S_GPIO_SLR_OFFSET)
#define LM3S_GPIOD_DEN                (LM3S_GPIOD_BASE + LM3S_GPIO_DEN_OFFSET)
#define LM3S_GPIOD_LOCK               (LM3S_GPIOD_BASE + LM3S_GPIO_LOCK_OFFSET)
#define LM3S_GPIOD_CR                 (LM3S_GPIOD_BASE + LM3S_GPIO_CR_OFFSET)
#define LM3S_GPIOD_PERIPHID4          (LM3S_GPIOD_BASE + LM3S_GPIO_PERIPHID4_OFFSET)
#define LM3S_GPIOD_PERIPHID5          (LM3S_GPIOD_BASE + LM3S_GPIO_PERIPHID5_OFFSET)
#define LM3S_GPIOD_PERIPHID6          (LM3S_GPIOD_BASE + LM3S_GPIO_PERIPHID6_OFFSET)
#define LM3S_GPIOD_PERIPHID7          (LM3S_GPIOD_BASE + LM3S_GPIO_PERIPHID7_OFFSET)
#define LM3S_GPIOD_PERIPHID0          (LM3S_GPIOD_BASE + LM3S_GPIO_PERIPHID0_OFFSET)
#define LM3S_GPIOD_PERIPHID1          (LM3S_GPIOD_BASE + LM3S_GPIO_PERIPHID1_OFFSET)
#define LM3S_GPIOD_PERIPHID2          (LM3S_GPIOD_BASE + LM3S_GPIO_PERIPHID2_OFFSET)
#define LM3S_GPIOD_PERIPHID3          (LM3S_GPIOD_BASE + LM3S_GPIO_PERIPHID3_OFFSET)
#define LM3S_GPIOD_PCELLID0           (LM3S_GPIOD_BASE + LM3S_GPIO_PCELLID0_OFFSET)
#define LM3S_GPIOD_PCELLID1           (LM3S_GPIOD_BASE + LM3S_GPIO_PCELLID1_OFFSET)
#define LM3S_GPIOD_PCELLID2           (LM3S_GPIOD_BASE + LM3S_GPIO_PCELLID2_OFFSET)
#define LM3S_GPIOD_PCELLID3           (LM3S_GPIOD_BASE + LM3S_GPIO_PCELLID3_OFFSET)

#define LM3S_GPIOE_DATA               (LM3S_GPIOE_BASE + LM3S_GPIO_DATA_OFFSET)
#define LM3S_GPIOE_DIR                (LM3S_GPIOE_BASE + LM3S_GPIO_DIR_OFFSET)
#define LM3S_GPIOE_IS                 (LM3S_GPIOE_BASE + LM3S_GPIO_IS_OFFSET)
#define LM3S_GPIOE_IBE                (LM3S_GPIOE_BASE + LM3S_GPIO_IBE_OFFSET)
#define LM3S_GPIOE_IEV                (LM3S_GPIOE_BASE + LM3S_GPIO_IEV_OFFSET)
#define LM3S_GPIOE_IM                 (LM3S_GPIOE_BASE + LM3S_GPIO_IM_OFFSET)
#define LM3S_GPIOE_RIS                (LM3S_GPIOE_BASE + LM3S_GPIO_RIS_OFFSET)
#define LM3S_GPIOE_MIS                (LM3S_GPIOE_BASE + LM3S_GPIO_MIS_OFFSET)
#define LM3S_GPIOE_ICR                (LM3S_GPIOE_BASE + LM3S_GPIO_ICR_OFFSET)
#define LM3S_GPIOE_AFSEL              (LM3S_GPIOE_BASE + LM3S_GPIO_AFSEL_OFFSET)
#define LM3S_GPIOE_DR2R               (LM3S_GPIOE_BASE + LM3S_GPIO_DR2R_OFFSET)
#define LM3S_GPIOE_DR4R               (LM3S_GPIOE_BASE + LM3S_GPIO_DR4R_OFFSET)
#define LM3S_GPIOE_DR8R               (LM3S_GPIOE_BASE + LM3S_GPIO_DR8R_OFFSET)
#define LM3S_GPIOE_ODR                (LM3S_GPIOE_BASE + LM3S_GPIO_ODR_OFFSET)
#define LM3S_GPIOE_PUR                (LM3S_GPIOE_BASE + LM3S_GPIO_PUR_OFFSET)
#define LM3S_GPIOE_PDR                (LM3S_GPIOE_BASE + LM3S_GPIO_PDR_OFFSET)
#define LM3S_GPIOE_SLR                (LM3S_GPIOE_BASE + LM3S_GPIO_SLR_OFFSET)
#define LM3S_GPIOE_DEN                (LM3S_GPIOE_BASE + LM3S_GPIO_DEN_OFFSET)
#define LM3S_GPIOE_LOCK               (LM3S_GPIOE_BASE + LM3S_GPIO_LOCK_OFFSET)
#define LM3S_GPIOE_CR                 (LM3S_GPIOE_BASE + LM3S_GPIO_CR_OFFSET)
#define LM3S_GPIOE_PERIPHID4          (LM3S_GPIOE_BASE + LM3S_GPIO_PERIPHID4_OFFSET)
#define LM3S_GPIOE_PERIPHID5          (LM3S_GPIOE_BASE + LM3S_GPIO_PERIPHID5_OFFSET)
#define LM3S_GPIOE_PERIPHID6          (LM3S_GPIOE_BASE + LM3S_GPIO_PERIPHID6_OFFSET)
#define LM3S_GPIOE_PERIPHID7          (LM3S_GPIOE_BASE + LM3S_GPIO_PERIPHID7_OFFSET)
#define LM3S_GPIOE_PERIPHID0          (LM3S_GPIOE_BASE + LM3S_GPIO_PERIPHID0_OFFSET)
#define LM3S_GPIOE_PERIPHID1          (LM3S_GPIOE_BASE + LM3S_GPIO_PERIPHID1_OFFSET)
#define LM3S_GPIOE_PERIPHID2          (LM3S_GPIOE_BASE + LM3S_GPIO_PERIPHID2_OFFSET)
#define LM3S_GPIOE_PERIPHID3          (LM3S_GPIOE_BASE + LM3S_GPIO_PERIPHID3_OFFSET)
#define LM3S_GPIOE_PCELLID0           (LM3S_GPIOE_BASE + LM3S_GPIO_PCELLID0_OFFSET)
#define LM3S_GPIOE_PCELLID1           (LM3S_GPIOE_BASE + LM3S_GPIO_PCELLID1_OFFSET)
#define LM3S_GPIOE_PCELLID2           (LM3S_GPIOE_BASE + LM3S_GPIO_PCELLID2_OFFSET)
#define LM3S_GPIOE_PCELLID3           (LM3S_GPIOE_BASE + LM3S_GPIO_PCELLID3_OFFSET)

#define LM3S_GPIOF_DATA               (LM3S_GPIOF_BASE + LM3S_GPIO_DATA_OFFSET)
#define LM3S_GPIOF_DIR                (LM3S_GPIOF_BASE + LM3S_GPIO_DIR_OFFSET)
#define LM3S_GPIOF_IS                 (LM3S_GPIOF_BASE + LM3S_GPIO_IS_OFFSET)
#define LM3S_GPIOF_IBE                (LM3S_GPIOF_BASE + LM3S_GPIO_IBE_OFFSET)
#define LM3S_GPIOF_IEV                (LM3S_GPIOF_BASE + LM3S_GPIO_IEV_OFFSET)
#define LM3S_GPIOF_IM                 (LM3S_GPIOF_BASE + LM3S_GPIO_IM_OFFSET)
#define LM3S_GPIOF_RIS                (LM3S_GPIOF_BASE + LM3S_GPIO_RIS_OFFSET)
#define LM3S_GPIOF_MIS                (LM3S_GPIOF_BASE + LM3S_GPIO_MIS_OFFSET)
#define LM3S_GPIOF_ICR                (LM3S_GPIOF_BASE + LM3S_GPIO_ICR_OFFSET)
#define LM3S_GPIOF_AFSEL              (LM3S_GPIOF_BASE + LM3S_GPIO_AFSEL_OFFSET)
#define LM3S_GPIOF_DR2R               (LM3S_GPIOF_BASE + LM3S_GPIO_DR2R_OFFSET)
#define LM3S_GPIOF_DR4R               (LM3S_GPIOF_BASE + LM3S_GPIO_DR4R_OFFSET)
#define LM3S_GPIOF_DR8R               (LM3S_GPIOF_BASE + LM3S_GPIO_DR8R_OFFSET)
#define LM3S_GPIOF_ODR                (LM3S_GPIOF_BASE + LM3S_GPIO_ODR_OFFSET)
#define LM3S_GPIOF_PUR                (LM3S_GPIOF_BASE + LM3S_GPIO_PUR_OFFSET)
#define LM3S_GPIOF_PDR                (LM3S_GPIOF_BASE + LM3S_GPIO_PDR_OFFSET)
#define LM3S_GPIOF_SLR                (LM3S_GPIOF_BASE + LM3S_GPIO_SLR_OFFSET)
#define LM3S_GPIOF_DEN                (LM3S_GPIOF_BASE + LM3S_GPIO_DEN_OFFSET)
#define LM3S_GPIOF_LOCK               (LM3S_GPIOF_BASE + LM3S_GPIO_LOCK_OFFSET)
#define LM3S_GPIOF_CR                 (LM3S_GPIOF_BASE + LM3S_GPIO_CR_OFFSET)
#define LM3S_GPIOF_PERIPHID4          (LM3S_GPIOF_BASE + LM3S_GPIO_PERIPHID4_OFFSET)
#define LM3S_GPIOF_PERIPHID5          (LM3S_GPIOF_BASE + LM3S_GPIO_PERIPHID5_OFFSET)
#define LM3S_GPIOF_PERIPHID6          (LM3S_GPIOF_BASE + LM3S_GPIO_PERIPHID6_OFFSET)
#define LM3S_GPIOF_PERIPHID7          (LM3S_GPIOF_BASE + LM3S_GPIO_PERIPHID7_OFFSET)
#define LM3S_GPIOF_PERIPHID0          (LM3S_GPIOF_BASE + LM3S_GPIO_PERIPHID0_OFFSET)
#define LM3S_GPIOF_PERIPHID1          (LM3S_GPIOF_BASE + LM3S_GPIO_PERIPHID1_OFFSET)
#define LM3S_GPIOF_PERIPHID2          (LM3S_GPIOF_BASE + LM3S_GPIO_PERIPHID2_OFFSET)
#define LM3S_GPIOF_PERIPHID3          (LM3S_GPIOF_BASE + LM3S_GPIO_PERIPHID3_OFFSET)
#define LM3S_GPIOF_PCELLID0           (LM3S_GPIOF_BASE + LM3S_GPIO_PCELLID0_OFFSET)
#define LM3S_GPIOF_PCELLID1           (LM3S_GPIOF_BASE + LM3S_GPIO_PCELLID1_OFFSET)
#define LM3S_GPIOF_PCELLID2           (LM3S_GPIOF_BASE + LM3S_GPIO_PCELLID2_OFFSET)
#define LM3S_GPIOF_PCELLID3           (LM3S_GPIOF_BASE + LM3S_GPIO_PCELLID3_OFFSET)

#define LM3S_GPIOG_DATA               (LM3S_GPIOG_BASE + LM3S_GPIO_DATA_OFFSET)
#define LM3S_GPIOG_DIR                (LM3S_GPIOG_BASE + LM3S_GPIO_DIR_OFFSET)
#define LM3S_GPIOG_IS                 (LM3S_GPIOG_BASE + LM3S_GPIO_IS_OFFSET)
#define LM3S_GPIOG_IBE                (LM3S_GPIOG_BASE + LM3S_GPIO_IBE_OFFSET)
#define LM3S_GPIOG_IEV                (LM3S_GPIOG_BASE + LM3S_GPIO_IEV_OFFSET)
#define LM3S_GPIOG_IM                 (LM3S_GPIOG_BASE + LM3S_GPIO_IM_OFFSET)
#define LM3S_GPIOG_RIS                (LM3S_GPIOG_BASE + LM3S_GPIO_RIS_OFFSET)
#define LM3S_GPIOG_MIS                (LM3S_GPIOG_BASE + LM3S_GPIO_MIS_OFFSET)
#define LM3S_GPIOG_ICR                (LM3S_GPIOG_BASE + LM3S_GPIO_ICR_OFFSET)
#define LM3S_GPIOG_AFSEL              (LM3S_GPIOG_BASE + LM3S_GPIO_AFSEL_OFFSET)
#define LM3S_GPIOG_DR2R               (LM3S_GPIOG_BASE + LM3S_GPIO_DR2R_OFFSET)
#define LM3S_GPIOG_DR4R               (LM3S_GPIOG_BASE + LM3S_GPIO_DR4R_OFFSET)
#define LM3S_GPIOG_DR8R               (LM3S_GPIOG_BASE + LM3S_GPIO_DR8R_OFFSET)
#define LM3S_GPIOG_ODR                (LM3S_GPIOG_BASE + LM3S_GPIO_ODR_OFFSET)
#define LM3S_GPIOG_PUR                (LM3S_GPIOG_BASE + LM3S_GPIO_PUR_OFFSET)
#define LM3S_GPIOG_PDR                (LM3S_GPIOG_BASE + LM3S_GPIO_PDR_OFFSET)
#define LM3S_GPIOG_SLR                (LM3S_GPIOG_BASE + LM3S_GPIO_SLR_OFFSET)
#define LM3S_GPIOG_DEN                (LM3S_GPIOG_BASE + LM3S_GPIO_DEN_OFFSET)
#define LM3S_GPIOG_LOCK               (LM3S_GPIOG_BASE + LM3S_GPIO_LOCK_OFFSET)
#define LM3S_GPIOG_CR                 (LM3S_GPIOG_BASE + LM3S_GPIO_CR_OFFSET)
#define LM3S_GPIOG_PERIPHID4          (LM3S_GPIOG_BASE + LM3S_GPIO_PERIPHID4_OFFSET)
#define LM3S_GPIOG_PERIPHID5          (LM3S_GPIOG_BASE + LM3S_GPIO_PERIPHID5_OFFSET)
#define LM3S_GPIOG_PERIPHID6          (LM3S_GPIOG_BASE + LM3S_GPIO_PERIPHID6_OFFSET)
#define LM3S_GPIOG_PERIPHID7          (LM3S_GPIOG_BASE + LM3S_GPIO_PERIPHID7_OFFSET)
#define LM3S_GPIOG_PERIPHID0          (LM3S_GPIOG_BASE + LM3S_GPIO_PERIPHID0_OFFSET)
#define LM3S_GPIOG_PERIPHID1          (LM3S_GPIOG_BASE + LM3S_GPIO_PERIPHID1_OFFSET)
#define LM3S_GPIOG_PERIPHID2          (LM3S_GPIOG_BASE + LM3S_GPIO_PERIPHID2_OFFSET)
#define LM3S_GPIOG_PERIPHID3          (LM3S_GPIOG_BASE + LM3S_GPIO_PERIPHID3_OFFSET)
#define LM3S_GPIOG_PCELLID0           (LM3S_GPIOG_BASE + LM3S_GPIO_PCELLID0_OFFSET)
#define LM3S_GPIOG_PCELLID1           (LM3S_GPIOG_BASE + LM3S_GPIO_PCELLID1_OFFSET)
#define LM3S_GPIOG_PCELLID2           (LM3S_GPIOG_BASE + LM3S_GPIO_PCELLID2_OFFSET)
#define LM3S_GPIOG_PCELLID3           (LM3S_GPIOG_BASE + LM3S_GPIO_PCELLID3_OFFSET)

#define LM3S_GPIOH_DATA               (LM3S_GPIOH_BASE + LM3S_GPIO_DATA_OFFSET)
#define LM3S_GPIOH_DIR                (LM3S_GPIOH_BASE + LM3S_GPIO_DIR_OFFSET)
#define LM3S_GPIOH_IS                 (LM3S_GPIOH_BASE + LM3S_GPIO_IS_OFFSET)
#define LM3S_GPIOH_IBE                (LM3S_GPIOH_BASE + LM3S_GPIO_IBE_OFFSET)
#define LM3S_GPIOH_IEV                (LM3S_GPIOH_BASE + LM3S_GPIO_IEV_OFFSET)
#define LM3S_GPIOH_IM                 (LM3S_GPIOH_BASE + LM3S_GPIO_IM_OFFSET)
#define LM3S_GPIOH_RIS                (LM3S_GPIOH_BASE + LM3S_GPIO_RIS_OFFSET)
#define LM3S_GPIOH_MIS                (LM3S_GPIOH_BASE + LM3S_GPIO_MIS_OFFSET)
#define LM3S_GPIOH_ICR                (LM3S_GPIOH_BASE + LM3S_GPIO_ICR_OFFSET)
#define LM3S_GPIOH_AFSEL              (LM3S_GPIOH_BASE + LM3S_GPIO_AFSEL_OFFSET)
#define LM3S_GPIOH_DR2R               (LM3S_GPIOH_BASE + LM3S_GPIO_DR2R_OFFSET)
#define LM3S_GPIOH_DR4R               (LM3S_GPIOH_BASE + LM3S_GPIO_DR4R_OFFSET)
#define LM3S_GPIOH_DR8R               (LM3S_GPIOH_BASE + LM3S_GPIO_DR8R_OFFSET)
#define LM3S_GPIOH_ODR                (LM3S_GPIOH_BASE + LM3S_GPIO_ODR_OFFSET)
#define LM3S_GPIOH_PUR                (LM3S_GPIOH_BASE + LM3S_GPIO_PUR_OFFSET)
#define LM3S_GPIOH_PDR                (LM3S_GPIOH_BASE + LM3S_GPIO_PDR_OFFSET)
#define LM3S_GPIOH_SLR                (LM3S_GPIOH_BASE + LM3S_GPIO_SLR_OFFSET)
#define LM3S_GPIOH_DEN                (LM3S_GPIOH_BASE + LM3S_GPIO_DEN_OFFSET)
#define LM3S_GPIOH_LOCK               (LM3S_GPIOH_BASE + LM3S_GPIO_LOCK_OFFSET)
#define LM3S_GPIOH_CR                 (LM3S_GPIOH_BASE + LM3S_GPIO_CR_OFFSET)
#define LM3S_GPIOH_PERIPHID4          (LM3S_GPIOH_BASE + LM3S_GPIO_PERIPHID4_OFFSET)
#define LM3S_GPIOH_PERIPHID5          (LM3S_GPIOH_BASE + LM3S_GPIO_PERIPHID5_OFFSET)
#define LM3S_GPIOH_PERIPHID6          (LM3S_GPIOH_BASE + LM3S_GPIO_PERIPHID6_OFFSET)
#define LM3S_GPIOH_PERIPHID7          (LM3S_GPIOH_BASE + LM3S_GPIO_PERIPHID7_OFFSET)
#define LM3S_GPIOH_PERIPHID0          (LM3S_GPIOH_BASE + LM3S_GPIO_PERIPHID0_OFFSET)
#define LM3S_GPIOH_PERIPHID1          (LM3S_GPIOH_BASE + LM3S_GPIO_PERIPHID1_OFFSET)
#define LM3S_GPIOH_PERIPHID2          (LM3S_GPIOH_BASE + LM3S_GPIO_PERIPHID2_OFFSET)
#define LM3S_GPIOH_PERIPHID3          (LM3S_GPIOH_BASE + LM3S_GPIO_PERIPHID3_OFFSET)
#define LM3S_GPIOH_PCELLID0           (LM3S_GPIOH_BASE + LM3S_GPIO_PCELLID0_OFFSET)
#define LM3S_GPIOH_PCELLID1           (LM3S_GPIOH_BASE + LM3S_GPIO_PCELLID1_OFFSET)
#define LM3S_GPIOH_PCELLID2           (LM3S_GPIOH_BASE + LM3S_GPIO_PCELLID2_OFFSET)
#define LM3S_GPIOH_PCELLID3           (LM3S_GPIOH_BASE + LM3S_GPIO_PCELLID3_OFFSET)

#define LM3S_GPIOJ_DATA               (LM3S_GPIOJ_BASE + LM3S_GPIO_DATA_OFFSET)
#define LM3S_GPIOJ_DIR                (LM3S_GPIOJ_BASE + LM3S_GPIO_DIR_OFFSET)
#define LM3S_GPIOJ_IS                 (LM3S_GPIOJ_BASE + LM3S_GPIO_IS_OFFSET)
#define LM3S_GPIOJ_IBE                (LM3S_GPIOJ_BASE + LM3S_GPIO_IBE_OFFSET)
#define LM3S_GPIOJ_IEV                (LM3S_GPIOJ_BASE + LM3S_GPIO_IEV_OFFSET)
#define LM3S_GPIOJ_IM                 (LM3S_GPIOJ_BASE + LM3S_GPIO_IM_OFFSET)
#define LM3S_GPIOJ_RIS                (LM3S_GPIOJ_BASE + LM3S_GPIO_RIS_OFFSET)
#define LM3S_GPIOJ_MIS                (LM3S_GPIOJ_BASE + LM3S_GPIO_MIS_OFFSET)
#define LM3S_GPIOJ_ICR                (LM3S_GPIOJ_BASE + LM3S_GPIO_ICR_OFFSET)
#define LM3S_GPIOJ_AFSEL              (LM3S_GPIOJ_BASE + LM3S_GPIO_AFSEL_OFFSET)
#define LM3S_GPIOJ_DR2R               (LM3S_GPIOJ_BASE + LM3S_GPIO_DR2R_OFFSET)
#define LM3S_GPIOJ_DR4R               (LM3S_GPIOJ_BASE + LM3S_GPIO_DR4R_OFFSET)
#define LM3S_GPIOJ_DR8R               (LM3S_GPIOJ_BASE + LM3S_GPIO_DR8R_OFFSET)
#define LM3S_GPIOJ_ODR                (LM3S_GPIOJ_BASE + LM3S_GPIO_ODR_OFFSET)
#define LM3S_GPIOJ_PUR                (LM3S_GPIOJ_BASE + LM3S_GPIO_PUR_OFFSET)
#define LM3S_GPIOJ_PDR                (LM3S_GPIOJ_BASE + LM3S_GPIO_PDR_OFFSET)
#define LM3S_GPIOJ_SLR                (LM3S_GPIOJ_BASE + LM3S_GPIO_SLR_OFFSET)
#define LM3S_GPIOJ_DEN                (LM3S_GPIOJ_BASE + LM3S_GPIO_DEN_OFFSET)
#define LM3S_GPIOJ_LOCK               (LM3S_GPIOJ_BASE + LM3S_GPIO_LOCK_OFFSET)
#define LM3S_GPIOJ_CR                 (LM3S_GPIOJ_BASE + LM3S_GPIO_CR_OFFSET)
#define LM3S_GPIOJ_PERIPHID4          (LM3S_GPIOJ_BASE + LM3S_GPIO_PERIPHID4_OFFSET)
#define LM3S_GPIOJ_PERIPHID5          (LM3S_GPIOJ_BASE + LM3S_GPIO_PERIPHID5_OFFSET)
#define LM3S_GPIOJ_PERIPHID6          (LM3S_GPIOJ_BASE + LM3S_GPIO_PERIPHID6_OFFSET)
#define LM3S_GPIOJ_PERIPHID7          (LM3S_GPIOJ_BASE + LM3S_GPIO_PERIPHID7_OFFSET)
#define LM3S_GPIOJ_PERIPHID0          (LM3S_GPIOJ_BASE + LM3S_GPIO_PERIPHID0_OFFSET)
#define LM3S_GPIOJ_PERIPHID1          (LM3S_GPIOJ_BASE + LM3S_GPIO_PERIPHID1_OFFSET)
#define LM3S_GPIOJ_PERIPHID2          (LM3S_GPIOJ_BASE + LM3S_GPIO_PERIPHID2_OFFSET)
#define LM3S_GPIOJ_PERIPHID3          (LM3S_GPIOJ_BASE + LM3S_GPIO_PERIPHID3_OFFSET)
#define LM3S_GPIOJ_PCELLID0           (LM3S_GPIOJ_BASE + LM3S_GPIO_PCELLID0_OFFSET)
#define LM3S_GPIOJ_PCELLID1           (LM3S_GPIOJ_BASE + LM3S_GPIO_PCELLID1_OFFSET)
#define LM3S_GPIOJ_PCELLID2           (LM3S_GPIOJ_BASE + LM3S_GPIO_PCELLID2_OFFSET)
#define LM3S_GPIOJ_PCELLID3           (LM3S_GPIOJ_BASE + LM3S_GPIO_PCELLID3_OFFSET)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#include <linux/types.h>

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

/****************************************************************************
 * Name: lm3s_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on bit-encoded description of the pin.
 *
 ****************************************************************************/

int lm3s_configgpio(uint32_t cfgset);

/****************************************************************************
 * Name: lm3s_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ****************************************************************************/

void lm3s_gpiowrite(uint32_t pinset, int value);

/****************************************************************************
 * Name: lm3s_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ****************************************************************************/

int lm3s_gpioread(uint32_t pinset, int value);

void lm3s_gpioirqenable(uint32_t pinset);

#endif

#endif /* __ARCH_ARM_TM4C_GPIO_H */
