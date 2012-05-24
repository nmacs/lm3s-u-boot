/****************************************************************************
 * drivers/spi/lm3s_spi.c
 *
 *   Copyright (C) 2009-2012 Max Nekludov. All rights reserved.
 *   Author: Max Nekludov <macscomp@gmail.com>
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
 ****************************************************************************/

#include <common.h>
#include <malloc.h>
#include <spi.h>
#include <stdint.h>
#include <asm/arch/lm3s_internal.h>
#include <asm/arch/hardware.h>

struct lm3s_spi_slave {
  struct spi_slave slave;
  unsigned int mode;
  uint32_t cpsdvsr;
  uint32_t cr0;
  uint32_t cr1;
};

#define to_lm3s_spi_slave(s) container_of(s, struct lm3s_spi_slave, slave)
#define SSI_BASE LM3S_SSI0_BASE

/****************************************************************************
 * Name: ssi_getreg
 *
 * Description:
 *   Read the SSI register at this offeset
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *   offset - Offset to the SSI register from the register base address
 *
 * Returned Value:
 *   Value of the register at this offset
 *
 ****************************************************************************/

static inline uint32_t ssi_getreg(unsigned int offset)
{
  return getreg32(SSI_BASE + offset);
}

/****************************************************************************
 * Name: ssi_putreg
 *
 * Description:
 *   Write the value to the SSI register at this offeset
 *
 * Input Parameters:
 *   offset - Offset to the SSI register from the register base address
 *   value  - Value to write
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void ssi_putreg(unsigned int offset, uint32_t value)
{
  putreg32(value, SSI_BASE + offset);
}

/****************************************************************************
 * Name: ssi_setmode
 *
 * Description:
 *   Set the SPI mode. Optional.  See enum spi_mode_e for mode definitions
 *
 * Input Parameters:
 *   mode - The SPI mode requested
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void ssi_setmode(struct lm3s_spi_slave *priv, unsigned int mode)
{
  uint32_t modebits;
  uint32_t regval;

  switch (mode)
  {
  case SPI_MODE_0: /* CPOL=0 CHPHA=0 */
    modebits = 0;
    break;

  case SPI_MODE_1: /* CPOL=0 CHPHA=1 */
    modebits = SSI_CR0_SPH;
    break;

  case SPI_MODE_2: /* CPOL=1 CHPHA=0 */
    modebits = SSI_CR0_SPO;
   break;

  case SPI_MODE_3: /* CPOL=1 CHPHA=1 */
    modebits = SSI_CR0_SPH|SSI_CR0_SPO;
    break;

  default:
    return;
  }

  /* Then set the selected mode: Freescale SPI format, mode0-3 */

  regval  = priv->cr0;
  regval &= ~(SSI_CR0_FRF_MASK|SSI_CR0_SPH|SSI_CR0_SPO);
  regval |= modebits;
  priv->cr0 = regval;
}

/****************************************************************************
 * Name: ssi_setbits
 *
 * Description:
 *   Set the number if bits per word.
 *
 * Input Parameters:
 *   nbits - The number of bits requests
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

static void ssi_setbits(struct lm3s_spi_slave *priv, int nbits)
{
  uint32_t regval;

  if (nbits >=4 && nbits <= 16)
  {
    regval  = priv->cr0;
    regval &= ~SSI_CR0_DSS_MASK;
    regval |= ((nbits - 1) << SSI_CR0_DSS_SHIFT);
    priv->cr0 = regval;
  }
}

/****************************************************************************
 * Name: ssi_calculatefrequency
 *
 * Description:
 *   Calculate the SPI frequency.
 *
 * Input Parameters:
 *   priv -      Device-specific state data
 *   frequency - The SPI frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

static void ssi_setfrequency(struct lm3s_spi_slave *priv, uint32_t frequency)
{
  uint32_t maxdvsr;
  uint32_t cpsdvsr;
  uint32_t scr;
  uint32_t regval;

  /* "The serial bit rate is derived by dividing down the input clock
   *  (FSysClk). The clock is first divided by an even prescale value
   *  CPSDVSR from 2 to 254, which is programmed in the SSI Clock Prescale
   *  (SSI_CPSR) register ... The clock is further divided by a value
   *  from 1 to 256, which is 1 + SCR, where SCR is the value programmed
   *  i n the SSI Control0 (SSICR0) register ...
   *
   * "The frequency of the output clock SSIClk is defined by:
   *
   *    "SSIClk = FSysClk / (CPSDVSR * (1 + SCR))
   *
   * "Note: Although the SSIClk transmit clock can theoretically be 25 MHz,
   *  the module may not be able to operate at that speed. For master mode,
   *  the system clock must be at least two times faster than the SSIClk.
   *  For slave mode, the system clock must be at least 12 times faster
   *  than the SSIClk."
   */

  if (frequency > SYSCLK_FREQUENCY/2)
  {
    frequency = SYSCLK_FREQUENCY/2;
  }

  /* Find optimal values for CPSDVSR and SCR.  This loop is inefficient,
   * but should not have to execute many times.
   *
   * EXAMPLE 1: SYSCLK_FREQUENCY=50,000,0000 and frequency=400,000.
   *
   *   maxcvsr = 125
   *   1. cpsdvsr = 2, scr = 61 -> DONE
   *
   *   This would correspond to an actual frequency of:
   *   50,000,000 / (2 * (62)) = 403,226
   *
   * EXAMPLE 2: SYSCLK_FREQUENCY=50,000,0000 and frequency=25,000,000.
   *
   *   maxcvsr = 2
   *   1. cpsdvsr = 2, scr = 0 -> DONE
   *
   *   This would correspond to an actual frequency of:
   *   50,000,000 / (2 * (1)) = 25,000,000
   */

  maxdvsr = SYSCLK_FREQUENCY / frequency;
  cpsdvsr = 0;
  do
  {
    cpsdvsr += 2;
    scr = (maxdvsr / cpsdvsr) - 1;
  }
  while (scr > 255);

  priv->cpsdvsr = cpsdvsr;

  regval = priv->cr0;
  regval &= ~SSI_CR0_SCR_MASK;
  regval |= (scr << SSI_CR0_SCR_SHIFT);
  priv->cr0 = regval;

  /* Calcluate the actual frequency */
  //actual = SYSCLK_FREQUENCY / (cpsdvsr * (scr + 1));
}

/****************************************************************************
 * Name: ssi_disable
 *
 * Description:
 *   Disable SSI operation.  NOTE: The SSI must be disabled before any control
 *   registers can be re-programmed.
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *
 * Returned Value:
 *   State of the SSI before the SSE was disabled
 *
 * Assumption:
 *   Caller holds a lock on the SPI bus (if CONFIG_SPI_OWNBUS not defined)
 *
 ****************************************************************************/

static void ssi_disable()
{
  uint32_t regval;

  regval = ssi_getreg(LM3S_SSI_CR1_OFFSET);
  regval &= ~SSI_CR1_SSE;
  ssi_putreg(priv, LM3S_SSI_CR1_OFFSET, regval);
}

/****************************************************************************
 * Name: ssi_enable
 *
 * Description:
 *   Restore the SSI operational state
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *   enable - The previous operational state
 *
 * Returned Value:
 *
 * Assumption:
 *   Caller holds a lock on the SPI bus (if CONFIG_SPI_OWNBUS not defined)
 *
 ****************************************************************************/

static void ssi_enable()
{
  uint32_t regval = ssi_getreg(LM3S_SSI_CR1_OFFSET);
  regval  |= SSI_CR1_SSE;
  ssi_putreg(LM3S_SSI_CR1_OFFSET, regval);
}

int spi_cs_is_valid(unsigned int bus, unsigned int cs)
{
  return bus == 0 && cs != 0;
}

void spi_cs_activate(struct spi_slave *slave)
{
  lm3s_gpiowrite(slave->cs, 0);
  debug("%s: SPI_CS_GPIO:%x\n", __func__, slave->cs);
}

void spi_cs_deactivate(struct spi_slave *slave)
{
  lm3s_gpiowrite(slave->cs, 1);
  debug("%s: SPI_CS_GPIO:%x\n", __func__, slave->cs);
}

void spi_init()
{
  uint32_t regval;

  regval = getreg32(LM3S_SYSCON_RCGC1);
  regval |= SYSCON_RCGC1_SSI0;
  putreg32(regval, LM3S_SYSCON_RCGC1);

  /* Set all CR1 fields to reset state.  This will be master mode. */

  ssi_putreg(LM3S_SSI_CR1_OFFSET, 0);

  /* Set all CR0 fields to the reset state. This will also select Freescale SPI mode. */

  ssi_putreg(LM3S_SSI_CR0_OFFSET, 0);

  /* Disable all SSI interrupt sources. */

  ssi_putreg(LM3S_SSI_IM_OFFSET, 0);
}

struct spi_slave *spi_setup_slave(unsigned int bus, unsigned int cs,
                unsigned int max_hz, unsigned int mode)
{
  struct lm3s_spi_slave *lss;
  lss = (struct lm3s_spi_slave *)malloc(sizeof(*lss));
  if (!lss)
    return 0;

  memset(lss, 0, sizeof(*lss));

  lss->slave.bus = bus;
  lss->slave.cs = cs;

  ssi_setmode(lss, mode);

  ssi_setbits(lss, 8);

  ssi_setfrequency(lss, 400000);

  return &lss->slave;
}

void spi_free_slave(struct spi_slave *slave)
{
  struct lm3s_spi_slave *lss = to_lm3s_spi_slave(slave);
  free(lss);
}

int spi_claim_bus(struct spi_slave *slave)
{
  struct lm3s_spi_slave *lss = to_lm3s_spi_slave(slave);

  debug("%s: bus:%i cs:%i\n", __func__, slave->bus, slave->cs);

  /* Set CPDVSR */

  ssi_putreg(priv, LM3S_SSI_CPSR_OFFSET, lss->cpsdvsr);

  /* Set CR0 and CR1 */

  ssi_putreg(LM3S_SSI_CR0_OFFSET, lss->cr0);
  ssi_putreg(LM3S_SSI_CR1_OFFSET, lss->cr1);

  ssi_enable();

  return 0;
}

void spi_release_bus(struct spi_slave *slave)
{
  //struct lm3s_spi_slave *lss = to_lm3s_spi_slave(slave);

  debug("%s: bus:%i cs:%i\n", __func__, slave->bus, slave->cs);

  ssi_disable();
}

int spi_xfer(struct spi_slave *slave, unsigned int bitlen, const void *dout,
                void *din, unsigned long flags)
{
  return 0;
}
