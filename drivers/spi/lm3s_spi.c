/****************************************************************************
 * drivers/spi/lm3s_spi.c
 *
 *   Copyright (C) 2009-2012 Max Nekludov. All rights reserved.
 *   Author: Max Nekludov <macscomp@gmail.com>
 *
 *   Based on nuttx:
 *
 *   arch/arm/src/lm32/lm3s_ssi.c
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
 ****************************************************************************/

#include <common.h>
#include <malloc.h>
#include <spi.h>
#include <stdint.h>
#include <asm/arch/hardware.h>
#include <watchdog.h>
#ifdef CONFIG_STELLARIS_DMA
#include <asm/arch/dma.h>
#endif

struct lm3s_spi_slave {
  struct spi_slave slave;

  uint32_t base;

  uint32_t cpsdvsr;
  uint32_t cr0;

  void  *txbuffer;              /* Source buffer */
  void  *rxbuffer;              /* Destination buffer */

  int      ntxwords;            /* Number of words left to transfer on the Tx FIFO */
  int      nrxwords;            /* Number of words received on the Rx FIFO */
  int      nwords;              /* Number of words to be exchanged */
  uint8_t  nbits;               /* Current number of bits per word */
  uint32_t actual;

#ifdef CONFIG_STELLARIS_DMA
	uint32_t dma_rx_flags;
	uint32_t dma_tx_flags;
	uint32_t dma_rx_channel;
	uint32_t dma_tx_channel;
	void *dma_rx_buffer;
	void *dma_tx_buffer;
	uint32_t xfer_size;
#else
  void  (*txword)(struct lm3s_spi_slave *priv);
  void  (*rxword)(struct lm3s_spi_slave *priv);
#endif
};

#ifdef CONFIG_STELLARIS_DMA
static char ssi0_dma_rx_buffer[DMA_MAX_TRANSFER_SIZE];
static char ssi0_dma_tx_buffer[DMA_MAX_TRANSFER_SIZE];
static char ssi1_dma_rx_buffer[DMA_MAX_TRANSFER_SIZE];
static char ssi1_dma_tx_buffer[DMA_MAX_TRANSFER_SIZE];
#endif

#ifdef CONFIG_BOARD_TRANSLATE_CS
extern int board_translate_cs(unsigned int* translated_cs, unsigned int cs, unsigned int bus);
#endif

/* The number of (16-bit) words that will fit in the Tx FIFO */
#define LM3S_TXFIFO_WORDS 8

/* Configuration settings */
#ifndef CONFIG_SSI_TXLIMIT
#  define CONFIG_SSI_TXLIMIT (LM3S_TXFIFO_WORDS/2)
#endif

#if CONFIG_SSI_TXLIMIT < 1 || CONFIG_SSI_TXLIMIT > LM3S_TXFIFO_WORDS
#  error "Invalid range for CONFIG_SSI_TXLIMIT"
#endif

#if CONFIG_SSI_TXLIMIT && CONFIG_SSI_TXLIMIT < (LM3S_TXFIFO_WORDS/2)
#  error "CONFIG_SSI_TXLIMIT must be at least half the TX FIFO size"
#endif

#define to_lm3s_spi_slave(s) container_of(s, struct lm3s_spi_slave, slave)
#define SSI_BASE LM3S_SSI0_BASE

//#define SSI_DEBUG          /* Define to enable debug */
//#define SSI_VERBOSE_DEBUG  /* Define to enable verbose debug */

#ifdef SSI_DEBUG
#  define ssidbg  debug
#  ifdef SSI_VERBOSE_DEBUG
#    define ssivdbg debug
#  else
#    define ssivdbg(x...)
#  endif
#else
#  define ssidbg(x...)
#  define ssivdbg(x...)
#endif

static inline uint32_t get_ssibase(unsigned int bus)
{
  switch(bus)
  {
  case 0:
    return LM3S_SSI0_BASE;
  case 1:
    return LM3S_SSI1_BASE;
  default:
    return 0;
  }
}

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

static inline uint32_t ssi_getreg(struct lm3s_spi_slave *priv,
                                     unsigned int offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: ssi_putreg
 *
 * Description:
 *   Write the value to the SSI register at this offeset
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *   offset - Offset to the SSI register from the register base address
 *   value  - Value to write
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void ssi_putreg(struct lm3s_spi_slave *priv,
                                 unsigned int offset, uint32_t value)
{
  putreg32(value, priv->base + offset);
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
    priv->nbits = nbits;
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

  if (frequency > CONFIG_SYSCLK_FREQUENCY/2)
  {
    frequency = CONFIG_SYSCLK_FREQUENCY/2;
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

  maxdvsr = CONFIG_SYSCLK_FREQUENCY / frequency;
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
  priv->actual = CONFIG_SYSCLK_FREQUENCY / (cpsdvsr * (scr + 1));
	
	ssidbg("Actual frequency: %u\n", priv->actual);
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

static void ssi_disable(struct lm3s_spi_slave *priv)
{
  uint32_t regval;

  regval = ssi_getreg(priv, LM3S_SSI_CR1_OFFSET);
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

static void ssi_enable(struct lm3s_spi_slave *priv)
{
  uint32_t regval = ssi_getreg(priv, LM3S_SSI_CR1_OFFSET);
  regval  |= SSI_CR1_SSE;
  ssi_putreg(priv, LM3S_SSI_CR1_OFFSET, regval);
}

#ifndef CONFIG_STELLARIS_DMA
/****************************************************************************
 * Name: ssi_txnull, ssi_txuint16, and ssi_txuint8
 *
 * Description:
 *   Transfer all ones, a uint8_t, or uint16_t to Tx FIFO and update the txbuffer
 *   pointer appropriately.  The selected function dependes on (1) if there
 *   is a source txbuffer provided, and (2) if the number of bits per
 *   word is <=8 or >8.
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void ssi_txnull(struct lm3s_spi_slave *priv)
{
  ssivdbg("TX: ->0xffff\n");
  ssi_putreg(priv, LM3S_SSI_DR_OFFSET, 0xffff);
}

static void ssi_txuint16(struct lm3s_spi_slave *priv)
{
  uint16_t *ptr    = (uint16_t*)priv->txbuffer;
  ssivdbg("TX: %p->%04x\n", ptr, *ptr);
  ssi_putreg(priv, LM3S_SSI_DR_OFFSET, (uint32_t)(*ptr++));
  priv->txbuffer = (void*)ptr;
}

static void ssi_txuint8(struct lm3s_spi_slave *priv)
{
  uint8_t *ptr   = (uint8_t*)priv->txbuffer;
  ssivdbg("TX: %p->%02x\n", ptr, *ptr);
  ssi_putreg(priv, LM3S_SSI_DR_OFFSET, (uint32_t)(*ptr++));
  priv->txbuffer = (void*)ptr;
}

/****************************************************************************
 * Name: ssi_rxnull, ssi_rxuint16, and ssi_rxuint8
 *
 * Description:
 *   Discard input, save a uint8_t, or or save a uint16_t from Tx FIFO in the
 *   user rxvbuffer and update the rxbuffer pointer appropriately.  The
 *   selected function dependes on (1) if there is a desination rxbuffer
 *   provided, and (2) if the number of bits per word is <=8 or >8.
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void ssi_rxnull(struct lm3s_spi_slave *priv)
{
#ifdef SSI_DEBUG_VERBOSE
  uint32_t regval  = ssi_getreg(priv, LM3S_SSI_DR_OFFSET);
  ssivdbg("RX: discard %04x\n", regval);
#else
  (void)ssi_getreg(priv, LM3S_SSI_DR_OFFSET);
#endif
}

static void ssi_rxuint16(struct lm3s_spi_slave *priv)
{
  uint16_t *ptr    = (uint16_t*)priv->rxbuffer;
  *ptr           = (uint16_t)ssi_getreg(priv, LM3S_SSI_DR_OFFSET);
  ssivdbg("RX: %p<-%04x\n", ptr, *ptr);
  priv->rxbuffer = (void*)(++ptr);
}

static void ssi_rxuint8(struct lm3s_spi_slave *priv)
{
  uint8_t *ptr   = (uint8_t*)priv->rxbuffer;
  *ptr           = (uint8_t)ssi_getreg(priv, LM3S_SSI_DR_OFFSET);
  ssivdbg("RX: %p<-%02x\n", ptr, *ptr);
  priv->rxbuffer = (void*)(++ptr);
}
#endif

/****************************************************************************
 * Name: ssi_txfifofull
 *
 * Description:
 *   Return true if the Tx FIFO is full
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *
 * Returned Value:
 *   true: Not full
 *
 ****************************************************************************/

static inline int ssi_txfifofull(struct lm3s_spi_slave *priv)
{
  return (ssi_getreg(priv, LM3S_SSI_SR_OFFSET) & SSI_SR_TNF) == 0;
}

/****************************************************************************
 * Name: ssi_rxfifoempty
 *
 * Description:
 *   Return true if the Rx FIFO is empty
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *
 * Returned Value:
 *   true: Not empty
 *
 ****************************************************************************/

static inline int ssi_rxfifoempty(struct lm3s_spi_slave *priv)
{
  return (ssi_getreg(priv, LM3S_SSI_SR_OFFSET) & SSI_SR_RNE) == 0;
}

/****************************************************************************
 * Name: ssi_performtx
 *
 * Description:
 *   If the Tx FIFO is empty, then transfer as many words as we can to
 *   the FIFO.
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *
 * Returned Value:
 *   The number of words written to the Tx FIFO (a value from 0 to 8,
 *   inclusive).
 *
 ****************************************************************************/

#ifndef CONFIG_STELLARIS_DMA
#if CONFIG_SSI_TXLIMIT == 1 && defined(CONFIG_SSI_POLLWAIT)
static inline int ssi_performtx(struct lm3s_spi_slave *priv)
{
  /* Check if the Tx FIFO is full and more data to transfer */

  if (!ssi_txfifofull(priv) && priv->ntxwords > 0)
    {
      /* Transfer one word to the Tx FIFO */

      priv->txword(priv);
      priv->ntxwords--;
      return 1;
    }
  return 0;
}

#else /* CONFIG_SSI_TXLIMIT == 1 CONFIG_SSI_POLLWAIT */

static int ssi_performtx(struct lm3s_spi_slave *priv)
{
#ifndef CONFIG_SSI_POLLWAIT
  uint32_t regval;
#endif
  int ntxd = 0;  /* Number of words written to Tx FIFO */

  /* Check if the Tx FIFO is full */

  if (!ssi_txfifofull(priv))
    {
      /* Not full.. Check if all of the Tx words have been sent */

      if (priv->ntxwords > 0)
        {
          /* No.. Transfer more words until either the Tx FIFO is full or
           * until all of the user provided data has been sent.
           */
#ifdef CONFIG_SSI_TXLIMIT
          /* Further limit the number of words that we put into the Tx
           * FIFO to CONFIG_SSI_TXLIMIT.  Otherwise, we could
           * overrun the Rx FIFO on a very fast SSI bus.
           */
          for (; ntxd < priv->ntxwords && ntxd < CONFIG_SSI_TXLIMIT && !ssi_txfifofull(priv); ntxd++)
#else
          for (; ntxd < priv->ntxwords && !ssi_txfifofull(priv); ntxd++)
#endif
            {
               priv->txword(priv);
            }

          /* Update the count of words to to transferred */

          priv->ntxwords -= ntxd;
        }

      /* Check again... Now have all of the Tx words been sent? */

#ifndef CONFIG_SSI_POLLWAIT
      regval = ssi_getreg(priv, LM3S_SSI_IM_OFFSET);
      if (priv->ntxwords > 0)
        {
          /* No.. Enable the Tx FIFO interrupt.  This interrupt occurs
           * when the Tx FIFO is 1/2 full or less.
           */

#ifdef CONFIG_DEBUG
          regval |= (SSI_IM_TX|SSI_RIS_ROR);
#else
          regval |= SSI_IM_TX;
#endif
        }
      else
        {
          /* Yes.. Disable the Tx FIFO interrupt.  The final stages of
           * the transfer will be driven by Rx FIFO interrupts.
           */

          regval &= ~(SSI_IM_TX|SSI_RIS_ROR);
        }
      ssi_putreg(priv, LM3S_SSI_IM_OFFSET, regval);
#endif /* CONFIG_SSI_POLLWAIT */
    }
  return ntxd;
}

#endif /* CONFIG_SSI_TXLIMIT == 1 CONFIG_SSI_POLLWAIT */

/****************************************************************************
 * Name: ssi_performrx
 *
 * Description:
 *   Transfer as many bytes as possible from the Rx FIFO to the user Rx
 *   buffer (if one was provided).
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void ssi_performrx(struct lm3s_spi_slave *priv)
{
#ifndef CONFIG_SSI_POLLWAIT
  uint32_t regval;
#endif

  /* Loop while data is available in the Rx FIFO */

  while (!ssi_rxfifoempty(priv))
    {
      /* Have all of the requested words been transferred from the Rx FIFO? */

      if (priv->nrxwords < priv->nwords)
        {
          /* No.. Read more data from Rx FIFO */

          priv->rxword(priv);
          priv->nrxwords++;
        }
    }

  /* The Rx FIFO is now empty.  While there is Tx data to be sent, the
   * transfer will be driven by Tx FIFO interrupts.  The final part
   * of the transfer is driven by Rx FIFO interrupts only.
   */

#ifndef CONFIG_SSI_POLLWAIT
  regval = ssi_getreg(priv, LM3S_SSI_IM_OFFSET);
  if (priv->ntxwords == 0 && priv->nrxwords < priv->nwords)
    {
       /* There are no more outgoing words to send, but there are
        * additional incoming words expected (I would think that this
        * a real corner case, be we will handle it with an extra
        * interrupt, probably an Rx timeout).
        */

#ifdef CONFIG_DEBUG
      regval |= (SSI_IM_RX|SSI_IM_RT|SSI_IM_ROR);
#else
      regval |= (SSI_IM_RX|SSI_IM_RT);
#endif
    }
  else
    {
      /* No.. there are either more Tx words to send or all Rx words
       * have received.  Disable Rx FIFO interrupts.
       */

      regval &= ~(SSI_IM_RX|SSI_IM_RT);
    }
  ssi_putreg(priv, LM3S_SSI_IM_OFFSET, regval);
#endif /* CONFIG_SSI_POLLWAIT */
}
#endif /* CONFIG_STELLARIS_DMA */

#ifdef CONFIG_STELLARIS_DMA
static int spi_transfer_step(struct lm3s_spi_slave *priv)
{
	WATCHDOG_RESET();
	
	ssivdbg("%s: ntxwords %d, nrxwords %d, nwords %d, SR %08x\n",
          __func__, priv->ntxwords, priv->nrxwords, priv->nwords,
          ssi_getreg(priv, LM3S_SSI_SR_OFFSET));

	if (priv->ntxwords == 0)
    return 0;
	
	priv->xfer_size = min(priv->ntxwords, DMA_MAX_TRANSFER_SIZE);
	
	if( priv->txbuffer )
	{
		dma_memcpy(priv->dma_tx_buffer, priv->txbuffer, priv->xfer_size);
		priv->txbuffer = (char*)priv->txbuffer + priv->xfer_size;
	}
	
	ssivdbg("%s: xfer_size %u\n", __func__, priv->xfer_size);
	
	dma_setup_xfer(priv->dma_rx_channel, priv->dma_rx_buffer,
								 priv->base + LM3S_SSI_DR_OFFSET, priv->xfer_size, priv->dma_rx_flags);
	dma_setup_xfer(priv->dma_tx_channel, priv->base + LM3S_SSI_DR_OFFSET,
								 priv->dma_tx_buffer, priv->xfer_size, priv->dma_tx_flags);
	dma_start_xfer(priv->dma_rx_channel);
	dma_start_xfer(priv->dma_tx_channel);
	
#ifdef CONFIG_ARCH_TM4C12
	putreg32(SSI_DMACTL_RXDMAE | SSI_DMACTL_TXDMAE, priv->base + LM3S_SSI_DMACTL_OFFSET);
#endif /* CONFIG_ARCH_TM4C12 */
	
	return 1;
}
#endif

/****************************************************************************
 * Name: ssi_transfer
 *
 * Description:
 *   Exchange a block data with the SPI device
 *
 * Input Parameters:
 *   priv   - Device-specific state data
 *   txbuffer - The buffer of data to send to the device (may be NULL).
 *   rxbuffer - The buffer to receive data from the device (may be NULL).
 *   nwords   - The total number of words to be exchanged.  If the interface
 *              uses <= 8 bits per word, then this is the number of uint8_t's;
 *              if the interface uses >8 bits per word, then this is the
 *              number of uint16_t's
 *
 * Returned Value:
 *   0: success, <0:Negated error number on failure
 *
 * Assumption:
 *   Caller holds a lock on the SPI bus (if CONFIG_SPI_OWNBUS not defined)
 *
 ****************************************************************************/

static int ssi_transfer(struct lm3s_spi_slave *priv, const void *txbuffer,
                        void *rxbuffer, unsigned int nwords)
{
#ifndef CONFIG_SSI_POLLWAIT
  irqstate_t flags;
#endif
  int ntxd;

  ssidbg("ssi_transfer: txbuffer: %p rxbuffer: %p nwords: %d\n", txbuffer, rxbuffer, nwords);

  /* Set up to perform the transfer */

  priv->txbuffer     = (uint8_t*)txbuffer; /* Source buffer */
  priv->rxbuffer     = (uint8_t*)rxbuffer; /* Destination buffer */
  priv->ntxwords     = nwords;             /* Number of words left to send */
  priv->nrxwords     = 0;                  /* Number of words received */
  priv->nwords       = nwords;             /* Total number of exchanges */

#ifdef CONFIG_STELLARIS_DMA
	priv->dma_tx_flags = DMA_XFER_MEMORY_TO_DEVICE;
	priv->dma_rx_flags = DMA_XFER_DEVICE_TO_MEMORY;

//	if (dev_priv->bits_per_word > 8)
//	{
//		priv_master->dma_tx_flags |= DMA_XFER_UNIT_WORD;
//		priv_master->dma_rx_flags |= DMA_XFER_UNIT_WORD;
//	}
//  else
//	{
		priv->dma_tx_flags |= DMA_XFER_UNIT_BYTE;
		priv->dma_rx_flags |= DMA_XFER_UNIT_BYTE;
//	}
#else
  /* Set up the low-level data transfer function pointers */

  if (priv->nbits > 8)
    {
      priv->txword = ssi_txuint16;
      priv->rxword = ssi_rxuint16;
    }
  else
    {
      priv->txword = ssi_txuint8;
      priv->rxword = ssi_rxuint8;
    }

  if (!txbuffer)
    {
      priv->txword = ssi_txnull;
    }

  if (!rxbuffer)
    {
      priv->rxword = ssi_rxnull;
    }
#endif

  /* Prime the Tx FIFO to start the sequence (saves one interrupt).
   * At this point, all SSI interrupts should be disabled, but the
   * operation of ssi_performtx() will set up the interrupts
   * approapriately (if nwords > TxFIFO size).
   */

#ifndef CONFIG_SSI_POLLWAIT
  flags = irqsave();
  ssivdbg("ntxwords: %d nrxwords: %d nwords: %d SR: %08x\n",
          priv->ntxwords, priv->nrxwords, priv->nwords,
          ssi_getreg(priv, LM3S_SSI_SR_OFFSET));

  ntxd  = ssi_performtx(priv);

  /* For the case where nwords < Tx FIFO size, ssi_performrx will
   * configure interrupts correctly for the final phase of the
   * the transfer.
   */

  ssi_performrx(priv);

  ssivdbg("ntxwords: %d nrxwords: %d nwords: %d SR: %08x IM: %08x\n",
          priv->ntxwords, priv->nrxwords, priv->nwords,
          ssi_getreg(priv, LM3S_SSI_SR_OFFSET),
          ssi_getreg(priv, LM3S_SSI_IM_OFFSET));

  /* Wait for the transfer to complete.  Since there is no handshake
   * with SPI, the following should complete even if there are problems
   * with the transfer, so it should be safe with no timeout.
   */

  ssivdbg("Waiting for transfer complete\n");
  irqrestore(flags);
  do
    {
      ssi_semtake(&priv->xfrsem);
    }
  while (priv->nrxwords < priv->nwords);
  ssidbg("Transfer complete\n");

#else
  /* Perform the transfer using polling logic.  This will totally
   * dominate the CPU until the transfer is complete.  Only recommended
   * if (1) your SPI is very fast, and (2) if you only use very short
   * transfers.
   */
	
#ifdef CONFIG_STELLARIS_DMA
	ssidbg("Performing transfer...\n");
	while (spi_transfer_step(priv)) {
		uint32_t regval2;
		
		dma_wait_xfer_complete(priv->dma_rx_channel);
		
		regval2 = ssi_getreg(priv, LM3S_SSI_DMACTL_OFFSET);
		regval2 &= ~SSI_DMACTL_TXDMAE;
		ssi_putreg(priv, LM3S_SSI_DMACTL_OFFSET, regval2);
		ssi_putreg(priv, LM3S_SSI_ICR_OFFSET, SSI_ICR_DMATX);
		
		regval2 = ssi_getreg(priv, LM3S_SSI_DMACTL_OFFSET);
		regval2 &= ~SSI_DMACTL_RXDMAE;
		ssi_putreg(priv, LM3S_SSI_DMACTL_OFFSET, regval2);
		ssi_putreg(priv, LM3S_SSI_ICR_OFFSET, SSI_ICR_DMARX);
		
#ifdef SSI_VERBOSE_DEBUG
		ssivdbg("RX data %u bytes (buf %p):\n", priv->xfer_size, priv->dma_rx_buffer);
		print_buffer(0, priv->dma_rx_buffer, 1, priv->xfer_size, 0);
#endif
		
		if( priv->rxbuffer )
		{
			dma_memcpy(priv->rxbuffer, priv->dma_rx_buffer, priv->xfer_size);
			priv->rxbuffer = (char*)priv->rxbuffer + priv->xfer_size;
		}
		priv->nrxwords += priv->xfer_size;
		priv->ntxwords -= priv->xfer_size;
	}
	ssidbg("Transfer complete\n");
#else /* CONFIG_STELLARIS_DMA */
  do
    {
      /* Handle outgoing Tx FIFO transfers */

      ntxd = ssi_performtx(priv);

      /* Handle incoming Rx FIFO transfers */

      ssi_performrx(priv);
    }
  while (priv->nrxwords < priv->nwords);
#endif /* CONFIG_STELLARIS_DMA */
#endif

  return 0;
}

int spi_cs_is_valid(unsigned int bus, unsigned int cs)
{
  ssidbg("%s: bus %u, cs %u\n", __func__, bus, cs);

  if( get_ssibase(bus) == 0 )
  {
    ssidbg("%s: invalid bus\n", __func__);
    return 0;
  }

#ifdef CONFIG_BOARD_TRANSLATE_CS
  if( board_translate_cs(&cs, cs, bus) != 0 )
  {
    ssidbg("%s: invalid cs\n", __func__);
    return 0;
  }
#endif

  return 1;
}

void spi_cs_activate(struct spi_slave *slave)
{
  if (slave->cs)
    gpiowrite(slave->cs, 0);
  ssivdbg("%s: SPI_CS_GPIO:%x\n", __func__, slave->cs);
}

void spi_cs_deactivate(struct spi_slave *slave)
{
  if (slave->cs)
    gpiowrite(slave->cs, 1);
  ssivdbg("%s: SPI_CS_GPIO:%x\n", __func__, slave->cs);
}

int spi_init(void)
{
  ssi_clock_ctrl(0, SYS_ENABLE_CLOCK);
	ssi_clock_ctrl(1, SYS_ENABLE_CLOCK);
	return 0;
}

void spi_set_speed(struct spi_slave *slave, uint hz)
{
  struct lm3s_spi_slave *lss = to_lm3s_spi_slave(slave);
  ssi_setfrequency(lss, hz);
}

struct spi_slave *spi_setup_slave(unsigned int bus, unsigned int cs,
                unsigned int max_hz, unsigned int mode)
{
  ssidbg("%s: bus %u, cs %u, max_hz %u, mode %u\n", __func__, bus, cs, max_hz, mode);

  uint32_t ssi_base = get_ssibase(bus);
  if( !ssi_base )
  {
    ssidbg("%s: invalid bus\n", __func__);
    return 0;
  }

#ifdef CONFIG_BOARD_TRANSLATE_CS
  if( board_translate_cs(&cs, cs, bus) != 0 )
  {
    ssidbg("%s: invalid cs\n", __func__);
    return 0;
  }
#endif

  struct lm3s_spi_slave *lss;
  lss = (struct lm3s_spi_slave *)malloc(sizeof(*lss));
  if (!lss)
  {
    ssidbg("%s: fail to allocate memory for lm3s_spi_slave\n", __func__);
    return 0;
  }

  memset(lss, 0, sizeof(*lss));

  lss->slave.bus = bus;
  lss->slave.cs = cs;
  lss->base = ssi_base;

  ssi_setmode(lss, mode);
  ssi_setbits(lss, 8);
  ssi_setfrequency(lss, max_hz);

#ifdef CONFIG_STELLARIS_DMA
	if (ssi_base == LM3S_SSI0_BASE) {
		lss->dma_rx_channel = DMA_CHANNEL_SSI0_RX;
		lss->dma_tx_channel = DMA_CHANNEL_SSI0_TX;
		lss->dma_rx_buffer = ssi0_dma_rx_buffer;
		lss->dma_tx_buffer = ssi0_dma_tx_buffer;
	}
	else if (ssi_base == LM3S_SSI1_BASE) {
		lss->dma_rx_channel = DMA_CHANNEL_SSI1_RX;
		lss->dma_tx_channel = DMA_CHANNEL_SSI1_TX;
		lss->dma_rx_buffer = ssi1_dma_rx_buffer;
		lss->dma_tx_buffer = ssi1_dma_tx_buffer;
	}
#endif

  return &lss->slave;
}

void spi_free_slave(struct spi_slave *slave)
{
  ssidbg("%s\n", __func__);
  struct lm3s_spi_slave *lss = to_lm3s_spi_slave(slave);
  free(lss);
}

int spi_claim_bus(struct spi_slave *slave)
{
  struct lm3s_spi_slave *lss = to_lm3s_spi_slave(slave);
  ssivdbg("%s: bus:%i cs:%x\n", __func__, slave->bus, slave->cs);

  /* Set CR1 */
  ssi_putreg(lss, LM3S_SSI_CR1_OFFSET, 0);

  /* Set CPDVSR */
  ssi_putreg(lss, LM3S_SSI_CPSR_OFFSET, lss->cpsdvsr);

  /* Set CR0 */
  ssi_putreg(lss, LM3S_SSI_CR0_OFFSET, lss->cr0);

  ssi_enable(lss);

  return 0;
}

void spi_release_bus(struct spi_slave *slave)
{
  struct lm3s_spi_slave *lss = to_lm3s_spi_slave(slave);

  ssivdbg("%s: bus:%i cs:%x\n", __func__, slave->bus, slave->cs);

  ssi_disable(lss);
}

static inline void spi_xferdone(struct spi_slave *slave, unsigned long flags)
{
  if (flags & SPI_XFER_END)
    spi_cs_deactivate(slave);
}

int spi_xfer(struct spi_slave *slave, unsigned int bitlen, const void *dout,
                void *din, unsigned long flags)
{
  struct lm3s_spi_slave *lss = to_lm3s_spi_slave(slave);
  uint bytes = bitlen / 8;

	WATCHDOG_RESET();

  ssivdbg("%s: bus:%i cs:%x bitlen:%i bytes:%i flags:%lx\n", __func__,
        slave->bus, slave->cs, bitlen, bytes, flags);

  if (bitlen == 0)
  {
    spi_xferdone(slave, flags);
    return 0;
  }

  /* we can only do 8 bit transfers */
  if (bitlen % 8)
  {
    spi_xferdone(slave, flags | SPI_XFER_END);
    return 0;
  }

  if (flags & SPI_XFER_BEGIN)
    spi_cs_activate(slave);

  ssi_transfer(lss, dout, din, bytes);

  spi_xferdone(slave, flags);

  return 0;
}
