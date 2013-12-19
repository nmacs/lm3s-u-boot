/*
 * 2011 PVV
 * 2010 Alexander Kudjashev
 *
 * Adapted Simtec Electronics linux driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <config.h>
#include <common.h>
#include <malloc.h>
#include <net.h>
#include <spi.h>
#include <linux/types.h>
#include "ks8851.h"

#define KSZ8851_DRIVERNAME "ksz8851snl"

#ifdef DEBUG
# define DEBUGOUT(fmt,args...) printf(fmt ,##args)
#else
# define DEBUGOUT(fmt,args...)
#endif

extern void NetReceive(volatile uchar *, int);

#define MAX_BUF_SIZE    (4096 + 8)

/* shift for byte-enable data */
#define BYTE_EN(_x) ((_x) << 2)

/* turn register number and byte-enable mask into data for start of packet */
#define MK_OP(_byteen, _reg) (BYTE_EN(_byteen) | (_reg)  << (8+2) | (_reg) >> 6)

union ks8851_tx_hdr {
    uchar   txb[6];
    ushort  txw[3];
};

typedef struct ks8851_device {
	struct eth_device	*dev;	/* back pointer */
	struct spi_slave	*slave;
	ushort  fid;
	ushort  rc_rxqcr;
	uchar   buff[MAX_BUF_SIZE];
	union ks8851_tx_hdr txh;
} ks8851_dev_t;

uchar def_mac_addr[] = {0x00, 0x10, 0xA1, 0x86, 0x95, 0x11};

static void ks_reg8_write(ks8851_dev_t *ks, ushort reg, ushort val)
{
    ushort  txb[2];
    ushort  bit;

    bit = 1 << (reg & 3);

    txb[0] = MK_OP(bit, reg) | KS_SPIOP_WR;
    txb[1] = val;

	spi_claim_bus(ks->slave);
	spi_xfer(ks->slave, 24, txb, 0, SPI_XFER_BEGIN | SPI_XFER_END);
	spi_release_bus(ks->slave);
}

/*
 * write a 16 bit register to ks8851 chip
*/
static void ks_reg16_write(ks8851_dev_t *ks, ushort reg, ushort val)
{
    ushort txb[2];

    txb[0] = MK_OP(reg & 2 ? 0xC : 0x03, reg) | KS_SPIOP_WR;
    txb[1] = val;

	spi_claim_bus(ks->slave);
	spi_xfer(ks->slave, 32, txb, 0, SPI_XFER_BEGIN | SPI_XFER_END);
	spi_release_bus(ks->slave);
}

/*
 * issue read register command and return the data
 */
static void ks_reg_read(ks8851_dev_t *ks, ushort op, uchar *rxb, ushort len)
{
    ushort txb = op | KS_SPIOP_RD;

	spi_claim_bus(ks->slave);
	spi_xfer(ks->slave, 16, &txb, 0, SPI_XFER_BEGIN);
	spi_xfer(ks->slave, 8*len, 0, rxb, SPI_XFER_END);
	spi_release_bus(ks->slave);
}

/*
 * read 8 bit register from device
 */
static ushort ks_reg8_read(ks8851_dev_t *ks, ushort reg)
{
    uchar rxb = 0;

    ks_reg_read(ks, MK_OP(1 << (reg & 3), reg), &rxb, 1);

    return rxb;
}

/*
 * read 16 bit register from device
 */
static ushort ks_reg16_read(ks8851_dev_t *ks, ushort reg)
{
    ushort rxb = 0;

    ks_reg_read(ks, MK_OP(reg & 2 ? 0xC : 0x3, reg), (uchar*)&rxb, 2);

    return rxb;
}

/*
 * read 32 bit register from device
 */
static uint ks_reg32_read(ks8851_dev_t *ks, ushort reg)
{
    uint rxb = 0;

    ks_reg_read(ks, MK_OP(0x0f, reg), (uchar*)&rxb, 4);

    return rxb;
}

/*
 * set KS8851 MAC address
*/
void ks_mac_set(ks8851_dev_t *ks)
{
	if (getenv ("ethaddr")) {
		int i;
		uchar enetaddr[6];
		DEBUGOUT("Set MAC %s\n", getenv ("ethaddr"));
		eth_getenv_enetaddr("ethaddr", enetaddr);
		for(i = 0; i < 6; i++)
			ks_reg8_write(ks, KS_MAR(i), enetaddr[i]);
	}
}

/*
 * read data from the receive fifo
 */
static void ks_fifo_read(ks8851_dev_t *ks, uchar *buff, ushort len)
{
	uchar txb = KS_SPIOP_RXFIFO;
	spi_claim_bus(ks->slave);
	spi_xfer(ks->slave, 8, &txb, 0, SPI_XFER_BEGIN);
	spi_xfer(ks->slave, 8*len, 0, buff, SPI_XFER_END);
	spi_release_bus(ks->slave);
}

/*
 * receive packets from the host
 */
static int ksz_recv(struct eth_device *dev)
{
	uint    rxh;
	ushort  rxfc, rxlen, status;
	ks8851_dev_t *ks = dev->priv;
	
	//status = ks_reg16_read(ks, KS_ISR);
	//if(status & IRQ_RXI == 0)
	//	return 0;

	rxfc = ks_reg8_read(ks, KS_RXFC);
	debug("%s: rxfc:%u\n", __func__, (unsigned)rxfc);

	for (; rxfc != 0; rxfc--) {
		rxh = ks_reg32_read(ks, KS_RXFHSR);
		/* the length of the packet includes the 32bit CRC */
		rxlen = (rxh >> 16) & 0xFFF;
		
		debug("ksz8851: rxh:%u, rxlen:%i\n", rxh, (int)rxlen);

		/* setup Receive Frame Data Pointer Auto-Increment */
		ks_reg16_write(ks, KS_RXFDPR, RXFDPR_RXFPAI);

		/* start the packet dma process, and set auto-dequeue rx */
		ks_reg16_write(ks, KS_RXQCR, ks->rc_rxqcr | RXQCR_SDA | RXQCR_ADRFE);

		if(rxlen > 4) {
			unsigned int rxalign;
			rxlen -= 4;
			debug("ksz8851: rxlen:%i\n", (int)rxlen);
			rxalign = ALIGN(rxlen, 4);
			if ((rxlen + 8) > MAX_BUF_SIZE)
			{
				printf("rx_err: buffer overflow len=%d\n", rxlen);
				rxalign = MAX_BUF_SIZE - 8;
			}
			/* align the packet length to 4 bytes, and add 4 bytes
					as we're getting the rx status header as well */
			ks_fifo_read(ks, ks->buff, rxalign + 8);

			NetReceive(ks->buff + 8, rxlen);
		}

		ks_reg16_write(ks, KS_RXQCR, ks->rc_rxqcr);
	}
	
			//memset(ks->buff, 0, MAX_BUF_SIZE);
	return 0;
}

static int ksz_process(struct eth_device *dev)
{
	unsigned status;
	unsigned handled = 0;
	ks8851_dev_t *ks = dev->priv;
	
	status = ks_reg16_read(ks, KS_ISR);
	if (!(status & IRQ_RXI))
		return 0;
	
	debug("%s: status 0x%04x\n", __func__, status);
	
	if (status & IRQ_LCI)
		handled |= IRQ_LCI;
	
	if (status & IRQ_LDI) {
		unsigned short pmecr = ks_reg16_read(ks, KS_PMECR);
		pmecr &= ~PMECR_WKEVT_MASK;
		ks_reg16_write(ks, KS_PMECR, pmecr | PMECR_WKEVT_LINK);

		handled |= IRQ_LDI;
	}
	
	if (status & IRQ_RXPSI)
		handled |= IRQ_RXPSI;
	
	if (status & IRQ_TXI) {
		handled |= IRQ_TXI;
	}
	
	if (status & IRQ_RXI)
		handled |= IRQ_RXI;
	
	if (status & IRQ_SPIBEI) {
		debug("%s: spi bus error\n", __func__);
		handled |= IRQ_SPIBEI;
	}
	
	ks_reg16_write(ks, KS_ISR, handled);
	
	if (status & IRQ_RXI)
		ksz_recv(dev);
	
	/* if something stopped the rx process,
	 * then do something about restarting it.
	 */
	if (status & IRQ_RXPSI) {
		debug("%s: restart receive\n", __func__);
		/* setup receiver control */
		ks_reg16_write(ks, KS_RXCR1, (RXCR1_RXPAFMA |   /* mac filter */
														RXCR1_RXFCE |       /* enable flow control */
														RXCR1_RXUE |        /* unicast enable */
														RXCR1_RXE |         /* enable rx block */
														RXCR1_RXBE));       /* broadcast enable */
		/* transfer entire frames out in one go */
		ks_reg16_write(ks, KS_RXCR2, RXCR2_SRDBL_FRAME);
	}
	
	debug("%s: done\n", __func__);
	
	return 0;
}

/*
 * write packet to TX FIFO
 */
static int ksz_send(struct eth_device *dev,volatile void *txp, int len)
{
	ushort fid = 0;

	ks8851_dev_t *ks = dev->priv;

	fid = ks->fid++;
	fid &= TXFR_TXFID_MASK;

	/* start header at txb[1] to align txw entries */
	ks->txh.txb[1] = KS_SPIOP_TXFIFO;
	ks->txh.txw[1] = fid;
	ks->txh.txw[2] = len;

	ks_reg16_write(ks, KS_RXQCR, ks->rc_rxqcr | RXQCR_SDA);

	spi_claim_bus(ks->slave);
	spi_xfer(ks->slave, 8*5, &ks->txh.txb[1], 0, SPI_XFER_BEGIN );
	spi_xfer(ks->slave, 8*ALIGN(len, 4), (uchar*)txp, 0, SPI_XFER_END );
	spi_release_bus(ks->slave);
	ks_reg16_write(ks, KS_RXQCR, ks->rc_rxqcr);
	ks_reg16_write(ks, KS_TXQCR, TXQCR_METFE);
}
/**
 * ks8851_read_selftest - read the selftest memory info.
 * @ks: The device state
 *
 * Read and check the TX/RX memory selftest information.
 */
static int ks_read_selftest(ks8851_dev_t *ks)
{
	unsigned both_done = MBIR_TXMBF | MBIR_RXMBF;
	int ret = 0;
	unsigned rd;

	rd = ks_reg16_read(ks, KS_MBIR);

	if ((rd & both_done) != both_done) {
		printf("Memory selftest not finished\n");
		return 0;
	}

	if (rd & MBIR_TXMBFA) {
		printf("TX memory selftest fail\n");
		ret |= 1;
	}

	if (rd & MBIR_RXMBFA) {
		printf("RX memory selftest fail\n");
		ret |= 2;
	}

	return 0;
}

static void ks8851_soft_reset(ks8851_dev_t *ks, unsigned op)
{
	ks_reg16_write(ks, KS_GRR, op);
	udelay(1000);	/* wait a short time to effect reset */
	ks_reg16_write(ks, KS_GRR, 0);
	udelay(1000);	/* wait for condition to clear */
}

/*
 * set power mode of the device
 */
static void ks_powermode_set(ks8851_dev_t *ks, ushort pwrmode)
{
    ushort pmecr;

    pmecr = ks_reg16_read(ks, KS_PMECR);
    pmecr &= ~PMECR_PM_MASK;
    pmecr |= pwrmode;

    ks_reg16_write(ks, KS_PMECR, pmecr);
}

/*
 * configure network device
 */
static void ks_config(ks8851_dev_t *ks)
{
    /* setup transmission parameters */
    ks_reg16_write(ks, KS_TXCR, (TXCR_TXE |     /* enable transmit process */
                            TXCR_TXPE |     /* pad to min length */
                            TXCR_TXCRC |    /* add CRC */
                            TXCR_TXFCE));   /* enable flow control */

    /* Enable QMU TxQ Auto-Enqueue frame */
//    ks_reg16_write(ks, KS_TXQCR, TXQCR_AETFE);

    /* auto-increment tx data, reset tx pointer */
    ks_reg16_write(ks, KS_TXFDPR, TXFDPR_TXFPAI);

    /* setup receiver control */
    ks_reg16_write(ks, KS_RXCR1, (RXCR1_RXPAFMA |   /* mac filter */
                            RXCR1_RXFCE |       /* enable flow control */
                            RXCR1_RXUE |        /* unicast enable */
                            RXCR1_RXE |         /* enable rx block */
                            RXCR1_RXBE));       /* broadcast enable */

    /* transfer entire frames out in one go */
    ks_reg16_write(ks, KS_RXCR2, RXCR2_SRDBL_FRAME);

    /* set receive counter timeouts */
    ks_reg16_write(ks, KS_RXDTTR, 1000); /* 1ms after first frame to IRQ */
    ks_reg16_write(ks, KS_RXDBCTR, 4096); /* >4Kbytes in buffer to IRQ */
    ks_reg16_write(ks, KS_RXFCTR, 10);  /* 10 frames to IRQ */

		ks->rc_rxqcr = (RXQCR_RXFCTE |  /* IRQ on frame count exceeded */
			RXQCR_RXDBCTE | /* IRQ on byte count exceeded */
			RXQCR_RXDTTE);  /* IRQ on time exceeded */

    ks_reg16_write(ks, KS_RXQCR, ks->rc_rxqcr);

#define STD_IRQ (IRQ_LCI |	/* Link Change */	\
		 IRQ_TXI |	/* TX done */		\
		 IRQ_RXI |	/* RX done */		\
		 IRQ_SPIBEI |	/* SPI bus error */	\
		 IRQ_TXPSI |	/* TX process stop */	\
		 IRQ_RXPSI)	/* RX process stop */

    ks_reg16_write(ks, KS_ISR, STD_IRQ);
    ks_reg16_write(ks, KS_IER, STD_IRQ);
}

static int wait_link_up(ks8851_dev_t *ks, int timeout)
{
	while( timeout > 0 )
	{
		int status = ks_reg16_read(ks, KS_P1MBSR);
		if( status & 1 << 2 )
			return 0;
		udelay(1000);
		timeout--;
	}

	return -1;
}

/*
 * KS8851 chip initialization
*/
static int ksz_init(struct eth_device *dev, bd_t *bis)
{
    ks8851_dev_t *ks = dev->priv;
    ushort chip_id;

    /* bring chip out of any power saving mode it was in */
    ks_powermode_set(ks, PMECR_PM_NORMAL);

		ks8851_soft_reset(ks, GRR_GSR);

    /* simple check for a valid chip being connected to the bus */
    chip_id = ks_reg16_read(ks, KS_CIDER);
    DEBUGOUT("ks8851 chip ID=0x%x\n", chip_id);
    if((chip_id & ~CIDER_REV_MASK) != CIDER_ID) {
        printf("Error: the ks8851 chip ID is wrong\n");
        return -1;
    }

    ks->fid = 0;

    udelay(5000);
    ks_read_selftest(ks);

    ks_mac_set(ks);

		ks8851_soft_reset(ks, GRR_QMU);
    ks_config(ks);

    return wait_link_up(ks, 5000);
}

/*
 * close network device
 */
static void ksz_halt(struct eth_device *dev)
{
    ks8851_dev_t *ks = dev->priv;

    /* turn off the IRQs and ack any outstanding */
    ks_reg16_write(ks, KS_IER, 0x0000);
    ks_reg16_write(ks, KS_ISR, 0xffff);

    /* shutdown RX process */
    ks_reg16_write(ks, KS_RXCR1, 0x0000);

    /* shutdown TX process */
    ks_reg16_write(ks, KS_TXCR, 0x0000);

    /* set powermode to soft power down to save power */
    ks_powermode_set(ks, PMECR_PM_SOFTDOWN);
}

int ks8851_initialize(unsigned int bus, unsigned int cs,
	unsigned int max_hz, unsigned int mode)
{
	struct eth_device *dev;
	ks8851_dev_t *ks;

	/* try to allocate, check and clear eth_device object */
	dev = malloc(sizeof(*dev));
	if (!dev) {
					return -1;
	}
	memset(dev, 0, sizeof(*dev));

	/* try to allocate, check and clear enc_dev_t object */
	ks = malloc(sizeof(*ks));
	if (!ks) {
		free(dev);
		return -1;
	}
	memset(ks, 0, sizeof(*ks));

	/* try to setup the SPI slave */
	ks->slave = spi_setup_slave(bus, cs, max_hz, mode);
	if (!ks->slave) {
		printf("ks8851: invalid SPI device %i:%i\n", bus, cs);
		free(ks);
		free(dev);
		return -1;
	}

	ks->dev = dev;

	/* now fill the eth_device object */
	dev->priv = ks;
	dev->init = ksz_init;
	dev->halt = ksz_halt;
	dev->send = ksz_send;
	//dev->recv = ksz_recv;
	dev->recv = ksz_process;

	sprintf(dev->name, "ksz8851:%i.%i", bus,cs);
	eth_register(dev);

	ksz_halt(dev);

	return 0;
}
