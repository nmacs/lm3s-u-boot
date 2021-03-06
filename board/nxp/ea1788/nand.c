/*
 * Copyright (C) 2011 by NXP Semiconductors
 * All rights reserved.
 *
 * @Author: Kevin Wells
 * @Descr: Simple NAND interface driver
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <common.h>

#if defined(CONFIG_CMD_NAND)

#include <nand.h>
#include <asm/io.h>
#include <asm/arch/hardware.h>
#include <asm/arch/lpc17_regs.h>

/* Set this define to 0 to use hardware detection */
#define USE_NAND_DELAY 0

/*
 * EA1788 board setup with CS1 as chip select. ALE uses A19, CLE uses A21,
 * and NAND RDY/BUSY is read on GPIO P2.21. CS1 is controlled as a GPIO
 * instead of a CS on GPIO P4.31 to lock the CS state during NAND transfers.
 */
#define NAND_ALE_OFFS	(1 << 19)
#define NAND_CLE_OFFS	(1 << 20)

static void ea1788_set_nand_cs(unsigned int set)
{
	if (set)
		lpc17xx_gpio_set_state(4, 31, 1);
	else
		lpc17xx_gpio_set_state(4, 31, 0);
}

static void ea1788_nand_hwcontrol(struct mtd_info *mtd, int cmd,
	unsigned int ctrl)
{
	struct nand_chip *this = mtd->priv;
	char *pCMD;
	long IO_ADDR_W;

	if (ctrl & NAND_CTRL_CHANGE) {
		IO_ADDR_W = (long) this->IO_ADDR_W &
			~(NAND_CLE_OFFS | NAND_ALE_OFFS);

		if (ctrl & NAND_CLE)
			IO_ADDR_W |= NAND_CLE_OFFS;
		else if (ctrl & NAND_ALE)
			IO_ADDR_W |= NAND_ALE_OFFS;

		if (ctrl & NAND_NCE)
			ea1788_set_nand_cs(0);
		else
			ea1788_set_nand_cs(1);

		this->IO_ADDR_W = (void *) IO_ADDR_W;
	}

	if (cmd != NAND_CMD_NONE) {
		pCMD = (char *) this->IO_ADDR_W;
		*pCMD = cmd;
	}

	return;
}

#if USE_NAND_DELAY == 0
static int ea1788_nand_device_ready(struct mtd_info *mtd)
{
	/* If NAND doesn't work, check jumper JP2-3-4 on base board */
	return lpc17xx_gpio_get_state(2, 21);
}
#endif

/*
 * Setup NAND timing and standard NAND functions
 */
int board_nand_init(struct nand_chip *nand)
{
	/* Setup CS1 on p4.31 for GPIO output instead of CS1 */
	lpc17xx_gpio_set_output(4, 31);
	ea1788_set_nand_cs(1);

	/* P2.21 for NAND ready configured as GPIO input */
	lpc17xx_gpio_set_input(2, 21);

	nand->cmd_ctrl = ea1788_nand_hwcontrol;
#if USE_NAND_DELAY == 0
	nand->dev_ready = ea1788_nand_device_ready;
#else
	nand->chip_delay = USE_NAND_DELAY;
#endif
	nand->ecc.mode = NAND_ECC_SOFT;
	nand->options = NAND_USE_FLASH_BBT;

	return 0;
}
#endif
