/*
 * Command for accessing SPI flash.
 *
 * Copyright (C) 2008 Atmel Corporation
 * Licensed under the GPL-2 or later.
 */

#include <common.h>
#include <spi_flash.h>

#include <asm/io.h>

#ifndef CONFIG_SF_DEFAULT_SPEED
# define CONFIG_SF_DEFAULT_SPEED	1000000
#endif
#ifndef CONFIG_SF_DEFAULT_MODE
# define CONFIG_SF_DEFAULT_MODE		SPI_MODE_3
#endif

static struct spi_flash *flash = 0;

#ifdef CONFIG_MTD_DEVICE
#include <linux/mtd/mtd.h>
#include <jffs2/load_kernel.h>
struct mtd_info serial_mtd;
#endif

struct spi_flash* get_current_flash()
{
  if( flash == 0 )
    puts("WARNING: No SPI flash selected. Please run `sf probe'\n");

  return flash;
}

/*
 * This function computes the length argument for the erase command.
 * The length on which the command is to operate can be given in two forms:
 * 1. <cmd> offset len  - operate on <'offset',  'len')
 * 2. <cmd> offset +len - operate on <'offset',  'round_up(len)')
 * If the second form is used and the length doesn't fall on the
 * sector boundary, than it will be adjusted to the next sector boundary.
 * If it isn't in the flash, the function will fail (return -1).
 * Input:
 *    arg: length specification (i.e. both command arguments)
 * Output:
 *    len: computed length for operation
 * Return:
 *    1: success
 *   -1: failure (bad format, bad address).
 */
static int sf_parse_len_arg(char *arg, ulong *len)
{
	char *ep;
	char round_up_len; /* indicates if the "+length" form used */
	ulong len_arg;

	round_up_len = 0;
	if (*arg == '+') {
		round_up_len = 1;
		++arg;
	}

	len_arg = simple_strtoul(arg, &ep, 16);
	if (ep == arg || *ep != '\0')
		return -1;

	if (round_up_len && flash->sector_size > 0)
		*len = ROUND(len_arg - 1, flash->sector_size);
	else
		*len = len_arg;

	return 1;
}

#ifdef CONFIG_MTD_DEVICE
int serial_mtd_read(struct mtd_info *mtd, loff_t from, size_t len, size_t *retlen, u_char *buf)
{
  struct spi_flash *flash = mtd->priv;
  if( spi_flash_read(flash, from, len, buf) )
    return -1;

  *retlen = len;

  return 0;
}

int serial_mtd_write(struct mtd_info *mtd, loff_t to, size_t len, size_t *retlen, const u_char *buf)
{
  struct spi_flash *flash = mtd->priv;
  if( spi_flash_write(flash, to, len, buf) )
    return -1;

  *retlen = len;

  return 0;
}

int serial_mtd_erase(struct mtd_info *mtd, struct erase_info *instr)
{
  struct spi_flash *flash = mtd->priv;
  if( spi_flash_erase(flash, instr->addr, instr->len) )
  {
    instr->fail_addr = instr->addr;
    instr->callback(instr);
    return -1;
  }

  instr->callback(instr);
  return 0;
}

#endif

static int do_spi_flash_probe(int argc, char * const argv[])
{
	unsigned int bus = 0;
	unsigned int cs;
	unsigned int speed = CONFIG_SF_DEFAULT_SPEED;
	unsigned int mode = CONFIG_SF_DEFAULT_MODE;
	char *endp;
	struct spi_flash *new;

	if (argc < 2)
		return -1;

	cs = simple_strtoul(argv[1], &endp, 0);
	if (*argv[1] == 0 || (*endp != 0 && *endp != ':'))
		return -1;
	if (*endp == ':') {
		if (endp[1] == 0)
			return -1;

		bus = cs;
		cs = simple_strtoul(endp + 1, &endp, 0);
		if (*endp != 0)
			return -1;
	}

	if (argc >= 3) {
		speed = simple_strtoul(argv[2], &endp, 0);
		if (*argv[2] == 0 || *endp != 0)
			return -1;
	}
	if (argc >= 4) {
		mode = simple_strtoul(argv[3], &endp, 16);
		if (*argv[3] == 0 || *endp != 0)
			return -1;
	}

	new = spi_flash_probe(bus, cs, speed, mode);
	if (!new) {
		printf("Failed to initialize SPI flash at %u:%u\n", bus, cs);
		return 1;
	}

	if (flash)
		spi_flash_free(flash);
	flash = new;

#ifdef CONFIG_MTD_DEVICE
  /*
   * Add MTD device so that we can reference it later
   * via the mtdcore infrastructure (e.g. ubi).
   */
  serial_mtd.type = MTD_DATAFLASH;
  serial_mtd.flags = MTD_WRITEABLE;
  serial_mtd.name = "serial0";
  serial_mtd.size = flash->size;
  serial_mtd.erasesize = flash->sector_size;
  serial_mtd.writesize = 1;
  serial_mtd.index = 0;
  serial_mtd.read = serial_mtd_read;
  serial_mtd.write = serial_mtd_write;
  serial_mtd.erase = serial_mtd_erase;
  serial_mtd.priv = flash;

  add_mtd_device(&serial_mtd);
#endif

	return 0;
}

static int do_spi_flash_read_write(int argc, char * const argv[])
{
	unsigned long addr;
	unsigned long offset;
	unsigned long len;
	void *buf;
	char *endp;
	int ret;

	if (argc < 4)
		return -1;

	addr = simple_strtoul(argv[1], &endp, 16);
	if (*argv[1] == 0 || *endp != 0)
		return -1;
	offset = simple_strtoul(argv[2], &endp, 16);
	if (*argv[2] == 0 || *endp != 0)
		return -1;
	len = simple_strtoul(argv[3], &endp, 16);
	if (*argv[3] == 0 || *endp != 0)
		return -1;

	buf = map_physmem(addr, len, MAP_WRBACK);
	if (!buf) {
		puts("Failed to map physical memory\n");
		return 1;
	}

	if (strcmp(argv[0], "read") == 0)
		ret = spi_flash_read(flash, offset, len, buf);
	else
		ret = spi_flash_write(flash, offset, len, buf);

	unmap_physmem(buf, len);

	if (ret) {
		printf("SPI flash %s failed\n", argv[0]);
		return 1;
	}

	return 0;
}

static int do_spi_flash_erase(int argc, char * const argv[])
{
	unsigned long offset;
	unsigned long len;
	char *endp;
	int ret;

	if (argc < 3)
		return -1;

	offset = simple_strtoul(argv[1], &endp, 16);
	if (*argv[1] == 0 || *endp != 0)
		return -1;

	ret = sf_parse_len_arg(argv[2], &len);
	if (ret != 1)
		return -1;

	ret = spi_flash_erase(flash, offset, len);
	if (ret) {
		printf("SPI flash %s failed\n", argv[0]);
		return 1;
	}

	return 0;
}

static int do_spi_flash_eraseall(int argc, char * const argv[])
{
	int ret;

	ret = spi_flash_erase(flash, 0, flash->size);
	if (ret) {
		printf("SPI flash %s failed\n", argv[0]);
		return 1;
	}

	return 0;
}

static int do_spi_flash(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	const char *cmd;
	int ret;

	/* need at least two arguments */
	if (argc < 2)
		goto usage;

	cmd = argv[1];
	--argc;
	++argv;

	if (strcmp(cmd, "probe") == 0) {
		ret = do_spi_flash_probe(argc, argv);
		goto done;
	}

	/* The remaining commands require a selected device */
	if (!flash) {
		puts("No SPI flash selected. Please run `sf probe'\n");
		return 1;
	}

	if (strcmp(cmd, "read") == 0 || strcmp(cmd, "write") == 0)
		ret = do_spi_flash_read_write(argc, argv);
	else if (strcmp(cmd, "erase") == 0)
		ret = do_spi_flash_erase(argc, argv);
	else if (strcmp(cmd, "eraseall") == 0)
		ret = do_spi_flash_eraseall(argc, argv);
	else
		ret = -1;

done:
	if (ret != -1)
		return ret;

usage:
	return cmd_usage(cmdtp);
}

U_BOOT_CMD(
	sf,	5,	1,	do_spi_flash,
	"SPI flash sub-system",
	"probe [bus:]cs [hz] [mode]	- init flash device on given SPI bus\n"
	"				  and chip select\n"
	"sf read addr offset len 	- read `len' bytes starting at\n"
	"				  `offset' to memory at `addr'\n"
	"sf write addr offset len	- write `len' bytes from memory\n"
	"				  at `addr' to flash at `offset'\n"
	"sf erase offset [+]len		- erase `len' bytes from `offset'\n"
	"				  `+len' round up `len' to block size"
);
