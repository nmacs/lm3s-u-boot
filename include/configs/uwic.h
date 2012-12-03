#ifndef __UWIC_H
#define __UWIC_H

#define CONFIG_LM3S
#define CONFIG_ARCH_LM3S1D21
#include <asm/arch-lm3s/chip.h>
#include "uwic_pins.h"

/* RCC settings */

#define SYSCON_RCC_XTAL      SYSCON_RCC_XTAL4000KHZ /* S2E on-board crystal is 8.00 MHz */
#define XTAL_FREQUENCY       4000000

/* Oscillator source is the main oscillator (not internal, internal/4, 30KHz or
 * 30KHz from hibernate module) */

#define SYSCON_RCC_OSCSRC    SYSCON_RCC_OSCSRC_MOSC
#define SYSCON_RCC2_OSCSRC   SYSCON_RCC2_OSCSRC2_MOSC
#define OSCSRC_FREQUENCY     XTAL_FREQUENCY

/* Use system divider = 4; this corresponds to a system clock frequency
 * of (400 / 2) / 4 = 50MHz
 */

#define LM3S_SYSDIV          4
#define SYSCLK_FREQUENCY     50000000  /* 50MHz */

/* Other RCC settings:
 *
 * - Main and internal oscillators enabled.
 * - PLL and sys dividers not bypassed
 * - PLL not powered down
 * - No auto-clock gating reset
 */

#define LM3S_RCC_VALUE (SYSCON_RCC_OSCSRC | SYSCON_RCC_XTAL | SYSCON_RCC_USESYSDIV | SYSCON_RCC_SYSDIV(LM3S_SYSDIV))

/* RCC2 settings -- RCC2 not used.  Other RCC2 settings
 *
 * - PLL and sys dividers not bypassed.
 * - PLL not powered down
 * - Not using RCC2
 */

#define LM3S_RCC2_VALUE (SYSCON_RCC2_OSCSRC | SYSCON_RCC2_SYSDIV(LM3S_SYSDIV))

/*
 * This should be setup to the board specific rate for the external oscillator
 */
#define CONFIG_LM3S_OSC_RATE		XTAL_FREQUENCY

/*
 * select the UART used on the board. Five UARTs are available labelled
 * UART0 to UART4. Select 0 to 4 for the UART used for uboot.
 */
#define CONFIG_LM3S_UART_BOOT		0

/*
 * Linux machine type
 */
#define CONFIG_LM3S_MACHID		10001

/*
 * LPC1788 built-in FLASH
 */
#define CONFIG_SYS_MAX_FLASH_BANKS 1
#define CONFIG_SYS_MAX_FLASH_SECT  32
#define CONFIG_SYS_FLASH_BASE      LM3S_FLASH_BASE

/*
 * CPU options
 */
#define CONFIG_ARM_THUMB		1
#define CONFIG_SYS_NO_ICACHE		1
#define CONFIG_SYS_NO_DCACHE		1

/*
 * If the CONFIG_USE_BOARD_MPU_TABLE is defined, a MPU setup table needs
 * to be setup for the board that initializes the MPU permissions. If
 * this is undefined, the MPU will be setup with RW for the entire address
 * range.
 *
 * No MPU Initialization required.
 */
#undef CONFIG_USE_BOARD_MPU_TABLE

/*
 * Needed for board init
 */
#undef CONFIG_SKIP_LOWLEVEL_INIT

/*
 * SDRAM physical Memory Map
 */
#define CONFIG_NR_DRAM_BANKS		1
#define PHYS_SDRAM_1			 LM3S_EPI0RAM_BASE /* SDRAM Bank #1 */
#define PHYS_SDRAM_1_SIZE		0x00800000 /* 8 MB SDRAM */
#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM_1

/*
 * 2KHz clock tick
 */
#define	CONFIG_SYS_HZ			1000

/*
 * Address and size of stored environment Data
 */
#define CONFIG_ENV_IS_NOWHERE
#define CONFIG_ENV_SIZE			0x40000 /* 2 blocks */

/*
 * Area and size for malloc
 */
#define CONFIG_SYS_MALLOC_LEN		(CONFIG_ENV_SIZE + 1024 * 1024)

/*
 * SPI configuration
 */
#define CONFIG_SPI
#define CONFIG_LM3S_SPI
#define CONFIG_SSI_POLLWAIT
#define CONFIG_SSI_TXLIMIT 8

/*
 * SPI FLASH configuration
 */
#define CONFIG_SPI_FLASH
#define CONFIG_SPI_FLASH_STMICRO
#define CONFIG_BOARD_TRANSLATE_CS

/*
 * No support for IRQs
 */
#undef CONFIG_USE_IRQ

/*
 * Stack size and global data size
 */
#define CONFIG_STACKSIZE		(32 * 1024)
#define CONFIG_SYS_GBL_DATA_SIZE	128

/*
 * ATAG support
 */
#define CONFIG_CMDLINE_TAG		1
#define CONFIG_SETUP_MEMORY_TAGS	1
#define CONFIG_INITRD_TAG		1

/*
 * System UART selection, valid selections include 0, 1, 2, 3
 */
#define CONFIG_LM3S_UART		0 /* 0 - 2 */
#define CONFIG_BAUDRATE			115200
#define CONFIG_SYS_BAUDRATE_TABLE	{9600, 19200, 38400, 57600, 115200}

#define CONFIG_UART0_BAUD CONFIG_BAUDRATE
#define CONFIG_UART0_BITS 8
#define CONFIG_UART0_PARITY 0
#define CONFIG_UART0_2STOP 0

/*
 * Disable UARTs unused in boot process
 */
#define CONFIG_UART1_DISABLE
#define CONFIG_UART2_DISABLE

/*
 * Default load address for programs
 */
#define CONFIG_SYS_LOAD_ADDR		0xA0100000

/*
 * Default boot delay is 3 seconds
 */
#define CONFIG_BOOTDELAY		3
#define CONFIG_ZERO_BOOTDELAY_CHECK

/*
 * Supported compression algorithms
 */
//#define CONFIG_GZIP

/*
 * Command line configuration.
 */
#include <config_cmd_default.h>

/*
 * Prompt and command buffer setup
 */
#define CONFIG_SYS_LONGHELP
#define	CONFIG_SYS_CBSIZE		256
#define	CONFIG_SYS_PROMPT		"uwic-bl> "
#define	CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE+sizeof(CONFIG_SYS_PROMPT)+16)
#define	CONFIG_SYS_MAXARGS		16
#define CONFIG_SYS_BARGSIZE		CONFIG_SYS_CBSIZE

/*
 * Default range for the memory tests
 */
#define CONFIG_SYS_MEMTEST_START	PHYS_SDRAM_1
#define CONFIG_SYS_MEMTEST_END		(PHYS_SDRAM_1 + PHYS_SDRAM_1_SIZE)

/*
 * BOOTM command
 */
//#define CONFIG_CMD_BOOTM
#define CONFIG_SYS_BOOTM_LEN (4*1024*1024) // Set max gzip linux image to 4 Mbytes (on-board FLASH size)
//#define CONFIG_BOOTM_LINUX

/*
 * Liraries
 */
#define CONFIG_RBTREE
#define CONFIG_LZO

/*
 * File systems
 */
//#define CONFIG_CMD_JFFS2
//#define CONFIG_JFFS2_DEV "serial0"
#define CONFIG_CMD_SF
#define CONFIG_SF_DEFAULT_SPEED 5000000
//#define CONFIG_JFFS2_SERIAL
//#define DEFAULT_EMPTY_SCAN_SIZE 256

#define CONFIG_MTD_DEVICE
#define CONFIG_CMD_MTDPARTS
#define CONFIG_MTD_PARTITIONS
#define CONFIG_CMD_UBI
#define CONFIG_CMD_UBIFS

#define MTDIDS_DEFAULT "serial0=flash"
#define MTDPARTS_DEFAULT "mtdparts=flash:-(root)"

/*
 * Other commands
 */
#define CONFIG_CMD_ENV
#define CONFIG_CMD_ECHO   /* echo arguments   */
#define CONFIG_CMD_RUN    /* run command in env variable  */

#undef CONFIG_CMD_NAND
#undef CONFIG_CMD_BDI    /* bdinfo     */
#undef CONFIG_CMD_BOOTD  /* bootd      */
#undef CONFIG_CMD_CONSOLE  /* coninfo      */
#undef CONFIG_CMD_EDITENV  /* editenv      */
#undef CONFIG_CMD_FPGA   /* FPGA configuration Support */
#undef CONFIG_CMD_IMI    /* iminfo     */
#undef CONFIG_CMD_ITEST  /* Integer (and string) test  */
#undef CONFIG_CMD_FLASH  /* flinfo, erase, protect */
#undef CONFIG_CMD_IMLS   /* List all found images  */
#undef CONFIG_CMD_LOADB  /* loadb      */
#undef CONFIG_CMD_LOADS  /* loads      */
#undef CONFIG_CMD_MEMORY /* md mm nm mw cp cmp crc base loop mtest */
#undef CONFIG_CMD_MISC   /* Misc functions like sleep etc*/
#undef CONFIG_CMD_NET    /* bootp, tftpboot, rarpboot  */
#undef CONFIG_CMD_NFS    /* NFS support      */
#undef CONFIG_CMD_SAVEENV  /* saveenv      */
#undef CONFIG_CMD_SETGETDCR  /* DCR support on 4xx   */
#undef CONFIG_CMD_SOURCE /* "source" command support */
#undef CONFIG_CMD_XIMG   /* Load part of Multi Image */

/*
 * Initial data areas
 */
#define CONFIG_SYS_INIT_RAM_ADDR LM3S_SRAM_BASE
#define CONFIG_SYS_INIT_RAM_SIZE LM3S_SRAM_SIZE
#define CONFIG_SYS_GBL_DATA_OFFSET	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_GBL_DATA_OFFSET)

/*
 * Misc
 */
//#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_UART0_SERIAL_CONSOLE

//#define DEBUG

//#define CONFIG_RAM_BOOT
#define CONFIG_FLASH_BOOT

/*
 * Boot options
 */
//#define CONFIG_BOOTARGS "earlyprintk ignore_loglevel root=/dev/nfs nfsroot=192.168.100.2:/nfsroot ip=192.168.100.10::192.168.100.1:255.255.255.0"
#define CONFIG_BOOTARGS "ubi.mtd=1 root=ubi0:rootfs rootfstype=ubifs"

#if defined(CONFIG_FLASH_BOOT)
#  define CONFIG_BOOTCOMMAND "sf probe 0;ubi part serial0,0;ubifsmount rootfs;ubifsload 0x60400000 /boot/linux.bin;bootm 0x60400000"
#elif defined(CONFIG_RAM_BOOT)
#  define CONFIG_BOOTCOMMAND "bootm 0x60400000"
#endif

#endif /* __EA1788_H */
