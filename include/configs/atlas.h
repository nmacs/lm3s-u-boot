#ifndef __UWIC_H
#define __UWIC_H

#define CONFIG_TM4C
#define CONFIG_ARCH_TM4C12
#include <asm/arch/chip.h>
#include "atlas_pins.h"

/* 
 *SysClock settings 
 */
#define CONFIG_XTAL_FREQUENCY       12000000  /* 12MHz */
#define CONFIG_SYSCLK_FREQUENCY     120000000 /* 120MHz */
#define CONFIG_PIOSC_FREQUENCY      16000000  /* 16MHz */
#define CONFIG_ALT_FREQUENCY        CONFIG_PIOSC_FREQUENCY

/*
 * Linux machine type
 */
#define CONFIG_LM3S_MACHID		10002

/*
 * Built-in FLASH
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
 * DMA
 */
#define CONFIG_STELLARIS_DMA

/*
 * SPI configuration
 */
#define CONFIG_SPI
#define CONFIG_LM3S_SPI
#define CONFIG_SSI_POLLWAIT
#define CONFIG_SSI_TXLIMIT 8
#define CONFIG_BOARD_TRANSLATE_CS

/*
 * SPI FLASH configuration
 */
#define CONFIG_SPI_FLASH
#define CONFIG_SPI_FLASH_STMICRO
#define CONFIG_SPI_FLASH_WINBOND

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
 * System UART selection, valid selections include 0, 1, 2
 */
#define CONFIG_LM3S_UART		1 /* 0 - 2 */
#define CONFIG_BAUDRATE			115200
#define CONFIG_SYS_BAUDRATE_TABLE	{9600, 19200, 38400, 57600, 115200}

#define CONFIG_UART1_BAUD CONFIG_BAUDRATE
#define CONFIG_UART1_BITS 8
#define CONFIG_UART1_PARITY 0
#define CONFIG_UART1_2STOP 0

/*
 * Disable UARTs unused in boot process
 */
#define CONFIG_UART0_DISABLE
#define CONFIG_UART2_DISABLE

/*
 * Default load address for programs
 */
#define CONFIG_SYS_LOAD_ADDR		0x60400000

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
#undef CONFIG_LZMA

/*
 * File systems
 */
#define CONFIG_CMD_CRAMFS
#define CONFIG_CRAMFS_CMDLINE
#define CONFIG_CMD_CRAMFS_ADDR 0x50000

#define CONFIG_CMD_SF
#define CONFIG_SF_DEFAULT_SPEED 80000000

#define CONFIG_MTD_DEVICE
#define CONFIG_CMD_MTDPARTS
#define CONFIG_MTD_PARTITIONS
#define CONFIG_CMD_UBI
#define CONFIG_CMD_UBIFS

#define MTDIDS_DEFAULT "serial0=flash"
#define MTDPARTS_DEFAULT "mtdparts=flash:-(root)"

#define CONFIG_FLASHFORMAT "sf probe 0;sf eraseall;ubi part serial0,0;ubi create rootfs"

/*
 * ETH PHY
 */
#define CONFIG_KS8851
#define CONFIG_KS8851_BUS      1
#define CONFIG_KS8851_CS       0
#define CONFIG_KS8851_MAXHZ    5000000
#define CONFIG_KS8851_SPI_MODE SPI_MODE_0

/*
 * Network
 */
#define CONFIG_CMD_NET
#define CONFIG_NET_MULTI
#define CONFIG_CMD_PING
#define CONFIG_CMD_NFS
#define CONFIG_CMD_DHCP
#define CONFIG_ETHADDR        4C:22:D0:B8:78:AE
#define CONFIG_IPADDR         10.65.100.205
#define CONFIG_GATEWAYIP      10.65.100.1
#define CONFIG_NETMASK        255.255.255.0

/*
 * Other commands
 */
#define CONFIG_CMD_ENV
#define CONFIG_CMD_ECHO   /* echo arguments   */
#define CONFIG_CMD_RUN    /* run command in env variable  */
#define CONFIG_CMD_SECLD

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
#undef CONFIG_CMD_LOADS  /* loads      */
#undef CONFIG_CMD_MEMORY /* md mm nm mw cp cmp crc base loop mtest */
#undef CONFIG_CMD_MISC   /* Misc functions like sleep etc*/
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
#define CONFIG_UART1_SERIAL_CONSOLE

//#define DEBUG
//#define CONFIG_UBIFS_FS_DEBUG

#ifdef DEBUG
# define CONFIG_DISPLAY_FLASH
# define CONFIG_DISPLAY_BUNNER
# define CONFIG_DISPLAY_CPUINFO
#else
# define CONFIG_SYS_CONSOLE_INFO_QUIET
#endif

/*
 * Watchdog
 */
#define CONFIG_WATCHDOG
#define CONFIG_WATCHDOG_TIMEOUT 60 /* seconds */

/*
 * Flash Boot (normal)
 */
#define CONFIG_BOOTCOMMAND    "sf probe 0;ubi part serial0,0;ubifsmount rootfs;" \
                              "secld 0x60400000 ubifs:///boot/linux.bin cramfs:///pub_key;" \
                              "secld 0x60800000 ubifs:///boot/initrd.bin cramfs:///pub_key;" \
                              "bootm 0x60400080 0x60800080"

/*
 * Network Boot
 */
/*#define CONFIG_NFSBOOTCOMMAND "set bootargs debug console=ttyS1,115200 root=/dev/nfs nfsroot=${serverip}:${rootpath} ip=${ipaddr}::${gatewayip}:${netmask} ubi.mtd=1;" \
                              "nfs ${rootpath}/boot/linux.bin;" \
                              "bootm 0x60400080"
*/
#define CONFIG_NFSBOOTCOMMAND "tftp 0x60400000 linux.bin; tftp 0x60800000 initrd.bin; bootm 0x60400080 0x60800080"

#define CONFIG_BOOTFILE      "linux.bin"
#define CONFIG_SERVERIP       255.255.255.255
//#define CONFIG_ROOTPATH       /nfsroot

/*
 * LibTomCrypt & LibTomMath
 */
#define CONFIG_LIBTOMCRYPT
#define LTC_SOURCE
#define LTC_NO_STDIO
#define LTC_NO_WCHAR
#define LTC_NO_SIGNAL
#define LTC_NO_FILE
#define LTM_DESC

#define LTC_NO_CIPHERS
#define LTC_NO_MODES
#define LTC_NO_HASHES
#define LTC_SHA1
#define LTC_NO_MACS
#define LTC_NO_PRNGS
#define LTC_NO_PK
#define LTC_MRSA
#define LTC_DER
#define LTC_NO_CURVES
#define LTC_NO_TEST
#define LTC_NO_PKCS

#define CONFIG_LIBTOMMATH
#define LTM_NO_STDIO

#endif /* __EA1788_H */
