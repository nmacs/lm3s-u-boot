#include <common.h>
#include <asm/io.h>
#include <stdint.h>
#include <spi.h>
#include <asm/arch/internal.h>
#include <asm/arch/hardware.h>

DECLARE_GLOBAL_DATA_PTR;

static void setup_pins(void);
static void setup_sdram(void);

//extern void cm3_reset_cpu(ulong addr);
#if 0
/*
 * Reset command causes a crash on unmodified u-boot code, use
 * cm_reset instead.
 */
void reset_cpu(ulong addr)
{
	cm3_reset_cpu(0x0);

}
#else
void reset_cpu(ulong addr)
{
	cm3_reset_cpu(0x0);
}

int cm_reset(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	puts ("resetting ...\n");

	udelay (50000);				/* wait 50 ms */

	disable_interrupts();
	cm3_reset_cpu(0x0);

	/*NOTREACHED*/
	return 0;
}

U_BOOT_CMD(
	cmreset, 1, 0,	cm_reset,
	"Perform RESET of the CPU",
	""
);
#endif

void red_LED_on(void)
{
  configgpio(GPIO_CPU_LED);
  gpiowrite(GPIO_CPU_LED, 1);
}

void red_LED_off(void)
{
  configgpio(GPIO_CPU_LED);
  gpiowrite(GPIO_CPU_LED, 0);
}

void red_LED_toggle(void)
{
	configgpio(GPIO_CPU_LED);

	if( gpioread(GPIO_CPU_LED, 0) )
		gpiowrite(GPIO_CPU_LED, 0);
	else
		gpiowrite(GPIO_CPU_LED, 1);
}

static void sleep_if_outage(void)
{
  while(gpioread(GPIO_POWER_FAIL, 0) != 0) {}
}

void lowlevel_board_init(void)
{
	setup_pins();
	sleep_if_outage();
	clockconfig();
	epi_clock_ctrl(SYS_ENABLE_CLOCK);
	setup_sdram();
}

#ifdef CONFIG_KS8851
int board_eth_init(void)
{
	return ks8851_initialize(CONFIG_KS8851_BUS, CONFIG_KS8851_CS, CONFIG_KS8851_MAXHZ, CONFIG_KS8851_SPI_MODE);
}
#endif

int board_init(void)
{
	gd->bd->bi_arch_number = CONFIG_LM3S_MACHID;
	/* location of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM_1 + 0x100;
	return 0;
}

int misc_init_r(void)
{
	return 0;
}

int dram_init(void)
{
	gd->ram_size = PHYS_SDRAM_1_SIZE;
	return (0);
}

static void setup_pins(void)
{
  configgpio(GPIO_EPI0_S0);
  configgpio(GPIO_EPI0_S1);
  configgpio(GPIO_EPI0_S2);
  configgpio(GPIO_EPI0_S3);
  configgpio(GPIO_EPI0_S4);
  configgpio(GPIO_EPI0_S5);
  configgpio(GPIO_EPI0_S6);
  configgpio(GPIO_EPI0_S7);
  configgpio(GPIO_EPI0_S8);
  configgpio(GPIO_EPI0_S9);
  configgpio(GPIO_EPI0_S10);
  configgpio(GPIO_EPI0_S11);
  configgpio(GPIO_EPI0_S12);
  configgpio(GPIO_EPI0_S13);
  configgpio(GPIO_EPI0_S14);
  configgpio(GPIO_EPI0_S15);
  configgpio(GPIO_EPI0_S16);
  configgpio(GPIO_EPI0_S17);
  configgpio(GPIO_EPI0_S18);
  configgpio(GPIO_EPI0_S19);
  configgpio(GPIO_EPI0_S24);
  configgpio(GPIO_EPI0_S25);
  configgpio(GPIO_EPI0_S26);
  configgpio(GPIO_EPI0_S27);
  configgpio(GPIO_EPI0_S28);
  configgpio(GPIO_EPI0_S29);
  configgpio(GPIO_EPI0_S30);
  configgpio(GPIO_EPI0_S31);
	
	configgpio(GPIO_VUNREG_MON);
	configgpio(GPIO_POWER_FAIL);
	
	configgpio(GPIO_CAP_CHRG);
	configgpio(GPIO_CAP_STATUS);

	configgpio(GPIO_TL_PWR_ON);
	configgpio(GPIO_TL_SHUTDOWN);
	configgpio(GPIO_TL_PWRMON);
	configgpio(GPIO_TL_SPI_MRDY);
	configgpio(GPIO_TL_SPI_SRDY);
	configgpio(GPIO_TL_IF_EN);

  configgpio(GPIO_UART0_RX);
  configgpio(GPIO_UART0_TX);
	configgpio(GPIO_UART0_DTR);
	configgpio(GPIO_UART0_RTS);
	configgpio(GPIO_UART0_CTS);
	
	configgpio(GPIO_UART1_TX);
  configgpio(GPIO_UART1_RX);
	
	configgpio(GPIO_UART2_TX);
  configgpio(GPIO_UART2_RX);

  configgpio(GPIO_SSI0_CLK);
  configgpio(GPIO_SSI0_RX);
  configgpio(GPIO_SSI0_TX);
	
  configgpio(GPIO_SSI0_CS_EE);
  configgpio(GPIO_SSI0_CS_SF);
  configgpio(GPIO_SSI0_CS_ETH);
	
  configgpio(GPIO_ETH_INTRN);

  configgpio(GPIO_CPU_LED);
}

static void setup_sdram(void)
{
  uint32_t regval;

  regval = EPI_BAUD_COUNT0(0);
  putreg32(regval, LM3S_EPI0_BAUD);

  regval = EPI_CFG_MODE_SDRAM;
  putreg32(regval, LM3S_EPI0_CFG);

  regval = EPI_SDRAMCFG_SIZE_8MB | EPI_SDRAMCFG_FREQ_50_100MHZ | EPI_SDRAMCFG_RFSH(1024);
  putreg32(regval, LM3S_EPI0_SDRAMCFG);

  regval = EPI_ADDRMAP_ERADR_6 | EPI_ADDRMAP_ERSZ_16MB;
  putreg32(regval, LM3S_EPI0_ADDRMAP);

  /* Waite for SDRAM is ready */
  while( (getreg32(LM3S_EPI0_STAT) & EPI_STAT_INITSEQ_MASK) ) {}
}

int board_translate_cs(unsigned int* translated_cs, unsigned int cs)
{
  switch(cs)
  {
  case 0:
    *translated_cs = GPIO_SSI0_CS_SF;
    return 0;
  case 1:
    *translated_cs = GPIO_SSI0_CS_EE;
    return 0;
  case 2:
    *translated_cs = GPIO_SSI0_CS_ETH;
    return 0;
  default:
    return -1;
  }
}

extern size_t *_u_boot_bss_start;
extern size_t *_u_boot_bss_end;

extern size_t *_u_boot_data_load_start;
extern size_t *_u_boot_data_start;
extern size_t *_u_boot_data_end;

void init_data_sections()
{
  memcpy(_u_boot_data_start, _u_boot_data_load_start,
      (char*)_u_boot_data_end - (char*)_u_boot_data_start);
  memset(_u_boot_bss_start, 0, (char*)_u_boot_bss_end - (char*)_u_boot_bss_start);
}
