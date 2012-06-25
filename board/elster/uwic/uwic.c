#include <common.h>
#include <asm/io.h>
#include <stdint.h>
#include <asm/arch/lm3s_internal.h>
#include <asm/arch/hardware.h>

DECLARE_GLOBAL_DATA_PTR;

static void uwic_setup_pins();
static void uwic_setup_sdram();
static void uwic_switch_on_epi_clock();

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
  lm3s_configgpio(GPIO_CPU_LED);
  lm3s_gpiowrite(GPIO_CPU_LED, 1);
}

void red_LED_off(void)
{
  lm3s_configgpio(GPIO_CPU_LED);
  lm3s_gpiowrite(GPIO_CPU_LED, 0);
}

/*void red_LED_toggle(void)
{
	lm3s_configgpio(GPIO_CPU_LED);
	lm3s_gpiowrite(GPIO_CPU_LED, lm3s_gpioread(GPIO_CPU_LED, 0) ? 0 : 1);
}*/

void red_LED_toggle(void)
{
	lm3s_configgpio(GPIO_CPU_LED);

	if( lm3s_gpioread(GPIO_CPU_LED, 0) )
		lm3s_gpiowrite(GPIO_CPU_LED, 0);
	else
		lm3s_gpiowrite(GPIO_CPU_LED, 1);
}

void lowlevel_board_init(void)
{
  lm3s_clockconfig(LM3S_RCC_VALUE, LM3S_RCC2_VALUE);

  uwic_switch_on_epi_clock();

  /* Board specific pin setup */
  uwic_setup_pins();

  /* Setup SDRAM */
  uwic_setup_sdram();
}

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

static void uwic_setup_pins()
{
  lm3s_configgpio(GPIO_EPI0_S0);
  lm3s_configgpio(GPIO_EPI0_S1);
  lm3s_configgpio(GPIO_EPI0_S2);
  lm3s_configgpio(GPIO_EPI0_S3);
  lm3s_configgpio(GPIO_EPI0_S4);
  lm3s_configgpio(GPIO_EPI0_S5);
  lm3s_configgpio(GPIO_EPI0_S6);
  lm3s_configgpio(GPIO_EPI0_S7);
  lm3s_configgpio(GPIO_EPI0_S8);
  lm3s_configgpio(GPIO_EPI0_S9);
  lm3s_configgpio(GPIO_EPI0_S10);
  lm3s_configgpio(GPIO_EPI0_S11);
  lm3s_configgpio(GPIO_EPI0_S12);
  lm3s_configgpio(GPIO_EPI0_S13);
  lm3s_configgpio(GPIO_EPI0_S14);
  lm3s_configgpio(GPIO_EPI0_S15);
  lm3s_configgpio(GPIO_EPI0_S16);
  lm3s_configgpio(GPIO_EPI0_S17);
  lm3s_configgpio(GPIO_EPI0_S18);
  lm3s_configgpio(GPIO_EPI0_S19);
  lm3s_configgpio(GPIO_EPI0_S20);
  lm3s_configgpio(GPIO_EPI0_S21);
  lm3s_configgpio(GPIO_EPI0_S22);
  lm3s_configgpio(GPIO_EPI0_S23);
  lm3s_configgpio(GPIO_EPI0_S24);
  lm3s_configgpio(GPIO_EPI0_S25);
  lm3s_configgpio(GPIO_EPI0_S26);
  lm3s_configgpio(GPIO_EPI0_S27);
  lm3s_configgpio(GPIO_EPI0_S28);
  lm3s_configgpio(GPIO_EPI0_S29);
  lm3s_configgpio(GPIO_EPI0_S30);
  lm3s_configgpio(GPIO_EPI0_S31);

  lm3s_configgpio(GPIO_UART0_RX);
  lm3s_configgpio(GPIO_UART0_TX);

  lm3s_configgpio(GPIO_SSI0_CLK);
  lm3s_configgpio(GPIO_SSI0_RX);
  lm3s_configgpio(GPIO_SSI0_TX);
  lm3s_configgpio(GPIO_SSI0_CS_EE);
  lm3s_configgpio(GPIO_SSI0_CS_SF);
  lm3s_configgpio(GPIO_SSI0_CS_ETH);
  lm3s_configgpio(GPIO_ETH_INTRN);

  lm3s_configgpio(GPIO_POWER_HOLD);
  lm3s_configgpio(GPIO_POWER_FAIL);
  lm3s_configgpio(GPIO_CPU_LED);

  lm3s_configgpio(GPIO_UART1_TX);
  lm3s_configgpio(GPIO_UART1_RX);

  lm3s_configgpio(GPIO_UART2_TX);
  lm3s_configgpio(GPIO_UART2_RX);
}

static void uwic_switch_on_epi_clock()
{
  uint32_t regval;
  regval  = getreg32(LM3S_SYSCON_RCGC1);
  regval |= SYSCON_RCGC1_EPI0;
  putreg32(regval, LM3S_SYSCON_RCGC1);
}

static void uwic_setup_sdram()
{
  uint32_t regval;

  regval = EPI_BAUD_COUNT0(0);
  putreg32(regval, LM3S_EPI0_BAUD);

  regval = EPI_CFG_MODE_SDRAM;
  putreg32(regval, LM3S_EPI0_CFG);

  regval = EPI_SDRAMCFG_SIZE_8MB | EPI_SDRAMCFG_FREQ_30_50MHZ | EPI_SDRAMCFG_RFSH(1024);
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
