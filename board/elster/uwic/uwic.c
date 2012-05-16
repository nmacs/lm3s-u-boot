#include <common.h>
#include <asm/io.h>
#include <stdint.h>
#include <asm/arch/lm3s_internal.h>
#include <asm/arch/hardware.h>

DECLARE_GLOBAL_DATA_PTR;

static void uwic_setup_pins();

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
  lm3s_gpiowrite(GPIO_CPU_LED, 1);
}

void red_LED_off(void)
{
  lm3s_gpiowrite(GPIO_CPU_LED, 0);
}

void lowlevel_board_init(void)
{
  lm3s_clockconfig(LM3S_RCC_VALUE, LM3S_RCC2_VALUE);

  /* Board specific pin setup */
  uwic_setup_pins();

  serial_init();
  print_cpuinfo();
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
  lm3s_configgpio(GPIO_UART0_RX);
  lm3s_configgpio(GPIO_UART0_TX);

  lm3s_configgpio(GPIO_SSI0_CLK);
  lm3s_configgpio(GPIO_SSI0_FSS);
  lm3s_configgpio(GPIO_SSI0_RX);
  lm3s_configgpio(GPIO_SSI0_TX);
  lm3s_configgpio(GPIO_SSI0_CS_EE);
  lm3s_configgpio(GPIO_SSI0_CS_SF);

  lm3s_configgpio(GPIO_POWER_HOLD);
  lm3s_configgpio(GPIO_POWER_FAIL);
  lm3s_configgpio(GPIO_CPU_LED);

  lm3s_configgpio(GPIO_UART1_TX);
  lm3s_configgpio(GPIO_UART1_RX);

  lm3s_configgpio(GPIO_UART2_TX);
  lm3s_configgpio(GPIO_UART2_RX);
}