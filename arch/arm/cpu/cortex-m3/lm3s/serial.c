/**************************************************************************
 * Included Files
 **************************************************************************/

#include <common.h>
#include <stdint.h>
#include <asm/io.h>
#include <asm/arch/hardware.h>
#include <asm/arch/lm3s_internal.h>

/**************************************************************************
 * Pre-processor Definitions
 **************************************************************************/

/* Configuration **********************************************************/

#if LM3S_NUARTS < 2
#  undef  CONFIG_UART1_DISABLE
#  undef  CONFIG_UART1_SERIAL_CONSOLE
#  define CONFIG_UART1_DISABLE 1
#endif

#if LM3S_NUARTS < 3
#  undef  CONFIG_UART2_DISABLE
#  undef  CONFIG_UART2_SERIAL_CONSOLE
#  define CONFIG_UART2_DISABLE 1
#endif

/* Is there a serial console? */

#if defined(CONFIG_UART0_SERIAL_CONSOLE) && !defined(CONFIG_UART0_DISABLE)
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_UART1_SERIAL_CONSOLE) && !defined(CONFIG_UART1_DISABLE)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#elif defined(CONFIG_UART2_SERIAL_CONSOLE) && !defined(CONFIG_UART2_DISABLE)
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  define HAVE_CONSOLE 1
#else
#  warning "No valid CONFIG_UARTn_SERIAL_CONSOLE Setting"
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef HAVE_CONSOLE
#endif

/* Select UART parameters for the selected console */

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#  define LM3S_CONSOLE_BASE     LM3S_UART0_BASE
#  define LM3S_CONSOLE_BAUD     CONFIG_UART0_BAUD
#  define LM3S_CONSOLE_BITS     CONFIG_UART0_BITS
#  define LM3S_CONSOLE_PARITY   CONFIG_UART0_PARITY
#  define LM3S_CONSOLE_2STOP    CONFIG_UART0_2STOP
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#  define LM3S_CONSOLE_BASE     LM3S_UART1_BASE
#  define LM3S_CONSOLE_BAUD     CONFIG_UART1_BAUD
#  define LM3S_CONSOLE_BITS     CONFIG_UART1_BITS
#  define LM3S_CONSOLE_PARITY   CONFIG_UART1_PARITY
#  define LM3S_CONSOLE_2STOP    CONFIG_UART1_2STOP
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#  define LM3S_CONSOLE_BASE     LM3S_UART2_BASE
#  define LM3S_CONSOLE_BAUD     CONFIG_UART2_BAUD
#  define LM3S_CONSOLE_BITS     CONFIG_UART2_BITS
#  define LM3S_CONSOLE_PARITY   CONFIG_UART2_PARITY
#  define LM3S_CONSOLE_2STOP    CONFIG_UART2_2STOP
#else
#  error "No CONFIG_UARTn_SERIAL_CONSOLE Setting"
#endif

/* Get LCRH settings */

#if LM3S_CONSOLE_BITS == 5
#  define UART_LCRH_NBITS UART_LCRH_WLEN_5BITS
#elif LM3S_CONSOLE_BITS == 6
#  define UART_LCRH_NBITS UART_LCRH_WLEN_6BITS
#elif LM3S_CONSOLE_BITS == 7
#  define UART_LCRH_NBITS UART_LCRH_WLEN_7BITS
#elif LM3S_CONSOLE_BITS == 8
#  define UART_LCRH_NBITS UART_LCRH_WLEN_8BITS
#else
#  error "Number of bits not supported"
#endif

#if LM3S_CONSOLE_PARITY == 0
#  define UART_LCRH_PARITY (0)
#elif LM3S_CONSOLE_PARITY == 1
#  define UART_LCRH_PARITY UART_LCRH_PEN
#elif LM3S_CONSOLE_PARITY == 2
#  define UART_LCRH_PARITY (UART_LCRH_PEN|UART_LCRH_EPS)
#else
#  error "Invalid parity selection"
#endif

#if LM3S_CONSOLE_2STOP != 0
#  define UART_LCRH_NSTOP UART_LCRH_STP2
#else
#  define UART_LCRH_NSTOP (0)
#endif

#define UART_LCRH_VALUE (UART_LCRH_NBITS|UART_LCRH_PARITY|UART_LCRH_NSTOP|UART_LCRH_FEN)

/* Calculate BAUD rate from the SYS clock:
 *
 * "The baud-rate divisor is a 22-bit number consisting of a 16-bit integer and a 6-bit
 *  fractional part. The number formed by these two values is used by the baud-rate generator
 *  to determine the bit period. Having a fractional baud-rate divider allows the UART to
 *  generate all the standard baud rates.
 *
 * "The 16-bit integer is loaded through the UART Integer Baud-Rate Divisor (UARTIBRD)
 *  register ... and the 6-bit fractional part is loaded with the UART Fractional Baud-Rate
 *  Divisor (UARTFBRD) register... The baud-rate divisor (BRD) has the following relationship
 *  to the system clock (where BRDI is the integer part of the BRD and BRDF is the fractional
 *  part, separated by a decimal place.):
 *
 *    "BRD = BRDI + BRDF = UARTSysClk / (16 * Baud Rate)
 *
 * "where UARTSysClk is the system clock connected to the UART. The 6-bit fractional number
 *  (that is to be loaded into the DIVFRAC bit field in the UARTFBRD register) can be calculated
 *  by taking the fractional part of the baud-rate divisor, multiplying it by 64, and adding 0.5
 *  to account for rounding errors:
 *
 *    "UARTFBRD[DIVFRAC] = integer(BRDF * 64 + 0.5)
 *
 * "The UART generates an internal baud-rate reference clock at 16x the baud-rate (referred
 *  to as Baud16). This reference clock is divided by 16 to generate the transmit clock, and is
 *  used for error detection during receive operations.
 *
 * "Along with the UART Line Control, High Byte (UARTLCRH) register ..., the UARTIBRD and
 *  UARTFBRD registers form an internal 30-bit register. This internal register is only
 *  updated when a write operation to UARTLCRH is performed, so any changes to the baud-rate
 *  divisor must be followed by a write to the UARTLCRH register for the changes to take effect. ..."
 */

#define LM3S_BRDDEN     (16 * LM3S_CONSOLE_BAUD)
#define LM3S_BRDI       (SYSCLK_FREQUENCY / LM3S_BRDDEN)
#define LM3S_REMAINDER  (SYSCLK_FREQUENCY - LM3S_BRDDEN * LM3S_BRDI)
#define LM3S_DIVFRAC    ((LM3S_REMAINDER * 64 + (LM3S_BRDDEN/2)) / LM3S_BRDDEN)

/* For example: LM3S_CONSOLE_BAUD = 115,200, SYSCLK_FREQUENCY = 50,000,000:
 *
 * LM3S_BRDDEN    = (16 * 115,200)                           = 1,843,200
 * LM3S_BRDI      = 50,000,000 / 1,843,200                   = 27
 * LM3S_REMAINDER = 50,000,000 - 1,843,200 * 27              = 233,600
 * LM3S_DIVFRAC   = (233,600 * 64 + 921,600) / 1,843,200     = 8
 *
 * Which should yied BAUD = 50,000,000 / (16 * (27 + 8/64)) = 115207.37
 */

DECLARE_GLOBAL_DATA_PTR;

/*
 * Set UART baudrate
 */
void serial_setbrg(void)
{

}

/*
 * it is the responsiblity of the board code to setup the proper pin
 * muxing for the selected UART
 */
int serial_init(void)
{
	uint32_t regval;
#if defined(HAVE_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)
  uint32_t ctl;
#endif

  /* Enable the selected UARTs and configure GPIO pins to need by the
   * the selected UARTs.  NOTE: The serial driver later depends on
   * this pin configuration -- whether or not a serial console is selected.
   */

regval  = getreg32(LM3S_SYSCON_RCGC1);

#ifndef CONFIG_UART0_DISABLE
  regval |= SYSCON_RCGC1_UART0;
#endif

#ifndef CONFIG_UART1_DISABLE
  regval |= SYSCON_RCGC1_UART1;
#endif

#ifndef CONFIG_UART2_DISABLE
  regval |= SYSCON_RCGC1_UART2;
#endif

/* NOTE: put LM3S_SYSCON_RCGC1 twice to workaround LM3S bug */
putreg32(regval, LM3S_SYSCON_RCGC1);
putreg32(regval, LM3S_SYSCON_RCGC1);

  /* Enable the selected console device */

#if defined(HAVE_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)
  /* Disable the UART by clearing the UARTEN bit in the UART CTL register */

  ctl = getreg32(LM3S_CONSOLE_BASE+LM3S_UART_CTL_OFFSET);
  ctl &= ~UART_CTL_UARTEN;
  putreg32(ctl, LM3S_CONSOLE_BASE+LM3S_UART_CTL_OFFSET);

  /* Write the integer portion of the BRD to the UART IBRD register */

  putreg32(LM3S_BRDI, LM3S_CONSOLE_BASE+LM3S_UART_IBRD_OFFSET);

  /* Write the fractional portion of the BRD to the UART FBRD register */

  putreg32(LM3S_DIVFRAC, LM3S_CONSOLE_BASE+LM3S_UART_FBRD_OFFSET);

  /* Write the desired serial parameters to the UART LCRH register */

  putreg32(UART_LCRH_VALUE, LM3S_CONSOLE_BASE+LM3S_UART_LCRH_OFFSET);

  /* Enable the UART by setting the UARTEN bit in the UART CTL register */

  ctl |= (UART_CTL_UARTEN|UART_CTL_TXE|UART_CTL_RXE);
  putreg32(ctl, LM3S_CONSOLE_BASE+LM3S_UART_CTL_OFFSET);
#endif

	return 0;
}

int serial_getc(void)
{
	/* Wait for a character from the UART */
	while ((getreg32(LM3S_CONSOLE_BASE + LM3S_UART_FR_OFFSET) & UART_FR_RXFE));

	return (int)(getreg32(LM3S_CONSOLE_BASE + LM3S_UART_DR_OFFSET) & UART_DR_DATA_MASK);
}

void serial_putc(const char ch)
{
  /* Wait until the TX FIFO is not full */
  while ((getreg32(LM3S_CONSOLE_BASE + LM3S_UART_FR_OFFSET) & UART_FR_TXFF) != 0);

  /* Then send the character */
  putreg32((uint32_t)ch, LM3S_CONSOLE_BASE + LM3S_UART_DR_OFFSET);

  /* If \n, also do \r */
  if (ch == '\n')
    serial_putc('\r');
}

int serial_tstc(void)
{
	/* Test for a character from the UART */
	return (getreg32(LM3S_CONSOLE_BASE + LM3S_UART_FR_OFFSET) & UART_FR_RXFE) == 0;
}

void serial_puts(const char *s)
{
	while (*s)
		serial_putc (*s++);
}

