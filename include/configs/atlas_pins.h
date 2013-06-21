#ifndef __UWIC_PINS_H
#define __UWIC_PINS_H

#define GPIO_SSI0_CLK    (GPIO_FUNC_PFIO      | GPIO_PORTA | GPIO_DF(15) | 2)       /* PA2: SSI0 clock (SSI0Clk) */
#define GPIO_SSI0_RX     (GPIO_FUNC_PFINPUT   | GPIO_PORTA | GPIO_DF(15) | 4)       /* PA4: SSI0 receive (SSI0Rx) */
#define GPIO_SSI0_TX     (GPIO_FUNC_PFOUTPUT  | GPIO_PORTA | GPIO_DF(15) | 5)       /* PA5: SSI0 transmit (SSI0Tx) */
#define GPIO_SSI0_CS_SF  (GPIO_FUNC_OUTPUT    | GPIO_PORTF | 3 | GPIO_VALUE_ONE)    /* PA7: SSI0 Serial Flash chip select */
#define GPIO_SSI0_CS_EE  (GPIO_FUNC_OUTPUT    | GPIO_PORTF | 2 | GPIO_VALUE_ONE)    /* PA6: SSI0 EEPROM chip select */
#define GPIO_SSI0_CS_ETH (GPIO_FUNC_OUTPUT    | GPIO_PORTA | 3 | GPIO_VALUE_ONE)    /* PA3: SSI0 ETH chip select */

#define GPIO_ETH_INTRN   (GPIO_FUNC_INTERRUPT | GPIO_PORTF | 1 | GPIO_INT_LOWLEVEL)/* PG5: ETH chip interrupt */

#define GPIO_CPU_LED     (GPIO_FUNC_OUTPUT    | GPIO_PORTE | 0)                    /* PB6: CPU LED */

#define GPIO_UART0_TX    (GPIO_FUNC_PFOUTPUT  | GPIO_PORTA | GPIO_DF(1) | 1 | GPIO_VALUE_ONE)       /* PA1: UART 0 transmit (U0Tx) */
#define GPIO_UART0_RX    (GPIO_FUNC_PFINPUT   | GPIO_PORTA | GPIO_DF(1) | 0 | GPIO_PADTYPE_STDWPU)  /* PA0: UART 0 receive (U0Rx) */
#define GPIO_UART0_DTR   (GPIO_FUNC_OUTPUT    | GPIO_PORTF | 4 | GPIO_VALUE_ZERO)
#define GPIO_UART0_RTS   (GPIO_FUNC_PFOUTPUT  | GPIO_PORTH | GPIO_DF(2) | 0)
#define GPIO_UART0_CTS   (GPIO_FUNC_PFOUTPUT  | GPIO_PORTH | GPIO_DF(2) | 1)

#define GPIO_UART1_TX    (GPIO_FUNC_PFOUTPUT | GPIO_PORTB | GPIO_DF(1) | 1 | GPIO_VALUE_ONE)        /* PB0: UART 1 transmit (U1Tx) */
#define GPIO_UART1_RX    (GPIO_FUNC_PFINPUT  | GPIO_PORTB | GPIO_DF(1) | 0 | GPIO_PADTYPE_STDWPU)   /* PB1: UART 1 receive (U1Rx) */

#define GPIO_UART2_TX    (GPIO_FUNC_PFOUTPUT | GPIO_PORTD | GPIO_DF(1) | 5 | GPIO_VALUE_ONE)        /* PE4: UART 2 transmit (U2Tx) */
#define GPIO_UART2_RX    (GPIO_FUNC_PFINPUT  | GPIO_PORTD | GPIO_DF(1) | 4 | GPIO_PADTYPE_STDWPU)   /* PD0: UART 2 receive (U2Rx) */

#define GPIO_EPI0_S0     (GPIO_FUNC_PFIO     | GPIO_PORTK | GPIO_DF(15) | 0 | GPIO_STRENGTH_8MA | GPIO_PADTYPE_STD)
#define GPIO_EPI0_S1     (GPIO_FUNC_PFIO     | GPIO_PORTK | GPIO_DF(15) | 1 | GPIO_STRENGTH_8MA | GPIO_PADTYPE_STD)
#define GPIO_EPI0_S2     (GPIO_FUNC_PFIO     | GPIO_PORTK | GPIO_DF(15) | 2 | GPIO_STRENGTH_8MA | GPIO_PADTYPE_STD)
#define GPIO_EPI0_S3     (GPIO_FUNC_PFIO     | GPIO_PORTK | GPIO_DF(15) | 3 | GPIO_STRENGTH_8MA | GPIO_PADTYPE_STD)
#define GPIO_EPI0_S4     (GPIO_FUNC_PFIO     | GPIO_PORTC | GPIO_DF(15) | 7 | GPIO_STRENGTH_8MA | GPIO_PADTYPE_STD)
#define GPIO_EPI0_S5     (GPIO_FUNC_PFIO     | GPIO_PORTC | GPIO_DF(15) | 6 | GPIO_STRENGTH_8MA | GPIO_PADTYPE_STD)
#define GPIO_EPI0_S6     (GPIO_FUNC_PFIO     | GPIO_PORTC | GPIO_DF(15) | 5 | GPIO_STRENGTH_8MA | GPIO_PADTYPE_STD)
#define GPIO_EPI0_S7     (GPIO_FUNC_PFIO     | GPIO_PORTC | GPIO_DF(15) | 4 | GPIO_STRENGTH_8MA | GPIO_PADTYPE_STD)
#define GPIO_EPI0_S8     (GPIO_FUNC_PFIO     | GPIO_PORTA | GPIO_DF(15) | 6 | GPIO_STRENGTH_8MA | GPIO_PADTYPE_STD)
#define GPIO_EPI0_S9     (GPIO_FUNC_PFIO     | GPIO_PORTA | GPIO_DF(15) | 7 | GPIO_STRENGTH_8MA | GPIO_PADTYPE_STD)
#define GPIO_EPI0_S10    (GPIO_FUNC_PFIO     | GPIO_PORTG | GPIO_DF(15) | 1 | GPIO_STRENGTH_8MA | GPIO_PADTYPE_STD)
#define GPIO_EPI0_S11    (GPIO_FUNC_PFIO     | GPIO_PORTG | GPIO_DF(15) | 0 | GPIO_STRENGTH_8MA | GPIO_PADTYPE_STD)
#define GPIO_EPI0_S12    (GPIO_FUNC_PFIO     | GPIO_PORTM | GPIO_DF(15) | 3 | GPIO_STRENGTH_8MA | GPIO_PADTYPE_STD)
#define GPIO_EPI0_S13    (GPIO_FUNC_PFIO     | GPIO_PORTM | GPIO_DF(15) | 2 | GPIO_STRENGTH_8MA | GPIO_PADTYPE_STD)
#define GPIO_EPI0_S14    (GPIO_FUNC_PFIO     | GPIO_PORTM | GPIO_DF(15) | 1 | GPIO_STRENGTH_8MA | GPIO_PADTYPE_STD)
#define GPIO_EPI0_S15    (GPIO_FUNC_PFIO     | GPIO_PORTM | GPIO_DF(15) | 0 | GPIO_STRENGTH_8MA | GPIO_PADTYPE_STD)
#define GPIO_EPI0_S16    (GPIO_FUNC_PFIO     | GPIO_PORTL | GPIO_DF(15) | 0 | GPIO_STRENGTH_8MA | GPIO_PADTYPE_STD)
#define GPIO_EPI0_S17    (GPIO_FUNC_PFIO     | GPIO_PORTL | GPIO_DF(15) | 1 | GPIO_STRENGTH_8MA | GPIO_PADTYPE_STD)
#define GPIO_EPI0_S18    (GPIO_FUNC_PFIO     | GPIO_PORTL | GPIO_DF(15) | 2 | GPIO_STRENGTH_8MA | GPIO_PADTYPE_STD)
#define GPIO_EPI0_S19    (GPIO_FUNC_PFIO     | GPIO_PORTL | GPIO_DF(15) | 3 | GPIO_STRENGTH_8MA | GPIO_PADTYPE_STD)
#define GPIO_EPI0_S24    (GPIO_FUNC_PFIO     | GPIO_PORTK | GPIO_DF(15) | 7 | GPIO_STRENGTH_8MA | GPIO_PADTYPE_STD)
#define GPIO_EPI0_S25    (GPIO_FUNC_PFIO     | GPIO_PORTK | GPIO_DF(15) | 6 | GPIO_STRENGTH_8MA | GPIO_PADTYPE_STD)
#define GPIO_EPI0_S26    (GPIO_FUNC_PFIO     | GPIO_PORTL | GPIO_DF(15) | 4 | GPIO_STRENGTH_8MA | GPIO_PADTYPE_STD)
#define GPIO_EPI0_S27    (GPIO_FUNC_PFIO     | GPIO_PORTB | GPIO_DF(15) | 2 | GPIO_STRENGTH_8MA | GPIO_PADTYPE_STD)
#define GPIO_EPI0_S28    (GPIO_FUNC_PFIO     | GPIO_PORTB | GPIO_DF(15) | 3 | GPIO_STRENGTH_8MA | GPIO_PADTYPE_STD)
#define GPIO_EPI0_S29    (GPIO_FUNC_PFIO     | GPIO_PORTP | GPIO_DF(15) | 2 | GPIO_STRENGTH_8MA | GPIO_PADTYPE_STD)
#define GPIO_EPI0_S30    (GPIO_FUNC_PFIO     | GPIO_PORTP | GPIO_DF(15) | 3 | GPIO_STRENGTH_8MA | GPIO_PADTYPE_STD)
#define GPIO_EPI0_S31    (GPIO_FUNC_PFIO     | GPIO_PORTK | GPIO_DF(15) | 5 | GPIO_STRENGTH_8MA | GPIO_PADTYPE_STD)

#endif /* __UWIC_PINS_H */