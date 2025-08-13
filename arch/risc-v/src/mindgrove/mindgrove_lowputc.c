
/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <arch/board/board.h>

#include "riscv_internal.h"
// #include "riscv_arch.h"
#include "hardware/mindgrove_uart.h"
#include"secure_iot_reg.h"
#include "mindgrove_config.h"
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Select UART parameters for the selected console */

#ifdef HAVE_SERIAL_CONSOLE

#  if defined(CONFIG_UART0_SERIAL_CONSOLE)
#    define MINDGROVE_CONSOLE_BASE        0x11300
#    define MINDGROVE_CONSOLE_BAUD        CONFIG_UART0_BAUD
#    define MINDGROVE_CONSOLE_BITS        CONFIG_UART0_BITS
#    define MINDGROVE_CONSOLE_PARITY      CONFIG_UART0_PARITY
#    define MINDGROVE_CONSOLE_2STOP       CONFIG_UART0_2STOP
#    define MINDGROVE_CONSOLE_TX          MINDGROVE_CONSOLE_BASE+TX_OFFSET
#    define MINDGROVE_CONSOLE_RX          GPIO_UART0_RX
#    define HAVE_UART
#  elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define MINDGROVE_CONSOLE_BASE        MINDGROVE_UART1_BASE
#    define MINDGROVE_CONSOLE_BAUD        CONFIG_UART1_BAUD
#    define MINDGROVE_CONSOLE_BITS        CONFIG_UART1_BITS
#    define MINDGROVE_CONSOLE_PARITY      CONFIG_UART1_PARITY
#    define MINDGROVE_CONSOLE_2STOP       CONFIG_UART1_2STOP
#    define MINDGROVE_CONSOLE_TX          GPIO_UART1_TX
#    define MINDGROVE_CONSOLE_RX          GPIO_UART1_RX
#    define HAVE_UART
#  endif
#endif /* HAVE_CONSOLE */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: riscv_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 ****************************************************************************/

void riscv_lowputc(char ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  /* Wait until the TX data register is empty */
  // uint8_t status;
  // status = getreg8();
  while( getreg16(MINDGROVE_CONSOLE_BASE+UART_STATUS_OFFSET) & STS_TX_FULL);
    // status =  getreg8(MINDGROVE_CONSOLE_BASE+UART_STATUS_OFFSET);
  
  /* Then send the character */

  ((UART_Type*)(MINDGROVE_CONSOLE_BASE))->TX_REG.data_8 = ch;

#endif /* HAVE_CONSOLE */
}

/****************************************************************************
 * Name: shakti_lowsetup
 *
 * Description:
 *   This performs basic initialization of the UART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 ****************************************************************************/
#define STOP_BITS(x) ( (x & 3) << 1) 				/*! 00 - 1 stop bits, 01 - 1.5 stop bits; 10 - 2 stop bits; 11 unused */
#define PARITY(x) ( (x & 3)  << 3 ) 				/*! 00 --- No parity; 01 -Odd Parity; 10 - Even Parity;  11 - Unused */
#define UART_TX_RX_LEN(x)       ( (x & 0x3) << 5) 	/*! Maximum length 32 bits */

void mindgrove_lowsetup(void)
{
#if defined(HAVE_UART)

  /* Enable and configure the selected console device */

#if defined(HAVE_SERIAL_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)

  /* default baudrate set by fpga fabric is 1e6 */
  unsigned int baud_count = (30000000 / (16 * 115200));
  putreg16(baud_count,MINDGROVE_CONSOLE_BASE);
  UART_REG(0)->CTRL = (STOP_BITS(0) | PARITY(0) | UART_TX_RX_LEN(8));
  // printf("hiiii");
  /* Enable TX */
  
#endif /* HAVE_SERIAL_CONSOLE && !CONFIG_SUPPRESS_UART_CONFIG */
#endif /* HAVE_UART */
}
