/****************************************************************************
 * arch/arm/src/am335x/am335x_lowputc.c
 *
 *   Copyright (C) 2018 Petro Karashchenko. All rights reserved.
 *   Author: Petro Karashchenko <petro.karashchneko@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "arm_arch.h"

#include "am335x_config.h"
#include "am335x_gpio.h"
#include "am335x_pinmux.h"
#include "hardware/am335x_uart.h"

/**************************************************************************
 * Pre-processor Definitions
 **************************************************************************/

/* Select UART parameters for the selected console */

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#  define CONSOLE_BASE     AM335X_UART0_VADDR
#  define CONSOLE_BAUD     CONFIG_UART0_BAUD
#  define CONSOLE_BITS     CONFIG_UART0_BITS
#  define CONSOLE_PARITY   CONFIG_UART0_PARITY
#  define CONSOLE_2STOP    CONFIG_UART0_2STOP
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#  define CONSOLE_BASE     AM335X_UART1_VADDR
#  define CONSOLE_BAUD     CONFIG_UART1_BAUD
#  define CONSOLE_BITS     CONFIG_UART1_BITS
#  define CONSOLE_PARITY   CONFIG_UART1_PARITY
#  define CONSOLE_2STOP    CONFIG_UART1_2STOP
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#  define CONSOLE_BASE     AM335X_UART2_VADDR
#  define CONSOLE_BAUD     CONFIG_UART2_BAUD
#  define CONSOLE_BITS     CONFIG_UART2_BITS
#  define CONSOLE_PARITY   CONFIG_UART2_PARITY
#  define CONSOLE_2STOP    CONFIG_UART2_2STOP
#elif defined(CONFIG_UART3_SERIAL_CONSOLE)
#  define CONSOLE_BASE     AM335X_UART3_VADDR
#  define CONSOLE_BAUD     CONFIG_UART3_BAUD
#  define CONSOLE_BITS     CONFIG_UART3_BITS
#  define CONSOLE_PARITY   CONFIG_UART3_PARITY
#  define CONSOLE_2STOP    CONFIG_UART3_2STOP
#elif defined(CONFIG_UART4_SERIAL_CONSOLE)
#  define CONSOLE_BASE     AM335X_UART4_VADDR
#  define CONSOLE_BAUD     CONFIG_UART4_BAUD
#  define CONSOLE_BITS     CONFIG_UART4_BITS
#  define CONSOLE_PARITY   CONFIG_UART4_PARITY
#  define CONSOLE_2STOP    CONFIG_UART4_2STOP
#elif defined(CONFIG_UART5_SERIAL_CONSOLE)
#  define CONSOLE_BASE     AM335X_UART5_VADDR
#  define CONSOLE_BAUD     CONFIG_UART5_BAUD
#  define CONSOLE_BITS     CONFIG_UART5_BITS
#  define CONSOLE_PARITY   CONFIG_UART5_PARITY
#  define CONSOLE_2STOP    CONFIG_UART5_2STOP
#elif defined(HAVE_SERIAL_CONSOLE)
#  error "No CONFIG_UARTn_SERIAL_CONSOLE Setting"
#endif

/* Get word length setting for the console */

#if CONSOLE_BITS == 5
#  define CONSOLE_LCR_DLS UART_LCR_DLS_5BITS
#elif CONSOLE_BITS == 6
#  define CONSOLE_LCR_DLS UART_LCR_DLS_6BITS
#elif CONSOLE_BITS == 7
#  define CONSOLE_LCR_DLS UART_LCR_DLS_7BITS
#elif CONSOLE_BITS == 8
#  define CONSOLE_LCR_DLS UART_LCR_DLS_8BITS
#elif defined(HAVE_SERIAL_CONSOLE)
#  error "Invalid CONFIG_UARTn_BITS setting for console "
#endif

/* Get parity setting for the console */

#if CONSOLE_PARITY == 0
#  define CONSOLE_LCR_PAR UART_LCR_PARITY_NONE
#elif CONSOLE_PARITY == 1
#  define CONSOLE_LCR_PAR UART_LCR_PARITY_ODD
#elif CONSOLE_PARITY == 2
#  define CONSOLE_LCR_PAR UART_LCR_PARITY_EVEN
#elif CONSOLE_PARITY == 3
#elif defined(HAVE_SERIAL_CONSOLE)
#    error "Invalid CONFIG_UARTn_PARITY setting for CONSOLE"
#endif

/* Get stop-bit setting for the console and UART0-3 */

#if CONSOLE_2STOP != 0
#  define CONSOLE_LCR_STOP UART_LCR_STOP_2BITS
#else
#  define CONSOLE_LCR_STOP UART_LCR_STOP_1BITS
#endif

/* LCR and FCR values for the console */

#define CONSOLE_LCR_VALUE (CONSOLE_LCR_DLS | CONSOLE_LCR_PAR | \
                           CONSOLE_LCR_STOP)
#define CONSOLE_FCR_VALUE (UART_FCR_FIFO_EN | UART_FCR_RFT_60CHAR | \
                           UART_FCR_TFT_56CHAR)

/* SCLK is the UART input clock.
 *
 * Through experimentation, it has been found that the serial clock is
 * OSC24M
 */

#define AM335X_SCLK 48000000

/* The output baud rate is equal to the serial clock (SCLK) frequency divided
 * by sixteen times the value of the baud rate divisor, as follows:
 *
 * baud rate = Fsclk / (16 * divisor).
 */

#define CONSOLE_DL (AM335X_SCLK / (CONSOLE_BAUD << 4))

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 ****************************************************************************/

void arm_lowputc(char ch)
{
#if defined(HAVE_UART_DEVICE) && defined(HAVE_SERIAL_CONSOLE)
  /* Wait for the transmitter to be available */

  while ((getreg32(CONSOLE_BASE + AM335X_UART_LSR_OFFSET) & UART_LSR_THRE) == 0)
    {
    }

  /* Send the character */

  putreg32((uint32_t)ch, CONSOLE_BASE + AM335X_UART_THR_OFFSET);
  while ((getreg32(CONSOLE_BASE + AM335X_UART_LSR_OFFSET) & UART_LSR_THRE) == 0)
    {
    }
#endif
}

/****************************************************************************
 * Name: am335x_lowsetup
 *
 * Description:
 *   This performs basic initialization of the UART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 ****************************************************************************/

void am335x_lowsetup(void)
{
#ifdef HAVE_UART_DEVICE
  /* Enable power and clocking to the UART peripheral */

#warning Missing logic

  /* Configure UART pins for the selected CONSOLE.  If there are multiple
   * pin options for a given UART, the the applicable option must be
   * disambiguated in the board.h header file.
   */

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
  am335x_gpio_config(GPIO_UART0_TXD);
  am335x_gpio_config(GPIO_UART0_RXD);
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
  am335x_gpio_config(GPIO_UART1_TXD);
  am335x_gpio_config(GPIO_UART1_RXD);
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
  am335x_gpio_config(GPIO_UART2_TXD);
  am335x_gpio_config(GPIO_UART2_RXD);
#elif defined(CONFIG_UART3_SERIAL_CONSOLE)
  am335x_gpio_config(GPIO_UART3_TXD);
  am335x_gpio_config(GPIO_UART3_RXD);
#elif defined(CONFIG_UART4_SERIAL_CONSOLE)
  am335x_gpio_config(GPIO_UART4_TXD);
  am335x_gpio_config(GPIO_UART4_RXD);
#elif defined(CONFIG_UART5_SERIAL_CONSOLE)
  am335x_gpio_config(GPIO_UART5_TXD);
  am335x_gpio_config(GPIO_UART5_RXD);
#endif

#if defined(HAVE_SERIAL_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)
  /* Configure the console (only) */

#if 0
  /* Performing Software Reset of the module. */

  putreg32(UART_SYSC_SRESET | getreg32(CONSOLE_BASE + AM335X_UART_SYSC_OFFSET),
          CONSOLE_BASE + AM335X_UART_SYSC_OFFSET);

  /* Wait until the process of Module Reset is complete. */

  while (!(getreg32(CONSOLE_BASE + AM335X_UART_SYSS_OFFSET) & UART_SYSS_RESET_DONE))
    {
    }
#endif

  /* Put UART to disabled mode */

  putreg32(UART_MDR1_MODE_DISABLE, CONSOLE_BASE + AM335X_UART_MDR1_OFFSET);

  /* Enter configuration mode and enable access to Latch Divisor DLAB=1 */

  putreg32(UART_LCR_CONFIG_MODE_B, CONSOLE_BASE + AM335X_UART_LCR_OFFSET);

  /* Set Divisor values to zero to be able to write FCR correctly */

  putreg32(0, CONSOLE_BASE + AM335X_UART_DLH_OFFSET);
  putreg32(0, CONSOLE_BASE + AM335X_UART_DLL_OFFSET);

  /* Enable writing FCR[5:4] */

  putreg32(UART_EFR_ENHANCEDEN, CONSOLE_BASE + AM335X_UART_EFR_OFFSET);

  /* Exit configuration mode and enable access to Latch Divisor DLAB=1 */

  putreg32(UART_LCR_CONFIG_MODE_A, CONSOLE_BASE + AM335X_UART_LCR_OFFSET);

  /* Clear FIFOs */

  putreg32(UART_FCR_RFIFO_CLEAR | UART_FCR_TFIFO_CLEAR,
           CONSOLE_BASE + AM335X_UART_FCR_OFFSET);

  /* Configure the FIFOs */

  putreg32(CONSOLE_FCR_VALUE, CONSOLE_BASE + AM335X_UART_FCR_OFFSET);

  /* Set the BAUD divisor */

  putreg32((CONSOLE_DL >> 8) & UART_DLH_MASK, CONSOLE_BASE + AM335X_UART_DLH_OFFSET);
  putreg32(CONSOLE_DL & UART_DLL_MASK, CONSOLE_BASE + AM335X_UART_DLL_OFFSET);

  /* Clear DLAB */

  putreg32(CONSOLE_LCR_VALUE, CONSOLE_BASE + AM335X_UART_LCR_OFFSET);

  /* Enable Auto-Flow Control in the Modem Control Register */

#if defined(CONFIG_SERIAL_IFLOWCONTROL) || defined(CONFIG_SERIAL_OFLOWCONTROL)
#  warning Missing logic
#endif

  putreg32(UART_MDR1_MODE_16X, CONSOLE_BASE + AM335X_UART_MDR1_OFFSET);
#endif
#endif /* HAVE_UART_DEVICE */
}
