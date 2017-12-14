/************************************************************************************
 * arch/arm/src/lpc54xx/lpc54_lowputc.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Parts of this file were adapted from sample code provided for the LPC54xx
 * family from NXP which has a compatible BSD license.
 *
 *   Copyright (c) 2016, Freescale Semiconductor, Inc.
 *   Copyright (c) 2016 - 2017 , NXP
 *   All rights reserved.
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include "up_arch.h"
#include "up_internal.h"

#include "chip/lpc54_memorymap.h"
#include "chip/lpc54_syscon.h"
#include "chip/lpc54_flexcomm.h"
#include "chip/lpc54_pinmux.h"
#include "chip/lpc54_usart.h"

#include "lpc54_config.h"
#include "lpc54_enableclk.h"
#include "lpc54_clockconfig.h"
#include "lpc54_gpio.h"
#include "lpc54_lowputc.h"

#include <arch/board/board.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#if defined(CONFIG_USART0_SERIAL_CONSOLE)
#  define CONSOLE_BASE        LPC54_FLEXCOMM0_BASE
#  define CONSOLE_BAUD        CONFIG_USART0_BAUD
#  define CONSOLE_FCLK        BOARD_FLEXCOMM0_FCLK
#  define CONSOLE_PARITY      CONFIG_USART0_PARITY
#  define CONSOLE_BITS        CONFIG_USART0_BITS
#  ifdef CONFIG_USART0_2STOP
#    define CONSOLE_STOPBITS2 true
#  else
#    define CONSOLE_STOPBITS2 false
#  endif
#  ifdef CONFIG_USART0_IFLOWCONTROL
#    define CONSOLE_IFLOW     true
#  else
#    define CONSOLE_IFLOW     false
#  endif
#  ifdef CONFIG_USART0_OFLOWCONTROL
#    define CONSOLE_OFLOW     true
#  else
#    define CONSOLE_OFLOW     false
#  endif
#elif defined(CONFIG_USART1_SERIAL_CONSOLE)
#  define CONSOLE_BASE        LPC54_FLEXCOMM1_BASE
#  define CONSOLE_BAUD        CONFIG_USART1_BAUD
#  define CONSOLE_FCLK        BOARD_FLEXCOMM1_FCLK
#  define CONSOLE_PARITY      CONFIG_USART1_PARITY
#  define CONSOLE_BITS        CONFIG_USART1_BITS
#  ifdef CONFIG_USART1_2STOP
#    define CONSOLE_STOPBITS2 true
#  else
#    define CONSOLE_STOPBITS2 false
#  endif
#  ifdef CONFIG_USART1_IFLOWCONTROL
#    define CONSOLE_IFLOW     true
#  else
#    define CONSOLE_IFLOW     false
#  endif
#  ifdef CONFIG_USART1_OFLOWCONTROL
#    define CONSOLE_OFLOW     true
#  else
#    define CONSOLE_OFLOW     false
#  endif
#elif defined(CONFIG_USART2_SERIAL_CONSOLE)
#  define CONSOLE_BASE        LPC54_FLEXCOMM2_BASE
#  define CONSOLE_BAUD        CONFIG_USART2_BAUD
#  define CONSOLE_FCLK        BOARD_FLEXCOMM2_FCLK
#  define CONSOLE_PARITY      CONFIG_USART2_PARITY
#  define CONSOLE_BITS        CONFIG_USART2_BITS
#  ifdef CONFIG_USART2_2STOP
#    define CONSOLE_STOPBITS2 true
#  else
#    define CONSOLE_STOPBITS2 false
#  endif
#  ifdef CONFIG_USART2_IFLOWCONTROL
#    define CONSOLE_IFLOW     true
#  else
#    define CONSOLE_IFLOW     false
#  endif
#  ifdef CONFIG_USART2_OFLOWCONTROL
#    define CONSOLE_OFLOW     true
#  else
#    define CONSOLE_OFLOW     false
#  endif
#elif defined(CONFIG_USART3_SERIAL_CONSOLE)
#  define CONSOLE_BASE        LPC54_FLEXCOMM3_BASE
#  define CONSOLE_BAUD        CONFIG_USART3_BAUD
#  define CONSOLE_FCLK        BOARD_FLEXCOMM3_FCLK
#  define CONSOLE_PARITY      CONFIG_USART3_PARITY
#  define CONSOLE_BITS        CONFIG_USART3_BITS
#  ifdef CONFIG_USART3_2STOP
#    define CONSOLE_STOPBITS2 true
#  else
#    define CONSOLE_STOPBITS2 false
#  endif
#  ifdef CONFIG_USART3_IFLOWCONTROL
#    define CONSOLE_IFLOW     true
#  else
#    define CONSOLE_IFLOW     false
#  endif
#  ifdef CONFIG_USART3_OFLOWCONTROL
#    define CONSOLE_OFLOW     true
#  else
#    define CONSOLE_OFLOW     false
#  endif
#elif defined(CONFIG_USART4_SERIAL_CONSOLE)
#  define CONSOLE_BASE        LPC54_FLEXCOMM4_BASE
#  define CONSOLE_BAUD        CONFIG_USART4_BAUD
#  define CONSOLE_FCLK        BOARD_FLEXCOMM4_FCLK
#  define CONSOLE_PARITY      CONFIG_USART4_PARITY
#  define CONSOLE_BITS        CONFIG_USART4_BITS
#  ifdef CONFIG_USART4_2STOP
#    define CONSOLE_STOPBITS2 true
#  else
#    define CONSOLE_STOPBITS2 false
#  endif
#  ifdef CONFIG_USART4_IFLOWCONTROL
#    define CONSOLE_IFLOW     true
#  else
#    define CONSOLE_IFLOW     false
#  endif
#  ifdef CONFIG_USART4_OFLOWCONTROL
#    define CONSOLE_OFLOW     true
#  else
#    define CONSOLE_OFLOW     false
#  endif
#elif defined(CONFIG_USART5_SERIAL_CONSOLE)
#  define CONSOLE_BASE        LPC54_FLEXCOMM5_BASE
#  define CONSOLE_BAUD        CONFIG_USART5_BAUD
#  define CONSOLE_FCLK        BOARD_FLEXCOMM5_FCLK
#  define CONSOLE_PARITY      CONFIG_USART5_PARITY
#  define CONSOLE_BITS        CONFIG_USART5_BITS
#  ifdef CONFIG_USART5_2STOP
#    define CONSOLE_STOPBITS2 true
#  else
#    define CONSOLE_STOPBITS2 false
#  endif
#  ifdef CONFIG_USART5_IFLOWCONTROL
#    define CONSOLE_IFLOW     true
#  else
#    define CONSOLE_IFLOW     false
#  endif
#  ifdef CONFIG_USART5_OFLOWCONTROL
#    define CONSOLE_OFLOW     true
#  else
#    define CONSOLE_OFLOW     false
#  endif
#elif defined(CONFIG_USART6_SERIAL_CONSOLE)
#  define CONSOLE_BASE        LPC54_FLEXCOMM6_BASE
#  define CONSOLE_BAUD        CONFIG_USART6_BAUD
#  define CONSOLE_FCLK        BOARD_FLEXCOMM6_FCLK
#  define CONSOLE_PARITY      CONFIG_USART6_PARITY
#  define CONSOLE_BITS        CONFIG_USART6_BITS
#  ifdef CONFIG_USART6_2STOP
#    define CONSOLE_STOPBITS2 true
#  else
#    define CONSOLE_STOPBITS2 false
#  endif
#  ifdef CONFIG_USART6_IFLOWCONTROL
#    define CONSOLE_IFLOW     true
#  else
#    define CONSOLE_IFLOW     false
#  endif
#  ifdef CONFIG_USART6_OFLOWCONTROL
#    define CONSOLE_OFLOW     true
#  else
#    define CONSOLE_OFLOW     false
#  endif
#elif defined(CONFIG_USART7_SERIAL_CONSOLE)
#  define CONSOLE_BASE        LPC54_FLEXCOMM7_BASE
#  define CONSOLE_BAUD        CONFIG_USART7_BAUD
#  define CONSOLE_FCLK        BOARD_FLEXCOMM7_FCLK
#  define CONSOLE_PARITY      CONFIG_USART7_PARITY
#  define CONSOLE_BITS        CONFIG_USART7_BITS
#  ifdef CONFIG_USART7_2STOP
#    define CONSOLE_STOPBITS2 true
#  else
#    define CONSOLE_STOPBITS2 false
#  endif
#  ifdef CONFIG_USART7_IFLOWCONTROL
#    define CONSOLE_IFLOW     true
#  else
#    define CONSOLE_IFLOW     false
#  endif
#  ifdef CONFIG_USART7_OFLOWCONTROL
#    define CONSOLE_OFLOW     true
#  else
#    define CONSOLE_OFLOW     false
#  endif
#elif defined(CONFIG_USART8_SERIAL_CONSOLE)
#  define CONSOLE_BASE        LPC54_FLEXCOMM8_BASE
#  define CONSOLE_BAUD        CONFIG_USART8_BAUD
#  define CONSOLE_FCLK        BOARD_FLEXCOMM8_FCLK
#  define CONSOLE_PARITY      CONFIG_USART8_PARITY
#  define CONSOLE_BITS        CONFIG_USART8_BITS
#  ifdef CONFIG_USART8_2STOP
#    define CONSOLE_STOPBITS2 true
#  else
#    define CONSOLE_STOPBITS2 false
#  endif
#  ifdef CONFIG_USART8_IFLOWCONTROL
#    define CONSOLE_IFLOW     true
#  else
#    define CONSOLE_IFLOW     false
#  endif
#  ifdef CONFIG_USART8_OFLOWCONTROL
#    define CONSOLE_OFLOW     true
#  else
#    define CONSOLE_OFLOW     false
#  endif
#elif defined(CONFIG_USART9_SERIAL_CONSOLE)
#  define CONSOLE_BASE        LPC54_FLEXCOMM9_BASE
#  define CONSOLE_BAUD        CONFIG_USART9_BAUD
#  define CONSOLE_FCLK        BOARD_FLEXCOMM9_FCLK
#  define CONSOLE_PARITY      CONFIG_USART9_PARITY
#  define CONSOLE_BITS        CONFIG_USART9_BITS
#  ifdef CONFIG_USART9_2STOP
#    define CONSOLE_STOPBITS2 true
#  else
#    define CONSOLE_STOPBITS2 false
#  endif
#  ifdef CONFIG_USART9_IFLOWCONTROL
#    define CONSOLE_IFLOW     true
#  else
#    define CONSOLE_IFLOW     false
#  endif
#  ifdef CONFIG_USART9_OFLOWCONTROL
#    define CONSOLE_OFLOW     true
#  else
#    define CONSOLE_OFLOW     false
#  endif
#endif

/************************************************************************************
 * Private Data
 ************************************************************************************/

#ifdef HAVE_USART_CONSOLE
/* USART console configuration */

static const struct uart_config_s g_console_config=
{
  .baud      = CONSOLE_BAUD,
  .fclk      = CONSOLE_FCLK,
  .parity    = CONSOLE_PARITY,
  .bits      = CONSOLE_BITS,
  .txlevel   = LPC54_USART_FIFO_DEPTH / 2,
  .rxlevel   = 0,
  .stopbits2 = CONSOLE_STOPBITS2,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .iflow     = CONSOLE_IFLOW,
#endif
#ifdef CONFIG_SERIAL_OFLOWCONTROL
  .oflow     = CONSOLE_OFLOW,
#endif
};
#endif /* HAVE_USART_CONSOLE */

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: lp54_setbaud
 *
 * Description:
 *   Configure the USART BAUD.
 *
 ************************************************************************************/

#ifdef HAVE_USART_DEVICE
static void lp54_setbaud(uintptr_t base, FAR const struct uart_config_s *config)
{
  uint32_t bestdiff = (uint32_t)-1;
  uint32_t bestosr  = 15;
  uint32_t bestbrg  = (uint32_t)-1;
  uint32_t lastosr  = 0;  /* Initialized only to avoid warnings */
  uint32_t lastbrg  = 0;
  uint32_t osr;
  uint32_t brg;
  uint32_t diff;
  uint32_t baud;

  /* Smaller values of OSR can make the sampling position within a data bit less
   * accurate and may potentially cause more noise errors or incorrect data.
   */

  for (osr = bestosr; osr >= 8; osr--)
    {
      brg = (config->fclk / ((osr + 1) * config->baud)) - 1;
      if (brg > 0xffff)
        {
          continue;
        }

      baud = config->fclk / ((osr + 1) * (brg + 1));
      if (config->baud < baud)
        {
          diff =  baud - config->baud;
        }
      else
        {
          diff = config->baud - baud;
        }

      if (diff < bestdiff)
        {
          bestdiff = diff;
          bestosr  = osr;
          bestbrg  = brg;
        }

      lastosr  = osr;
      lastbrg  = brg;
    }

  /* Check for value over range */

  if (bestbrg > 0xffff)
    {
      bestosr = lastosr;
      bestbrg = lastbrg;
    }

  putreg32(bestosr, base + LPC54_USART_OSR_OFFSET);
  putreg32(bestbrg, base + LPC54_USART_BRG_OFFSET);
}
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: lpc54_lowsetup
 *
 * Description:
 *   Called at the very beginning of _start.  Performs low level initialization
 *   including setup of the console USART.  This USART initialization is done
 *   early so that the serial console is available for debugging very early in
 *   the boot sequence.
 *
 ************************************************************************************/

void lpc54_lowsetup(void)
{
  /* Enable the IOCON and all GPIO modules */

  putreg32(SYSCON_AHBCLKCTRL0_IOCON | SYSCON_AHBCLKCTRL0_GPIO0 |
           SYSCON_AHBCLKCTRL0_GPIO1 | SYSCON_AHBCLKCTRL0_GPIO2 |
           SYSCON_AHBCLKCTRL0_GPIO3, LPC54_SYSCON_AHBCLKCTRLSET0);

  putreg32(SYSCON_AHBCLKCTRL2_GPIO4 | SYSCON_AHBCLKCTRL2_GPIO5,
           LPC54_SYSCON_AHBCLKCTRLSET2);

  /* TODO: Configure Fractional Rate Generator in case it is selected as a Flexcomm
   * clock source.
   */

#ifdef HAVE_USART_DEVICE
#ifdef HAVE_USART0
  /* Attach 12 MHz clock to FLEXCOMM0 */

  lpc54_flexcomm0_enableclk();

  /* Set FLEXCOMM0 to the USART peripheral, locking that configuration in place. */

  putreg32(FLEXCOMM_PSELID_PERSEL_USART | FLEXCOMM_PSELID_LOCK,
           LPC54_FLEXCOMM0_PSELID);

   /* Configure USART0 pins (defined in board.h) */

  lpc54_gpio_config(GPIO_USART0_RXD);
  lpc54_gpio_config(GPIO_USART0_TXD);
#ifdef CONFIG_USART0_OFLOWCONTROL
  lpc54_gpio_config(GPIO_USART0_CTS);
#endif
#ifdef CONFIG_USART0_IFLOWCONTROL
  lpc54_gpio_config(GPIO_USART0_RTS);
#endif

  /* Set up the FLEXCOMM0 function clock */

  putreg32(BOARD_FLEXCOMM0_CLKSEL, LPC54_SYSCON_FCLKSEL0);

#endif
#ifdef HAVE_USART1
  /* Attach 12 MHz clock to FLEXCOMM1 */

  lpc54_flexcomm1_enableclk();

  /* Set FLEXCOMM1 to the USART peripheral, locking that configuration in place. */

  putreg32(FLEXCOMM_PSELID_PERSEL_USART | FLEXCOMM_PSELID_LOCK,
           LPC54_FLEXCOMM1_PSELID);

   /* Configure USART1 pins (defined in board.h) */

  lpc54_gpio_config(GPIO_USART1_RXD);
  lpc54_gpio_config(GPIO_USART1_TXD);
#ifdef CONFIG_USART1_OFLOWCONTROL
  lpc54_gpio_config(GPIO_USART1_CTS);
#endif
#ifdef CONFIG_USART1_IFLOWCONTROL
  lpc54_gpio_config(GPIO_USART1_RTS);
#endif

  /* Set up the FLEXCOMM1 function clock */

  putreg32(BOARD_FLEXCOMM1_CLKSEL, LPC54_SYSCON_FCLKSEL1);

#endif
#ifdef HAVE_USART2
  /* Attach 12 MHz clock to FLEXCOMM2 */

  lpc54_flexcomm2_enableclk();

  /* Set FLEXCOMM2 to the USART peripheral, locking that configuration in place. */

  putreg32(FLEXCOMM_PSELID_PERSEL_USART | FLEXCOMM_PSELID_LOCK,
           LPC54_FLEXCOMM2_PSELID);

   /* Configure USART2 pins (defined in board.h) */

  lpc54_gpio_config(GPIO_USART2_RXD);
  lpc54_gpio_config(GPIO_USART2_TXD);
#ifdef CONFIG_USART2_OFLOWCONTROL
  lpc54_gpio_config(GPIO_USART2_CTS);
#endif
#ifdef CONFIG_USART2_IFLOWCONTROL
  lpc54_gpio_config(GPIO_USART2_RTS);
#endif

  /* Set up the FLEXCOMM0 function clock */

  putreg32(BOARD_FLEXCOMM2_CLKSEL, LPC54_SYSCON_FCLKSEL2);

#endif
#ifdef HAVE_USART3
  /* Attach 12 MHz clock to FLEXCOMM3 */

  lpc54_flexcomm3_enableclk();

  /* Set FLEXCOMM3 to the USART peripheral, locking that configuration in place. */

  putreg32(FLEXCOMM_PSELID_PERSEL_USART | FLEXCOMM_PSELID_LOCK,
           LPC54_FLEXCOMM3_PSELID);

   /* Configure USART3 pins (defined in board.h) */

  lpc54_gpio_config(GPIO_USART3_RXD);
  lpc54_gpio_config(GPIO_USART3_TXD);
#ifdef CONFIG_USART3_OFLOWCONTROL
  lpc54_gpio_config(GPIO_USART3_CTS);
#endif
#ifdef CONFIG_USART3_IFLOWCONTROL
  lpc54_gpio_config(GPIO_USART3_RTS);
#endif

  /* Set up the FLEXCOMM3 function clock */

  putreg32(BOARD_FLEXCOMM3_CLKSEL, LPC54_SYSCON_FCLKSEL3);

#endif
#ifdef HAVE_USART4
  /* Attach 12 MHz clock to FLEXCOMM4 */

  lpc54_flexcomm4_enableclk();

  /* Set FLEXCOMM4 to the USART peripheral, locking that configuration in place. */

  putreg32(FLEXCOMM_PSELID_PERSEL_USART | FLEXCOMM_PSELID_LOCK,
         LPC54_FLEXCOMM4_PSELID);

   /* Configure USART4 pins (defined in board.h) */

  lpc54_gpio_config(GPIO_USART4_RXD);
  lpc54_gpio_config(GPIO_USART4_TXD);
#ifdef CONFIG_USART4_OFLOWCONTROL
  lpc54_gpio_config(GPIO_USART4_CTS);
#endif
#ifdef CONFIG_USART4_IFLOWCONTROL
  lpc54_gpio_config(GPIO_USART4_RTS);
#endif

  /* Set up the FLEXCOMM4 function clock */

  putreg32(BOARD_FLEXCOMM4_CLKSEL, LPC54_SYSCON_FCLKSEL4);

#endif
#ifdef HAVE_USART5
  /* Attach 12 MHz clock to FLEXCOMM5 */

  lpc54_flexcomm5_enableclk();

  /* Set FLEXCOMM5 to the USART peripheral, locking that configuration in place. */

  putreg32(FLEXCOMM_PSELID_PERSEL_USART | FLEXCOMM_PSELID_LOCK,
           LPC54_FLEXCOMM5_PSELID);

   /* Configure USART5 pins (defined in board.h) */

  lpc54_gpio_config(GPIO_USART5_RXD);
  lpc54_gpio_config(GPIO_USART5_TXD);
#ifdef CONFIG_USART5_OFLOWCONTROL
  lpc54_gpio_config(GPIO_USART5_CTS);
#endif
#ifdef CONFIG_USART5_IFLOWCONTROL
  lpc54_gpio_config(GPIO_USART5_RTS);
#endif

  /* Set up the FLEXCOMM5 function clock */

  putreg32(BOARD_FLEXCOMM5_CLKSEL, LPC54_SYSCON_FCLKSEL5);

#endif
#ifdef HAVE_USART6
  /* Attach 12 MHz clock to FLEXCOMM6 */

  lpc54_flexcomm6_enableclk();

  /* Set FLEXCOMM6 to the USART peripheral, locking that configuration in place. */

  putreg32(FLEXCOMM_PSELID_PERSEL_USART | FLEXCOMM_PSELID_LOCK,
           LPC54_FLEXCOMM6_PSELID);

   /* Configure USART6 pins (defined in board.h) */

  lpc54_gpio_config(GPIO_USART6_RXD);
  lpc54_gpio_config(GPIO_USART6_TXD);
#ifdef CONFIG_USART6_OFLOWCONTROL
  lpc54_gpio_config(GPIO_USART6_CTS);
#endif
#ifdef CONFIG_USART6_IFLOWCONTROL
  lpc54_gpio_config(GPIO_USART6_RTS);
#endif

  /* Set up the FLEXCOMM6 function clock */

  putreg32(BOARD_FLEXCOMM6_CLKSEL, LPC54_SYSCON_FCLKSEL6);

#endif
#ifdef HAVE_USART7
  /* Attach 12 MHz clock to FLEXCOMM7 */

  lpc54_flexcomm7_enableclk();

  /* Set FLEXCOMM7 to the USART peripheral, locking that configuration in place. */

  putreg32(FLEXCOMM_PSELID_PERSEL_USART | FLEXCOMM_PSELID_LOCK,
           LPC54_FLEXCOMM7_PSELID);

   /* Configure USART7 pins (defined in board.h) */

  lpc54_gpio_config(GPIO_USART7_RXD);
  lpc54_gpio_config(GPIO_USART7_TXD);
#ifdef CONFIG_USART7_OFLOWCONTROL
  lpc54_gpio_config(GPIO_USART7_CTS);
#endif
#ifdef CONFIG_USART7_IFLOWCONTROL
  lpc54_gpio_config(GPIO_USART7_RTS);
#endif

  /* Set up the FLEXCOMM7 function clock */

  putreg32(BOARD_FLEXCOMM7_CLKSEL, LPC54_SYSCON_FCLKSEL7);

#endif
#ifdef HAVE_USART8
  /* Attach 12 MHz clock to FLEXCOMM8 */

  lpc54_flexcomm8_enableclk();

  /* Set FLEXCOMM8 to the USART peripheral, locking that configuration in place. */

  putreg32(FLEXCOMM_PSELID_PERSEL_USART | FLEXCOMM_PSELID_LOCK,
           LPC54_FLEXCOMM8_PSELID);

   /* Configure USART8 pins (defined in board.h) */

  lpc54_gpio_config(GPIO_USART8_RXD);
  lpc54_gpio_config(GPIO_USART8_TXD);
#ifdef CONFIG_USART8_OFLOWCONTROL
  lpc54_gpio_config(GPIO_USART8_CTS);
#endif
#ifdef CONFIG_USART8_IFLOWCONTROL
  lpc54_gpio_config(GPIO_USART8_RTS);
#endif

  /* Set up the FLEXCOMM0 function clock */

  putreg32(BOARD_FLEXCOMM8_CLKSEL, LPC54_SYSCON_FCLKSEL8);

#endif
#ifdef HAVE_USART9
  /* Attach 12 MHz clock to FLEXCOMM9 */

  lpc54_flexcomm9_enableclk();

  /* Set FLEXCOMM9 to the USART peripheral, locking that configuration in place. */

  putreg32(FLEXCOMM_PSELID_PERSEL_USART | FLEXCOMM_PSELID_LOCK,
           LPC54_FLEXCOMM9_PSELID);

   /* Configure USART9 pins (defined in board.h) */

  lpc54_gpio_config(GPIO_USART9_RXD);
  lpc54_gpio_config(GPIO_USART9_TXD);
#ifdef CONFIG_USART9_OFLOWCONTROL
  lpc54_gpio_config(GPIO_USART9_CTS);
#endif
#ifdef CONFIG_USART9_IFLOWCONTROL
  lpc54_gpio_config(GPIO_USART9_RTS);
#endif

  /* Set up the FLEXCOMM9 function clock */

  putreg32(BOARD_FLEXCOMM9_CLKSEL, LPC54_SYSCON_FCLKSEL9);

#endif

#ifdef HAVE_USART_CONSOLE
  /* Configure the console USART (if any) */

  lpc54_usart_configure(CONSOLE_BASE, &g_console_config);

#endif /* HAVE_USART_CONSOLE */
#endif /* HAVE_USART_DEVICE */
}

/************************************************************************************
 * Name: lpc54_usart_configure
 *
 * Description:
 *   Configure a UART for non-interrupt driven operation
 *
 ************************************************************************************/

#ifdef HAVE_USART_DEVICE
void lpc54_usart_configure(uintptr_t base, FAR const struct uart_config_s *config)
{
  uint32_t regval;

  /* Configure baud */

  lp54_setbaud(base, config);

  /* Configure RX and TX FIFOs */
  /* Empty and enable FIFOs */

  regval  = getreg32(base + LPC54_USART_FIFOCFG_OFFSET);
  regval |= (USART_FIFOCFG_ENABLERX | USART_FIFOCFG_EMPTYRX);
  regval |= (USART_FIFOCFG_ENABLETX | USART_FIFOCFG_EMPTYTX);
  putreg32(regval, base + LPC54_USART_FIFOCFG_OFFSET);

  /* Setup trigger level */

  regval  = getreg32(base + LPC54_USART_FIFOTRIG_OFFSET);
  regval &= ~(USART_FIFOTRIG_TXLVL_MASK | USART_FIFOTRIG_RXLVL_MASK);
  regval |= USART_FIFOTRIG_RXLVL(config->rxlevel);
  regval |= USART_FIFOTRIG_TXLVL(config->txlevel);
  putreg32(regval, base + LPC54_USART_FIFOTRIG_OFFSET);

  /* Enable trigger events */

  regval |= (USART_FIFOTRIG_RXLVLENA | USART_FIFOTRIG_TXLVLENA);
  putreg32(regval, base + LPC54_USART_FIFOTRIG_OFFSET);

  /* Setup configuration and enable USART */

  regval  = USART_CFG_ENABLE;

  switch (config->bits)
    {
      case 7:
        regval |= USART_CFG_DATALEN_7BIT;
        break;

      default:
      case 8:
        regval |= USART_CFG_DATALEN_8BIT;
        break;

      case 9:
        regval |= USART_CFG_DATALEN_9BIT;
        break;
    }

  switch (config->parity)
    {
      default:
      case 0:
        regval |= USART_CFG_PARITYSEL_NONE;
        break;

      case 1:
        regval |= USART_CFG_PARITYSEL_ODD;
        break;

      case 2:
        regval |= USART_CFG_PARITYSEL_EVEN;
        break;
    }

  if (config->stopbits2)
    {
      regval |= USART_CFG_STOPLEN;
    }

  putreg32(regval, base + LPC54_USART_CFG_OFFSET);
}
#endif

/****************************************************************************
 * Name: lpc54_usart_disable
 *
 * Description:
 *   Disable a USART.  it will be necessary to again call
 *   lpc54_usart_configure() in order to use this USART channel again.
 *
 ****************************************************************************/

#ifdef HAVE_USART_DEVICE
void lpc54_usart_disable(uintptr_t base)
{
  /* Disable interrupts */

  putreg32(USART_FIFOINT_ALL, base + LPC54_USART_FIFOINTENCLR_OFFSET);

  /* Disable the UART */

  putreg32(0, base + LPC54_USART_CFG_OFFSET);

  /* Disable the FIFOs */

  putreg32(0, base + LPC54_USART_FIFOCFG_OFFSET);
  putreg32(0, base + LPC54_USART_FIFOTRIG_OFFSET);
}
#endif

/****************************************************************************
 * Name: up_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 ****************************************************************************/

void up_lowputc(char ch)
{
#ifdef HAVE_USART_CONSOLE
  irqstate_t flags;

  for (; ; )
    {
      /* Wait for the transmit FIFO to be not full */

      while ((getreg32(CONSOLE_BASE + LPC54_USART_FIFOSTAT_OFFSET) &
             USART_FIFOSTAT_TXNOTFULL) == 0)
        {
        }

      /* Disable interrupts so that the fest test and the transmission are
       * atomic.
       */

      flags = enter_critical_section();
      if ((getreg32(CONSOLE_BASE + LPC54_USART_FIFOSTAT_OFFSET) &
          USART_FIFOSTAT_TXNOTFULL) != 0)
        {
          /* Send the character */

          putreg32((uint32_t)ch, CONSOLE_BASE + LPC54_USART_FIFOWR_OFFSET);
          leave_critical_section(flags);
          return;
        }

      leave_critical_section(flags);
    }
#endif
}
