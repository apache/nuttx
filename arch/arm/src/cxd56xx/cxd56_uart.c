/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_uart.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include <errno.h>

#include "arm_internal.h"
#include "arm_arch.h"

#include "chip.h"
#include "cxd56_config.h"
#include "cxd56_clock.h"
#include "cxd56_uart.h"
#include "cxd56_pinconfig.h"
#include "cxd56_powermgr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Select UART parameters for the selected console */

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
 #define CONSOLE_BASE     CXD56_UART0_BASE
 #define CONSOLE_BASEFREQ BOARD_UART0_BASEFREQ
 #define CONSOLE_BAUD     CONFIG_UART0_BAUD
 #define CONSOLE_BITS     CONFIG_UART0_BITS
 #define CONSOLE_PARITY   CONFIG_UART0_PARITY
 #define CONSOLE_2STOP    CONFIG_UART0_2STOP
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
 #define CONSOLE_BASE     CXD56_UART1_BASE
 #define CONSOLE_BASEFREQ BOARD_UART1_BASEFREQ
 #define CONSOLE_BAUD     CONFIG_UART1_BAUD
 #define CONSOLE_BITS     CONFIG_UART1_BITS
 #define CONSOLE_PARITY   CONFIG_UART1_PARITY
 #define CONSOLE_2STOP    CONFIG_UART1_2STOP
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
 #define CONSOLE_BASE     CXD56_UART2_BASE
 #define CONSOLE_BASEFREQ BOARD_UART2_BASEFREQ
 #define CONSOLE_BAUD     CONFIG_UART2_BAUD
 #define CONSOLE_BITS     CONFIG_UART2_BITS
 #define CONSOLE_PARITY   CONFIG_UART2_PARITY
 #define CONSOLE_2STOP    CONFIG_UART2_2STOP
#elif defined(HAVE_CONSOLE)
 #error "No CONFIG_UARTn_SERIAL_CONSOLE Setting"
#endif

/* Get word length setting for the console */

#if CONSOLE_BITS >= 5 && CONSOLE_BITS <= 8
 #define CONSOLE_LCR_WLS UART_LCR_WLEN(CONSOLE_BITS)
#elif defined(HAVE_CONSOLE)
 #error "Invalid CONFIG_UARTn_BITS setting for console "
#endif

/* Get parity setting for the console */

#if CONSOLE_PARITY == 0
 #define CONSOLE_LCR_PAR 0
#elif CONSOLE_PARITY == 1
 #define CONSOLE_LCR_PAR (UART_LCR_PEN)
#elif CONSOLE_PARITY == 2
 #define CONSOLE_LCR_PAR (UART_LCR_PEN | UART_LCR_EPS)
#elif CONSOLE_PARITY == 3
 #define CONSOLE_LCR_PAR (UART_LCR_PEN | UART_LCR_SPS)
#elif CONSOLE_PARITY == 4
 #define CONSOLE_LCR_PAR (UART_LCR_PEN | UART_LCR_EPS | UART_LCR_SPS)
#elif defined(HAVE_CONSOLE)
 #error "Invalid CONFIG_UARTn_PARITY setting for CONSOLE"
#endif

/* Get stop-bit setting for the console and UART0/1/2 */

#if CONSOLE_2STOP != 0
 #define CONSOLE_LCR_STOP UART_LCR_STP2
#else
 #define CONSOLE_LCR_STOP 0
#endif

/* LCR and FCR values for the console */

#define CONSOLE_LCR_VALUE (CONSOLE_LCR_WLS | CONSOLE_LCR_PAR | CONSOLE_LCR_STOP)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct uartdev
{
  uintptr_t uartbase; /* Base address of UART registers */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct uartdev g_uartdevs[] =
{
  {
    CXD56_UART0_BASE
  },
  {
    CXD56_UART1_BASE
  },
  {
    CXD56_UART2_BASE
  }
};

static uint32_t g_lcr;
static uint32_t g_cr;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_uart_pincontrol
 *
 * Description:
 *   Configure the UART pin
 *
 * Input Parameter:
 *   ch - channel number
 *   on - true: enable pin, false: disable pin
 *
 ****************************************************************************/

static void cxd56_uart_pincontrol(int ch, bool on)
{
  switch (ch)
    {
#ifdef CONFIG_CXD56_UART1
      case 1:
        if (on)
          {
            CXD56_PIN_CONFIGS(PINCONFS_SPI0A_UART1);
          }
        else
          {
            CXD56_PIN_CONFIGS(PINCONFS_SPI0A_GPIO);
          }
        break;
#endif

#ifdef CONFIG_CXD56_UART2
      case 2:
        if (on)
          {
            CXD56_PIN_CONFIGS(PINCONFS_UART2);
          }
        else
          {
            CXD56_PIN_CONFIGS(PINCONFS_UART2_GPIO);
          }
        break;
#endif
      default:
        break;
    }
}

/****************************************************************************
 * Name: cxd56_uart_start
 *
 * Description:
 *   Start a UART.  These functions are used by the serial driver when a
 *   UART is start.
 *
 ****************************************************************************/

static void cxd56_uart_start(int ch)
{
  cxd56_setbaud(CONSOLE_BASE, CONSOLE_BASEFREQ, CONSOLE_BAUD);

  putreg32(g_lcr, g_uartdevs[ch].uartbase + CXD56_UART_LCR_H);

  putreg32(g_cr, g_uartdevs[ch].uartbase + CXD56_UART_CR);
}

/****************************************************************************
 * Name: cxd56_uart_stop
 *
 * Description:
 *   Stop a UART.  These functions are used by the serial driver when a
 *   UART is stop.
 *
 ****************************************************************************/

static void cxd56_uart_stop(int ch)
{
  uint32_t cr;

  while (UART_FR_BUSY & getreg32(g_uartdevs[ch].uartbase + CXD56_UART_FR));

  cr   = getreg32(g_uartdevs[ch].uartbase + CXD56_UART_CR);
  g_cr = cr;
  cr &= ~UART_CR_EN;
  putreg32(cr, g_uartdevs[ch].uartbase + CXD56_UART_CR);

  g_lcr = getreg32(g_uartdevs[ch].uartbase + CXD56_UART_LCR_H);
  putreg32(0, g_uartdevs[ch].uartbase + CXD56_UART_LCR_H);
}

/****************************************************************************
 * Name: cxd56_uart_have_rxdata
 *
 * Description:
 *   Check if there is the remaining data in received FIFO or not
 *
 ****************************************************************************/

static int cxd56_uart_have_rxdata(int ch)
{
  if (UART_FR_RXFE & getreg32(g_uartdevs[ch].uartbase + CXD56_UART_FR))
    {
      return 0; /* Rx FIFO is empty */
    }
  else
    {
      return 1; /* Rx FIFO have data */
    }
}

/****************************************************************************
 * Name: cxd56_uart_clockchange
 *
 * Description:
 *   pm event callback for uart console
 *
 ****************************************************************************/

static int cxd56_uart_clockchange(uint8_t id)
{
#ifdef HAVE_CONSOLE
#if defined(CONFIG_UART0_SERIAL_CONSOLE)
  int ch = 0;
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
  int ch = 1;
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
  int ch = 2;
#endif

  switch (id)
    {
      case CXD56_PM_CALLBACK_ID_CLK_CHG_START:
        cxd56_uart_stop(ch);
        break;

      case CXD56_PM_CALLBACK_ID_CLK_CHG_END:
        cxd56_uart_start(ch);
        break;

      case CXD56_PM_CALLBACK_ID_HOT_BOOT:
        if (cxd56_uart_have_rxdata(ch))
          {
            return -1; /* don't restart if processing data in rxfifo */
          }

        cxd56_uart_stop(ch);
        cxd56_uart_start(ch);
        break;

      default:
        break;
    }

#endif
  return 0;
}

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
#if defined HAVE_UART && defined HAVE_CONSOLE
  /* Wait for the transmitter to be available */

  while ((getreg32(CONSOLE_BASE + CXD56_UART_FR) & UART_FLAG_TXFF));

  /* Send the character */

  putreg32((uint32_t)ch, CONSOLE_BASE + CXD56_UART_DR);
#endif
}

/****************************************************************************
 * Name: cxd56_lowsetup
 *
 * Description:
 *   This performs basic initialization of the UART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 *   The USART0/2/3 and UART1 peripherals are configured using the following
 *   registers:
 *   1. Baud rate: In the LCR register, set bit DLAB = 1. This enables access
 *      to registers DLL and DLM for setting the baud rate. Also, if needed,
 *      set the fractional baud rate in the fractional divider
 *   2. UART FIFO: Use bit FIFO enable (bit 0) in FCR register to
 *      enable FIFO.
 *   3. Pins: Select UART pins through the PINSEL registers and pin modes
 *      through the PINMODE registers. UART receive pins should not have
 *      pull-down resistors enabled.
 *   4. Interrupts: To enable UART interrupts set bit DLAB = 0 in the LCRF
 *      register. This enables access to IER. Interrupts are enabled
 *      in the NVIC using the appropriate Interrupt Set Enable register.
 *   5. DMA: UART transmit and receive functions can operate with the
 *      GPDMA controller.
 *
 ****************************************************************************/

void cxd56_lowsetup(void)
{
#ifdef HAVE_UART
  /* Enable clocking and  for all console UART and disable power for
   * other UARTs
   */

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
  cxd56_uart_setup(0);
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
  cxd56_uart_setup(1);
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
  cxd56_uart_setup(2);
#endif

  /* Configure the console (only) */

#if defined(HAVE_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)
    {
      uint32_t val;
      val = getreg32(CONSOLE_BASE + CXD56_UART_CR);
      if (val & UART_CR_EN)
        {
          return;
        }
    }

  putreg32(CONSOLE_LCR_VALUE, CONSOLE_BASE + CXD56_UART_LCR_H);
  cxd56_setbaud(CONSOLE_BASE, CONSOLE_BASEFREQ, CONSOLE_BAUD);
  putreg32(0, CONSOLE_BASE + CXD56_UART_IFLS);
  putreg32(UART_INTR_ALL, CONSOLE_BASE + CXD56_UART_ICR);

#endif
#endif
}

/****************************************************************************
 * Name: cxd56_uart_reset
 *
 * Description:
 *   Reset a UART.  These functions are used by the serial driver when a
 *   UART is closed.
 *
 ****************************************************************************/

void cxd56_uart_reset(int ch)
{
  /* Configure pin */

  cxd56_uart_pincontrol(ch, false);

  /* TODO: clock down */

#ifdef CONFIG_CXD56_UART2
  if (ch == 2)
    {
      cxd56_img_uart_clock_disable();
    }
#endif
}

void cxd56_uart_setup(int ch)
{
  uint32_t cr;
  uint32_t lcr;

  /* TODO: clock configuration */

#ifdef CONFIG_CXD56_UART2
  if (ch == 2)
    {
      cxd56_img_uart_clock_enable();
    }
#endif

  cr = getreg32(g_uartdevs[ch].uartbase + CXD56_UART_CR);
  putreg32(cr & ~(1 << 0), g_uartdevs[ch].uartbase + CXD56_UART_CR);

  lcr = getreg32(g_uartdevs[ch].uartbase + CXD56_UART_LCR_H);
  putreg32(lcr & ~(1 << 4), g_uartdevs[ch].uartbase + CXD56_UART_LCR_H);

  putreg32(cr, g_uartdevs[ch].uartbase + CXD56_UART_CR);

  /* Configure pin */

  cxd56_uart_pincontrol(ch, true);
}

/****************************************************************************
 * Name: cxd56_setbaud
 *
 ****************************************************************************/

void cxd56_setbaud(uintptr_t uartbase, uint32_t basefreq, uint32_t baud)
{
  uint32_t ibrd;
  uint32_t fbrd;
  uint32_t div;
  uint32_t lcr_h;

  irqstate_t flags = spin_lock_irqsave();

  if (uartbase == CXD56_UART2_BASE)
    {
      basefreq = cxd56_get_img_uart_baseclock();
    }
  else if (uartbase == CXD56_UART1_BASE)
    {
      basefreq = cxd56_get_com_baseclock();
    }
  else
    {
      spin_unlock_irqrestore(flags);
      return;
    }

  div  = basefreq / (16 * baud / 100);
  ibrd = div / 100;

  /* fbrd will be up to 63 ((99 * 64 + 50) / 100 = 6386 / 100 = 63) */

  fbrd = (((div % 100) * 64) + 50) / 100;

  /* Invalid baud rate divider setting combination */

  if (ibrd == 0 || (ibrd == 65535 && fbrd != 0))
    {
      goto finish;
    }

  putreg32(ibrd, uartbase + CXD56_UART_IBRD);
  putreg32(fbrd, uartbase + CXD56_UART_FBRD);

  /* Baud rate is updated by writing to LCR_H */

  lcr_h = getreg32(uartbase + CXD56_UART_LCR_H);
  putreg32(lcr_h, uartbase + CXD56_UART_LCR_H);

finish:
  spin_unlock_irqrestore(flags);
}

/****************************************************************************
 * Name: cxd56_uart_initialize
 *
 * Description:
 *   Various initial registration
 *
 ****************************************************************************/

int cxd56_uart_initialize(void)
{
#ifdef HAVE_UART
#  if defined(CONFIG_UART0_SERIAL_CONSOLE)
  cxd56_pm_register_callback(PM_CLOCK_HIF_UART0, cxd56_uart_clockchange);
#  elif defined(CONFIG_UART1_SERIAL_CONSOLE)
  cxd56_pm_register_callback(PM_CLOCK_SYS_UART1, cxd56_uart_clockchange);
#  elif defined(CONFIG_UART2_SERIAL_CONSOLE)
  cxd56_pm_register_callback(PM_CLOCK_APP_UART, cxd56_uart_clockchange);
#  endif
#endif
  return 0;
}
