/****************************************************************************
 * arch/arm/src/cxd32xx/cxd32_uart.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/spinlock.h>
#include <arch/board/board.h>

#include <errno.h>

#include "arm_internal.h"
#include "chip.h"
#include "cxd32_config.h"
#include "cxd32_uart.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Select UART parameters for the selected console */

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
 #define CONSOLE_BASE     CXD32_UART0_BASE
 #define CONSOLE_BASEFREQ CXD32_UART_BASEFREQ
 #define CONSOLE_BAUD     CONFIG_UART0_BAUD
 #define CONSOLE_BITS     CONFIG_UART0_BITS
 #define CONSOLE_PARITY   CONFIG_UART0_PARITY
 #define CONSOLE_2STOP    CONFIG_UART0_2STOP
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

static spinlock_t g_cxd32xx_lock = SP_UNLOCKED;

static const struct uartdev g_uartdevs[] =
{
  {
    CXD32_UART0_BASE
  },
  {
    CXD32_UART1_BASE
  },
  {
    CXD32_UART2_BASE
  }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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

  while ((getreg32(CONSOLE_BASE + CXD32_UART_FR) & UART_FLAG_TXFF));

  /* Send the character */

  putreg32((uint32_t)ch, CONSOLE_BASE + CXD32_UART_DR);
#endif
}

/****************************************************************************
 * Name: cxd32_lowsetup
 *
 * Description:
 *   Called at the very beginning of _start.  Performs low level
 *   initialization of the serial console.
 *
 ****************************************************************************/

void cxd32_lowsetup(void)
{
#ifdef HAVE_UART
  /* Enable clocking and  for all console UART and disable power for
   * other UARTs
   */

  /* Configure the console (only) */

#if defined(HAVE_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)
    {
      uint32_t val;
      val = getreg32(CONSOLE_BASE + CXD32_UART_CR);
      if (val & UART_CR_EN)
        {
          return;
        }
    }

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
  cxd32_uart_setup(0);
  cxd32_uart_initialize(0);

  putreg32(CONSOLE_LCR_VALUE, CONSOLE_BASE + CXD32_UART_LCR_H);
  cxd32_setbaud(CONSOLE_BASE, CONSOLE_BASEFREQ, CONSOLE_BAUD);
  putreg32(0, CONSOLE_BASE + CXD32_UART_IFLS);
  putreg32(UART_INTR_ALL, CONSOLE_BASE + CXD32_UART_ICR);
#endif

#endif
#endif
}

/****************************************************************************
 * Name: cxd32_uart_reset
 *
 * Description:
 *   Reset a UART.  This function is used by the serial driver when a
 *   UART is closed.
 *
 ****************************************************************************/

void cxd32_uart_reset(int ch)
{
  /* XXX: disabling uart controller */
}

/****************************************************************************
 * Name: cxd32_uart_setup
 *
 ****************************************************************************/

void cxd32_uart_setup(int ch)
{
  /* XXX: enabling uart contrller */
}

/****************************************************************************
 * Name: cxd32_uart_initialize
 *
 * Description:
 *   Various initial registration
 *
 ****************************************************************************/

void cxd32_uart_initialize(int ch)
{
  uint32_t regval;

  regval = 0x00000000;

  /* disable Control register */

  putreg32(regval, g_uartdevs[ch].uartbase + CXD32_UART_CR);

  /* init IrDA low-power counter register */

  putreg32(regval, g_uartdevs[ch].uartbase + CXD32_UART_ILPR);

  /* init Integer baud rate register */

  putreg32(regval, g_uartdevs[ch].uartbase + CXD32_UART_IBRD);

  /* init Fractional baud rate register */

  putreg32(regval, g_uartdevs[ch].uartbase + CXD32_UART_FBRD);

  /* init DMA control register */

  putreg32(regval, g_uartdevs[ch].uartbase + CXD32_UART_DMACR);
}

/****************************************************************************
 * Name: cxd32_setbaud
 *
 ****************************************************************************/

#define DIV_MASK  0x000000FF
#define FBRD_MASK 0x3F

void cxd32_setbaud(uintptr_t uartbase, uint32_t basefreq, uint32_t baud)
{
  uint64_t div;
  uint32_t ibrd;
  uint32_t fbrd;
  uint32_t lcr_h;

  irqstate_t flags = spin_lock_irqsave(&g_cxd32xx_lock);

  div  = (uint64_t)(basefreq);
  div *= (uint64_t)(256);
  div /= (uint64_t)(16);
  div /= (uint64_t)(baud);
  ibrd = (uint32_t)(div >> 8);
  fbrd = (uint32_t)(((div & DIV_MASK) * 64 + 0x80) / 256);

  putreg32(ibrd + (fbrd >> 6), uartbase + CXD32_UART_IBRD);
  putreg32(fbrd & FBRD_MASK, uartbase + CXD32_UART_FBRD);

  /* Baud rate is updated by writing to LCR_H. */

  lcr_h = getreg32(uartbase + CXD32_UART_LCR_H);
  putreg32(lcr_h, uartbase + CXD32_UART_LCR_H);

  spin_unlock_irqrestore(&g_cxd32xx_lock, flags);
}
