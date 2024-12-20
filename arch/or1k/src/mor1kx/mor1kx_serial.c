/****************************************************************************
 * arch/or1k/src/mor1kx/mor1kx_serial.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>

#include <arch/board/board.h>

#include "or1k_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OR1K_UART_BASE (0x90000000)

#define OR1K_SYS_CLK (20000000)
#define OR1K_BAUD (115200)
#define OR1K_DIVISOR (OR1K_SYS_CLK / (16*OR1K_BAUD))

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef HAVE_SERIAL_CONSOLE
static spinlock_t g_serial_lock = SP_UNLOCKED;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: or1k_earlyserialinit
 *
 * Description:
 *   Performs the low level USART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before sam_serialinit.
 *
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT
void or1k_earlyserialinit(void)
{
  /* Disable all USARTS */

#ifdef HAVE_SERIAL_CONSOLE
  /* Mark the serial console (if any) */

  /* CONSOLE_DEV.isconsole = true; */
#endif
}
#endif

/****************************************************************************
 * Name: or1k_serialinit
 *
 * Description:
 *   Register serial console and serial ports.
 *
 ****************************************************************************/

#ifdef USE_SERIALDRIVER
void or1k_serialinit(void)
{
  /* Register the console */

#ifdef HAVE_SERIAL_CONSOLE
  uart_register("/dev/console", &CONSOLE_DEV);
#endif

  /* Register all USARTs */
}
#endif

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug  writes
 *
 ****************************************************************************/

void up_putc(int ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  irqstate_t flags;

  /* All interrupts must be disabled to prevent re-entrancy and to prevent
   * interrupts from firing in the serial driver code.
   */

  flags = spin_lock_irqsave(&g_serial_lock);

  /* or1k_lowputc(ch); */

  spin_unlock_irqrestore(&g_serial_lock, flags);
#endif
}
