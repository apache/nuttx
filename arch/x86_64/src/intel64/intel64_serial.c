/****************************************************************************
 * arch/x86_64/src/intel64/intel64_serial.c
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

#include <nuttx/arch.h>
#include <nuttx/serial/uart_16550.h>
#include <nuttx/serial/uart_pci_16550.h>

#include <arch/io.h>

#include "chip.h"
#include "x86_64_internal.h"

/* This is a "stub" file to suppport up_putc if no real serial driver is
 * configured.  Normally, drivers/serial/uart_16550.c provides the serial
 * driver for this platform.
 */

#ifdef USE_SERIALDRIVER

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uart_getreg(), uart_putreg()
 *
 * Description:
 *   These functions must be provided by the processor-specific code in order
 *   to correctly access 16550 registers
 *
 ****************************************************************************/

uart_datawidth_t uart_getreg(struct u16550_s *priv, unsigned int offset)
{
  return inb(priv->uartbase + offset);
}

void uart_putreg(struct u16550_s *priv, unsigned int offset,
                 uart_datawidth_t value)
{
  /* Intel x86 platform require OUT2 of MCR being set
   * for interrupt to be triggered. We make sure that bit is stuck at 1.
   */

  if (offset == UART_MCR_OFFSET)
    {
      value |= UART_MCR_OUT2;
    }

  outb(value, priv->uartbase + offset);
}

#elif defined(CONFIG_MULTBOOT2_FB_TERM)

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

void up_lowputc(char ch)
{
  fb_putc(ch);
}

void up_putc(int ch)
{
  up_lowputc(ch);
}
#endif

#ifdef USE_EARLYSERIALINIT
void x86_64_earlyserialinit(void)
{
#ifndef CONFIG_16550_NO_SERIAL_CONSOLE
  u16550_earlyserialinit();
#endif
}
#endif

#ifdef USE_SERIALDRIVER
void x86_64_serialinit(void)
{
#ifdef CONFIG_16550_PCI_UART
  /* Initialize PCI UART 16550 */

  pci_u16550_init();
#endif

#ifdef CONFIG_16550_UART
  /* Initialize UART 16550 */

  u16550_serialinit();
#endif
}
#endif
