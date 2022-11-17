/****************************************************************************
 * boards/x86_64/intel64/qemu-intel64/src/qemu_boot.c
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

#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/serial/uart_16550.h>
#include <arch/board/board.h>

#include "x86_64_internal.h"
#include "qemu_intel64.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: x86_64_boardinitialize
 *
 * Description:
 *   All x86_64 architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void x86_64_boardinitialize(void)
{
#if defined(CONFIG_16550_UART0) && (CONFIG_16550_UART0_BASE == 0x3f8)
  uart_putreg(CONFIG_16550_UART0_BASE, UART_MCR_OFFSET, UART_MCR_OUT2);
#endif

#if defined(CONFIG_16550_UART1) && (CONFIG_16550_UART1_BASE == 0x3f8)
  uart_putreg(CONFIG_16550_UART1_BASE, UART_MCR_OFFSET, UART_MCR_OUT2);
#endif

  /* Configure on-board LEDs if LED support has been selected. */

#ifdef CONFIG_ARCH_LEDS
  board_autoled_initialize();
#endif
}
