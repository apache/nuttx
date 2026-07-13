/****************************************************************************
 * arch/mips/src/jz4780/jz4780_lowinit.c
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
#include <assert.h>
#include <debug.h>

#include <arch/jz4780/cp0.h>
#include <arch/board/board.h>
#include <nuttx/serial/uart_16550.h>

#include "mips_internal.h"

#include "jz4780_lowinit.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: jz4780_pbclk
 *
 * Description:
 *   Configure peripheral bus clocking
 *
 * Assumptions:
 *   Interrupts are disabled.
 *
 ****************************************************************************/

static inline void jz4780_pbclk(void)
{
  putreg32(CKPCR_CK32CTL_RTCLK, CKPCR);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: jz4780_lowinit
 *
 * Description:
 *   This performs basic low-level initialization of the system.
 *
 * Assumptions:
 *   Interrupts have not yet been enabled.
 *
 ****************************************************************************/

void jz4780_lowinit(void)
{
  /* Configure peripheral clocking */

  jz4780_pbclk();

  /* Perform early serial initialization (so that we will have debug output
   * available as soon as possible).
   */

#ifdef USE_EARLYSERIALINIT
  u16550_earlyserialinit();
#endif

  /* Perform board-level initialization */

  jz4780_boardinitialize();
}

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
  return *(volatile uart_addrwidth_t *)(priv->uartbase + offset);
}

void uart_putreg(struct u16550_s *priv, unsigned int offset,
                 uart_datawidth_t value)
{
  *(volatile uart_addrwidth_t *)(priv->uartbase + offset) = value;
}

/****************************************************************************
 * Name: mips_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that mips_earlyserialinit was called previously.
 *
 ****************************************************************************/

void mips_serialinit(void)
{
  u16550_serialinit();
}

/****************************************************************************
 * Name: jz_physramaddr
 *
 * Description:
 *   Given the virtual address of a RAM memory location, return the physical
 *   address of that location.
 *
 ****************************************************************************/

uintptr_t jz_physramaddr(uintptr_t virtramaddr)
{
  return virtramaddr & 0x1fffffff;
}

/****************************************************************************
 * Name: jz_virtramaddr
 *
 * Description:
 *   Give the physical address of a RAM memory location, return the virtual
 *   address of that location.
 *
 ****************************************************************************/

uintptr_t jz_virtramaddr(uintptr_t physramaddr)
{
  if ((physramaddr & 0xffffffe0) == 0)
    {
      return 0;
    }

  return 0x80000000 | physramaddr;
}

#if CONFIG_MM_REGIONS > 1
void mips_addregion(void)
{
}
#endif
