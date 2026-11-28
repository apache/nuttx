/****************************************************************************
 * arch/risc-v/src/spacemit-k1x-rcpu/spacemit_k1x_rcpu_clockconfig.c
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

#include "spacemit_k1x_rcpu_clockconfig.h"
#include "hardware/spacemit_k1x_rcpu_ccu.h"
#include "chip.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spacemit_k1x_rcpu_clockconfig
 *
 * Description:
 *   Bring up the peripheral clocks the RCPU needs and settle the core
 *   clock.
 *
 *   The RCPU does not come out of reset on its own.  The application
 *   processor releases it: spacemit_rproc_start() in the K1 Linux kernel
 *   enables the ipc2ap clock and the audio subsystem reset, writes the boot
 *   entry and the DDR remap base, and only then writes MCU_EXECUTION_CTRL.
 *   By the time this code runs the audio power domain and the core clock
 *   are running, so nothing here has to create them.
 *
 *   What the application processor does NOT do is leave the peripheral clock
 *   gates enabled.  They come out of reset disabled, and the consequence is
 *   subtle: a peripheral whose functional clock is off cannot be reached
 *   over the bus, and reads come back as a constant instead of faulting.
 *   On the console UART that shows up as a transmitter that accepts data
 *   THR and never shifts a single bit out, which is indistinguishable from a
 *   wiring fault if you only look at the connector.
 *
 *   So the console UART has to be clocked and taken out of reset here,
 *   before the first byte is written.
 *
 ****************************************************************************/

void spacemit_k1x_rcpu_clockconfig(void)
{
  volatile uint32_t *reg;
  uint32_t value;

  /* 1. Ungate the 24.576 MHz audio PLL divider.
   *
   * The console UART functional clock selection is only a mux: the clock it
   * selects still has to be gated out of the audio PLL block.  This gate is
   * disabled by reset and is not enabled by the application processor.
   */

  reg = (volatile uint32_t *)SPACEMIT_K1X_RCPU_APBS_SPARE2;
  *reg |= SPACEMIT_K1X_RCPU_PLL1_AUD_24P5_GATE;

  /* 2. Console UART clock and reset.
   *
   * Select the 24 MHz crystal rather than the audio PLL so that the baud
   * rate does not depend on PLL lock, divide by one, enable the bus and
   * functional clocks and release the reset.
   */

  reg = (volatile uint32_t *)(SPACEMIT_K1X_RCPU_SYSCTRL_BASE +
                              SPACEMIT_K1X_RCPU_SYSCTRL_UART0_CLK_RST);
  *reg = (SPACEMIT_K1X_RCPU_CONSOLE_FCK_SEL <<
          SPACEMIT_K1X_RCPU_CLK_FCLK_SEL_SHIFT) |
         (0u << SPACEMIT_K1X_RCPU_CLK_FCLK_DIV_SHIFT) |
         SPACEMIT_K1X_RCPU_CLK_PCLK_EN |
         SPACEMIT_K1X_RCPU_CLK_FCLK_EN |
         SPACEMIT_K1X_RCPU_CLK_SW_RSTN;

  /* 3. Core clock.
   *
   * Program the clock source and divider rather than inheriting whatever the
   * previous boot stage left behind, so the tick rate does not depend on
   * the application processor's firmware revision.  Read-modify-write, so
   * the gate and other control bits in the same register survive.
   */

  reg   = (volatile uint32_t *)SPACEMIT_K1X_RCPU_CLK_RES_CTRL;
  value = *reg;

  value &= ~(SPACEMIT_K1X_RCPU_CORE_MUX_MASK |
             SPACEMIT_K1X_RCPU_CORE_DIV_MASK);
  value |= (SPACEMIT_K1X_RCPU_CORE_SRC <<
            SPACEMIT_K1X_RCPU_CORE_MUX_SHIFT) &
           SPACEMIT_K1X_RCPU_CORE_MUX_MASK;
  value |= (SPACEMIT_K1X_RCPU_CORE_DIV <<
            SPACEMIT_K1X_RCPU_CORE_DIV_SHIFT) &
           SPACEMIT_K1X_RCPU_CORE_DIV_MASK;

  *reg = value;
}
