/****************************************************************************
 * arch/arm/src/nrf91/nrf91_clockconfig.c
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
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "nrf91_clockconfig.h"
#include "hardware/nrf91_clock.h"
#include "hardware/nrf91_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf91_clockconfig
 *
 * Description:
 *   Called to initialize the NRF91xxx.  This does whatever setup is needed
 *   to put the MCU in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.  This function also performs
 *   other low-level chip as necessary.
 *
 ****************************************************************************/

void nrf91_clockconfig(void)
{
#ifdef CONFIG_NRF91_HFCLK_XTAL
  /* Initialize HFCLK crystal oscillator */

  putreg32(0x0, NRF91_CLOCK_EVENTS_HFCLKSTARTED);
  putreg32(0x1, NRF91_CLOCK_TASKS_HFCLKSTART);

  while (!getreg32(NRF91_CLOCK_EVENTS_HFCLKSTARTED))
    {
      /* wait for external oscillator to start */
    }
#endif

#ifdef CONFIG_NRF91_USE_LFCLK
  /* Initialize LFCLK */

#if defined(CONFIG_NRF91_LFCLK_XTAL)
  putreg32(CLOCK_LFCLKSRC_SRC_LFXO, NRF91_CLOCK_LFCLKSRC);
#else
  putreg32(CLOCK_LFCLKSRC_SRC_LFRC, NRF91_CLOCK_LFCLKSRC);
#endif

  /* Trigger LFCLK start */

  putreg32(0x0, NRF91_CLOCK_EVENTS_LFCLKSTARTED);
  putreg32(0x1, NRF91_CLOCK_TASKS_LFCLKSTART);

  while (!getreg32(NRF91_CLOCK_EVENTS_LFCLKSTARTED))
    {
      /* Wait for LFCLK to be running */
    }

#if defined(CONFIG_NRF91_LFCLK_RC)
  /* TODO: calibrate LFCLK RC oscillator */
#endif
#endif
}
