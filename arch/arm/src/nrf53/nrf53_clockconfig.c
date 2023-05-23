/****************************************************************************
 * arch/arm/src/nrf53/nrf53_clockconfig.c
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
#include "nrf53_clockconfig.h"
#include "hardware/nrf53_clock.h"
#include "hardware/nrf53_power.h"
#include "hardware/nrf53_gpio.h"

#ifdef CONFIG_NRF53_APPCORE
#  include "nrf53_oscconfig.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf53_clockconfig
 *
 * Description:
 *   Called to initialize the NRF53xxx.  This does whatever setup is needed
 *   to put the MCU in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.  This function also performs
 *   other low-level chip as necessary.
 *
 ****************************************************************************/

void nrf53_clockconfig(void)
{
#ifdef CONFIG_NRF53_APPCORE
  /* Configure oscillators */

  nrf53_oscconfig();
#endif

#ifdef CONFIG_NRF53_HFCLK_XTAL
  /* Initialize HFCLK crystal oscillator */

  putreg32(0x0, NRF53_CLOCK_EVENTS_HFCLKSTARTED);
  putreg32(0x1, NRF53_CLOCK_TASKS_HFCLKSTART);

  while (!getreg32(NRF53_CLOCK_EVENTS_HFCLKSTARTED))
    {
      /* wait for external oscillator to start */
    }
#endif

#ifdef CONFIG_NRF53_USE_LFCLK
  /* Initialize LFCLK */

#if defined(CONFIG_NRF53_LFCLK_XTAL)
  putreg32(CLOCK_LFCLKSRC_SRC_LFXO, NRF53_CLOCK_LFCLKSRC);
#elif defined(CONFIG_NRF53_LFCLK_SYNTH)
  putreg32(CLOCK_LFCLKSRC_SRC_LFSYNT, NRF53_CLOCK_LFCLKSRC);
#else
  putreg32(CLOCK_LFCLKSRC_SRC_LFRC, NRF53_CLOCK_LFCLKSRC);
#endif

  /* Trigger LFCLK start */

  putreg32(0x0, NRF53_CLOCK_EVENTS_LFCLKSTARTED);
  putreg32(0x1, NRF53_CLOCK_TASKS_LFCLKSTART);

  /* NOTE: Oscillator must be configured on the app core */

  while (!getreg32(NRF53_CLOCK_EVENTS_LFCLKSTARTED))
    {
      /* Wait for LFCLK to be running */
    }

#if defined(CONFIG_NRF53_LFCLK_RC)
  /* TODO: calibrate LFCLK RC oscillator */
#endif
#endif

#ifdef CONFIG_NRF53_USE_HFCLK192M
  /* Initialize HFCLK192M */

#if defined(CONFIG_NRF53_HFCLK192M_192)
  putreg32(CLOCK_HFCLK192MSRC_DIV1, NRF53_CLOCK_HFCLK192MSRC);
#elif defined(CONFIG_NRF53_HFCLK192M_96)
  putreg32(CLOCK_HFCLK192MSRC_DIV2, NRF53_CLOCK_HFCLK192MSRC);
#elif defined(CONFIG_NRF53_HFCLK192M_48)
  putreg32(CLOCK_HFCLK192MSRC_DIV4, NRF53_CLOCK_HFCLK192MSRC);
#endif

  /* Trigger HFCLK192M start */

  putreg32(0x0, NRF53_CLOCK_EVENTS_HFCLK192MSTARTED);
  putreg32(0x1, NRF53_CLOCK_TASKS_HFCLK192MSTART);

  while (!getreg32(NRF53_CLOCK_EVENTS_HFCLK192MSTARTED))
    {
      /* Wait for HFCLK192M to be running */
    }
#endif
}
