/****************************************************************************
 * arch/arm/src/nrf54l/nrf54l_clockconfig.c
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
#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/nrf54l_clock.h"
#include "hardware/nrf54l_ficr.h"
#include "hardware/nrf54l_kmu.h"
#include "hardware/nrf54l_osc.h"
#include "hardware/nrf54l_tampc.h"
#include "nrf54l_clockconfig.h"
#include "nrf54l_oscconfig.h"

/* Factory trim and startup errata follow Nordic's system_nrf54l.c.
 * See the port documentation for the exact reference revision.
 */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_NRF54L_DEBUG_ACCESS
/****************************************************************************
 * Name: nrf54l_debug_open
 *
 * Description:
 *   Enable debug access unless an earlier boot stage locked protection.
 *
 ****************************************************************************/

static void nrf54l_debug_open(uintptr_t addr)
{
  /* Respect a protection signal already locked by an earlier boot stage. */

  if ((getreg32(addr) & TAMPC_LOCKED) == 0)
    {
      putreg32(TAMPC_UNLOCK, addr);
      putreg32(TAMPC_OPEN, addr);
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_clockconfig
 *
 * Description:
 *   Apply factory trim and startup errata, then start the high-speed clock.
 *
 ****************************************************************************/

void nrf54l_clockconfig(void)
{
  uint32_t addr;
  uint32_t part;
  uint32_t variant;
  unsigned int i;

#ifndef CONFIG_ARCH_CHIP_NRF54L15
  /* Start the LM20 KMU boot preparation before applying factory trim. */

  (void)getreg32(NRF54L_KMU_STATUS);
#endif

  part = getreg32(NRF54L_FICR_PART);
  variant = getreg32(NRF54L_FICR_VARIANT);

  /* Anomaly 37 applies to L15 and LM20A, and LM20B revision 1 onward. */

  if (part == 0x1c || part == 0x29 || (part == 0x33 && variant != 0))
    {
      putreg32(1, NRF54L_TAD_BASE + 0x040c);
    }

#ifdef CONFIG_NRF54L_DEBUG_ACCESS
  nrf54l_debug_open(NRF54L_TAMPC_DBGEN);
  nrf54l_debug_open(NRF54L_TAMPC_NIDEN);
  nrf54l_debug_open(NRF54L_TAMPC_SPIDEN);
  nrf54l_debug_open(NRF54L_TAMPC_SPNIDEN);
  nrf54l_debug_open(NRF54L_TAMPC_AP_DBGEN);
#endif

  for (i = 0; i < 64; i++)
    {
      addr = getreg32(NRF54L_FICR_TRIM_ADDR(i));
      if (addr == UINT32_MAX || addr == 0)
        {
          break;
        }

      putreg32(getreg32(NRF54L_FICR_TRIM_DATA(i)), addr);
    }

  /* L15 revision 1 startup anomalies 31, 32 and 40. */

  if (part == 0x1c && variant == 1)
    {
      putreg32(0x040a0078, 0x5008a7ac);
      putreg32(20 | (1 << 5), 0x50120624);
      modifyreg32(0x5012063c, 1 << 19, 0);
      if (getreg32(NRF54L_FICR_BASE + 0x334) <= 0x180a1d00)
        {
          putreg32(0x1ea9e040, 0x50120640);
        }
    }

  /* Configure oscillators */

  nrf54l_oscconfig();

  putreg32(OSC_PLL_FREQ_128M, NRF54L_OSC_PLL_FREQ);
  putreg32(0, NRF54L_CLOCK_EVENTS_XOSTARTED);
  putreg32(1, NRF54L_CLOCK_TASKS_XOSTART);

  for (i = 0; i < 10000000; i++)
    {
      if (getreg32(NRF54L_CLOCK_EVENTS_XOSTARTED) != 0)
        {
          break;
        }
    }

  if (i == 10000000)
    {
      PANIC();
    }

#ifdef CONFIG_NRF54L_USE_LFCLK
  /* Initialize LFCLK */

#if defined(CONFIG_NRF54L_LFCLK_XTAL)
  putreg32(CLOCK_LFCLK_SRC_SRC_LFXO, NRF54L_CLOCK_LFCLK_SRC);
#elif defined(CONFIG_NRF54L_LFCLK_SYNTH)
  putreg32(CLOCK_LFCLK_SRC_SRC_LFSYNT, NRF54L_CLOCK_LFCLK_SRC);
#else
  putreg32(CLOCK_LFCLK_SRC_SRC_LFRC, NRF54L_CLOCK_LFCLK_SRC);
#endif

  /* Trigger LFCLK start */

  putreg32(0x0, NRF54L_CLOCK_EVENTS_LFCLKSTARTED);
  putreg32(0x1, NRF54L_CLOCK_TASKS_LFCLKSTART);

  while (!getreg32(NRF54L_CLOCK_EVENTS_LFCLKSTARTED))
    {
      /* Wait for LFCLK to be running */
    }

#if defined(CONFIG_NRF54L_LFCLK_RC)
  /* TODO: calibrate LFCLK RC oscillator */
#endif
#endif

#ifndef CONFIG_ARCH_CHIP_NRF54L15
  for (i = 0; i < 10000000 && getreg32(NRF54L_KMU_STATUS) == 1; i++)
    {
    }

  if (i == 10000000)
    {
      PANIC();
    }
#endif
}
