/****************************************************************************
 * arch/tricore/src/tc4xx/tc4xx_wdt.c
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

#include <arch/arch.h>

#include "tricore_internal.h"
#include "hardware/tc4xx_wdt.h"
#include "tc4xx_wdt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Core index in CORE_ID */

#define TC4XX_CORE_ID_MASK         0x7

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tc4xx_wdt_disable
 *
 * Description:
 *   Request a watchdog to be disabled: unlock CTRLA with the password if it
 *   is locked, set the disable request in CTRLB and lock CTRLA again.
 *
 ****************************************************************************/

static void tc4xx_wdt_disable(uintptr_t ctrla, uintptr_t ctrlb)
{
  uint32_t a = getreg32(ctrla);
  uint32_t b = getreg32(ctrlb);
  uint32_t pw = ((a & TC4XX_WTU_CTRLA_PW_MASK) >> TC4XX_WTU_CTRLA_PW_SHIFT) ^
                TC4XX_WTU_PW_INVERTED;

  if ((a & TC4XX_WTU_CTRLA_LCK) != 0)
    {
      a &= ~(TC4XX_WTU_CTRLA_LCK | TC4XX_WTU_CTRLA_PW_MASK);
      a |= pw << TC4XX_WTU_CTRLA_PW_SHIFT;
    }

  putreg32(a, ctrla);
  putreg32(b | TC4XX_WTU_CTRLB_DR, ctrlb);
  putreg32(a | TC4XX_WTU_CTRLA_LCK, ctrla);
}

/****************************************************************************
 * Name: tc4xx_wdt_password
 ****************************************************************************/

static uint32_t tc4xx_wdt_password(uintptr_t ctrla)
{
  return ((getreg32(ctrla) & TC4XX_WTU_CTRLA_PW_MASK) >>
          TC4XX_WTU_CTRLA_PW_SHIFT) ^ TC4XX_WTU_PW_INVERTED;
}

/****************************************************************************
 * Name: tc4xx_wdt_service_one
 *
 * Description:
 *   Service a watchdog: unlock CTRLA with the password, with the inverted
 *   timer value if the timer check is enabled, then lock it again.
 *
 ****************************************************************************/

static void tc4xx_wdt_service_one(uintptr_t ctrla, uint32_t pw)
{
  uint32_t a = getreg32(ctrla);

  if ((a & TC4XX_WTU_CTRLA_LCK) != 0)
    {
      uint32_t stat = getreg32(ctrla + TC4XX_WTU_STAT_OFFSET);

      a &= ~(TC4XX_WTU_CTRLA_LCK | TC4XX_WTU_CTRLA_PW_MASK);
      a |= pw << TC4XX_WTU_CTRLA_PW_SHIFT;
      if ((stat & TC4XX_WTU_STAT_TC) != 0)
        {
          a &= ~(0xffff << TC4XX_WTU_CTRLA_TCVI_SHIFT);
          a |= (~stat >> TC4XX_WTU_STAT_TIM_SHIFT) <<
               TC4XX_WTU_CTRLA_TCVI_SHIFT;
        }

      putreg32(a, ctrla);
    }

  putreg32(a | TC4XX_WTU_CTRLA_LCK, ctrla);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tc4xx_wdt_service
 *
 * Description:
 *   Service the CPU0 and the system watchdog, for the long waits of the
 *   clock setup, which runs before the watchdogs are disabled.
 *
 ****************************************************************************/

void tc4xx_wdt_service(void)
{
  tc4xx_wdt_service_one(TC4XX_WTU_WDTCPU_CTRLA(0),
                        tc4xx_wdt_password(TC4XX_WTU_WDTCPU_CTRLA(0)));
  tc4xx_wdt_service_one(TC4XX_WTU_WDTSYS_CTRLA,
                        tc4xx_wdt_password(TC4XX_WTU_WDTSYS_CTRLA));
}

/****************************************************************************
 * Name: tricore_wdt_disable
 *
 * Description:
 *   Disable the watchdog of the calling CPU and the system watchdog.
 *
 ****************************************************************************/

void tricore_wdt_disable(void)
{
  uint32_t cpu = tricore_mfcr(TRICORE_CPU_CORE_ID) & TC4XX_CORE_ID_MASK;

  tc4xx_wdt_disable(TC4XX_WTU_WDTCPU_CTRLA(cpu),
                    TC4XX_WTU_WDTCPU_CTRLB(cpu));
  tc4xx_wdt_disable(TC4XX_WTU_WDTSYS_CTRLA, TC4XX_WTU_WDTSYS_CTRLB);
}
