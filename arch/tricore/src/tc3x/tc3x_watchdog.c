/****************************************************************************
 * arch/tricore/src/tc3x/tc3x_watchdog.c
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

#include <arch/arch.h>
#include <nuttx/bits.h>
#include "tricore_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TC3X_SCU_BASE     0xf0036000
#define TC3X_WDTCPU0CON0  (TC3X_SCU_BASE + 0x024c)
#define TC3X_WDTSCON0     (TC3X_SCU_BASE + 0x02a8)
#define TC3X_WDTCPU0_CON1 (TC3X_SCU_BASE + 0x0250)
#define TC3X_WDTS_CON1    (TC3X_SCU_BASE + 0x02ac)

#define TC3X_WDT_CON1_DR  BIT(3)

/* TC3X has cores 0/1/2; the CORE_ID register reports them at indices
 * 0/1/2 plus a logical alias index 6.  Remap index 6 to 5 so the WDT
 * register stride math below works for all readings.
 */

#define TC3X_CORE_ID_ALIAS_6  6
#define TC3X_CORE_ID_REMAP_6  5

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void wdt_modify(uintptr_t con0, bool endinit)
{
  uint32_t val = getreg32(con0);
  uint32_t pw = ((val >> 2) & 0x3fff) ^ 0x003f;
  uint32_t want = endinit ? BIT(0) : 0;
  int      i;

  /* Password access: LCK=1, ENDINIT=1 */

  if (val & BIT(1))
    {
      putreg32((val & 0xffff0000) | (pw << 2) | BIT(0), con0);
    }

  /* Modify access: LCK=1, ENDINIT=desired */

  putreg32((val & 0xffff0000) | (pw << 2) | BIT(1) | want, con0);

  for (i = 0; i < 10000; i++)
    {
      if ((getreg32(con0) & BIT(0)) == want)
        {
          break;
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void aurix_cpu_endinit_enable(bool enable)
{
  uint32_t core_id;

  TRICORE_MFCR(TRICORE_CPU_CORE_ID, core_id);
  core_id &= TRICORE_CPU_CORE_ID_MASK;

  if (core_id == TC3X_CORE_ID_ALIAS_6)
    {
      core_id = TC3X_CORE_ID_REMAP_6;
    }

  wdt_modify(TC3X_WDTCPU0CON0 + (core_id * 12), enable);
}

void aurix_safety_endinit_enable(bool enable)
{
  wdt_modify(TC3X_WDTSCON0, enable);
}

void tricore_wdt_disable(void)
{
  uint32_t core_id;

  TRICORE_MFCR(TRICORE_CPU_CORE_ID, core_id);
  core_id &= TRICORE_CPU_CORE_ID_MASK;

  if (core_id == TC3X_CORE_ID_ALIAS_6)
    {
      core_id = TC3X_CORE_ID_REMAP_6;
    }

  if (core_id == 0)
    {
      aurix_safety_endinit_enable(false);
      putreg32(TC3X_WDT_CON1_DR, TC3X_WDTS_CON1);
      aurix_safety_endinit_enable(true);
    }

  aurix_cpu_endinit_enable(false);
  putreg32(TC3X_WDT_CON1_DR, TC3X_WDTCPU0_CON1 + (core_id * 12));
  aurix_cpu_endinit_enable(true);
}
