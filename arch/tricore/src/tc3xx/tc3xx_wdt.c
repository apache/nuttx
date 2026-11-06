/****************************************************************************
 * arch/tricore/src/tc3xx/tc3xx_wdt.c
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

#include <stdbool.h>
#include <stdint.h>

#include <arch/arch.h>

#include "tricore_internal.h"
#include "tc3xx_wdt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SCU watchdog timers: one per CPU and the safety watchdog */

#define TC3XX_SCU_WDTCPU_CON0(n)   (0xf003624c + (n) * 0xc)
#define TC3XX_SCU_WDTCPU_CON1(n)   (0xf0036250 + (n) * 0xc)
#define TC3XX_SCU_WDTS_CON0        0xf00362a8
#define TC3XX_SCU_WDTS_CON1        0xf00362ac

#define TC3XX_WDT_CON0_ENDINIT     (1 << 0)          /* End of init */
#define TC3XX_WDT_CON0_LCK         (1 << 1)          /* Lock */
#define TC3XX_WDT_CON0_PW_SHIFT    2                 /* Password */
#define TC3XX_WDT_CON0_PW_MASK     (0x3fff << TC3XX_WDT_CON0_PW_SHIFT)
#define TC3XX_WDT_CON0_REL_MASK    (0xffff << 16)    /* Reload value */
#define TC3XX_WDT_CON1_DR          (1 << 3)          /* Disable request */

/* The low six password bits read inverted */

#define TC3XX_WDT_PW_INVERTED      0x3f

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tc3xx_wdt_endinit
 *
 * Description:
 *   Set or clear ENDINIT of a watchdog with its password, unlocking CON0
 *   first if it is locked, and wait until ENDINIT has changed.
 *
 ****************************************************************************/

static void tc3xx_wdt_endinit(uintptr_t con0, uint32_t pw, bool set)
{
  uint32_t regval = getreg32(con0);
  uint32_t rel = regval & TC3XX_WDT_CON0_REL_MASK;

  pw <<= TC3XX_WDT_CON0_PW_SHIFT;
  if ((regval & TC3XX_WDT_CON0_LCK) != 0)
    {
      putreg32(rel | pw | TC3XX_WDT_CON0_ENDINIT, con0);
    }

  putreg32(rel | pw | TC3XX_WDT_CON0_LCK |
           (set ? TC3XX_WDT_CON0_ENDINIT : 0), con0);

  while (((getreg32(con0) & TC3XX_WDT_CON0_ENDINIT) != 0) != set);
}

/****************************************************************************
 * Name: tc3xx_wdt_disable
 *
 * Description:
 *   Request a watchdog to be disabled.
 *
 ****************************************************************************/

static void tc3xx_wdt_disable(uintptr_t con0, uintptr_t con1)
{
  uint32_t pw = ((getreg32(con0) & TC3XX_WDT_CON0_PW_MASK) >>
                 TC3XX_WDT_CON0_PW_SHIFT) ^ TC3XX_WDT_PW_INVERTED;

  tc3xx_wdt_endinit(con0, pw, false);
  putreg32(getreg32(con1) | TC3XX_WDT_CON1_DR, con1);
  tc3xx_wdt_endinit(con0, pw, true);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tc3xx_wdt_safety_endinit
 *
 * Description:
 *   Set or clear the safety ENDINIT, which protects the clock registers.
 *
 ****************************************************************************/

void tc3xx_wdt_safety_endinit(bool set)
{
  uint32_t pw = ((getreg32(TC3XX_SCU_WDTS_CON0) & TC3XX_WDT_CON0_PW_MASK) >>
                 TC3XX_WDT_CON0_PW_SHIFT) ^ TC3XX_WDT_PW_INVERTED;

  tc3xx_wdt_endinit(TC3XX_SCU_WDTS_CON0, pw, set);
}

/****************************************************************************
 * Name: tricore_wdt_disable
 *
 * Description:
 *   Disable the watchdog of the calling CPU and the safety watchdog.  CPU5
 *   has the core ID 6 on TC39x, hence the limit to 5.
 *
 ****************************************************************************/

void tricore_wdt_disable(void)
{
  uint32_t cpu = tricore_mfcr(TRICORE_CPU_CORE_ID);

  cpu = cpu > 5 ? 5 : cpu;
  tc3xx_wdt_disable(TC3XX_SCU_WDTCPU_CON0(cpu), TC3XX_SCU_WDTCPU_CON1(cpu));
  tc3xx_wdt_disable(TC3XX_SCU_WDTS_CON0, TC3XX_SCU_WDTS_CON1);
}
