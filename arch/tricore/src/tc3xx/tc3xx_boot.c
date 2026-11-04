/****************************************************************************
 * arch/tricore/src/tc3xx/tc3xx_boot.c
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
#include <nuttx/compiler.h>

#include "tricore_internal.h"

#include "IfxScuWdt.h"
#include "IfxScuCcu.h"
#include "Ifx_Cfg_Ssw.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_bist_k1/k2
 *
 * Description:
 *   Built in selftests, called from tricore_head.S while the C environment
 *   is not up yet: the stack lives in PSPR and no global variable may be
 *   touched.  The TC3xx BIST callouts (LBIST/MONBIST/MBIST) are disabled by
 *   default, so only the watchdogs are serviced here.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_BIST
void tricore_bist_k1(void)
{
  IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
  IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());
}

void tricore_bist_k2(void)
{
}
#endif

/****************************************************************************
 * Name: tricore_boot
 *
 * Description:
 *   Chip wide bring-up, called once from tricore_boot_primary() on CPU0
 *   after the C environment is up.
 *
 *   Covers what iLLD spreads over __StartUpSoftware phases 2, 4 and the
 *   watchdog handling of __Core0_start: the SCU CPU/safety watchdogs are
 *   disabled and the clock system (PLL with the matching flash wait states)
 *   is initialized.  CSA chaining, cache setup, BTV/BIV/ISP and the C
 *   runtime init are already done by tricore_head.S.
 *
 ****************************************************************************/

void tricore_boot(void)
{
  /* CPU and safety watchdogs are enabled by default after Boot-FW.  Disable
   * them to ensure the normal startup of the system.
   */

  IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
  IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());

  /* Initialize the clock system.  This matches the iLLD Phase4 PLL_INIT
   * callout (IFX_CFG_SSW_CALLOUT_PLL_INIT).  A non-zero return marks a PLL
   * lock failure; keep going so the console still comes up for diagnosis.
   */

#if (IFX_CFG_SSW_ENABLE_PLL_INIT == 1)
  (void)IfxScuCcu_init(&IfxScuCcu_defaultClockConfig);
#endif
}
