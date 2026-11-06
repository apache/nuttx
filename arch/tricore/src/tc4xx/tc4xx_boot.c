/****************************************************************************
 * arch/tricore/src/tc4xx/tc4xx_boot.c
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

#include "IfxWtu.h"
#include "Ifx_Cfg_Ssw.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* iLLD keeps these SSW hook prototypes private to Ifx_Ssw_Tc0.c, which is
 * no longer built, so declare them here.
 */

void Ifx_Ssw_Lbist(void);
void Ifx_Ssw_Monbist(void);
void Ifx_Ssw_MbistDsprsDmaRam(void);
void Ifx_Ssw_Mbist(void);
void Ifx_Ssw_FwCheck(void);
void Ifx_Ssw_Pms_Init(void);
void Ifx_Ssw_Pms_InitCheck(void);
void Ifx_Ssw_ShutdownPwrDomains(void);
void Ifx_Ssw_Pmic_VoltageShift(void);
void Ifx_Ssw_CsrmSync_C(void);
void Ifx_Ssw_CsrmSync_D(void);
void Ifx_Ssw_Autosar_ServicesStart(void);
void Ifx_Ssw_PowerOnCrystalOsc(void);
void Ifx_Ssw_XtalSrc_Check(void);
void Ifx_Ssw_PllInit(void);
void Ifx_Ssw_Smu(void);
void Ifx_Ssw_SafetyLibraryTests(void);
void Ifx_Ssw_AP_Init(void);

#ifdef CONFIG_ARCH_BIST
void tricore_bist_k1(void);
void tricore_bist_k2(void);
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Default empty SSW hooks, overridden by Ifx_Cfg_Ssw.c when enabled */

weak_function void Ifx_Ssw_Lbist(void)
{
}

weak_function void Ifx_Ssw_Monbist(void)
{
}

weak_function void Ifx_Ssw_MbistDsprsDmaRam(void)
{
}

weak_function void Ifx_Ssw_Mbist(void)
{
}

weak_function void Ifx_Ssw_FwCheck(void)
{
}

weak_function void Ifx_Ssw_Pms_Init(void)
{
}

weak_function void Ifx_Ssw_Pms_InitCheck(void)
{
}

weak_function void Ifx_Ssw_ShutdownPwrDomains(void)
{
}

weak_function void Ifx_Ssw_Pmic_VoltageShift(void)
{
}

weak_function void Ifx_Ssw_CsrmSync_C(void)
{
}

weak_function void Ifx_Ssw_CsrmSync_D(void)
{
}

weak_function void Ifx_Ssw_Autosar_ServicesStart(void)
{
}

weak_function void Ifx_Ssw_PowerOnCrystalOsc(void)
{
}

weak_function void Ifx_Ssw_XtalSrc_Check(void)
{
}

weak_function void Ifx_Ssw_PllInit(void)
{
}

weak_function void Ifx_Ssw_Smu(void)
{
}

weak_function void Ifx_Ssw_SafetyLibraryTests(void)
{
}

weak_function void Ifx_Ssw_AP_Init(void)
{
}

/****************************************************************************
 * Name: tricore_bist_k1/k2
 *
 * Description:
 *   Built in selftests, called from tricore_head.S while the C environment
 *   is not up yet: the stack lives in PSPR and no global variable may be
 *   touched.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_BIST
void tricore_bist_k1(void)
{
  IfxWtu_disableCpuWatchdog(IfxWtu_getCpuWatchdogPassword());
  IfxWtu_disableSystemWatchdog(IfxWtu_getSystemWatchdogPassword());

  Ifx_Ssw_Lbist();
  Ifx_Ssw_Monbist();
  Ifx_Ssw_MbistDsprsDmaRam();
}

void tricore_bist_k2(void)
{
  Ifx_Ssw_Mbist();
}
#endif

/****************************************************************************
 * Name: tricore_boot
 *
 * Description:
 *   Chip wide bring-up, called once from tricore_boot_primary() on CPU0
 *   after the C environment is up.
 *
 *   Covers what iLLD spreads over __StartUpSoftware phases 2, 4, 5 and the
 *   first half of 7: power and EVR, unused power domains, crystal and PLL
 *   with the matching flash wait states, and the PROT/APU access
 *   protection.
 *
 ****************************************************************************/

void tricore_boot(void)
{
  /* All WDTs except WDTCPU0 and system watchdog WDTSYS are in
   * disabled mode after Boot-FW execution. Disable the watchdog
   * to ensure the normal startup of the system.
   */

  IfxWtu_disableCpuWatchdog(IfxWtu_getCpuWatchdogPassword());
  IfxWtu_disableSystemWatchdog(IfxWtu_getSystemWatchdogPassword());

  Ifx_Ssw_FwCheck();
  Ifx_Ssw_Pms_Init();
  Ifx_Ssw_Pms_InitCheck();

  Ifx_Ssw_ShutdownPwrDomains();
  Ifx_Ssw_Pmic_VoltageShift();
  Ifx_Ssw_CsrmSync_C();
  Ifx_Ssw_Autosar_ServicesStart();

#if (IFX_CFG_SSW_ENABLE_PLL_INIT == 1)
  Ifx_Ssw_PowerOnCrystalOsc();
  Ifx_Ssw_XtalSrc_Check();
  Ifx_Ssw_PllInit();
#endif

  Ifx_Ssw_CsrmSync_D();

  Ifx_Ssw_Smu();

  Ifx_Ssw_SafetyLibraryTests();

  Ifx_Ssw_AP_Init();
}
