/****************************************************************************
 * arch/tricore/src/illd/tc4xx/Configurations/Ifx_Cfg_Ssw.c
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

#include <string.h>

#include "Ifx_Cfg_Ssw.h"
#include "Ifx_Ssw_Infra.h"

#if (IFX_CFG_SSW_ENABLE_LBIST == 1)
#include "IfxApProt.h"
#include "IfxTriLbist.h"
#endif

#include "IfxVmt.h"
#include "IfxPmsPm.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#if defined(__TASKING__)
#pragma optimize RL
#elif defined(__GNUC__)
#pragma GCC optimize ("O1")
#endif

#if (IFX_CFG_SSW_ENABLE_LBIST == 1)
void Ifx_Ssw_Lbist(void)
{
  uint16 result;

  IfxApProt_setState((Ifx_PROT_PROT *)&TRI_PROTSE, IfxApProt_State_config);
  TRI_ACCEN_WRA.U = 0xFFFFFFFFU;
  TRI_ACCEN_WRB.U = 0xFFFFFFFFU;
  IfxApProt_setState((Ifx_PROT_PROT *)&TRI_PROTSE, IfxApProt_State_run);

  result = IfxTriLbist_evaluateResult(&IfxTriLbist_keyOnDefaultConfig.signature[0]);

  if (result)
    {
      if (IfxTriLbist_getFailCount() >= 2)
        {
          Ifx_Ssw_debug();
        }
      else
        {
          IfxTriLbist_incrementFailCount();
        }

      IfxTriLbist_resetLbist();
      IfxTriLbist_triggerInline(&IfxTriLbist_keyOnDefaultConfig);
    }
}
#endif

#if (IFX_CFG_SSW_ENABLE_MONBIST == 1)
void Ifx_Ssw_Monbist(void)
{
}
#endif

#if (IFX_CFG_SSW_ENABLE_MBIST_DSPRS_DMARAM == 1)
void Ifx_Ssw_MbistDsprsDmaRam(void)
{
}
#endif

#if IFX_CFG_SSW_ENABLE_PLL_INIT == 1U
#include "IfxClock.h"

void Ifx_Ssw_PowerOnCrystalOsc(void)
{
  Ifx_CLOCK_OSCCON scuOsccon;
  unsigned int     initError         = 0U;
  unsigned int     timeoutCycleCount = IFX_CFG_SSW_CCUCON_LCK_BIT_TIMEOUT_COUNT;
  scuOsccon.U = CLOCK_OSCCON.U;

#if (IFX_CFG_SSW_CLOCK_EXT_CLOCK == 1)
  scuOsccon.B.MODE = 2U;
#else
  scuOsccon.B.MODE = 0U;
#endif

  while (CLOCK_CCUSTAT.B.LCK != 0U)
    {
      IFX_CFG_SSW_LOOP_TIMEOUT_CHECK(timeoutCycleCount, initError);
    }

  CLOCK_OSCCON.U = scuOsccon.U;

  if (initError)
    {
      Ifx_Ssw_debug();
    }
}

void Ifx_Ssw_PllInit(void)
{
  if (IfxClock_init(&IfxClock_defaultClockConfig) != 0U)
    {
      __debug();
    }
}
#endif

#if IFX_CFG_SSW_ENABLE_MBIST == 1U
void Ifx_Ssw_Mbist(void)
{
  IfxVmt_clearSram(IfxVmt_MbistSel_lmu0);
  IfxVmt_clearSram(IfxVmt_MbistSel_lmu1);
  IfxVmt_clearSram(IfxVmt_MbistSel_lmu2);
  IfxVmt_clearSram(IfxVmt_MbistSel_lmu3);
  IfxVmt_clearSram(IfxVmt_MbistSel_lmu4);
  IfxVmt_clearSram(IfxVmt_MbistSel_lmu5);
  IfxVmt_clearSram(IfxVmt_MbistSel_lmu6);
  IfxVmt_clearSram(IfxVmt_MbistSel_lmu7);
  IfxVmt_clearSram(IfxVmt_MbistSel_lmu8);
  IfxVmt_clearSram(IfxVmt_MbistSel_lmu9);

  /* clear CAN Message RAM */

  IfxVmt_clearSram(IfxVmt_MbistSel_mcan0);
  IfxVmt_clearSram(IfxVmt_MbistSel_mcan1);
  IfxVmt_clearSram(IfxVmt_MbistSel_mcan2);
  IfxVmt_clearSram(IfxVmt_MbistSel_mcan3);
  IfxVmt_clearSram(IfxVmt_MbistSel_mcan4);
}
#endif

#if IFX_CFG_SSW_ENABLE_SMU == 1U
void Ifx_Ssw_Smu(void)
{
}
#endif

#if (IFX_CFG_SSW_ENABLE_KEYOFF_LBIST == 1)
void Ifx_Ssw_Keyoff_Lbist(void)
{
}
#endif

#if IFX_CFG_SSW_ENABLE_KEYOFF_MBIST == 1U
void Ifx_Ssw_Keyoff_Mbist(void)
{
}
#endif

#if (IFX_CFG_SSW_ENABLE_KEYOFF_MBIST_DSPRS_DMARAM == 1)
void Ifx_Ssw_Keyoff_MbistDsprsDmaRam(void)
{
}
#endif

#if IFX_CFG_SSW_ENABLE_AP_INIT == 1U
void weak_function Ifx_Ssw_AP_Init(void)
{
}
#endif

void Ifx_Ssw_MultiCore_Sync_Cpu0(void)
{
}

void Ifx_Ssw_MultiCore_Sync_Cpu1(void)
{
}

void Ifx_Ssw_MultiCore_Sync_Cpu2(void)
{
}

void Ifx_Ssw_MultiCore_Sync_Cpu3(void)
{
}

void Ifx_Ssw_MultiCore_Sync_Cpu4(void)
{
}

void Ifx_Ssw_MultiCore_Sync_Cpu5(void)
{
}

#if defined(__TASKING__)
#pragma endoptimize
#elif defined(__GNUC__)
#pragma GCC reset_options
#endif
