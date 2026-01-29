/****************************************************************************
 * arch/tricore/src/illd/tc3xx/Configurations/Ifx_Cfg_Ssw.c
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

#include "Ifx_Cfg_Ssw.h"
#include "Ifx_Ssw_Infra.h"

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
  if (!IfxScuLbist_isDone())
    {
      if(Ifx_Ssw_isColdPoweronReset())
        {
          IfxScuLbist_triggerInline(&IfxScuLbist_defaultConfig);
        }
    }

  if (!IfxScuLbist_evaluateResult(IfxScuLbist_defaultConfig.signature))
    {
       __debug();
       while(1);
    }

    Ifx_Ssw_jumpBackToLink();
}
#endif

#if defined(__TASKING__)
#pragma endoptimize
#elif defined(__GNUC__)
#pragma GCC reset_options
#endif
