/****************************************************************************
 * arch/tricore/src/illd/tc3xx/Configurations/Ifx_Cfg_Ssw.h
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

#ifndef __ARCH_TRICORE_SRC_ILLD_TC3XX_CONFIGURATIONS_IFX_CFG_SSW_H
#define __ARCH_TRICORE_SRC_ILLD_TC3XX_CONFIGURATIONS_IFX_CFG_SSW_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "Ifx_Cfg.h"
#include "Ifx_Ssw.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef IFX_CFG_SSW_ENABLE_LBIST
#define IFX_CFG_SSW_ENABLE_LBIST          (0U)
#endif

#ifndef IFX_CFG_SSW_ENABLE_MONBIST
#define IFX_CFG_SSW_ENABLE_MONBIST        (0U)
#endif

#ifndef IFX_CFG_SSW_ENABLE_PLL_INIT
#define IFX_CFG_SSW_ENABLE_PLL_INIT       (1U)
#endif

#ifndef IFX_CFG_SSW_ENABLE_MBIST
#define IFX_CFG_SSW_ENABLE_MBIST          (0U)
#endif

#ifndef IFX_CFG_SSW_ENABLE_SMU
#define IFX_CFG_SSW_ENABLE_SMU            (0U)
#endif

#ifndef IFX_CFG_SSW_ENABLE_EMEM_INIT
#define IFX_CFG_SSW_ENABLE_EMEM_INIT      (0U)
#endif

#ifndef IFX_SCU_FLASHWAITSTATECHECK
#define IFX_SCU_FLASHWAITSTATECHECK       (0U)
#endif

#if IFX_CFG_SSW_ENABLE_LBIST == 1U
#include "IfxScuLbist.h"

extern void Ifx_Ssw_Lbist(void);
#define IFX_CFG_SSW_CALLOUT_LBIST()               \
{                                                 \
  Ifx_Ssw_jumpToFunctionWithLink(&Ifx_Ssw_Lbist); \
}
#endif

#if IFX_CFG_SSW_ENABLE_MONBIST == 1U
#include "IfxSmuStdby.h"

extern void Ifx_Ssw_Monbist(void);
#define IFX_CFG_SSW_CALLOUT_MONBIST()               \
{                                                   \
  Ifx_Ssw_jumpToFunctionWithLink(&Ifx_Ssw_Monbist); \
}
#endif

#if IFX_CFG_SSW_ENABLE_PLL_INIT == 1U
#include "IfxScuCcu.h"

#define IFX_CFG_SSW_CALLOUT_PLL_INIT()                    \
{                                                         \
  if (IfxScuCcu_init(&IfxScuCcu_defaultClockConfig) == 1) \
    {                                                     \
      __debug();                                          \
    }                                                     \
}

#endif

#if IFX_CFG_SSW_ENABLE_MBIST == 1U
#include "IfxMtu.h"

#define IFX_CFG_SSW_CALLOUT_MBIST()                             \
{                                                               \
  IFX_EXTERN const IfxMtu_MbistConfig *const mbistGangConfig[]; \
  if (IfxMtu_runMbistAll(mbistGangConfig) == 1U)                \
    {                                                           \
      __debug();                                                \
    }                                                           \
}

#endif
#endif /* __ARCH_TRICORE_SRC_ILLD_TC3XX_CONFIGURATIONS_IFX_CFG_SSW_H */
