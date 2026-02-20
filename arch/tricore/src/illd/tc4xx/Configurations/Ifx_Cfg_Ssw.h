/****************************************************************************
 * arch/tricore/src/illd/tc4xx/Configurations/Ifx_Cfg_Ssw.h
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

#ifndef __ARCH_TRICORE_SRC_ILLD_TC4XX_CONFIGURATIONS_IFX_CFG_SSW_H
#define __ARCH_TRICORE_SRC_ILLD_TC4XX_CONFIGURATIONS_IFX_CFG_SSW_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "Ifx_Cfg.h"
#include "Ifx_Ssw.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef IFX_CFG_SSW_ENABLE_LBIST
#define IFX_CFG_SSW_ENABLE_LBIST                         (0U)
#endif

#ifndef IFX_CFG_SSW_ENABLE_MONBIST
#define IFX_CFG_SSW_ENABLE_MONBIST                       (0U)
#endif

#ifndef IFX_CFG_SSW_ENABLE_MBIST_DSPRS_DMARAM
#define IFX_CFG_SSW_ENABLE_MBIST_DSPRS_DMARAM            (0U)
#endif

#ifndef IFX_CFG_SSW_ENABLE_XTALSRC_CHECK
#define IFX_CFG_SSW_ENABLE_XTALSRC_CHECK                 (0U)
#endif

#ifndef IFX_CFG_SSW_ENABLE_PLL_INIT
#define IFX_CFG_SSW_ENABLE_PLL_INIT                      (1U)
#endif

#ifndef IFX_CFG_SSW_ENABLE_MBIST
#define IFX_CFG_SSW_ENABLE_MBIST                         (1U)
#endif

#ifndef IFX_CFG_SSW_ENABLE_SMU
#define IFX_CFG_SSW_ENABLE_SMU                           (0U)
#endif

#ifndef IFX_CFG_SSW_ENABLE_AP_INIT
#define IFX_CFG_SSW_ENABLE_AP_INIT                       (1U)
#endif

#ifndef IFX_CFG_SSW_ENABLE_KEYOFF_LBIST
#define IFX_CFG_SSW_ENABLE_KEYOFF_LBIST                  (0U)
#endif

#ifndef IFX_CFG_SSW_ENABLE_KEYOFF_MBIST
#define IFX_CFG_SSW_ENABLE_KEYOFF_MBIST                  (0U)
#endif

#ifndef IFX_CFG_SSW_ENABLE_KEYOFF_MBIST_DSPRS_DMARAM
#define IFX_CFG_SSW_ENABLE_KEYOFF_MBIST_DSPRS_DMARAM     (0U)
#endif

#endif /* __ARCH_TRICORE_SRC_ILLD_TC4XX_CONFIGURATIONS_IFX_CFG_SSW_H */
