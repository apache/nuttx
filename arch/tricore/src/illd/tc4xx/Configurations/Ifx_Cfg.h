/****************************************************************************
 * arch/tricore/src/illd/tc4xx/Configurations/Ifx_Cfg.h
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

#ifndef __ARCH_TRICORE_SRC_ILLD_TC4XX_CONFIGURATIONS_IFX_CFG_H
#define __ARCH_TRICORE_SRC_ILLD_TC4XX_CONFIGURATIONS_IFX_CFG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#define DEVICE_TC4DX  1

#define IFX_CFG_CLOCK_XTAL_FREQUENCY    (25000000)
#define IFX_CFG_CLOCK_SYSPLL_FREQUENCY  (500000000)
#define IFX_CFG_CLOCK_PPUPLL_FREQUENCY  (450000000)
#define IFX_CFG_CLOCK_PERPLL1_FREQUENCY (160000000)
#define IFX_CFG_CLOCK_PERPLL2_FREQUENCY (200000000)
#define IFX_CFG_CLOCK_PERPLL3_FREQUENCY (200000000)

#define IFX_CFG_CPU_CLOCK_FREQUENCY     IFX_CFG_CLOCK_SYSPLL_FREQUENCY

#define IFX_PROT_ENABLED                0
#define IFX_CFG_VIRTUALIZATION_DISABLED 1
#define IFX_CFG_STM_MORPHING_ENABLE     1
#define IFX_CLOCK_BYPASS_FOSC_F         0

#define IFX_STM_RESOULTION              IFX_CFG_CLOCK_SYSPLL_FREQUENCY

#define IFX_CFG_INTERRUPT_INTERVAL      (0.003)

#define IFX_CFG_CPU0_PRIO               10
#define IFX_CFG_CPU1_PRIO               10
#define IFX_CFG_CPU2_PRIO               10
#define IFX_CFG_CPU3_PRIO               10
#define IFX_CFG_CPU4_PRIO               10
#define IFX_CFG_CPU5_PRIO               10

#define ISR_PRIORITY_STM0_TICK          1
#define ISR_PRIORITY_GETH0_DMA0_TX      2
#define ISR_PRIORITY_GETH0_DMA0_RX      3

#ifdef CONFIG_TRICORE_BL
#  define IFX_CFG_SSW_ENABLE_TRICORE0 1
#  define IFX_CFG_SSW_ENABLE_TRICORE1 0
#  define IFX_CFG_SSW_ENABLE_TRICORE2 0
#  define IFX_CFG_SSW_ENABLE_TRICORE3 0
#  define IFX_CFG_SSW_ENABLE_TRICORE4 0
#  define IFX_CFG_SSW_ENABLE_TRICORE5 0
#else
#  define IFX_CFG_SSW_ENABLE_TRICORE0 1
#  define IFX_CFG_SSW_ENABLE_TRICORE1 1
#  define IFX_CFG_SSW_ENABLE_TRICORE2 1
#  define IFX_CFG_SSW_ENABLE_TRICORE3 1
#  define IFX_CFG_SSW_ENABLE_TRICORE4 1
#  define IFX_CFG_SSW_ENABLE_TRICORE5 1
#endif

#endif /* __ARCH_TRICORE_SRC_ILLD_TC4XX_CONFIGURATIONS_IFX_CFG_H */
