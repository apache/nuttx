/****************************************************************************
 * arch/arm64/src/imx9/hardware/imx9_trdc.h
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

#ifndef __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX9_TRDC_H
#define __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX9_TRDC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/imx9_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMX9_TRDC_HWCFG0                (IMX9_TRDC_BASE + 0xf0)

#define IMX9_MBC0_MEM_GLBAC(n)          (0x20 + (n << 2))
#define IMX9_MBC_MEM_BLK_CFG_0(m, n)    (0x200 * m + 0x40 + (n << 2))
#define IMX9_MBC_MEM_BLK_CFG_I(m, n, i) (0x200 * m + 0x40 + (80 << 2) + (i - 1) * 0x28 + (n << 2))
#define IMX9_MRC0_DOM_RGD_W(m, n)       (0x100 * m + 0x40 + (n << 3))

#define FSB_BASE                0x47510000UL
#define FSB_SHADOW_OFF          0x8000UL

#define BLK_CTRL_NS_ANOMIX_BASE  IMX9_BLK_CTRL_NS_AONMIX1_BASE

#define DID_NUM 16
#define MBC_MAX_NUM 4
#define MRC_MAX_NUM 2
#define MBC_NUM(HWCFG) ((HWCFG >> 16) & 0xF)
#define MRC_NUM(HWCFG) ((HWCFG >> 24) & 0x1F)

#define MBC_BLK_NUM(GLBCFG) (GLBCFG & 0x3FF)
#define MRC_RGN_NUM(GLBCFG) (GLBCFG & 0x1F)

#define GLBAC_SETTING_MASK (0x7777)
#define GLBAC_LOCK_MASK	BIT(31)

#endif /* __ARCH_ARM64_SRC_IMX9_HARDWARE_IMX9_TRDC_H */
