/****************************************************************************
 * arch/arm/src/imxrt/hardware/rt118x/imxrt118x_trdc.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_TRDC_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_TRDC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/rt118x/imxrt118x_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RT1180 has three TRDC instances; the bases live in
 * imxrt118x_memorymap.h.
 *
 *   TRDC1 -> AONMIX                (0x44270000)
 *   TRDC2 -> WAKEUPMIX / MEGAMIX   (0x42460000)
 *   TRDC3 -> NETC / media          (0x42810000)
 *
 * The HWCFG0 offset and the MBC/MRC register offsets below are from the
 * RT1180 TRDC register layout.
 */

#define IMXRT_TRDC_HWCFG0_OFFSET        0xf0

#define IMXRT_MBC0_MEM_GLBAC(n)          (0x20 + (n << 2))
#define IMXRT_MBC_MEM_BLK_CFG_0(m, n)    (0x200 * m + 0x40 + (n << 2))
#define IMXRT_MBC_MEM_BLK_CFG_I(m, n, i) \
          (0x200 * m + 0x40 + (80 << 2) + (i - 1) * 0x28 + (n << 2))
#define IMXRT_MRC0_DOM_RGD_W(m, n)       (0x100 * m + 0x40 + (n << 3))

/* Fused-off peripheral filtering is not ported: RT1180 exposes no equivalent
 * FSB shadow at a known-good offset here, and nothing in this port depends
 * on it.  The fuse-handling code is therefore omitted rather than left
 * half-wired.
 */

#define DID_NUM 16
#define MBC_MAX_NUM 4
#define MRC_MAX_NUM 2
#define MBC_NUM(HWCFG) ((HWCFG >> 16) & 0xF)
#define MRC_NUM(HWCFG) ((HWCFG >> 24) & 0x1F)

#define MBC_BLK_NUM(GLBCFG) (GLBCFG & 0x3FF)
#define MRC_RGN_NUM(GLBCFG) (GLBCFG & 0x1F)

#define GLBAC_SETTING_MASK (0x7777)
#define GLBAC_LOCK_MASK	BIT(31)

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_TRDC_H */
