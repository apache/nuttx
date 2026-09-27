/****************************************************************************
 * arch/arm/src/imxrt/hardware/rt118x/imxrt118x_blkctrl.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_BLKCTRL_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_BLKCTRL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/imxrt_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* BLK_CTRL_S_AONMIX registers (IMXRT1180RM Ch. 15.4.1).
 *
 * The AON secure block controls the CM33 and CM7 interrupt masks, initial
 * vector table pointers, low-power handshakes, DAP access sticky bits and
 * NMI routing.  Bases for the non-secure alias are provided in
 * hardware/rt118x/imxrt118x_memorymap.h.
 */

#define IMXRT_AON_CM33_IRQ_MASK(n)      (IMXRT_BLK_CTRL_S_AONMIX_BASE + 0x000 + \
                                         ((n) << 2))       /* 8 x 32b IRQ mask */
#define IMXRT_AON_CM7_IRQ_MASK(n)       (IMXRT_BLK_CTRL_S_AONMIX_BASE + 0x020 + \
                                         ((n) << 2))
#define IMXRT_AON_EDGELOCK_RESET_MASK   (IMXRT_BLK_CTRL_S_AONMIX_BASE + 0x058)
#define IMXRT_AON_EDGELOCK_IRQ_MASK     (IMXRT_BLK_CTRL_S_AONMIX_BASE + 0x05c)
#define IMXRT_AON_M33_CFG               (IMXRT_BLK_CTRL_S_AONMIX_BASE + 0x060)
#define IMXRT_AON_M33_INITSVTOR         (IMXRT_BLK_CTRL_S_AONMIX_BASE + 0x064)
#define IMXRT_AON_M33_INITNSVTOR        (IMXRT_BLK_CTRL_S_AONMIX_BASE + 0x068)
#define IMXRT_AON_M7_CFG                (IMXRT_BLK_CTRL_S_AONMIX_BASE + 0x080)
#define IMXRT_AON_AXBS_AON_CTRL         (IMXRT_BLK_CTRL_S_AONMIX_BASE + 0x090)
#define IMXRT_AON_DAP_ACCESS_STKYBIT    (IMXRT_BLK_CTRL_S_AONMIX_BASE + 0x100)
#define IMXRT_AON_LP_HANDSHAKE          (IMXRT_BLK_CTRL_S_AONMIX_BASE + 0x110)
#define IMXRT_AON_EDGELOCK_HALT_ST      (IMXRT_BLK_CTRL_S_AONMIX_BASE + 0x114)
#define IMXRT_AON_ECC_MEM_INIT          (IMXRT_BLK_CTRL_S_AONMIX_BASE + 0x120)
#define IMXRT_AON_IOMUXC_DOMAIN_CFG     (IMXRT_BLK_CTRL_S_AONMIX_BASE + 0x148)
#define IMXRT_AON_IOMUXC_AON_DOMAIN_CFG (IMXRT_BLK_CTRL_S_AONMIX_BASE + 0x14c)
#define IMXRT_AON_NMI_CTRL              (IMXRT_BLK_CTRL_S_AONMIX_BASE + 0x154)

/* M33_CFG bit fields */

#define AON_M33_CFG_LOCK_NSVTOR         (1u << 0)
#define AON_M33_CFG_LOCK_SVTOR          (1u << 1)
#define AON_M33_CFG_TCM_SIZE_SHIFT      (3)
#define AON_M33_CFG_TCM_SIZE_MASK       (0x3u << AON_M33_CFG_TCM_SIZE_SHIFT)

/* M7_CFG bit fields */

#define AON_M7_CFG_TCM_SIZE_SHIFT       (0)
#define AON_M7_CFG_TCM_SIZE_MASK        (0x7u)
#define AON_M7_CFG_WAIT                 (1u << 4)
#define AON_M7_CFG_CORECLK_FORCE_ON     (1u << 5)
#define AON_M7_CFG_HCLK_FORCE_ON        (1u << 6)
#define AON_M7_CFG_INITVTOR_SHIFT       (7)
#define AON_M7_CFG_INITVTOR_MASK        (0x01ffffffu << AON_M7_CFG_INITVTOR_SHIFT)
#define AON_M7_CFG_INITVTOR(x)          (((x) << AON_M7_CFG_INITVTOR_SHIFT) & \
                                         AON_M7_CFG_INITVTOR_MASK)

/* NMI_CTRL bit fields */

#define AON_NMI_CTRL_M7_NMI_MASK        (1u << 0)
#define AON_NMI_CTRL_M33_NMI_MASK       (1u << 1)

/* SRC general registers (IMXRT1180RM Ch. 27) *******************************/

#define IMXRT_SRC_AUTHEN_CTRL           (IMXRT_SRC_BASE + 0x004)
#define IMXRT_SRC_SCR                   (IMXRT_SRC_BASE + 0x010)  /* Control */
#define IMXRT_SRC_SRTMR                 (IMXRT_SRC_BASE + 0x014)  /* Reset Trigger */
#define IMXRT_SRC_SRMASK                (IMXRT_SRC_BASE + 0x018)  /* Reset Mask */
#define IMXRT_SRC_SBMR1                 (IMXRT_SRC_BASE + 0x040)  /* Boot Mode 1 */
#define IMXRT_SRC_SBMR2                 (IMXRT_SRC_BASE + 0x044)  /* Boot Mode 2 */
#define IMXRT_SRC_SRSR_BBSM             (IMXRT_SRC_BASE + 0x04c)
#define IMXRT_SRC_SRSR                  (IMXRT_SRC_BASE + 0x050)  /* Reset Status */

/* SRC.SCR bit fields */

#define SRC_SCR_BT_RELEASE_M7           (1u << 0)   /* Release M7 from boot hold */

/* SRC MIX_SLICE registers (IMXRT1180RM Ch. 27, per power domain slice).
 * Bases for each slice (AON/WAKEUP/MEGA/NETC/CM33/CM7 platforms) are in
 * hardware/rt118x/imxrt118x_memorymap.h.
 */

#define IMXRT_SRC_SLICE_AUTHEN_CTRL(base)          ((base) + 0x004)
#define IMXRT_SRC_SLICE_SW_CTRL(base)              ((base) + 0x010)
#define IMXRT_SRC_SLICE_FUNC_STAT(base)            ((base) + 0x014)
#define IMXRT_SRC_SLICE_UPI_STAT_0(base)           ((base) + 0x020)
#define IMXRT_SRC_SLICE_UPI_STAT_1(base)           ((base) + 0x024)
#define IMXRT_SRC_SLICE_LPM_SETTING_0(base)        ((base) + 0x030)
#define IMXRT_SRC_SLICE_LPM_SETTING_1(base)        ((base) + 0x034)
#define IMXRT_SRC_SLICE_LPM_SETTING_2(base)        ((base) + 0x038)
#define IMXRT_SRC_SLICE_EDGELOCK_HDSK_CTRL(base)   ((base) + 0x040)
#define IMXRT_SRC_SLICE_EDGELOCK_HDSK_STAT(base)   ((base) + 0x044)
#define IMXRT_SRC_SLICE_PSW_CTRL(base)             ((base) + 0x05c)
#define IMXRT_SRC_SLICE_PSW_STAT(base)             ((base) + 0x060)
#define IMXRT_SRC_SLICE_MLPL_CFG(base)             ((base) + 0x084)
#define IMXRT_SRC_SLICE_MLPL_STAT(base)            ((base) + 0x088)

/* SLICE_SW_CTRL bit fields */

#define SRC_SLICE_SW_CTRL_PSW_OFF_SOFT      (1u << 0)   /* 1 = software power off */
#define SRC_SLICE_SW_CTRL_RST_CTRL_SOFT     (1u << 2)   /* 1 = software reset assert */
#define SRC_SLICE_SW_CTRL_ISO_ON_SOFT       (1u << 4)   /* 1 = software isolation on */
#define SRC_SLICE_SW_CTRL_EDGELOCK_HDSK_SOFT (1u << 6)  /* Edgelock handshake */
#define SRC_SLICE_SW_CTRL_PDN_SOFT          (1u << 31)  /* 1 = SW power-down sequence */

/* FUNC_STAT bit fields (all read-only) */

#define SRC_SLICE_FUNC_STAT_PSW_STAT        (1u << 0)   /* 0 = power on, 1 = power off */
#define SRC_SLICE_FUNC_STAT_RST_STAT        (1u << 2)   /* 0 = reset held, 1 = released */
#define SRC_SLICE_FUNC_STAT_ISO_STAT        (1u << 4)   /* 0 = isolation off */
#define SRC_SLICE_FUNC_STAT_EDGELOCK_HDSK   (1u << 6)   /* Edgelock handshake done */

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_BLKCTRL_H */
