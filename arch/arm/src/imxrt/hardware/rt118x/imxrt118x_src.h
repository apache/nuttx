/****************************************************************************
 * arch/arm/src/imxrt/hardware/rt118x/imxrt118x_src.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_SRC_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_SRC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/imxrt_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SRC general registers (IMXRT1180RM Ch. 27) *******************************/

/* Register offsets *********************************************************/

#define IMXRT_SRC_AUTHEN_CTRL_OFFSET       0x0004
#define IMXRT_SRC_SCR_OFFSET               0x0010  /* Control */
#define IMXRT_SRC_SRTMR_OFFSET             0x0014  /* Reset Trigger */
#define IMXRT_SRC_SRMASK_OFFSET            0x0018  /* Reset Mask */
#define IMXRT_SRC_SBMR1_OFFSET             0x0040  /* Boot Mode 1 */
#define IMXRT_SRC_SBMR2_OFFSET             0x0044  /* Boot Mode 2 */
#define IMXRT_SRC_SRSR_BBSM_OFFSET         0x004c
#define IMXRT_SRC_SRSR_OFFSET              0x0050  /* Reset Status */
#define IMXRT_SRC_GPR0_OFFSET              0x0054  /* General Purpose Register 0 */

/* Register addresses *******************************************************/

#define IMXRT_SRC_AUTHEN_CTRL   (IMXRT_SRC_BASE + IMXRT_SRC_AUTHEN_CTRL_OFFSET)
#define IMXRT_SRC_SCR           (IMXRT_SRC_BASE + IMXRT_SRC_SCR_OFFSET)
#define IMXRT_SRC_SRTMR         (IMXRT_SRC_BASE + IMXRT_SRC_SRTMR_OFFSET)
#define IMXRT_SRC_SRMASK        (IMXRT_SRC_BASE + IMXRT_SRC_SRMASK_OFFSET)
#define IMXRT_SRC_SBMR1         (IMXRT_SRC_BASE + IMXRT_SRC_SBMR1_OFFSET)
#define IMXRT_SRC_SBMR2         (IMXRT_SRC_BASE + IMXRT_SRC_SBMR2_OFFSET)
#define IMXRT_SRC_SRSR_BBSM     (IMXRT_SRC_BASE + IMXRT_SRC_SRSR_BBSM_OFFSET)
#define IMXRT_SRC_SRSR          (IMXRT_SRC_BASE + IMXRT_SRC_SRSR_OFFSET)

#define IMXRT_SRC_GPR(n)        (IMXRT_SRC_BASE + IMXRT_SRC_GPR0_OFFSET + \
                                 ((n) * 4))

/* SRC.SCR bit fields */

#define SRC_SCR_BT_RELEASE_M7   (1u << 0)   /* Release M7 from boot hold */

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

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_SRC_H */
