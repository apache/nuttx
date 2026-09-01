/****************************************************************************
 * arch/arm/src/imxrt/hardware/rt118x/imxrt118x_gpc.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_GPC_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_GPC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/imxrt_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPC_CPU_CTRL (IMXRT1180RM Ch. 34).  Two instances share the same base:
 *   index 0 (offset 0x000) - Cortex-M33
 *   index 1 (offset 0x800) - Cortex-M7
 */

#define IMXRT_GPC_CM_STEP               (0x800)

#define IMXRT_GPC_CM_AUTHEN_CTRL(idx)   (IMXRT_GPC_CPU_CTRL_BASE + 0x004 + \
                                         (idx) * IMXRT_GPC_CM_STEP)
#define IMXRT_GPC_CM_MISC(idx)          (IMXRT_GPC_CPU_CTRL_BASE + 0x00c + \
                                         (idx) * IMXRT_GPC_CM_STEP)
#define IMXRT_GPC_CM_MODE_CTRL(idx)     (IMXRT_GPC_CPU_CTRL_BASE + 0x010 + \
                                         (idx) * IMXRT_GPC_CM_STEP)
#define IMXRT_GPC_CM_MODE_STAT(idx)     (IMXRT_GPC_CPU_CTRL_BASE + 0x014 + \
                                         (idx) * IMXRT_GPC_CM_STEP)

/* Convenience aliases */

#define IMXRT_GPC_CM33_MODE_CTRL        IMXRT_GPC_CM_MODE_CTRL(0)
#define IMXRT_GPC_CM33_MODE_STAT        IMXRT_GPC_CM_MODE_STAT(0)
#define IMXRT_GPC_CM7_MODE_CTRL         IMXRT_GPC_CM_MODE_CTRL(1)
#define IMXRT_GPC_CM7_MODE_STAT         IMXRT_GPC_CM_MODE_STAT(1)

/* CM_MODE_CTRL bit fields */

#define GPC_CM_MODE_CTRL_TARGET_SHIFT   (0)
#define GPC_CM_MODE_CTRL_TARGET_MASK    (0x3u << GPC_CM_MODE_CTRL_TARGET_SHIFT)
#define GPC_CM_MODE_CTRL_TARGET(x)      (((x) << GPC_CM_MODE_CTRL_TARGET_SHIFT) & \
                                         GPC_CM_MODE_CTRL_TARGET_MASK)
#define GPC_CM_MODE_TARGET_RUN          (0)
#define GPC_CM_MODE_TARGET_WAIT         (1)
#define GPC_CM_MODE_TARGET_STOP         (2)
#define GPC_CM_MODE_TARGET_SUSPEND      (3)

#define GPC_CM_MODE_CTRL_WFE_EN         (1u << 4)   /* WFE can trigger low power */

/* CM_MODE_STAT bit fields */

#define GPC_CM_MODE_STAT_CURRENT_SHIFT  (0)
#define GPC_CM_MODE_STAT_CURRENT_MASK   (0x3u << GPC_CM_MODE_STAT_CURRENT_SHIFT)
#define GPC_CM_MODE_STAT_PREVIOUS_SHIFT (2)
#define GPC_CM_MODE_STAT_PREVIOUS_MASK  (0x3u << GPC_CM_MODE_STAT_PREVIOUS_SHIFT)

/* CM_MISC bit fields */

#define GPC_CM_MISC_NMI_STAT            (1u << 0)
#define GPC_CM_MISC_SLEEP_HOLD_EN       (1u << 1)
#define GPC_CM_MISC_SLEEP_HOLD_STAT     (1u << 2)

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_GPC_H */
