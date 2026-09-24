/****************************************************************************
 * arch/arm/src/imxrt/hardware/rt117x/imxrt117x_src.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_RT117X_IMXRT117X_SRC_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_RT117X_IMXRT117X_SRC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/imxrt_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* System Reset Controller.  Partial: the registers that boot, hold and
 * diagnose the CM4 (RM chapter 25); the per-slice authentication and the
 * other reset slices are not described yet.
 */

#define IMXRT_SRC_SCR_OFFSET              0x0000  /* SRC Control */
#define IMXRT_SRC_SRMR_OFFSET             0x0004  /* SRC Reset Mode */
#define IMXRT_SRC_SRSR_OFFSET             0x0010  /* SRC Reset Status */
#define IMXRT_SRC_CTRL_M4CORE_OFFSET      0x0284  /* M4 core slice control */
#define IMXRT_SRC_STAT_M4CORE_OFFSET      0x0290  /* M4 core slice status */

#define IMXRT_SRC_SCR                     (IMXRT_SRC_BASE + IMXRT_SRC_SCR_OFFSET)
#define IMXRT_SRC_SRMR                    (IMXRT_SRC_BASE + IMXRT_SRC_SRMR_OFFSET)
#define IMXRT_SRC_SRSR                    (IMXRT_SRC_BASE + IMXRT_SRC_SRSR_OFFSET)
#define IMXRT_SRC_CTRL_M4CORE             (IMXRT_SRC_BASE + IMXRT_SRC_CTRL_M4CORE_OFFSET)
#define IMXRT_SRC_STAT_M4CORE             (IMXRT_SRC_BASE + IMXRT_SRC_STAT_M4CORE_OFFSET)

#define SRC_SCR_BT_RELEASE_M4             (1 << 0)  /* Release CM4 from reset */

/* Reset mode per source: 0 resets the system, 3 resets nothing. */

#define SRC_SRMR_M4LOCKUP_SHIFT           (6)
#define SRC_SRMR_M4LOCKUP_MASK            (3 << SRC_SRMR_M4LOCKUP_SHIFT)
#define SRC_SRMR_M4LOCKUP_NONE            (3 << SRC_SRMR_M4LOCKUP_SHIFT)

/* Reset status: sticky causes, write 1 to clear. */

#define SRC_SRSR_M7_LOCKUP                (1 << 2)  /* CM7 lockup */
#define SRC_SRSR_WDOG                     (1 << 5)  /* WDOG1 timeout */
#define SRC_SRSR_M4_LOCKUP                (1 << 12) /* CM4 lockup */

#define SRC_CTRL_M4CORE_SW_RESET          (1 << 0)  /* Software reset of CM4 slice */
#define SRC_STAT_M4CORE_UNDER_RST         (1 << 0)  /* CM4 slice in reset */

/* IOMUXC LPSR GPR0/1 hold the CM4 initial vector table address. */

#define IMXRT_IOMUXC_LPSR_GPR_GPR0_OFFSET (0x0000)
#define IMXRT_IOMUXC_LPSR_GPR_GPR1_OFFSET (0x0004)

#define IMXRT_IOMUXC_LPSR_GPR_GPR0        (IMXRT_IOMUXCLPSRGPR_BASE + IMXRT_IOMUXC_LPSR_GPR_GPR0_OFFSET)
#define IMXRT_IOMUXC_LPSR_GPR_GPR1        (IMXRT_IOMUXCLPSRGPR_BASE + IMXRT_IOMUXC_LPSR_GPR_GPR1_OFFSET)

/* GPR0[15:3] holds VTOR[15:3], GPR1[15:0] holds VTOR[31:16]. */

#define GPR0_CM4_INIT_VTOR_LOW(vtor)      ((vtor) & 0x0000fff8)
#define GPR1_CM4_INIT_VTOR_HIGH(vtor)     (((vtor) >> 16) & 0x0000ffff)

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_RT117X_IMXRT117X_SRC_H */
