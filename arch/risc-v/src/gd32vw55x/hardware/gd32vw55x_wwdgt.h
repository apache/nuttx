/****************************************************************************
 * arch/risc-v/src/gd32vw55x/hardware/gd32vw55x_wwdgt.h
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

#ifndef __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_WWDGT_H
#define __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_WWDGT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "gd32vw55x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define GD32VW55X_WWDGT_CTL_OFFSET   0x0000  /* Control register */
#define GD32VW55X_WWDGT_CFG_OFFSET   0x0004  /* Configuration register */
#define GD32VW55X_WWDGT_STAT_OFFSET  0x0008  /* Status register */

/* Register addresses *******************************************************/

#define GD32VW55X_WWDGT_CTL  (GD32VW55X_WWDGT_BASE + GD32VW55X_WWDGT_CTL_OFFSET)
#define GD32VW55X_WWDGT_CFG  (GD32VW55X_WWDGT_BASE + GD32VW55X_WWDGT_CFG_OFFSET)
#define GD32VW55X_WWDGT_STAT (GD32VW55X_WWDGT_BASE + GD32VW55X_WWDGT_STAT_OFFSET)

/* Register bit definitions *************************************************/

/* Control register */

#define WWDGT_CTL_CNT_SHIFT          (0)       /* Bits 0-6: Counter value */
#define WWDGT_CTL_CNT_MASK           (0x7f << WWDGT_CTL_CNT_SHIFT)
#  define WWDGT_CTL_CNT_MAX          (0x3f << WWDGT_CTL_CNT_SHIFT)
#  define WWDGT_CTL_CNT_RESET        (0x40 << WWDGT_CTL_CNT_SHIFT)
#define WWDGT_CTL_WDGTEN             (1 << 7)  /* Bit 7: Counter enable */

/* Configuration register */

#define WWDGT_CFG_WIN_SHIFT          (0)       /* Bits 0-6: Window value */
#define WWDGT_CFG_WIN_MASK           (0x7f << WWDGT_CFG_WIN_SHIFT)
#define WWDGT_CFG_PSC_SHIFT          (7)       /* Bits 7-8: Prescaler */
#define WWDGT_CFG_PSC_MASK           (3 << WWDGT_CFG_PSC_SHIFT)
#  define WWDGT_CFG_PSC_DIV1         (0 << WWDGT_CFG_PSC_SHIFT)
#  define WWDGT_CFG_PSC_DIV2         (1 << WWDGT_CFG_PSC_SHIFT)
#  define WWDGT_CFG_PSC_DIV4         (2 << WWDGT_CFG_PSC_SHIFT)
#  define WWDGT_CFG_PSC_DIV8         (3 << WWDGT_CFG_PSC_SHIFT)
#define WWDGT_CFG_EWIE               (1 << 9)  /* Bit 9: Early wakeup int en */

/* Status register */

#define WWDGT_STAT_EWIF              (1 << 0)  /* Early wakeup int flag */

#endif /* __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_WWDGT_H */
