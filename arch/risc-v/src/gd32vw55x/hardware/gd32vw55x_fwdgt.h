/****************************************************************************
 * arch/risc-v/src/gd32vw55x/hardware/gd32vw55x_fwdgt.h
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

#ifndef __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_FWDGT_H
#define __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_FWDGT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "gd32vw55x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define GD32VW55X_FWDGT_CTL_OFFSET   0x0000  /* Control register */
#define GD32VW55X_FWDGT_PSC_OFFSET   0x0004  /* Prescaler register */
#define GD32VW55X_FWDGT_RLD_OFFSET   0x0008  /* Reload register */
#define GD32VW55X_FWDGT_STAT_OFFSET  0x000c  /* Status register */

/* Register addresses *******************************************************/

#define GD32VW55X_FWDGT_CTL  (GD32VW55X_FWDGT_BASE + GD32VW55X_FWDGT_CTL_OFFSET)
#define GD32VW55X_FWDGT_PSC  (GD32VW55X_FWDGT_BASE + GD32VW55X_FWDGT_PSC_OFFSET)
#define GD32VW55X_FWDGT_RLD  (GD32VW55X_FWDGT_BASE + GD32VW55X_FWDGT_RLD_OFFSET)
#define GD32VW55X_FWDGT_STAT (GD32VW55X_FWDGT_BASE + GD32VW55X_FWDGT_STAT_OFFSET)

/* Register bit definitions *************************************************/

/* Control register */

#define FWDGT_CTL_CMD_SHIFT          (0)       /* Bits 0-15: Command value */
#define FWDGT_CTL_CMD_MASK           (0xffff << FWDGT_CTL_CMD_SHIFT)

/* Key values written to the CTL register */

#define FWDGT_CTL_KEY_WRITEEN        0x5555    /* Enable PSC/RLD write access */
#define FWDGT_CTL_KEY_WRITEDIS       0x0000    /* Disable PSC/RLD write access */
#define FWDGT_CTL_KEY_RELOAD         0xaaaa    /* Reload the counter */
#define FWDGT_CTL_KEY_ENABLE         0xcccc    /* Start the counter */

/* Prescaler register */

#define FWDGT_PSC_SHIFT              (0)       /* Bits 0-2: Prescaler divider */
#define FWDGT_PSC_MASK               (7 << FWDGT_PSC_SHIFT)
#  define FWDGT_PSC_DIV4             (0 << FWDGT_PSC_SHIFT)  /* Divide by 4 */
#  define FWDGT_PSC_DIV8             (1 << FWDGT_PSC_SHIFT)  /* Divide by 8 */
#  define FWDGT_PSC_DIV16            (2 << FWDGT_PSC_SHIFT)  /* Divide by 16 */
#  define FWDGT_PSC_DIV32            (3 << FWDGT_PSC_SHIFT)  /* Divide by 32 */
#  define FWDGT_PSC_DIV64            (4 << FWDGT_PSC_SHIFT)  /* Divide by 64 */
#  define FWDGT_PSC_DIV128           (5 << FWDGT_PSC_SHIFT)  /* Divide by 128 */
#  define FWDGT_PSC_DIV256           (6 << FWDGT_PSC_SHIFT)  /* Divide by 256 */

#define FWDGT_PSC_MAX                (6)       /* Largest prescaler selection */

/* Reload register */

#define FWDGT_RLD_SHIFT              (0)       /* Bits 0-11: Reload value */
#define FWDGT_RLD_MASK               (0xfff << FWDGT_RLD_SHIFT)
#define FWDGT_RLD_MAX                (0xfff)   /* Maximum reload value */

/* Status register */

#define FWDGT_STAT_PUD               (1 << 0)  /* Prescaler update ongoing */
#define FWDGT_STAT_RUD               (1 << 1)  /* Reload update ongoing */

#endif /* __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_FWDGT_H */
