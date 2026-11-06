/****************************************************************************
 * arch/tricore/src/common/tricore_stm.h
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

#ifndef __ARCH_TRICORE_SRC_COMMON_TRICORE_STM_H
#define __ARCH_TRICORE_SRC_COMMON_TRICORE_STM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* System timer (STM) of CPU0 and its interrupt.  On TC4xx the STM is part
 * of the CPU0 SFR space and has a single 64-bit timer register.
 */

#ifdef CONFIG_ARCH_CHIP_TC4XX
#  define TRICORE_STM0_BASE        0xf8800000  /* MODULE_CPU0 */
#  define TRICORE_STM0_IRQ         10          /* SRC_STMCPU0_SR2 */

#  define TRICORE_STM_ABS_OFFSET   0x20        /* 64-bit timer */
#  define TRICORE_STM_CMP0_OFFSET  0x120       /* Compare register 0 */
#  define TRICORE_STM_CMCON_OFFSET 0x128       /* Compare match control */
#  define TRICORE_STM_ICR_OFFSET   0x12c       /* Interrupt control */
#  define TRICORE_STM_ISCR_OFFSET  0x130       /* Interrupt set/clear */
#  define TRICORE_STM_OCS_OFFSET   0x4         /* OCDS control and status */

#  define TRICORE_STM_ICR_CMP0OS   (1 << 1)    /* CMP0 output, 0: STMIR0 */
#else
#  define TRICORE_STM0_BASE        0xf0001000  /* MODULE_STM0 */
#  define TRICORE_STM0_IRQ         192         /* SRC_STM0SR0 */

#  define TRICORE_STM_TIM0_OFFSET  0x10        /* Timer bits [31:0] */
#  define TRICORE_STM_CAP_OFFSET   0x2c        /* Timer bits [63:32] */
#  define TRICORE_STM_CMP0_OFFSET  0x30        /* Compare register 0 */
#  define TRICORE_STM_CMCON_OFFSET 0x38        /* Compare match control */
#  define TRICORE_STM_ICR_OFFSET   0x3c        /* Interrupt control */
#  define TRICORE_STM_ISCR_OFFSET  0x40        /* Interrupt set/clear */
#  define TRICORE_STM_OCS_OFFSET   0xe8        /* OCDS control and status */

#  define TRICORE_STM_ICR_CMP0OS   (1 << 2)    /* CMP0 output, 0: STMIR0 */

/* OCS can only be written with OCDS enabled (CBS_OSTATE.OEN) */

#  define TRICORE_CBS_OSTATE       0xf0000480
#  define TRICORE_CBS_OSTATE_OEN   (1 << 0)
#endif

#define TRICORE_STM_CMCON_MSIZE0   (0x1f << 0) /* CMP0 compare size - 1 */
#define TRICORE_STM_CMCON_MSTART0  (0x1f << 8) /* CMP0 first compared bit */
#define TRICORE_STM_ICR_CMP0EN     (1 << 0)    /* CMP0 interrupt enable */
#define TRICORE_STM_ISCR_CMP0IRR   (1 << 0)    /* Clear CMP0 request */
#define TRICORE_STM_OCS_SUS_HARD   (1 << 24)   /* Hard suspend on halt */
#define TRICORE_STM_OCS_SUS_MASK   (0xf << 24)
#define TRICORE_STM_OCS_SUS_P      (1 << 28)   /* SUS write protection */

#endif /* __ARCH_TRICORE_SRC_COMMON_TRICORE_STM_H */
