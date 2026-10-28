/****************************************************************************
 * arch/tricore/src/common/tricore_irq.h
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

#ifndef __ARCH_TRICORE_SRC_COMMON_TRICORE_IRQ_H
#define __ARCH_TRICORE_SRC_COMMON_TRICORE_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NR_IRQS                     2048

#define TRICORE_DEFAULT_IR_PRIO     255
#define TRICORE_DEFAULT_IR_TOS      0

#if defined(CONFIG_ARCH_CHIP_FAMILY_TC4X)
#  define TRICORE_IR_INT_BASE       0xf4430000
#  define TRICORE_IR_SRC_BASE       0xf4432000
#  define TRICORE_STM_BASE_SRN      8
#elif defined(CONFIG_ARCH_CHIP_FAMILY_TC3X)
#  define TRICORE_IR_INT_BASE       0xf0037000
#  define TRICORE_IR_SRC_BASE       0xf0038000
#  define TRICORE_STM_BASE_SRN      192
#endif

#define TRICORE_IR_GET_SRC(irq)     (TRICORE_IR_SRC_BASE + ((irq) * 4))

#define SRCR_SRPN_SHIFT             0
#define SRCR_SRPN_MASK              (0xff << SRCR_SRPN_SHIFT)

#if defined(CONFIG_ARCH_CHIP_FAMILY_TC4X)

#  define SRCR_VM_SHIFT             8
#  define SRCR_VM_MASK              (0x7 << SRCR_VM_SHIFT)

#  define SRCR_CS                   BIT(11)

#  define SRCR_TOS_SHIFT            12
#  define SRCR_TOS_MASK             (0xf << SRCR_TOS_SHIFT)

#  define SRCR_SRE                  BIT(23)
#  define SRCR_SRR                  BIT(24)
#  define SRCR_CLRR                 BIT(25)
#  define SRCR_SETR                 BIT(26)
#  define SRCR_IOV                  BIT(27)
#  define SRCR_IOVCLR               BIT(28)

#  define TRICORE_IR_LASR_OFFSET    0x0c20
#  define TRICORE_IR_LASR_STRIDE    0x34
#  define TRICORE_IR_LASR_PIPN_SHIFT 16
#  define TRICORE_IR_LASR_PIPN_MASK (0x7ff << TRICORE_IR_LASR_PIPN_SHIFT)

#  define TRICORE_IR_GET_TOS(coreid) (coreid)

#elif defined(CONFIG_ARCH_CHIP_FAMILY_TC3X)

#  define SRCR_SRE                  (1u << 10)

#  define SRCR_TOS_SHIFT            11
#  define SRCR_TOS_MASK             (0x7 << SRCR_TOS_SHIFT)

#  define SRCR_ECC_SHIFT            16
#  define SRCR_ECC_MASK             (0x1f << SRCR_ECC_SHIFT)

#  define SRCR_SRR                  BIT(24)
#  define SRCR_CLRR                 BIT(25)
#  define SRCR_SETR                 BIT(26)
#  define SRCR_IOV                  BIT(27)
#  define SRCR_IOVCLR               BIT(28)
#  define SRCR_SWS                  BIT(29)
#  define SRCR_SWSCLR               BIT(30)

#  define TRICORE_IR_LASR_OFFSET    0x204
#  define TRICORE_IR_LASR_STRIDE    0x10
#  define TRICORE_IR_LASR_PIPN_SHIFT 16
#  define TRICORE_IR_LASR_PIPN_MASK (0x3ff << TRICORE_IR_LASR_PIPN_SHIFT)

#  define TRICORE_IR_GET_TOS(coreid) ((coreid) == 0 ? 0 : (coreid) + 1)

#endif

#define TRICORE_IR_LASR(tos) \
  (TRICORE_IR_INT_BASE + TRICORE_IR_LASR_OFFSET + \
   ((tos) * TRICORE_IR_LASR_STRIDE))

#define TRICORE_STM_IR_OFFSET       0x20
#define TRICORE_STM_IR_CPUw_SRx(w, x) \
  (TRICORE_IR_SRC_BASE + TRICORE_STM_IR_OFFSET + \
   ((w) * 0x40) + ((x) * 0x4))
#define TRICORE_STM_IR_CPU0_SR(x)   TRICORE_STM_IR_CPUw_SRx(0, (x))
#define TRICORE_STM_IR_SRN(x)       (TRICORE_STM_BASE_SRN + (x))

#endif /* __ARCH_TRICORE_SRC_COMMON_TRICORE_IRQ_H */
