/****************************************************************************
 * arch/tricore/include/tc3xx/irq.h
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

/* This file should never be included directly but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_TRICORE_INCLUDE_TC3XX_IRQ_H
#define __ARCH_TRICORE_INCLUDE_TC3XX_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* Service Request Control (SRC) register bit definitions.
 *
 * TC3xx layout:
 *   [7:0]   SRPN  - Service Request Priority Number
 *   [10]    SRE   - Service Request Enable
 *   [13:11] TOS   - Type of Service (3-bit, CPU0-5/DMA)
 *   No VM / CS fields.
 *
 * Common bits (identical on both TC3xx/TC4xx):
 *   [24] SRR, [25] CLRR, [26] SETR, [27] IOV, [28] IOVCLR
 */

#define SRCR_SRPN_SHIFT          0
#define SRCR_SRPN_MASK           (0xffu << SRCR_SRPN_SHIFT)
#define SRCR_VM_SHIFT            0
#define SRCR_VM_MASK             0
#define SRCR_CS                  0
#define SRCR_TOS_SHIFT           11
#define SRCR_TOS_MASK            (0x7u << SRCR_TOS_SHIFT)
#define SRCR_SRE                 (1u << 10)
#define SRCR_SRR                 (1u << 24)
#define SRCR_CLRR                (1u << 25)
#define SRCR_SETR                (1u << 26)
#define SRCR_IOV                 (1u << 27)
#define SRCR_IOVCLR              (1u << 28)

/* Service Request Control (SRC) */

#define TRICORE_SRC_BASEADDR     0xf0038000
#define TRICORE_SRC_GPSR0_SR0    0xf0038990
#define TRICORE_INT_SRB0_ADDR    0xf0037010
#define TRICORE_INT_LASR0_ADDR   0xf0037204
#define TRICORE_INT_LASR_SPACING 0x10

/* Type of Service (TOS) */

#define TRICORE_TOS_CPU0         0   /* CPU0 */
#define TRICORE_TOS_DMA          1   /* DMA */
#define TRICORE_TOS_CPU1         2   /* CPU1 */
#define TRICORE_TOS_CPU2         3   /* CPU2 */
#define TRICORE_TOS_CPU3         4   /* CPU3 */
#define TRICORE_TOS_CPU4         5   /* CPU4 */
#define TRICORE_TOS_CPU5         6   /* CPU5 */

#define TRICORE_UART_RX_IRQ      21

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
extern "C"
{
#endif

#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_TRICORE_INCLUDE_TC3XX_IRQ_H */
