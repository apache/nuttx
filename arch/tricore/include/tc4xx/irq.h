/****************************************************************************
 * arch/tricore/include/tc4xx/irq.h
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

#ifndef __ARCH_TRICORE_INCLUDE_TC4XX_IRQ_H
#define __ARCH_TRICORE_INCLUDE_TC4XX_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* Service Request Control (SRC) */

#define TRICORE_SRC_BASEADDR     0xf4432000  /* TC4DX */
#define TRICORE_SRC_GPSR0_SR0    0xf4433460  /* TC4DX */
#define TRICORE_INT_SRB0_ADDR    0xf4430b00  /* TC4DX */

/* Type of Service (TOS) */

#define TRICORE_TOS_CPU0         0   /* CPU0 */
#define TRICORE_TOS_CPU1         1   /* CPU1 */
#define TRICORE_TOS_CPU2         2   /* CPU2 */
#define TRICORE_TOS_CPU3         3   /* CPU3 */
#define TRICORE_TOS_CPU4         4   /* CPU4 */
#define TRICORE_TOS_CPU5         5   /* CPU5 */
#define TRICORE_TOS_CSRM         6   /* CSRM */
#define TRICORE_TOS_DMA0         7   /* DMA0 */
#define TRICORE_TOS_DMA1         8   /* DMA1 */
#define TRICORE_TOS_GTM          9   /* GTM */
#define TRICORE_TOS_PPU          10  /* PPU */

#define TRICORE_UART_RX_IRQ      173

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

#endif /* __ARCH_TRICORE_INCLUDE_TC4XX_IRQ_H */
