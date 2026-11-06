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

#define TRICORE_UART_RX_IRQ      21

/* Interrupt router: the IRQ number is the index of the service request
 * control (SRC) register from TRICORE_SRC_BASE.
 */

#define TRICORE_SRC_BASE         0xf0038000  /* SRC_CPU0SB, IRQ 0 */
#define TRICORE_SRC_GPSR_BASE    0xf0038990  /* SRC_GPSR0SR0 */
#define TRICORE_INT_SRB_BASE     0xf0037010  /* INT_SRB0 */

#define TRICORE_SRC_SRPN_MASK    0xff        /* Service request priority */
#define TRICORE_SRC_SRE          (1 << 10)   /* Service request enable */
#define TRICORE_SRC_TOS_SHIFT    11          /* Type of service (core) */
#define TRICORE_SRC_TOS_MASK     (0x7 << TRICORE_SRC_TOS_SHIFT)
#define TRICORE_SRC_CLRR         (1 << 25)   /* Clear request */

/* TOS of a core: 0 is CPU0, 1 is the DMA, 2 to 6 are CPU1 to CPU5 */

#define TRICORE_SRC_TOS_CPU(n)   ((n) == 0 ? 0 : (n) + 1)

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
