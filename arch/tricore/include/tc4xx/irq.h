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

#define TRICORE_UART_RX_IRQ      173

/* Interrupt router: the IRQ number is the index of the service request
 * control (SRC) register from TRICORE_SRC_BASE.
 */

#define TRICORE_SRC_BASE         0xf4432000  /* SRC_CPU0SB, IRQ 0 */
#define TRICORE_SRC_GPSR_BASE    0xf4433460  /* SRC_GPSR0SR0 */
#define TRICORE_INT_SRB_BASE     0xf4430b00  /* INT_SRB0 */

#define TRICORE_SRC_SRPN_MASK    0xff        /* Service request priority */
#define TRICORE_SRC_VM_MASK      (0x7 << 8)  /* Virtual machine */
#define TRICORE_SRC_CS           (1 << 11)   /* Set by the CSRM core */
#define TRICORE_SRC_TOS_SHIFT    12          /* Type of service (core) */
#define TRICORE_SRC_TOS_MASK     (0xf << TRICORE_SRC_TOS_SHIFT)
#define TRICORE_SRC_SRE          (1 << 23)   /* Service request enable */
#define TRICORE_SRC_CLRR         (1 << 25)   /* Clear request */

/* TOS of a core: 0 to 5 are CPU0 to CPU5, 6 is the CSRM core */

#define TRICORE_SRC_TOS_CPU(n)   (n)

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
