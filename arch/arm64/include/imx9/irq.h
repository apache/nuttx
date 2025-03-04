/****************************************************************************
 * arch/arm64/include/imx9/irq.h
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

#ifndef __ARCH_ARM64_INCLUDE_IMX9_IRQ_H
#define __ARCH_ARM64_INCLUDE_IMX9_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_ARCH_CHIP_IMX93)
#  include <arch/imx9/imx93_irq.h>
#else
#  error "Unrecognized i.MX9 architecture"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMX9_IRQ_SOFTWARE0          (0)   /* Cortex-A55 Software Generated Interrupt 0 */
#define IMX9_IRQ_SOFTWARE1          (1)   /* Cortex-A55 Software Generated Interrupt 1 */
#define IMX9_IRQ_SOFTWARE2          (2)   /* Cortex-A55 Software Generated Interrupt 2 */
#define IMX9_IRQ_SOFTWARE3          (3)   /* Cortex-A55 Software Generated Interrupt 3 */
#define IMX9_IRQ_SOFTWARE4          (4)   /* Cortex-A55 Software Generated Interrupt 4 */
#define IMX9_IRQ_SOFTWARE5          (5)   /* Cortex-A55 Software Generated Interrupt 5 */
#define IMX9_IRQ_SOFTWARE6          (6)   /* Cortex-A55 Software Generated Interrupt 6 */
#define IMX9_IRQ_SOFTWARE7          (7)   /* Cortex-A55 Software Generated Interrupt 7 */
#define IMX9_IRQ_SOFTWARE8          (8)   /* Cortex-A55 Software Generated Interrupt 8 */
#define IMX9_IRQ_SOFTWARE9          (9)   /* Cortex-A55 Software Generated Interrupt 9 */
#define IMX9_IRQ_SOFTWARE10         (10)  /* Cortex-A55 Software Generated Interrupt 10 */
#define IMX9_IRQ_SOFTWARE11         (11)  /* Cortex-A55 Software Generated Interrupt 11 */
#define IMX9_IRQ_SOFTWARE12         (12)  /* Cortex-A55 Software Generated Interrupt 12 */
#define IMX9_IRQ_SOFTWARE13         (13)  /* Cortex-A55 Software Generated Interrupt 13 */
#define IMX9_IRQ_SOFTWARE14         (14)  /* Cortex-A55 Software Generated Interrupt 14 */
#define IMX9_IRQ_SOFTWARE15         (15)  /* Cortex-A55 Software Generated Interrupt 15 */
#define IMX9_IRQ_VIRTUALMAINTENANCE (25)  /* Cortex-A55 Virtual Maintenance Interrupt */
#define IMX9_IRQ_HYPERVISORTIMER    (26)  /* Cortex-A55 Hypervisor Timer Interrupt */
#define IMX9_IRQ_VIRTUALTIMER       (27)  /* Cortex-A55 Virtual Timer Interrupt */
#define IMX9_IRQ_LEGACYFASTINT      (28)  /* Cortex-A55 Legacy nFIQ signal Interrupt */
#define IMX9_IRQ_SECUREPHYTIMER     (29)  /* Cortex-A55 Secure Physical Timer Interrupt */
#define IMX9_IRQ_NONSECUREPHYTIMER  (30)  /* Cortex-A55 Non-secure Physical Timer Interrupt */
#define IMX9_IRQ_LEGACYIRQ          (31)  /* Cortex-A55 Legacy nIRQ Interrupt */

#define IMX9_IRQ_EXT                (32)  /* Vector number of the first ext int */

#endif /* __ARCH_ARM64_INCLUDE_IMX9_IRQ_H */
