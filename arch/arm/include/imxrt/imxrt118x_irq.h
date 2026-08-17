/****************************************************************************
 * arch/arm/include/imxrt/imxrt118x_irq.h
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

#ifndef __ARCH_ARM_INCLUDE_IMXRT_IMXRT118X_IRQ_H
#define __ARCH_ARM_INCLUDE_IMXRT_IMXRT118X_IRQ_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MIMXRT1186 Cortex-M33 external interrupt numbering follows the NXP CMSIS
 * device header.  Add 16 to each CMSIS IRQn to obtain a NuttX vector number.
 */

#define IMXRT_IRQ_TMR1          (IMXRT_IRQ_EXTINT + 0)
#define IMXRT_IRQ_GPIO1_0       (IMXRT_IRQ_EXTINT + 10)
#define IMXRT_IRQ_GPIO1_1       (IMXRT_IRQ_EXTINT + 11)
#define IMXRT_IRQ_LPIT1         (IMXRT_IRQ_EXTINT + 15)
#define IMXRT_IRQ_LPUART1       (IMXRT_IRQ_EXTINT + 19)
#define IMXRT_IRQ_LPUART2       (IMXRT_IRQ_EXTINT + 20)
#define IMXRT_IRQ_GPT1          (IMXRT_IRQ_EXTINT + 209)
#define IMXRT_IRQ_GPT2          (IMXRT_IRQ_EXTINT + 210)
#define IMXRT_IRQ_USBPHY1       (IMXRT_IRQ_EXTINT + 212)
#define IMXRT_IRQ_USBOTG1       (IMXRT_IRQ_EXTINT + 215)

#define IMXRT_IRQ_NEXTINT       (IMXRT_IRQ_EXTINT + 239)
#define NR_IRQS                 IMXRT_IRQ_NEXTINT

#endif /* __ARCH_ARM_INCLUDE_IMXRT_IMXRT118X_IRQ_H */
