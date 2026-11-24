/****************************************************************************
 * arch/arm/include/pp502x/irq.h
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

/* This file should never be included directly but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_PP502X_IRQ_H
#define __ARCH_ARM_INCLUDE_PP502X_IRQ_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IRQ numbers 0-31 are the bits of the normal interrupt bank and 32-63 are
 * the bits of the high bank.  Bit 30 of the normal bank is the cascade of
 * the high bank and is not dispatched as an IRQ of its own.
 */

#define PP502X_IRQ_TIMER1       0   /* Timer 1 */
#define PP502X_IRQ_TIMER2       1   /* Timer 2 */
#define PP502X_IRQ_MAILBOX      4   /* Inter-core mailbox */
#define PP502X_IRQ_I2S          10  /* I2S FIFO */
#define PP502X_IRQ_USB          20  /* USB device controller */
#define PP502X_IRQ_IDE          23  /* IDE controller */
#define PP502X_IRQ_FIREWIRE     25  /* FireWire link */
#define PP502X_IRQ_DMA          26  /* DMA controller */
#define PP502X_IRQ_HI           30  /* High bank cascade */

#define PP502X_IRQ_HIBASE       32
#define PP502X_IRQ_GPIO0        (PP502X_IRQ_HIBASE + 0)  /* GPIO ports A-D */
#define PP502X_IRQ_GPIO1        (PP502X_IRQ_HIBASE + 1)  /* GPIO ports E-H */
#define PP502X_IRQ_GPIO2        (PP502X_IRQ_HIBASE + 2)  /* GPIO ports I-L */
#define PP502X_IRQ_SER0         (PP502X_IRQ_HIBASE + 4)  /* UART 0 */
#define PP502X_IRQ_SER1         (PP502X_IRQ_HIBASE + 5)  /* UART 1 */
#define PP502X_IRQ_I2C          (PP502X_IRQ_HIBASE + 8)  /* I2C */

#define PP502X_IRQ_SYSTIMER     PP502X_IRQ_TIMER1

#define NR_IRQS                 64

#endif /* __ARCH_ARM_INCLUDE_PP502X_IRQ_H */
