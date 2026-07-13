/****************************************************************************
 * arch/mips/include/jz4780/irq_jz4780.h
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

#ifndef __ARCH_MIPS_INCLUDE_JZ4780_IRQ_JZ4780_H
#define __ARCH_MIPS_INCLUDE_JZ4780_IRQ_JZ4780_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Interrupt vector numbers.  These should be used to attach to interrupts
 * and to change interrupt priorities.
 */

#define IRQ_OFFSET1             32
#define IRQ_VIRTUAL             32

/*  Interrupts from Source 0 */

#define JZ4780_IRQ_LCD          31
#define JZ4780_IRQ_CIM          30
#define JZ4780_IRQ_IPU          29
#define JZ4780_IRQ_GPS          28
#define JZ4780_IRQ_TCU0         27
#define JZ4780_IRQ_TCU1         26
#define JZ4780_IRQ_TCU2         25
#define JZ4780_IRQ_GPS_1MS      24
#define JZ4780_IRQ_LCD1         23
#define JZ4780_IRQ_IPU1         22
#define JZ4780_IRQ_OTG          21
#define JZ4780_IRQ_EHCI         20
#define JZ4780_IRQ_X2D          19
#define JZ4780_IRQ_SADC         18
#define JZ4780_IRQ_GPIOA        17
#define JZ4780_IRQ_GPIOB        16
#define JZ4780_IRQ_GPIOC        15
#define JZ4780_IRQ_GPIOD        14
#define JZ4780_IRQ_GPIOE        13
#define JZ4780_IRQ_GPIOF        12
#define JZ4780_IRQ_TSSI1        11
#define JZ4780_IRQ_PDMA         10
#define JZ4780_IRQ_TSSI0        9
#define JZ4780_IRQ_SSI0         8
#define JZ4780_IRQ_SSI1         7
#define JZ4780_IRQ_RSRV0        6
#define JZ4780_IRQ_OHCI         5
#define JZ4780_IRQ_HDMI_WKUP    4
#define JZ4780_IRQ_HDMI         3
#define JZ4780_IRQ_BCH          2
#define JZ4780_IRQ_AIC0         1
#define JZ4780_IRQ_AIC1         0

/* Interrupts from Source 1 */

#define JZ4780_IRQ_GPU          (31 + IRQ_OFFSET1)
#define JZ4780_IRQ_VPU          (30 + IRQ_OFFSET1)
#define JZ4780_IRQ_PDMAM        (29 + IRQ_OFFSET1)
#define JZ4780_IRQ_SMB0         (28 + IRQ_OFFSET1)
#define JZ4780_IRQ_SMB1         (27 + IRQ_OFFSET1)
#define JZ4780_IRQ_SMB2         (26 + IRQ_OFFSET1)
#define JZ4780_IRQ_SMB3         (25 + IRQ_OFFSET1)
#define JZ4780_IRQ_SMB4         (24 + IRQ_OFFSET1)
#define JZ4780_IRQ_ETHC         (23 + IRQ_OFFSET1)
#define JZ4780_IRQ_NEMC         (22 + IRQ_OFFSET1)
#define JZ4780_IRQ_RSRVD1       (21 + IRQ_OFFSET1)
#define JZ4780_IRQ_DDR          (20 + IRQ_OFFSET1)
#define JZ4780_IRQ_UART0        (19 + IRQ_OFFSET1)
#define JZ4780_IRQ_UART1        (18 + IRQ_OFFSET1)
#define JZ4780_IRQ_UART2        (17 + IRQ_OFFSET1)
#define JZ4780_IRQ_UART3        (16 + IRQ_OFFSET1)
#define JZ4780_IRQ_CPM          (15 + IRQ_OFFSET1)
#define JZ4780_IRQ_HARB0        (14 + IRQ_OFFSET1)
#define JZ4780_IRQ_RSRVD2       (13 + IRQ_OFFSET1)
#define JZ4780_IRQ_HARB2        (12 + IRQ_OFFSET1)
#define JZ4780_IRQ_COMPRESS     (11 + IRQ_OFFSET1)
#define JZ4780_IRQ_GPVLC        (10 + IRQ_OFFSET1)
#define JZ4780_IRQ_KBC          ( 9 + IRQ_OFFSET1)
#define JZ4780_IRQ_PCM0         ( 8 + IRQ_OFFSET1)
#define JZ4780_IRQ_RSVD3        ( 7 + IRQ_OFFSET1)
#define JZ4780_IRQ_SCC          ( 6 + IRQ_OFFSET1)
#define JZ4780_IRQ_MSC0         ( 5 + IRQ_OFFSET1)
#define JZ4780_IRQ_MSC1         ( 4 + IRQ_OFFSET1)
#define JZ4780_IRQ_MSC2         ( 3 + IRQ_OFFSET1)
#define JZ4780_IRQ_UART4        ( 2 + IRQ_OFFSET1)
#define JZ4780_IRQ_OWI          ( 1 + IRQ_OFFSET1)
#define JZ4780_IRQ_RTC          ( 0 + IRQ_OFFSET1)

/* Virtual interrupts */

#define IRQ_SWINT0              ( 8 + IRQ_OFFSET1 + IRQ_VIRTUAL)
#define IRQ_TMR7                ( 7 + IRQ_OFFSET1 + IRQ_VIRTUAL)
#define IRQ_TMR6                ( 6 + IRQ_OFFSET1 + IRQ_VIRTUAL)
#define IRQ_TMR4                ( 4 + IRQ_OFFSET1 + IRQ_VIRTUAL)
#define IRQ_TMR3                ( 3 + IRQ_OFFSET1 + IRQ_VIRTUAL)
#define IRQ_TMR1                ( 1 + IRQ_OFFSET1 + IRQ_VIRTUAL)
#define IRQ_TMR0                ( 0 + IRQ_OFFSET1 + IRQ_VIRTUAL)

#define NR_IRQS                 (IRQ_OFFSET1 + IRQ_VIRTUAL + 32)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_INCLUDE_JZ4780_IRQ_JZ4780_H */

