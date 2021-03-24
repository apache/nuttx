/****************************************************************************
 * arch/arm/src/dm320/dm320_intc.h
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

#ifndef __ARCH_ARM_SRC_DM320_DM320_INTC_H
#define __ARCH_ARM_SRC_DM320_DM320_INTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Interrupt Controller Registers */

#define DM320_INTC_FIQ0       (DM320_PERIPHERALS_VADDR + 0x0500) /* FIQ Interrupt Flag Register #0 */
#define DM320_INTC_FIQ1       (DM320_PERIPHERALS_VADDR + 0x0502) /* FIQ Interrupt Flag Register #1 */
#define DM320_INTC_FIQ2       (DM320_PERIPHERALS_VADDR + 0x0504) /* FIQ Interrupt Flag Register #2 */
#define DM320_INTC_IRQ0       (DM320_PERIPHERALS_VADDR + 0x0508) /* IRQ Interrupt Flag Register #0 */
#define DM320_INTC_IRQ1       (DM320_PERIPHERALS_VADDR + 0x050A) /* IRQ Interrupt Flag Register #1 */
#define DM320_INTC_IRQ2       (DM320_PERIPHERALS_VADDR + 0x050C) /* IRQ Interrupt Flag Register #2 */
#define DM320_INTC_FIQENTRY0  (DM320_PERIPHERALS_VADDR + 0x0510) /* FIQ Entry Address Register #0 */
#define DM320_INTC_FIQENTRY1  (DM320_PERIPHERALS_VADDR + 0x0512) /* FIQ Entry Address Register #1 */
#define DM320_INTC_FIQENTLCK0 (DM320_PERIPHERALS_VADDR + 0x0514) /* FIQ Lock Entry Address Register #1 */
#define DM320_INTC_FIQENTLCK1 (DM320_PERIPHERALS_VADDR + 0x0516) /* FIQ Lock Entry Address Register #1 */
#define DM320_INTC_IRQENTRY0  (DM320_PERIPHERALS_VADDR + 0x0518) /* IRQ Entry Address Register #0 */
#define DM320_INTC_IRQENTRY1  (DM320_PERIPHERALS_VADDR + 0x051A) /* IRQ Entry Address Register #1 */
#define DM320_INTC_IRQENTLCK0 (DM320_PERIPHERALS_VADDR + 0x051C) /* IRQ Lock Entry Address Register #1 */
#define DM320_INTC_IRQENTLCK1 (DM320_PERIPHERALS_VADDR + 0x051E) /* Lock Entry Address Register #1 */
#define DM320_INTC_FISEL0     (DM320_PERIPHERALS_VADDR + 0x0520) /* FIQ select register #0 */
#define DM320_INTC_FISEL1     (DM320_PERIPHERALS_VADDR + 0x0522) /* FIQ select register #1 */
#define DM320_INTC_FISEL2     (DM320_PERIPHERALS_VADDR + 0x0524) /* FIQ select register #2 */
#define DM320_INTC_EINT0      (DM320_PERIPHERALS_VADDR + 0x0528) /* Interrupt Enable Register #0 */
#define DM320_INTC_EINT1      (DM320_PERIPHERALS_VADDR + 0x052A) /* Interrupt Enable Register #1 */
#define DM320_INTC_EINT2      (DM320_PERIPHERALS_VADDR + 0x052C) /* Interrupt Enable Register #2 */
#define DM320_INTC_INTRAW     (DM320_PERIPHERALS_VADDR + 0x0530) /* Interrupt Raw Register */
#define DM320_INTC_EABASE0    (DM320_PERIPHERALS_VADDR + 0x0538) /* Entry Table Base Address Register #0 */
#define DM320_INTC_EABASE1    (DM320_PERIPHERALS_VADDR + 0x053A) /* Entry Table Base Address Register #1 */
#define DM320_INTC_INTPRI00   (DM320_PERIPHERALS_VADDR + 0x0540) /* Interrupt Priority Register #0 */
#define DM320_INTC_INTPRI01   (DM320_PERIPHERALS_VADDR + 0x0542) /* Interrupt Priority Register #1 */
#define DM320_INTC_INTPRI02   (DM320_PERIPHERALS_VADDR + 0x0544) /* Interrupt Priority Register #2 */
#define DM320_INTC_INTPRI03   (DM320_PERIPHERALS_VADDR + 0x0546) /* Interrupt Priority Register #3 */
#define DM320_INTC_INTPRI04   (DM320_PERIPHERALS_VADDR + 0x0548) /* Interrupt Priority Register #4 */
#define DM320_INTC_INTPRI05   (DM320_PERIPHERALS_VADDR + 0x054A) /* Interrupt Priority Register #5 */
#define DM320_INTC_INTPRI06   (DM320_PERIPHERALS_VADDR + 0x054C) /* Interrupt Priority Register #6 */
#define DM320_INTC_INTPRI07   (DM320_PERIPHERALS_VADDR + 0x054E) /* Interrupt Priority Register #7 */
#define DM320_INTC_INTPRI08   (DM320_PERIPHERALS_VADDR + 0x0550) /* Interrupt Priority Register #8 */
#define DM320_INTC_INTPRI09   (DM320_PERIPHERALS_VADDR + 0x0552) /* Interrupt Priority Register #9 */
#define DM320_INTC_INTPRI10   (DM320_PERIPHERALS_VADDR + 0x0554) /* Interrupt Priority Register #10 */
#define DM320_INTC_INTPRI11   (DM320_PERIPHERALS_VADDR + 0x0556) /* Interrupt Priority Register #11 */
#define DM320_INTC_INTPRI12   (DM320_PERIPHERALS_VADDR + 0x0558) /* Interrupt Priority Register #12 */
#define DM320_INTC_INTPRI13   (DM320_PERIPHERALS_VADDR + 0x055A) /* Interrupt Priority Register #13 */
#define DM320_INTC_INTPRI14   (DM320_PERIPHERALS_VADDR + 0x055C) /* Interrupt Priority Register #14 */
#define DM320_INTC_INTPRI15   (DM320_PERIPHERALS_VADDR + 0x055E) /* Interrupt Priority Register #15 */
#define DM320_INTC_INTPRI16   (DM320_PERIPHERALS_VADDR + 0x0560) /* Interrupt Priority Register #16 */
#define DM320_INTC_INTPRI17   (DM320_PERIPHERALS_VADDR + 0x0562) /* Interrupt Priority Register #17 */
#define DM320_INTC_INTPRI18   (DM320_PERIPHERALS_VADDR + 0x0564) /* Interrupt Priority Register #18 */
#define DM320_INTC_INTPRI19   (DM320_PERIPHERALS_VADDR + 0x0566) /* Interrupt Priority Register #19 */

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

#endif

#endif /* __ARCH_ARM_SRC_DM320_DM320_INTC_H */
