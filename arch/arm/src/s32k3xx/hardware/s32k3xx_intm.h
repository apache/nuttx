/****************************************************************************
 * arch/arm/src/s32k3xx/hardware/s32k3xx_intm.h
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

/* Copyright 2022 NXP */

#ifndef __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_INTM_H
#define __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_INTM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/s32k3xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* INTM Register Offsets ****************************************************/

#define S32K3XX_INTM_MM_OFFSET       (0x00) /* Monitor Mode Register (INTM_MM) */
#define S32K3XX_INTM_IACK_OFFSET     (0x04) /* Interrupt Acknowledge Register (ITNM_IACK) */
#define S32K3XX_INTM_IRQSEL0_OFFSET  (0x08) /* Interrupt Request Select 0 Register (INTM_IRQSEL0) */
#define S32K3XX_INTM_LATENCY0_OFFSET (0x0c) /* INTM_LATENCY0 Register */
#define S32K3XX_INTM_TIMER0_OFFSET   (0x10) /* Timer 0 Register (INTM_TIMER0) */
#define S32K3XX_INTM_STATUS0_OFFSET  (0x14) /* Status 0 Register (INTM_STATUS0) */
#define S32K3XX_INTM_IRQSEL1_OFFSET  (0x18) /* Interrupt Request Select 1 Register (INTM_IRQSEL1) */
#define S32K3XX_INTM_LATENCY1_OFFSET (0x1c) /* INTM_LATENCY1 Register */
#define S32K3XX_INTM_TIMER1_OFFSET   (0x20) /* Timer 1 Register (INTM_TIMER1) */
#define S32K3XX_INTM_STATUS1_OFFSET  (0x24) /* Status 1 Register (INTM_STATUS1) */
#define S32K3XX_INTM_IRQSEL2_OFFSET  (0x28) /* Interrupt Request Select 2 Register (INTM_IRQSEL2) */
#define S32K3XX_INTM_LATENCY2_OFFSET (0x2c) /* INTM_LATENCY2 Register */
#define S32K3XX_INTM_TIMER2_OFFSET   (0x30) /* Timer 2 Register (INTM_TIMER2) */
#define S32K3XX_INTM_STATUS2_OFFSET  (0x34) /* Status 2 Register (INTM_STATUS2) */
#define S32K3XX_INTM_IRQSEL3_OFFSET  (0x38) /* Interrupt Request Select 3 Register (INTM_IRQSEL3) */
#define S32K3XX_INTM_LATENCY3_OFFSET (0x3c) /* INTM_LATENCY3 Register */
#define S32K3XX_INTM_TIMER3_OFFSET   (0x40) /* Timer 3 Register (INTM_TIMER3) */
#define S32K3XX_INTM_STATUS3_OFFSET  (0x44) /* Status 3 Register (INTM_STATUS3) */

/* INTM Register Addresses **************************************************/

#define S32K3XX_INTM_MM              (S32K3XX_INTM_BASE + S32K3XX_INTM_MM_OFFSET)
#define S32K3XX_INTM_IACK            (S32K3XX_INTM_BASE + S32K3XX_INTM_IACK_OFFSET)
#define S32K3XX_INTM_IRQSEL0         (S32K3XX_INTM_BASE + S32K3XX_INTM_IRQSEL0_OFFSET)
#define S32K3XX_INTM_LATENCY0        (S32K3XX_INTM_BASE + S32K3XX_INTM_LATENCY0_OFFSET)
#define S32K3XX_INTM_TIMER0          (S32K3XX_INTM_BASE + S32K3XX_INTM_TIMER0_OFFSET)
#define S32K3XX_INTM_STATUS0         (S32K3XX_INTM_BASE + S32K3XX_INTM_STATUS0_OFFSET)
#define S32K3XX_INTM_IRQSEL1         (S32K3XX_INTM_BASE + S32K3XX_INTM_IRQSEL1_OFFSET)
#define S32K3XX_INTM_LATENCY1        (S32K3XX_INTM_BASE + S32K3XX_INTM_LATENCY1_OFFSET)
#define S32K3XX_INTM_TIMER1          (S32K3XX_INTM_BASE + S32K3XX_INTM_TIMER1_OFFSET)
#define S32K3XX_INTM_STATUS1         (S32K3XX_INTM_BASE + S32K3XX_INTM_STATUS1_OFFSET)
#define S32K3XX_INTM_IRQSEL2         (S32K3XX_INTM_BASE + S32K3XX_INTM_IRQSEL2_OFFSET)
#define S32K3XX_INTM_LATENCY2        (S32K3XX_INTM_BASE + S32K3XX_INTM_LATENCY2_OFFSET)
#define S32K3XX_INTM_TIMER2          (S32K3XX_INTM_BASE + S32K3XX_INTM_TIMER2_OFFSET)
#define S32K3XX_INTM_STATUS2         (S32K3XX_INTM_BASE + S32K3XX_INTM_STATUS2_OFFSET)
#define S32K3XX_INTM_IRQSEL3         (S32K3XX_INTM_BASE + S32K3XX_INTM_IRQSEL3_OFFSET)
#define S32K3XX_INTM_LATENCY3        (S32K3XX_INTM_BASE + S32K3XX_INTM_LATENCY3_OFFSET)
#define S32K3XX_INTM_TIMER3          (S32K3XX_INTM_BASE + S32K3XX_INTM_TIMER3_OFFSET)
#define S32K3XX_INTM_STATUS3         (S32K3XX_INTM_BASE + S32K3XX_INTM_STATUS3_OFFSET)

/* INTM Register Bitfield Definitions ***************************************/

/* Monitor Mode Register (INTM_MM) */

#define INTM_MM                      (1 << 0) /* Bit 0: Monitor Mode (MM) */
                                              /* Bits 1-31: Reserved */

/* Interrupt Acknowledge Register (ITNM_IACK) */

#define INTM_IACK_IRQ_SHIFT          (0)      /* Bits 0-9: Interrupt Request Number to stop INTM_TIMERn (IRQ)*/
#define INTM_IACK_IRQ_MASK           (0x03ff << INTM_IACK_IRQ_SHIFT)
                                              /* Bits 10-31: Reserved */

/* Interrupt Request Select n Register (INTM_IRQSELn) */

#define INTM_IRQSEL_IRQ_SHIFT        (0)      /* Bits 0-9: Interrupt Request Number to Monitor (IRQ) */
#define INTM_IRQSEL_IRQ_MASK         (0x03ff << INTM_IRQSEL_IRQ_SHIFT)
                                              /* Bits 10-31: Reserved */

/* INTM_LATENCYn Register */

#define INTM_LATENCY_LAT_SHIFT       (0)      /* Bits 0-23: Maximum number of INTM clock cycles allowed for the monitored interrupt request (LAT) */
#define INTM_LATENCY_LAT_MASK        (0xffffff << INTM_LATENCY_LAT_SHIFT)
#  define INTM_LATENCY_LAT_MAX       (0xfffffd << INTM_LATENCY_LAT_SHIFT) /* Maximum allowed latency, see reference manual */

                                              /* Bits 24-31: Reserved */

/* Timer n Register (INTM_TIMERn) */

#define INTM_TIMER_SHIFT             (0)      /* Bits 0-23: Count the number of INTM clock cycles (TIMER) */
#define INTM_TIMER_MASK              (0xffffff << INTM_TIMER_SHIFT)
                                              /* Bits 24-31: Reserved */

/* Status n Register (INTM_STATUSn) */

#define INTM_STATUS                  (1 << 0) /* Bit 0: Monitor status (STATUS) */
                                              /* Bits 1-31: Reserved */

#endif /* __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_INTM_H */
