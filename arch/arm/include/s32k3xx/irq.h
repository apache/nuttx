/****************************************************************************
 * arch/arm/include/s32k3xx/irq.h
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

/* This file should never be included directly but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_S32K3XX_IRQ_H
#define __ARCH_ARM_INCLUDE_S32K3XX_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IRQ numbers.
 * The IRQ number corresponds to the vector number and hence maps directly to
 * bits in the NVIC.  This does, however, waste several words of memory in
 * the IRQ to handle mapping tables.
 */

/* Processor Exceptions (vectors 0-15) */

#define S32K3XX_IRQ_RESERVED    (0) /* Reserved vector (only used with CONFIG_DEBUG_FEATURES) */
                                    /* Vector  0: Initial Stack Pointer */
                                    /* Vector  1: Initial Program Counter (not handled as an IRQ) */
#define S32K3XX_IRQ_NMI         (2) /* Vector  2: Non-Maskable Interrupt */
#define S32K3XX_IRQ_HARDFAULT   (3) /* Vector  3: Hard Fault */
#define S32K3XX_IRQ_MEMFAULT    (4) /* Vector  4: Memory Management Fault */
#define S32K3XX_IRQ_BUSFAULT    (5) /* Vector  5: Bus Fault */
#define S32K3XX_IRQ_USAGEFAULT  (6) /* Vector  6: Usage Fault */
                                    /* Vector  7: Reserved */
                                    /* Vector  8: Reserved */
                                    /* Vector  9: Reserved */
                                    /* Vector 10: Reserved */
#define S32K3XX_IRQ_SVCALL     (11) /* Vector 11: Supervisor Call */
#define S32K3XX_IRQ_DBGMONITOR (12) /* Vector 12: Debug Monitor */
                                    /* Vector 13: Reserved */
#define S32K3XX_IRQ_PENDSV     (14) /* Vector 14: Pendable System Service Request */
#define S32K3XX_IRQ_SYSTICK    (15) /* Vector 15: System Tick */

/* External interrupts (vectors >= 16).
 * These definitions are chip-specific.
 */

#define S32K3XX_IRQ_EXTINT     (16) /* Vector 16: Vector number of the first external interrupt */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#if defined(CONFIG_ARCH_CHIP_S32K344)
#  include <arch/chip/s32k3x4_irq.h>
#else
#  error Unrecognized S32K3XX part
#endif

#endif /* __ARCH_ARM_INCLUDE_S32K3XX_IRQ_H */
