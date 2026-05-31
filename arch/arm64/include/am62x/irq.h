/****************************************************************************
 * arch/arm64/include/am62x/irq.h
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

/* Reference: AM62x TRM (SPRSP43), Chapter 9 — Interrupt Architecture
 *
 * The AM62x GIC-600 supports:
 *   - 16  SGIs  (Software Generated Interrupts)   IRQ  0 – 15
 *   - 16  PPIs  (Private Peripheral Interrupts)   IRQ 16 – 31
 *   - 480 SPIs  (Shared Peripheral Interrupts)    IRQ 32 – 511
 *
 * Total: 512 interrupt lines.
 *
 * All INTIDs listed here are INTID = GIC_SPI_number + 32.
 * E.g. UART0 GIC_SPI 178 -> INTID 210.
 */

#ifndef __ARCH_ARM64_INCLUDE_AM62X_IRQ_H
#define __ARCH_ARM64_INCLUDE_AM62X_IRQ_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Total number of interrupt IDs allocated in the GIC INTID space.
 * NuttX arrays are sized to NR_IRQS entries, so set this to the maximum
 * INTID we actually use + 1.  512 covers all AM62x SPIs.
 */

#define NR_IRQS                   512

/* SGI interrupts (per-core, software-generated) */

#define AM62X_IRQ_SGI0            0
#define AM62X_IRQ_SGI1            1
#define AM62X_IRQ_SGI2            2
#define AM62X_IRQ_SGI3            3
#define AM62X_IRQ_SGI4            4
#define AM62X_IRQ_SGI5            5
#define AM62X_IRQ_SGI6            6
#define AM62X_IRQ_SGI7            7
#define AM62X_IRQ_SGI8            8
#define AM62X_IRQ_SGI9            9
#define AM62X_IRQ_SGI10           10
#define AM62X_IRQ_SGI11           11
#define AM62X_IRQ_SGI12           12
#define AM62X_IRQ_SGI13           13
#define AM62X_IRQ_SGI14           14
#define AM62X_IRQ_SGI15           15

/* PPI interrupts (per-core, private peripherals)
 *
 * Generic timer PPIs follow the architected GIC assignments:
 *   - Hypervisor timer          PPI 10 -> INTID 26
 *   - Virtual timer             PPI 11 -> INTID 27
 *   - Secure physical timer     PPI 13 -> INTID 29
 *   - Non-secure physical timer PPI 14 -> INTID 30
 */

#define AM62X_IRQ_PPI_HYPTIMER    26   /* Hypervisor timer             */
#define AM62X_IRQ_PPI_VTIMER      27   /* Virtual timer (EL1)          */
#define AM62X_IRQ_PPI_PTIMER      30   /* Physical timer NS EL1/EL0    */
#define AM62X_IRQ_PPI_PTIMER_S    29   /* Physical timer secure EL3    */
#define AM62X_IRQ_PPI_PTIMER_NS   30   /* Physical timer NS EL1/EL0    */

/* Main domain SPI interrupts (INTID = SPI_offset + 32).
 * Source: AM62x TRM Table 9-17 (Interrupt Map).
 */

/* UART (main domain, 16550-compatible)
 * Source: Linux arch/arm64/boot/dts/ti/k3-am62.dtsi
 *   UART0 GIC_SPI 178 -> INTID 210, UART6 GIC_SPI 184 -> INTID 216
 */

#define AM62X_IRQ_UART0           210   /* GIC_SPI 178: UART0 main domain */
#define AM62X_IRQ_UART1           211   /* GIC_SPI 179                    */
#define AM62X_IRQ_UART2           212   /* GIC_SPI 180                    */
#define AM62X_IRQ_UART3           213   /* GIC_SPI 181                    */
#define AM62X_IRQ_UART4           214   /* GIC_SPI 182                    */
#define AM62X_IRQ_UART5           215   /* GIC_SPI 183                    */
#define AM62X_IRQ_UART6           216   /* GIC_SPI 184                    */

/* GPIO */

#define AM62X_IRQ_GPIO0            57
#define AM62X_IRQ_GPIO1            58

/* I2C */

#define AM62X_IRQ_I2C0            193   /* GIC_SPI 161 */
#define AM62X_IRQ_I2C1            194   /* GIC_SPI 162 */
#define AM62X_IRQ_I2C2            195   /* GIC_SPI 163 */
#define AM62X_IRQ_I2C3            196   /* GIC_SPI 164 */

#endif /* __ARCH_ARM64_INCLUDE_AM62X_IRQ_H */
