/****************************************************************************
 * arch/risc-v/include/bl808/irq.h
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

#ifndef __ARCH_RISCV_INCLUDE_BL808_IRQ_H
#define __ARCH_RISCV_INCLUDE_BL808_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Map RISC-V exception code to NuttX IRQ */

#define BL808_IRQ_NUM_BASE (16)

/* IRQs tied to M0 core are given a virtual IRQ number.
 * Offset of 67 chosen to avoid overlap with highest D0
 * IRQ, the PDS interrupt.
 */

#define BL808_M0_IRQ_OFFSET (67)

#define BL808_D0_MAX_EXTIRQ (BL808_IRQ_NUM_BASE + 66)
#define BL808_M0_MAX_EXTIRQ (BL808_IRQ_NUM_BASE + 63)

/* NR_IRQs corresponds to highest possible
 * interrupt number, WIFI IPC IRQ on M0.
 */

#define NR_IRQS (RISCV_IRQ_SEXT + BL808_M0_IRQ_OFFSET + BL808_M0_MAX_EXTIRQ)

/* D0 IRQs ******************************************************************/

#define BL808_IRQ_UART3 (RISCV_IRQ_SEXT + BL808_IRQ_NUM_BASE + 4)
#define BL808_IRQ_I2C2 (RISCV_IRQ_SEXT + BL808_IRQ_NUM_BASE + 5)
#define BL808_IRQ_I2C3 (RISCV_IRQ_SEXT + BL808_IRQ_NUM_BASE + 6)
#define BL808_IRQ_SPI1 (RISCV_IRQ_SEXT + BL808_IRQ_NUM_BASE + 7)
#define BL808_IRQ_D0_IPC (RISCV_IRQ_SEXT + BL808_IRQ_NUM_BASE + 38)
#define BL808_IRQ_TIMER1_CH0 (RISCV_IRQ_SEXT + BL808_IRQ_NUM_BASE + 61)
#define BL808_IRQ_TIMER1_CH1 (RISCV_IRQ_SEXT + BL808_IRQ_NUM_BASE + 62)
#define BL808_IRQ_WDT1 (RISCV_IRQ_SEXT + BL808_IRQ_NUM_BASE + 63)
#define BL808_IRQ_M0IC (RISCV_IRQ_SEXT + BL808_IRQ_NUM_BASE + 65)

/* M0 IRQs ******************************************************************/

#define BL808_IRQ_GPADC (RISCV_IRQ_SEXT + BL808_IRQ_NUM_BASE + BL808_M0_IRQ_OFFSET + 25)
#define BL808_IRQ_SPI0  (RISCV_IRQ_SEXT + BL808_IRQ_NUM_BASE + BL808_M0_IRQ_OFFSET + 27)
#define BL808_IRQ_UART0 (RISCV_IRQ_SEXT + BL808_IRQ_NUM_BASE + BL808_M0_IRQ_OFFSET + 28)
#define BL808_IRQ_UART1 (RISCV_IRQ_SEXT + BL808_IRQ_NUM_BASE + BL808_M0_IRQ_OFFSET + 29)
#define BL808_IRQ_UART2 (RISCV_IRQ_SEXT + BL808_IRQ_NUM_BASE + BL808_M0_IRQ_OFFSET + 30)
#define BL808_IRQ_I2C0 (RISCV_IRQ_SEXT + BL808_IRQ_NUM_BASE + BL808_M0_IRQ_OFFSET + 32)
#define BL808_IRQ_TIMER0_CH0 (RISCV_IRQ_SEXT + BL808_IRQ_NUM_BASE + BL808_M0_IRQ_OFFSET + 36)
#define BL808_IRQ_TIMER0_CH1 (RISCV_IRQ_SEXT + BL808_IRQ_NUM_BASE + BL808_M0_IRQ_OFFSET + 37)
#define BL808_IRQ_WDT0 (RISCV_IRQ_SEXT + BL808_IRQ_NUM_BASE + BL808_M0_IRQ_OFFSET + 38)
#define BL808_IRQ_I2C1 (RISCV_IRQ_SEXT + BL808_IRQ_NUM_BASE + BL808_M0_IRQ_OFFSET + 39)

#endif /* __ARCH_RISCV_INCLUDE_BL808_IRQ_H */
