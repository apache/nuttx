/****************************************************************************
 * boards/risc-v/litex/arty_a7/include/vexii_irq.h
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
 * Auto-generated: 2025-12-28 11:16:55
 * Source: LiteX soc.h
 *
 ****************************************************************************/

#ifndef __BOARDS_RISCV_LITEX_ARTY_A7_INCLUDE_VEXII_IRQ_H
#define __BOARDS_RISCV_LITEX_ARTY_A7_INCLUDE_VEXII_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/mode.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Custom IRQ definitions for LiteX VexiiRISCV core */

/* Map RISC-V external IRQs to NuttX IRQ numbers */

#define LITEX_IRQ_UART0                (RISCV_IRQ_EXT + 1)
#define LITEX_IRQ_TIMER0               (RISCV_IRQ_EXT + 2)
#define LITEX_IRQ_ETHMAC               (RISCV_IRQ_EXT + 3)
#define LITEX_IRQ_GPIO_BASE            (RISCV_IRQ_EXT + 4)
#define LITEX_IRQ_GPIO_LENGTH          8

/* The last hardware IRQ number */

#define LITEX_IRQ_LAST        (LITEX_IRQ_GPIO_BASE + LITEX_IRQ_GPIO_LENGTH)

/* Second level GPIO interrupts if enabled */

#ifdef CONFIG_LITEX_GPIO_IRQ
#  define LITEX_NIRQ_GPIO           (LITEX_IRQ_GPIO_LENGTH * 32)
#  define LITEX_FIRST_GPIOIRQ       (LITEX_IRQ_LAST + 1)
#  define LITEX_LAST_GPIOIRQ        (LITEX_FIRST_GPIOIRQ + LITEX_NIRQ_GPIO)
#else
#  define LITEX_NIRQ_GPIO           0
#endif

/* Total number of IRQs */

#define NR_IRQS            (LITEX_IRQ_LAST + LITEX_NIRQ_GPIO + 1)

#endif /* __BOARDS_RISCV_LITEX_ARTY_A7_INCLUDE_VEXII_IRQ_H */
