/****************************************************************************
 * arch/arm/src/rtl8720f/ameba_gpio_chip.h
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

#ifndef __ARCH_ARM_SRC_RTL8720F_AMEBA_GPIO_CHIP_H
#define __ARCH_ARM_SRC_RTL8720F_AMEBA_GPIO_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Per-chip GPIO parameters for the shared driver
 * (arch/arm/src/common/ameba/ameba_gpio.c).  The driver logic, pin encoding
 * and fwlib API are identical across every Ameba ARM chip, but the port
 * count, the per-port NVIC vectors and the RCC gate bits are not.  Each chip
 * supplies its own <ameba_gpio_chip.h> on the include path (arch/.../chip);
 * the common driver sizes its tables and wires its vectors from the macros
 * below, so nothing IC-specific is left in common.
 *
 * RTL8720F (Ameba WHC, KM4) drives all of its GPIO through a single 32-pin
 * controller (GPIO_PORTx[1] / GPIO_PORT_A) served by one NVIC vector
 * (RTL8720F_IRQ_GPIOA).  This is unlike RTL8721Dx (ports A and B, two
 * vectors) or RTL8721F (ports A, B and C, three vectors): the SDK fwlib on
 * this part exposes only GPIO_PORT_A (IS_GPIO_PORT_NUM() accepts port A
 * only), so the shared driver is configured for a single port here.  Board
 * pin tables therefore use port A (AMEBA_PA()) pins.
 *
 * GPIO_INTStatusGet and GPIO_INTStatusClearEdge are in the RTL8720F ROM
 * symbol table (ameba_rom_symbol_acut_s.ld), so fwlib
 * ram_common/ameba_gpio.c does NOT need to be compiled in -- unlike
 * RTL8721Dx where those symbols live in ram_common.
 */

/* Number of GPIO ports (banks) this chip exposes. */

#define AMEBA_GPIO_NPORTS      1

/* NVIC vector for each port, as an initialiser indexed by port number
 * (0 = A).  Its width must match AMEBA_GPIO_NPORTS.
 */

#define AMEBA_GPIO_PORT_IRQS   { RTL8720F_IRQ_GPIOA }

/* APBPeriph_GPIO / APBPeriph_GPIO_CLOCK (sysreg_lsys.h): the peripheral and
 * clock bits RCC_PeriphClockCmd() gates for the GPIO block.  On RTL8720F
 * both are ((1 << 30) | (1 << 4)); same value as RTL8721Dx and RTL8721F.
 */

#define AMEBA_APBPERIPH_GPIO   (((uint32_t)1 << 30) | ((uint32_t)1 << 4))

#endif /* __ARCH_ARM_SRC_RTL8720F_AMEBA_GPIO_CHIP_H */
