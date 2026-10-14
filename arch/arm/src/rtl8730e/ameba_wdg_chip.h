/****************************************************************************
 * arch/arm/src/rtl8730e/ameba_wdg_chip.h
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

#ifndef __ARCH_ARM_SRC_RTL8730E_AMEBA_WDG_CHIP_H
#define __ARCH_ARM_SRC_RTL8730E_AMEBA_WDG_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Per-chip watchdog wiring for RTL8730E (amebasmart, CA32).  The shared
 * driver (arch/arm/src/common/ameba/ameba_wdg.c) includes this header to
 * learn which watchdog instance to drive and its interrupt line.  The
 * shared driver is never edited -- it reads only the two macros below.
 *
 * amebasmart carries five watchdog instances: an always-on IWDG (WDG0) plus
 * one "system" WDG per core/domain, each at its own base and with its own
 * secure/non-secure alias split:
 *
 *   WDG0 (IWDG)  0x42008400  always-on, LP domain
 *   WDG1         0x51000400
 *   WDG2         0x41000440  KM4 non-secure system watchdog
 *   WDG3         0x51000480
 *   WDG4         0x410004c0  CA32 (AP) non-secure system watchdog
 *
 * This port runs on the CA32 core, and -- matching the vendor HAL
 * wdt_api.c, which selects WDG4_DEV / WDG4_IRQ for CONFIG_ARM_CORE_CA32
 * (KM4 gets WDG2 instead) -- we drive WDG4, the CA32 non-secure system
 * watchdog.  As with the other CA32 drivers the non-secure register alias
 * (0x41xxxxxx) is the one reachable from this core.
 *
 * BOTH the base address and the interrupt vector are chip-specific.  The WDG
 * register block, structures, magic keys and the ms-based timeout are the
 * same on every current Ameba chip, so they live in the shared driver; only
 * the two values below move per chip (verified against the SoC
 * hal_platform.h and the CA32 section of ameba_vector_table.h, not guessed).
 *
 * NuttX IRQ = GIC SPI number + 32 on this chip (see irq.h); WDG4_IRQ is
 * GIC SPI 62 in the CA32 enum, giving NuttX IRQ 94.
 */

#define AMEBA_WDG_BASE            0x410004c0ul
#define AMEBA_WDG_IRQ             RTL8730E_IRQ_WDG4

#endif /* __ARCH_ARM_SRC_RTL8730E_AMEBA_WDG_CHIP_H */
