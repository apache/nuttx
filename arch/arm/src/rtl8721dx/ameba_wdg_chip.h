/****************************************************************************
 * arch/arm/src/rtl8721dx/ameba_wdg_chip.h
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

#ifndef __ARCH_ARM_SRC_RTL8721DX_AMEBA_WDG_CHIP_H
#define __ARCH_ARM_SRC_RTL8721DX_AMEBA_WDG_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Per-chip watchdog wiring for RTL8721DX (amebadplus).  The shared driver
 * (arch/arm/src/common/ameba/ameba_wdg.c) includes this header to learn
 * which watchdog instance to drive and its interrupt line.  A port to
 * another Ameba chip supplies a same-named header on the chip include path;
 * the shared driver is never edited -- it reads only the two macros below.
 *
 * The Ameba SoCs carry several watchdog instances (an always-on IWDG plus
 * one "system" WDG per CPU, each with a secure and a non-secure alias).
 * This NuttX port runs on the KM4 core, and -- matching the vendor HAL
 * wdt_api.c, which selects KM4_NS_WDG_DEV / KM4_NS_WDG_IRQ for
 * CONFIG_ARM_CORE_CM4 -- we drive the KM4 non-secure system watchdog.  As
 * with the I2C/SPI drivers the non-secure register alias (0x41xxxxxx) is the
 * one actually reachable from this core, so that is the base used here.
 *
 * BOTH the base address and the interrupt vector are chip-specific.  The WDG
 * register block, structures, magic keys and the ms-based timeout are the
 * same on every current Ameba chip, so they live in the shared driver; only
 * the two values below move per chip (verified against each SoC's
 * hal_platform.h and ameba_vector_table.h):
 *
 *   chip         KM4/CPU non-secure system WDG base   NuttX IRQ
 *   -----------  -----------------------------------  ---------------------
 *   amebadplus   WDG2_REG_BASE            0x41008D80  KM4_NS_WDG_IRQ    = 65
 *   amebalite    WDG2_REG_BASE            0x4101F040  (see its vector table)
 *   amebasmart   WDG2_REG_BASE            0x41000440  (see its vector table)
 *   amebagreen2  WDG2_REG_BASE            0x4080AD80  CPU0_NS_WDG_IRQ   = 69
 *   RTL8720F     WDG2_REG_BASE            0x40801D80  KM4NS_WDG_IRQ     = 49
 *
 * A new chip only edits this header: AMEBA_WDG_BASE is the register base of
 * its non-secure system WDG (cast to the fwlib WDG_TypeDef * by the shared
 * driver) and AMEBA_WDG_IRQ carries the NuttX IRQ number of that WDG's
 * early-interrupt line -- see the ADC/RTC chip headers for the same
 * data-driven, never-computed pattern.
 */

#define AMEBA_WDG_BASE            0x41008d80ul
#define AMEBA_WDG_IRQ             RTL8721DX_IRQ_KM4_NS_WDG

#endif /* __ARCH_ARM_SRC_RTL8721DX_AMEBA_WDG_CHIP_H */
