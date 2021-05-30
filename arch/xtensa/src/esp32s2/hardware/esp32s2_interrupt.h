/****************************************************************************
 * arch/xtensa/src/esp32s2/hardware/esp32s2_interrupt.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S2_HARDWARE_ESP32S2_INTERRUPT_H
#define __ARCH_XTENSA_SRC_ESP32S2_HARDWARE_ESP32S2_INTERRUPT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "esp32s2_soc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* INTERRUPT_PRO_MAC_INTR_MAP_REG register
 * MAC_INTR interrupt configuration register
 */

#define INTERRUPT_PRO_MAC_INTR_MAP_REG (DR_REG_INTERRUPT_BASE + 0x0)

/* INTERRUPT_PRO_MAC_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map MAC_INTR interrupt signal to one of the CPU
 * interrupts.
 */

#define INTERRUPT_PRO_MAC_INTR_MAP    0x0000001F
#define INTERRUPT_PRO_MAC_INTR_MAP_M  (INTERRUPT_PRO_MAC_INTR_MAP_V << INTERRUPT_PRO_MAC_INTR_MAP_S)
#define INTERRUPT_PRO_MAC_INTR_MAP_V  0x0000001F
#define INTERRUPT_PRO_MAC_INTR_MAP_S  0

/* INTERRUPT_PRO_MAC_NMI_MAP_REG register
 * MAC_NMI interrupt configuration register
 */

#define INTERRUPT_PRO_MAC_NMI_MAP_REG (DR_REG_INTERRUPT_BASE + 0x4)

/* INTERRUPT_PRO_MAC_NMI_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map MAC_NMI interrupt signal to one of the CPU
 * interrupts.
 */

#define INTERRUPT_PRO_MAC_NMI_MAP    0x0000001F
#define INTERRUPT_PRO_MAC_NMI_MAP_M  (INTERRUPT_PRO_MAC_NMI_MAP_V << INTERRUPT_PRO_MAC_NMI_MAP_S)
#define INTERRUPT_PRO_MAC_NMI_MAP_V  0x0000001F
#define INTERRUPT_PRO_MAC_NMI_MAP_S  0

/* INTERRUPT_PRO_PWR_INTR_MAP_REG register
 * PWR_INTR interrupt configuration register
 */

#define INTERRUPT_PRO_PWR_INTR_MAP_REG (DR_REG_INTERRUPT_BASE + 0x8)

/* INTERRUPT_PRO_PWR_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map PWR_INTR interrupt signal to one of the CPU
 * interrupts.
 */

#define INTERRUPT_PRO_PWR_INTR_MAP    0x0000001F
#define INTERRUPT_PRO_PWR_INTR_MAP_M  (INTERRUPT_PRO_PWR_INTR_MAP_V << INTERRUPT_PRO_PWR_INTR_MAP_S)
#define INTERRUPT_PRO_PWR_INTR_MAP_V  0x0000001F
#define INTERRUPT_PRO_PWR_INTR_MAP_S  0

/* INTERRUPT_PRO_BB_INT_MAP_REG register
 * BB_INT interrupt configuration register
 */

#define INTERRUPT_PRO_BB_INT_MAP_REG (DR_REG_INTERRUPT_BASE + 0xc)

/* INTERRUPT_PRO_BB_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map BB_INT interrupt signal to one of the CPU
 * interrupts.
 */

#define INTERRUPT_PRO_BB_INT_MAP    0x0000001F
#define INTERRUPT_PRO_BB_INT_MAP_M  (INTERRUPT_PRO_BB_INT_MAP_V << INTERRUPT_PRO_BB_INT_MAP_S)
#define INTERRUPT_PRO_BB_INT_MAP_V  0x0000001F
#define INTERRUPT_PRO_BB_INT_MAP_S  0

/* INTERRUPT_PRO_BT_MAC_INT_MAP_REG register
 * BT_MAC_INT interrupt configuration register
 */

#define INTERRUPT_PRO_BT_MAC_INT_MAP_REG (DR_REG_INTERRUPT_BASE + 0x10)

/* INTERRUPT_PRO_BT_MAC_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map BT_MAC_INT interrupt signal to one of the
 * CPU interrupts.
 */

#define INTERRUPT_PRO_BT_MAC_INT_MAP    0x0000001F
#define INTERRUPT_PRO_BT_MAC_INT_MAP_M  (INTERRUPT_PRO_BT_MAC_INT_MAP_V << INTERRUPT_PRO_BT_MAC_INT_MAP_S)
#define INTERRUPT_PRO_BT_MAC_INT_MAP_V  0x0000001F
#define INTERRUPT_PRO_BT_MAC_INT_MAP_S  0

/* INTERRUPT_PRO_BT_BB_INT_MAP_REG register
 * BT_BB_INT interrupt configuration register
 */

#define INTERRUPT_PRO_BT_BB_INT_MAP_REG (DR_REG_INTERRUPT_BASE + 0x14)

/* INTERRUPT_PRO_BT_BB_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map BT_BB_INT interrupt signal to one of the CPU
 * interrupts.
 */

#define INTERRUPT_PRO_BT_BB_INT_MAP    0x0000001F
#define INTERRUPT_PRO_BT_BB_INT_MAP_M  (INTERRUPT_PRO_BT_BB_INT_MAP_V << INTERRUPT_PRO_BT_BB_INT_MAP_S)
#define INTERRUPT_PRO_BT_BB_INT_MAP_V  0x0000001F
#define INTERRUPT_PRO_BT_BB_INT_MAP_S  0

/* INTERRUPT_PRO_BT_BB_NMI_MAP_REG register
 * BT_BB_NMI interrupt configuration register
 */

#define INTERRUPT_PRO_BT_BB_NMI_MAP_REG (DR_REG_INTERRUPT_BASE + 0x18)

/* INTERRUPT_PRO_BT_BB_NMI_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map BT_BB_NMI interrupt signal to one of the CPU
 * interrupts.
 */

#define INTERRUPT_PRO_BT_BB_NMI_MAP    0x0000001F
#define INTERRUPT_PRO_BT_BB_NMI_MAP_M  (INTERRUPT_PRO_BT_BB_NMI_MAP_V << INTERRUPT_PRO_BT_BB_NMI_MAP_S)
#define INTERRUPT_PRO_BT_BB_NMI_MAP_V  0x0000001F
#define INTERRUPT_PRO_BT_BB_NMI_MAP_S  0

/* INTERRUPT_PRO_RWBT_IRQ_MAP_REG register
 * RWBT_IRQ interrupt configuration register
 */

#define INTERRUPT_PRO_RWBT_IRQ_MAP_REG (DR_REG_INTERRUPT_BASE + 0x1c)

/* INTERRUPT_PRO_RWBT_IRQ_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map RWBT_IRQ interrupt signal to one of the CPU
 * interrupts.
 */

#define INTERRUPT_PRO_RWBT_IRQ_MAP    0x0000001F
#define INTERRUPT_PRO_RWBT_IRQ_MAP_M  (INTERRUPT_PRO_RWBT_IRQ_MAP_V << INTERRUPT_PRO_RWBT_IRQ_MAP_S)
#define INTERRUPT_PRO_RWBT_IRQ_MAP_V  0x0000001F
#define INTERRUPT_PRO_RWBT_IRQ_MAP_S  0

/* INTERRUPT_PRO_RWBLE_IRQ_MAP_REG register
 * RWBLE_IRQ interrupt configuration register
 */

#define INTERRUPT_PRO_RWBLE_IRQ_MAP_REG (DR_REG_INTERRUPT_BASE + 0x20)

/* INTERRUPT_PRO_RWBLE_IRQ_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map RWBLE_IRQ interrupt signal to one of the CPU
 * interrupts.
 */

#define INTERRUPT_PRO_RWBLE_IRQ_MAP    0x0000001F
#define INTERRUPT_PRO_RWBLE_IRQ_MAP_M  (INTERRUPT_PRO_RWBLE_IRQ_MAP_V << INTERRUPT_PRO_RWBLE_IRQ_MAP_S)
#define INTERRUPT_PRO_RWBLE_IRQ_MAP_V  0x0000001F
#define INTERRUPT_PRO_RWBLE_IRQ_MAP_S  0

/* INTERRUPT_PRO_RWBT_NMI_MAP_REG register
 * RWBT_NMI interrupt configuration register
 */

#define INTERRUPT_PRO_RWBT_NMI_MAP_REG (DR_REG_INTERRUPT_BASE + 0x24)

/* INTERRUPT_PRO_RWBT_NMI_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map RWBT_NMI interrupt signal to one of the CPU
 * interrupts.
 */

#define INTERRUPT_PRO_RWBT_NMI_MAP    0x0000001F
#define INTERRUPT_PRO_RWBT_NMI_MAP_M  (INTERRUPT_PRO_RWBT_NMI_MAP_V << INTERRUPT_PRO_RWBT_NMI_MAP_S)
#define INTERRUPT_PRO_RWBT_NMI_MAP_V  0x0000001F
#define INTERRUPT_PRO_RWBT_NMI_MAP_S  0

/* INTERRUPT_PRO_RWBLE_NMI_MAP_REG register
 * RWBLE_NMI interrupt configuration register
 */

#define INTERRUPT_PRO_RWBLE_NMI_MAP_REG (DR_REG_INTERRUPT_BASE + 0x28)

/* INTERRUPT_PRO_RWBLE_NMI_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map RWBLE_NMI interrupt signal to one of the CPU
 * interrupts.
 */

#define INTERRUPT_PRO_RWBLE_NMI_MAP    0x0000001F
#define INTERRUPT_PRO_RWBLE_NMI_MAP_M  (INTERRUPT_PRO_RWBLE_NMI_MAP_V << INTERRUPT_PRO_RWBLE_NMI_MAP_S)
#define INTERRUPT_PRO_RWBLE_NMI_MAP_V  0x0000001F
#define INTERRUPT_PRO_RWBLE_NMI_MAP_S  0

/* INTERRUPT_PRO_SLC0_INTR_MAP_REG register
 * SLC0_INTR interrupt configuration register
 */

#define INTERRUPT_PRO_SLC0_INTR_MAP_REG (DR_REG_INTERRUPT_BASE + 0x2c)

/* INTERRUPT_PRO_SLC0_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map SLC0_INTR interrupt signal to one of the CPU
 * interrupts.
 */

#define INTERRUPT_PRO_SLC0_INTR_MAP    0x0000001F
#define INTERRUPT_PRO_SLC0_INTR_MAP_M  (INTERRUPT_PRO_SLC0_INTR_MAP_V << INTERRUPT_PRO_SLC0_INTR_MAP_S)
#define INTERRUPT_PRO_SLC0_INTR_MAP_V  0x0000001F
#define INTERRUPT_PRO_SLC0_INTR_MAP_S  0

/* INTERRUPT_PRO_SLC1_INTR_MAP_REG register
 * SLC1_INTR interrupt configuration register
 */

#define INTERRUPT_PRO_SLC1_INTR_MAP_REG (DR_REG_INTERRUPT_BASE + 0x30)

/* INTERRUPT_PRO_SLC1_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map SLC1_INTR interrupt signal to one of the CPU
 * interrupts.
 */

#define INTERRUPT_PRO_SLC1_INTR_MAP    0x0000001F
#define INTERRUPT_PRO_SLC1_INTR_MAP_M  (INTERRUPT_PRO_SLC1_INTR_MAP_V << INTERRUPT_PRO_SLC1_INTR_MAP_S)
#define INTERRUPT_PRO_SLC1_INTR_MAP_V  0x0000001F
#define INTERRUPT_PRO_SLC1_INTR_MAP_S  0

/* INTERRUPT_PRO_UHCI0_INTR_MAP_REG register
 * UHCI0_INTR interrupt configuration register
 */

#define INTERRUPT_PRO_UHCI0_INTR_MAP_REG (DR_REG_INTERRUPT_BASE + 0x34)

/* INTERRUPT_PRO_UHCI0_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map UHCI0_INTR interrupt signal to one of the
 * CPU interrupts.
 */

#define INTERRUPT_PRO_UHCI0_INTR_MAP    0x0000001F
#define INTERRUPT_PRO_UHCI0_INTR_MAP_M  (INTERRUPT_PRO_UHCI0_INTR_MAP_V << INTERRUPT_PRO_UHCI0_INTR_MAP_S)
#define INTERRUPT_PRO_UHCI0_INTR_MAP_V  0x0000001F
#define INTERRUPT_PRO_UHCI0_INTR_MAP_S  0

/* INTERRUPT_PRO_UHCI1_INTR_MAP_REG register
 * UHCI1_INTR interrupt configuration register
 */

#define INTERRUPT_PRO_UHCI1_INTR_MAP_REG (DR_REG_INTERRUPT_BASE + 0x38)

/* INTERRUPT_PRO_UHCI1_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map UHCI1_INTR interrupt signal to one of the
 * CPU interrupts.
 */

#define INTERRUPT_PRO_UHCI1_INTR_MAP    0x0000001F
#define INTERRUPT_PRO_UHCI1_INTR_MAP_M  (INTERRUPT_PRO_UHCI1_INTR_MAP_V << INTERRUPT_PRO_UHCI1_INTR_MAP_S)
#define INTERRUPT_PRO_UHCI1_INTR_MAP_V  0x0000001F
#define INTERRUPT_PRO_UHCI1_INTR_MAP_S  0

/* INTERRUPT_PRO_TG_T0_LEVEL_INT_MAP_REG register
 * TG_T0_LEVEL_INT interrupt configuration register
 */

#define INTERRUPT_PRO_TG_T0_LEVEL_INT_MAP_REG (DR_REG_INTERRUPT_BASE + 0x3c)

/* INTERRUPT_PRO_TG_T0_LEVEL_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map TG_T0_LEVEL_INT interrupt signal to one of
 * the CPU interrupts.
 */

#define INTERRUPT_PRO_TG_T0_LEVEL_INT_MAP    0x0000001F
#define INTERRUPT_PRO_TG_T0_LEVEL_INT_MAP_M  (INTERRUPT_PRO_TG_T0_LEVEL_INT_MAP_V << INTERRUPT_PRO_TG_T0_LEVEL_INT_MAP_S)
#define INTERRUPT_PRO_TG_T0_LEVEL_INT_MAP_V  0x0000001F
#define INTERRUPT_PRO_TG_T0_LEVEL_INT_MAP_S  0

/* INTERRUPT_PRO_TG_T1_LEVEL_INT_MAP_REG register
 * TG_T1_LEVEL_INT interrupt configuration register
 */

#define INTERRUPT_PRO_TG_T1_LEVEL_INT_MAP_REG (DR_REG_INTERRUPT_BASE + 0x40)

/* INTERRUPT_PRO_TG_T1_LEVEL_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map TG_T1_LEVEL_INT interrupt signal to one of
 * the CPU interrupts.
 */

#define INTERRUPT_PRO_TG_T1_LEVEL_INT_MAP    0x0000001F
#define INTERRUPT_PRO_TG_T1_LEVEL_INT_MAP_M  (INTERRUPT_PRO_TG_T1_LEVEL_INT_MAP_V << INTERRUPT_PRO_TG_T1_LEVEL_INT_MAP_S)
#define INTERRUPT_PRO_TG_T1_LEVEL_INT_MAP_V  0x0000001F
#define INTERRUPT_PRO_TG_T1_LEVEL_INT_MAP_S  0

/* INTERRUPT_PRO_TG_WDT_LEVEL_INT_MAP_REG register
 * TG_WDT_LEVEL_INT interrupt configuration register
 */

#define INTERRUPT_PRO_TG_WDT_LEVEL_INT_MAP_REG (DR_REG_INTERRUPT_BASE + 0x44)

/* INTERRUPT_PRO_TG_WDT_LEVEL_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map TG_WDT_LEVEL_INT interrupt signal to one of
 * the CPU interrupts.
 */

#define INTERRUPT_PRO_TG_WDT_LEVEL_INT_MAP    0x0000001F
#define INTERRUPT_PRO_TG_WDT_LEVEL_INT_MAP_M  (INTERRUPT_PRO_TG_WDT_LEVEL_INT_MAP_V << INTERRUPT_PRO_TG_WDT_LEVEL_INT_MAP_S)
#define INTERRUPT_PRO_TG_WDT_LEVEL_INT_MAP_V  0x0000001F
#define INTERRUPT_PRO_TG_WDT_LEVEL_INT_MAP_S  0

/* INTERRUPT_PRO_TG_LACT_LEVEL_INT_MAP_REG register
 * TG_LACT_LEVEL_INT interrupt configuration register
 */

#define INTERRUPT_PRO_TG_LACT_LEVEL_INT_MAP_REG (DR_REG_INTERRUPT_BASE + 0x48)

/* INTERRUPT_PRO_TG_LACT_LEVEL_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map TG_LACT_LEVEL_INT interrupt signal to one of
 * the CPU interrupts.
 */

#define INTERRUPT_PRO_TG_LACT_LEVEL_INT_MAP    0x0000001F
#define INTERRUPT_PRO_TG_LACT_LEVEL_INT_MAP_M  (INTERRUPT_PRO_TG_LACT_LEVEL_INT_MAP_V << INTERRUPT_PRO_TG_LACT_LEVEL_INT_MAP_S)
#define INTERRUPT_PRO_TG_LACT_LEVEL_INT_MAP_V  0x0000001F
#define INTERRUPT_PRO_TG_LACT_LEVEL_INT_MAP_S  0

/* INTERRUPT_PRO_TG1_T0_LEVEL_INT_MAP_REG register
 * TG1_T0_LEVEL_INT interrupt configuration register
 */

#define INTERRUPT_PRO_TG1_T0_LEVEL_INT_MAP_REG (DR_REG_INTERRUPT_BASE + 0x4c)

/* INTERRUPT_PRO_TG1_T0_LEVEL_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map TG1_T0_LEVEL_INT interrupt signal to one of
 * the CPU interrupts.
 */

#define INTERRUPT_PRO_TG1_T0_LEVEL_INT_MAP    0x0000001F
#define INTERRUPT_PRO_TG1_T0_LEVEL_INT_MAP_M  (INTERRUPT_PRO_TG1_T0_LEVEL_INT_MAP_V << INTERRUPT_PRO_TG1_T0_LEVEL_INT_MAP_S)
#define INTERRUPT_PRO_TG1_T0_LEVEL_INT_MAP_V  0x0000001F
#define INTERRUPT_PRO_TG1_T0_LEVEL_INT_MAP_S  0

/* INTERRUPT_PRO_TG1_T1_LEVEL_INT_MAP_REG register
 * TG1_T1_LEVEL_INT interrupt configuration register
 */

#define INTERRUPT_PRO_TG1_T1_LEVEL_INT_MAP_REG (DR_REG_INTERRUPT_BASE + 0x50)

/* INTERRUPT_PRO_TG1_T1_LEVEL_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map TG1_T1_LEVEL_INT interrupt signal to one of
 * the CPU interrupts.
 */

#define INTERRUPT_PRO_TG1_T1_LEVEL_INT_MAP    0x0000001F
#define INTERRUPT_PRO_TG1_T1_LEVEL_INT_MAP_M  (INTERRUPT_PRO_TG1_T1_LEVEL_INT_MAP_V << INTERRUPT_PRO_TG1_T1_LEVEL_INT_MAP_S)
#define INTERRUPT_PRO_TG1_T1_LEVEL_INT_MAP_V  0x0000001F
#define INTERRUPT_PRO_TG1_T1_LEVEL_INT_MAP_S  0

/* INTERRUPT_PRO_TG1_WDT_LEVEL_INT_MAP_REG register
 * TG1_WDT_LEVEL_INT interrupt configuration register
 */

#define INTERRUPT_PRO_TG1_WDT_LEVEL_INT_MAP_REG (DR_REG_INTERRUPT_BASE + 0x54)

/* INTERRUPT_PRO_TG1_WDT_LEVEL_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map TG1_WDT_LEVEL_INT interrupt signal to one of
 * the CPU interrupts.
 */

#define INTERRUPT_PRO_TG1_WDT_LEVEL_INT_MAP    0x0000001F
#define INTERRUPT_PRO_TG1_WDT_LEVEL_INT_MAP_M  (INTERRUPT_PRO_TG1_WDT_LEVEL_INT_MAP_V << INTERRUPT_PRO_TG1_WDT_LEVEL_INT_MAP_S)
#define INTERRUPT_PRO_TG1_WDT_LEVEL_INT_MAP_V  0x0000001F
#define INTERRUPT_PRO_TG1_WDT_LEVEL_INT_MAP_S  0

/* INTERRUPT_PRO_TG1_LACT_LEVEL_INT_MAP_REG register
 * TG1_LACT_LEVEL_INT interrupt configuration register
 */

#define INTERRUPT_PRO_TG1_LACT_LEVEL_INT_MAP_REG (DR_REG_INTERRUPT_BASE + 0x58)

/* INTERRUPT_PRO_TG1_LACT_LEVEL_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map TG1_LACT_LEVEL_INT interrupt signal to one
 * of the CPU interrupts.
 */

#define INTERRUPT_PRO_TG1_LACT_LEVEL_INT_MAP    0x0000001F
#define INTERRUPT_PRO_TG1_LACT_LEVEL_INT_MAP_M  (INTERRUPT_PRO_TG1_LACT_LEVEL_INT_MAP_V << INTERRUPT_PRO_TG1_LACT_LEVEL_INT_MAP_S)
#define INTERRUPT_PRO_TG1_LACT_LEVEL_INT_MAP_V  0x0000001F
#define INTERRUPT_PRO_TG1_LACT_LEVEL_INT_MAP_S  0

/* INTERRUPT_PRO_GPIO_INTERRUPT_PRO_MAP_REG register
 * GPIO_INTERRUPT_PRO interrupt configuration register
 */

#define INTERRUPT_PRO_GPIO_INTERRUPT_PRO_MAP_REG (DR_REG_INTERRUPT_BASE + 0x5c)

/* INTERRUPT_PRO_GPIO_INTERRUPT_PRO_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map GPIO_INTERRUPT_PRO interrupt signal to one
 * of the CPU interrupts.
 */

#define INTERRUPT_PRO_GPIO_INTERRUPT_PRO_MAP    0x0000001F
#define INTERRUPT_PRO_GPIO_INTERRUPT_PRO_MAP_M  (INTERRUPT_PRO_GPIO_INTERRUPT_PRO_MAP_V << INTERRUPT_PRO_GPIO_INTERRUPT_PRO_MAP_S)
#define INTERRUPT_PRO_GPIO_INTERRUPT_PRO_MAP_V  0x0000001F
#define INTERRUPT_PRO_GPIO_INTERRUPT_PRO_MAP_S  0

/* INTERRUPT_PRO_GPIO_INTERRUPT_PRO_NMI_MAP_REG register
 * GPIO_INTERRUPT_PRO_NMI interrupt configuration register
 */

#define INTERRUPT_PRO_GPIO_INTERRUPT_PRO_NMI_MAP_REG (DR_REG_INTERRUPT_BASE + 0x60)

/* INTERRUPT_PRO_GPIO_INTERRUPT_PRO_NMI_MAP : R/W; bitpos: [4:0]; default:
 * 16;
 * This register is used to map GPIO_INTERRUPT_PRO_NMI interrupt signal to
 * one of the CPU interrupts.
 */

#define INTERRUPT_PRO_GPIO_INTERRUPT_PRO_NMI_MAP    0x0000001F
#define INTERRUPT_PRO_GPIO_INTERRUPT_PRO_NMI_MAP_M  (INTERRUPT_PRO_GPIO_INTERRUPT_PRO_NMI_MAP_V << INTERRUPT_PRO_GPIO_INTERRUPT_PRO_NMI_MAP_S)
#define INTERRUPT_PRO_GPIO_INTERRUPT_PRO_NMI_MAP_V  0x0000001F
#define INTERRUPT_PRO_GPIO_INTERRUPT_PRO_NMI_MAP_S  0

/* INTERRUPT_PRO_GPIO_INTERRUPT_APP_MAP_REG register
 * GPIO_INTERRUPT_APP interrupt configuration register
 */

#define INTERRUPT_PRO_GPIO_INTERRUPT_APP_MAP_REG (DR_REG_INTERRUPT_BASE + 0x64)

/* INTERRUPT_PRO_GPIO_INTERRUPT_APP_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map GPIO_INTERRUPT_APP interrupt signal to one
 * of the CPU interrupts.
 */

#define INTERRUPT_PRO_GPIO_INTERRUPT_APP_MAP    0x0000001F
#define INTERRUPT_PRO_GPIO_INTERRUPT_APP_MAP_M  (INTERRUPT_PRO_GPIO_INTERRUPT_APP_MAP_V << INTERRUPT_PRO_GPIO_INTERRUPT_APP_MAP_S)
#define INTERRUPT_PRO_GPIO_INTERRUPT_APP_MAP_V  0x0000001F
#define INTERRUPT_PRO_GPIO_INTERRUPT_APP_MAP_S  0

/* INTERRUPT_PRO_GPIO_INTERRUPT_APP_NMI_MAP_REG register
 * GPIO_INTERRUPT_APP_NMI interrupt configuration register
 */

#define INTERRUPT_PRO_GPIO_INTERRUPT_APP_NMI_MAP_REG (DR_REG_INTERRUPT_BASE + 0x68)

/* INTERRUPT_PRO_GPIO_INTERRUPT_APP_NMI_MAP : R/W; bitpos: [4:0]; default:
 * 16;
 * This register is used to map GPIO_INTERRUPT_APP_NMI interrupt signal to
 * one of the CPU interrupts.
 */

#define INTERRUPT_PRO_GPIO_INTERRUPT_APP_NMI_MAP    0x0000001F
#define INTERRUPT_PRO_GPIO_INTERRUPT_APP_NMI_MAP_M  (INTERRUPT_PRO_GPIO_INTERRUPT_APP_NMI_MAP_V << INTERRUPT_PRO_GPIO_INTERRUPT_APP_NMI_MAP_S)
#define INTERRUPT_PRO_GPIO_INTERRUPT_APP_NMI_MAP_V  0x0000001F
#define INTERRUPT_PRO_GPIO_INTERRUPT_APP_NMI_MAP_S  0

/* INTERRUPT_PRO_DEDICATED_GPIO_IN_INTR_MAP_REG register
 * DEDICATED_GPIO_IN_INTR interrupt configuration register
 */

#define INTERRUPT_PRO_DEDICATED_GPIO_IN_INTR_MAP_REG (DR_REG_INTERRUPT_BASE + 0x6c)

/* INTERRUPT_PRO_DEDICATED_GPIO_IN_INTR_MAP : R/W; bitpos: [4:0]; default:
 * 16;
 * This register is used to map DEDICATED_GPIO_IN_INTR interrupt signal to
 * one of the CPU interrupts.
 */

#define INTERRUPT_PRO_DEDICATED_GPIO_IN_INTR_MAP    0x0000001F
#define INTERRUPT_PRO_DEDICATED_GPIO_IN_INTR_MAP_M  (INTERRUPT_PRO_DEDICATED_GPIO_IN_INTR_MAP_V << INTERRUPT_PRO_DEDICATED_GPIO_IN_INTR_MAP_S)
#define INTERRUPT_PRO_DEDICATED_GPIO_IN_INTR_MAP_V  0x0000001F
#define INTERRUPT_PRO_DEDICATED_GPIO_IN_INTR_MAP_S  0

/* INTERRUPT_PRO_CPU_INTR_FROM_CPU_0_MAP_REG register
 * CPU_INTR_FROM_CPU_0 interrupt configuration register
 */

#define INTERRUPT_PRO_CPU_INTR_FROM_CPU_0_MAP_REG (DR_REG_INTERRUPT_BASE + 0x70)

/* INTERRUPT_PRO_CPU_INTR_FROM_CPU_0_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map CPU_INTR_FROM_CPU_0 interrupt signal to one
 * of the CPU interrupts.
 */

#define INTERRUPT_PRO_CPU_INTR_FROM_CPU_0_MAP    0x0000001F
#define INTERRUPT_PRO_CPU_INTR_FROM_CPU_0_MAP_M  (INTERRUPT_PRO_CPU_INTR_FROM_CPU_0_MAP_V << INTERRUPT_PRO_CPU_INTR_FROM_CPU_0_MAP_S)
#define INTERRUPT_PRO_CPU_INTR_FROM_CPU_0_MAP_V  0x0000001F
#define INTERRUPT_PRO_CPU_INTR_FROM_CPU_0_MAP_S  0

/* INTERRUPT_PRO_CPU_INTR_FROM_CPU_1_MAP_REG register
 * CPU_INTR_FROM_CPU_1 interrupt configuration register
 */

#define INTERRUPT_PRO_CPU_INTR_FROM_CPU_1_MAP_REG (DR_REG_INTERRUPT_BASE + 0x74)

/* INTERRUPT_PRO_CPU_INTR_FROM_CPU_1_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map CPU_INTR_FROM_CPU_1 interrupt signal to one
 * of the CPU interrupts.
 */

#define INTERRUPT_PRO_CPU_INTR_FROM_CPU_1_MAP    0x0000001F
#define INTERRUPT_PRO_CPU_INTR_FROM_CPU_1_MAP_M  (INTERRUPT_PRO_CPU_INTR_FROM_CPU_1_MAP_V << INTERRUPT_PRO_CPU_INTR_FROM_CPU_1_MAP_S)
#define INTERRUPT_PRO_CPU_INTR_FROM_CPU_1_MAP_V  0x0000001F
#define INTERRUPT_PRO_CPU_INTR_FROM_CPU_1_MAP_S  0

/* INTERRUPT_PRO_CPU_INTR_FROM_CPU_2_MAP_REG register
 * CPU_INTR_FROM_CPU_2 interrupt configuration register
 */

#define INTERRUPT_PRO_CPU_INTR_FROM_CPU_2_MAP_REG (DR_REG_INTERRUPT_BASE + 0x78)

/* INTERRUPT_PRO_CPU_INTR_FROM_CPU_2_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map CPU_INTR_FROM_CPU_2 interrupt signal to one
 * of the CPU interrupts.
 */

#define INTERRUPT_PRO_CPU_INTR_FROM_CPU_2_MAP    0x0000001F
#define INTERRUPT_PRO_CPU_INTR_FROM_CPU_2_MAP_M  (INTERRUPT_PRO_CPU_INTR_FROM_CPU_2_MAP_V << INTERRUPT_PRO_CPU_INTR_FROM_CPU_2_MAP_S)
#define INTERRUPT_PRO_CPU_INTR_FROM_CPU_2_MAP_V  0x0000001F
#define INTERRUPT_PRO_CPU_INTR_FROM_CPU_2_MAP_S  0

/* INTERRUPT_PRO_CPU_INTR_FROM_CPU_3_MAP_REG register
 * CPU_INTR_FROM_CPU_3 interrupt configuration register
 */

#define INTERRUPT_PRO_CPU_INTR_FROM_CPU_3_MAP_REG (DR_REG_INTERRUPT_BASE + 0x7c)

/* INTERRUPT_PRO_CPU_INTR_FROM_CPU_3_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map CPU_INTR_FROM_CPU_3 interrupt signal to one
 * of the CPU interrupts.
 */

#define INTERRUPT_PRO_CPU_INTR_FROM_CPU_3_MAP    0x0000001F
#define INTERRUPT_PRO_CPU_INTR_FROM_CPU_3_MAP_M  (INTERRUPT_PRO_CPU_INTR_FROM_CPU_3_MAP_V << INTERRUPT_PRO_CPU_INTR_FROM_CPU_3_MAP_S)
#define INTERRUPT_PRO_CPU_INTR_FROM_CPU_3_MAP_V  0x0000001F
#define INTERRUPT_PRO_CPU_INTR_FROM_CPU_3_MAP_S  0

/* INTERRUPT_PRO_SPI_INTR_1_MAP_REG register
 * SPI_INTR_1 interrupt configuration register
 */

#define INTERRUPT_PRO_SPI_INTR_1_MAP_REG (DR_REG_INTERRUPT_BASE + 0x80)

/* INTERRUPT_PRO_SPI_INTR_1_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map SPI_INTR_1 interrupt signal to one of the
 * CPU interrupts.
 */

#define INTERRUPT_PRO_SPI_INTR_1_MAP    0x0000001F
#define INTERRUPT_PRO_SPI_INTR_1_MAP_M  (INTERRUPT_PRO_SPI_INTR_1_MAP_V << INTERRUPT_PRO_SPI_INTR_1_MAP_S)
#define INTERRUPT_PRO_SPI_INTR_1_MAP_V  0x0000001F
#define INTERRUPT_PRO_SPI_INTR_1_MAP_S  0

/* INTERRUPT_PRO_SPI_INTR_2_MAP_REG register
 * SPI_INTR_2 interrupt configuration register
 */

#define INTERRUPT_PRO_SPI_INTR_2_MAP_REG (DR_REG_INTERRUPT_BASE + 0x84)

/* INTERRUPT_PRO_SPI_INTR_2_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map SPI_INTR_2 interrupt signal to one of the
 * CPU interrupts.
 */

#define INTERRUPT_PRO_SPI_INTR_2_MAP    0x0000001F
#define INTERRUPT_PRO_SPI_INTR_2_MAP_M  (INTERRUPT_PRO_SPI_INTR_2_MAP_V << INTERRUPT_PRO_SPI_INTR_2_MAP_S)
#define INTERRUPT_PRO_SPI_INTR_2_MAP_V  0x0000001F
#define INTERRUPT_PRO_SPI_INTR_2_MAP_S  0

/* INTERRUPT_PRO_SPI_INTR_3_MAP_REG register
 * SPI_INTR_3 interrupt configuration register
 */

#define INTERRUPT_PRO_SPI_INTR_3_MAP_REG (DR_REG_INTERRUPT_BASE + 0x88)

/* INTERRUPT_PRO_SPI_INTR_3_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map SPI_INTR_3 interrupt signal to one of the
 * CPU interrupts.
 */

#define INTERRUPT_PRO_SPI_INTR_3_MAP    0x0000001F
#define INTERRUPT_PRO_SPI_INTR_3_MAP_M  (INTERRUPT_PRO_SPI_INTR_3_MAP_V << INTERRUPT_PRO_SPI_INTR_3_MAP_S)
#define INTERRUPT_PRO_SPI_INTR_3_MAP_V  0x0000001F
#define INTERRUPT_PRO_SPI_INTR_3_MAP_S  0

/* INTERRUPT_PRO_I2S0_INT_MAP_REG register
 * I2S0_INT interrupt configuration register
 */

#define INTERRUPT_PRO_I2S0_INT_MAP_REG (DR_REG_INTERRUPT_BASE + 0x8c)

/* INTERRUPT_PRO_I2S0_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map I2S0_INT interrupt signal to one of the CPU
 * interrupts.
 */

#define INTERRUPT_PRO_I2S0_INT_MAP    0x0000001F
#define INTERRUPT_PRO_I2S0_INT_MAP_M  (INTERRUPT_PRO_I2S0_INT_MAP_V << INTERRUPT_PRO_I2S0_INT_MAP_S)
#define INTERRUPT_PRO_I2S0_INT_MAP_V  0x0000001F
#define INTERRUPT_PRO_I2S0_INT_MAP_S  0

/* INTERRUPT_PRO_I2S1_INT_MAP_REG register
 * I2S1_INT interrupt configuration register
 */

#define INTERRUPT_PRO_I2S1_INT_MAP_REG (DR_REG_INTERRUPT_BASE + 0x90)

/* INTERRUPT_PRO_I2S1_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map I2S1_INT interrupt signal to one of the CPU
 * interrupts.
 */

#define INTERRUPT_PRO_I2S1_INT_MAP    0x0000001F
#define INTERRUPT_PRO_I2S1_INT_MAP_M  (INTERRUPT_PRO_I2S1_INT_MAP_V << INTERRUPT_PRO_I2S1_INT_MAP_S)
#define INTERRUPT_PRO_I2S1_INT_MAP_V  0x0000001F
#define INTERRUPT_PRO_I2S1_INT_MAP_S  0

/* INTERRUPT_PRO_UART_INTR_MAP_REG register
 * UART_INT interrupt configuration register
 */

#define INTERRUPT_PRO_UART_INTR_MAP_REG (DR_REG_INTERRUPT_BASE + 0x94)

/* INTERRUPT_PRO_UART_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map UART_INT interrupt signal to one of the CPU
 * interrupts.
 */

#define INTERRUPT_PRO_UART_INTR_MAP    0x0000001F
#define INTERRUPT_PRO_UART_INTR_MAP_M  (INTERRUPT_PRO_UART_INTR_MAP_V << INTERRUPT_PRO_UART_INTR_MAP_S)
#define INTERRUPT_PRO_UART_INTR_MAP_V  0x0000001F
#define INTERRUPT_PRO_UART_INTR_MAP_S  0

/* INTERRUPT_PRO_UART1_INTR_MAP_REG register
 * UART1_INT interrupt configuration register
 */

#define INTERRUPT_PRO_UART1_INTR_MAP_REG (DR_REG_INTERRUPT_BASE + 0x98)

/* INTERRUPT_PRO_UART1_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map UART1_INT interrupt signal to one of the CPU
 * interrupts.
 */

#define INTERRUPT_PRO_UART1_INTR_MAP    0x0000001F
#define INTERRUPT_PRO_UART1_INTR_MAP_M  (INTERRUPT_PRO_UART1_INTR_MAP_V << INTERRUPT_PRO_UART1_INTR_MAP_S)
#define INTERRUPT_PRO_UART1_INTR_MAP_V  0x0000001F
#define INTERRUPT_PRO_UART1_INTR_MAP_S  0

/* INTERRUPT_PRO_UART2_INTR_MAP_REG register
 * UART2_INT interrupt configuration register
 */

#define INTERRUPT_PRO_UART2_INTR_MAP_REG (DR_REG_INTERRUPT_BASE + 0x9c)

/* INTERRUPT_PRO_UART2_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map UART2_INT interrupt signal to one of the CPU
 * interrupts.
 */

#define INTERRUPT_PRO_UART2_INTR_MAP    0x0000001F
#define INTERRUPT_PRO_UART2_INTR_MAP_M  (INTERRUPT_PRO_UART2_INTR_MAP_V << INTERRUPT_PRO_UART2_INTR_MAP_S)
#define INTERRUPT_PRO_UART2_INTR_MAP_V  0x0000001F
#define INTERRUPT_PRO_UART2_INTR_MAP_S  0

/* INTERRUPT_PRO_SDIO_HOST_INTERRUPT_MAP_REG register
 * SDIO_HOST_INTERRUPT configuration register
 */

#define INTERRUPT_PRO_SDIO_HOST_INTERRUPT_MAP_REG (DR_REG_INTERRUPT_BASE + 0xa0)

/* INTERRUPT_PRO_SDIO_HOST_INTERRUPT_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map SDIO_HOST_INTERRUPT signal to one of the CPU
 * interrupts.
 */

#define INTERRUPT_PRO_SDIO_HOST_INTERRUPT_MAP    0x0000001F
#define INTERRUPT_PRO_SDIO_HOST_INTERRUPT_MAP_M  (INTERRUPT_PRO_SDIO_HOST_INTERRUPT_MAP_V << INTERRUPT_PRO_SDIO_HOST_INTERRUPT_MAP_S)
#define INTERRUPT_PRO_SDIO_HOST_INTERRUPT_MAP_V  0x0000001F
#define INTERRUPT_PRO_SDIO_HOST_INTERRUPT_MAP_S  0

/* INTERRUPT_PRO_PWM0_INTR_MAP_REG register
 * PWM0_INTR interrupt configuration register
 */

#define INTERRUPT_PRO_PWM0_INTR_MAP_REG (DR_REG_INTERRUPT_BASE + 0xa4)

/* INTERRUPT_PRO_PWM0_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map PWM0_INTR interrupt signal to one of the CPU
 * interrupts.
 */

#define INTERRUPT_PRO_PWM0_INTR_MAP    0x0000001F
#define INTERRUPT_PRO_PWM0_INTR_MAP_M  (INTERRUPT_PRO_PWM0_INTR_MAP_V << INTERRUPT_PRO_PWM0_INTR_MAP_S)
#define INTERRUPT_PRO_PWM0_INTR_MAP_V  0x0000001F
#define INTERRUPT_PRO_PWM0_INTR_MAP_S  0

/* INTERRUPT_PRO_PWM1_INTR_MAP_REG register
 * PWM1_INTR interrupt configuration register
 */

#define INTERRUPT_PRO_PWM1_INTR_MAP_REG (DR_REG_INTERRUPT_BASE + 0xa8)

/* INTERRUPT_PRO_PWM1_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map PWM1_INTR interrupt signal to one of the CPU
 * interrupts.
 */

#define INTERRUPT_PRO_PWM1_INTR_MAP    0x0000001F
#define INTERRUPT_PRO_PWM1_INTR_MAP_M  (INTERRUPT_PRO_PWM1_INTR_MAP_V << INTERRUPT_PRO_PWM1_INTR_MAP_S)
#define INTERRUPT_PRO_PWM1_INTR_MAP_V  0x0000001F
#define INTERRUPT_PRO_PWM1_INTR_MAP_S  0

/* INTERRUPT_PRO_PWM2_INTR_MAP_REG register
 * PWM2_INTR interrupt configuration register
 */

#define INTERRUPT_PRO_PWM2_INTR_MAP_REG (DR_REG_INTERRUPT_BASE + 0xac)

/* INTERRUPT_PRO_PWM2_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map PWM2_INTR interrupt signal to one of the CPU
 * interrupts.
 */

#define INTERRUPT_PRO_PWM2_INTR_MAP    0x0000001F
#define INTERRUPT_PRO_PWM2_INTR_MAP_M  (INTERRUPT_PRO_PWM2_INTR_MAP_V << INTERRUPT_PRO_PWM2_INTR_MAP_S)
#define INTERRUPT_PRO_PWM2_INTR_MAP_V  0x0000001F
#define INTERRUPT_PRO_PWM2_INTR_MAP_S  0

/* INTERRUPT_PRO_PWM3_INTR_MAP_REG register
 * PWM3_INTR interrupt configuration register
 */

#define INTERRUPT_PRO_PWM3_INTR_MAP_REG (DR_REG_INTERRUPT_BASE + 0xb0)

/* INTERRUPT_PRO_PWM3_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map PWM3_INTR interrupt signal to one of the CPU
 * interrupts.
 */

#define INTERRUPT_PRO_PWM3_INTR_MAP    0x0000001F
#define INTERRUPT_PRO_PWM3_INTR_MAP_M  (INTERRUPT_PRO_PWM3_INTR_MAP_V << INTERRUPT_PRO_PWM3_INTR_MAP_S)
#define INTERRUPT_PRO_PWM3_INTR_MAP_V  0x0000001F
#define INTERRUPT_PRO_PWM3_INTR_MAP_S  0

/* INTERRUPT_PRO_LEDC_INT_MAP_REG register
 * LEDC_INTR interrupt configuration register
 */

#define INTERRUPT_PRO_LEDC_INT_MAP_REG (DR_REG_INTERRUPT_BASE + 0xb4)

/* INTERRUPT_PRO_LEDC_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map LEDC_INTR interrupt signal to one of the CPU
 * interrupts.
 */

#define INTERRUPT_PRO_LEDC_INT_MAP    0x0000001F
#define INTERRUPT_PRO_LEDC_INT_MAP_M  (INTERRUPT_PRO_LEDC_INT_MAP_V << INTERRUPT_PRO_LEDC_INT_MAP_S)
#define INTERRUPT_PRO_LEDC_INT_MAP_V  0x0000001F
#define INTERRUPT_PRO_LEDC_INT_MAP_S  0

/* INTERRUPT_PRO_EFUSE_INT_MAP_REG register
 * EFUSE_INT interrupt configuration register
 */

#define INTERRUPT_PRO_EFUSE_INT_MAP_REG (DR_REG_INTERRUPT_BASE + 0xb8)

/* INTERRUPT_PRO_EFUSE_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map EFUSE_INT interrupt signal to one of the CPU
 * interrupts.
 */

#define INTERRUPT_PRO_EFUSE_INT_MAP    0x0000001F
#define INTERRUPT_PRO_EFUSE_INT_MAP_M  (INTERRUPT_PRO_EFUSE_INT_MAP_V << INTERRUPT_PRO_EFUSE_INT_MAP_S)
#define INTERRUPT_PRO_EFUSE_INT_MAP_V  0x0000001F
#define INTERRUPT_PRO_EFUSE_INT_MAP_S  0

/* INTERRUPT_PRO_CAN_INT_MAP_REG register
 * CAN_INT interrupt configuration register
 */

#define INTERRUPT_PRO_CAN_INT_MAP_REG (DR_REG_INTERRUPT_BASE + 0xbc)

/* INTERRUPT_PRO_CAN_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map CAN_INT interrupt signal to one of the CPU
 * interrupts.
 */

#define INTERRUPT_PRO_CAN_INT_MAP    0x0000001F
#define INTERRUPT_PRO_CAN_INT_MAP_M  (INTERRUPT_PRO_CAN_INT_MAP_V << INTERRUPT_PRO_CAN_INT_MAP_S)
#define INTERRUPT_PRO_CAN_INT_MAP_V  0x0000001F
#define INTERRUPT_PRO_CAN_INT_MAP_S  0

/* INTERRUPT_PRO_USB_INTR_MAP_REG register
 * USB_INT interrupt configuration register
 */

#define INTERRUPT_PRO_USB_INTR_MAP_REG (DR_REG_INTERRUPT_BASE + 0xc0)

/* INTERRUPT_PRO_USB_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map USB_INT interrupt signal to one of the CPU
 * interrupts.
 */

#define INTERRUPT_PRO_USB_INTR_MAP    0x0000001F
#define INTERRUPT_PRO_USB_INTR_MAP_M  (INTERRUPT_PRO_USB_INTR_MAP_V << INTERRUPT_PRO_USB_INTR_MAP_S)
#define INTERRUPT_PRO_USB_INTR_MAP_V  0x0000001F
#define INTERRUPT_PRO_USB_INTR_MAP_S  0

/* INTERRUPT_PRO_RTC_CORE_INTR_MAP_REG register
 * RTC_CORE_INTR interrupt configuration register
 */

#define INTERRUPT_PRO_RTC_CORE_INTR_MAP_REG (DR_REG_INTERRUPT_BASE + 0xc4)

/* INTERRUPT_PRO_RTC_CORE_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map RTC_CORE_INTR interrupt signal to one of the
 * CPU interrupts.
 */

#define INTERRUPT_PRO_RTC_CORE_INTR_MAP    0x0000001F
#define INTERRUPT_PRO_RTC_CORE_INTR_MAP_M  (INTERRUPT_PRO_RTC_CORE_INTR_MAP_V << INTERRUPT_PRO_RTC_CORE_INTR_MAP_S)
#define INTERRUPT_PRO_RTC_CORE_INTR_MAP_V  0x0000001F
#define INTERRUPT_PRO_RTC_CORE_INTR_MAP_S  0

/* INTERRUPT_PRO_RMT_INTR_MAP_REG register
 * RMT_INTR interrupt configuration register
 */

#define INTERRUPT_PRO_RMT_INTR_MAP_REG (DR_REG_INTERRUPT_BASE + 0xc8)

/* INTERRUPT_PRO_RMT_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map RMT_INTR interrupt signal to one of the CPU
 * interrupts.
 */

#define INTERRUPT_PRO_RMT_INTR_MAP    0x0000001F
#define INTERRUPT_PRO_RMT_INTR_MAP_M  (INTERRUPT_PRO_RMT_INTR_MAP_V << INTERRUPT_PRO_RMT_INTR_MAP_S)
#define INTERRUPT_PRO_RMT_INTR_MAP_V  0x0000001F
#define INTERRUPT_PRO_RMT_INTR_MAP_S  0

/* INTERRUPT_PRO_PCNT_INTR_MAP_REG register
 * PCNT_INTR interrupt configuration register
 */

#define INTERRUPT_PRO_PCNT_INTR_MAP_REG (DR_REG_INTERRUPT_BASE + 0xcc)

/* INTERRUPT_PRO_PCNT_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map PCNT_INTR interrupt signal to one of the CPU
 * interrupts.
 */

#define INTERRUPT_PRO_PCNT_INTR_MAP    0x0000001F
#define INTERRUPT_PRO_PCNT_INTR_MAP_M  (INTERRUPT_PRO_PCNT_INTR_MAP_V << INTERRUPT_PRO_PCNT_INTR_MAP_S)
#define INTERRUPT_PRO_PCNT_INTR_MAP_V  0x0000001F
#define INTERRUPT_PRO_PCNT_INTR_MAP_S  0

/* INTERRUPT_PRO_I2C_EXT0_INTR_MAP_REG register
 * I2C_EXT0_INTR interrupt configuration register
 */

#define INTERRUPT_PRO_I2C_EXT0_INTR_MAP_REG (DR_REG_INTERRUPT_BASE + 0xd0)

/* INTERRUPT_PRO_I2C_EXT0_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map I2C_EXT0_INTR interrupt signal to one of the
 * CPU interrupts.
 */

#define INTERRUPT_PRO_I2C_EXT0_INTR_MAP    0x0000001F
#define INTERRUPT_PRO_I2C_EXT0_INTR_MAP_M  (INTERRUPT_PRO_I2C_EXT0_INTR_MAP_V << INTERRUPT_PRO_I2C_EXT0_INTR_MAP_S)
#define INTERRUPT_PRO_I2C_EXT0_INTR_MAP_V  0x0000001F
#define INTERRUPT_PRO_I2C_EXT0_INTR_MAP_S  0

/* INTERRUPT_PRO_I2C_EXT1_INTR_MAP_REG register
 * I2C_EXT1_INTR interrupt configuration register
 */

#define INTERRUPT_PRO_I2C_EXT1_INTR_MAP_REG (DR_REG_INTERRUPT_BASE + 0xd4)

/* INTERRUPT_PRO_I2C_EXT1_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map I2C_EXT1_INTR interrupt signal to one of the
 * CPU interrupts.
 */

#define INTERRUPT_PRO_I2C_EXT1_INTR_MAP    0x0000001F
#define INTERRUPT_PRO_I2C_EXT1_INTR_MAP_M  (INTERRUPT_PRO_I2C_EXT1_INTR_MAP_V << INTERRUPT_PRO_I2C_EXT1_INTR_MAP_S)
#define INTERRUPT_PRO_I2C_EXT1_INTR_MAP_V  0x0000001F
#define INTERRUPT_PRO_I2C_EXT1_INTR_MAP_S  0

/* INTERRUPT_PRO_RSA_INTR_MAP_REG register
 * RSA_INTR interrupt configuration register
 */

#define INTERRUPT_PRO_RSA_INTR_MAP_REG (DR_REG_INTERRUPT_BASE + 0xd8)

/* INTERRUPT_PRO_RSA_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map RSA_INTR interrupt signal to one of the CPU
 * interrupts.
 */

#define INTERRUPT_PRO_RSA_INTR_MAP    0x0000001F
#define INTERRUPT_PRO_RSA_INTR_MAP_M  (INTERRUPT_PRO_RSA_INTR_MAP_V << INTERRUPT_PRO_RSA_INTR_MAP_S)
#define INTERRUPT_PRO_RSA_INTR_MAP_V  0x0000001F
#define INTERRUPT_PRO_RSA_INTR_MAP_S  0

/* INTERRUPT_PRO_SHA_INTR_MAP_REG register
 * SHA_INTR interrupt configuration register
 */

#define INTERRUPT_PRO_SHA_INTR_MAP_REG (DR_REG_INTERRUPT_BASE + 0xdc)

/* INTERRUPT_PRO_SHA_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map SHA_INTR interrupt signal to one of the CPU
 * interrupts.
 */

#define INTERRUPT_PRO_SHA_INTR_MAP    0x0000001F
#define INTERRUPT_PRO_SHA_INTR_MAP_M  (INTERRUPT_PRO_SHA_INTR_MAP_V << INTERRUPT_PRO_SHA_INTR_MAP_S)
#define INTERRUPT_PRO_SHA_INTR_MAP_V  0x0000001F
#define INTERRUPT_PRO_SHA_INTR_MAP_S  0

/* INTERRUPT_PRO_AES_INTR_MAP_REG register
 * AES_INTR interrupt configuration register
 */

#define INTERRUPT_PRO_AES_INTR_MAP_REG (DR_REG_INTERRUPT_BASE + 0xe0)

/* INTERRUPT_PRO_AES_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map AES_INTR interrupt signal to one of the CPU
 * interrupts.
 */

#define INTERRUPT_PRO_AES_INTR_MAP    0x0000001F
#define INTERRUPT_PRO_AES_INTR_MAP_M  (INTERRUPT_PRO_AES_INTR_MAP_V << INTERRUPT_PRO_AES_INTR_MAP_S)
#define INTERRUPT_PRO_AES_INTR_MAP_V  0x0000001F
#define INTERRUPT_PRO_AES_INTR_MAP_S  0

/* INTERRUPT_PRO_SPI2_DMA_INT_MAP_REG register
 * SPI2_DMA_INT interrupt configuration register
 */

#define INTERRUPT_PRO_SPI2_DMA_INT_MAP_REG (DR_REG_INTERRUPT_BASE + 0xe4)

/* INTERRUPT_PRO_SPI2_DMA_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map AES_INTR interrupt signal to one of the CPU
 * interrupts.
 */

#define INTERRUPT_PRO_SPI2_DMA_INT_MAP    0x0000001F
#define INTERRUPT_PRO_SPI2_DMA_INT_MAP_M  (INTERRUPT_PRO_SPI2_DMA_INT_MAP_V << INTERRUPT_PRO_SPI2_DMA_INT_MAP_S)
#define INTERRUPT_PRO_SPI2_DMA_INT_MAP_V  0x0000001F
#define INTERRUPT_PRO_SPI2_DMA_INT_MAP_S  0

/* INTERRUPT_PRO_SPI3_DMA_INT_MAP_REG register
 * SPI3_DMA_INT interrupt configuration register
 */

#define INTERRUPT_PRO_SPI3_DMA_INT_MAP_REG (DR_REG_INTERRUPT_BASE + 0xe8)

/* INTERRUPT_PRO_SPI3_DMA_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map SPI3_DMA_INT dma interrupt signal to one of
 * the CPU interrupts.
 */

#define INTERRUPT_PRO_SPI3_DMA_INT_MAP    0x0000001F
#define INTERRUPT_PRO_SPI3_DMA_INT_MAP_M  (INTERRUPT_PRO_SPI3_DMA_INT_MAP_V << INTERRUPT_PRO_SPI3_DMA_INT_MAP_S)
#define INTERRUPT_PRO_SPI3_DMA_INT_MAP_V  0x0000001F
#define INTERRUPT_PRO_SPI3_DMA_INT_MAP_S  0

/* INTERRUPT_PRO_WDG_INT_MAP_REG register
 * WDG_INT interrupt configuration register
 */

#define INTERRUPT_PRO_WDG_INT_MAP_REG (DR_REG_INTERRUPT_BASE + 0xec)

/* INTERRUPT_PRO_WDG_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map WDG_INT interrupt signal to one of the CPU
 * interrupts.
 */

#define INTERRUPT_PRO_WDG_INT_MAP    0x0000001F
#define INTERRUPT_PRO_WDG_INT_MAP_M  (INTERRUPT_PRO_WDG_INT_MAP_V << INTERRUPT_PRO_WDG_INT_MAP_S)
#define INTERRUPT_PRO_WDG_INT_MAP_V  0x0000001F
#define INTERRUPT_PRO_WDG_INT_MAP_S  0

/* INTERRUPT_PRO_TIMER_INT1_MAP_REG register
 * TIMER_INT1 interrupt configuration register
 */

#define INTERRUPT_PRO_TIMER_INT1_MAP_REG (DR_REG_INTERRUPT_BASE + 0xf0)

/* INTERRUPT_PRO_TIMER_INT1_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map TIMER_INT1 interrupt signal to one of the
 * CPU interrupts.
 */

#define INTERRUPT_PRO_TIMER_INT1_MAP    0x0000001F
#define INTERRUPT_PRO_TIMER_INT1_MAP_M  (INTERRUPT_PRO_TIMER_INT1_MAP_V << INTERRUPT_PRO_TIMER_INT1_MAP_S)
#define INTERRUPT_PRO_TIMER_INT1_MAP_V  0x0000001F
#define INTERRUPT_PRO_TIMER_INT1_MAP_S  0

/* INTERRUPT_PRO_TIMER_INT2_MAP_REG register
 * TIMER_INT2 interrupt configuration register
 */

#define INTERRUPT_PRO_TIMER_INT2_MAP_REG (DR_REG_INTERRUPT_BASE + 0xf4)

/* INTERRUPT_PRO_TIMER_INT2_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map TIMER_INT2 interrupt signal to one of the
 * CPU interrupts.
 */

#define INTERRUPT_PRO_TIMER_INT2_MAP    0x0000001F
#define INTERRUPT_PRO_TIMER_INT2_MAP_M  (INTERRUPT_PRO_TIMER_INT2_MAP_V << INTERRUPT_PRO_TIMER_INT2_MAP_S)
#define INTERRUPT_PRO_TIMER_INT2_MAP_V  0x0000001F
#define INTERRUPT_PRO_TIMER_INT2_MAP_S  0

/* INTERRUPT_PRO_TG_T0_EDGE_INT_MAP_REG register
 * TG_T0_EDGE_INT interrupt configuration register
 */

#define INTERRUPT_PRO_TG_T0_EDGE_INT_MAP_REG (DR_REG_INTERRUPT_BASE + 0xf8)

/* INTERRUPT_PRO_TG_T0_EDGE_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map TG_T0_EDGE_INT interrupt signal to one of
 * the CPU interrupts.
 */

#define INTERRUPT_PRO_TG_T0_EDGE_INT_MAP    0x0000001F
#define INTERRUPT_PRO_TG_T0_EDGE_INT_MAP_M  (INTERRUPT_PRO_TG_T0_EDGE_INT_MAP_V << INTERRUPT_PRO_TG_T0_EDGE_INT_MAP_S)
#define INTERRUPT_PRO_TG_T0_EDGE_INT_MAP_V  0x0000001F
#define INTERRUPT_PRO_TG_T0_EDGE_INT_MAP_S  0

/* INTERRUPT_PRO_TG_T1_EDGE_INT_MAP_REG register
 * TG_T1_EDGE_INT interrupt configuration register
 */

#define INTERRUPT_PRO_TG_T1_EDGE_INT_MAP_REG (DR_REG_INTERRUPT_BASE + 0xfc)

/* INTERRUPT_PRO_TG_T1_EDGE_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map TG_T1_EDGE_INT interrupt signal to one of
 * the CPU interrupts.
 */

#define INTERRUPT_PRO_TG_T1_EDGE_INT_MAP    0x0000001F
#define INTERRUPT_PRO_TG_T1_EDGE_INT_MAP_M  (INTERRUPT_PRO_TG_T1_EDGE_INT_MAP_V << INTERRUPT_PRO_TG_T1_EDGE_INT_MAP_S)
#define INTERRUPT_PRO_TG_T1_EDGE_INT_MAP_V  0x0000001F
#define INTERRUPT_PRO_TG_T1_EDGE_INT_MAP_S  0

/* INTERRUPT_PRO_TG_WDT_EDGE_INT_MAP_REG register
 * TG_WDT_EDGE_INT interrupt configuration register
 */

#define INTERRUPT_PRO_TG_WDT_EDGE_INT_MAP_REG (DR_REG_INTERRUPT_BASE + 0x100)

/* INTERRUPT_PRO_TG_WDT_EDGE_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map TG_WDT_EDGE_INT interrupt signal to one of
 * the CPU interrupts.
 */

#define INTERRUPT_PRO_TG_WDT_EDGE_INT_MAP    0x0000001F
#define INTERRUPT_PRO_TG_WDT_EDGE_INT_MAP_M  (INTERRUPT_PRO_TG_WDT_EDGE_INT_MAP_V << INTERRUPT_PRO_TG_WDT_EDGE_INT_MAP_S)
#define INTERRUPT_PRO_TG_WDT_EDGE_INT_MAP_V  0x0000001F
#define INTERRUPT_PRO_TG_WDT_EDGE_INT_MAP_S  0

/* INTERRUPT_PRO_TG_LACT_EDGE_INT_MAP_REG register
 * TG_LACT_EDGE_INT interrupt configuration register
 */

#define INTERRUPT_PRO_TG_LACT_EDGE_INT_MAP_REG (DR_REG_INTERRUPT_BASE + 0x104)

/* INTERRUPT_PRO_TG_LACT_EDGE_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map TG_LACT_EDGE_INT interrupt signal to one of
 * the CPU interrupts.
 */

#define INTERRUPT_PRO_TG_LACT_EDGE_INT_MAP    0x0000001F
#define INTERRUPT_PRO_TG_LACT_EDGE_INT_MAP_M  (INTERRUPT_PRO_TG_LACT_EDGE_INT_MAP_V << INTERRUPT_PRO_TG_LACT_EDGE_INT_MAP_S)
#define INTERRUPT_PRO_TG_LACT_EDGE_INT_MAP_V  0x0000001F
#define INTERRUPT_PRO_TG_LACT_EDGE_INT_MAP_S  0

/* INTERRUPT_PRO_TG1_T0_EDGE_INT_MAP_REG register
 * TG1_T0_EDGE_INT interrupt configuration register
 */

#define INTERRUPT_PRO_TG1_T0_EDGE_INT_MAP_REG (DR_REG_INTERRUPT_BASE + 0x108)

/* INTERRUPT_PRO_TG1_T0_EDGE_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map TG1_T0_EDGE_INT interrupt signal to one of
 * the CPU interrupts.
 */

#define INTERRUPT_PRO_TG1_T0_EDGE_INT_MAP    0x0000001F
#define INTERRUPT_PRO_TG1_T0_EDGE_INT_MAP_M  (INTERRUPT_PRO_TG1_T0_EDGE_INT_MAP_V << INTERRUPT_PRO_TG1_T0_EDGE_INT_MAP_S)
#define INTERRUPT_PRO_TG1_T0_EDGE_INT_MAP_V  0x0000001F
#define INTERRUPT_PRO_TG1_T0_EDGE_INT_MAP_S  0

/* INTERRUPT_PRO_TG1_T1_EDGE_INT_MAP_REG register
 * TG1_T1_EDGE_INT interrupt configuration register
 */

#define INTERRUPT_PRO_TG1_T1_EDGE_INT_MAP_REG (DR_REG_INTERRUPT_BASE + 0x10c)

/* INTERRUPT_PRO_TG1_T1_EDGE_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map TG1_T1_EDGE_INT interrupt signal to one of
 * the CPU interrupts.
 */

#define INTERRUPT_PRO_TG1_T1_EDGE_INT_MAP    0x0000001F
#define INTERRUPT_PRO_TG1_T1_EDGE_INT_MAP_M  (INTERRUPT_PRO_TG1_T1_EDGE_INT_MAP_V << INTERRUPT_PRO_TG1_T1_EDGE_INT_MAP_S)
#define INTERRUPT_PRO_TG1_T1_EDGE_INT_MAP_V  0x0000001F
#define INTERRUPT_PRO_TG1_T1_EDGE_INT_MAP_S  0

/* INTERRUPT_PRO_TG1_WDT_EDGE_INT_MAP_REG register
 * TG1_WDT_EDGE_INT interrupt configuration register
 */

#define INTERRUPT_PRO_TG1_WDT_EDGE_INT_MAP_REG (DR_REG_INTERRUPT_BASE + 0x110)

/* INTERRUPT_PRO_TG1_WDT_EDGE_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map TG1_WDT_EDGE_INT interrupt signal to one of
 * the CPU interrupts.
 */

#define INTERRUPT_PRO_TG1_WDT_EDGE_INT_MAP    0x0000001F
#define INTERRUPT_PRO_TG1_WDT_EDGE_INT_MAP_M  (INTERRUPT_PRO_TG1_WDT_EDGE_INT_MAP_V << INTERRUPT_PRO_TG1_WDT_EDGE_INT_MAP_S)
#define INTERRUPT_PRO_TG1_WDT_EDGE_INT_MAP_V  0x0000001F
#define INTERRUPT_PRO_TG1_WDT_EDGE_INT_MAP_S  0

/* INTERRUPT_PRO_TG1_LACT_EDGE_INT_MAP_REG register
 * TG1_LACT_EDGE_INT interrupt configuration register
 */

#define INTERRUPT_PRO_TG1_LACT_EDGE_INT_MAP_REG (DR_REG_INTERRUPT_BASE + 0x114)

/* INTERRUPT_PRO_TG1_LACT_EDGE_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map TG1_LACT_EDGE_INT interrupt signal to one of
 * the CPU interrupts.
 */

#define INTERRUPT_PRO_TG1_LACT_EDGE_INT_MAP    0x0000001F
#define INTERRUPT_PRO_TG1_LACT_EDGE_INT_MAP_M  (INTERRUPT_PRO_TG1_LACT_EDGE_INT_MAP_V << INTERRUPT_PRO_TG1_LACT_EDGE_INT_MAP_S)
#define INTERRUPT_PRO_TG1_LACT_EDGE_INT_MAP_V  0x0000001F
#define INTERRUPT_PRO_TG1_LACT_EDGE_INT_MAP_S  0

/* INTERRUPT_PRO_CACHE_IA_INT_MAP_REG register
 * CACHE_IA_INT interrupt configuration register
 */

#define INTERRUPT_PRO_CACHE_IA_INT_MAP_REG (DR_REG_INTERRUPT_BASE + 0x118)

/* INTERRUPT_PRO_CACHE_IA_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map CACHE_IA_INT interrupt signal to one of the
 * CPU interrupts.
 */

#define INTERRUPT_PRO_CACHE_IA_INT_MAP    0x0000001F
#define INTERRUPT_PRO_CACHE_IA_INT_MAP_M  (INTERRUPT_PRO_CACHE_IA_INT_MAP_V << INTERRUPT_PRO_CACHE_IA_INT_MAP_S)
#define INTERRUPT_PRO_CACHE_IA_INT_MAP_V  0x0000001F
#define INTERRUPT_PRO_CACHE_IA_INT_MAP_S  0

/* INTERRUPT_PRO_SYSTIMER_TARGET0_INT_MAP_REG register
 * SYSTIMER_TARGET0_INT interrupt configuration register
 */

#define INTERRUPT_PRO_SYSTIMER_TARGET0_INT_MAP_REG (DR_REG_INTERRUPT_BASE + 0x11c)

/* INTERRUPT_PRO_SYSTIMER_TARGET0_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map SYSTIMER_TARGET0_INT interrupt signal to one
 * of the CPU interrupts.
 */

#define INTERRUPT_PRO_SYSTIMER_TARGET0_INT_MAP    0x0000001F
#define INTERRUPT_PRO_SYSTIMER_TARGET0_INT_MAP_M  (INTERRUPT_PRO_SYSTIMER_TARGET0_INT_MAP_V << INTERRUPT_PRO_SYSTIMER_TARGET0_INT_MAP_S)
#define INTERRUPT_PRO_SYSTIMER_TARGET0_INT_MAP_V  0x0000001F
#define INTERRUPT_PRO_SYSTIMER_TARGET0_INT_MAP_S  0

/* INTERRUPT_PRO_SYSTIMER_TARGET1_INT_MAP_REG register
 * SYSTIMER_TARGET1_INT interrupt configuration register
 */

#define INTERRUPT_PRO_SYSTIMER_TARGET1_INT_MAP_REG (DR_REG_INTERRUPT_BASE + 0x120)

/* INTERRUPT_PRO_SYSTIMER_TARGET1_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map SYSTIMER_TARGET1_INT interrupt signal to one
 * of the CPU interrupts.
 */

#define INTERRUPT_PRO_SYSTIMER_TARGET1_INT_MAP    0x0000001F
#define INTERRUPT_PRO_SYSTIMER_TARGET1_INT_MAP_M  (INTERRUPT_PRO_SYSTIMER_TARGET1_INT_MAP_V << INTERRUPT_PRO_SYSTIMER_TARGET1_INT_MAP_S)
#define INTERRUPT_PRO_SYSTIMER_TARGET1_INT_MAP_V  0x0000001F
#define INTERRUPT_PRO_SYSTIMER_TARGET1_INT_MAP_S  0

/* INTERRUPT_PRO_SYSTIMER_TARGET2_INT_MAP_REG register
 * SYSTIMER_TARGET2_INT interrupt configuration register
 */

#define INTERRUPT_PRO_SYSTIMER_TARGET2_INT_MAP_REG (DR_REG_INTERRUPT_BASE + 0x124)

/* INTERRUPT_PRO_SYSTIMER_TARGET2_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map SYSTIMER_TARGET2_INT interrupt signal to one
 * of the CPU interrupts.
 */

#define INTERRUPT_PRO_SYSTIMER_TARGET2_INT_MAP    0x0000001F
#define INTERRUPT_PRO_SYSTIMER_TARGET2_INT_MAP_M  (INTERRUPT_PRO_SYSTIMER_TARGET2_INT_MAP_V << INTERRUPT_PRO_SYSTIMER_TARGET2_INT_MAP_S)
#define INTERRUPT_PRO_SYSTIMER_TARGET2_INT_MAP_V  0x0000001F
#define INTERRUPT_PRO_SYSTIMER_TARGET2_INT_MAP_S  0

/* INTERRUPT_PRO_ASSIST_DEBUG_INTR_MAP_REG register
 * ASSIST_DEBUG_INTR interrupt configuration register
 */

#define INTERRUPT_PRO_ASSIST_DEBUG_INTR_MAP_REG (DR_REG_INTERRUPT_BASE + 0x128)

/* INTERRUPT_PRO_ASSIST_DEBUG_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map ASSIST_DEBUG_INTR interrupt signal to one of
 * the CPU interrupts.
 */

#define INTERRUPT_PRO_ASSIST_DEBUG_INTR_MAP    0x0000001F
#define INTERRUPT_PRO_ASSIST_DEBUG_INTR_MAP_M  (INTERRUPT_PRO_ASSIST_DEBUG_INTR_MAP_V << INTERRUPT_PRO_ASSIST_DEBUG_INTR_MAP_S)
#define INTERRUPT_PRO_ASSIST_DEBUG_INTR_MAP_V  0x0000001F
#define INTERRUPT_PRO_ASSIST_DEBUG_INTR_MAP_S  0

/* INTERRUPT_PRO_PMS_PRO_IRAM0_ILG_INTR_MAP_REG register
 * PMS_PRO_IRAM0_ILG interrupt configuration register
 */

#define INTERRUPT_PRO_PMS_PRO_IRAM0_ILG_INTR_MAP_REG (DR_REG_INTERRUPT_BASE + 0x12c)

/* INTERRUPT_PRO_PMS_PRO_IRAM0_ILG_INTR_MAP : R/W; bitpos: [4:0]; default:
 * 16;
 * This register is used to map PMS_PRO_IRAM0_ILG interrupt signal to one of
 * the CPU interrupts.
 */

#define INTERRUPT_PRO_PMS_PRO_IRAM0_ILG_INTR_MAP    0x0000001F
#define INTERRUPT_PRO_PMS_PRO_IRAM0_ILG_INTR_MAP_M  (INTERRUPT_PRO_PMS_PRO_IRAM0_ILG_INTR_MAP_V << INTERRUPT_PRO_PMS_PRO_IRAM0_ILG_INTR_MAP_S)
#define INTERRUPT_PRO_PMS_PRO_IRAM0_ILG_INTR_MAP_V  0x0000001F
#define INTERRUPT_PRO_PMS_PRO_IRAM0_ILG_INTR_MAP_S  0

/* INTERRUPT_PRO_PMS_PRO_DRAM0_ILG_INTR_MAP_REG register
 * PMS_PRO_DRAM0_ILG interrupt configuration register
 */

#define INTERRUPT_PRO_PMS_PRO_DRAM0_ILG_INTR_MAP_REG (DR_REG_INTERRUPT_BASE + 0x130)

/* INTERRUPT_PRO_PMS_PRO_DRAM0_ILG_INTR_MAP : R/W; bitpos: [4:0]; default:
 * 16;
 * This register is used to map PMS_PRO_DRAM0_ILG interrupt signal to one of
 * the CPU interrupts.
 */

#define INTERRUPT_PRO_PMS_PRO_DRAM0_ILG_INTR_MAP    0x0000001F
#define INTERRUPT_PRO_PMS_PRO_DRAM0_ILG_INTR_MAP_M  (INTERRUPT_PRO_PMS_PRO_DRAM0_ILG_INTR_MAP_V << INTERRUPT_PRO_PMS_PRO_DRAM0_ILG_INTR_MAP_S)
#define INTERRUPT_PRO_PMS_PRO_DRAM0_ILG_INTR_MAP_V  0x0000001F
#define INTERRUPT_PRO_PMS_PRO_DRAM0_ILG_INTR_MAP_S  0

/* INTERRUPT_PRO_PMS_PRO_DPORT_ILG_INTR_MAP_REG register
 * PMS_PRO_DPORT_ILG interrupt configuration register
 */

#define INTERRUPT_PRO_PMS_PRO_DPORT_ILG_INTR_MAP_REG (DR_REG_INTERRUPT_BASE + 0x134)

/* INTERRUPT_PRO_PMS_PRO_DPORT_ILG_INTR_MAP : R/W; bitpos: [4:0]; default:
 * 16;
 * This register is used to map PMS_PRO_DPORT_ILG interrupt signal to one of
 * the CPU interrupts.
 */

#define INTERRUPT_PRO_PMS_PRO_DPORT_ILG_INTR_MAP    0x0000001F
#define INTERRUPT_PRO_PMS_PRO_DPORT_ILG_INTR_MAP_M  (INTERRUPT_PRO_PMS_PRO_DPORT_ILG_INTR_MAP_V << INTERRUPT_PRO_PMS_PRO_DPORT_ILG_INTR_MAP_S)
#define INTERRUPT_PRO_PMS_PRO_DPORT_ILG_INTR_MAP_V  0x0000001F
#define INTERRUPT_PRO_PMS_PRO_DPORT_ILG_INTR_MAP_S  0

/* INTERRUPT_PRO_PMS_PRO_AHB_ILG_INTR_MAP_REG register
 * PMS_PRO_AHB_ILG interrupt configuration register
 */

#define INTERRUPT_PRO_PMS_PRO_AHB_ILG_INTR_MAP_REG (DR_REG_INTERRUPT_BASE + 0x138)

/* INTERRUPT_PRO_PMS_PRO_AHB_ILG_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map PMS_PRO_AHB_ILG interrupt signal to one of
 * the CPU interrupts.
 */

#define INTERRUPT_PRO_PMS_PRO_AHB_ILG_INTR_MAP    0x0000001F
#define INTERRUPT_PRO_PMS_PRO_AHB_ILG_INTR_MAP_M  (INTERRUPT_PRO_PMS_PRO_AHB_ILG_INTR_MAP_V << INTERRUPT_PRO_PMS_PRO_AHB_ILG_INTR_MAP_S)
#define INTERRUPT_PRO_PMS_PRO_AHB_ILG_INTR_MAP_V  0x0000001F
#define INTERRUPT_PRO_PMS_PRO_AHB_ILG_INTR_MAP_S  0

/* INTERRUPT_PRO_PMS_PRO_CACHE_ILG_INTR_MAP_REG register
 * PMS_PRO_CACHE_ILG interrupt configuration register
 */

#define INTERRUPT_PRO_PMS_PRO_CACHE_ILG_INTR_MAP_REG (DR_REG_INTERRUPT_BASE + 0x13c)

/* INTERRUPT_PRO_PMS_PRO_CACHE_ILG_INTR_MAP : R/W; bitpos: [4:0]; default:
 * 16;
 * This register is used to map PMS_PRO_CACHE_ILG interrupt signal to one of
 * the CPU interrupts.
 */

#define INTERRUPT_PRO_PMS_PRO_CACHE_ILG_INTR_MAP    0x0000001F
#define INTERRUPT_PRO_PMS_PRO_CACHE_ILG_INTR_MAP_M  (INTERRUPT_PRO_PMS_PRO_CACHE_ILG_INTR_MAP_V << INTERRUPT_PRO_PMS_PRO_CACHE_ILG_INTR_MAP_S)
#define INTERRUPT_PRO_PMS_PRO_CACHE_ILG_INTR_MAP_V  0x0000001F
#define INTERRUPT_PRO_PMS_PRO_CACHE_ILG_INTR_MAP_S  0

/* INTERRUPT_PRO_PMS_DMA_APB_I_ILG_INTR_MAP_REG register
 * PMS_DMA_APB_I_ILG  interrupt configuration register
 */

#define INTERRUPT_PRO_PMS_DMA_APB_I_ILG_INTR_MAP_REG (DR_REG_INTERRUPT_BASE + 0x140)

/* INTERRUPT_PRO_PMS_DMA_APB_I_ILG_INTR_MAP : R/W; bitpos: [4:0]; default:
 * 16;
 * This register is used to map PMS_DMA_APB_I_ILG interrupt signal to one of
 * the CPU interrupts.
 */

#define INTERRUPT_PRO_PMS_DMA_APB_I_ILG_INTR_MAP    0x0000001F
#define INTERRUPT_PRO_PMS_DMA_APB_I_ILG_INTR_MAP_M  (INTERRUPT_PRO_PMS_DMA_APB_I_ILG_INTR_MAP_V << INTERRUPT_PRO_PMS_DMA_APB_I_ILG_INTR_MAP_S)
#define INTERRUPT_PRO_PMS_DMA_APB_I_ILG_INTR_MAP_V  0x0000001F
#define INTERRUPT_PRO_PMS_DMA_APB_I_ILG_INTR_MAP_S  0

/* INTERRUPT_PRO_PMS_DMA_RX_I_ILG_INTR_MAP_REG register
 * PMS_DMA_RX_I_ILG interrupt configuration register
 */

#define INTERRUPT_PRO_PMS_DMA_RX_I_ILG_INTR_MAP_REG (DR_REG_INTERRUPT_BASE + 0x144)

/* INTERRUPT_PRO_PMS_DMA_RX_I_ILG_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map PMS_DMA_RX_I_ILG interrupt signal to one of
 * the CPU interrupts.
 */

#define INTERRUPT_PRO_PMS_DMA_RX_I_ILG_INTR_MAP    0x0000001F
#define INTERRUPT_PRO_PMS_DMA_RX_I_ILG_INTR_MAP_M  (INTERRUPT_PRO_PMS_DMA_RX_I_ILG_INTR_MAP_V << INTERRUPT_PRO_PMS_DMA_RX_I_ILG_INTR_MAP_S)
#define INTERRUPT_PRO_PMS_DMA_RX_I_ILG_INTR_MAP_V  0x0000001F
#define INTERRUPT_PRO_PMS_DMA_RX_I_ILG_INTR_MAP_S  0

/* INTERRUPT_PRO_PMS_DMA_TX_I_ILG_INTR_MAP_REG register
 * PMS_DMA_TX_I_ILG interrupt configuration register
 */

#define INTERRUPT_PRO_PMS_DMA_TX_I_ILG_INTR_MAP_REG (DR_REG_INTERRUPT_BASE + 0x148)

/* INTERRUPT_PRO_PMS_DMA_TX_I_ILG_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map PMS_DMA_TX_I_ILG interrupt signal to one of
 * the CPU interrupts.
 */

#define INTERRUPT_PRO_PMS_DMA_TX_I_ILG_INTR_MAP    0x0000001F
#define INTERRUPT_PRO_PMS_DMA_TX_I_ILG_INTR_MAP_M  (INTERRUPT_PRO_PMS_DMA_TX_I_ILG_INTR_MAP_V << INTERRUPT_PRO_PMS_DMA_TX_I_ILG_INTR_MAP_S)
#define INTERRUPT_PRO_PMS_DMA_TX_I_ILG_INTR_MAP_V  0x0000001F
#define INTERRUPT_PRO_PMS_DMA_TX_I_ILG_INTR_MAP_S  0

/* INTERRUPT_PRO_SPI_MEM_REJECT_INTR_MAP_REG register
 * SPI_MEM_REJECT_INTR interrupt configuration register
 */

#define INTERRUPT_PRO_SPI_MEM_REJECT_INTR_MAP_REG (DR_REG_INTERRUPT_BASE + 0x14c)

/* INTERRUPT_PRO_SPI_MEM_REJECT_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map SPI_MEM_REJECT_INTR interrupt signal to one
 * of the CPU interrupts.
 */

#define INTERRUPT_PRO_SPI_MEM_REJECT_INTR_MAP    0x0000001F
#define INTERRUPT_PRO_SPI_MEM_REJECT_INTR_MAP_M  (INTERRUPT_PRO_SPI_MEM_REJECT_INTR_MAP_V << INTERRUPT_PRO_SPI_MEM_REJECT_INTR_MAP_S)
#define INTERRUPT_PRO_SPI_MEM_REJECT_INTR_MAP_V  0x0000001F
#define INTERRUPT_PRO_SPI_MEM_REJECT_INTR_MAP_S  0

/* INTERRUPT_PRO_DMA_COPY_INTR_MAP_REG register
 * DMA_COPY_INTR interrupt configuration register
 */

#define INTERRUPT_PRO_DMA_COPY_INTR_MAP_REG (DR_REG_INTERRUPT_BASE + 0x150)

/* INTERRUPT_PRO_DMA_COPY_INTR_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map DMA_COPY_INTR interrupt signal to one of the
 * CPU interrupts.
 */

#define INTERRUPT_PRO_DMA_COPY_INTR_MAP    0x0000001F
#define INTERRUPT_PRO_DMA_COPY_INTR_MAP_M  (INTERRUPT_PRO_DMA_COPY_INTR_MAP_V << INTERRUPT_PRO_DMA_COPY_INTR_MAP_S)
#define INTERRUPT_PRO_DMA_COPY_INTR_MAP_V  0x0000001F
#define INTERRUPT_PRO_DMA_COPY_INTR_MAP_S  0

/* INTERRUPT_PRO_SPI4_DMA_INT_MAP_REG register
 * SPI4_DMA_INT interrupt configuration register
 */

#define INTERRUPT_PRO_SPI4_DMA_INT_MAP_REG (DR_REG_INTERRUPT_BASE + 0x154)

/* INTERRUPT_PRO_SPI4_DMA_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map SPI4_DMA_INT interrupt signal to one of the
 * CPU interrupts.
 */

#define INTERRUPT_PRO_SPI4_DMA_INT_MAP    0x0000001F
#define INTERRUPT_PRO_SPI4_DMA_INT_MAP_M  (INTERRUPT_PRO_SPI4_DMA_INT_MAP_V << INTERRUPT_PRO_SPI4_DMA_INT_MAP_S)
#define INTERRUPT_PRO_SPI4_DMA_INT_MAP_V  0x0000001F
#define INTERRUPT_PRO_SPI4_DMA_INT_MAP_S  0

/* INTERRUPT_PRO_SPI_INTR_4_MAP_REG register
 * SPI_INTR_4 interrupt configuration register
 */

#define INTERRUPT_PRO_SPI_INTR_4_MAP_REG (DR_REG_INTERRUPT_BASE + 0x158)

/* INTERRUPT_PRO_SPI_INTR_4_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map SPI_INTR_4 interrupt signal to one of the
 * CPU interrupts.
 */

#define INTERRUPT_PRO_SPI_INTR_4_MAP    0x0000001F
#define INTERRUPT_PRO_SPI_INTR_4_MAP_M  (INTERRUPT_PRO_SPI_INTR_4_MAP_V << INTERRUPT_PRO_SPI_INTR_4_MAP_S)
#define INTERRUPT_PRO_SPI_INTR_4_MAP_V  0x0000001F
#define INTERRUPT_PRO_SPI_INTR_4_MAP_S  0

/* INTERRUPT_PRO_DCACHE_PRELOAD_INT_MAP_REG register
 * DCACHE_PRELOAD_INT interrupt configuration register
 */

#define INTERRUPT_PRO_DCACHE_PRELOAD_INT_MAP_REG (DR_REG_INTERRUPT_BASE + 0x15c)

/* INTERRUPT_PRO_DCACHE_PRELOAD_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map DCACHE_PRELOAD_INT interrupt signal to one
 * of the CPU interrupts.
 */

#define INTERRUPT_PRO_DCACHE_PRELOAD_INT_MAP    0x0000001F
#define INTERRUPT_PRO_DCACHE_PRELOAD_INT_MAP_M  (INTERRUPT_PRO_DCACHE_PRELOAD_INT_MAP_V << INTERRUPT_PRO_DCACHE_PRELOAD_INT_MAP_S)
#define INTERRUPT_PRO_DCACHE_PRELOAD_INT_MAP_V  0x0000001F
#define INTERRUPT_PRO_DCACHE_PRELOAD_INT_MAP_S  0

/* INTERRUPT_PRO_ICACHE_PRELOAD_INT_MAP_REG register
 * ICACHE_PRELOAD_INT interrupt configuration register
 */

#define INTERRUPT_PRO_ICACHE_PRELOAD_INT_MAP_REG (DR_REG_INTERRUPT_BASE + 0x160)

/* INTERRUPT_PRO_ICACHE_PRELOAD_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map ICACHE_PRELOAD_INT interrupt signal to one
 * of the CPU interrupts.
 */

#define INTERRUPT_PRO_ICACHE_PRELOAD_INT_MAP    0x0000001F
#define INTERRUPT_PRO_ICACHE_PRELOAD_INT_MAP_M  (INTERRUPT_PRO_ICACHE_PRELOAD_INT_MAP_V << INTERRUPT_PRO_ICACHE_PRELOAD_INT_MAP_S)
#define INTERRUPT_PRO_ICACHE_PRELOAD_INT_MAP_V  0x0000001F
#define INTERRUPT_PRO_ICACHE_PRELOAD_INT_MAP_S  0

/* INTERRUPT_PRO_APB_ADC_INT_MAP_REG register
 * APB_ADC_INT interrupt configuration register
 */

#define INTERRUPT_PRO_APB_ADC_INT_MAP_REG (DR_REG_INTERRUPT_BASE + 0x164)

/* INTERRUPT_PRO_APB_ADC_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map APB_ADC_INT interrupt signal to one of the
 * CPU interrupts.
 */

#define INTERRUPT_PRO_APB_ADC_INT_MAP    0x0000001F
#define INTERRUPT_PRO_APB_ADC_INT_MAP_M  (INTERRUPT_PRO_APB_ADC_INT_MAP_V << INTERRUPT_PRO_APB_ADC_INT_MAP_S)
#define INTERRUPT_PRO_APB_ADC_INT_MAP_V  0x0000001F
#define INTERRUPT_PRO_APB_ADC_INT_MAP_S  0

/* INTERRUPT_PRO_CRYPTO_DMA_INT_MAP_REG register
 * CRYPTO_DMA_INT interrupt configuration register
 */

#define INTERRUPT_PRO_CRYPTO_DMA_INT_MAP_REG (DR_REG_INTERRUPT_BASE + 0x168)

/* INTERRUPT_PRO_CRYPTO_DMA_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map CRYPTO_DMA_INT interrupt signal to one of
 * the CPU interrupts.
 */

#define INTERRUPT_PRO_CRYPTO_DMA_INT_MAP    0x0000001F
#define INTERRUPT_PRO_CRYPTO_DMA_INT_MAP_M  (INTERRUPT_PRO_CRYPTO_DMA_INT_MAP_V << INTERRUPT_PRO_CRYPTO_DMA_INT_MAP_S)
#define INTERRUPT_PRO_CRYPTO_DMA_INT_MAP_V  0x0000001F
#define INTERRUPT_PRO_CRYPTO_DMA_INT_MAP_S  0

/* INTERRUPT_PRO_CPU_PERI_ERROR_INT_MAP_REG register
 * CPU_PERI_ERROR_INT interrupt configuration register
 */

#define INTERRUPT_PRO_CPU_PERI_ERROR_INT_MAP_REG (DR_REG_INTERRUPT_BASE + 0x16c)

/* INTERRUPT_PRO_CPU_PERI_ERROR_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map CPU_PERI_ERROR_INT interrupt signal to one
 * of the CPU interrupts.
 */

#define INTERRUPT_PRO_CPU_PERI_ERROR_INT_MAP    0x0000001F
#define INTERRUPT_PRO_CPU_PERI_ERROR_INT_MAP_M  (INTERRUPT_PRO_CPU_PERI_ERROR_INT_MAP_V << INTERRUPT_PRO_CPU_PERI_ERROR_INT_MAP_S)
#define INTERRUPT_PRO_CPU_PERI_ERROR_INT_MAP_V  0x0000001F
#define INTERRUPT_PRO_CPU_PERI_ERROR_INT_MAP_S  0

/* INTERRUPT_PRO_APB_PERI_ERROR_INT_MAP_REG register
 * APB_PERI_ERROR_INT interrupt configuration register
 */

#define INTERRUPT_PRO_APB_PERI_ERROR_INT_MAP_REG (DR_REG_INTERRUPT_BASE + 0x170)

/* INTERRUPT_PRO_APB_PERI_ERROR_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map APB_PERI_ERROR_INT interrupt signal to one
 * of the CPU interrupts.
 */

#define INTERRUPT_PRO_APB_PERI_ERROR_INT_MAP    0x0000001F
#define INTERRUPT_PRO_APB_PERI_ERROR_INT_MAP_M  (INTERRUPT_PRO_APB_PERI_ERROR_INT_MAP_V << INTERRUPT_PRO_APB_PERI_ERROR_INT_MAP_S)
#define INTERRUPT_PRO_APB_PERI_ERROR_INT_MAP_V  0x0000001F
#define INTERRUPT_PRO_APB_PERI_ERROR_INT_MAP_S  0

/* INTERRUPT_PRO_DCACHE_SYNC_INT_MAP_REG register
 * DCACHE_SYNC_INT interrupt configuration register
 */

#define INTERRUPT_PRO_DCACHE_SYNC_INT_MAP_REG (DR_REG_INTERRUPT_BASE + 0x174)

/* INTERRUPT_PRO_DCACHE_SYNC_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map DCACHE_SYNC_INT interrupt signal to one of
 * the CPU interrupts.
 */

#define INTERRUPT_PRO_DCACHE_SYNC_INT_MAP    0x0000001F
#define INTERRUPT_PRO_DCACHE_SYNC_INT_MAP_M  (INTERRUPT_PRO_DCACHE_SYNC_INT_MAP_V << INTERRUPT_PRO_DCACHE_SYNC_INT_MAP_S)
#define INTERRUPT_PRO_DCACHE_SYNC_INT_MAP_V  0x0000001F
#define INTERRUPT_PRO_DCACHE_SYNC_INT_MAP_S  0

/* INTERRUPT_PRO_ICACHE_SYNC_INT_MAP_REG register
 * ICACHE_SYNC_INT interrupt configuration register
 */

#define INTERRUPT_PRO_ICACHE_SYNC_INT_MAP_REG (DR_REG_INTERRUPT_BASE + 0x178)

/* INTERRUPT_PRO_ICACHE_SYNC_INT_MAP : R/W; bitpos: [4:0]; default: 16;
 * This register is used to map ICACHE_SYNC_INT interrupt signal to one of
 * the CPU interrupts.
 */

#define INTERRUPT_PRO_ICACHE_SYNC_INT_MAP    0x0000001F
#define INTERRUPT_PRO_ICACHE_SYNC_INT_MAP_M  (INTERRUPT_PRO_ICACHE_SYNC_INT_MAP_V << INTERRUPT_PRO_ICACHE_SYNC_INT_MAP_S)
#define INTERRUPT_PRO_ICACHE_SYNC_INT_MAP_V  0x0000001F
#define INTERRUPT_PRO_ICACHE_SYNC_INT_MAP_S  0

/* INTERRUPT_PRO_INTR_STATUS_REG_0_REG register
 * Interrupt status register 0
 */

#define INTERRUPT_PRO_INTR_STATUS_REG_0_REG (DR_REG_INTERRUPT_BASE + 0x17c)

/* INTERRUPT_PRO_INTR_STATUS_0 : RO; bitpos: [31:0]; default: 0;
 * This register stores the status of the first 32 input interrupt sources.
 */

#define INTERRUPT_PRO_INTR_STATUS_0    0xFFFFFFFF
#define INTERRUPT_PRO_INTR_STATUS_0_M  (INTERRUPT_PRO_INTR_STATUS_0_V << INTERRUPT_PRO_INTR_STATUS_0_S)
#define INTERRUPT_PRO_INTR_STATUS_0_V  0xFFFFFFFF
#define INTERRUPT_PRO_INTR_STATUS_0_S  0

/* INTERRUPT_PRO_INTR_STATUS_REG_1_REG register
 * Interrupt status register 1
 */

#define INTERRUPT_PRO_INTR_STATUS_REG_1_REG (DR_REG_INTERRUPT_BASE + 0x180)

/* INTERRUPT_PRO_INTR_STATUS_1 : RO; bitpos: [31:0]; default: 0;
 * This register stores the status of the second 32 input interrupt sources.
 */

#define INTERRUPT_PRO_INTR_STATUS_1    0xFFFFFFFF
#define INTERRUPT_PRO_INTR_STATUS_1_M  (INTERRUPT_PRO_INTR_STATUS_1_V << INTERRUPT_PRO_INTR_STATUS_1_S)
#define INTERRUPT_PRO_INTR_STATUS_1_V  0xFFFFFFFF
#define INTERRUPT_PRO_INTR_STATUS_1_S  0

/* INTERRUPT_PRO_INTR_STATUS_REG_2_REG register
 * Interrupt status register 2
 */

#define INTERRUPT_PRO_INTR_STATUS_REG_2_REG (DR_REG_INTERRUPT_BASE + 0x184)

/* INTERRUPT_PRO_INTR_STATUS_2 : RO; bitpos: [31:0]; default: 0;
 * This register stores the status of the last 31 input interrupt sources.
 */

#define INTERRUPT_PRO_INTR_STATUS_2    0xFFFFFFFF
#define INTERRUPT_PRO_INTR_STATUS_2_M  (INTERRUPT_PRO_INTR_STATUS_2_V << INTERRUPT_PRO_INTR_STATUS_2_S)
#define INTERRUPT_PRO_INTR_STATUS_2_V  0xFFFFFFFF
#define INTERRUPT_PRO_INTR_STATUS_2_S  0

/* INTERRUPT_CLOCK_GATE_REG register
 * NMI interrupt signals mask register
 */

#define INTERRUPT_CLOCK_GATE_REG (DR_REG_INTERRUPT_BASE + 0x188)

/* INTERRUPT_PRO_NMI_MASK_HW : R/W; bitpos: [1]; default: 0;
 * This bit is used to disable all NMI interrupt signals to CPU.
 */

#define INTERRUPT_PRO_NMI_MASK_HW    (BIT(1))
#define INTERRUPT_PRO_NMI_MASK_HW_M  (INTERRUPT_PRO_NMI_MASK_HW_V << INTERRUPT_PRO_NMI_MASK_HW_S)
#define INTERRUPT_PRO_NMI_MASK_HW_V  0x00000001
#define INTERRUPT_PRO_NMI_MASK_HW_S  1

/* INTERRUPT_CLK_EN : R/W; bitpos: [0]; default: 1;
 * This bit is used to enable or disable the clock of interrupt matrix. 1:
 * enable the clock. 0: disable the clock.
 */

#define INTERRUPT_CLK_EN    (BIT(0))
#define INTERRUPT_CLK_EN_M  (INTERRUPT_CLK_EN_V << INTERRUPT_CLK_EN_S)
#define INTERRUPT_CLK_EN_V  0x00000001
#define INTERRUPT_CLK_EN_S  0

/* INTERRUPT_REG_DATE_REG register
 * Version control register
 */

#define INTERRUPT_REG_DATE_REG (DR_REG_INTERRUPT_BASE + 0xffc)

/* INTERRUPT_INTERRUPT_REG_DATE : R/W; bitpos: [27:0]; default: 26231168;
 * This is the version register.
 */

#define INTERRUPT_INTERRUPT_REG_DATE    0x0FFFFFFF
#define INTERRUPT_INTERRUPT_REG_DATE_M  (INTERRUPT_INTERRUPT_REG_DATE_V << INTERRUPT_INTERRUPT_REG_DATE_S)
#define INTERRUPT_INTERRUPT_REG_DATE_V  0x0FFFFFFF
#define INTERRUPT_INTERRUPT_REG_DATE_S  0

#endif /* __ARCH_XTENSA_SRC_ESP32S2_HARDWARE_ESP32S2_INTERRUPT_H */
