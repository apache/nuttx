/****************************************************************************
 * arch/risc-v/src/hpm6000/hardware/hpm_ioc.h
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

#ifndef __ARCH_RISCV_SRC_HPM6000_HARDWARE_HPM_IOC_H
#define __ARCH_RISCV_SRC_HPM6000_HARDWARE_HPM_IOC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hpm_ioc.h"

#define CONFIG_ARCH_FAMILY_HPM6300

#if defined (CONFIG_ARCH_FAMILY_HPM6300)
#  include "hardware/hpm6300/hpm6300_ioc.h"
#else
#  error Unrecognized HPM chip
#endif

#define DRIVE_260OHM                        (1)
#define DRIVE_130OHM                        (2)
#define DRIVE_88OHM                         (3)
#define DRIVE_65OHM                         (4)
#define DRIVE_52OHM                         (5)
#define DRIVE_43OHM                         (6)
#define DRIVE_37OHM                         (7)

#define SPEED_SLOW                          (0)
#define SPEED_MEDIUM                        (1)
#define SPEED_FAST                          (2)
#define SPEED_MAX                           (3)

#define PULL_DOWN_100K                      (0)
#define PULL_UP_100K                        (1)
#define PULL_UP_47K                         (2)
#define PULL_UP_22K                         (3)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Pad Alt Registers */

#define IOC_PAD_FUNC_ALT_SELECT_SHIFT       (0)
#define IOC_PAD_FUNC_ALT_SELECT_MASK        (1f << IOC_PAD_FUNC_ALT_SELECT_SHIFT)
#  define IOC_PAD_FUNC_ALT_SELECT(n)        ((uint32_t)(n) << IOC_PAD_FUNC_ALT_SELECT_SHIFT)
#  define IOC_PAD_FUNC_ALT_SELECT_ALT0      (0 << IOC_PAD_FUNC_ALT_SELECT_SHIFT)
#  define IOC_PAD_FUNC_ALT_SELECT_ALT1      (1 << IOC_PAD_FUNC_ALT_SELECT_SHIFT)
#  define IOC_PAD_FUNC_ALT_SELECT_ALT2      (2 << IOC_PAD_FUNC_ALT_SELECT_SHIFT)
#  define IOC_PAD_FUNC_ALT_SELECT_ALT3      (3 << IOC_PAD_FUNC_ALT_SELECT_SHIFT)
#  define IOC_PAD_FUNC_ALT_SELECT_ALT4      (4 << IOC_PAD_FUNC_ALT_SELECT_SHIFT)
#  define IOC_PAD_FUNC_ALT_SELECT_ALT5      (5 << IOC_PAD_FUNC_ALT_SELECT_SHIFT)
#  define IOC_PAD_FUNC_ALT_SELECT_ALT6      (6 << IOC_PAD_FUNC_ALT_SELECT_SHIFT)
#  define IOC_PAD_FUNC_ALT_SELECT_ALT7      (7 << IOC_PAD_FUNC_ALT_SELECT_SHIFT)
#  define IOC_PAD_FUNC_ALT_SELECT_ALT8      (8 << IOC_PAD_FUNC_ALT_SELECT_SHIFT)
#  define IOC_PAD_FUNC_ALT_SELECT_ALT9      (9 << IOC_PAD_FUNC_ALT_SELECT_SHIFT)
#  define IOC_PAD_FUNC_ALT_SELECT_ALT10     (10 << IOC_PAD_FUNC_ALT_SELECT_SHIFT)
#  define IOC_PAD_FUNC_ALT_SELECT_ALT11     (11 << IOC_PAD_FUNC_ALT_SELECT_SHIFT)
#  define IOC_PAD_FUNC_ALT_SELECT_ALT12     (12 << IOC_PAD_FUNC_ALT_SELECT_SHIFT)
#  define IOC_PAD_FUNC_ALT_SELECT_ALT13     (13 << IOC_PAD_FUNC_ALT_SELECT_SHIFT)
#  define IOC_PAD_FUNC_ALT_SELECT_ALT14     (14 << IOC_PAD_FUNC_ALT_SELECT_SHIFT)
#  define IOC_PAD_FUNC_ALT_SELECT_ALT15     (15 << IOC_PAD_FUNC_ALT_SELECT_SHIFT)
#  define IOC_PAD_FUNC_ALT_SELECT_ALT16     (16 << IOC_PAD_FUNC_ALT_SELECT_SHIFT)
#  define IOC_PAD_FUNC_ALT_SELECT_ALT17     (17 << IOC_PAD_FUNC_ALT_SELECT_SHIFT)
#  define IOC_PAD_FUNC_ALT_SELECT_ALT18     (18 << IOC_PAD_FUNC_ALT_SELECT_SHIFT)
#  define IOC_PAD_FUNC_ALT_SELECT_ALT19     (19 << IOC_PAD_FUNC_ALT_SELECT_SHIFT)
#  define IOC_PAD_FUNC_ALT_SELECT_ALT20     (20 << IOC_PAD_FUNC_ALT_SELECT_SHIFT)
#  define IOC_PAD_FUNC_ALT_SELECT_ALT21     (21 << IOC_PAD_FUNC_ALT_SELECT_SHIFT)
#  define IOC_PAD_FUNC_ALT_SELECT_ALT22     (22 << IOC_PAD_FUNC_ALT_SELECT_SHIFT)
#  define IOC_PAD_FUNC_ALT_SELECT_ALT23     (23 << IOC_PAD_FUNC_ALT_SELECT_SHIFT)
#  define IOC_PAD_FUNC_ALT_SELECT_ALT24     (24 << IOC_PAD_FUNC_ALT_SELECT_SHIFT)
#  define IOC_PAD_FUNC_ALT_SELECT_ALT25     (25 << IOC_PAD_FUNC_ALT_SELECT_SHIFT)
#  define IOC_PAD_FUNC_ALT_SELECT_ALT26     (26 << IOC_PAD_FUNC_ALT_SELECT_SHIFT)
#  define IOC_PAD_FUNC_ALT_SELECT_ALT27     (27 << IOC_PAD_FUNC_ALT_SELECT_SHIFT)
#  define IOC_PAD_FUNC_ALT_SELECT_ALT28     (28 << IOC_PAD_FUNC_ALT_SELECT_SHIFT)
#  define IOC_PAD_FUNC_ALT_SELECT_ALT29     (29 << IOC_PAD_FUNC_ALT_SELECT_SHIFT)
#  define IOC_PAD_FUNC_ALT_SELECT_ALT30     (30 << IOC_PAD_FUNC_ALT_SELECT_SHIFT)
#  define IOC_PAD_FUNC_ALT_SELECT_ALT31     (31 << IOC_PAD_FUNC_ALT_SELECT_SHIFT)

/* Pad Analog Registers */

#define IOC_PAD_FUNC_ANALOG                 (1 << 8)

/* Pad Loop Back Registers */

#define IOC_PAD_FUNC_LOOP_BACK              (1 << 16)

/* Pad Drive strength Registers */

#define IOC_PAD_PAD_DS_SHIFT                (0)
#define IOC_PAD_PAD_DS_MASK                 (0x7 << IOC_PAD_PAD_DS_SHIFT)
#  define IOC_PAD_PAD_DS(n)                 ((uint32_t)(n) << IOC_PAD_PAD_DS_SHIFT)
#  define IOC_PAD_PAD_DS_1V8_260OHM         (1 << IOC_PAD_PAD_DS_SHIFT)
#  define IOC_PAD_PAD_DS_1V8_130OHM         (2 << IOC_PAD_PAD_DS_SHIFT)
#  define IOC_PAD_PAD_DS_1V8_88OHM          (3 << IOC_PAD_PAD_DS_SHIFT)
#  define IOC_PAD_PAD_DS_1V8_65OHM          (4 << IOC_PAD_PAD_DS_SHIFT)
#  define IOC_PAD_PAD_DS_1V8_52OHM          (5 << IOC_PAD_PAD_DS_SHIFT)
#  define IOC_PAD_PAD_DS_1V8_43OHM          (6 << IOC_PAD_PAD_DS_SHIFT)
#  define IOC_PAD_PAD_DS_1V8_37OHM          (7 << IOC_PAD_PAD_DS_SHIFT)
#  define IOC_PAD_PAD_DS_3V3_157OHM         (1 << IOC_PAD_PAD_DS_SHIFT)
#  define IOC_PAD_PAD_DS_3V3_78OHM          (2 << IOC_PAD_PAD_DS_SHIFT)
#  define IOC_PAD_PAD_DS_3V3_53OHM          (3 << IOC_PAD_PAD_DS_SHIFT)
#  define IOC_PAD_PAD_DS_3V3_39OHM          (4 << IOC_PAD_PAD_DS_SHIFT)
#  define IOC_PAD_PAD_DS_3V3_32OHM          (5 << IOC_PAD_PAD_DS_SHIFT)
#  define IOC_PAD_PAD_DS_3V3_26OHM          (6 << IOC_PAD_PAD_DS_SHIFT)
#  define IOC_PAD_PAD_DS_3V3_23OHM          (7 << IOC_PAD_PAD_DS_SHIFT)

#define IOC_PAD_PAD_SPD_SHIFT               (4)
#define IOC_PAD_PAD_SPD_MASK                (0x3 << IOC_PAD_PAD_SPD_SHIFT)
#  define IOC_PAD_PAD_SPD(n)                ((uint32_t)(n) << IOC_PAD_PAD_SPD_SHIFT)
#  define IOC_PAD_PAD_SPD_SLOW              (0 << IOC_PAD_PAD_SPD_SHIFT)
#  define IOC_PAD_PAD_SPD_MEDIUM            (1 << IOC_PAD_PAD_SPD_SHIFT)
#  define IOC_PAD_PAD_SPD_FAST              (2 << IOC_PAD_PAD_SPD_SHIFT)
#  define IOC_PAD_PAD_SPD_MAX               (3 << IOC_PAD_PAD_SPD_SHIFT)

#define IOC_PAD_PAD_SR                      (1 << 6)
#define IOC_PAD_PAD_OD                      (1 << 8)
#define IOC_PAD_PAD_KE                      (1 << 16)
#define IOC_PAD_PAD_PE                      (1 << 17)
#define IOC_PAD_PAD_PS                      (1 << 18)

#define IOC_PAD_PAD_PRS_SHIFT               (20) /* Bit 20-21: Pull up/down internal resistance strength */
#define IOC_PAD_PAD_PRS_MASK                (0x3 << IOC_PAD_PAD_PRS_SHIFT)
#  define IOC_PAD_PAD_PRS(n)                ((uint32_t)(n) << IOC_PAD_PAD_PRS_SHIFT)
#  define IOC_PAD_PAD_PRS_DOWN_100K         (0 << IOC_PAD_PAD_PRS_SHIFT)
#  define IOC_PAD_PAD_PRS_UP_100K           (0 << IOC_PAD_PAD_PRS_SHIFT)
#  define IOC_PAD_PAD_PRS_UP_47K            (1 << IOC_PAD_PAD_PRS_SHIFT)
#  define IOC_PAD_PAD_PRS_UP_22K            (2 << IOC_PAD_PAD_PRS_SHIFT)

#define IOC_PAD_PAD_HYS                     (1 << 24) /* Bit 24: Schmitt Trigger Enable Field */

/* Defaults for drive conditions for each set of pins. These are a good
 * starting point but should be updated once you've got real hardware
 * to measure.
 */

#define IOC_PAD_UART_DEFAULT                (PAD_PULL_UP_22K | PAD_DRIVE_43OHM | \
                                             PAD_SLEW_SLOW | PAD_SPEED_SLOW | PAD_SCHMITT_TRIGGER)

#endif /* __ARCH_RISCV_SRC_HPM6000_HARDWARE_HPM_IOC_H */
