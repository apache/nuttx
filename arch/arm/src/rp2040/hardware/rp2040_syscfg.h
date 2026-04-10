/****************************************************************************
 * arch/arm/src/rp2040/hardware/rp2040_syscfg.h
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

#ifndef __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_SYSCFG_H
#define __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_SYSCFG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp2040_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define RP2040_SYSCFG_PROC_CONFIG_OFFSET                  (0x00000008)  /* Configuration for processors. */

/* Register definitions *****************************************************/

#define RP2040_SYSCFG_PROC_CONFIG                         (RP2040_SYSCFG_BASE + RP2040_SYSCFG_PROC_CONFIG_OFFSET)

/* Register bit definitions *************************************************/

#define RP2040_SYSCFG_PROC_CONFIG_MASK                    (0xff000003)
#define RP2040_SYSCFG_PROC_CONFIG_PROC1_DAP_INSTID_SHIFT  (28)
#define RP2040_SYSCFG_PROC_CONFIG_PROC1_DAP_INSTID_MASK   (0xf << RP2040_SYSCFG_PROC_CONFIG_PROC1_DAP_INSTID_SHIFT)
#define RP2040_SYSCFG_PROC_CONFIG_PROC0_DAP_INSTID_SHIFT  (24)
#define RP2040_SYSCFG_PROC_CONFIG_PROC0_DAP_INSTID_MASK   (0xf << RP2040_SYSCFG_PROC_CONFIG_PROC0_DAP_INSTID_SHIFT)

#endif /* __ARCH_ARM_SRC_RP2040_HARDWARE_RP2040_SYSCFG_H */
