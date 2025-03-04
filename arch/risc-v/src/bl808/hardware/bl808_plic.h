/****************************************************************************
 * arch/risc-v/src/bl808/hardware/bl808_plic.h
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

#ifndef __ARCH_RISCV_SRC_BL808_HARDWARE_BL808_PLIC_H
#define __ARCH_RISCV_SRC_BL808_HARDWARE_BL808_PLIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Interrupt Priority */

#define BL808_PLIC_PRIORITY  (BL808_PLIC_BASE + 0x000000)

/* Hart 0 S-Mode Interrupt Enable */

#define BL808_PLIC_ENABLE1   (BL808_PLIC_BASE + 0x002080)
#define BL808_PLIC_ENABLE2   (BL808_PLIC_BASE + 0x002084)

/* Hart 0 S-Mode Priority Threshold */

#define BL808_PLIC_THRESHOLD (BL808_PLIC_BASE + 0x201000)

/* Hart 0 S-Mode Claim / Complete */

#define BL808_PLIC_CLAIM     (BL808_PLIC_BASE + 0x201004)

#endif /* __ARCH_RISCV_SRC_BL808_HARDWARE_BL808_PLIC_H */
