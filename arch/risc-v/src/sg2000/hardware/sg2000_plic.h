/****************************************************************************
 * arch/risc-v/src/sg2000/hardware/sg2000_plic.h
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

#ifndef __ARCH_RISCV_SRC_SG2000_HARDWARE_SG2000_PLIC_H
#define __ARCH_RISCV_SRC_SG2000_HARDWARE_SG2000_PLIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Interrupt Priority */

#define SG2000_PLIC_PRIORITY  (SG2000_PLIC_BASE + 0x000000)

/* Hart 0 S-Mode Interrupt Enable */

#define SG2000_PLIC_ENABLE1   (SG2000_PLIC_BASE + 0x002080)
#define SG2000_PLIC_ENABLE2   (SG2000_PLIC_BASE + 0x002084)

/* Hart 0 S-Mode Priority Threshold */

#define SG2000_PLIC_THRESHOLD (SG2000_PLIC_BASE + 0x201000)

/* Hart 0 S-Mode Claim / Complete */

#define SG2000_PLIC_CLAIM     (SG2000_PLIC_BASE + 0x201004)

#endif /* __ARCH_RISCV_SRC_SG2000_HARDWARE_SG2000_PLIC_H */
