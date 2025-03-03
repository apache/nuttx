/****************************************************************************
 * arch/risc-v/src/eic7700x/hardware/eic7700x_plic.h
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

#ifndef __ARCH_RISCV_SRC_EIC7700X_HARDWARE_EIC7700X_PLIC_H
#define __ARCH_RISCV_SRC_EIC7700X_HARDWARE_EIC7700X_PLIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Interrupt Priority */

#define EIC7700X_PLIC_PRIORITY (EIC7700X_PLIC_BASE + 0x000000)

/* Hart 0 S-Mode Interrupt Enable */

#define EIC7700X_PLIC_ENABLE0     (EIC7700X_PLIC_BASE + 0x002080)
#define EIC7700X_PLIC_ENABLE_HART 0x100

/* Hart 0 S-Mode Priority Threshold */

#define EIC7700X_PLIC_THRESHOLD0     (EIC7700X_PLIC_BASE + 0x201000)
#define EIC7700X_PLIC_THRESHOLD_HART 0x2000

/* Hart 0 S-Mode Claim / Complete */

#define EIC7700X_PLIC_CLAIM0     (EIC7700X_PLIC_BASE + 0x201004)
#define EIC7700X_PLIC_CLAIM_HART 0x2000

#endif /* __ARCH_RISCV_SRC_EIC7700X_HARDWARE_EIC7700X_PLIC_H */
