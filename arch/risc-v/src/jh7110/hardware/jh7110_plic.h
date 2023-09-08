/****************************************************************************
 * arch/risc-v/src/jh7110/hardware/jh7110_plic.h
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

#ifndef __ARCH_RISCV_SRC_JH7110_HARDWARE_JH7110_PLIC_H
#define __ARCH_RISCV_SRC_JH7110_HARDWARE_JH7110_PLIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Interrupt Priority */

#define JH7110_PLIC_PRIORITY  (JH7110_PLIC_BASE + 0x000000)

/* Hart 1 S-Mode Interrupt Enable */

#define JH7110_PLIC_ENABLE1   (JH7110_PLIC_BASE + 0x002100)
#define JH7110_PLIC_ENABLE2   (JH7110_PLIC_BASE + 0x002104)

/* Hart 1 S-Mode Priority Threshold */

#define JH7110_PLIC_THRESHOLD (JH7110_PLIC_BASE + 0x202000)

/* Hart 1 S-Mode Claim / Complete */

#define JH7110_PLIC_CLAIM     (JH7110_PLIC_BASE + 0x202004)

#endif /* __ARCH_RISCV_SRC_JH7110_HARDWARE_JH7110_PLIC_H */
