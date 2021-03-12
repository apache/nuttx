/****************************************************************************
 * arch/risc-v/src/k210/hardware/k210_plic.h
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

#ifndef __ARCH_RISCV_SRC_K210_HARDWARE_K210_PLIC_H
#define __ARCH_RISCV_SRC_K210_HARDWARE_K210_PLIC_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define K210_PLIC_PRIORITY    (K210_PLIC_BASE + 0x000000)
#define K210_PLIC_PENDING1    (K210_PLIC_BASE + 0x001000)
#define K210_PLIC_ENABLE1     (K210_PLIC_BASE + 0x002000)
#define K210_PLIC_ENABLE2     (K210_PLIC_BASE + 0x002004)
#define K210_PLIC_THRESHOLD   (K210_PLIC_BASE + 0x200000)
#define K210_PLIC_CLAIM       (K210_PLIC_BASE + 0x200004)

#endif /* __ARCH_RISCV_SRC_K210_HARDWARE_K210_PLIC_H */
