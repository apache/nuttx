/****************************************************************************
 * arch/risc-v/src/qemu-rv32/hardware/qemu_rv32_plic.h
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

#ifndef __ARCH_RISCV_SRC_FE310_HARDWARE_FE310_PLIC_H
#define __ARCH_RISCV_SRC_FE310_HARDWARE_FE310_PLIC_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define QEMU_RV32_PLIC_PRIORITY    (QEMU_RV32_PLIC_BASE + 0x000000)
#define QEMU_RV32_PLIC_PENDING1    (QEMU_RV32_PLIC_BASE + 0x001000)
#define QEMU_RV32_PLIC_ENABLE1     (QEMU_RV32_PLIC_BASE + 0x002000)
#define QEMU_RV32_PLIC_ENABLE2     (QEMU_RV32_PLIC_BASE + 0x002004)
#define QEMU_RV32_PLIC_THRESHOLD   (QEMU_RV32_PLIC_BASE + 0x200000)
#define QEMU_RV32_PLIC_CLAIM       (QEMU_RV32_PLIC_BASE + 0x200004)

#endif /* __ARCH_RISCV_SRC_FE310_HARDWARE_FE310_PLIC_H */
