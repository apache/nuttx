/****************************************************************************
 * arch/risc-v/src/qemu-rv32/qemu_rv32_memorymap.h
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

#ifndef __ARCH_RISCV_SRC_QEMU_RV32_QEMU_RV32_MEMORYMAP_H
#define __ARCH_RISCV_SRC_QEMU_RV32_QEMU_RV32_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Idle thread stack starts from _ebss */

#ifndef __ASSEMBLY__
#define QEMU_RV32_IDLESTACK_BASE  (uint32_t)&_ebss
#else
#define QEMU_RV32_IDLESTACK_BASE  _ebss
#endif

#define QEMU_RV32_IDLESTACK_SIZE (CONFIG_IDLETHREAD_STACKSIZE & ~3)
#define QEMU_RV32_IDLESTACK_TOP  (QEMU_RV32_IDLESTACK_BASE + QEMU_RV32_IDLESTACK_SIZE)

#endif /* __ARCH_RISCV_SRC_QEMU_RV32_QEMU_RV32_MEMORYMAP_H */
