/****************************************************************************
 * arch/risc-v/src/k230/hardware/k230_plic.h
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

#ifndef __ARCH_RISCV_SRC_K230_HARDWARE_K230_PLIC_H
#define __ARCH_RISCV_SRC_K230_HARDWARE_K230_PLIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define K230_PLIC_PRIORITY    (K230_PLIC_BASE + 0x000000)
#define K230_PLIC_PENDING1    (K230_PLIC_BASE + 0x001000)

#ifdef CONFIG_ARCH_USE_S_MODE
#  define K230_PLIC_ENABLE1   (K230_PLIC_BASE + 0x002080)
#  define K230_PLIC_ENABLE2   (K230_PLIC_BASE + 0x002084)
#  define K230_PLIC_THRESHOLD (K230_PLIC_BASE + 0x201000)
#  define K230_PLIC_CLAIM     (K230_PLIC_BASE + 0x201004)
#else
#  define K230_PLIC_ENABLE1   (K230_PLIC_BASE + 0x002000)
#  define K230_PLIC_ENABLE2   (K230_PLIC_BASE + 0x002004)
#  define K230_PLIC_THRESHOLD (K230_PLIC_BASE + 0x200000)
#  define K230_PLIC_CLAIM     (K230_PLIC_BASE + 0x200004)
#endif

#endif /* __ARCH_RISCV_SRC_K230_HARDWARE_K230_PLIC_H */
