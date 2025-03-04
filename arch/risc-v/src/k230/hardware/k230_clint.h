/****************************************************************************
 * arch/risc-v/src/k230/hardware/k230_clint.h
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

#ifndef __ARCH_RISCV_SRC_K230_HARDWARE_K230_CLINT_H
#define __ARCH_RISCV_SRC_K230_HARDWARE_K230_CLINT_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define K230_CLINT_MSIP      (K230_CLINT_BASE + 0x0000)
#define K230_CLINT_MTIMECMP  (K230_CLINT_BASE + 0x4000)
#define K230_CLINT_SSIP      (K230_CLINT_BASE + 0xC000)
#define K230_CLINT_STIMECMP  (K230_CLINT_BASE + 0xD000)
#define K230_CLINT_MTIME     (K230_CLINT_BASE + 0xBFF8)
#define K230_CLINT_STIME     (K230_CLINT_BASE + 0xBFF8)

#define K230_CLINT_FREQ      (27000000)

#ifdef CONFIG_ARCH_USE_S_MODE
#  define K230_IPI           K230_CLINT_SSIP
#  define K230_TIME          K230_CLINT_STIME
#  define K230_TIMECMP       K230_CLINT_STIMECMP
#else
#  define K230_IPI           K230_CLINT_MSIP
#  define K230_TIME          K230_CLINT_MTIME
#  define K230_TIMECMP       K230_CLINT_MTIMECMP
#endif

#define RISCV_IPI            K230_IPI

#endif /* __ARCH_RISCV_SRC_K230_HARDWARE_K230_CLINT_H */
