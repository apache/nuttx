/****************************************************************************
 * libs/libc/machine/risc-v/arch_word.h
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

#ifndef __LIBS_LIBC_MACHINE_RISCV_ARCH_WORD_H
#define __LIBS_LIBC_MACHINE_RISCV_ARCH_WORD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Scanning a register at a time instead of a byte at a time.
 *
 * With ones in every byte's low bit and in every byte's high bit,
 *
 *   (x - ONES) & ~x & HIGHS
 *
 * is nonzero exactly when some byte of x is zero.  Subtracting one from a
 * zero byte borrows into its high bit while ~x keeps only bytes that had
 * no high bit of their own, so only genuinely zero bytes survive the
 * mask.  Finding a particular byte is the same test applied to x XOR a
 * word with that byte repeated.
 *
 * uintptr_t is the native register on both RV32 and RV64, so the same
 * source is four bytes a step on one and eight on the other.
 */

#define WORD_T          uintptr_t
#define WORD_BYTES      sizeof(WORD_T)
#define WORD_ONES       ((WORD_T)-1 / 0xff)          /* 0x0101..01 */
#define WORD_HIGHS      (WORD_ONES << 7)             /* 0x8080..80 */

#define WORD_HASZERO(x) (((x) - WORD_ONES) & ~(x) & WORD_HIGHS)
#define WORD_REPEAT(c)  (WORD_ONES * (unsigned char)(c))
#define WORD_ALIGNED(p) ((((uintptr_t)(p)) & (WORD_BYTES - 1)) == 0)

#endif /* __LIBS_LIBC_MACHINE_RISCV_ARCH_WORD_H */
