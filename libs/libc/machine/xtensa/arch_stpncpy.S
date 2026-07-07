/****************************************************************************
 * libs/libc/machine/xtensa/arch_stpncpy.S
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "xtensa_asm.h"

#include <arch/chip/core-isa.h>
#include <arch/xtensa/xtensa_abi.h>

#include "libc.h"

#ifdef LIBC_BUILD_STPNCPY

/****************************************************************************
 * Public Functions
 ****************************************************************************/

  .section .text
  .begin schedule
  .align  4
  .literal_position
  .global ARCH_LIBCFUN(stpncpy)
  .type ARCH_LIBCFUN(stpncpy), @function
ARCH_LIBCFUN(stpncpy):
  ENTRY(16)
  /* a2 = dst, a3 = src, a4 = n */
  /* Returns pointer to the byte after the last non-null byte written,
     or dst+n if src has n or more characters. */

  /* If n == 0, return dst */

  beqz  a4, .Lret

  /* Copy up to n bytes from src, stopping at null */

.Lcopy_loop:
  l8ui  a8, a3, 0
  s8i a8, a2, 0
  beqz  a8, .Lfill_zero
  addi  a2, a2, 1
  addi  a3, a3, 1
  addi  a4, a4, -1
  bnez  a4, .Lcopy_loop

  /* Copied n bytes without hitting null. Return dst+n (current a2). */

  RET(16)

  /* Hit null in src. a2 points to the null written.
     Save return value, then zero-fill remaining. */

.Lfill_zero:
  mov a10, a2         /* return value = pointer to first null written */
  addi  a4, a4, -1   /* we already wrote the null, decrement n */
  beqz  a4, .Lret_a10

  /* Zero-fill remaining bytes */

  addi  a2, a2, 1
  movi  a8, 0
.Lfill_loop:
  s8i a8, a2, 0
  addi  a2, a2, 1
  addi  a4, a4, -1
  bnez  a4, .Lfill_loop

.Lret_a10:
  mov a2, a10
  RET(16)

.Lret:
  RET(16)

  .end schedule

  .size ARCH_LIBCFUN(stpncpy), . - ARCH_LIBCFUN(stpncpy)

#endif
