/****************************************************************************
 * libs/libc/machine/arm/arch_umulh64.c
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

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* UMULH: high 64 bits of the 128-bit product a * b, the ARM form of
 * up_umulh64() declared in include/nuttx/lib/math32.h.  UMULL exists in
 * the ARM and Thumb-2 instruction sets only; a Thumb-1 target keeps the
 * portable fallback from libs/libc/machine/up_umulh64.c instead.
 */

#if !defined(__thumb__) || defined(__thumb2__)

uint64_t up_umulh64(uint64_t a, uint64_t b)
{
  uint32_t al = (uint32_t)a;
  uint32_t ah = (uint32_t)(a >> 32);
  uint32_t bl = (uint32_t)b;
  uint32_t bh = (uint32_t)(b >> 32);
  register uint32_t rlo asm("r0");
  register uint32_t rhi asm("r1");
  uint32_t r4;
  uint32_t r5;
  uint32_t r6;
  uint32_t r7;

  /* The middle carries of the four-product form chain upward through
   * adc; the low words are consumed or discarded as they are added.
   */

  __asm__ __volatile__
  (
    "umull %0, %1, %6, %8\n"    /* %1:%0 = al * bl */
    "umull %0, %2, %7, %8\n"    /* %2:%0 = ah * bl */
    "adds  %0, %0, %1\n"
    "adc   %2, %2, #0\n"
    "umull %1, %3, %6, %9\n"    /* %3:%1 = al * bh */
    "adds  %0, %0, %1\n"
    "adcs  %2, %2, %3\n"
    "umull %0, %3, %7, %9\n"    /* %3:%0 = ah * bh */
    "adc   %3, %3, #0\n"
    "adds  %4, %2, %0\n"
    "adc   %5, %3, #0"
    : "=&r"(r4), "=&r"(r5), "=&r"(r6), "=&r"(r7), "=&r"(rlo), "=&r"(rhi)
    : "r"(al), "r"(ah), "r"(bl), "r"(bh)
    : "cc"
  );

  return ((uint64_t)rhi << 32) | rlo;
}

#endif /* !__thumb__ || __thumb2__ */
