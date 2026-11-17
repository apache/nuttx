/****************************************************************************
 * libs/libc/machine/up_umulh64.c
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

/* UMULH: high 64 bits of the 128-bit product a * b, the portable form of
 * up_umulh64().  Weak so that libs/libc/machine/<arch> can replace it;
 * this copy is used only when the architecture supplies none.  A target
 * with a native 128-bit multiply inlines up_umulh64() from
 * include/nuttx/lib/math32.h and needs no copy here.
 */

#if !defined(__INT128_DEFINED)

uint64_t weak_function up_umulh64(uint64_t a, uint64_t b)
{
  uint64_t al = a & UINT32_MAX;
  uint64_t ah = a >> 32;
  uint64_t bl = b & UINT32_MAX;
  uint64_t bh = b >> 32;

  uint64_t m1 = al * bl;
  uint64_t m2 = ah * bl;
  uint64_t m3 = al * bh;
  uint64_t m4 = ah * bh;

  uint64_t c = (m1 >> 32) + (m2 & UINT32_MAX) + (m3 & UINT32_MAX);

  return m4 + (m2 >> 32) + (m3 >> 32) + (c >> 32);
}

#endif /* !__INT128_DEFINED */
