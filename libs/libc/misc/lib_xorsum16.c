/****************************************************************************
 * libs/libc/misc/lib_xorsum16.c
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

#include <stdbool.h>
#include <stdint.h>

#include <nuttx/lib/xorsum.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Shift that puts the byte at address a in its lane of a native uint64_t
 * read from the aligned word containing a.
 */

#ifdef CONFIG_ENDIAN_BIG
#  define XORSUM_LANE(a)  ((7 - ((uintptr_t)(a) & 7)) * 8)
#else
#  define XORSUM_LANE(a)  (((uintptr_t)(a) & 7) * 8)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xorsum16
 *
 * Description:
 *   Return the XOR of the 16-bit little-endian words of a buffer, see
 *   include/nuttx/lib/xorsum.h.
 *
 ****************************************************************************/

uint16_t xorsum16(FAR const void *buf, size_t len)
{
  FAR const uint8_t *src = buf;
  FAR const uint64_t *p;
  uint64_t sum = 0;
  uint16_t ret;
  bool odd = (uintptr_t)src & 1;

  /* Every byte goes into the lane of its address, so the folded sum below
   * depends only on address parity; the swap for an odd start makes it a
   * function of the offset from buf alone.
   */

  while (((uintptr_t)src & 7) && len > 0)
    {
      sum ^= (uint64_t)*src << XORSUM_LANE(src);
      src++;
      len--;
    }

  p = (FAR const uint64_t *)src;
  while (len >= 64)
    {
      sum ^= p[0] ^ p[1] ^ p[2] ^ p[3] ^ p[4] ^ p[5] ^ p[6] ^ p[7];
      p   += 8;
      len -= 64;
    }

  while (len >= 8)
    {
      sum ^= *p++;
      len -= 8;
    }

  src = (FAR const uint8_t *)p;
  while (len-- > 0)
    {
      sum ^= (uint64_t)*src << XORSUM_LANE(src);
      src++;
    }

  sum ^= sum >> 32;
  sum ^= sum >> 16;
  ret  = (uint16_t)sum;

#ifdef CONFIG_ENDIAN_BIG
  /* Lanes are mirrored: even addresses landed in the high byte */

  odd = !odd;
#endif

  return odd ? (uint16_t)((ret << 8) | (ret >> 8)) : ret;
}
