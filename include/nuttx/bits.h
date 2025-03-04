/****************************************************************************
 * include/nuttx/bits.h
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

#ifndef __INCLUDE_NUTTX_BITS_H
#define __INCLUDE_NUTTX_BITS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <assert.h>
#include <limits.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef BITS_PER_BYTE
# define BITS_PER_BYTE CHAR_BIT
#endif

#ifndef BITS_PER_LONG
# define BITS_PER_LONG (sizeof(unsigned long) * BITS_PER_BYTE)
#endif

#ifndef BITS_TO_LONGS
#  define BITS_TO_LONGS(nr) (((nr) + BITS_PER_LONG - 1) / BITS_PER_LONG)
#endif

#ifndef BITS_PER_LONG_LONG
# define BITS_PER_LONG_LONG (sizeof(unsigned long long) * BITS_PER_BYTE)
#endif

#define BIT_BYTE_MASK(nr)  (1ul << ((nr) % BITS_PER_BYTE))
#define BIT_WORD_MASK(nr)  (1ul << ((nr) % BITS_PER_LONG))
#define BIT_BYTE(nr)       ((nr) / BITS_PER_BYTE)
#define BIT_WORD(nr)       ((nr) / BITS_PER_LONG)
#define BIT_ULL_MASK(nr)   (1ull << ((nr) % BITS_PER_LONG_LONG))
#define BIT_ULL_WORD(nr)   ((nr) / BITS_PER_LONG_LONG)
#define BIT(nr)            (1ul << (nr))
#define BIT_ULL(nr)        (1ull << (nr))

/* Create a contiguous bitmask starting at bit position l and ending at
 * position h. For example
 * GENMASK_ULL(39, 21) gives us the 64bit vector 0x000000ffffe00000.
 */

#define __GENMASK(h, l) \
        (((~0ul) - (1ul << (l)) + 1) & \
         (~0ul >> (BITS_PER_LONG - 1 - (h))))
#define GENMASK(h, l) \
        (BUILD_BUG_ON_ZERO((l) > (h)) + __GENMASK(h, l))

#define __GENMASK_ULL(h, l) \
        (((~0ull) - (1ull << (l)) + 1) & \
         (~0ull >> (BITS_PER_LONG_LONG - 1 - (h))))
#define GENMASK_ULL(h, l) \
        (BUILD_BUG_ON_ZERO((l) > (h)) + __GENMASK_ULL(h, l))

#define BMVAL(val, lsb, msb) (((val) & GENMASK(msb, lsb)) >> (lsb))

/* Bitmap operations */

#define DECLARE_BITMAP(name, bits) \
        unsigned long name[BITS_TO_LONGS(bits)]

#define BITMAP_FIRST_WORD_MASK(start) (~0ul << ((start) & (BITS_PER_LONG - 1)))
#define BITMAP_LAST_WORD_MASK(len) (~0ul >> (-(len) & (BITS_PER_LONG - 1)))

#define set_bit(nr, addr) \
        (*(((FAR unsigned long *)(addr)) + BIT_WORD(nr)) |= \
        BIT_WORD_MASK(nr))

#define clear_bit(nr, addr) \
        (*(((FAR unsigned long *)(addr)) + BIT_WORD(nr)) &= \
        ~BIT_WORD_MASK(nr))

#define test_bit(nr, addr) \
        (*(((FAR unsigned long *)(addr)) + BIT_WORD(nr)) & \
        BIT_WORD_MASK(nr))

#define find_first_bit(addr, size) find_next_bit(addr, size, 0)
#define find_first_zero_bit(addr, size) find_next_zero_bit(addr, size, 0)

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLER__
unsigned long find_next_bit(FAR const unsigned long *addr,
                            unsigned long size,
                            unsigned long offset);
unsigned long find_next_zero_bit(FAR const unsigned long *addr,
                                 unsigned long size,
                                 unsigned long offset);

void bitmap_set(FAR unsigned long *bitmap,
                unsigned long satrt,
                unsigned long len);
void bitmap_clear(FAR unsigned long *bitmap,
                  unsigned long start,
                  unsigned long len);

int bitmap_allocate_region(FAR unsigned long *bitmap,
                           unsigned long start,
                           unsigned long len);
#define bitmap_release_region(bitmap, start, len) \
        bitmap_clear(bitmap, start, len)

/****************************************************************************
 * Name: bitmap_find_free_region
 *
 * Description:
 *   Find a contiguous memory region
 *
 *   Find a region of free (zero) len in a bitmap of size and
 *   allocate them (set them to one).
 *
 * Input Parameters:
 *   bitmap - Array of unsigned longs corresponding to the bitmap
 *   size   - Number of bits in the bitmap
 *   len    - Region size to find
 *
 * Returned Value:
 *   Return the bit offset in bitmap of the allocated region,
 *   otherwise return size
 ****************************************************************************/

unsigned long bitmap_find_free_region(FAR unsigned long *bitmap,
                                      unsigned long size,
                                      unsigned long len);

#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_BITS_H */
