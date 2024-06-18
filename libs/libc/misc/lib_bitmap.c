/****************************************************************************
 * libs/libc/misc/lib_bitmap.c
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

#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <string.h>

#include <nuttx/bits.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: find_next_zero_bit
 *
 * Description:
 *   This function is used to find the first cleared bit in a memory region.
 *
 *   Invoke to get the bit set in map, implement find_{first,next}_zero_bit.
 *
 * Input Parameters:
 *   addr   - Map start memory address
 *   size   - Max size to find bit
 *   offset - Offset of start to finding
 *
 * Returned Value:
 *   Return index position（0 ~ size-1）if Finded, otherwise return size
 ****************************************************************************/

unsigned long find_next_zero_bit(FAR const unsigned long *addr,
                                 unsigned long size,
                                 unsigned long offset)
{
  FAR const unsigned long *p = addr + BIT_WORD(offset);
  unsigned long result = offset & ~(BITS_PER_LONG - 1);
  unsigned long tmp;

  if (offset >= size)
    {
      return size;
    }

  size -= result;
  offset %= BITS_PER_LONG;
  if (offset)
    {
      tmp = *(p++);
      tmp |= ~0ul >> (BITS_PER_LONG - offset);
      if (size < BITS_PER_LONG)
        {
          goto found_first;
        }

      if (~tmp)
        {
          goto found_middle;
        }

      size -= BITS_PER_LONG;
      result += BITS_PER_LONG;
    }

  while (size & ~(BITS_PER_LONG - 1))
    {
      if (~(tmp = *(p++)))
        {
          goto found_middle;
        }

      result += BITS_PER_LONG;
      size -= BITS_PER_LONG;
    }

  if (!size)
    {
      return result;
    }

  tmp = *p;

found_first:
  tmp |= ~0ul << size;

  /* Are any bits zero? */

  if (tmp == ~0ul)
    {
      return result + size;
    }

found_middle:
  return result + ffsl(~tmp) - 1;
}
