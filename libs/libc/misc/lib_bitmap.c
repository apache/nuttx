/****************************************************************************
 * libs/libc/misc/lib_bitmap.c
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

#include <errno.h>
#include <strings.h>

#include <nuttx/bits.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: find_next_bit
 *
 * Description:
 *   This function is used to find the first set bit in a memory region.
 *
 *   Invoke to get the bit set in map, implement find_{first,next}_bit.
 *
 * Input Parameters:
 *   addr   - Map start memory address
 *   size   - Max size to find bit
 *   offset - Offset of start to finding
 *
 * Returned Value:
 *   Return index position (0 ~ size-1) if finded, otherwise return size
 ****************************************************************************/

unsigned long find_next_bit(FAR const unsigned long *addr,
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
      tmp &= ~0ul << offset;
      if (size < BITS_PER_LONG)
        {
          goto found_first;
        }

      if (tmp)
        {
          goto found_middle;
        }

      size -= BITS_PER_LONG;
      result += BITS_PER_LONG;
    }

  while (size & ~(BITS_PER_LONG - 1))
    {
      if ((tmp = *(p++)))
        {
          goto found_middle;
        }

      result += BITS_PER_LONG;
      size -= BITS_PER_LONG;
    }

  if (size == 0)
    {
      return result;
    }

  tmp = *p;

found_first:
  tmp &= ~0ul >> (BITS_PER_LONG - size);

  /* Are all bits zero? */

  if (tmp == 0ul)
    {
      return result + size;
    }

found_middle:
  return result + ffsl((long)tmp) - 1;
}

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
 *   Return index position (0 ~ size-1) if Finded, otherwise return size
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

  if (size == 0)
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
  return result + ffsl((long)~tmp) - 1;
}

/****************************************************************************
 * Name: bitmap_set
 ****************************************************************************/

void bitmap_set(FAR unsigned long *bitmap,
                unsigned long start,
                unsigned long len)
{
  FAR unsigned long *p = bitmap + BIT_WORD(start);
  unsigned long bits_to_set = BITS_PER_LONG - (start % BITS_PER_LONG);
  unsigned long mask_to_set = BITMAP_FIRST_WORD_MASK(start);
  unsigned long size = start + len;

  while (bits_to_set < len)
    {
      *p++ |= mask_to_set;
      len -= bits_to_set;
      bits_to_set = BITS_PER_LONG;
      mask_to_set = ~0ul;
    }

  if (len)
    {
      mask_to_set &= BITMAP_LAST_WORD_MASK(size);
      *p |= mask_to_set;
    }
}

/****************************************************************************
 * Name: bitmap_set
 ****************************************************************************/

void bitmap_clear(FAR unsigned long *bitmap,
                  unsigned long start,
                  unsigned long len)
{
  FAR unsigned long *p = bitmap + BIT_WORD(start);
  unsigned long bits_to_clear = BITS_PER_LONG - (start % BITS_PER_LONG);
  unsigned long mask_to_clear = BITMAP_FIRST_WORD_MASK(start);
  unsigned long size = start + len;

  while (bits_to_clear < len)
    {
      *p++ &= ~mask_to_clear;
      len -= bits_to_clear;
      bits_to_clear = BITS_PER_LONG;
      mask_to_clear = ~0ul;
    }

  if (len)
    {
      mask_to_clear &= BITMAP_LAST_WORD_MASK(size);
      *p &= ~mask_to_clear;
    }
}

/****************************************************************************
 * Name: bitmap_allocate_region
 ****************************************************************************/

int bitmap_allocate_region(FAR unsigned long *bitmap,
                           unsigned long start,
                           unsigned long len)
{
  if (find_next_bit(bitmap, start + len, start) < start + len)
    {
      return -EBUSY;
    }

  bitmap_set(bitmap, start, len);
  return 0;
}

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
                                      unsigned long len)
{
  unsigned long pos;
  unsigned long end;

  for (pos = 0; (end = pos + len) <= size; pos = end)
    {
      if (!bitmap_allocate_region(bitmap, pos, len))
        {
          return pos;
        }
    }

  return size;
}
