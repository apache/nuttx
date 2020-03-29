/****************************************************************************
 * mm/mm_gran/mm_granalloc.c
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

#include <assert.h>

#include <nuttx/mm/gran.h>

#include "mm_gran/mm_gran.h"

#ifdef CONFIG_GRAN

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gran_alloc
 *
 * Description:
 *   Allocate memory from the granule heap.
 *
 *   NOTE: The current implementation also restricts the maximum allocation
 *   size to 32 granules.  That restriction could be eliminated with some
 *   additional coding effort.
 *
 * Input Parameters:
 *   handle - The handle previously returned by gran_initialize
 *   size   - The size of the memory region to allocate.
 *
 * Returned Value:
 *   On success, a non-NULL pointer to the allocated memory is returned;
 *   NULL is returned on failure.
 *
 ****************************************************************************/

FAR void *gran_alloc(GRAN_HANDLE handle, size_t size)
{
  FAR struct gran_s *priv = (FAR struct gran_s *)handle;
  unsigned int ngranules;
  size_t       tmpmask;
  uintptr_t    alloc;
  uint32_t     curr;
  uint32_t     next;
  uint32_t     mask;
  int          granidx;
  int          gatidx;
  int          bitidx;
  int          shift;
  int          ret;

  DEBUGASSERT(priv != NULL && size <= 32 * (1 << priv->log2gran));

  if (priv != NULL && size > 0)
    {
      /* Get exclusive access to the GAT */

      ret = gran_enter_critical(priv);
      if (ret < 0)
        {
          return NULL;
        }

      /* How many contiguous granules we we need to find? */

      tmpmask   = (1 << priv->log2gran) - 1;
      ngranules = (size + tmpmask) >> priv->log2gran;

      /* Then create mask for that number of granules */

      DEBUGASSERT(ngranules <= 32);
      mask = 0xffffffff >> (32 - ngranules);

      /* Now search the granule allocation table for that number of contiguous */

      for (granidx = 0; granidx < priv->ngranules; granidx += 32)
        {
          /* Get the GAT index associated with the granule table entry */

          gatidx = granidx >> 5;
          curr = priv->gat[gatidx];

          /* Handle the case where there are no free granules in the entry */

          if (curr == 0xffffffff)
            {
              continue;
            }

          /* Get the next entry from the GAT to support a 64 bit shift */

          if (granidx < priv->ngranules)
            {
              next = priv->gat[gatidx + 1];
            }

          /* Use all ones when are at the last entry in the GAT (meaning
           * nothing can be allocated.
           */

          else
            {
              next = 0xffffffff;
            }

          /* Search through the allocations in the 'curr' GAT entry
           * to see if we can satisfy the allocation starting in that
           * entry.
           *
           * This loop continues until either all of the bits have been
           * examined (bitidx >= 32), or until there are insufficient
           * granules left to satisfy the allocation.
           */

          alloc = priv->heapstart + (granidx << priv->log2gran);

          for (bitidx = 0;
               bitidx < 32 &&
               (granidx + bitidx + ngranules) <= priv->ngranules;
              )
            {
              /* Break out if there are no further free bits in 'curr'.
               * All of the zero bits might have gotten shifted out.
               */

              if (curr == 0xffffffff)
                {
                  break;
                }

              /* Check for the first zero bit in the lower or upper 16-bits.
               * From the test above, we know that at least one of the 32-
               * bits in 'curr' is zero.
               */

              else if ((curr & 0x0000ffff) == 0x0000ffff)
                {
                  /* Not in the lower 16 bits.  The first free bit must be
                   * in the upper 16 bits.
                   */

                  shift = 16;
                }

              /* We know that the first free bit is now within the lower 16
               * bits of 'curr'.  Is it in the upper or lower byte?
               */

              else if ((curr & 0x0000ff) == 0x000000ff)
                {
                  /* Not in the lower 8 bits.  The first free bit must be in
                   * the upper 8 bits.
                   */

                  shift = 8;
                }

              /* We know that the first free bit is now within the lower 4
               * bits of 'curr'.  Is it in the upper or lower nibble?
               */

              else if ((curr & 0x00000f) == 0x0000000f)
                {
                  /* Not in the lower 4 bits.  The first free bit must be in
                   * the upper 4 bits.
                   */

                  shift = 4;
                }

              /* We know that the first free bit is now within the lower 4
               * bits of 'curr'.  Is it in the upper or lower pair?
               */

              else if ((curr & 0x000003) == 0x00000003)
                {
                  /* Not in the lower 2 bits.  The first free bit must be in
                   * the upper 2 bits.
                   */

                  shift = 2;
                }

              /* We know that the first free bit is now within the lower 4
               * bits of 'curr'.  Check if we have the allocation at this
               * bit position.
               */

              else if ((curr & mask) == 0)
                {
                  /* Yes.. mark these granules allocated */

                  gran_mark_allocated(priv, alloc, ngranules);

                  /* And return the allocation address */

                  gran_leave_critical(priv);
                  return (FAR void *)alloc;
                }

              /* The free allocation does not start at this position */

              else
                {
                  shift = 1;
                }

              /* Set up for the next time through the loop.  Perform a 64
               * bit shift to move to the next gran position and increment
               * to the next candidate allocation address.
               */

              alloc  += (shift << priv->log2gran);
              curr    = (curr >> shift) | (next << (32 - shift));
              next  >>= shift;
              bitidx += shift;
            }
        }

      gran_leave_critical(priv);
    }

  return NULL;
}

#endif /* CONFIG_GRAN */
