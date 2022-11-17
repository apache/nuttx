/****************************************************************************
 * mm/mm_gran/mm_granmark.c
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
 * Name: gran_mark_allocated
 *
 * Description:
 *   Mark a range of granules as allocated.
 *
 * Input Parameters:
 *   priv  - The granule heap state structure.
 *   alloc - The address of the allocation.
 *   ngranules - The number of granules allocated
 *
 * Returned Value:
 *   On success, a non-NULL pointer to the allocated memory is returned;
 *   NULL is returned on failure.
 *
 ****************************************************************************/

FAR void *gran_mark_allocated(FAR struct gran_s *priv, uintptr_t alloc,
                              unsigned int ngranules)
{
  unsigned int granno;
  unsigned int gatidx;
  unsigned int gatbit;
  unsigned int avail;
  uint32_t     gatmask;

  /* Determine the granule number of the allocation */

  granno = (alloc - priv->heapstart) >> priv->log2gran;

  /* Determine the GAT table index associated with the allocation */

  gatidx = granno >> 5;
  gatbit = granno & 31;

  /* Mark bits in the GAT entry or entries */

  avail = 32 - gatbit;
  if (ngranules > avail)
    {
      uint32_t gatmask2;

      gatmask    = 0xffffffff << gatbit;
      ngranules -= avail;
      gatmask2   = 0xffffffff >> (32 - ngranules);

      /* Check that the area is free, from both mask words */

      if (((priv->gat[gatidx] & gatmask) != 0) ||
          ((priv->gat[gatidx + 1] & gatmask2) != 0))
        {
          return NULL;
        }

      /* Mark bits in the first and second GAT entry */

      priv->gat[gatidx] |= gatmask;
      priv->gat[gatidx + 1] |= gatmask2;
    }

  /* Handle the case where where all of the granules come from one entry */

  else
    {
      gatmask   = 0xffffffff >> (32 - ngranules);
      gatmask <<= gatbit;

      /* Check that the area is free */

      if ((priv->gat[gatidx] & gatmask) != 0)
        {
          return NULL;
        }

      /* Mark bits in a single GAT entry */

      priv->gat[gatidx] |= gatmask;
    }

  return (FAR void *)alloc;
}

#endif /* CONFIG_GRAN */
