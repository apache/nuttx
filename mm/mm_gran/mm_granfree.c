/****************************************************************************
 * mm/mm_gran/mm_granfree.c
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
 * Name: gran_free
 *
 * Description:
 *   Return memory to the granule heap.
 *
 * Input Parameters:
 *   handle - The handle previously returned by gran_initialize
 *   memory - A pointer to memory previoiusly allocated by gran_alloc.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void gran_free(GRAN_HANDLE handle, FAR void *memory, size_t size)
{
  FAR struct gran_s *priv = (FAR struct gran_s *)handle;
  unsigned int granno;
  unsigned int gatidx;
  unsigned int gatbit;
  unsigned int granmask;
  unsigned int ngranules;
  unsigned int avail;
  uint32_t     gatmask;
  int          ret;

  DEBUGASSERT(priv != NULL && memory && size <= 32 * (1 << priv->log2gran));

  /* Get exclusive access to the GAT */

  do
    {
      ret = gran_enter_critical(priv);

      /* The only error Should happen on task cancellation.  We must
       * try again in this case to avoid stranding the granule memory.
       */

      DEBUGASSERT(ret == OK || ret == -ECANCELED);
    }
  while (ret < 0);

  /* Determine the granule number of the first granule in the allocation */

  granno = ((uintptr_t)memory - priv->heapstart) >> priv->log2gran;

  /* Determine the GAT table index and bit number associated with the
   * allocation.
   */

  gatidx = granno >> 5;
  gatbit = granno & 31;

  /* Determine the number of granules in the allocation */

  granmask =  (1 << priv->log2gran) - 1;
  ngranules = (size + granmask) >> priv->log2gran;

  /* Clear bits in the GAT entry or entries */

  avail = 32 - gatbit;
  if (ngranules > avail)
    {
      /* Clear bits in the first GAT entry */

      gatmask = (0xffffffff << gatbit);
      DEBUGASSERT((priv->gat[gatidx] & gatmask) == gatmask);

      priv->gat[gatidx] &= ~gatmask;
      ngranules -= avail;

      /* Clear bits in the second GAT entry */

      gatmask = 0xffffffff >> (32 - ngranules);
      DEBUGASSERT((priv->gat[gatidx + 1] & gatmask) == gatmask);

      priv->gat[gatidx + 1] &= ~gatmask;
    }

  /* Handle the case where where all of the granules came from one entry */

  else
    {
      /* Clear bits in a single GAT entry */

      gatmask   = 0xffffffff >> (32 - ngranules);
      gatmask <<= gatbit;
      DEBUGASSERT((priv->gat[gatidx] & gatmask) == gatmask);

      priv->gat[gatidx] &= ~gatmask;
    }

  gran_leave_critical(priv);
}

#endif /* CONFIG_GRAN */
