/****************************************************************************
 * mm/mm_gran/mm_granmark.c
 *
 *   Copyright (C) 2012, 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
 *   None
 *
 ****************************************************************************/

void gran_mark_allocated(FAR struct gran_s *priv, uintptr_t alloc,
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
      /* Mark bits in the first GAT entry */

      gatmask = 0xffffffff << gatbit;
      DEBUGASSERT((priv->gat[gatidx] & gatmask) == 0);

      priv->gat[gatidx] |= gatmask;
      ngranules -= avail;

      /* Mark bits in the second GAT entry */

      gatmask = 0xffffffff >> (32 - ngranules);
      DEBUGASSERT((priv->gat[gatidx + 1] & gatmask) == 0);

      priv->gat[gatidx + 1] |= gatmask;
    }

  /* Handle the case where where all of the granules come from one entry */

  else
    {
      /* Mark bits in a single GAT entry */

      gatmask   = 0xffffffff >> (32 - ngranules);
      gatmask <<= gatbit;
      DEBUGASSERT((priv->gat[gatidx] & gatmask) == 0);

      priv->gat[gatidx] |= gatmask;
      return;
    }
}

#endif /* CONFIG_GRAN */
