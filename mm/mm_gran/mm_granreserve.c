/****************************************************************************
 * mm/mm_gran/mm_granreserve.c
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
 * Name: gran_reserve
 *
 * Description:
 *   Reserve memory in the granule heap.  This will reserve the granules
 *   that contain the start and end addresses plus all of the granules
 *   in between.  This should be done early in the initialization sequence
 *   before any other allocations are made.
 *
 *   Reserved memory can never be allocated (it can be freed however which
 *   essentially unreserves the memory).
 *
 * Input Parameters:
 *   handle - The handle previously returned by gran_initialize
 *   start  - The address of the beginning of the region to be reserved.
 *   size   - The size of the region to be reserved
 *
 * Returned Value:
 *   On success, a non-NULL pointer to the allocated memory is returned;
 *   NULL is returned on failure.
 *
 ****************************************************************************/

FAR void *gran_reserve(GRAN_HANDLE handle, uintptr_t start, size_t size)
{
  FAR struct gran_s *priv = (FAR struct gran_s *)handle;
  FAR void *ret = NULL;

  DEBUGASSERT(priv != NULL);

  if (size > 0)
    {
      uintptr_t mask = (1 << priv->log2gran) - 1;
      uintptr_t end  = start + size - 1;
      unsigned int ngranules;

      /* Get the aligned (down) start address and the aligned (up) end
       * address
       */

      start &= ~mask;
      end = (end + mask) & ~mask;

      /* Calculate the new size in granules */

      ngranules = ((end - start) >> priv->log2gran) + 1;

      /* Must lock the granule allocator */

      if (gran_enter_critical(priv) < 0)
        {
          return NULL;
        }

      /* And reserve the granules */

      ret = gran_mark_allocated(priv, start, ngranules);
      gran_leave_critical(priv);
    }

  return ret;
}

#endif /* CONFIG_GRAN */
