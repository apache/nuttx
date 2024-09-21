/****************************************************************************
 * mm/mm_gran/mm_granreserve.c
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

#include <assert.h>
#include <debug.h>

#include <nuttx/mm/gran.h>

#include "mm_gran/mm_gran.h"
#include "mm_gran/mm_grantable.h"

#ifdef CONFIG_GRAN

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR void *gran_reserve(GRAN_HANDLE handle, uintptr_t start, size_t size)
{
  FAR gran_t *gran = (FAR gran_t *)handle;
  uintptr_t   end;
  size_t      ngran;
  size_t      posi;
  bool        avail;

  DEBUGASSERT(gran);
  if (!size || size > GRANBYTE(gran))
    {
      return NULL;
    }

  /* align down/up start/ending addresses */

  end = END_RSRV(gran, start, size);
  if (!GRAN_INRANGE(gran, end))
    {
      return NULL;
    }

  start = MEM_RSRV(gran, start);
  if (!GRAN_INRANGE(gran, start))
    {
      return NULL;
    }

  /* convert unit to granule */

  posi  = MEM2GRAN(gran, start);
  ngran = ((end - start) >> gran->log2gran) + 1;

  /* lock the granule allocator */

  if (gran_enter_critical(gran) < 0)
    {
      return NULL;
    }

  avail = gran_match(gran, posi, ngran, 0, NULL);
  if (avail)
    {
      gran_set(gran, posi, ngran);
    }

  gran_leave_critical(gran);

  graninfo("%s posi=%zu retp=%zx size=%zu n=%zu\n",
           avail ? "  done" : " error", posi, (size_t)start, size, ngran);

  return avail ? (FAR void *)start : NULL;
}

#endif /* CONFIG_GRAN */
