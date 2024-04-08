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
#include <debug.h>

#include <nuttx/mm/gran.h>

#include "mm_gran/mm_gran.h"
#include "mm_gran/mm_grantable.h"

#ifdef CONFIG_GRAN

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR void *gran_alloc(GRAN_HANDLE handle, size_t size)
{
  FAR gran_t *gran = (FAR gran_t *)handle;
  size_t ngran;
  int posi;
  int ret;
  uintptr_t retp;

  DEBUGASSERT(gran);
  ngran = NGRANULE(gran, size);
  if (!ngran || ngran > gran->ngranules)
    {
      return NULL;
    }

  ret = gran_enter_critical(gran);
  if (ret < 0)
    {
      return NULL;
    }

  posi = gran_search(gran, ngran);
  if (posi >= 0)
    {
      gran_set(gran, posi, ngran);
    }

  gran_leave_critical(gran);
  if (posi < 0)
    {
      return NULL;
    }

  retp = gran->heapstart + (posi << gran->log2gran);
  graninfo("heap=%"PRIxPTR" posi=%d retp=%"PRIxPTR" size=%zu n=%zu\n",
           gran->heapstart, posi, retp, size, ngran);
  DEBUGASSERT(retp >= gran->heapstart);
  DEBUGASSERT(retp < gran->heapstart + GRANBYTE(gran));
  return (FAR void *)retp;
}

#endif /* CONFIG_GRAN */
