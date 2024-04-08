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
#include <debug.h>

#include <nuttx/mm/gran.h>

#include "mm_gran/mm_gran.h"
#include "mm_gran/mm_grantable.h"

#ifdef CONFIG_GRAN

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void gran_free(GRAN_HANDLE handle, FAR void *memory, size_t size)
{
  FAR    gran_t *gran = (FAR gran_t *)handle;
  uint   ngran;
  size_t posi;
  int    ret;

  DEBUGASSERT(gran && memory && size);
  DEBUGASSERT(GRAN_PRODUCT(gran, memory));
  DEBUGASSERT(GRAN_INRANGE(gran, (((uintptr_t)memory) + size - 1)));

  posi  = MEM2GRAN(gran, memory);
  ngran = NGRANULE(gran, size);

  graninfo(" heap=%"PRIxPTR" posi=%zu addr=%zx size=%zu n=%d\n",
           gran->heapstart, posi, (size_t)memory, size, ngran);

  do
    {
      ret = gran_enter_critical(gran);
      DEBUGASSERT(ret == OK || ret == -ECANCELED);
    }
  while (ret < 0);  /* Retry upon task cancellation */

  /* check double free */

  DEBUGASSERT(gran_match(gran, posi, ngran, 1, NULL));
  gran_clear(gran, posi, ngran);
  gran_leave_critical(gran);
}

#endif /* CONFIG_GRAN */
