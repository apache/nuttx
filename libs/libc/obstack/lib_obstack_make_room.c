/****************************************************************************
 * libs/libc/obstack/lib_obstack_make_room.c
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

#include <obstack.h>
#include <assert.h>
#include <string.h>
#include "lib_obstack_malloc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: obstack_make_room
 *
 * Description:
 *   This is non-standard function that is probably available only on NuttX!
 *   Make sure that there is room in the buffer to fit object with given
 *   size. The allocation performed is in multiples of chunk_size specified
 *   for the obstack.
 *
 * Input Parameters:
 *   h: pointer to the handle where room should be made
 *   size: number of bytes to be free for growth
 *
 * Assumptions/Limitations:
 *   The obstack's chunk_size is expected to be power of two. This helps to
 *   eliminate division that might not be implemented in the HW and thus
 *   inefficient.
 *
 ****************************************************************************/

void obstack_make_room(FAR struct obstack *h, size_t size)
{
  unsigned i;
  size_t mask;

  DEBUGASSERT(h);

  if (obstack_room(h) >= size)
    {
      return; /* No need to allocate anything */
    }

  size_t object_size = obstack_object_size(h);

  size += object_size + sizeof(struct _obstack_chunk);

  /* Note: this is rounding up to the multiple of chunk size that is power of
   * two. Thus this creates limitation that chunks can be only power of two.
   */

  mask = h->chunk_size;
  for (i = 1; i < sizeof(size_t); i <<= 1)
      mask |= mask >> i;
  size = (size + mask) & ~mask;

  if (h->chunk == NULL ||
      h->object_base != (FAR char *)h->chunk + sizeof(struct _obstack_chunk))
    {
      /* Allocate new chunk if there is something in the chunk already or if
       * there is no chunk yet.
       */

      FAR struct _obstack_chunk *prev = h->chunk;
      h->chunk = lib_obstack_malloc(size);
      h->chunk->prev = prev;

      /* Move growing object to the new chunk */

      memcpy((FAR char *)h->chunk + sizeof(struct _obstack_chunk),
             h->object_base, object_size);
    }

  else
    {
      h->chunk = lib_obstack_realloc(h->chunk, size);
    }

  h->chunk->limit =
    (FAR char *)h->chunk + sizeof(struct _obstack_chunk) + size;
  h->object_base = (FAR char *)h->chunk + sizeof(struct _obstack_chunk);
  h->next_free = h->object_base + object_size;
}
