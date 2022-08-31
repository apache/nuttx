/****************************************************************************
 * libs/libc/obstack/lib_obstack_alloc.c
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
#include "lib_obstack_malloc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: obstack_alloc
 *
 * Description:
 *   Allocate an object of given size with uninitialized bytes.
 *   Compared to the GlibC version this uses malloc to allocate exactly
 *   required space (plus overhead) and nothing more.
 *
 * Input Parameters:
 *   h: pointer to the handle to allocate an object in
 *   size: number of bytes to allocate
 *
 ****************************************************************************/

FAR void *obstack_alloc(FAR struct obstack *h, size_t size)
{
  FAR struct _obstack_chunk *prev;
  FAR void *res;

  if (h->chunk == NULL || (h->chunk->limit - h->object_base) < size)
    {
      /* TODO: could we just expand current allocation? */

      prev = h->chunk;
      h->chunk = lib_obstack_malloc(size + sizeof(struct _obstack_chunk));
      h->chunk->limit =
        (FAR char *)h->chunk + sizeof(struct _obstack_chunk) + size;
      h->chunk->prev = prev;
      h->object_base = (FAR char *)h->chunk + sizeof(struct _obstack_chunk);
    }

  res = h->object_base;
  h->object_base += size;
  h->next_free = h->object_base;

  return res;
}
