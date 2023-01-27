/****************************************************************************
 * libs/libc/obstack/lib_obstack_finish.c
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
#include "lib_obstack_malloc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: obstack_finish
 *
 * Description:
 *   Finish growing object and receive address to it.
 *   Compared to the GlibC version this uses realloc to reduce buffer size to
 *   only allocated amount. The non-standard obstack_finish_norealloc can be
 *   used if you want the standard behavior for ever reason.
 *
 * Input Parameters:
 *   h: pointer to the handle used to grow the object.
 *
 * Returned Value:
 *   Permanent address to the object.
 *
 ****************************************************************************/

FAR void *obstack_finish(FAR struct obstack *h)
{
  size_t chsize;
  void *chbase;

  /* To reduce memory used by allocation we reallocate the chunk to fit
   * exactly data it contains.
   * This can be performed only if there is no other object preset in the
   * chunk already as we can't ensure that realloc won't move the data and
   * thus invalidate already returned address to the existing object.
   * The object can be there if called uses obstack_finish alongside with
   * obstack_finish_norealloc.
   */

  chbase = (FAR char *)h->chunk + sizeof(struct _obstack_chunk);
  if (chbase == h->object_base)
    {
      chsize = h->next_free - (FAR char *)h->chunk;
      h->chunk = lib_obstack_realloc(h->chunk, chsize);
      h->chunk->limit = (FAR char *)h->chunk + chsize;
      h->object_base = h->chunk->limit;
      h->next_free = h->chunk->limit;
      return (FAR char *)h->chunk + sizeof(struct _obstack_chunk);
    }

  return obstack_finish_norealloc(h);
}

/****************************************************************************
 * Name: obstack_finish_norealloc
 *
 * Description:
 *   Finish growing object and receive address to it without reallocating
 *   buffer to fit the object (keeping space for more growth).
 *
 * Input Parameters:
 *   h: pointer to the handle used to grow the object.
 *
 * Returned Value:
 *   Permanent address to the object.
 *
 ****************************************************************************/

FAR void *obstack_finish_norealloc(FAR struct obstack *h)
{
  FAR char *res = h->object_base;
  h->object_base = h->next_free;
  return res;
}
