/****************************************************************************
 * libs/libc/tls/tls_emutls.c
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

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <nuttx/atomic.h>
#include <nuttx/mutex.h>
#include <nuttx/tls.h>

#include "libc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* loc.offset of a control object is as wide as a pointer */

#if UINTPTR_MAX > UINT32_MAX
#  define emutls_load(p)     atomic64_read_acquire(p)
#  define emutls_store(p, v) atomic64_set_release(p, v)
#else
#  define emutls_load(p)     atomic_read_acquire(p)
#  define emutls_store(p, v) atomic_set_release(p, v)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The control object that GCC emits for a __thread variable when the
 * target has no native TLS, with the types of libgcc's emutls.c.
 */

struct __emutls_object
{
  emutls_word_t size;
  emutls_word_t align;
  union
  {
    emutls_pointer_t offset;
    FAR void *ptr;
  } loc;
  FAR void *templ;
};

/* The copies of the variables that one thread has used.  The TLS info of
 * the thread points to it.
 */

struct emutls_array_s
{
  uintptr_t size;
  FAR void *data[1];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The index of a variable is global: in a flat build all task groups share
 * this data, but each variable has its own index.
 */

static mutex_t g_emutls_lock = NXMUTEX_INITIALIZER;
static uintptr_t g_emutls_count;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: emutls_alloc
 *
 * Description:
 *   Allocate the copy of a variable for the calling thread, initialized
 *   from the template of the variable, or zeroed if it has none.
 *
 ****************************************************************************/

static FAR void *emutls_alloc(FAR struct __emutls_object *obj)
{
  size_t align = obj->align > sizeof(FAR void *) ?
                 obj->align : sizeof(FAR void *);
  FAR void *ptr;

  ptr = lib_memalign(align, obj->size);
  if (ptr == NULL)
    {
      abort();
    }

  if (obj->templ != NULL)
    {
      memcpy(ptr, obj->templ, obj->size);
    }
  else
    {
      memset(ptr, 0, obj->size);
    }

  return ptr;
}

/****************************************************************************
 * Name: emutls_getslot
 *
 * Description:
 *   Return the entry for index in the array of the calling thread.
 *
 ****************************************************************************/

static FAR void **emutls_getslot(FAR struct tls_info_s *tls,
                                 uintptr_t index)
{
  FAR struct emutls_array_s *array = tls->tl_emutls;
  FAR struct emutls_array_s *newarray;
  uintptr_t oldsize = array != NULL ? array->size : 0;
  uintptr_t newsize;

  if (index < oldsize)
    {
      return &array->data[index];
    }

  newsize = index + 16;
  newarray = lib_realloc(array, sizeof(struct emutls_array_s) +
                         (newsize - 1) * sizeof(FAR void *));
  if (newarray == NULL)
    {
      abort();
    }

  memset(&newarray->data[oldsize], 0,
         (newsize - oldsize) * sizeof(FAR void *));
  newarray->size = newsize;
  tls->tl_emutls = newarray;
  g_tls_emutls_destruct = tls_emutls_destruct;
  return &newarray->data[index];
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __wrap___emutls_get_address
 *
 * Description:
 *   Return the address of the calling thread's copy of a __thread
 *   variable.  libgcc's __emutls_get_address() keeps a single copy when
 *   libgcc is built without thread support; --wrap sends the calls here.
 *
 ****************************************************************************/

FAR void *__wrap___emutls_get_address(FAR void *arg)
{
  FAR struct __emutls_object *obj = arg;
  FAR void **slot;
  uintptr_t index;

  /* loc.offset is the index plus one, or 0 before the first use */

  index = emutls_load(&obj->loc.offset);
  if (index == 0)
    {
      nxmutex_lock(&g_emutls_lock);
      index = obj->loc.offset;
      if (index == 0)
        {
          index = ++g_emutls_count;
          emutls_store(&obj->loc.offset, index);
        }

      nxmutex_unlock(&g_emutls_lock);
    }

  index--;
  slot = emutls_getslot(tls_get_info(), index);
  if (*slot == NULL)
    {
      *slot = emutls_alloc(obj);
    }

  return *slot;
}

/****************************************************************************
 * Name: __wrap___emutls_register_common
 *
 * Description:
 *   Merge the size, alignment and template of a common __thread variable
 *   that more than one object file defines.
 *
 ****************************************************************************/

void __wrap___emutls_register_common(FAR void *arg, emutls_word_t size,
                                     emutls_word_t align, FAR void *templ)
{
  FAR struct __emutls_object *obj = arg;

  if (obj->size < size)
    {
      obj->size = size;
      obj->templ = NULL;
    }

  if (obj->align < align)
    {
      obj->align = align;
    }

  if (templ != NULL && size == obj->size)
    {
      obj->templ = templ;
    }
}

/* A link without --wrap, such as the user image of a protected build or a
 * program built outside NuttX, uses these names.  libc comes before libgcc
 * there, and libgcc's strong definitions win in a --whole-archive link.
 */

weak_alias(__wrap___emutls_get_address, __emutls_get_address)
weak_alias(__wrap___emutls_register_common, __emutls_register_common)

/****************************************************************************
 * Name: tls_emutls_destruct
 *
 * Description:
 *   Free the copies of the calling thread when it exits.
 *
 ****************************************************************************/

void tls_emutls_destruct(void)
{
  FAR struct tls_info_s *tls = tls_get_info();
  FAR struct emutls_array_s *array = tls->tl_emutls;
  uintptr_t i;

  if (array == NULL)
    {
      return;
    }

  tls->tl_emutls = NULL;
  for (i = 0; i < array->size; i++)
    {
      lib_free(array->data[i]);
    }

  lib_free(array);
}
