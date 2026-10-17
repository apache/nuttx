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

#include <pthread.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "libc.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The control object that GCC emits for a __thread variable when the
 * target has no native TLS, as defined by libgcc's emutls.c.
 */

struct __emutls_object
{
  size_t size;
  size_t align;
  union
  {
    uintptr_t offset;
    FAR void *ptr;
  } loc;
  FAR void *templ;
};

/* The copies of the variables that one thread has used */

struct emutls_array_s
{
  uintptr_t size;
  FAR void *data[1];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static pthread_once_t g_emutls_once = PTHREAD_ONCE_INIT;
static pthread_mutex_t g_emutls_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_key_t g_emutls_key;
static uintptr_t g_emutls_count;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: emutls_destroy
 *
 * Description:
 *   Free the copies of a thread when the thread exits.
 *
 ****************************************************************************/

static void emutls_destroy(FAR void *arg)
{
  FAR struct emutls_array_s *array = arg;
  uintptr_t i;

  for (i = 0; i < array->size; i++)
    {
      lib_free(array->data[i]);
    }

  lib_free(array);
}

/****************************************************************************
 * Name: emutls_init
 ****************************************************************************/

static void emutls_init(void)
{
  if (pthread_key_create(&g_emutls_key, emutls_destroy) != 0)
    {
      abort();
    }
}

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
 * Name: emutls_getarray
 *
 * Description:
 *   Return the array of the calling thread, large enough for index.
 *
 ****************************************************************************/

static FAR struct emutls_array_s *emutls_getarray(uintptr_t index)
{
  FAR struct emutls_array_s *array = pthread_getspecific(g_emutls_key);
  FAR struct emutls_array_s *newarray;
  uintptr_t oldsize = array != NULL ? array->size : 0;
  uintptr_t newsize;

  if (index <= oldsize)
    {
      return array;
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
  pthread_setspecific(g_emutls_key, newarray);
  return newarray;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __emutls_get_address
 *
 * Description:
 *   Return the address of the calling thread's copy of a __thread
 *   variable.  It replaces libgcc's version, which keeps a single copy
 *   when libgcc is built without thread support.  GCC declares it as a
 *   builtin with a void pointer argument.
 *
 ****************************************************************************/

FAR void *__emutls_get_address(FAR void *arg)
{
  FAR struct __emutls_object *obj = arg;
  FAR struct emutls_array_s *array;
  uintptr_t index;

  index = __atomic_load_n(&obj->loc.offset, __ATOMIC_ACQUIRE);
  if (index == 0)
    {
      pthread_once(&g_emutls_once, emutls_init);

      pthread_mutex_lock(&g_emutls_lock);
      index = obj->loc.offset;
      if (index == 0)
        {
          index = ++g_emutls_count;
          __atomic_store_n(&obj->loc.offset, index, __ATOMIC_RELEASE);
        }

      pthread_mutex_unlock(&g_emutls_lock);
    }

  array = emutls_getarray(index);
  if (array->data[index - 1] == NULL)
    {
      array->data[index - 1] = emutls_alloc(obj);
    }

  return array->data[index - 1];
}

/****************************************************************************
 * Name: __emutls_register_common
 *
 * Description:
 *   Merge the size, alignment and template of a common __thread variable
 *   that more than one object file defines.
 *
 ****************************************************************************/

void __emutls_register_common(FAR void *arg, size_t size, size_t align,
                              FAR void *templ)
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
