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
 * This version uses a cascaded task_tls + pthread_key approach:
 *   - task_tls index stored in obj->loc.offset (system-level, permanent)
 *   - task_tls value stores pthread_key+1 per task group (0 = not created)
 *   - pthread_getspecific/pthread_setspecific for per-thread data
 *
 * This avoids the problem of pthread keys being per-task-group while
 * __emutls_object is global.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <pthread.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/lib/lib.h>
#include <nuttx/tls.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The types and the layout of the control object follow the definitions of
 * emutls.c in libgcc, because the compiler emits the object and this file
 * only reads it.
 */

typedef unsigned int word __attribute__((mode(word)));

struct __emutls_object
{
  word size;
  word align;
  union
  {
    uintptr_t offset;  /* task_tls index + 1 (0 = not allocated) */
    FAR void *ptr;
  } loc;
  FAR void *templ;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Serializes the allocation of the task local storage index of a variable
 * and the creation of the pthread key of a task group.
 */

static pthread_mutex_t g_emutls_lock = PTHREAD_MUTEX_INITIALIZER;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: emutls_destroy
 *
 * Description:
 *   Frees the copy of a variable when the thread that used it exits.  It is
 *   the destructor of the pthread key of the variable.
 *
 * Input Parameters:
 *   ptr - The copy of the variable of the exiting thread
 *
 ****************************************************************************/

static void emutls_destroy(FAR void *ptr)
{
  lib_free(ptr);
}

/****************************************************************************
 * Name: emutls_alloc
 *
 * Description:
 *   Allocates the copy of a variable for the calling thread and gives it
 *   the initial value of the variable, which is the template of the object
 *   or zero when the variable has none.
 *
 * Input Parameters:
 *   obj - The control object of the variable
 *
 * Returned Value:
 *   The copy of the variable, or NULL when the allocation fails.
 *
 ****************************************************************************/

static FAR void *emutls_alloc(FAR struct __emutls_object *obj)
{
  FAR void *ret;

  if (obj->align > sizeof(FAR void *))
    {
      ret = lib_memalign(obj->align, obj->size);
    }
  else
    {
      ret = lib_malloc(obj->size);
    }

  if (ret != NULL)
    {
      if (obj->templ != NULL)
        {
          memcpy(ret, obj->templ, obj->size);
        }
      else
        {
          memset(ret, 0, obj->size);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: emutls_key
 *
 * Description:
 *   Returns the pthread key that holds the copies of the variable in the
 *   task group of the caller, and creates it on the first use.
 *
 * Input Parameters:
 *   tlsindex - The task local storage index of the variable
 *
 * Returned Value:
 *   The pthread key of the variable in this task group.  Panics when the
 *   key cannot be created, because the caller that the compiler emits has
 *   no way to report the failure.
 *
 ****************************************************************************/

static pthread_key_t emutls_key(int tlsindex)
{
  pthread_key_t key;
  uintptr_t value;

  /* Ensure this task group has a pthread key */

  value = task_tls_get_value(tlsindex);
  if (value == 0)
    {
      pthread_mutex_lock(&g_emutls_lock);
      value = task_tls_get_value(tlsindex);
      if (value == 0)
        {
          if (pthread_key_create(&key, emutls_destroy) != 0)
            {
              pthread_mutex_unlock(&g_emutls_lock);
              PANIC();
            }

          value = (uintptr_t)key + 1;
          if (task_tls_set_value(tlsindex, value) < 0)
            {
              pthread_key_delete(key);
              pthread_mutex_unlock(&g_emutls_lock);
              PANIC();
            }
        }

      pthread_mutex_unlock(&g_emutls_lock);
    }

  return (pthread_key_t)(value - 1);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __wrap___emutls_get_address
 *
 * Description:
 *   Return a per-thread pointer for the TLS variable described by obj.
 *   Overrides libgcc's single-threaded __emutls_get_address via --wrap
 *   linker option. Uses cascaded task_tls + pthread_key for per-thread
 *   isolation across task groups.
 *
 * Input Parameters:
 *   arg - Pointer to the __emutls_object descriptor (compiler-generated)
 *
 * Returned Value:
 *   Pointer to the per-thread TLS variable storage. Calls PANIC() on
 *   allocation failure since the compiler-generated caller cannot handle
 *   NULL.
 *
 ****************************************************************************/

FAR void *__wrap___emutls_get_address(FAR void *arg)
{
  FAR struct __emutls_object *obj = arg;
  pthread_key_t key;
  FAR void *ret;

  /* Ensure this object has a task_tls index allocated */

  if (obj->loc.offset == 0)
    {
      pthread_mutex_lock(&g_emutls_lock);
      if (obj->loc.offset == 0)
        {
          int tlsindex = task_tls_alloc(NULL);

          if (tlsindex < 0)
            {
              pthread_mutex_unlock(&g_emutls_lock);
              PANIC();
            }

          obj->loc.offset = (uintptr_t)tlsindex + 1;
        }

      pthread_mutex_unlock(&g_emutls_lock);
    }

  key = emutls_key((int)(obj->loc.offset - 1));

  /* Get or allocate per-thread storage */

  ret = pthread_getspecific(key);
  if (ret == NULL)
    {
      ret = emutls_alloc(obj);
      if (ret == NULL)
        {
          PANIC();
        }

      if (pthread_setspecific(key, ret) != 0)
        {
          lib_free(ret);
          PANIC();
        }
    }

  return ret;
}

/****************************************************************************
 * Name: __wrap___emutls_register_common
 *
 * Description:
 *   Register a TLS common symbol. Called by GCC during startup
 *   (.init_array, single-threaded context) for thread_local variables
 *   with common linkage. Updates the object's size, alignment, and
 *   initialization template using largest-wins semantics.
 *
 * Input Parameters:
 *   arg   - Pointer to the __emutls_object descriptor
 *   size  - Size of the TLS variable
 *   align - Alignment requirement of the TLS variable
 *   templ - Pointer to initialization template in .tdata (may be NULL)
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void __wrap___emutls_register_common(FAR void *arg, word size, word align,
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
