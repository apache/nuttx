/****************************************************************************
 * libs/libc/pthread/pthread_getspecific.c
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
#include <assert.h>

#include <nuttx/arch.h>
#include <nuttx/tls.h>

#if CONFIG_TLS_NELEM > 0

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_getspecific
 *
 * Description:
 *   The pthread_getspecific() function returns the value currently
 *   bound to the specified key on behalf of the calling thread.
 *
 *   The effect of calling pthread_getspecific() with a key value
 *   not obtained from pthread_key_create() or after a key has been
 *   deleted with pthread_key_delete() is undefined.
 *
 * Input Parameters:
 *   key = The data key to get or set
 *
 * Returned Value:
 *   The function pthread_getspecific() returns the thread-specific data
 *   associated with the given key.  If no thread specific data is
 *   associated with the key, then the value NULL is returned.
 *
 * POSIX Compatibility:
 *   - Both pthread_setspecific() and pthread_getspecific() may be
 *     called from a thread-specific data destructor function.
 *
 ****************************************************************************/

FAR void *pthread_getspecific(pthread_key_t key)
{
  FAR struct tls_info_s *info;
  FAR void *ret = NULL;

  DEBUGASSERT(key >= 0 && key < CONFIG_TLS_NELEM);
  if (key >= 0 && key < CONFIG_TLS_NELEM)
    {
      /* Get the TLS info structure from the current threads stack */

      info = up_tls_info();
      DEBUGASSERT(info != NULL);

      /* Get the element value from the TLS info. */

      ret = (FAR void *)info->tl_elem[key];
    }

  return ret;
}

#endif /* CONFIG_TLS_NELEM */
