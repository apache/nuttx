/****************************************************************************
 * libs/libc/pthread/pthread_setspecific.c
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

#include <nuttx/tls.h>

#if CONFIG_TLS_NELEM > 0

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_setspecific
 *
 * Description:
 *   The pthread_setspecific() function associates a thread-
 *   specific value with a key obtained via a previous call
 *   to pthread_key_create().  Different threads may bind
 *   different values to the same key.  These values are
 *   typically pointers to blocks of dynamically allocated
 *   memory that have been reserved for use by the calling
 *   thread.
 *
 *   The effect of calling pthread_setspecific() with
 *   a key value not obtained from pthread_key_create() or
 *   after a key has been deleted with pthread_key_delete()
 *   is undefined.
 *
 * Input Parameters:
 *   key = The data key to get or set
 *   value = The value to bind to the key.
 *
 * Returned Value:
 *   If successful, pthread_setspecific() will return zero (OK).
 *   Otherwise, an error number will be returned:
 *
 *      ENOMEM - Insufficient memory exists to associate
 *         the value with the key.
 *      EINVAL - The key value is invalid.
 *
 * POSIX Compatibility:
 *   - Both pthread_setspecific() and pthread_getspecific()
 *     may be called from a thread-specific data destructor
 *     function.
 *
 ****************************************************************************/

int pthread_setspecific(pthread_key_t key, FAR const void *value)
{
  int ret = tls_set_value((int)key, (uintptr_t)value);
  return ret < 0 ? -ret : 0;
}

#endif /* CONFIG_TLS_NELEM */

