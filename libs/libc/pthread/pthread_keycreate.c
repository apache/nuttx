/****************************************************************************
 * libs/libc/pthread/pthread_keycreate.c
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

#include <nuttx/tls.h>

#if CONFIG_TLS_NELEM > 0

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_key_create
 *
 * Description:
 *   This function creates a thread-specific data key visible to all threads
 *   in the system.  Although the same key value may be used by different
 *   threads, the values bound to the key by pthread_setspecific() are
 *   maintained on a per-thread basis and persist for the life of the calling
 *   thread.
 *
 *   Upon key creation, the value NULL will be associated with the new key
 *   in all active threads.  Upon thread creation, the value NULL will be
 *   associated with all defined keys in the new thread.
 *
 * Input Parameters:
 *   key        - A pointer to the key to create.
 *   destructor - An optional destructor() function that may be associated
 *                with each key that is invoked when a thread exits.
 *
 * Returned Value:
 *   If successful, the pthread_key_create() function will store the newly
 *   created key value at *key and return zero (OK).  Otherwise, an error
 *   number will be returned to indicate the error:
 *
 *      EAGAIN  - The system lacked sufficient resources to create another
 *                thread-specific data key, or the system-imposed limit on
 *                the total number of keys pers process {PTHREAD_KEYS_MAX}
 *                has been exceeded
 *      ENOMEM  - Insufficient memory exist to create the key.
 *
 ****************************************************************************/

int pthread_key_create(FAR pthread_key_t *key,
                       CODE void (*destructor)(FAR void *))
{
  int tlsindex;

  DEBUGASSERT(key != NULL);

  /* Allocate a TLS index */

  tlsindex = tls_alloc(destructor);

  /* Check if found a TLS index. */

  if (tlsindex >= 0)
    {
      *key = tlsindex;
      return OK;
    }

  return -tlsindex;
}

#endif /* CONFIG_TLS_NELEM */
