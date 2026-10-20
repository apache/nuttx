/****************************************************************************
 * libs/libc/tls/tls_destruct.c
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

#include <assert.h>
#include <limits.h>
#include <stdbool.h>

#include <nuttx/tls.h>

#if defined(CONFIG_TLS_NELEM) && CONFIG_TLS_NELEM > 0

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tls_destruct
 *
 * Description:
 *   Destruct all TLS data element associated with allocated key
 *
 * Input Parameters:
 *    None
 *
 * Returned Value:
 *   A set of allocated TLS index
 *
 ****************************************************************************/

void tls_destruct(void)
{
  FAR struct task_info_s *info = task_get_info();
  FAR struct tls_info_s *tls = tls_get_info();
  FAR void *tls_elem_ptr;
  tls_dtor_t destructor;
  bool again = true;
  int iteration;
  int candidate;

  DEBUGASSERT(info != NULL);

  /* A destructor may set a value again: repeat, as POSIX requires */

  for (iteration = 0; again && iteration < PTHREAD_DESTRUCTOR_ITERATIONS;
       iteration++)
    {
      again = false;
      for (candidate = CONFIG_TLS_NELEM - 1; candidate >= 0; candidate--)
        {
          tls_elem_ptr = (FAR void *)tls->tl_elem[candidate];
          destructor = info->ta_tlsdtor[candidate];
          tls->tl_elem[candidate] = 0;
          if (tls_elem_ptr && destructor)
            {
              destructor(tls_elem_ptr);
              again = true;
            }
        }
    }

  /* Clear the values that the last destructors set */

  for (candidate = 0; candidate < CONFIG_TLS_NELEM; candidate++)
    {
      tls->tl_elem[candidate] = 0;
    }
}

#endif /* CONFIG_TLS_NELEM > 0 */
