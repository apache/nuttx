/****************************************************************************
 * libs/libc/tls/tls_alloc.c
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

#include <sched.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/spinlock.h>
#include <nuttx/tls.h>
#include <nuttx/sched.h>

#if CONFIG_TLS_NELEM > 0

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tls_alloc
 *
 * Description:
 *   Allocate a group-unique TLS data index
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   A TLS index that is unique for use within this task group.
 *   If unsuccessful, an errno value will be returned and set to errno.
 *
 ****************************************************************************/

int tls_alloc(CODE void (*dtor)(FAR void *))
{
  FAR struct task_info_s *info = task_get_info();
  int candidate;
  int ret;

  DEBUGASSERT(info);

  /* Search for an unused index.  This is done in a critical section here to
   * avoid concurrent modification of the group TLS index set.
   */

  ret = _SEM_WAIT(&info->ta_sem);

  if (ret < 0)
    {
      ret = _SEM_ERRVAL(ret);
      return ret;
    }

  ret = -EAGAIN;

  for (candidate = 0; candidate < CONFIG_TLS_NELEM; candidate++)
    {
      /* Is this candidate index available? */

      tls_ndxset_t mask = (1 << candidate);
      if ((info->ta_tlsset & mask) == 0)
        {
          /* Yes.. allocate the index and break out of the loop */

          info->ta_tlsset |= mask;
          info->ta_tlsdtor[candidate] = dtor;
          ret = candidate;
          break;
        }
    }

  _SEM_POST(&info->ta_sem);

  return ret;
}

#endif /* CONFIG_TLS_NELEM > 0 */
