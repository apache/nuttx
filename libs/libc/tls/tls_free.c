/****************************************************************************
 * libs/libc/tls/tls_free.c
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

#include <nuttx/spinlock.h>
#include <nuttx/tls.h>
#include <nuttx/sched.h>

#if CONFIG_TLS_NELEM > 0

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tls_free
 *
 * Description:
 *   Release a group-unique TLS data index previous obtained by tls_alloc()
 *
 * Input Parameters:
 *   tlsindex - The previously allocated TLS index to be freed
 *
 * Returned Value:
 *   OK is returned on success;
 *   If unsuccessful an errno value will be returned and set to errno.
 *     -EINVAL    - the index to be freed is out of range.
 *     -EINTR     - the wait operation interrupted by signal
 *     -ECANCELED - the thread was canceled during waiting
 *
 ****************************************************************************/

int tls_free(int tlsindex)
{
  FAR struct task_info_s *info = task_get_info();
  tls_ndxset_t mask;
  int ret = -EINVAL;

  DEBUGASSERT((unsigned)tlsindex < CONFIG_TLS_NELEM && info != NULL);
  if ((unsigned)tlsindex < CONFIG_TLS_NELEM)
    {
      /* This is done while holding a semaphore here to avoid concurrent
       * modification of the group TLS index set.
       */

      mask  = (1 << tlsindex);

      ret = _SEM_WAIT(&info->ta_sem);
      if (ret == OK)
        {
          DEBUGASSERT((info->ta_tlsset & mask) != 0);
          info->ta_tlsset &= ~mask;
          _SEM_POST(&info->ta_sem);
        }
      else
        {
          ret = _SEM_ERRVAL(ret);
        }
    }

  return ret;
}

#endif /* CONFIG_TLS_NELEM > 0 */
