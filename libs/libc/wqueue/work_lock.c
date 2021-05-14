/****************************************************************************
 * libs/libc/wqueue/work_lock.c
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
#include <errno.h>

#include <nuttx/semaphore.h>

#include "wqueue/wqueue.h"

#if defined(CONFIG_LIB_USRWORK) && !defined(__KERNEL__)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: work_lock
 *
 * Description:
 *   Lock the user-mode work queue.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success, a negated errno on failure.  This error may be
 *   reported:
 *
 *   -EINTR - Wait was interrupted by a signal
 *
 ****************************************************************************/

int work_lock(void)
{
  int ret;

#ifdef CONFIG_BUILD_PROTECTED
  ret = _SEM_WAIT(&g_usrsem);
  if (ret < 0)
    {
      DEBUGASSERT(_SEM_ERRNO(ret) == EINTR ||
                  _SEM_ERRNO(ret) == ECANCELED);
      return -EINTR;
    }
#else
  ret = pthread_mutex_lock(&g_usrmutex);
  if (ret != 0)
    {
      DEBUGASSERT(ret == EINTR);
      return -EINTR;
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: work_unlock
 *
 * Description:
 *   Unlock the user-mode work queue.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void work_unlock(void)
{
#ifdef CONFIG_BUILD_PROTECTED
  _SEM_POST(&g_usrsem);
#else
  pthread_mutex_unlock(&g_usrmutex);
#endif
}

#endif /* CONFIG_LIB_USRWORK && !__KERNEL__*/
