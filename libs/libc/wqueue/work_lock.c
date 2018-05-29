/****************************************************************************
 * libs/libc/wqueue/work_lock.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <pthread.h>
#include <semaphore.h>
#include <assert.h>
#include <errno.h>

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
  ret = sem_wait(&g_usrsem);
  if (ret < 0)
    {
      DEBUGASSERT(errno == EINTR || errno == ECANCELED);
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
  (void)sem_post(&g_usrsem);
#else
  (void)pthread_mutex_unlock(&g_usrmutex);
#endif
}

#endif /* CONFIG_LIB_USRWORK && !__KERNEL__*/
