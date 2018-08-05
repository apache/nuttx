/****************************************************************************
 * sched/pthread/pthread_initialize.c
 *
 *   Copyright (C) 2007-2010, 2013, 2017-2018 Gregory Nutt. All rights
 *     reserved.
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

#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/semaphore.h>

#include "pthread/pthread.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_initialize
 *
 * Description:
 *   This is an internal OS function called only at power-up boot time.  It
 *   no longer does anything since all of the pthread data structures have
 *   been moved into the "task group"
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void pthread_initialize(void)
{
}

/****************************************************************************
 * Name: pthread_sem_take, pthread_sem_trytake, and
 *       pthread_sem_give
 *
 * Description:
 *   Support managed access to the private data sets.
 *
 *   REVISIT: These functions really do nothing more than match the return
 *   value of the semaphore functions (0 or -1 with errno set) to the
 *   return value of more pthread functions (0 or errno).  A better solution
 *   would be to use an internal version of the semaphore functions that
 *   return the error value in the correct form.
 *
 * Input Parameters:
 *  sem  - The semaphore to lock or unlock
 *  intr - false: ignore EINTR errors when locking; true tread EINTR as
 *         other errors by returning the errno value
 *
 * Returned Value:
 *   0 on success or an errno value on failure.
 *
 ****************************************************************************/

int pthread_sem_take(sem_t *sem, bool intr)
{
  int ret;

  /* Verify input parameters */

  DEBUGASSERT(sem != NULL);
  if (sem != NULL)
    {
      do
        {
          /* Take the semaphore (perhaps waiting) */

          ret = nxsem_wait(sem);
          if (ret < 0)
            {
              /* The only cases that an error should occur here is if the wait
               * was awakened by a signal or if the thread was canceled during
               * the wait.
               */

              DEBUGASSERT(ret == -EINTR || ret == -ECANCELED);

              /* When the error occurs in this case, should we errout?  Or
               * should we just continue waiting until we have the
               * semaphore?
               */

              if (intr)
                {
                  return -ret;
                }
            }
        }
      while (ret == -EINTR);

      /* We have the semaphore (or some awful, unexpected error has
       * occurred).
       */

      return OK;
    }

  return EINVAL;
}

#ifdef CONFIG_PTHREAD_MUTEX_UNSAFE
int pthread_sem_trytake(sem_t *sem)
{
  int ret = EINVAL;

  /* Verify input parameters */

  DEBUGASSERT(sem != NULL);
  if (sem != NULL)
    {
      /* Try to take the semaphore */

      int status = nxsem_trywait(sem);
      ret = status < 0 ? -status : OK;
    }

  return ret;
}
#endif

int pthread_sem_give(sem_t *sem)
{
  int ret;

  /* Verify input parameters */

  DEBUGASSERT(sem != NULL);
  if (sem != NULL)
    {
      /* Give the semaphore */

      ret = nxsem_post(sem);
      if (ret < 0)
        {
          return -ret;
        }

      return OK;
    }
  else
    {
      /* NULL semaphore pointer! */

      return EINVAL;
    }
}
