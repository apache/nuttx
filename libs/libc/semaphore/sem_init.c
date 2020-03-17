/****************************************************************************
 * libs/libc/sem/sem_init.c
 *
 *   Copyright (C) 2007-2009, 2011-2012, 2016-2017 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <limits.h>
#include <errno.h>

#include <nuttx/semaphore.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsem_init
 *
 * Description:
 *   This function initializes the UNNAMED semaphore sem. Following a
 *   successful call to nxsem_init(), the semaphore may be used in subsequent
 *   calls to nxsem_wait(), nxsem_post(), and nxsem_trywait().  The semaphore
 *   remains usable until it is destroyed.
 *
 *   Only sem itself may be used for performing synchronization. The result
 *   of referring to copies of sem in calls to nxsem_wait(), nxsem_trywait(),
 *   nxsem_post(), and nxsem_destroy() is undefined.
 *
 * Input Parameters:
 *   sem - Semaphore to be initialized
 *   pshared - Process sharing (not used)
 *   value - Semaphore initialization value
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 ****************************************************************************/

int nxsem_init(FAR sem_t *sem, int pshared, unsigned int value)
{
  /* Verify that a semaphore was provided and the count is within the valid
   * range.
   */

  if (sem != NULL && value <= SEM_VALUE_MAX)
    {
      /* Initialize the semaphore count */

      sem->semcount         = (int16_t)value;

      /* Initialize to support priority inheritance */

#ifdef CONFIG_PRIORITY_INHERITANCE
      sem->flags            = 0;
#  if CONFIG_SEM_PREALLOCHOLDERS > 0
      sem->hhead            = NULL;
#  else
      sem->holder[0].htcb   = NULL;
      sem->holder[0].counts = 0;
      sem->holder[1].htcb   = NULL;
      sem->holder[1].counts = 0;
#  endif
#endif
      return OK;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: sem_init
 *
 * Description:
 *   This function initializes the UNNAMED semaphore sem. Following a
 *   successful call to sem_init(), the semaphore may be used in subsequent
 *   calls to sem_wait(), sem_post(), and sem_trywait().  The semaphore
 *   remains usable until it is destroyed.
 *
 *   Only sem itself may be used for performing synchronization. The result
 *   of referring to copies of sem in calls to sem_wait(), sem_trywait(),
 *   sem_post(), and sem_destroy() is undefined.
 *
 * Input Parameters:
 *   sem - Semaphore to be initialized
 *   pshared - Process sharing (not used)
 *   value - Semaphore initialization value
 *
 * Returned Value:
 *   This returns zero (OK) if successful.  Otherwise, -1 (ERROR) is
 *   returned and the errno value is set appropriately.
 *
 ****************************************************************************/

int sem_init(FAR sem_t *sem, int pshared, unsigned int value)
{
  int ret;

  ret = nxsem_init(sem, pshared, value);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}
