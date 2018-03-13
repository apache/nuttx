/****************************************************************************
 * sched/semaphore/sem_setprotocol.c
 *
 *   Copyright (C) 2016-2017 Gregory Nutt. All rights reserved.
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

#include <assert.h>
#include <errno.h>

#include <nuttx/semaphore.h>

#include "semaphore/semaphore.h"

#ifdef CONFIG_PRIORITY_INHERITANCE

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsem_setprotocol
 *
 * Description:
 *    Set semaphore protocol attribute.
 *
 *    One particularly important use of this furnction is when a semaphore
 *    is used for inter-task communication like:
 *
 *      TASK A                 TASK B
 *      nxsem_init(sem, 0, 0);
 *      nxsem_wait(sem);
 *                             nxsem_post(sem);
 *      Awakens as holder
 *
 *    In this case priority inheritance can interfere with the operation of
 *    the semaphore.  The problem is that when TASK A is restarted it is a
 *    holder of the semaphore.  However, it never calls nxsem_post(sem) so it
 *    becomes *permanently* a holder of the semaphore and may have its
 *    priority boosted when any other task tries to acquire the semaphore.
 *
 *    The fix is to call nxsem_setprotocol(SEM_PRIO_NONE) immediately after
 *    the sem_init() call so that there will be no priority inheritance
 *    operations on this semaphore.
 *
 * Input Parameters:
 *    sem      - A pointer to the semaphore whose attributes are to be
 *               modified
 *    protocol - The new protocol to use
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 ****************************************************************************/

int nxsem_setprotocol(FAR sem_t *sem, int protocol)
{
  DEBUGASSERT(sem != NULL);

  switch (protocol)
    {
      case SEM_PRIO_NONE:
        /* Disable priority inheritance */

        sem->flags |= PRIOINHERIT_FLAGS_DISABLE;

        /* Remove any current holders */

        nxsem_destroyholder(sem);
        return OK;

      case SEM_PRIO_INHERIT:
        /* Enable priority inheritance (dangerous) */

        sem->flags &= ~PRIOINHERIT_FLAGS_DISABLE;
        return OK;

      case SEM_PRIO_PROTECT:
        /* Not yet supported */

        return -ENOSYS;

      default:
        break;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: sem_setprotocol
 *
 * Description:
 *    Set semaphore protocol attribute.
 *
 *    One particularly important use of this furnction is when a semaphore
 *    is used for inter-task communication like:
 *
 *      TASK A                 TASK B
 *      sem_init(sem, 0, 0);
 *      nxsem_wait(sem);
 *                             sem_post(sem);
 *      Awakens as holder
 *
 *    In this case priority inheritance can interfere with the operation of
 *    the semaphore.  The problem is that when TASK A is restarted it is a
 *    holder of the semaphore.  However, it never calls sem_post(sem) so it
 *    becomes *permanently* a holder of the semaphore and may have its
 *    priority boosted when any other task tries to acquire the semaphore.
 *
 *    The fix is to call sem_setprotocol(SEM_PRIO_NONE) immediately after
 *    the sem_init() call so that there will be no priority inheritance
 *    operations on this semaphore.
 *
 * Input Parameters:
 *    sem      - A pointer to the semaphore whose attributes are to be
 *               modified
 *    protocol - The new protocol to use
 *
 * Returned Value:
 *   This function is exposed as a non-standard application interface.  It
 *   returns zero (OK) if successful.  Otherwise, -1 (ERROR) is returned and
 *   the errno value is set appropriately.
 *
 ****************************************************************************/

int sem_setprotocol(FAR sem_t *sem, int protocol)
{
  int ret;

  ret = nxsem_setprotocol(sem, protocol);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}

#endif /* CONFIG_PRIORITY_INHERITANCE */
