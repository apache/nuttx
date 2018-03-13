/****************************************************************************
 * sched/semaphore/sem_trywait.c
 *
 *   Copyright (C) 2007-2009, 2016-2017 Gregory Nutt. All rights reserved.
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

#include <stdbool.h>
#include <semaphore.h>
#include <sched.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "sched/sched.h"
#include "semaphore/semaphore.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsem_trywait
 *
 * Description:
 *   This function locks the specified semaphore only if the semaphore is
 *   currently not locked.  Otherwise, it locks the semaphore.  In either
 *   case, the call returns without blocking.
 *
 * Input Parameters:
 *   sem - the semaphore descriptor
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   Possible returned errors:
 *
 *     EINVAL - Invalid attempt to get the semaphore
 *     EAGAIN - The semaphore is not available.
 *
 * Assumptions:
 *
 ****************************************************************************/

int nxsem_trywait(FAR sem_t *sem)
{
  FAR struct tcb_s *rtcb = this_task();
  irqstate_t flags;
  int ret;

  /* This API should not be called from interrupt handlers */

  DEBUGASSERT(sem != NULL && up_interrupt_context() == false);

  if (sem != NULL)
    {
      /* The following operations must be performed with interrupts disabled
       * because sem_post() may be called from an interrupt handler.
       */

      flags = enter_critical_section();

      /* If the semaphore is available, give it to the requesting task */

      if (sem->semcount > 0)
        {
          /* It is, let the task take the semaphore */

          sem->semcount--;
          rtcb->waitsem = NULL;
          ret = OK;
        }
      else
        {
          /* Semaphore is not available */

          ret = -EAGAIN;
        }

      /* Interrupts may now be enabled. */

      leave_critical_section(flags);
    }
  else
    {
      ret = -EINVAL;
    }

  return ret;
}

/****************************************************************************
 * Name: sem_trywait
 *
 * Description:
 *   This function locks the specified semaphore only if the semaphore is
 *   currently not locked.  Otherwise, it locks the semaphore.  In either
 *   case, the call returns without blocking.
 *
 * Input Parameters:
 *   sem - the semaphore descriptor
 *
 * Returned Value:
 *   Zero (OK) on success or -1 (ERROR) if unsuccessful. If this function
 *   returns -1(ERROR), then the cause of the failure will be reported in
 *   errno variable as:
 *
 *     EINVAL - Invalid attempt to get the semaphore
 *     EAGAIN - The semaphore is not available.
 *
 ****************************************************************************/

int sem_trywait(FAR sem_t *sem)
{
  int ret;

  /* Let nxsem_trywait do the real work */

  ret = nxsem_trywait(sem);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}
