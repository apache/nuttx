/****************************************************************************
 * sched/semaphore/sem_trywait.c
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

#include <stdbool.h>
#include <sched.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/init.h>
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
 *   currently not locked.  In either case, the call returns without
 *   blocking.
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

  /* This API should not be called from interrupt handlers & idleloop */

  DEBUGASSERT(sem != NULL && up_interrupt_context() == false);
  DEBUGASSERT(!OSINIT_IDLELOOP() || !sched_idletask());

  /* The following operations must be performed with interrupts disabled
   * because sem_post() may be called from an interrupt handler.
   */

  flags = enter_critical_section();

  /* If the semaphore is available, give it to the requesting task */

  if (sem->semcount > 0)
    {
      /* It is, let the task take the semaphore */

      sem->semcount--;
      nxsem_add_holder(sem);
      rtcb->waitobj = NULL;
      ret = OK;
    }
  else
    {
      /* Semaphore is not available */

      ret = -EAGAIN;
    }

  /* Interrupts may now be enabled. */

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: sem_trywait
 *
 * Description:
 *   This function locks the specified semaphore only if the semaphore is
 *   currently not locked.  In either case, the call returns without
 *   blocking.
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

  if (sem == NULL)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  /* Let nxsem_trywait do the real work */

  ret = nxsem_trywait(sem);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}
