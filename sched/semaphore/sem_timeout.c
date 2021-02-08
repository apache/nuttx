/****************************************************************************
 * sched/semaphore/sem_timeout.c
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

#include <errno.h>

#include <nuttx/sched.h>
#include <nuttx/wdog.h>

#include <nuttx/irq.h>

#include "semaphore/semaphore.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsem_timeout
 *
 * Description:
 *   This function is called if the timeout elapses before before a
 *   semaphore is acquired.
 *
 * Input Parameters:
 *   pid  - The task ID of the task to wakeup
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - Called from the context of the timer interrupt handler.
 *
 ****************************************************************************/

void nxsem_timeout(wdparm_t pid)
{
  FAR struct tcb_s *wtcb;
  irqstate_t flags;

  /* Disable interrupts to avoid race conditions */

  flags = enter_critical_section();

  /* Get the TCB associated with this PID.  It is possible that
   * task may no longer be active when this watchdog goes off.
   */

  wtcb = nxsched_get_tcb(pid);

  /* It is also possible that an interrupt/context switch beat us to the
   * punch and already changed the task's state.
   */

  if (wtcb && wtcb->task_state == TSTATE_WAIT_SEM)
    {
      /* Cancel the semaphore wait */

      nxsem_wait_irq(wtcb, ETIMEDOUT);
    }

  /* Interrupts may now be enabled. */

  leave_critical_section(flags);
}
