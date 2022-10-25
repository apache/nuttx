/****************************************************************************
 * sched/semaphore/sem_recover.c
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

#include <assert.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/sched.h>

#include "semaphore/semaphore.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsem_recover
 *
 * Description:
 *   This function is called from nxtask_recover() when a task is deleted via
 *   task_delete() or via pthread_cancel().  It current only checks on the
 *   case where a task is waiting for semaphore at the time that is was
 *   killed.
 *
 *   REVISIT:  A more complete implementation would release counts on all
 *   semaphores held by the thread.  That would, however, require some
 *   significant extension to the semaphore data structures because given
 *   only the task, there is not mechanism to traverse all of the semaphores
 *   with counts held by the task.
 *
 * Input Parameters:
 *   tcb - The TCB of the terminated task or thread
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   This function is called from task deletion logic in a safe context.
 *
 ****************************************************************************/

void nxsem_recover(FAR struct tcb_s *tcb)
{
  irqstate_t flags;

  /* The task is being deleted.  If it is waiting for a semaphore, then
   * increment the count on the semaphores.  This logic is almost identical
   * to what you see in nxsem_wait_irq() except that no attempt is made to
   * restart the exiting task.
   *
   * NOTE:  In the case that the task is waiting we can assume: (1) That the
   * task state is TSTATE_WAIT_SEM and (2) that the 'waitobj' in the TCB is
   * non-null.  If we get here via pthread_cancel() or via task_delete(),
   * then the task state should be preserved; it will be altered in other
   * cases but in those cases waitobj should be NULL anyway (but we do not
   * enforce that here).
   */

  flags = enter_critical_section();
  if (tcb->task_state == TSTATE_WAIT_SEM ||
      tcb->task_state == TSTATE_WAIT_MUTEX)
    {
      FAR sem_t *sem = tcb->waitobj;
      DEBUGASSERT(sem != NULL && sem->semcount < 0);

      /* Restore the correct priority of all threads that hold references
       * to this semaphore.
       */

      nxsem_canceled(tcb, sem);

      /* And increment the count on the semaphore.  This releases the count
       * that was taken by sem_wait().  This count decremented the semaphore
       * count to negative and caused the thread to be blocked in the first
       * place.
       */

      sem->semcount++;
    }

  /* Release all semphore holders for the task */

  nxsem_release_all(tcb);

  leave_critical_section(flags);
}
