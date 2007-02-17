/************************************************************
 * sem_wait.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
 ************************************************************/

/************************************************************
 * Included Files
 ************************************************************/

#include <sys/types.h>
#include <semaphore.h>
#include <errno.h>
#include <assert.h>
#include <nuttx/arch.h>
#include "os_internal.h"
#include "sem_internal.h"

/************************************************************
 * Compilation Switches
 ************************************************************/

/************************************************************
 * Definitions
 ************************************************************/

/************************************************************
 * Private Type Declarations
 ************************************************************/

/************************************************************
 * Global Variables
 ************************************************************/

/************************************************************
 * Private Variables
 ************************************************************/

/************************************************************
 * Private Functions
 ************************************************************/

/************************************************************
 * Public Functions
 ************************************************************/

/************************************************************
 * Function:  sem_wait
 *
 * Description:
 *   This function attempts to lock the semaphore referenced
 *   by sem.  If the semaphore value is (<=) zero, then the
 *   calling task will not return until it successfully
 *   acquires the lock.
 *
 * Parameters:
 *   sem - Semaphore descriptor.
 *
 * Return Value:
 *   0 (OK), or -1 (ERROR) is unsuccessful
 *   If this function returns -1 (ERROR), then the cause
 *   of the failure will be reported in "errno" as:
 *   - EINVAL:  Invalid attempt to get the semaphore
 *   - EINTR:   The wait was interrupted by the receipt of
 *              a signal.
 *
 * Assumptions:
 *
 ************************************************************/

int sem_wait (sem_t *sem)
{
  _TCB  *rtcb = (_TCB*)g_readytorun.head;
  int    ret = ERROR;
  uint32 saved_state;

  /* Assume any errors reported are due to invalid arguments. */

  *get_errno_ptr() = EINVAL;

  if (sem)
    {
      /* The following operations must be performed with interrupts
       * disabled because sem_post() may be called from an interrupt
       * handler.
       */

      saved_state = irqsave();

      /* Check if the lock is available */

      if (sem->semcount > 0)
        {
          /* It is, let the task take the semaphore. */

          sem->semcount--;
          rtcb->waitsem = NULL;
          ret = OK;
        }

      /* The semaphore is NOT available, We will have to block the
       * current thread of execution.
       */

      else
        {
          /* First, verify that the task is not already waiting on a
           * semaphore
           */

          if (rtcb->waitsem != NULL)
            {
              PANIC(OSERR_BADWAITSEM);
            }

          /* Handle the POSIX semaphore */

          sem->semcount--;

          /* Save the waited on semaphore in the TCB */

          rtcb->waitsem = sem;

          /* Add the TCB to the prioritized semaphore wait queue */

          up_block_task(rtcb, TSTATE_WAIT_SEM);

          /* When we resume at this point, either (1) the semaphore has been
           * assigned to this thread of execution, or (2) the semaphore wait
           * has been interrupted by a signal.
           */

          if (*get_errno_ptr() != EINTR)
            ret = OK;
          else
            sem->semcount++;
        }

      /* Interrupts may now be enabled. */

      irqrestore(saved_state);
    }

  return ret;
}
