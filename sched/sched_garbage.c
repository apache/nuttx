/****************************************************************************
 * sched/work_garbage.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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

#include <stdlib.h>

#include <nuttx/mm.h>

#include "os_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: sched_garbagecollection
 *
 * Description:
 *   Clean-up memory de-allocations that we queued because they could not
 *   be freed in that execution context (for example, if the memory was freed
 *   from an interrupt handler).
 *
 *   This logic may be called from the worker thread (see work_thread.c).
 *   If, however, CONFIG_SCHED_WORKQUEUE is not defined, then this logic will
 *   be called from the IDLE thread.  It is less optimal for the garbage
 *   collection to be called from the IDLE thread because it runs at a very
 *   low priority and could cause false memory out conditions.
 *
 * Input parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sched_garbagecollection(void)
{
  /* Check if there is anything in the delayed deallocation list. If there
   * is deallocate it now.  We must have exclusive access to the memory manager
   * to do this BUT the idle task cannot wait on a semaphore.  So we only do
   * the cleanup now if we can get the semaphore -- and this should be possible
   * because if the IDLE thread is running, no other task is!
   */

#ifdef CONFIG_SCHED_WORKQUEUE
  mm_takesemaphore();
#else
  if (mm_trysemaphore() == 0)
#endif
    {
      while (g_delayeddeallocations.head)
        {
          /* Remove the first delayed deallocation. */

          irqstate_t saved_state = irqsave();
          void *address = (void*)sq_remfirst((FAR sq_queue_t*)&g_delayeddeallocations);
          irqrestore(saved_state);

          /* Then deallocate it */

          if (address)
            {
              free(address);
            }
        }
      mm_givesemaphore();
    }
}

