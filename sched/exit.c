/****************************************************************************
 * sched/exit.c
 *
 *   Copyright (C) 2007, 2008, 2011 Gregory Nutt. All rights reserved.
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

#include <stdlib.h>
#include <unistd.h>
#include <debug.h>
#include <errno.h>
#include <nuttx/fs.h>
#include "os_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functionss
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: exit
 *
 * Description:
 *   The  exit()  function  causes normal process termination
 *   and the value of status & 0377 is returned to the parent.
 *
 *   All functions registered with atexit() and on_exit() are
 *   called, in the reverse order  of  their  registration.
 *
 *   All open streams are flushed and closed.  Files created
 *   by tmpfile() are  removed.
 *
 ****************************************************************************/

void exit(int status)
{
  _TCB *tcb = (_TCB*)g_readytorun.head;

  /* Only the lower 8 bits of the exit status are used */

  status &= 0xff;

  /* Flush all streams (File descriptors will be closed when
   * the TCB is deallocated.
   */

#if CONFIG_NFILE_STREAMS > 0
  (void)lib_flushall(tcb->streams);
#endif

  /* Wakeup any tasks waiting for this task to exit */

#ifdef CONFIG_SCHED_WAITPID /* Experimental */
  while (tcb->exitsem.semcount < 0)
    {
      /* "If more than one thread is suspended in waitpid() awaiting
       *  termination of the same process, exactly one thread will return
       *  the process status at the time of the target process termination." 
       *  Hmmm.. what do we return to the others?
       */

      if (tcb->stat_loc)
        {
          *tcb->stat_loc = status << 8;
           tcb->stat_loc = NULL;
        }

      /* Wake up the thread */

      sem_post(&tcb->exitsem);
    }
#endif

  /* If an exit function was registered, call it now. */

#ifdef CONFIG_SCHED_ATEXT
  if (tcb->exitfunc)
    {
      (*tcb->exitfunc)();
    }
#endif

  /* Then "really" exit */

  _exit(status);
}
