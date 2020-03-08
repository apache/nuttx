/****************************************************************************
 * sched/exit.c
 *
 *   Copyright (C) 2007-2008, 2011-2012, 2018 Gregory Nutt. All rights
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

#include <stdlib.h>
#include <unistd.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/fs/fs.h>

#include "task/task.h"
#include "group/group.h"
#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: exit
 *
 * Description:
 *   The exit() function causes normal process termination and the value of
 *   status & 0377 to be returned to the parent.
 *
 *   All functions registered with atexit() and on_exit() are called, in the
 *   reverse order of their registration.
 *
 *   All open streams are flushed and closed.
 *
 ****************************************************************************/

void exit(int status)
{
  FAR struct tcb_s *tcb = this_task();

  /* Only the lower 8-bits of status are used */

  status &= 0xff;

#ifdef CONFIG_SCHED_EXIT_KILL_CHILDREN
  /* Kill all of the children of the group, preserving only this thread.
   * exit() is normally called from the main thread of the task.  pthreads
   * exit through a different mechanism.
   */

  group_killchildren((FAR struct task_tcb_s *)tcb);
#endif

  /* Perform common task termination logic.  This will get called again later
   * through logic kicked off by _exit().  However, we need to call it before
   * calling _exit() in order to handle atexit() and on_exit() callbacks and
   * so that we can flush buffered I/O (both of which may required
   * suspending).
   */

  nxtask_exithook(tcb, status, false);

  /* Then "really" exit.  Only the lower 8 bits of the exit status are used. */

  _exit(status);
}
