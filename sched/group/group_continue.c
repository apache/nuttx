/****************************************************************************
 *  sched/group/group_continue.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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
#include <stdint.h>
#include <sched.h>
#include <pthread.h>

#include <nuttx/sched.h>

#include "sched/sched.h"
#include "group/group.h"

#ifdef HAVE_GROUP_MEMBERS

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: group_continue_handler
 *
 * Description:
 *   Callback from group_foreachchild that handles one member of the group.
 *
 * Input Parameters:
 *   pid - The ID of the group member that may be resumed.
 *   arg - Unused
 *
 * Returned Value:
 *   0 (OK) always
 *
 ****************************************************************************/

static int group_continue_handler(pid_t pid, FAR void *arg)
{
  FAR struct tcb_s *rtcb;

  /* Resume all threads */

  rtcb = sched_gettcb(pid);
  if (rtcb != NULL)
    {
      sched_continue(rtcb);
    }

  /* Always return zero.  We need to visit each member of the group*/

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: group_continue
 *
 * Description:
 *   Resume all members of the task group.  This is SIGCONT default signal
 *   action logic.
 *
 * Input Parameters:
 *   tcb - TCB of the task to be retained.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

int group_continue(FAR struct tcb_s *tcb)
{
  int ret;

  /* Lock the scheduler so that there this thread will not lose priority
   * until all of its children are suspended.
   */

  sched_lock();
  ret = group_foreachchild(tcb->group, group_continue_handler, NULL);
  sched_unlock();
  return ret;
}

#endif /* HAVE_GROUP_MEMBERS */
