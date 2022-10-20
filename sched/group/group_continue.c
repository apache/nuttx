/****************************************************************************
 * sched/group/group_continue.c
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

  rtcb = nxsched_get_tcb(pid);
  if (rtcb != NULL)
    {
      up_unblock_task(rtcb);
    }

  /* Always return zero.  We need to visit each member of the group */

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
 *   Note: this function should used within critical_section
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
