/****************************************************************************
 * sched/task/exit.c
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
 * Name: _exit
 *
 * Description:
 *   This function causes the currently executing task to cease
 *   to exist.  This is a special case of task_delete() where the task to
 *   be deleted is the currently executing task.  It is more complex because
 *   a context switch must be perform to the next ready to run task.
 *
 ****************************************************************************/

void _exit(int status)
{
  FAR struct tcb_s *tcb = this_task();

  /* Only the lower 8-bits of status are used */

  status &= 0xff;

#ifdef HAVE_GROUP_MEMBERS
  /* Kill all of the children of the group, preserving only this thread.
   * exit() is normally called from the main thread of the task.  pthreads
   * exit through a different mechanism.
   */

  group_kill_children(tcb);
#endif

  /* Perform common task termination logic.  This will get called again later
   * through logic kicked off by up_exit().  However, we need to call it here
   * so that we can flush buffered I/O (both of which may required
   * suspending). This will be fixed later when I/O flush is moved to libc.
   */

  nxtask_exithook(tcb, status, false);

  up_exit(status);
}
