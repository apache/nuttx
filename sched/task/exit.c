/****************************************************************************
 * sched/exit.c
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
#include "pthread/pthread.h"

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
  up_exit(status);
}

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

#ifdef HAVE_GROUP_MEMBERS
  /* Kill all of the children of the group, preserving only this thread.
   * exit() is normally called from the main thread of the task.  pthreads
   * exit through a different mechanism.
   */

  group_kill_children(tcb);
#endif

#ifdef CONFIG_PTHREAD_CLEANUP
  /* Perform any stack pthread clean-up callbacks */

  pthread_cleanup_popall(tcb);
#endif

#if !defined(CONFIG_DISABLE_PTHREAD) && !defined(CONFIG_PTHREAD_MUTEX_UNSAFE)
  /* Recover any mutexes still held by the canceled thread */

  pthread_mutex_inconsistent(tcb);
#endif

  /* Perform common task termination logic.  This will get called again later
   * through logic kicked off by _exit().  However, we need to call it before
   * calling _exit() in order to handle atexit() and on_exit() callbacks and
   * so that we can flush buffered I/O (both of which may required
   * suspending).
   */

  nxtask_exithook(tcb, status, false);

  /* Then "really" exit.  Only the lower 8 bits of the exit status are
   * used.
   */

  _exit(status);
}
