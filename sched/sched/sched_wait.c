/****************************************************************************
 * sched/sched/sched_wait.c
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

#include <sys/wait.h>
#include <signal.h>
#include <errno.h>

#include <nuttx/sched.h>

#include "sched/sched.h"

#if defined(CONFIG_SCHED_WAITPID) && defined(CONFIG_SCHED_HAVE_PARENT)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_wait
 ****************************************************************************/

pid_t nx_wait(FAR int *stat_loc)
{
  return nx_waitpid(INVALID_PROCESS_ID, stat_loc, 0);
}

/****************************************************************************
 * Name: wait
 *
 * Description:
 *   The wait() function will suspend execution of the calling thread until
 *   status information for one of its terminated child processes is
 *   available, or until delivery of a signal whose action is either to
 *   execute a signal-catching function or to terminate the process. If more
 *   than one thread is suspended in wait() or waitpid() awaiting termination
 *   of the same process, exactly one thread will return the process status
 *   at the time of the target process termination. If status information is
 *   available prior to the call to wait(), return will be immediate.
 *
 *   The waitpid() function will behave identically to wait(), if the pid
 *   argument is INVALID_PROCESS_ID and the options argument is 0. Otherwise,
 *   its behaviour will be modified by the values of the pid and options
 *   arguments.
 *
 * Input Parameters:
 *   stat_loc - The location to return the exit status
 *
 * Returned Value:
 *   See waitpid();
 *
 ****************************************************************************/

pid_t wait(FAR int *stat_loc)
{
  /* wait() is a cancellation point, but nothings needs to be done for this
   * trivial case.
   */

  return waitpid(INVALID_PROCESS_ID, stat_loc, 0);
}

#endif /* CONFIG_SCHED_WAITPID && CONFIG_SCHED_HAVE_PARENT */
