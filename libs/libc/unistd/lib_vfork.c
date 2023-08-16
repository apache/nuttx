/****************************************************************************
 * libs/libc/unistd/lib_vfork.c
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

#include <unistd.h>
#include <sys/wait.h>
#include <errno.h>
#include <debug.h>

#if defined(CONFIG_ARCH_HAVE_FORK) && defined(CONFIG_SCHED_WAITPID)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vfork
 *
 * Description:
 *   The vfork() function is implemented based on fork() function, on
 *   vfork(), the parent task need to wait until the child task is performing
 *   exec or running finished.
 *
 * Returned Value:
 *   Upon successful completion, vfork() returns 0 to the child process and
 *   returns the process ID of the child process to the parent process.
 *   Otherwise, -1 is returned to the parent, no child process is created,
 *   and errno is set to indicate the error.
 *
 ****************************************************************************/

pid_t vfork(void)
{
  int status = 0;
  int ret;
  pid_t pid = fork();

  if (pid != 0)
    {
      /* we are in parent task, and we need to wait the child task
       * until running finished or performing exec
       */

      ret = waitpid(pid, &status, WNOWAIT);
      if (ret < 0)
        {
          serr("ERROR: waitpid failed: %d\n", get_errno());
        }
    }

  return pid;
}

#endif /* CONFIG_ARCH_HAVE_FORK && CONFIG_SCHED_WAITPID */
