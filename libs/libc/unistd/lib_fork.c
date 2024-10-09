/****************************************************************************
 * libs/libc/unistd/lib_fork.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <nuttx/arch.h>
#include <nuttx/tls.h>

#include <unistd.h>
#include <stdio.h>
#include <sys/wait.h>
#include <errno.h>
#include <debug.h>

#if defined(CONFIG_ARCH_HAVE_FORK)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_PTHREAD_ATFORK
/****************************************************************************
 * Name:  atfork_prepare
 *
 * Description:
 *    Invoke this method in the parent process before fork starts
 *
 ****************************************************************************/

static void atfork_prepare(void)
{
  FAR struct task_info_s *info = task_get_info();
  FAR struct list_node *list = &info->ta_atfork;
  FAR struct pthread_atfork_s *entry;

  /* According to posix standard, the prepare handlers are called in reverse
   * order of registration
   * so we iterate over the func list in reverse order
   */

  nxmutex_lock(&info->ta_lock);
  list_for_every_entry_reverse(list, entry,
                               struct pthread_atfork_s, node)
    {
       if (entry->prepare != NULL)
         {
           entry->prepare();
         }
    }

  nxmutex_unlock(&info->ta_lock);
}

/****************************************************************************
 * Name:  atfork_child
 *
 * Description:
 *    Invoke this method in the child process after fork completes
 *
 ****************************************************************************/

static void atfork_child(void)
{
  FAR struct task_info_s *info = task_get_info();
  FAR struct list_node *list = &info->ta_atfork;
  FAR struct pthread_atfork_s *entry;

  /* The parent handlers are called in the order of registration */

  nxmutex_lock(&info->ta_lock);
  list_for_every_entry(list, entry,
                       struct pthread_atfork_s, node)
    {
       if (entry->child != NULL)
         {
           entry->child();
         }
    }

  nxmutex_unlock(&info->ta_lock);
}

/****************************************************************************
 * Name:  atfork_parent
 *
 * Description:
 *    Invoke this method in the parent process after fork completes
 *
 ****************************************************************************/

static void atfork_parent(void)
{
  FAR struct task_info_s *info = task_get_info();
  FAR struct list_node *list = &info->ta_atfork;
  FAR struct pthread_atfork_s *entry;

  /* The child handlers are called in the order of registration */

  nxmutex_lock(&info->ta_lock);
  list_for_every_entry(list, entry,
                       struct pthread_atfork_s, node)
    {
      if (entry->parent != NULL)
        {
          entry->parent();
        }
    }

  nxmutex_unlock(&info->ta_lock);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fork
 *
 * Description:
 *   The fork() function is a wrapper of up_fork() syscall
 *
 * Returned Value:
 *   Upon successful completion, fork() returns 0 to the child process and
 *   returns the process ID of the child process to the parent process.
 *   Otherwise, -1 is returned to the parent, no child process is created,
 *   and errno is set to indicate the error.
 *
 ****************************************************************************/

pid_t fork(void)
{
  pid_t pid;

#ifdef CONFIG_PTHREAD_ATFORK
  atfork_prepare();
#endif
  pid = up_fork();

#ifdef CONFIG_PTHREAD_ATFORK
  if (pid == 0)
    {
      atfork_child();
    }
  else
    {
      atfork_parent();
    }
#endif

  return pid;
}

#if defined(CONFIG_SCHED_WAITPID)

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
  pid_t pid;

#ifdef CONFIG_PTHREAD_ATFORK
  atfork_prepare();
#endif
  pid = up_fork();

#ifdef CONFIG_PTHREAD_ATFORK
  if (pid == 0)
    {
      atfork_child();
    }
  else
    {
      atfork_parent();
    }
#endif

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

#endif /* CONFIG_SCHED_WAITPID */

#endif /* CONFIG_ARCH_HAVE_FORK */
