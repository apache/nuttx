/****************************************************************************
 * sched/group/group_setupidlefiles.c
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

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sched.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/net/net.h>

#include "group/group.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: group_setupidlefiles
 *
 * Description:
 *   Configure the idle thread's TCB.
 *
 * Input Parameters:
 *   tcb - tcb of the idle task.
 *
 * Returned Value:
 *   0 is returned on success; a negated errno value is returned on a
 *   failure.
 *
 ****************************************************************************/

int group_setupidlefiles(FAR struct task_tcb_s *tcb)
{
#if defined(CONFIG_DEV_CONSOLE) || defined(CONFIG_DEV_NULL)
  int fd;
#endif

  DEBUGASSERT(tcb->cmn.group != NULL);

  /* Open stdin, dup to get stdout and stderr. This should always
   * be the first file opened and, hence, should always get file
   * descriptor 0.
   */

#if defined(CONFIG_DEV_CONSOLE) || defined(CONFIG_DEV_NULL)
#ifdef CONFIG_DEV_CONSOLE
  fd = nx_open("/dev/console", O_RDWR);
#else
  fd = nx_open("/dev/null", O_RDWR);
#endif
  if (fd == 0)
    {
      /* Successfully opened stdin (fd == 0) */

      nx_dup2(0, 1);
      nx_dup2(0, 2);
    }
  else
    {
      /* We failed to open stdin OR for some reason, we opened
       * it and got some file descriptor other than 0.
       */

      if (fd > 0)
        {
          sinfo("Open stdin fd: %d\n", fd);
          nx_close(fd);
        }
      else
        {
          serr("ERROR: Failed to open stdin: %d\n", fd);
        }

      return -ENFILE;
    }
#else
  /* This configuration can confuse user programs and libraries.
   * Eg. a program which opens a file and then prints something to
   * STDERR_FILENO (2) can end up with something undesirable if the
   * file descriptor for the file happens to be 2.
   * It's a common practice to keep 0-2 always open even if they are
   * /dev/null to avoid that kind of problems. Thus the following warning.
   */
#warning file descriptors 0-2 are not opened
#endif /* defined(CONFIG_DEV_CONSOLE) || defined(CONFIG_DEV_NULL) */

  /* Allocate file/socket streams for the TCB */

#ifdef CONFIG_FILE_STREAM
  return group_setupstreams(tcb);
#else
  return OK;
#endif
}
