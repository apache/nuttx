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
#ifdef CONFIG_DEV_CONSOLE
  int fd;
#endif

  DEBUGASSERT(tcb->cmn.group != NULL);

  /* Open stdin, dup to get stdout and stderr. This should always
   * be the first file opened and, hence, should always get file
   * descriptor 0.
   */

#ifdef CONFIG_DEV_CONSOLE
  fd = nx_open("/dev/console", O_RDWR);
  if (fd == 0)
    {
      /* Successfully opened /dev/console as stdin (fd == 0) */

      nx_dup2(0, 1);
      nx_dup2(0, 2);
    }
  else
    {
      /* We failed to open /dev/console OR for some reason, we opened
       * it and got some file descriptor other than 0.
       */

      if (fd > 0)
        {
          sinfo("Open /dev/console fd: %d\n", fd);
          nx_close(fd);
        }
      else
        {
          serr("ERROR: Failed to open /dev/console: %d\n", fd);
        }

      return -ENFILE;
    }
#endif

  /* Allocate file/socket streams for the TCB */

#ifdef CONFIG_FILE_STREAM
  return group_setupstreams(tcb);
#else
  return OK;
#endif
}
