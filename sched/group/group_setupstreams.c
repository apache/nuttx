/****************************************************************************
 * sched/group/group_setupstreams.c
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

#include <sched.h>
#include <fcntl.h>
#include <assert.h>

#include <nuttx/fs/fs.h>
#include <nuttx/net/net.h>

#include "group/group.h"

/* Make sure that there are file or socket descriptors in the system and
 * that some number of streams have been configured.
 */

#ifdef CONFIG_FILE_STREAM

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: group_setupstreams
 *
 * Description:
 *   Setup streams data structures that may be used for standard C buffered
 *   I/O with underlying socket or file descriptors
 *
 ****************************************************************************/

int group_setupstreams(FAR struct task_tcb_s *tcb)
{
  DEBUGASSERT(tcb && tcb->cmn.group);

  /* fdopen to get the stdin, stdout and stderr streams. The following logic
   * depends on the fact that the library layer will allocate FILEs in order.
   *
   * fd = 0 is stdin  (read-only)
   * fd = 1 is stdout (write-only, append)
   * fd = 2 is stderr (write-only, append)
   */

  fs_fdopen(0, O_RDONLY,         (FAR struct tcb_s *)tcb, NULL);
  fs_fdopen(1, O_WROK | O_CREAT, (FAR struct tcb_s *)tcb, NULL);
  fs_fdopen(2, O_WROK | O_CREAT, (FAR struct tcb_s *)tcb, NULL);

  return OK;
}

#endif /* CONFIG_FILE_STREAM */
