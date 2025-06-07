/****************************************************************************
 * fs/vfs/fs_dup3.c
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

#include <nuttx/fs/fs.h>

#include <unistd.h>
#include <sched.h>
#include <errno.h>

#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dup3
 *
 * Description:
 *   Clone a file descriptor or socket descriptor to a specific descriptor
 *   number and specific flags.
 *
 ****************************************************************************/

int dup3(int fd1, int fd2, int flags)
{
  int ret;

  ret = fdlist_dup3(nxsched_get_fdlist_from_tcb(this_task()),
                    fd1, fd2, flags);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}
