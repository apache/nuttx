/****************************************************************************
 * fs/vfs/fs_dup.c
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

#include <unistd.h>
#include <sched.h>
#include <assert.h>
#include <errno.h>
#include <fcntl.h>

#include <nuttx/fs/fs.h>

#include "inode/inode.h"
#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: file_dup
 *
 * Description:
 *   Equivalent to the standard dup() function except that it
 *   accepts a struct file instance instead of a file descriptor.
 *
 * Returned Value:
 *   The new file descriptor is returned on success; a negated errno value
 *   is returned on any failure.
 *
 ****************************************************************************/

int file_dup(FAR struct file *filep, int minfd, int flags)
{
  return fdlist_dupfile(nxsched_get_fdlist_from_tcb(this_task()),
                        flags, minfd, filep);
}

/****************************************************************************
 * Name: dup
 *
 * Description:
 *   Clone a file or socket descriptor to an arbitrary descriptor number
 *
 ****************************************************************************/

int dup(int fd)
{
  FAR struct file *filep;
  int ret;

  /* Get the file structure corresponding to the file descriptor. */

  ret = file_get(fd, &filep);
  if (ret < 0)
    {
      goto err;
    }

  /* Let file_dup() do the real work */

  ret = file_dup(filep, 0, 0);
  file_put(filep);
  if (ret < 0)
    {
      goto err;
    }

  return ret;

err:
  set_errno(-ret);
  return ERROR;
}
