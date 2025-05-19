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

#ifdef CONFIG_FDCHECK
#  include <nuttx/fdcheck.h>
#endif

#include <nuttx/fs/fs.h>
#include "inode/inode.h"

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
  FAR struct file *filep2;
  int fd2;
  int ret;

#ifdef CONFIG_FDCHECK
  minfd = fdcheck_restore(minfd);
#endif

  /* Pass the close on exec flag to file_allocate */

  fd2 = file_allocate(g_root_inode, flags, 0, NULL, minfd, true);
  if (fd2 < 0)
    {
      return fd2;
    }

  /* Get the associated file pointer, taking a reference to it */

  ret = fs_getfilep(fd2, &filep2);
  if (ret >= 0)
    {
      ret = file_dup2(filep, filep2);
      fs_putfilep(filep2);
    }

  if (ret >= 0)
    {
      return fd2;
    }

  /* There was an error: release filep2 */

  fs_putfilep(filep2);
  return ret;
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
  int ret;

  /* Let nx_dup() do the real work */

  ret = nx_dup(fd, 0, 0);
  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  return ret;
}
