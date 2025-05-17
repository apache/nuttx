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
#ifdef CONFIG_FDSAN
  uint64_t f_tag_fdsan; /* File owner fdsan tag, init to 0 */
#endif

#ifdef CONFIG_FDCHECK
  uint8_t f_tag_fdcheck; /* File owner fdcheck tag, init to 0 */

  minfd = fdcheck_restore(minfd);
#endif

  fd2 = file_allocate(g_root_inode, flags, 0, NULL, minfd, true);
  if (fd2 < 0)
    {
      return fd2;
    }

  ret = fs_getfilep(fd2, &filep2);
#ifdef CONFIG_FDSAN
  f_tag_fdsan = nx_fcntl(fd2, FIOC_GETTAG_FDSAN, 0);
#endif

#ifdef CONFIG_FDCHECK
  f_tag_fdcheck = nx_fcntl(fd2, FIOC_GETTAG_FDCHECK, 0);
#endif
  DEBUGASSERT(ret >= 0);

  ret = file_dup2(filep, filep2);
  if (ret >= 0 && flags & O_CLOEXEC)
    {
      /* Set close on exec flag */

      ret = nx_fcntl(fd2, F_SETFD, flags & O_CLOEXEC ? FD_CLOEXEC : 0);
    }

#ifdef CONFIG_FDSAN
  nx_fcntl(fd2, FIOC_SETTAG_FDSAN, f_tag_fdsan);
#endif

#ifdef CONFIG_FDCHECK
  nx_fcntl(fd2, FIOC_SETTAG_FDCHECK, f_tag_fdcheck);
#endif

  fs_putfilep(filep2);
  if (ret >= 0)
    {
      return fd2;
    }

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
