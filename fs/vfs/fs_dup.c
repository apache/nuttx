/****************************************************************************
 * fs/vfs/fs_dup.c
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

int file_dup(FAR struct file *filep, int minfd)
{
  struct file filep2;
  int fd2;
  int ret;

  /* Let file_dup2() do the real work */

  memset(&filep2, 0, sizeof(filep2));
  ret = file_dup2(filep, &filep2);
  if (ret < 0)
    {
      return ret;
    }

  /* Then allocate a new file descriptor for the inode */

  fd2 = files_allocate(filep2.f_inode, filep2.f_oflags,
                       filep2.f_pos, filep2.f_priv, minfd);
  if (fd2 < 0)
    {
      file_close(&filep2);
      return fd2;
    }

  return fd2;
}

/****************************************************************************
 * Name: nx_dup
 *
 * Description:
 *   nx_dup() is similar to the standard 'dup' interface except that is
 *   not a cancellation point and it does not modify the errno variable.
 *
 *   nx_dup() is an internal NuttX interface and should not be called from
 *   applications.
 *
 * Returned Value:
 *   The new file descriptor is returned on success; a negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int nx_dup(int fd)
{
  FAR struct file *filep;
  int ret;

  /* Get the file structure corresponding to the file descriptor. */

  ret = fs_getfilep(fd, &filep);
  if (ret < 0)
    {
      return ret;
    }

  DEBUGASSERT(filep != NULL);

  /* Let file_dup() do the real work */

  return file_dup(filep, 0);
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

  ret = nx_dup(fd);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}
