/****************************************************************************
 * fs/vfs/fs_dupfd.c
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
#include <errno.h>
#include <assert.h>

#include <nuttx/fs/fs.h>

#include "inode/inode.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DUP_ISOPEN(filep) (filep->f_inode != NULL)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: file_dup
 *
 * Description:
 *   Equivalent to the non-standard fs_dupfd() function except that it
 *   accepts a struct file instance instead of a file descriptor.
 *
 * Returned Value:
 *   The new file descriptor is returned on success; a negated errno value
 *   is returned on any failure.
 *
 ****************************************************************************/

int file_dup(FAR struct file *filep, int minfd)
{
  FAR struct file *filep2;
  int fd2;
  int ret;

  /* Verify that fd is a valid, open file descriptor */

  if (!DUP_ISOPEN(filep))
    {
      return -EBADF;
    }

  /* Then allocate a new file descriptor for the inode */

  fd2 = files_allocate(NULL, 0, 0, minfd);
  if (fd2 < 0)
    {
      return -EMFILE;
    }

  ret = fs_getfilep(fd2, &filep2);
  if (ret < 0)
    {
      files_release(fd2);
      return ret;
    }

  ret = file_dup2(filep, filep2);
  if (ret < 0)
    {
      files_release(fd2);
      return ret;
    }

  return fd2;
}

/****************************************************************************
 * Name: fs_dupfd
 *
 * Description:
 *   Clone a file descriptor 'fd' to an arbitrary descriptor number (any
 *   value greater than or equal to 'minfd').
 *
 * Returned Value:
 *   The new file descriptor is returned on success; a negated errno value
 *   is returned on any failure.
 *
 ****************************************************************************/

int fs_dupfd(int fd, int minfd)
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

  return file_dup(filep, minfd);
}
