/****************************************************************************
 * fs/vfs/fs_read.c
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

#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <sched.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/cancelpt.h>

#include "inode/inode.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: file_read
 *
 * Description:
 *   file_read() is an internal OS interface.  It is functionally similar to
 *   the standard read() interface except:
 *
 *    - It does not modify the errno variable,
 *    - It is not a cancellation point,
 *    - It accepts a file structure instance instead of file descriptor.
 *
 * Input Parameters:
 *   filep  - File structure instance
 *   buf    - User-provided to save the data
 *   nbytes - The maximum size of the user-provided buffer
 *
 * Returned Value:
 *   The positive non-zero number of bytes read on success, 0 on if an
 *   end-of-file condition, or a negated errno value on any failure.
 *
 ****************************************************************************/

ssize_t file_read(FAR struct file *filep, FAR void *buf, size_t nbytes)
{
  FAR struct inode *inode;
  int ret = -EBADF;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  /* Was this file opened for read access? */

  if ((filep->f_oflags & O_RDOK) == 0)
    {
      /* No.. File is not read-able */

      ret = -EACCES;
    }

  /* Is a driver or mountpoint registered? If so, does it support the read
   * method?
   */

  else if (inode != NULL && inode->u.i_ops && inode->u.i_ops->read)
    {
      /* Yes.. then let it perform the read.  NOTE that for the case of the
       * mountpoint, we depend on the read methods being identical in
       * signature and position in the operations vtable.
       */

      ret = (int)inode->u.i_ops->read(filep,
                                     (FAR char *)buf,
                                     (size_t)nbytes);
    }

  /* Return the number of bytes read (or possibly an error code) */

  return ret;
}

/****************************************************************************
 * Name: nx_read
 *
 * Description:
 *   nx_read() is an internal OS interface.  It is functionally similar to
 *   the standard read() interface except:
 *
 *    - It does not modify the errno variable, and
 *    - It is not a cancellation point.
 *
 * Input Parameters:
 *   fd     - File descriptor to read from
 *   buf    - User-provided to save the data
 *   nbytes - The maximum size of the user-provided buffer
 *
 * Returned Value:
 *   The positive non-zero number of bytes read on success, 0 on if an
 *   end-of-file condition, or a negated errno value on any failure.
 *
 ****************************************************************************/

ssize_t nx_read(int fd, FAR void *buf, size_t nbytes)
{
  FAR struct file *filep;
  ssize_t ret;

  /* First, get the file structure.  Note that on failure,
   * fs_getfilep() will return the errno.
   */

  ret = (ssize_t)fs_getfilep(fd, &filep);
  if (ret < 0)
    {
      return ret;
    }

  /* Then let file_read do all of the work. */

  return file_read(filep, buf, nbytes);
}

/****************************************************************************
 * Name: read
 *
 * Description:
 *   The standard, POSIX read interface.
 *
 * Input Parameters:
 *   fd     - File descriptor to read from
 *   buf    - User-provided to save the data
 *   nbytes - The maximum size of the user-provided buffer
 *
 * Returned Value:
 *   The positive non-zero number of bytes read on success, 0 on if an
 *   end-of-file condition, or -1 on failure with errno set appropriately.
 *
 ****************************************************************************/

ssize_t read(int fd, FAR void *buf, size_t nbytes)
{
  ssize_t ret;

  /* read() is a cancellation point */

  enter_cancellation_point();

  /* Let nx_read() do the real work */

  ret = nx_read(fd, buf, nbytes);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  leave_cancellation_point();
  return ret;
}
