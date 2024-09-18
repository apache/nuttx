/****************************************************************************
 * fs/vfs/fs_uio.c
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

#include <sys/uio.h>
#include <sys/types.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/uio.h>

#include <assert.h>
#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iovec_total_len
 *
 * Description:
 *   Return the total length of data in bytes.
 *   Or -EOVERFLOW.
 *
 ****************************************************************************/

ssize_t iovec_total_len(FAR const struct iovec *iov, int iovcnt)
{
  size_t len = 0;
  int i;

  for (i = 0; i < iovcnt; i++)
    {
      if (SSIZE_MAX - len < iov[i].iov_len)
        {
          return -EOVERFLOW;
        }

      len += iov[i].iov_len;
    }

  return len;
}

/****************************************************************************
 * Name: iovec_compat_readv
 *
 * Description:
 *   Emulate readv using file_operation::read.
 *
 ****************************************************************************/

ssize_t iovec_compat_readv(FAR struct file *filep,
                           FAR const struct iovec *iov,
                           int iovcnt)
{
  FAR struct inode *inode = filep->f_inode;
  ssize_t ntotal;
  ssize_t nread;
  size_t remaining;
  FAR uint8_t *buffer;
  int i;

  DEBUGASSERT(inode->u.i_ops->read != NULL);

  /* Process each entry in the struct iovec array */

  for (i = 0, ntotal = 0; i < iovcnt; i++)
    {
      /* Ignore zero-length reads */

      if (iov[i].iov_len > 0)
        {
          buffer    = iov[i].iov_base;
          remaining = iov[i].iov_len;

          /* Read repeatedly as necessary to fill buffer */

          do
            {
              nread = inode->u.i_ops->read(filep, (void *)buffer,
                                                  remaining);

              /* Check for a read error */

              if (nread < 0)
                {
                  return ntotal ? ntotal : nread;
                }

              /* Check for an end-of-file condition */

              else if (nread == 0)
                {
                  return ntotal;
                }

              /* Update pointers and counts in order to handle partial
               * buffer reads.
               */

              buffer    += nread;
              remaining -= nread;
              ntotal    += nread;
            }
          while (remaining > 0);
        }
    }

  return ntotal;
}

/****************************************************************************
 * Name: iovec_compat_writev
 *
 * Description:
 *   Emulate writev using file_operation::write.
 *
 ****************************************************************************/

ssize_t iovec_compat_writev(FAR struct file *filep,
                            FAR const struct iovec *iov,
                            int iovcnt)
{
  FAR struct inode *inode = filep->f_inode;
  ssize_t ntotal;
  ssize_t nwritten;
  size_t remaining;
  FAR uint8_t *buffer;
  int i;

  DEBUGASSERT(inode->u.i_ops->write != NULL);

  /* Process each entry in the struct iovec array */

  for (i = 0, ntotal = 0; i < iovcnt; i++)
    {
      /* Ignore zero-length writes */

      if (iov[i].iov_len > 0)
        {
          buffer    = iov[i].iov_base;
          remaining = iov[i].iov_len;

          /* Write repeatedly as necessary to write the entire buffer */

          do
            {
              nwritten = inode->u.i_ops->write(filep, (void *)buffer,
                                                      remaining);

              /* Check for a write error */

              if (nwritten < 0)
                {
                  return ntotal ? ntotal : nwritten;
                }

              /* Update pointers and counts in order to handle partial
               * buffer writes.
               */

              buffer    += nwritten;
              remaining -= nwritten;
              ntotal    += nwritten;
            }
          while (remaining > 0);
        }
    }

  return ntotal;
}
