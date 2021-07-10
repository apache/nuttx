/****************************************************************************
 * fs/vfs/fs_sendfile.c
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

#include <sys/sendfile.h>
#include <stdbool.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/net/net.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static ssize_t copyfile(FAR struct file *outfile, FAR struct file *infile,
                        off_t *offset, size_t count)
{
  FAR uint8_t *iobuffer;
  FAR uint8_t *wrbuffer;
  off_t startpos = 0;
  ssize_t nbytesread;
  ssize_t nbyteswritten;
  size_t  ntransferred;
  bool endxfr;

  /* Get the current file position. */

  if (offset)
    {
      off_t newpos;

      /* Use file_seek to get the current file position */

      startpos = file_seek(infile, 0, SEEK_CUR);
      if (startpos < 0)
        {
          return startpos;
        }

      /* Use file_seek again to set the new file position */

      newpos = file_seek(infile, *offset, SEEK_SET);
      if (newpos < 0)
        {
          return newpos;
        }
    }

  /* Allocate an I/O buffer */

  iobuffer = kmm_malloc(CONFIG_SENDFILE_BUFSIZE);
  if (!iobuffer)
    {
      return -ENOMEM;
    }

  /* Now transfer 'count' bytes from the infile to the outfile */

  for (ntransferred = 0, endxfr = false; ntransferred < count && !endxfr; )
    {
      /* Loop until the read side of the transfer comes to some conclusion */

      do
        {
          /* Read a buffer of data from the infile */

          nbytesread = count - ntransferred;
          if (nbytesread > CONFIG_SENDFILE_BUFSIZE)
            {
              nbytesread = CONFIG_SENDFILE_BUFSIZE;
            }

          nbytesread = file_read(infile, iobuffer, nbytesread);

          /* Check for end of file */

          if (nbytesread == 0)
            {
              /* End of file.  Break out and return current number of bytes
               * transferred.
               */

              endxfr = true;
              break;
            }

          /* Check for a read ERROR.  EINTR is a special case.  This function
           * should break out and return an error if EINTR is returned and
           * no data has been transferred.  But what should it do if some
           * data has been transferred?  I suppose just continue?
           */

          else if (nbytesread < 0)
            {
              /* EINTR is not an error (but will still stop the copy) */

              if (nbytesread != -EINTR || ntransferred == 0)
                {
                  /* Read error.  Break out and return the error condition. */

                  ntransferred = nbytesread;
                  endxfr       = true;
                  break;
                }
            }
        }
      while (nbytesread < 0);

      /* Was anything read? */

      if (!endxfr)
        {
          /* Yes.. Loop until the read side of the transfer comes to some
           * conclusion.
           */

          wrbuffer = iobuffer;
          do
            {
              /* Write the buffer of data to the outfile */

              nbyteswritten = file_write(outfile, wrbuffer, nbytesread);

              /* Check for a complete (or partial) write.  write() should not
               * return zero.
               */

              if (nbyteswritten >= 0)
                {
                  /* Advance the buffer pointer and decrement the number of
                   * bytes remaining in the iobuffer.  Typically, nbytesread
                   * will now be zero.
                   */

                  wrbuffer     += nbyteswritten;
                  nbytesread   -= nbyteswritten;

                  /* Increment the total number of bytes successfully
                   * transferred.
                   */

                  ntransferred += nbyteswritten;
                }

              /* Otherwise an error occurred */

              else
                {
                  /* Check for a write ERROR.  EINTR is a special case.  This
                   * function should break out and return an error if EINTR
                   * is returned and no data has been transferred.  But what
                   * should it do if some data has been transferred?  I
                   * suppose just continue?
                   */

                  if (nbyteswritten != -EINTR || ntransferred == 0)
                    {
                      /* Write error.  Break out and return the error
                       * condition.
                       */

                      ntransferred = nbyteswritten;
                      endxfr       = true;
                      break;
                    }
                }
            }
          while (nbytesread > 0);
        }
    }

  /* Release the I/O buffer */

  kmm_free(iobuffer);

  /* Return the current file position */

  if (offset)
    {
      /* Use file_seek to get the current file position */

      off_t curpos = file_seek(infile, 0, SEEK_CUR);
      if (curpos < 0)
        {
          return curpos;
        }

      /* Return the current file position */

      *offset = curpos;

      /* Use file_seek again to restore the original file position */

      startpos = file_seek(infile, startpos, SEEK_SET);
      if (startpos < 0)
        {
          return startpos;
        }
    }

  /* Finally return the number of bytes actually transferred (or ERROR
   * if any failure occurred).
   */

  return ntransferred;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: file_sendfile
 *
 * Description:
 *   Equivalent to the standard sendfile function except that is accepts a
 *   struct file instance instead of a file descriptor.
 *
 ****************************************************************************/

ssize_t file_sendfile(FAR struct file *outfile, FAR struct file *infile,
                      off_t *offset, size_t count)
{
#ifdef CONFIG_NET_SENDFILE
  /* Check the destination file descriptor:  Is it a (probable) file
   * descriptor?  Check the source file:  Is it a normal file?
   */

  FAR struct socket *psock;

  psock = file_socket(outfile);
  if (psock != NULL)
    {
      /* Then let psock_sendfile do the work. */

      int ret = psock_sendfile(psock, infile, offset, count);
      if (ret >= 0 || ret != -ENOSYS)
        {
          return ret;
        }

      /* Fall back to the slow path if errno equals ENOSYS,
       * because psock_sendfile fail to optimize this transfer.
       */
    }
#endif

  /* No... then this is probably a file-to-file transfer.  The generic
   * copyfile() can handle that case.
   */

  return copyfile(outfile, infile, offset, count);
}

/****************************************************************************
 * Name: sendfile
 *
 * Description:
 *   sendfile() copies data between one file descriptor and another.
 *   Used with file descriptors it basically just wraps a sequence of
 *   reads() and writes() to perform a copy.
 *
 *   If the destination descriptor is a socket, it gives a better
 *   performance than simple reds() and writes(). The data is read directly
 *   into the net buffer and the whole tcp window is filled if possible.
 *
 *   NOTE: This interface is *not* specified in POSIX.1-2001, or other
 *   standards.  The implementation here is very similar to the Linux
 *   sendfile interface.  Other UNIX systems implement sendfile() with
 *   different semantics and prototypes.  sendfile() should not be used
 *   in portable programs.
 *
 * Input Parameters:
 *   infd   - A file (or socket) descriptor opened for reading
 *   outfd  - A descriptor opened for writing.
 *   offset - If 'offset' is not NULL, then it points to a variable
 *            holding the file offset from which sendfile() will start
 *            reading data from 'infd'.  When sendfile() returns, this
 *            variable will be set to the offset of the byte following
 *            the last byte that was read.  If 'offset' is not NULL,
 *            then sendfile() does not modify the current file offset of
 *            'infd'; otherwise the current file offset is adjusted to
 *            reflect the number of bytes read from 'infd.'
 *
 *            If 'offset' is NULL, then data will be read from 'infd'
 *            starting at the current file offset, and the file offset
 *            will be updated by the call.
 *   count -  The number of bytes to copy between the file descriptors.
 *
 * Returned Value:
 *   If the transfer was successful, the number of bytes written to outfd is
 *   returned.  On error, -1 is returned, and errno is set appropriately.
 *   There error values are those returned by read() or write() plus:
 *
 *   EINVAL - Bad input parameters.
 *   ENOMEM - Could not allocated an I/O buffer
 *
 ****************************************************************************/

ssize_t sendfile(int outfd, int infd, off_t *offset, size_t count)
{
  FAR struct file *outfile;
  FAR struct file *infile;
  int ret;

  ret = fs_getfilep(outfd, &outfile);
  if (ret < 0)
    {
      goto errout;
    }

  ret = fs_getfilep(infd, &infile);
  if (ret < 0)
    {
      goto errout;
    }

  ret = file_sendfile(outfile, infile, offset, count);
  if (ret < 0)
    {
      goto errout;
    }

  return ret;

errout:
  set_errno(-ret);
  return ERROR;
}
