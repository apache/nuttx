/****************************************************************************
 * libs/libc/misc/lib_sendfile.c
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
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>

#include <nuttx/fs/fs.h>

#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sendfile / lib_sendfile
 *
 * Description:
 *   sendfile() copies data between one file descriptor and another.
 *   sendfile() basically just wraps a sequence of reads() and writes()
 *   to perform a copy.  It serves a purpose in systems where there is
 *   a penalty for copies to between user and kernal space, but really
 *   nothing in NuttX but provide some Linux compatible (and adding
 *   another 'almost standard' interface).
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

#ifdef CONFIG_NET_SENDFILE
ssize_t lib_sendfile(int outfd, int infd, off_t *offset, size_t count)
#else
ssize_t sendfile(int outfd, int infd, off_t *offset, size_t count)
#endif
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

      /* Use lseek to get the current file position */

      startpos = _NX_SEEK(infd, 0, SEEK_CUR);
      if (startpos < 0)
        {
          int errcode = _NX_GETERRNO(startpos);
          _NX_SETERRNO(errcode);
          return ERROR;
        }

      /* Use lseek again to set the new file position */

      newpos = _NX_SEEK(infd, *offset, SEEK_SET);
      if (newpos < 0)
        {
          int errcode = _NX_GETERRNO(newpos);
          _NX_SETERRNO(errcode);
          return ERROR;
        }
    }

  /* Allocate an I/O buffer */

  iobuffer = (FAR void *)lib_malloc(CONFIG_LIB_SENDFILE_BUFSIZE);
  if (!iobuffer)
    {
      set_errno(ENOMEM);
      return ERROR;
    }

  /* Now transfer 'count' bytes from the infd to the outfd */

  for (ntransferred = 0, endxfr = false; ntransferred < count && !endxfr; )
    {
      /* Loop until the read side of the transfer comes to some conclusion */

      do
        {
          /* Read a buffer of data from the infd */

          nbytesread = _NX_READ(infd, iobuffer, CONFIG_LIB_SENDFILE_BUFSIZE);

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
              int errcode = _NX_GETERRNO(nbytesread);

              /* EINTR is not an error (but will still stop the copy) */

              if (errcode != EINTR || ntransferred == 0)
                {
                  /* Read error.  Break out and return the error condition. */

                  _NX_SETERRNO(nbytesread);
                  ntransferred = ERROR;
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
              /* Write the buffer of data to the outfd */

              nbyteswritten = _NX_WRITE(outfd, wrbuffer, nbytesread);

              /* Check for a complete (or parial) write.  write() should not
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
                  int errcode = _NX_GETERRNO(nbyteswritten);

                  /* Check for a read ERROR.  EINTR is a special case.  This
                   * function should break out and return an error if EINTR
                   * is returned and no data has been transferred.  But what
                   * should it do if some data has been transferred?  I
                   * suppose just continue?
                   */

                  if (errcode != EINTR || ntransferred == 0)
                    {
                      /* Write error.  Break out and return the error
                       * condition.
                       */

                      _NX_SETERRNO(nbyteswritten);
                      ntransferred = ERROR;
                      endxfr       = true;
                      break;
                    }
                }
            }
          while (nbytesread > 0);
        }
    }

  /* Release the I/O buffer */

  lib_free(iobuffer);

  /* Return the current file position */

  if (offset)
    {
      /* Use lseek to get the current file position */

      off_t curpos = _NX_SEEK(infd, 0, SEEK_CUR);
      if (curpos < 0)
        {
          int errcode = _NX_GETERRNO(curpos);
          _NX_SETERRNO(errcode);
          return ERROR;
        }

      /* Return the current file position */

      *offset = curpos;

      /* Use lseek again to restore the original file position */

      startpos = _NX_SEEK(infd, startpos, SEEK_SET);
      if (startpos < 0)
        {
          int errcode = _NX_GETERRNO(startpos);
          _NX_SETERRNO(errcode);
          return ERROR;
        }
    }

  /* Finally return the number of bytes actually transferred (or ERROR
   * if any failure occurred).
   */

  return ntransferred;
}
