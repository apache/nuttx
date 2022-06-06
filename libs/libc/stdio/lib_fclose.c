/****************************************************************************
 * libs/libc/stdio/lib_fclose.c
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
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>

#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fclose
 *
 * Description:
 *   The fclose() function will flush the stream pointed to by stream
 *   (writing any buffered output data using lib_fflush()) and close the
 *   underlying file descriptor.
 *
 * Returned Value:
 *   Upon successful completion 0 is returned. Otherwise, EOF is returned
 *   and the global variable errno is set to indicate the error. In either
 *   case any further access (including another call to fclose()) to the
 *   stream results in undefined behaviour.
 *
 ****************************************************************************/

int fclose(FAR FILE *stream)
{
  FAR struct streamlist *slist;
  FAR FILE *prev = NULL;
  FAR FILE *next;
  int errcode = EINVAL;
  int ret = ERROR;
  int status;

  /* Verify that a stream was provided. */

  if (stream)
    {
      ret = OK;

      /* If the stream was opened for writing, then flush the stream */

      if ((stream->fs_oflags & O_WROK) != 0)
        {
          ret = lib_fflush(stream, true);
          errcode = get_errno();
        }

      /* Skip close the builtin streams(stdin, stdout and stderr) */

      if (stream == stdin || stream == stdout || stream == stderr)
        {
          goto done;
        }

      /* Remove FILE structure from the stream list */

      slist = nxsched_get_streams();
      lib_stream_semtake(slist);

      for (next = slist->sl_head; next; prev = next, next = next->fs_next)
        {
          if (next == stream)
            {
              if (next == slist->sl_head)
                {
                  slist->sl_head = next->fs_next;
                }
              else
                {
                  prev->fs_next = next->fs_next;
                }

              if (next == slist->sl_tail)
                {
                  slist->sl_tail = prev;
                }

              break;
            }
        }

      lib_stream_semgive(slist);

      /* Check that the underlying file descriptor corresponds to an an open
       * file.
       */

      if (stream->fs_fd >= 0)
        {
          /* Close the file descriptor and save the return status */

          status = close(stream->fs_fd);

          /* If close() returns an error but flush() did not then make sure
           * that we return the close() error condition.
           */

          if (ret == OK)
            {
              ret = status;
              errcode = get_errno();
            }
        }

#ifndef CONFIG_STDIO_DISABLE_BUFFERING
      /* Destroy the semaphore */

      _RMUTEX_DESTROY(&stream->fs_lock);

      /* Release the buffer */

      if (stream->fs_bufstart != NULL &&
          (stream->fs_flags & __FS_FLAG_UBF) == 0)
        {
          lib_free(stream->fs_bufstart);
        }
#endif

      lib_free(stream);
    }

done:
  /* On an error, reset the errno to the first error encountered and return
   * EOF.
   */

  if (ret != OK)
    {
      set_errno(errcode);
      return EOF;
    }

  /* Return success */

  return OK;
}
