/****************************************************************************
 * libs/libc/stdio/lib_libflushall.c
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

#include <stdbool.h>
#include <fcntl.h>
#include <errno.h>

#include <nuttx/fs/fs.h>

#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_flushall
 *
 * Description:
 *   Called either (1) by the OS when a task exits, or (2) from fflush()
 *   when a NULL stream argument is provided.
 *
 ****************************************************************************/

int lib_flushall(FAR struct streamlist *list)
{
  int lasterrno = OK;
  int ret;

  /* Make sure that there are streams associated with this thread */

  if (list)
    {
      FAR FILE *stream;
      int i;

      /* Process each stream in the thread's stream list */

      lib_stream_semtake(list);

      for (i = 0; i < 3; i++)
        {
          lib_fflush(&list->sl_std[i], true);
        }

      stream = list->sl_head;
      for (; stream != NULL; stream = stream->fs_next)
        {
          /* If the stream is opened for writing, then flush all of
           * the pending write data in the stream.
           */

          if ((stream->fs_oflags & O_WROK) != 0)
            {
              /* Flush the writable FILE */

              ret = lib_fflush(stream, true);
              if (ret < 0)
                {
                  /* An error occurred during the flush AND/OR we were unable
                   * to flush all of the buffered write data.  Remember the
                   * last errcode.
                   */

                  lasterrno = ret;
                }
            }
        }

      lib_stream_semgive(list);
    }

  /* If any flush failed, return the errorcode of the last failed flush */

  return lasterrno;
}
