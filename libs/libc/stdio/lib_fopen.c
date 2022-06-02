/****************************************************************************
 * libs/libc/stdio/lib_fopen.c
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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

#include "libc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Open mode flags */

#define MODE_R    (1 << 0) /* Bit 0: "r{b|x|+}" open for reading */
#define MODE_W    (1 << 1) /* Bit 1: "w{b|x|+}" open for writing, truncating,
                            * or creating file */
#define MODE_A    (1 << 2) /* Bit 2: "a{b|x|+}" open for writing, appending
                            * the to file */

#define MODE_NONE 0        /* No access mode determined */
#define MODE_MASK (MODE_R | MODE_W | MODE_A)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fdopen
 ****************************************************************************/

FAR FILE *fdopen(int fd, FAR const char *mode)
{
  FAR FILE *filep = NULL;
  int oflags;
  int ret;

  /* Map the open mode string to open flags */

  oflags = lib_mode2oflags(mode);
  if (oflags >= 0)
    {
      ret = fs_fdopen(fd, oflags, NULL, &filep);
      if (ret < 0)
        {
          set_errno(-ret);
        }
    }

  return filep;
}

/****************************************************************************
 * Name: fopen
 ****************************************************************************/

FAR FILE *fopen(FAR const char *path, FAR const char *mode)
{
  FAR FILE *filep = NULL;
  int oflags;
  int fd;
  int ret;

  /* Map the open mode string to open flags */

  oflags = lib_mode2oflags(mode);
  if (oflags < 0)
    {
      return NULL;
    }

  /* Open the file */

  fd = open(path, oflags, 0666);

  /* If the open was successful, then call fdopen() using the file
   * descriptor returned by open.  If open failed, then just return the
   * NULL stream -- open() has already set the errno.
   */

  if (fd >= 0)
    {
      ret = fs_fdopen(fd, oflags, NULL, &filep);
      if (ret < 0)
        {
          /* Don't forget to close the file descriptor if any other
           * failures are reported by fdopen().
           */

          close(fd);

          set_errno(-ret);
          filep = NULL;
        }
    }

  return filep;
}

/****************************************************************************
 * Name: lib_mode2oflags
 ****************************************************************************/

int lib_mode2oflags(FAR const char *mode)
{
  unsigned int state;
  int oflags;

  /* Verify that a mode string was provided.  */

  DEBUGASSERT(mode);

  /* Parse the mode string to determine the corresponding open flags */

  state  = MODE_NONE;
  oflags = 0;

  for (; *mode; mode++)
    {
      switch (*mode)
        {
          /* Open for read access ("r{b|x|+}") */

          case 'r' :
            if (state == MODE_NONE)
              {
                /* Open for read access */

                oflags = O_RDOK;
                state  = MODE_R;
              }
            else
              {
                goto errout;
              }
            break;

          /* Open for write access ("w{b|x|+}") */

          case 'w' :
            if (state == MODE_NONE)
              {
                /* Open for write access, truncating any existing file */

                oflags = (O_WROK | O_CREAT | O_TRUNC);
                state  = MODE_W;
              }
            else
              {
                goto errout;
              }
            break;

          /* Open for write/append access ("a{b|x|+}") */

          case 'a' :
            if (state == MODE_NONE)
              {
                /* Write to the end of the file */

                oflags = O_WROK | O_CREAT | O_APPEND;
                state  = MODE_A;
              }
            else
              {
                goto errout;
              }
            break;

          /* Open for update access ("{r|w|a|b|x}+") */

          case '+' :
            switch (state & MODE_MASK)
              {
                case MODE_R:
                  {
                    /* Retain any binary and exclusive mode selections */

                    oflags &= (O_BINARY | O_EXCL);

                    /* Open for read/write access */

                    oflags |= O_RDWR;
                 }
                 break;

                case MODE_W:
                  {
                    /* Retain any binary and exclusive mode selections */

                    oflags &= (O_BINARY | O_EXCL);

                    /* Open for write read/access, truncating any existing
                     * file.
                     */

                    oflags |= (O_RDWR | O_CREAT | O_TRUNC);
                  }
                  break;

                case MODE_A:
                  {
                    /* Retain any binary and exclusive mode selections */

                    oflags &= (O_BINARY | O_EXCL);

                    /* Read from the beginning of the file; write to the
                     * end,
                     */

                    oflags |= (O_RDWR | O_CREAT | O_APPEND);
                  }
                  break;

                default:
                  goto errout;
                  break;
              }
            break;

          /* Open for binary access ("{r|w|a|x|+}b") */

          case 'b' :
            if ((state & MODE_MASK) != MODE_NONE)
              {
                /* The file is opened in binary mode */

                oflags |= O_BINARY;
              }
            else
              {
                goto errout;
              }
            break;

          /* Open for exclusive access ("{r|w|a|b|+}x") */

          case 'x' :
            if ((state & MODE_MASK) != MODE_NONE)
              {
                /* The file is opened in exclusive mode */

                oflags |= O_EXCL;
              }
            else
              {
                goto errout;
              }
            break;

          /* Open for text (translated) access ("{r|w|a|x|+}t") */

          case 't' :
            if ((state & MODE_MASK) != MODE_NONE)
              {
                /* The file is opened in text mode */

                oflags |= O_TEXT;
              }
            else
              {
                goto errout;
              }
            break;

          /* Unrecognized or unsupported mode */

          default:
            goto errout;
            break;
        }
    }

  return oflags;

/* Both fopen and fdopen should fail with errno == EINVAL if the mode
 * string is invalid.
 */

errout:
  set_errno(EINVAL);
  return ERROR;
}
