/****************************************************************************
 * libs/libc/stdio/lib_fopen.c
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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

#ifdef CONFIG_FDSAN
#  include <android/fdsan.h>
#endif

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

#define FLAG_KEEP (O_TEXT | O_CLOEXEC | O_EXCL)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fdopen
 ****************************************************************************/

FAR FILE *fdopen(int fd, FAR const char *mode)
{
  FAR struct streamlist *list = lib_get_streams();
  FAR FILE *filep = NULL;
  int oflags;
  int ret;

  /* Map the open mode string to open flags */

  oflags = lib_mode2oflags(mode);
  if (oflags < 0)
    {
      return NULL;
    }

  /* Allocate FILE structure */

  if (fd >= 3)
    {
      filep = lib_zalloc(sizeof(FILE));
      if (filep == NULL)
        {
          ret = -ENOMEM;
          goto errout;
        }

      /* Add FILE structure to the stream list */

      ret = nxmutex_lock(&list->sl_lock);
      if (ret < 0)
        {
          lib_free(filep);
          goto errout;
        }

      sq_addlast(&filep->fs_entry, &list->sl_queue);

      nxmutex_unlock(&list->sl_lock);

      /* Initialize the mutex the manages access to the buffer */

      nxrmutex_init(&filep->fs_lock);

#ifdef CONFIG_FDSAN
      android_fdsan_exchange_owner_tag(fd, 0,
          android_fdsan_create_owner_tag(ANDROID_FDSAN_OWNER_TYPE_FILE,
                                        (uintptr_t)filep));
#endif
    }
  else
    {
      filep = &list->sl_std[fd];
    }

#if !defined(CONFIG_STDIO_DISABLE_BUFFERING) && CONFIG_STDIO_BUFFER_SIZE > 0
  /* Set up pointers */

  filep->fs_bufstart = filep->fs_buffer;
  filep->fs_bufend   = filep->fs_bufstart + CONFIG_STDIO_BUFFER_SIZE;
  filep->fs_bufpos   = filep->fs_bufstart;
  filep->fs_bufread  = filep->fs_bufstart;
  filep->fs_flags    = __FS_FLAG_UBF; /* Fake setvbuf and fclose */

#  ifdef CONFIG_STDIO_LINEBUFFER
  /* Setup buffer flags */

  filep->fs_flags   |= __FS_FLAG_LBF; /* Line buffering */

#  endif /* CONFIG_STDIO_LINEBUFFER */
#endif /* !CONFIG_STDIO_DISABLE_BUFFERING && CONFIG_STDIO_BUFFER_SIZE > 0 */

  /* Save the file description and open flags.  Setting the
   * file descriptor locks this stream.
   */

  filep->fs_cookie   = (FAR void *)(intptr_t)fd;
  filep->fs_oflags   = oflags;

  /* Assign custom callbacks to NULL. */

  filep->fs_iofunc.read  = NULL;
  filep->fs_iofunc.write = NULL;
  filep->fs_iofunc.seek  = NULL;
  filep->fs_iofunc.close = NULL;

  return filep;

errout:
  set_errno(-ret);
  return NULL;
}

/****************************************************************************
 * Name: fopen
 ****************************************************************************/

FAR FILE *fopen(FAR const char *path, FAR const char *mode)
{
  FAR FILE *filep = NULL;
  int oflags;
  int fd;

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
      filep = fdopen(fd, mode);
      if (filep == NULL)
        {
          /* Don't forget to close the file descriptor if any other
           * failures are reported by fdopen().
           */

          close(fd);
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
          /* Open for read access ("r{m|b|x|+}") */

          case 'r' :
            if (state == MODE_NONE)
              {
                /* Open for read access */

                oflags = O_RDOK | O_TEXT;
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

                oflags = O_WROK | O_CREAT | O_TRUNC | O_TEXT;
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

                oflags = O_WROK | O_CREAT | O_APPEND | O_TEXT;
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

                    oflags &= FLAG_KEEP;

                    /* Open for read/write access */

                    oflags |= O_RDWR;
                 }
                 break;

                case MODE_W:
                  {
                    /* Retain any binary and exclusive mode selections */

                    oflags &= FLAG_KEEP;

                    /* Open for write read/access, truncating any existing
                     * file.
                     */

                    oflags |= O_RDWR | O_CREAT | O_TRUNC;
                  }
                  break;

                case MODE_A:
                  {
                    /* Retain any binary and exclusive mode selections */

                    oflags &= FLAG_KEEP;

                    /* Read from the beginning of the file; write to the
                     * end,
                     */

                    oflags |= O_RDWR | O_CREAT | O_APPEND;
                  }
                  break;

                default:
                  goto errout;
              }
            break;

          /* Attempt to access the file using mmap. */

          case 'm' :
            if (state != MODE_R)
              {
                goto errout;
              }
            break;

          /* Open for binary access ("{r|w|a|x|+}b") */

          case 'b' :
            if ((state & MODE_MASK) != MODE_NONE)
              {
                /* The file is opened in binary mode */

                oflags &= ~O_TEXT;
              }
            else
              {
                goto errout;
              }
            break;

          /* Open for close on execute */

          case 'e' :
            if ((state & MODE_MASK) != MODE_NONE)
              {
                /* The file will be closed on execute */

                oflags |= O_CLOEXEC;
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
