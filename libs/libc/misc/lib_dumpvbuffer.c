/****************************************************************************
 * libs/libc/misc/lib_dumpvbuffer.c
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
#include <nuttx/compiler.h>

#include <stdint.h>
#include <stdio.h>
#include <debug.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/* 32 bytes displayed per line */
#define _NITEMS           (16)

/* 16 * (2 hex chars + 1 blank) + 1 space */
#define _ASCIICHAR_OFFSET (_NITEMS * 3 + 1)

/* _ASCIICHAR_OFFSET + 16 ASCII char, 1 NULL */
#define _LINESIZE         (_ASCIICHAR_OFFSET + _NITEMS + 1)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_nibble
 *
 * Description:
 *  Convert a binary nibble to a hexadecimal character.
 *
 ****************************************************************************/

static char lib_nibble(unsigned char nibble)
{
  if (nibble < 10)
    {
      return '0' + nibble;
    }
  else
    {
      return 'a' + nibble - 10;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_writevbuffer
 *
 * Description:
 *  Do a pretty buffer dump from multiple buffers to a specified file
 *  descriptor.
 *
 *  A fairly large on-stack buffer is used for the case where timestamps are
 *  applied to each line.
 *
 ****************************************************************************/

void lib_writevbuffer(int fd, FAR const char *msg,
                      FAR const struct iovec *iov, int iovcnt)
{
  FAR const struct iovec *piov = iov;
  FAR const uint8_t *iov_buf;
  char line[_LINESIZE];
  size_t niovpos = 0;
  size_t ntotal = 0;
  size_t nbufpos;
  unsigned int i;
  unsigned int j;
  unsigned int k;
  FAR char *ptr;

  if (msg)
    {
      if (fd > 0)
        {
          dprintf(fd, "%s (%p):\n", msg, iov->iov_base);
        }
      else
        {
          syslog(LOG_INFO, "%s (%p):\n", msg, iov->iov_base);
        }
    }

  for (i = 0; i < iovcnt; i++)
    {
      /* Ignore zero-length writes */

      if (iov[i].iov_len <= 0)
        {
          break;
        }

      ntotal += iov[i].iov_len;
    }

  if (ntotal == 0)
    {
      return;
    }

  /* Initialize the separator and terminator */

  line[_ASCIICHAR_OFFSET - 1] = ' ';
  line[_LINESIZE - 1] = '\0';

  iov_buf = piov->iov_base;

  for (i = 0; i < ntotal; i += _NITEMS)
    {
      ptr = line;

      for (j = 0; j < _NITEMS; j++)
        {
          k = i + j;

          if (k < ntotal)
            {
              /* Redirect the IO Vector */

              if (k == niovpos + piov->iov_len)
                {
                  niovpos += piov->iov_len;
                  piov++;
                  iov_buf = piov->iov_base;
                }

              /* Generate hex values */

              nbufpos = k - niovpos;

              *ptr++ = lib_nibble((iov_buf[nbufpos] >> 4) & 0xf);
              *ptr++ = lib_nibble(iov_buf[nbufpos] & 0xf);

              /* Generate printable characters */

              if (iov_buf[nbufpos] >= 0x20 && iov_buf[nbufpos] < 0x7f)
                {
                  line[_ASCIICHAR_OFFSET + j] = iov_buf[nbufpos];
                }
              else
                {
                  line[_ASCIICHAR_OFFSET + j] = '.';
                }
            }
          else
            {
              *ptr++ = ' ';
              *ptr++ = ' ';
              line[_ASCIICHAR_OFFSET + j] = '\0';
            }

          *ptr++ = ' ';  /* Plus 1 byte */
        }

      if (fd > 0)
        {
          dprintf(fd, "%04x  %s\n", i, line);
        }
      else
        {
          syslog(LOG_INFO, "%04x  %s\n", i, line);
        }
    }
}

/****************************************************************************
 * Name: lib_dumpvbuffer
 *
 * Description:
 *  Do a pretty buffer dump from multiple buffers.
 *
 *  A fairly large on-stack buffer is used for the case where timestamps are
 *  applied to each line.
 *
 ****************************************************************************/

void lib_dumpvbuffer(FAR const char *msg, FAR const struct iovec *iov,
                     int iovcnt)
{
  lib_writevbuffer(-1, msg, iov, iovcnt);
}
