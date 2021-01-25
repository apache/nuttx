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

#include <string.h>
#include <stdint.h>
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
  FAR const struct iovec *piov = iov;
  unsigned int len = 0;
  char line[_LINESIZE];
  unsigned int nitems;
  unsigned int i = 0;
  FAR char *ptr;

  if (msg)
    {
      syslog(LOG_INFO, "%s (%p):\n", msg, iov->iov_base);
    }

  /* Initialize the separator and terminator */

  line[_ASCIICHAR_OFFSET - 1] = ' ';
  line[_LINESIZE - 1] = '\0';

  while (piov != &iov[iovcnt] && piov->iov_len)
    {
      ptr = line;

      for (nitems = 0; nitems < _NITEMS; nitems++)
        {
          FAR const uint8_t *iov_buf = piov->iov_base;

          *ptr++ = lib_nibble((iov_buf[len] >> 4) & 0xf);
          *ptr++ = lib_nibble(iov_buf[len] & 0xf);
          *ptr++ = ' ';

          /* Generate printable characters */

          if (iov_buf[len] >= 0x20 && iov_buf[len] < 0x7f)
            {
              line[_ASCIICHAR_OFFSET + nitems] = iov_buf[len];
            }
          else
            {
              line[_ASCIICHAR_OFFSET + nitems] = '.';
            }

          if (++len == piov->iov_len)
            {
              len = 0;
              if (++piov == &iov[iovcnt])
                {
                  memset(ptr, ' ', (_NITEMS - nitems - 1) * 3);
                  memset(&line[_ASCIICHAR_OFFSET + nitems + 1], ' ',
                         _NITEMS - nitems - 1);
                  break;
                }
            }
        }

      syslog(LOG_INFO, "%04x  %s\n", i++ * _NITEMS, line);
    }
}
