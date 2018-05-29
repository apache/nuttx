/****************************************************************************
 * libs/libc/misc/lib_dumpbuffer.c
 *
 *   Copyright (C) 2009, 2011, 2014, 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>
#include <debug.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#define _NITEMS   32                 /* 32 bytes displayed per line */
#define _LINESIZE (3 * _NITEMS + 4)  /* 2 hex chars, ASCII char, 3 spaces, NUL */

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
 * Name: lib_dumpbuffer
 *
 * Description:
 *  Do a pretty buffer dump.
 *
 *  A fairly large on-stack buffer is used for the case where timestamps are
 *  applied to each line.
 *
 ****************************************************************************/

void lib_dumpbuffer(FAR const char *msg, FAR const uint8_t *buffer,
                    unsigned int buflen)
{
  char buf[_LINESIZE];
  unsigned int i;
  unsigned int j;
  unsigned int k;

  syslog(LOG_INFO, "%s (%p):\n", msg, buffer);
  for (i = 0; i < buflen; i += _NITEMS)
    {
      FAR char *ptr = buf;

      /* Generate hex values:  2 * _NITEMS + 1 bytes */

      for (j = 0; j < _NITEMS; j++)
        {
          k = i + j;

          if (j == (_NITEMS / 2))
            {
              *ptr++ = ' ';
            }

          if (k < buflen)
            {
              *ptr++ = lib_nibble((buffer[k] >> 4) & 0xf);
              *ptr++ = lib_nibble(buffer[k] & 0xf);
            }
          else
            {
              *ptr++ = ' ';
              *ptr++ = ' ';
            }
        }

      *ptr++ = ' ';  /* Plus 1 byte */

      /* Generate printable characters:  Plus 1 * _NITEMS + 1 bytes */

      for (j = 0; j < _NITEMS; j++)
        {
         k = i + j;

          if (j == (_NITEMS / 2))
            {
              *ptr++ = ' ';
            }

          if (k < buflen)
            {
              if (buffer[k] >= 0x20 && buffer[k] < 0x7f)
                {
                  *ptr++ = buffer[k];
                }
              else
                {
                  *ptr++ = '.';
                }
            }
        }

      *ptr = '\0';  /* Plus 1 byte */
      syslog(LOG_INFO, "%04x: %s\n", i, buf);
   }
}
