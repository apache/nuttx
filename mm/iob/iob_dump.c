/****************************************************************************
 * mm/iob/iob_dump.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <debug.h>

#include <nuttx/mm/iob.h>

#include "iob.h"

#ifdef CONFIG_DEBUG_FEATURES

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#ifndef MIN
#  define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

#define IOB_NITEMS   32                    /* 32 bytes displayed per line */
#define IOB_LINESIZE (3 * IOB_NITEMS + 4)  /* 2 hex chars, ASCII char, 3 spaces, NUL */

/****************************************************************************
 * Name: iob_nibble
 *
 * Description:
 *  Convert a binary nibble to a hexadecimal character.
 *
 ****************************************************************************/

static char iob_nibble(unsigned char nibble)
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
 * Name: iob_dump
 *
 * Description:
 *   Dump the contents of a I/O buffer chain
 *
 ****************************************************************************/

void iob_dump(FAR const char *msg, FAR struct iob_s *iob, unsigned int len,
              unsigned int offset)
{
  FAR struct iob_s *head;
  uint8_t data[IOB_NITEMS];
  char buf[IOB_LINESIZE];
  unsigned int maxlen;
  unsigned int nbytes;
  unsigned int lndx;
  unsigned int cndx;

  head = iob;
  syslog(LOG_DEBUG, "%s: iob=%p pktlen=%d\n", msg, head, head->io_pktlen);

  /* Check if the offset is beyond the data in the I/O buffer chain */

  if (offset > head->io_pktlen)
    {
      ioberr("ERROR: offset is past the end of data: %u > %u\n",
             offset, head->io_pktlen);
      return;
    }

  /* Dump I/O buffer headers */

  for (; iob; iob = iob->io_flink)
    {
      syslog(LOG_DEBUG, "  iob=%p len=%d offset=%d\n",
             iob, iob->io_len, iob->io_offset);
    }

  /* Get the amount of data to be displayed, limited by the amount that we
   * have beyond the offset.
   */

  maxlen = head->io_pktlen - offset;
  len = MIN(len, maxlen);

  /* Then beginning printing with the buffer containing the offset in groups
   * of IOB_NITEMS bytes.
   */

  for (lndx = 0; lndx < len; lndx += IOB_NITEMS, offset += IOB_NITEMS)
    {
      /* Copy IOB_NITEMS-bytes into our local buffer from the current offset */

      nbytes = iob_copyout(data, head, IOB_NITEMS, offset);

      /* Make sure that we have something to print */

      if (nbytes > 0)
        {
          FAR char *ptr = buf;

          for (cndx = 0; cndx < IOB_NITEMS; cndx++)
            {
              if (cndx == IOB_NITEMS / 2)
                {
                  *ptr++ = ' ';
                }

              if ((lndx + cndx) < len)
                {
                  *ptr++ = iob_nibble((data[cndx] >> 4) & 0xf);
                  *ptr++ = iob_nibble(data[cndx] & 0xf);
                }
              else
                {
                  *ptr++ = ' ';
                  *ptr++ = ' ';
                }
            }

          *ptr++ = ' ';
          for (cndx = 0; cndx < IOB_NITEMS; cndx++)
            {
              if (cndx == IOB_NITEMS / 2)
                {
                  *ptr++ = ' ';
                }

              if ((lndx + cndx) < len)
                {
                  if (data[cndx] >= 0x20 && data[cndx] < 0x7f)
                    {
                      *ptr++ = data[cndx];
                    }
                  else
                    {
                      *ptr++ = '.';
                    }
                }
            }

          *ptr = '\0';
          syslog(LOG_DEBUG, "  %04x: %s\n", offset, buf);
        }
    }
}

#endif /* CONFIG_DEBUG_FEATURES */
