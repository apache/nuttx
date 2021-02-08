/****************************************************************************
 * mm/iob/iob_dump.c
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
      /* Copy IOB_NITEMS-bytes into our local buffer from the current
       * offset
       */

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
