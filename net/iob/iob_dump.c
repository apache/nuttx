/****************************************************************************
 * net/iob/iob_dump.c
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

#include <nuttx/net/iob.h>

#ifdef CONFIG_DEBUG

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/* Select the lowest level debug interface available */

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_ARCH_LOWPUTC
#    define message(format, ...) lowsyslog(format, ##__VA_ARGS__)
#  else
#    define message(format, ...) syslog(format, ##__VA_ARGS__)
#  endif
#else
#  ifdef CONFIG_ARCH_LOWPUTC
#    define message lowsyslog
#  else
#    define message syslog
#  endif
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: iob_dump
 *
 * Description:
 *   Dump the contents of a I/O buffer chain
 *
 ****************************************************************************/

void iob_dump(FAR const char *msg, FAR struct iob_s *iob)
{
  FAR struct iob_s *head = iob;
  FAR const uint8_t *buffer;
  uint8_t data[32];
  unsigned int nbytes;
  unsigned int i;
  unsigned int j;
  int len;

  message("%s: IOB=%p pktlen=%d\n", msg, head, head->io_pktlen);

  buffer = &iob->io_data[iob->io_offset];
  len    = iob->io_len;

  for (i = 0; i < head->io_pktlen && iob; i += 32)
    {
      /* Copy 32-bytes into our local buffer */

      for (nbytes = 0; nbytes < 32; nbytes++)
        {
          data[nbytes] = *buffer++;

          /* If we have exhausted the data in this I/O buffer,
           * then skip to the next I/O buffer in the chain.
           */

          if (--len <= 0)
           {
             iob = iob->io_flink;
             if (!iob)
               {
                 /* Ooops... we are at the end of the chain.
                  * break out with iob = NULL, len == 0, and
                  * nbytes <= 32.
                  */

                 len = 0;
                 break;
               }

             /* Get the data from the next I/O buffer in the chain */

             buffer = &iob->io_data[iob->io_offset];
             len = iob->io_len;
           }
        }

      /* Make sure that we have something to print */

      if (nbytes > 0)
        {
          message("%04x: ", i);
          for (j = 0; j < 32; j++)
            {
              if (j == 16)
                {
                  message(" ");
                }

              if (i + j < head->io_pktlen)
                {
                  message("%02x", buffer[j]);
                }
              else
                {
                  message("  ");
                }
            }

          message(" ");
          for (j = 0; j < 32; j++)
            {
              if (j == 16)
                {
                  message(" ");
                }

              if (i + j < head->io_pktlen)
                {
                  if (buffer[j] >= 0x20 && buffer[j] < 0x7f)
                    {
                      message("%c", buffer[j]);
                    }
                  else
                    {
                      message(".");
                    }
                }
            }

          message("\n");
        }
    }
}

#endif /* CONFIG_DEBUG */
