/****************************************************************************
 * libc/misc/lib_dumpbuffer.c
 *
 *   Copyright (C) 2009, 2011, 2014 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_dumpbuffer
 *
 * Description:
 *  Do a pretty buffer dump
 *
 ****************************************************************************/

void lib_dumpbuffer(FAR const char *msg, FAR const uint8_t *buffer,
                    unsigned int buflen)
{
  unsigned int i;
  unsigned int j;
  unsigned int k;

  lowsyslog(LOG_INFO, "%s (%p):\n", msg, buffer);
  for (i = 0; i < buflen; i += 32)
    {
      lowsyslog(LOG_INFO, "%04x: ", i);
      for (j = 0; j < 32; j++)
        {
          k = i + j;

          if (j == 16)
            {
              lowsyslog(LOG_INFO, " ");
            }

          if (k < buflen)
            {
              lowsyslog(LOG_INFO, "%02x", buffer[k]);
            }
          else
            {
              lowsyslog(LOG_INFO, "  ");
            }
        }

      lowsyslog(LOG_INFO, " ");
      for (j = 0; j < 32; j++)
        {
         k = i + j;

          if (j == 16)
            {
              lowsyslog(LOG_INFO, " ");
            }

          if (k < buflen)
            {
              if (buffer[k] >= 0x20 && buffer[k] < 0x7f)
                {
                  lowsyslog(LOG_INFO, "%c", buffer[k]);
                }
              else
                {
                  lowsyslog(LOG_INFO, ".");
                }
            }
        }

      lowsyslog(LOG_INFO, "\n");
   }
}
