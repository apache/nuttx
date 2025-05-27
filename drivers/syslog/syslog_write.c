/****************************************************************************
 * drivers/syslog/syslog_write.c
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

#include <sys/types.h>
#include <assert.h>

#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <nuttx/syslog/syslog.h>

#include "syslog.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: syslog_safe_to_block
 *
 * Description:
 *   Check if it is safe to block for write. If not, the write  defaults to a
 *   non-blocking method.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   true if it is safe to block; false otherwise.
 *
 ****************************************************************************/

static bool syslog_safe_to_block(void)
{
  FAR const struct tcb_s *rtcb;

  /* It's not safe to block in interrupts or when executing the idle loop */

  if (up_interrupt_context() || sched_idletask())
    {
      return false;
    }

  /* It's not safe to block if a signal is being delivered */

  rtcb = nxsched_self();
  if (rtcb->sigdeliver != NULL)
    {
      return false;
    }

  return true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: syslog_write_foreach
 *
 * Description:
 *   This provides a default write method for syslog devices that do not
 *   support multiple byte writes  This functions simply loops, outputting
 *   one character at a time.
 *
 * Input Parameters:
 *   buffer - The buffer containing the data to be output
 *   buflen - The number of bytes in the buffer
 *
 * Returned Value:
 *   On success, the number of characters written is returned.  A negated
 *   errno value is returned on any failure.
 *
 ****************************************************************************/

ssize_t syslog_write_foreach(FAR const char *buffer,
                             size_t buflen, bool force)
{
  syslog_write_t write;
  syslog_putc_t  putc;
  size_t nwritten = 0;
  ssize_t ret;
  int i;

  for (i = 0; i < CONFIG_SYSLOG_MAX_CHANNELS; i++)
    {
      FAR syslog_channel_t *channel = g_syslog_channel[i];

      if (channel == NULL)
        {
          break;
        }

#ifdef CONFIG_SYSLOG_IOCTL
      if (channel->sc_state & SYSLOG_CHANNEL_DISABLE)
        {
          continue;
        }
#endif

      write = !force ? channel->sc_ops->sc_write :
                       channel->sc_ops->sc_write_force;
      if (write != NULL)
        {
          nwritten = 0;

#ifdef CONFIG_SYSLOG_CRLF
          if (!(channel->sc_state & SYSLOG_CHANNEL_DISABLE_CRLF))
            {
              size_t head;

              for (head = 0; head < buflen; head++)
                {
                  if (buffer[head] != '\n')
                    {
                      continue;
                    }

                  ret = write(channel, buffer + nwritten, head - nwritten);
                  if (ret >= 0)
                    {
                      ret = write(channel, "\r\n", 2);
                    }

                  if (ret < 0)
                    {
                      return ret;
                    }

                  nwritten = head + 1;
                }
            }
#endif

          if (nwritten < buflen)
            {
              ret = write(channel, buffer + nwritten, buflen - nwritten);
              if (ret < 0)
                {
                  return ret;
                }
              else
                {
                  nwritten += ret;
                }
            }
        }
      else
        {
          putc = !force ? channel->sc_ops->sc_putc :
                          channel->sc_ops->sc_force;
          if (putc == NULL)
            {
              continue;
            }

#ifdef CONFIG_SYSLOG_CRLF
          if (channel->sc_state & SYSLOG_CHANNEL_DISABLE_CRLF)
#endif
            {
              for (nwritten = 0; nwritten < buflen; nwritten++)
                {
                  putc(channel, buffer[nwritten]);
                }
            }
#ifdef CONFIG_SYSLOG_CRLF
          else
            {
              for (nwritten = 0; nwritten < buflen; nwritten++)
                {
                  if (buffer[nwritten] == '\n')
                    {
                      /* Add CR */

                      putc(channel, '\r');
                    }

                  putc(channel, buffer[nwritten]);
                }
            }
#endif
        }
    }

  return nwritten;
}

/****************************************************************************
 * Name: syslog_write
 *
 * Description:
 *   This is the low-level, multiple character, system logging interface.
 *
 * Input Parameters:
 *   buffer - The buffer containing the data to be output
 *   buflen - The number of bytes in the buffer
 *
 * Returned Value:
 *   On success, the number of characters written is returned.  A negated
 *   errno value is returned on any failure.
 *
 ****************************************************************************/

ssize_t syslog_write(FAR const char *buffer, size_t buflen)
{
  bool force = !syslog_safe_to_block();

#ifdef CONFIG_SYSLOG_INTBUFFER
  if (force)
    {
      syslog_add_intbuffer(buffer, buflen);
      return buflen;
    }
  else
    {
      /* Flush any characters that may have been added to the interrupt
       * buffer.
       */

      syslog_flush_intbuffer(false);
    }
#endif

  return syslog_write_foreach(buffer, buflen, force);
}
