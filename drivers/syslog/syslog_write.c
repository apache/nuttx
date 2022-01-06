/****************************************************************************
 * drivers/syslog/syslog_write.c
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
 * Name: syslog_default_write
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

static ssize_t syslog_default_write(FAR const char *buffer, size_t buflen)
{
  int i;
  size_t nwritten = 0;

  if (up_interrupt_context() || sched_idletask())
    {
      for (nwritten = 0; nwritten < buflen; nwritten++)
        {
#ifdef CONFIG_SYSLOG_INTBUFFER
          if (up_interrupt_context())
            {
              syslog_add_intbuffer(buffer[nwritten]);
            }
          else
#endif
            {
              for (i = 0; i < CONFIG_SYSLOG_MAX_CHANNELS; i++)
                {
                  if (g_syslog_channel[i] == NULL)
                    {
                      break;
                    }

                  DEBUGASSERT(g_syslog_channel[i]->sc_ops->sc_force != NULL);
                  g_syslog_channel[i]->sc_ops->sc_force(g_syslog_channel[i],
                                                        buffer[nwritten]);
                }
            }
        }
    }
  else
    {
      for (i = 0; i < CONFIG_SYSLOG_MAX_CHANNELS; i++)
        {
          if (g_syslog_channel[i] == NULL)
            {
              break;
            }

          if (g_syslog_channel[i]->sc_ops->sc_write)
            {
              nwritten =
                g_syslog_channel[i]->sc_ops->sc_write(g_syslog_channel[i],
                                                      buffer, buflen);
            }
          else
            {
              DEBUGASSERT(g_syslog_channel[i]->sc_ops->sc_putc != NULL);

              for (nwritten = 0; nwritten < buflen; nwritten++)
                {
                  g_syslog_channel[i]->sc_ops->sc_putc(g_syslog_channel[i],
                                                       buffer[nwritten]);
                }
            }
        }
    }

  return nwritten;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
#ifdef CONFIG_SYSLOG_INTBUFFER
  if (!up_interrupt_context() && !sched_idletask())
    {
      /* Flush any characters that may have been added to the interrupt
       * buffer.
       */

      syslog_flush_intbuffer(false);
    }
#endif

  return syslog_default_write(buffer, buflen);
}
