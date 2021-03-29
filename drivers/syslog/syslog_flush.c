/****************************************************************************
 * drivers/syslog/syslog_flush.c
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

#include <stdbool.h>
#include <assert.h>

#include <nuttx/syslog/syslog.h>

#include "syslog.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: syslog_flush
 *
 * Description:
 *   This is called by system crash-handling logic.  It must flush any
 *   buffered data to the SYSLOG device.
 *
 *   Interrupts are disabled at the time of the crash and this logic must
 *   perform the flush using low-level, non-interrupt driven logic.
 *
 *   REVISIT:  There is an implementation problem in that if a character
 *   driver is the underlying device, then there is no mechanism to flush
 *   the data buffered in the driver with interrupts disabled.
 *
 *   Currently, this function on (a) dumps the interrupt buffer (if the
 *   SYSLOG interrupt buffer is enabled), and (b) only the SYSLOG interface
 *   supports supports the 'sc_force()' method.
 *
 * Input Parameters:
 *   ch - The character to add to the SYSLOG (must be positive).
 *
 * Returned Value:
 *   Zero (OK)is returned on  success.  A negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

int syslog_flush(void)
{
  int i;

#ifdef CONFIG_SYSLOG_INTBUFFER
  /* Flush any characters that may have been added to the interrupt
   * buffer.
   */

  syslog_flush_intbuffer(true);
#endif

  for (i = 0; i < CONFIG_SYSLOG_MAX_CHANNELS; i++)
    {
      if (g_syslog_channel[i] == NULL)
        {
          break;
        }

      /* Then flush all of the buffered output to the SYSLOG device */

      if (g_syslog_channel[i]->sc_ops->sc_flush != NULL)
        {
          g_syslog_channel[i]->sc_ops->sc_flush(g_syslog_channel[i]);
        }
    }

  return OK;
}
