/****************************************************************************
 * drivers/syslog/syslog_force.c
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
#include <stdbool.h>
#include <assert.h>

#include <nuttx/syslog/syslog.h>

#include "syslog.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: syslog_force
 *
 * Description:
 *   This is the low-level system logging interface.  This version forces
 *   the output and is only used in emergency situations (e.g., in assertion
 *   handling).
 *
 * Input Parameters:
 *   ch - The character to add to the SYSLOG (must be positive).
 *
 * Returned Value:
 *   On success, the character is echoed back to the caller. A negated errno
 *   value is returned on any failure.
 *
 ****************************************************************************/

int syslog_force(int ch)
{
  int i;

#ifdef CONFIG_SYSLOG_INTBUFFER
  /* Flush any characters that may have been added to the interrupt
   * buffer through the emergency channel
   */

  syslog_flush_intbuffer(true);
#endif

  for (i = 0; i < CONFIG_SYSLOG_MAX_CHANNELS; i++)
    {
      if (g_syslog_channel[i] == NULL)
        {
          break;
        }

      DEBUGASSERT(g_syslog_channel[i]->sc_ops->sc_force != NULL);

      /* Then send the character to the emergency channel */

      g_syslog_channel[i]->sc_ops->sc_force(g_syslog_channel[i], ch);
    }

  return ch;
}
