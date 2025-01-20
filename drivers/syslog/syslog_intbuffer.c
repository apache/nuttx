/****************************************************************************
 * drivers/syslog/syslog_intbuffer.c
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

#include <stdbool.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/syslog/syslog.h>
#include <nuttx/spinlock.h>
#include <nuttx/circbuf.h>

#include "syslog.h"

#ifdef CONFIG_SYSLOG_INTBUFFER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if CONFIG_SYSLOG_INTBUFSIZE > 65535
#  undef  CONFIG_SYSLOG_INTBUFSIZE
#  define CONFIG_SYSLOG_INTBUFSIZE 65535
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure encapsulates the interrupt buffer state */

struct syslog_intbuffer_s
{
  struct circbuf_s circ;
  spinlock_t       splock;
  uint8_t          buffer[CONFIG_SYSLOG_INTBUFSIZE];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct syslog_intbuffer_s g_syslog_intbuffer =
{
  CIRCBUF_INITIALIZER(g_syslog_intbuffer.buffer,
                      sizeof(g_syslog_intbuffer.buffer)),
  SP_UNLOCKED,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: syslog_flush_internal
 *
 * Description:
 *   Flush any characters that may have been added to the interrupt buffer
 *   to the SYSLOG device.
 *
 * Input Parameters:
 *   force   - Use the force() method of the channel vs. the putc() method.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Interrupts may or may not be disabled.
 *
 ****************************************************************************/

static void syslog_flush_internal(bool force, size_t buflen)
{
  FAR char *buffer;
  size_t size;

  /* This logic is performed with the scheduler disabled to protect from
   * concurrent modification by other tasks.
   */

  do
    {
      buffer = circbuf_get_readptr(&g_syslog_intbuffer.circ, &size);
      if (size > 0)
        {
          size = (size >= buflen) ? buflen : size;
          syslog_write_foreach(buffer, size, force);
          circbuf_readcommit(&g_syslog_intbuffer.circ, size);
          buflen -= size;
        }
    }
  while (size > 0 && buflen > 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: syslog_add_intbuffer
 *
 * Description:
 *   Add one more character to the interrupt buffer.  In the event of
 *   buffer overflowed, the character will be dropped.  The indication
 *   "[truncated]\n" will be appended to the end of the interrupt buffer.
 *
 * Input Parameters:
 *   ch - The character to add to the interrupt buffer (must be positive).
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - Called either from (1) interrupt handling logic with interrupts
 *     disabled or from an IDLE thread with interrupts enabled.
 *   - Requires caution because there may be an interrupted execution of
 *     syslog_flush_intbuffer():  Only the outndx can be modified.
 *
 ****************************************************************************/

void syslog_add_intbuffer(FAR const char *buffer, size_t buflen)
{
  irqstate_t flags;
  size_t space;

  /* Disable concurrent modification from interrupt handling logic */

  flags = spin_lock_irqsave_wo_note(&g_syslog_intbuffer.splock);

  space = circbuf_space(&g_syslog_intbuffer.circ);

  if (space >= buflen)
    {
      circbuf_write(&g_syslog_intbuffer.circ, buffer, buflen);
    }
  else if (buflen <= sizeof(g_syslog_intbuffer.buffer))
    {
      syslog_flush_internal(true, buflen - space);
      circbuf_write(&g_syslog_intbuffer.circ, buffer, buflen);
    }
  else
    {
      syslog_flush_internal(true, sizeof(g_syslog_intbuffer.buffer));
      space = buflen - sizeof(g_syslog_intbuffer.buffer);
      syslog_write_foreach(buffer, space, true);
      circbuf_write(&g_syslog_intbuffer.circ,
                    buffer + space, buflen - space);
    }

  spin_unlock_irqrestore_wo_note(&g_syslog_intbuffer.splock, flags);
}

/****************************************************************************
 * Name: syslog_flush_intbuffer
 *
 * Description:
 *   Flush any characters that may have been added to the interrupt buffer
 *   to the SYSLOG device.
 *
 * Input Parameters:
 *   force   - Use the force() method of the channel vs. the putc() method.
 *
 * Returned Value:
 *   On success, the character is echoed back to the caller.  A negated
 *   errno value is returned on any failure.
 *
 * Assumptions:
 *   Interrupts may or may not be disabled.
 *
 ****************************************************************************/

void syslog_flush_intbuffer(bool force)
{
  irqstate_t flags;

  flags = spin_lock_irqsave_wo_note(&g_syslog_intbuffer.splock);
  syslog_flush_internal(force, sizeof(g_syslog_intbuffer.buffer));
  spin_unlock_irqrestore_wo_note(&g_syslog_intbuffer.splock, flags);
}

#endif /* CONFIG_SYSLOG_INTBUFFER */
