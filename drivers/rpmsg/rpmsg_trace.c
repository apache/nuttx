/****************************************************************************
 * drivers/rpmsg/rpmsg_trace.c
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

#include <stdarg.h>
#include <stdint.h>
#include <sys/param.h>
#include <stdbool.h>
#include <stdio.h>
#include <execinfo.h>

#include <nuttx/sched_note.h>
#include <nuttx/spinlock.h>
#include <nuttx/streams.h>
#include <nuttx/rpmsg/rpmsg.h>

#include "rpmsg_procfs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* nc_length is a uint8_t, so a payload has to be split into notes that
 * each fit in it.
 */

#define RPMSG_TRACE_CHUNK (UINT8_MAX - SIZEOF_NOTE_EVENT(0))

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Keeps a text record and its binary payload next to each other in the
 * note stream when several CPUs trace at the same time.
 */

static spinlock_t g_rpmsg_trace_lock = SP_UNLOCKED;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpmsg_trace
 ****************************************************************************/

void rpmsg_trace(FAR const char *name, bool bt, FAR const void *buf,
                 size_t len, FAR const char *format, ...)
{
  va_list ap;

  va_start(ap, format);
  rpmsg_vtrace(name, bt, buf, len, format, ap);
  va_end(ap);
}

/****************************************************************************
 * Name: rpmsg_vtrace
 ****************************************************************************/

void rpmsg_vtrace(FAR const char *name, bool bt, FAR const void *buf,
                  size_t len, FAR const char *format, va_list ap)
{
  irqstate_t flags;

  if (!rpmsg_procfs_trace_allow(name))
    {
      return;
    }

#if CONFIG_RPMSG_TRACE_BACKTRACE > 0
  if (bt)
    {
      FAR void *frames[CONFIG_RPMSG_TRACE_BACKTRACE];
      char btbuf[BACKTRACE_BUFFER_SIZE(CONFIG_RPMSG_TRACE_BACKTRACE)];
      char line[256];
      va_list copy;
      int n;

      /* The note driver keeps the raw arguments and formats them later,
       * so the message must already be a string. 256 bytes is the size
       * of one note printf record.
       */

      va_copy(copy, ap);
      n = vsnprintf(line, sizeof(line), format, copy);
      va_end(copy);

      n = MIN(MAX(n, 0), (int)sizeof(line) - 1);
      if (n > 0 && line[n - 1] == '\n')
        {
          n--;
        }

      backtrace(frames, CONFIG_RPMSG_TRACE_BACKTRACE);
      backtrace_format(btbuf, sizeof(btbuf), frames,
                       CONFIG_RPMSG_TRACE_BACKTRACE);
      snprintf(line + n, sizeof(line) - n, ", %s", btbuf);
      flags = spin_lock_irqsave(&g_rpmsg_trace_lock);
      sched_note_printf(NOTE_TAG_RPMSG, "%s", line);
    }
  else
#endif
    {
      va_list copy;

      /* va_list is an array on some ABIs, so &ap of a parameter is not a
       * va_list *; take a local copy.
       */

      va_copy(copy, ap);
      flags = spin_lock_irqsave(&g_rpmsg_trace_lock);
      sched_note_vprintf(NOTE_TAG_RPMSG, format, copy);
      va_end(copy);
    }

  while (buf != NULL && len > 0)
    {
      size_t chunk = MIN(len, RPMSG_TRACE_CHUNK);

      sched_note_event(NOTE_TAG_RPMSG, NOTE_DUMP_BINARY, buf, chunk);
      buf = (FAR const uint8_t *)buf + chunk;
      len -= chunk;
    }

  spin_unlock_irqrestore(&g_rpmsg_trace_lock, flags);
}
