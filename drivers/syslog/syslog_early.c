/****************************************************************************
 * drivers/syslog/syslog_early.c
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

#include <stdio.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/streams.h>
#include <nuttx/syslog/syslog.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct early_syslograwstream_s
{
  struct lib_outstream_s common;
  int last_ch;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void early_syslog_putc(FAR struct lib_outstream_s *self, int ch)
{
  FAR struct early_syslograwstream_s *stream = (FAR void *)self;

  DEBUGASSERT(stream != NULL);

  stream->last_ch = ch;
  if (ch == '\n')
    {
      up_putc('\r');
    }

  up_putc(ch);

  self->nput++;
}

/****************************************************************************
 * Name: early_syslog_puts
 ****************************************************************************/

static ssize_t early_syslog_puts(FAR struct lib_outstream_s *self,
                                 FAR const void *buff, size_t len)
{
  FAR struct early_syslograwstream_s *stream = (FAR void *)self;

  DEBUGASSERT(stream != NULL);

  if (len <= 0)
    {
      return 0;
    }

  stream->last_ch = ((FAR const char *)buff)[len - 1];

  up_nputs(buff, len);

  return len;
}

/****************************************************************************
 * Name: early_vsyslog
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int early_vsyslog(FAR const IPTR char *fmt, FAR va_list *ap)
{
  struct early_syslograwstream_s stream;
  int ret = 0;

  /* Initialize the output stream structure. */

  stream.common.putc  = early_syslog_putc;
  stream.common.puts  = early_syslog_puts;
  stream.common.nput  = 0;
  stream.common.flush = lib_noflush;
  stream.last_ch      = '\0';

  /* Format the string into the output stream. */

  ret += lib_vsprintf_internal(&stream.common, fmt, *ap);

  /* Ensure the output ends with a newline. */

  if (stream.last_ch != '\n')
    {
      lib_stream_putc(&stream.common, '\n');
      ret++;
    }

  return ret;
}

/****************************************************************************
 * Name: early_syslog
 *
 * Description:
 *   Provides a minimal SYSLOG output facility that can be used during the
 *   very early boot phase or when the system is in a down state, before the
 *   full SYSLOG subsystem or scheduler becomes available.
 *
 *   This function supports basic formatted output similar to printf(), and
 *   sends the resulting characters directly to the low-level output device
 *   using up_putc().  It is primarily intended for debugging or diagnostic
 *   messages in contexts where interrupts may be disabled and locking is
 *   not yet functional.
 *
 *   The function automatically appends a newline character ('\n') if the
 *   formatted message does not already end with one, to keep log output
 *   properly aligned in serial consoles or early boot traces.
 *
 * Input Parameters:
 *   fmt - A printf-style format string.
 *   ... - Variable arguments corresponding to the format specifiers.
 *
 * Returned Value:
 *   Returns the total number of characters output, including any newline
 *   character appended automatically.
 *
 * Notes:
 *   - This function performs no buffering or synchronization.
 *     It directly outputs each character through up_putc(), which should
 *     be safe for use before full system initialization or during panic.
 *   - The internal output stream (early_syslograwstream_s) is simplified
 *     and only supports character and string operations.
 *   - Once the SYSLOG subsystem is initialized, standard syslog_xxx()
 *     interfaces should be used instead.
 *
 ****************************************************************************/

void early_syslog(FAR const char *fmt, ...)
{
  va_list ap;

  /* Let early_vsyslog do the work */

  va_start(ap, fmt);
  early_vsyslog(fmt, &ap);
  va_end(ap);
}

