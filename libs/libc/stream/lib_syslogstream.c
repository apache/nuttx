/****************************************************************************
 * libs/libc/stream/lib_syslogstream.c
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

#include <assert.h>
#include <errno.h>
#include <stddef.h>
#include <syslog.h>

#include <nuttx/streams.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: syslogstream_putc
 ****************************************************************************/

static void syslogstream_putc(FAR struct lib_outstream_s *self, int ch)
{
  FAR struct lib_syslogstream_s *stream =
                                       (FAR struct lib_syslogstream_s *)self;

  DEBUGASSERT(stream != NULL);
  syslog(stream->priority, "%c", ch);
  stream->common.nput++;
}

static ssize_t syslogstream_puts(FAR struct lib_outstream_s *self,
                                 FAR const void *buff, size_t len)
{
  FAR struct lib_syslogstream_s *stream =
                                       (FAR struct lib_syslogstream_s *)self;

  DEBUGASSERT(stream != NULL);
  if (len <= 0)
    {
      return 0;
    }

  syslog(stream->priority, "%.*s", (int)len, (FAR const char *)buff);
  return len;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_syslogstream
 *
 * Description:
 *   Initializes syslog stream
 *
 * Input Parameters:
 *   stream   - User allocated, uninitialized instance of struct
 *              lib_syslogstream_s to be initialized.
 *   priority - log priority.
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

void lib_syslogstream(FAR struct lib_syslogstream_s *stream, int priority)
{
  DEBUGASSERT(stream != NULL);

  /* Initialize the common fields */

  stream->common.nput  = 0;
  stream->common.putc  = syslogstream_putc;
  stream->common.puts  = syslogstream_puts;
  stream->common.flush = lib_noflush;
  stream->priority     = priority;
}
