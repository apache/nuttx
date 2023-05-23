/****************************************************************************
 * libs/libc/stream/lib_syslogstream.c
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

static void syslogstream_putc(FAR struct lib_outstream_s *this, int ch)
{
  FAR struct lib_syslogstream_s *stream =
                                       (FAR struct lib_syslogstream_s *)this;

  DEBUGASSERT(stream != NULL);
  syslog(stream->priority, "%c", ch);
  stream->public.nput++;
}

static int syslogstream_puts(FAR struct lib_outstream_s *this,
                             FAR const void *buff, int len)
{
  FAR struct lib_syslogstream_s *stream =
                                       (FAR struct lib_syslogstream_s *)this;

  DEBUGASSERT(stream != NULL);
  if (len <= 0)
    {
      return 0;
    }

  syslog(stream->priority, "%.*s", len, (FAR const char *)buff);
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

  stream->public.nput  = 0;
  stream->public.putc  = syslogstream_putc;
  stream->public.puts  = syslogstream_puts;
  stream->public.flush = lib_noflush;
  stream->priority     = priority;
}
