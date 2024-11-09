/****************************************************************************
 * libs/libc/stream/lib_syslograwstream.c
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

#include <nuttx/streams.h>
#include <nuttx/syslog/syslog.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_SYSLOG_BUFFER
/****************************************************************************
 * Name: syslograwstream_flush
 ****************************************************************************/

static int syslograwstream_flush(FAR struct lib_outstream_s *self)
{
  FAR struct lib_syslograwstream_s *stream = (FAR void *)self;
  int ret = OK;

  DEBUGASSERT(stream != NULL);

  /* Do we have an IO buffer? Is there anything buffered? */

  if (stream->offset > 0)
    {
      /* Yes write the buffered data */

      do
        {
          ssize_t nbytes = syslog_write(stream->buffer, stream->offset);
          if (nbytes < 0)
            {
              ret = nbytes;
            }
          else
            {
              ret = OK;
            }
        }
      while (ret == -EINTR);
    }

  stream->offset = 0;
  return ret;
}

/****************************************************************************
 * Name: syslograwstream_addchar
 ****************************************************************************/

static void syslograwstream_addchar(FAR struct lib_syslograwstream_s *stream,
                                    int ch)
{
  /* Add the incoming character to the buffer */

  stream->buffer[stream->offset] = ch;
  stream->offset++;

  /* Increment the total number of bytes buffered. */

  stream->common.nput++;

  /* Is the buffer full? */

  if (stream->offset >= CONFIG_SYSLOG_BUFSIZE)
    {
      /* Yes.. then flush the buffer */

      syslograwstream_flush(&stream->common);
    }
}

/****************************************************************************
 * Name: syslograwstream_addstring
 ****************************************************************************/

static ssize_t
syslograwstream_addstring(FAR struct lib_syslograwstream_s *stream,
                          FAR const char *buff, size_t len)
{
  ssize_t ret = 0;

  do
    {
      size_t remain = CONFIG_SYSLOG_BUFSIZE - stream->offset;
      remain = remain > len - ret ? len - ret : remain;
      memcpy(stream->buffer + stream->offset, buff + ret, remain);
      stream->offset += remain;
      ret += remain;

      /* Is the buffer enough? */

      if (stream->offset >= CONFIG_SYSLOG_BUFSIZE)
        {
          /* Yes.. then flush the buffer */

          syslograwstream_flush(&stream->common);
        }
    }
  while (ret < len);

  /* Increment the total number of bytes buffered. */

  stream->common.nput += len;
  return len;
}
#endif

/****************************************************************************
 * Name: syslograwstream_putc
 ****************************************************************************/

static void syslograwstream_putc(FAR struct lib_outstream_s *self, int ch)
{
  FAR struct lib_syslograwstream_s *stream = (FAR void *)self;

  DEBUGASSERT(stream != NULL);
  stream->last_ch = ch;

  /* Discard carriage returns */

  if (ch != '\r')
    {
#ifdef CONFIG_SYSLOG_BUFFER
      /* Add the incoming character to the buffer */

      syslograwstream_addchar(stream, ch);
#else
      int ret;

      /* Try writing until the write was successful or until an
       * irrecoverable error occurs.
       */

      do
        {
          /* Write the character to the supported logging device.  On
           * failure, syslog_putc returns a negated errno value.
           */

          ret = syslog_putc(ch);
          if (ret >= 0)
            {
              self->nput++;
              return;
            }

          /* The special return value -EINTR means that syslog_putc() was
           * awakened by a signal.  This is not a real error and must be
           * ignored in this context.
           */
        }
      while (ret == -EINTR);
#endif
    }
}

static ssize_t syslograwstream_puts(FAR struct lib_outstream_s *self,
                                    FAR const void *buff, size_t len)
{
  FAR struct lib_syslograwstream_s *stream = (FAR void *)self;

  DEBUGASSERT(stream != NULL);
  if (len <= 0)
    {
      return 0;
    }

  stream->last_ch = ((FAR const char *)buff)[len - 1];

#ifdef CONFIG_SYSLOG_BUFFER

  /* Add the incoming string to the buffer */

  return syslograwstream_addstring(stream, buff, len);
#else
  ssize_t ret;

  /* Try writing until the write was successful or until an
   * irrecoverable error occurs.
   */

  do
    {
      /* Write the buffer to the supported logging device.  On
       * failure, syslog_write returns a negated errno value.
       */

      ret = syslog_write(buff, len);
      if (ret >= 0)
        {
          self->nput += ret;
          return ret;
        }

      /* The special return value -EINTR means that syslog_putc() was
       * awakened by a signal.  This is not a real error and must be
       * ignored in this context.
       */
    }
  while (ret == -EINTR);

  return ret;

#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_syslograwstream_open
 *
 * Description:
 *   Initializes a stream for use with the configured syslog interface.
 *   Only accessible from with the OS SYSLOG logic.
 *
 * Input Parameters:
 *   stream - User allocated, uninitialized instance of struct
 *            lib_syslograwstream_s to be initialized.
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

void lib_syslograwstream_open(FAR struct lib_syslograwstream_s *stream)
{
  DEBUGASSERT(stream != NULL);

  /* Initialize the common fields */

  stream->common.putc  = syslograwstream_putc;
  stream->common.puts  = syslograwstream_puts;
  stream->common.nput  = 0;
  stream->last_ch      = '\0';

#ifdef CONFIG_SYSLOG_BUFFER
  stream->common.flush = syslograwstream_flush;
  stream->offset       = 0;
#else
  stream->common.flush = lib_noflush;
#endif
}

/****************************************************************************
 * Name: lib_syslograwstream_close
 *
 * Description:
 *   Free resources held by the syslog stream.
 *
 * Input Parameters:
 *   stream - User allocated, uninitialized instance of struct
 *            lib_syslograwstream_s to be initialized.
 *
 * Returned Value:
 *   None (Resources freed).
 *
 ****************************************************************************/

#ifdef CONFIG_SYSLOG_BUFFER
void lib_syslograwstream_close(FAR struct lib_syslograwstream_s *stream)
{
  DEBUGASSERT(stream != NULL);

  syslograwstream_flush(&stream->common);
}
#endif
