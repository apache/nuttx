/****************************************************************************
 * libs/libc/stream/lib_syslograwstream.c
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

#include <nuttx/mm/iob.h>
#include <nuttx/streams.h>
#include <nuttx/syslog/syslog.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_SYSLOG_BUFFER
/****************************************************************************
 * Name: syslograwstream_flush
 ****************************************************************************/

static int syslograwstream_flush(FAR struct lib_outstream_s *ostream)
{
  FAR struct lib_syslograwstream_s *stream = (FAR void *)ostream;
  int ret = OK;

  DEBUGASSERT(stream != NULL);

  /* Do we have an IO buffer? Is there anything buffered? */

  if (stream->base != NULL && stream->offset > 0)
    {
      /* Yes write the buffered data */

      do
        {
          ssize_t nbytes = syslog_write(stream->base, stream->offset);
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

  stream->base[stream->offset] = ch;
  stream->offset++;

  /* Increment the total number of bytes buffered. */

  stream->public.nput++;

  /* Is the buffer full? */

  if (stream->offset >= stream->size)
    {
      /* Yes.. then flush the buffer */

      syslograwstream_flush(&stream->public);
    }
}

/****************************************************************************
 * Name: syslograwstream_addstring
 ****************************************************************************/

static int
syslograwstream_addstring(FAR struct lib_syslograwstream_s *stream,
                          FAR const char *buff, int len)
{
  int ret = 0;

  do
    {
      int remain = stream->size - stream->offset;
      remain = remain > len - ret ? len - ret : remain;
      memcpy(stream->base + stream->offset, buff + ret, remain);
      stream->offset += remain;
      ret += remain;

      /* Is the buffer enough? */

      if (stream->offset >= stream->size)
        {
          /* Yes.. then flush the buffer */

          syslograwstream_flush(&stream->public);
        }
    }
  while (ret < len);

  /* Increment the total number of bytes buffered. */

  stream->public.nput += len;
  return len;
}
#endif

/****************************************************************************
 * Name: syslograwstream_putc
 ****************************************************************************/

static void syslograwstream_putc(FAR struct lib_outstream_s *this, int ch)
{
  FAR struct lib_syslograwstream_s *stream =
                                    (FAR struct lib_syslograwstream_s *)this;

  DEBUGASSERT(stream != NULL);
  stream->last_ch = ch;

  /* Discard carriage returns */

  if (ch != '\r')
    {
#  ifdef CONFIG_SYSLOG_BUFFER
      /* Do we have an IO buffer? */

      if (stream->base != NULL)
        {
          /* Add the incoming character to the buffer */

          syslograwstream_addchar(stream, ch);
        }
      else
#  endif
        {
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
                  this->nput++;
                  return;
                }

              /* The special return value -EINTR means that syslog_putc() was
               * awakened by a signal.  This is not a real error and must be
               * ignored in this context.
               */
            }
          while (ret == -EINTR);
        }
    }
}

static int syslograwstream_puts(FAR struct lib_outstream_s *this,
                                FAR const void *buff, int len)
{
  FAR struct lib_syslograwstream_s *stream =
                                    (FAR struct lib_syslograwstream_s *)this;
  int ret;

  DEBUGASSERT(stream != NULL);
  if (len <= 0)
    {
      return 0;
    }

  stream->last_ch = ((FAR const char *)buff)[len - 1];

#ifdef CONFIG_SYSLOG_BUFFER

  /* Do we have an IO buffer? */

  if (stream->base != NULL)
    {
      /* Add the incoming string to the buffer */

      ret = syslograwstream_addstring(stream, buff, len);
    }
  else
#endif
    {
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
              this->nput += ret;
              return ret;
            }

          /* The special return value -EINTR means that syslog_putc() was
           * awakened by a signal.  This is not a real error and must be
           * ignored in this context.
           */
        }
      while (ret == -EINTR);
    }

  return ret;
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

  stream->public.putc  = syslograwstream_putc;
  stream->public.puts  = syslograwstream_puts;
  stream->public.nput  = 0;

#ifdef CONFIG_SYSLOG_BUFFER
  stream->public.flush = syslograwstream_flush;

  /* Allocate an IOB */

#  ifdef CONFIG_MM_IOB
  stream->iob = iob_tryalloc(true);
  if (stream->iob != NULL)
    {
      stream->base = (FAR void *)stream->iob->io_data;
      stream->size = sizeof(stream->iob->io_data);
    }
#  else
  stream->base = stream->buffer;
  stream->size = sizeof(stream->buffer);
#  endif
#else
  stream->public.flush = lib_noflush;
#endif
  stream->offset = 0;
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

  /* Verify that there is an IOB attached (there should be) */

#  ifdef CONFIG_MM_IOB
  if (stream->iob != NULL)
    {
      /* Flush the output buffered in the IOB */

      syslograwstream_flush(&stream->public);

      /* Free the IOB */

      iob_free(stream->iob);
      stream->iob = NULL;
    }
#  else
  syslograwstream_flush(&stream->public);
#  endif
}
#endif
