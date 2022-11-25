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

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>

#include <nuttx/mm/iob.h>
#include <nuttx/streams.h>
#include <nuttx/syslog/syslog.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: syslogstream_flush
 ****************************************************************************/

#ifdef CONFIG_SYSLOG_BUFFER
static int syslogstream_flush(FAR struct lib_syslogstream_s *stream)
{
  FAR struct iob_s *iob;
  int ret = OK;

  DEBUGASSERT(stream != NULL);
  iob = stream->iob;

  /* Do we have an IO buffer? Is there anything buffered? */

  if (iob != NULL && iob->io_len > 0)
    {
      /* Yes write the buffered data */

      do
        {
          ssize_t nbytes = syslog_write((FAR const char *)iob->io_data,
                                        iob->io_len);
          if (nbytes < 0)
            {
              ret = nbytes;
            }
          else
            {
              iob->io_len = 0;
              ret = OK;
            }
        }
      while (ret == -EINTR);
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: syslogstream_addchar
 ****************************************************************************/

#ifdef CONFIG_SYSLOG_BUFFER
static void syslogstream_addchar(FAR struct lib_syslogstream_s *stream,
                                 int ch)
{
  FAR struct iob_s *iob = stream->iob;

  /* Add the incoming character to the buffer */

  iob->io_data[iob->io_len] = ch;
  iob->io_len++;

  /* Increment the total number of bytes buffered. */

  stream->public.nput++;

  /* Is the buffer full? */

  if (iob->io_len >= CONFIG_IOB_BUFSIZE)
    {
      /* Yes.. then flush the buffer */

      syslogstream_flush(stream);
    }
}
#endif

/****************************************************************************
 * Name: syslogstream_putc
 ****************************************************************************/

static void syslogstream_putc(FAR struct lib_outstream_s *this, int ch)
{
  FAR struct lib_syslogstream_s *stream =
                                      (FAR struct lib_syslogstream_s *)this;

  DEBUGASSERT(stream != NULL);
  stream->last_ch = ch;

  /* Discard carriage returns */

  if (ch != '\r')
    {
#ifdef CONFIG_SYSLOG_BUFFER
      /* Do we have an IO buffer? */

      if (stream->iob != NULL)
        {
          /* Add the incoming character to the buffer */

          syslogstream_addchar(stream, ch);
        }
      else
#endif
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_syslogstream_open
 *
 * Description:
 *   Initializes a stream for use with the configured syslog interface.
 *   Only accessible from with the OS SYSLOG logic.
 *
 * Input Parameters:
 *   stream - User allocated, uninitialized instance of struct
 *            lib_syslogstream_s to be initialized.
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

void lib_syslogstream_open(FAR struct lib_syslogstream_s *stream)
{
#ifdef CONFIG_SYSLOG_BUFFER
  FAR struct iob_s *iob;
#endif

  DEBUGASSERT(stream != NULL);

  /* Initialize the common fields */

  stream->public.put   = syslogstream_putc;
  stream->public.flush = lib_noflush;
  stream->public.nput  = 0;

#ifdef CONFIG_SYSLOG_BUFFER
  /* Allocate an IOB */

  iob                  = iob_tryalloc(true);
  stream->iob          = iob;

  if (iob != NULL)
    {
      /* Initialize the IOB */

      iob->io_len      = 0;
      iob->io_offset   = 0;
      iob->io_pktlen   = 0;
    }
#endif
}

/****************************************************************************
 * Name: lib_syslogstream_close
 *
 * Description:
 *   Free resources held by the syslog stream.
 *
 * Input Parameters:
 *   stream - User allocated, uninitialized instance of struct
 *            lib_syslogstream_s to be initialized.
 *
 * Returned Value:
 *   None (Resources freed).
 *
 ****************************************************************************/

void lib_syslogstream_close(FAR struct lib_syslogstream_s *stream)
{
  DEBUGASSERT(stream != NULL);

  if (stream->last_ch != '\n')
    {
      syslogstream_putc(&stream->public, '\n');
    }

#ifdef CONFIG_SYSLOG_BUFFER
  /* Verify that there is an IOB attached (there should be) */

  if (stream->iob != NULL)
    {
      /* Flush the output buffered in the IOB */

      syslogstream_flush(stream);

      /* Free the IOB */

      iob_free(stream->iob);
      stream->iob = NULL;
    }
#endif
}
