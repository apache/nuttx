/****************************************************************************
 * drivers/syslog/syslog_stream.c
 *
 *   Copyright (C) 2012, 2016-2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <unistd.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/syslog/syslog.h>
#include <nuttx/streams.h>

#include "syslog.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: syslogstream_putc
 ****************************************************************************/

static void syslogstream_putc(FAR struct lib_outstream_s *this, int ch)
{
#ifdef CONFIG_SYSLOG_BUFFER
  FAR struct lib_syslogstream_s *stream = (FAR struct lib_syslogstream_s *)this;

  /* Add the incoming character to the buffer */

  stream->buf[stream->nbuf] = ch;
  stream->nbuf++;
  this->nput++;

  /* Is the buffer full?  Did we encounter a new line? */

  if (stream->nbuf >= CONFIG_SYSLOG_BUFSIZE || ch == '\n')
    {
      /* Yes.. then flush the buffer */

      (void)this->flush(this);
    }
#else
  int ret;

  /* Try writing until the write was successful or until an irrecoverable
   * error occurs.
   */

  do
    {
      /* Write the character to the supported logging device.  On failure,
       * syslog_putc returns EOF with the errno value set;
       */

      ret = syslog_putc(ch);
      if (ret != EOF)
        {
          this->nput++;
          return;
        }

      /* The special errno value -EINTR means that syslog_putc() was
       * awakened by a signal.  This is not a real error and must be
       * ignored in this context.
       */
    }
  while (get_errno() == -EINTR);
#endif
}

/****************************************************************************
 * Name: syslogstream_flush
 ****************************************************************************/

#ifdef CONFIG_SYSLOG_BUFFER
static int syslogstream_flush(FAR struct lib_outstream_s *this)
{
  FAR struct lib_syslogstream_s *stream = (FAR struct lib_syslogstream_s *)this;
  int ret = OK;

  /* Is there anything buffered? */

  if (stream->nbuf > 0)
    {
      /* Yes write the buffered data */

      do
        {
          int status = syslog_write(stream->buf, stream->nbuf);
          if (status < 0)
            {
              ret = -get_errno();
            }
          else
            {
              stream->nbuf = 0;
              ret = OK;
            }
        }
      while (ret == -EINTR);
    }

  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: syslogstream
 *
 * Description:
 *   Initializes a stream for use with the configured syslog interface.
 *   Only accessible from with the OS SYSLOG logic.
 *
 * Input parameters:
 *   stream - User allocated, uninitialized instance of struct
 *            lib_syslogstream_s to be initialized.
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

void syslogstream(FAR struct lib_syslogstream_s *stream)
{
#ifdef CONFIG_SYSLOG_BUFFER
  stream->public.put   = syslogstream_putc;
  stream->public.flush = syslogstream_flush;
  stream->public.nput  = 0;
  stream->nbuf         = 0;
#else
  stream->public.put   = syslogstream_putc;
  stream->public.flush = lib_noflush;
  stream->public.nput  = 0;
#endif
}
