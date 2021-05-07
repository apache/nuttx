/****************************************************************************
 * drivers/syslog/syslog_emergstream.c
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
 * Name: emergstream_putc
 ****************************************************************************/

static void emergstream_putc(FAR struct lib_outstream_s *this, int ch)
{
  int ret;

  /* Try writing until the write was successful or until an irrecoverable
   * error occurs.
   */

  do
    {
      /* Write the character to the supported logging device.  On failure,
       * syslog_force returns a negated errno value.
       */

      ret = syslog_force(ch);
      if (ret >= 0)
        {
          this->nput++;
          return;
        }

      /* The special return value -EINTR means that syslog_force() was
       * awakened by a signal.  This is not a real error and must be
       * ignored in this context.
       */
    }
  while (ret == -EINTR);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: emergstream
 *
 * Description:
 *   Initializes a stream for use with the configured emergency syslog
 *   interface.  Only accessible from with the OS SYSLOG logic.
 *
 * Input Parameters:
 *   stream - User allocated, uninitialized instance of struct
 *            lib_outstream_s to be initialized.
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

void emergstream(FAR struct lib_outstream_s *stream)
{
  stream->put   = emergstream_putc;
  stream->flush = lib_noflush;
  stream->nput  = 0;
}
