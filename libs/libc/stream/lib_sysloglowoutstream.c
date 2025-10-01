/****************************************************************************
 * libs/libc/stream/lib_sysloglowoutstream.c
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

#include <nuttx/arch.h>
#include <nuttx/streams.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sysloglowoutstream_putc
 ****************************************************************************/

static void
sysloglowoutstream_putc(FAR struct lib_outstream_s *self, int ch)
{
  FAR struct lib_sysloglowoutstream_s *stream = (FAR void *)self;

  DEBUGASSERT(stream != NULL);

  stream->last_ch = ch;

  up_putc(ch);

  self->nput++;
}

/****************************************************************************
 * Name: sysloglowoutstream_puts
 ****************************************************************************/

static ssize_t sysloglowoutstream_puts(FAR struct lib_outstream_s *self,
                                 FAR const void *buff, size_t len)
{
  FAR struct lib_sysloglowoutstream_s *stream = (FAR void *)self;

  DEBUGASSERT(stream != NULL);

  ssize_t nput = 0;

  if (len <= 0)
    {
      return 0;
    }

  stream->last_ch = ((FAR const char *)buff)[len - 1];

  while (len-- > 0)
    {
      up_putc(((FAR const char *)buff)[nput++]);
    }

  return nput;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_sysloglowoutstream
 *
 * Description:
 *   Initializes syslog stream
 *
 * Input Parameters:
 *   stream   - User allocated, uninitialized instance of struct
 *              lib_outstream_s to be initialized.
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

void lib_sysloglowoutstream(FAR struct lib_sysloglowoutstream_s *stream)
{
  DEBUGASSERT(stream != NULL);

  /* Initialize the common fields */

  stream->common.nput  = 0;
  stream->common.putc  = sysloglowoutstream_putc;
  stream->common.puts  = sysloglowoutstream_puts;
  stream->common.flush = lib_noflush;
  stream->last_ch      = '\0';
}
