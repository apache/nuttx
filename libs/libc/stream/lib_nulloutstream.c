/****************************************************************************
 * libs/libc/stream/lib_nulloutstream.c
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

#include <stdio.h>
#include <assert.h>
#include <errno.h>

#include "libc.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void nulloutstream_putc(FAR struct lib_outstream_s *self, int ch)
{
  UNUSED(ch);
  DEBUGASSERT(self);
  self->nput++;
}

static int nulloutstream_puts(FAR struct lib_outstream_s *self,
                              FAR const void *buffer, int len)
{
  UNUSED(buffer);
  UNUSED(len);
  DEBUGASSERT(self);
  self->nput += len;
  return len;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_nulloutstream
 *
 * Description:
 *   Initializes a NULL streams. The initialized stream will write all data
 *   to the bit-bucket.
 *
 * Input Parameters:
 *   stream - User allocated, uninitialized instance of struct
 *            lib_outstream_s to be initialized.
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

void lib_nulloutstream(FAR struct lib_outstream_s *stream)
{
  stream->putc  = nulloutstream_putc;
  stream->puts  = nulloutstream_puts;
  stream->flush = lib_noflush;
  stream->nput  = 0;
}
