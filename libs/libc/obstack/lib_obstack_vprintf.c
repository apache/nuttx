/****************************************************************************
 * libs/libc/obstack/lib_obstack_vprintf.c
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

#include <obstack.h>
#include <assert.h>
#include <nuttx/streams.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct obstack_stream
{
  struct lib_outstream_s common;
  FAR struct obstack *h;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static ssize_t obstack_puts(FAR struct lib_outstream_s *self,
                            FAR const void *buf, size_t len)
{
  FAR struct obstack_stream *stream = (FAR struct obstack_stream *)self;

  DEBUGASSERT(self);

  obstack_grow(stream->h, buf, len);

  return len;
}

static void obstack_putc(FAR struct lib_outstream_s *self, int ch)
{
  FAR struct obstack_stream *stream = (FAR struct obstack_stream *)self;

  DEBUGASSERT(self);

  obstack_1grow(stream->h, ch);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: obstack_vprintf
 *
 * Description:
 *   This is similar to the vasprintf except it uses obstack to allocate
 *   string on. The characters are written onto the end of the currently
 *   growing object and terminated by null byte.
 *
 *   The same remarks are applied here as for obstack_printf regarding the
 *   definition location in GlibC.
 *
 * Input Parameters:
 *   h: pointer to the handle used to grow the object.
 *   fmt: format string
 *   ap: format string input as a variable argument list
 *
 * Returned Value:
 *   Number of characters added to the obstack excluding the null byte.
 *
 ****************************************************************************/

int obstack_vprintf(FAR struct obstack *h, FAR const char *fmt, va_list ap)
{
  struct obstack_stream outstream;

  outstream.common.putc = obstack_putc;
  outstream.common.puts = obstack_puts;
  outstream.common.flush = lib_noflush;
  outstream.common.nput = 0;
  outstream.h = h;

  int nbytes = lib_vsprintf(&outstream.common, fmt, ap);

  if (nbytes < 0)
    {
      obstack_free(h, obstack_finish(h));
      return ERROR;
    }

  return nbytes;
}
