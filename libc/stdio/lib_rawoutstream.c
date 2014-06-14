/****************************************************************************
 * libc/stdio/lib_rawsostream.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

#include <unistd.h>
#include <assert.h>
#include <errno.h>

#include "lib_internal.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rawsostream_putc
 ****************************************************************************/

static void rawsostream_putc(FAR struct lib_sostream_s *this, int ch)
{
  FAR struct lib_rawsostream_s *rthis = (FAR struct lib_rawsostream_s *)this;
  int nwritten;
  char buffer = ch;

  DEBUGASSERT(this && rthis->fd >= 0);

  /* Loop until the character is successfully transferred or until an
   * irrecoverable error occurs.
   */

  do
    {
      nwritten = write(rthis->fd, &buffer, 1);
      if (nwritten == 1)
        {
          this->nput++;
          return;
        }

      /* The only expected error is EINTR, meaning that the write operation
       * was awakened by a signal.  Zero would not be a valid return value
       * from write().
       */

      DEBUGASSERT(nwritten < 0);
    }
  while (get_errno() == EINTR);
}

/****************************************************************************
 * Name: rawsostream_seek
 ****************************************************************************/

static off_t rawsostream_seek(FAR struct lib_sostream_s *this, off_t offset,
                              int whence)
{
  FAR struct lib_rawsostream_s *mthis = (FAR struct lib_rawsostream_s *)this;

  DEBUGASSERT(this);
  return lseek(mthis->fd, offset, whence);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_rawsostream
 *
 * Description:
 *   Initializes a stream for use with a file descriptor.
 *
 * Input parameters:
 *   outstream - User allocated, uninitialized instance of struct
 *               lib_rawsostream_s to be initialized.
 *   fd        - User provided file/socket descriptor (must have been opened
 *               for write access).
 *
 * Returned Value:
 *   None (User allocated instance initialized).
 *
 ****************************************************************************/

void lib_rawsostream(FAR struct lib_rawsostream_s *outstream, int fd)
{
  outstream->public.put   = rawsostream_putc;
#ifdef CONFIG_STDIO_LINEBUFFER
  outstream->public.flush = lib_snoflush;
#endif
  outstream->public.seek  = rawsostream_seek;
  outstream->public.nput  = 0;
  outstream->fd           = fd;
}
