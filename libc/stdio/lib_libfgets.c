/****************************************************************************
 * libc/stdio/lib_libfgets.c
 *
 *   Copyright (C) 2007-2008, 2011-2014 Gregory Nutt. All rights reserved.
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

#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include <assert.h>
#include <debug.h>

#include "lib_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Some environments may return CR as end-of-line, others LF, and others
 * both.  If not specified, the logic here assumes either (but not both) as
 * the default.
 */

#if defined(CONFIG_EOL_IS_CR)
#  undef  CONFIG_EOL_IS_LF
#  undef  CONFIG_EOL_IS_BOTH_CRLF
#  undef  CONFIG_EOL_IS_EITHER_CRLF
#elif defined(CONFIG_EOL_IS_LF)
#  undef  CONFIG_EOL_IS_CR
#  undef  CONFIG_EOL_IS_BOTH_CRLF
#  undef  CONFIG_EOL_IS_EITHER_CRLF
#elif defined(CONFIG_EOL_IS_BOTH_CRLF)
#  undef  CONFIG_EOL_IS_CR
#  undef  CONFIG_EOL_IS_LF
#  undef  CONFIG_EOL_IS_EITHER_CRLF
#elif defined(CONFIG_EOL_IS_EITHER_CRLF)
#  undef  CONFIG_EOL_IS_CR
#  undef  CONFIG_EOL_IS_LF
#  undef  CONFIG_EOL_IS_BOTH_CRLF
#else
#  undef  CONFIG_EOL_IS_CR
#  undef  CONFIG_EOL_IS_LF
#  undef  CONFIG_EOL_IS_BOTH_CRLF
#  define CONFIG_EOL_IS_EITHER_CRLF 1
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: consume_eol
 *
 * Description:
 *   If 'consume' is true, then consume_eol() will read and discard bytes from
 *   'stream' until an EOF or a newline encountered or until a read error
 *   occurs.
 *
 **************************************************************************/

static void consume_eol(FILE *stream, bool consume)
{
  if (consume)
    {
      int ch;

      do
        {
          ch = fgetc(stream);
        }
#if  defined(CONFIG_EOL_IS_LF) || defined(CONFIG_EOL_IS_BOTH_CRLF)
      while (ch != EOF && ch != '\n');
#elif defined(CONFIG_EOL_IS_CR)
      while (ch != EOF && ch != '\r');
#else /* elif defined(CONFIG_EOL_IS_EITHER_CRLF) */
      while (ch != EOF && ch != '\n' && ch != '\r');
#endif
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_fgets
 *
 * Description:
 *   lib_fgets() implements the core logic for both fgets() and gets_s().
 *   lib_fgets() reads in at most one less than 'buflen' characters from
 *   stream and stores them into the buffer pointed to by 'buf'. Reading
 *   stops after an EOF or a newline encountered or after a read error
 *   occurs.
 *
 *   If a newline is read, it is stored into the buffer only if 'keepnl' is
 *   set true.  A null terminator is always stored after the last character
 *   in the buffer.
 *
 *   If 'buflen'-1 bytes were read into 'buf' without encountering an EOF
 *   or newline then the following behavior depends on the value of
 *   'consume':  If consume is true, then lib_fgets() will continue reading
 *   bytes and discarding them until an EOF or a newline encountered or
 *   until a read error occurs.  Otherwise, lib_fgets() returns with the
 *   remaining of the incoming stream buffer.
 *
 **************************************************************************/

FAR char *lib_fgets(FAR char *buf, size_t buflen, FILE *stream,
                    bool keepnl, bool consume)
{
  size_t nch = 0;

  /* Sanity checks */

  if (!stream || !buf || stream->fs_fd < 0)
    {
      return NULL;
    }

  /* Make sure that we have a buffer and space for at least one character */

  if (buflen < 1)
    {
      consume_eol(stream, consume);
      return NULL;
    }

  /* Make sure that we have space for something in addition to the NUL
   * terminator.
   */

  if (buflen < 2)
    {
      *buf = '\0';
      consume_eol(stream, consume);
      return buf;
    }

  /* Read characters until we have a full line. On each the loop we must
   * be assured that there are two free bytes in the line buffer:  One for
   * the next character and one for the null terminator.
   */

  for (;;)
    {
      /* Get the next character */

      int ch = fgetc(stream);

      /* Check for end-of-line.  This is tricky only in that some
       * environments may return CR as end-of-line, others LF, and
       * others both.
       */

#if  defined(CONFIG_EOL_IS_LF) || defined(CONFIG_EOL_IS_BOTH_CRLF)
      if (ch == '\n')
#elif defined(CONFIG_EOL_IS_CR)
      if (ch == '\r')
#else /* elif defined(CONFIG_EOL_IS_EITHER_CRLF) */
      if (ch == '\n' || ch == '\r')
#endif
        {
          if (keepnl)
            {
              /* Store newline is stored in the buffer */

              buf[nch++] = '\n';
            }

          /* Terminate the string */

          buf[nch] = '\0';
          return buf;
        }

      /* Check for end-of-file */

      else if (ch == EOF)
        {
          /* End of file with no data? */

          if (!nch)
            {
              /* Yes.. return NULL as the end of file mark */

              return NULL;
            }
          else
            {
              /* No, terminate the accumulated string */

              buf[nch] = '\0';
              return buf;
            }
        }

      /* Otherwise, check if the character is printable and, if so, put the
       * character in the line buffer
       */

      else if (isprint(ch))
        {
          buf[nch++] = ch;

          /* Check if there is room for another character and the line's
           * NUL terminator.  If not then we have to end the line now,
           * perhaps consuming any data up to the end-of-line.
           */

          if (nch + 1 >= buflen)
            {
              buf[nch] = '\0';
              consume_eol(stream, consume);
              return buf;
            }
        }
    }
}
