/************************************************************
 * lib_fgets.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
 ************************************************************/

/************************************************************
 * Included Files
 ************************************************************/

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <debug.h>

/************************************************************
 * Definitions
 ************************************************************/

/* In some systems, the underlying serial logic may
 * automatically echo characters back to the console.  We
 * will assume that that is not the case here
 */

#define CONFIG_FGETS_ECHO 1

/* Some environments may return CR as end-of-line, others LF, and
 * others both.  The logic here assumes either but not both.
 */

#undef  CONFIG_EOL_IS_CR
#undef  CONFIG_EOL_IS_LF
#undef  CONFIG_EOL_IS_BOTH_CRLF
#define CONFIG_EOL_IS_EITHER_CRLF 1

/************************************************************
 * Private Type Declarations
 ************************************************************/

/************************************************************
 * Private Function Prototypes
 ************************************************************/

/************************************************************
 * Global Function Prototypes
 ************************************************************/

/************************************************************
 * Global Constant Data
 ************************************************************/

/************************************************************
 * Global Variables
 ************************************************************/

/************************************************************
 * Private Constant Data
 ************************************************************/

/************************************************************
 * Private Variables
 ************************************************************/

/************************************************************
 * Private Functions
 ************************************************************/

/************************************************************
 * Name: _lib_rawgetc
 ************************************************************/

static inline int _lib_rawgetc(int fd)
{
  char buffer;
  (void)read(fd, &buffer, 1);
  return buffer;
}

/************************************************************
 * Name: _lib_consoleputc
 ************************************************************/

#ifdef CONFIG_FGETS_ECHO
static inline void _lib_consoleputc(int ch)
{
  char buffer = ch;
  (void)write(1, &buffer, 1);
}
#endif

/************************************************************
 * Name: _lib_consoleputs
 ************************************************************/

#ifdef CONFIG_FGETS_ECHO
static inline void _lib_consoleputs(char *s)
{
  (void)write(1, s, strlen(s));
}
#endif

/************************************************************
 * Global Functions
 ************************************************************/

/************************************************************
 * Name: fgets
 *
 * Description:
 *   fgets() reads in at most one less than 'n' characters
 *   from stream and stores them into the buffer pointed to
 *   by 's'. Reading stops after an EOF or a newline.  If a
 *   newline is read, it is stored into the buffer.  A null
 *   terminator is stored after the last character in the
 *   buffer.
 *
 * Assumptions:
 *   If the stream corresponds to stdin (fd=0) this version
 *   will assume that we are reading characters from a
 *   VT100 console and that stdout (fd=1) is also available.
 *   This will not work well if fd=0 corresponds to a raw
 *   byte steam.
 *
 **********************************************************/

char *fgets(FAR char *s, int n, FILE *stream)
{
#ifdef CONFIG_FGETS_ECHO
  boolean console;
#endif
  int     escape = 0;
  int     nch = 0;

  /* Sanity checks */

  if (!stream || !s || n < 1 || stream->fs_filedes < 0)
    {
      return NULL;
    }

  if (n < 2)
    {
      *s = '\0';
      return s;
    }

  /* Check if the stream is stdin */

#ifdef CONFIG_FGETS_ECHO
  console = (stream->fs_filedes == 0);

  /* <esc>[K is the VT100 command that erases to the end of the line. */

  if (console)
    {
      _lib_consoleputs("\033[K");
    }
#endif

  /* Read characters until we have a full line. On each
   * the loop we must be assured that there are two free bytes
   * in the line buffer:  One for the next character and one for
   * the null terminator.
   */

  for(;;)
    {
      /* Get the next character */

      int ch = _lib_rawgetc(stream->fs_filedes);

      /* Are we processing a VT100 escape sequence */

      if (escape)
        {
          /* Yes, is it an <esc>[, 3 byte sequence */

          if (ch != 0x5b || escape == 2)
            {
              /* We are finished with the escape sequence */

              escape = 0;
              ch = 'a';
            }
          else
            {
              /* The next character is the end of a 3-byte sequence.
               * NOTE:  Some of the <esc>[ sequences are longer than
               * 3-bytes, but I have not encountered any in normal use
               * yet and, so, have not provided the decoding logic.
               */

              escape = 2;
            }
        }

      /* Check for backspace */

      else if (ch == 0x08)
        {
          /* Eliminate that last character in the buffer. */

          if (nch > 0)
            {
              nch--;

#ifdef CONFIG_FGETS_ECHO
              if (console)
                {
                  /* Echo the backspace character on the console */

                  _lib_consoleputc(ch);
                }
#endif
            }
        }

      /* Check for the beginning of a VT100 escape sequence */

      else if (console && ch == 0x1b)
        {
          /* The next character is escaped */

          escape = 1;
        }

      /* Check for end-of-line.  This is tricky only in that some
       * environments may return CR as end-of-line, others LF, and
       * others both.
       */

#if  defined(CONFIG_EOL_IS_LF) || defined(CONFIG_EOL_IS_BOTH_CRLF)
      else if (ch == '\n')
#elif defined(CONFIG_EOL_IS_CR)
      else if (ch == '\r')
#elif CONFIG_EOL_IS_EITHER_CRLF
      else if (ch == '\n' || ch == '\r')
#endif
        {
          /* The newline is stored in the buffer along
           * with the null terminator.
           */

          s[nch++] = '\n';
          s[nch]   = '\0';

#ifdef CONFIG_FGETS_ECHO
          if (console)
            {
              /* Echo the newline to the console */

              _lib_consoleputc('\n');
            }
#endif
          return s;
        }

      /* Check for end-of-file */

      else if (ch == EOF)
        {
          /* Terminate the line */

          s[nch]   = '\0';
          return s;
        }

      /* Otherwise, check if the character is printable and,
       * if so, put the character in the line buffer
       */

      else if (isprint(ch))
        {
          s[nch++] = ch;

#ifdef CONFIG_FGETS_ECHO
          if (console)
            {
              /* Echo the character to the console */

              _lib_consoleputc(ch);
            }
#endif
          /* Check if there is room for another character
           * and the line's null terminator.  If not then
           * we have to end the line now.
           */

          if (nch + 1 >= n)
            {
              s[nch] = '\0';
              return s;
            }
        }
    }

}
