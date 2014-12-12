/****************************************************************************
 * libc/stdio/lib_sscanf.c
 *
 *   Copyright (C) 2007, 2008, 2011-2014 Gregory Nutt. All rights reserved.
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

#include <nuttx/compiler.h>

#include <sys/types.h>

#include <stdarg.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <debug.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAXLN 128

#ifndef MIN
#  define MIN(a,b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#  define MAX(a,b) (((a) > (b)) ? (a) : (b))
#endif

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Global Function Prototypes
 ****************************************************************************/

int vsscanf(FAR const char *buf, FAR const char *fmt, va_list ap);

/****************************************************************************
 * Global Constant Data
 ****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/****************************************************************************
 * Private Constant Data
 ****************************************************************************/

static const char spaces[] = " \t\n\r\f\v";

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  findwidth
 *
 * Description:
 *    Try to figure out the width of the input data.
 *
 ****************************************************************************/

static int findwidth(FAR const char *buf, FAR const char *fmt)
{
  FAR const char *next = fmt + 1;

  /* No... is there a space after the format? Or does the format string end
   * here?
   */

  if (isspace(*next) || *next == 0)
    {
      /* Use the input up until the first white space is encountered. */

      return strcspn(buf, spaces);
    }

  /* No.. Another possibility is the format character is followed by
   * some recognizable delimiting value.
   */

  if (*next != '%')
    {
      /* If so we will say that the string ends there if we can find that
       * delimiter in the input string.
       */

      FAR const char *ptr = strchr(buf, *next);
      if (ptr)
        {
          return (int)(ptr - buf);
        }
    }

  /* No... the format has no delimiter and is back-to-back with the next
   * format (or is followed by a delimiter that does not exist in the
   * input string).  At this point we just bail and Use the input up until
   * the first white space is encountered.
   *
   * NOTE:  This means that values from the following format may be
   * concatenated with the first. This is a bug.  We have no generic way of
   * determining the width of the data if there is no fieldwidth, no space
   * separating the input, and no usable delimiter character.
   */

  return strcspn(buf, spaces);
}

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Function:  sscanf
 *
 * Description:
 *    ANSI standard sscanf implementation.
 *
 ****************************************************************************/

int sscanf(FAR const char *buf, FAR const char *fmt, ...)
{
  va_list ap;
  int     count;

  va_start(ap, fmt);
  count = vsscanf((FAR const char*)buf, fmt, ap);
  va_end(ap);
  return count;
}

/****************************************************************************
 * Function:  vsscanf
 *
 * Description:
 *    ANSI standard vsscanf implementation.
 *
 ****************************************************************************/

int vsscanf(FAR const char *buf, FAR const char *fmt, va_list ap)
{
  FAR const char *bufstart;
  FAR char       *tv;
  FAR const char *tc;
  bool            lflag;
  bool            noassign;
  int             count;
  int             width;
  int             base = 10;
  char            tmp[MAXLN];

  lvdbg("vsscanf: buf=\"%s\" fmt=\"%s\"\n", buf, fmt);

  /* Remember the start of the input buffer.  We will need this for %n
   * calculations.
   */

  bufstart = buf;

  /* Parse the format, extracting values from the input buffer as needed */

  count    = 0;
  width    = 0;
  noassign = false;
  lflag    = false;

  /* Loop until all characters in the fmt string have been processed.  We
   * may have to continue loop after reaching the end the input data in
   * order to handle trailing %n format specifiers.
   */

  while (*fmt)
    {
      /* Skip over white space */

      while (isspace(*fmt))
        {
          fmt++;
        }

      /* Check for a conversion specifier */

      if (*fmt == '%')
        {
          lvdbg("vsscanf: Specifier found\n");

          /* Check for qualifiers on the conversion specifier */

          fmt++;
          for (; *fmt; fmt++)
            {
              lvdbg("vsscanf: Processing %c\n", *fmt);

              if (strchr("dibouxcsefgn%", *fmt))
                {
                  break;
                }

              if (*fmt == '*')
                {
                  noassign = true;
                }
              else if (*fmt == 'l' || *fmt == 'L')
                {
                  /* NOTE: Missing check for long long ('ll') */

                  lflag = true;
                }
              else if (*fmt >= '1' && *fmt <= '9')
                {
                  for (tc = fmt; isdigit(*fmt); fmt++);
                  strncpy(tmp, tc, fmt - tc);
                  tmp[fmt - tc] = '\0';
                  width = atoi(tmp);
                  fmt--;
                }
            }

          /* Process %s:  String conversion */

          if (*fmt == 's')
            {
              lvdbg("vsscanf: Performing string conversion\n");

              /* Get a pointer to the char * value.  We need to do this even
               * if we have reached the end of the input data in order to
               * update the 'ap' variable.
               */

              tv = NULL;      /* To avoid warnings about begin uninitialized */
              if (!noassign)
                {
                  tv    = va_arg(ap, char*);
                  tv[0] = '\0';
                }

              /* But we only perform the data conversion is we still have
               * bytes remaining in the input data stream.
               */

              if (*buf)
                {
                  /* Skip over white space */

                  while (isspace(*buf))
                    {
                      buf++;
                    }

                  /* Was a fieldwidth specified? */

                  if (!width)
                    {
                      /* No... Guess a field width using some heuristics */

                      int tmpwidth = findwidth(buf, fmt);
                      width = MIN(sizeof(tmp) - 1, tmpwidth);
                    }

                  /* Copy the string (if we are making an assignment) */

                  if (!noassign)
                    {
                      strncpy(tv, buf, width);
                      tv[width] = '\0';
                      count++;
                    }

                  /* Update the buffer pointer past the string in the input */

                  buf += width;
                }
            }

          /* Process %c:  Character conversion */

          else if (*fmt == 'c')
            {
              lvdbg("vsscanf: Performing character conversion\n");

              /* Get a pointer to the char * value.  We need to do this even
               * if we have reached the end of the input data in order to
               * update the 'ap' variable.
               */

              tv = NULL;      /* To avoid warnings about beign uninitialized */
              if (!noassign)
                {
                  tv    = va_arg(ap, char*);
                  tv[0] = '\0';
                }

              /* But we only perform the data conversion is we still have
               * bytes remaining in the input data stream.
               */

              if (*buf)
                {
                  /* Was a field width specified? */

                  if (!width)
                    {
                      /* No, then width is this one single character */

                      width = 1;
                    }

                  /* Copy the character(s) (if we are making an assignment) */

                  if (!noassign)
                    {
                      strncpy(tv, buf, width);
                      tv[width] = '\0';
                      count++;
                    }

                  /* Update the buffer pointer past the character(s) in the
                   * input
                   */

                  buf += width;
                }
            }

          /* Process %d, %o, %b, %x, %u:  Various integer conversions */

          else if (strchr("dobxu", *fmt))
            {
              FAR long *plong = NULL;
              FAR int  *pint  = NULL;
              bool sign;

              lvdbg("vsscanf: Performing integer conversion\n");

              /* Get a pointer to the integer value.  We need to do this even
               * if we have reached the end of the input data in order to
               * update the 'ap' variable.
               */

              if (!noassign)
                {
                  /* We have to check whether we need to return a long or an
                   * int.
                   */

                  if (lflag)
                    {
                      plong = va_arg(ap, long*);
                      *plong = 0;
                    }
                  else
                    {
                      pint = va_arg(ap, int*);
                      *pint = 0;
                    }
                }

              /* But we only perform the data conversion if we still have
               * bytes remaining in the input data stream.
               */

              if (*buf)
                {
                  FAR char *endptr;
                  int       errsave;
                  long      tmplong;

                  /* Skip over any white space before the integer string */

                  while (isspace(*buf))
                    {
                      buf++;
                    }

                  /* The base of the integer conversion depends on the
                   * specific conversion specification.
                   */

                  sign = false;
                  switch (*fmt)
                    {
                    default:
                    case 'd':
                      sign = true;
                    case 'u':
                      base = 10;
                      break;

                    case 'X':
                    case 'x':
                      base = 16;
                      break;

                    case 'o':
                      base = 8;
                      break;

                    case 'b':
                      base = 2;
                      break;
                    }

                  /* Was a field width specified? */

                  if (!width)
                    {
                      /* No... Guess a field width using some heuristics */

                      int tmpwidth = findwidth(buf, fmt);
                      width = MIN(sizeof(tmp) - 1, tmpwidth);
                    }

                  /* Copy the numeric string into a temporary working
                   * buffer.
                   */

                  strncpy(tmp, buf, width);
                  tmp[width] = '\0';

                  lvdbg("vsscanf: tmp[]=\"%s\"\n", tmp);

                  /* Perform the integer conversion */
                  /* Preserve the errno value */

                  errsave = get_errno();
                  set_errno(0);
                  if (sign)
                    {
                      tmplong = strtol(tmp, &endptr, base);
                    }
                  else
                    {
                      tmplong = strtoul(tmp, &endptr, base);
                    }

                  /* Check if the number was successfully converted */

                  if (tmp == endptr || get_errno() == ERANGE)
                    {
                      return count;
                    }

                  /* Move by the actual number of characters converted */

                  buf += (endptr - tmp);
                  set_errno(errsave);
                  if (!noassign)
                    {

                      /* We have to check whether we need to return a long
                       * or an int.
                       */

                      if (lflag)
                        {
                          lvdbg("vsscanf: Return %ld to 0x%p\n",
                                tmplong, plong);
                          *plong = tmplong;
                        }
                      else
                        {
                          lvdbg("vsscanf: Return %ld to 0x%p\n",
                                tmplong, pint);
                          *pint = (int)tmplong;
                        }

                      count++;
                    }
                }
            }

          /* Process %f:  Floating point conversion */

          else if (*fmt == 'f' || *fmt == 'F')
            {
#ifdef CONFIG_HAVE_DOUBLE
              FAR double_t *pd = NULL;
#endif
              FAR float    *pf = NULL;

              lvdbg("vsscanf: Performing floating point conversion\n");

              /* Get a pointer to the double value.  We need to do this even
               * if we have reached the end of the input data in order to
               * update the 'ap' variable.
               */

              if (!noassign)
                {
                  /* We have to check whether we need to return a float or a
                   * double.
                   */

#ifdef CONFIG_HAVE_DOUBLE
                  if (lflag)
                    {
                      pd  = va_arg(ap, double_t*);
                      *pd = 0.0;
                    }
                  else
#endif
                    {
                      pf  = va_arg(ap, float*);
                      *pf = 0.0;
                    }
                }

#ifdef CONFIG_LIBC_FLOATINGPOINT
              /* But we only perform the data conversion is we still have
               * bytes remaining in the input data stream.
               */

              if (*buf)
                {
                  /* Skip over any white space before the real string */

                  while (isspace(*buf))
                    {
                      buf++;
                    }

                  /* Was a fieldwidth specified? */

                  if (!width)
                    {
                      /* No... Guess a field width using some heuristics */

                      width = findwidth(buf, fmt);
                    }

                  /* Copy the real string into a temporary working buffer. */

                  strncpy(tmp, buf, width);
                  tmp[width] = '\0';
                  buf += width;

                  lvdbg("vsscanf: tmp[]=\"%s\"\n", tmp);

                  /* Perform the floating point conversion */

                  if (!noassign)
                    {
                      /* strtod always returns a double */

                      FAR char *endptr;
                      int       errsave;
                      double_t  dvalue;

                      /* Preserve the errno value */

                      errsave = get_errno();
                      set_errno(0);
                      dvalue  = strtod(tmp, &endptr);

                      /* Check if the number was successfully converted */

                      if (tmp == endptr || get_errno() == ERANGE)
                        {
                          return count;
                        }

                      set_errno(errsave);

                      /* We have to check whether we need to return a float
                       * or a double.
                       */

#ifdef CONFIG_HAVE_DOUBLE
                      if (lflag)
                        {
                          lvdbg("vsscanf: Return %f to %p\n", dvalue, pd);
                          *pd = dvalue;
                        }
                      else
#endif
                        {
                          lvdbg("vsscanf: Return %f to %p\n", dvalue, pf);
                          *pf = (float)dvalue;
                        }

                      count++;
                    }
                }
#endif
            }

          /* Process %n:  Character count */

          else if (*fmt == 'n')
            {
              lvdbg("vsscanf: Performing character count\n");

              if (!noassign)
                {
                  size_t nchars = (size_t)(buf - bufstart);

                  /* Note %n does not count as a conversion */

                  if (lflag)
                    {
                      FAR long *plong = va_arg(ap, long*);
                      *plong = (long)nchars;
                    }
                  else
                    {
                      FAR int *pint = va_arg(ap, int*);
                      *pint = (int)nchars;
                    }
                }
            }

          width    = 0;
          noassign = false;
          lflag    = false;

          fmt++;
        }

    /* It is not a conversion specifier */

      else if (*buf)
        {
          /* Skip over any leading spaces in the input buffer */

          while (isspace(*buf))
            {
              buf++;
            }

          /* Skip over matching characters in the buffer and format */

          if (*fmt != *buf)
            {
              break;
            }
          else
            {
              fmt++;
              buf++;
            }
        }
      else
        {
          /* NULL terminator encountered */

          break;
        }
    }

  /* sscanf is required to return EOF if the input ends before the first
   * matching failure or conversion.
   */

  return count ? count : EOF;
}
