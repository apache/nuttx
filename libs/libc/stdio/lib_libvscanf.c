/****************************************************************************
 * libs/libc/stdio/lib_libvscanf.c
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

#include <nuttx/streams.h>

#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <ctype.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CONFIG_LIBC_LONG_LONG is not a valid selection of the compiler does not
 * support long long types.
 */

#ifndef CONFIG_HAVE_LONG_LONG
#  undef CONFIG_LIBC_LONG_LONG
#endif

#define MAXLN   128

#define HH_MOD -2
#define H_MOD  -1
#define NO_MOD  0
#define L_MOD   1
#define LL_MOD  2

/* Support special access to CODE-space strings for Harvard architectures */

#ifdef CONFIG_ARCH_ROMGETC
#  define fmt_char(fmt)   up_romgetc(fmt)
#else
#  define fmt_char(fmt)   (*(fmt))
#endif

#define buf_arg(buf, type) \
  ((buf) = (FAR char *)(buf) + sizeof(*(type)0), \
  (type)((FAR char *)(buf) - sizeof(*(type)0)))

#define next_arg(varg, vabuf, type) \
  (varg) ? va_arg((vabuf).ap, type) : buf_arg((vabuf).buf, type)

#define buf_arg_width(buf, type, width) \
  ((buf) = (FAR char *)(buf) + (width), (type)((FAR char *)(buf) - (width)))

#define next_arg_width(varg, vabuf, type, width) \
  (varg) ? va_arg((vabuf).ap, type) : buf_arg_width((vabuf).buf, type, width)

/****************************************************************************
 * Private Types
 ****************************************************************************/

union vabuf_u
{
  FAR const void *buf;
  va_list ap;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  findscanset
 *
 * Description:
 *    Fill in the given table from the scanset at the given format.
 *    Return a pointer to the character the closing ']'.
 *    The table has a 1 wherever characters should be considered part of the
 *    scanset.
 *
 *    Function findscanset based on source function __sccl of FreeBSD
 *   (https://github.com/lattera/freebsd/blob/master/sys/kern/subr_scanf.c)
 *
 ****************************************************************************/

#ifdef CONFIG_LIBC_SCANSET
static FAR const char *findscanset(FAR const char *fmt,
                                   FAR unsigned char set[32])
{
  int c;
  int n;
  int v;
  int i;

  fmt++;                        /* Skip '[' */

  /* First `clear' the whole table */

  c = fmt_char(fmt++);                   /* First char hat => negated scanset */
  if (c == '^')
    {
      v = 1;                    /* Default => accept */
      c = fmt_char(fmt++);      /* Get new first char */
    }
  else
    {
      v = 0;                    /* Default => reject */
    }

  memset(set, 0, 32);
  if (c == 0)
    {
      goto doexit;
    }

  /* Now set the entries corresponding to the actual scanset to the opposite
   * of the above. The first character may be ']' (or '-') without being
   * special; the last character may be '-'.
   */

  for (; ; )
    {
      set[c / 8] |= (1 << (c % 8));     /* Take character c */

doswitch:
      n = fmt_char(fmt++);       /* Examine the next */
      switch (n)
        {
        case 0:                /* Format ended too soon */
        case ']':              /* End of scanset */
          goto doexit;

        case '-':
          /* A scanset of the form [01+-] is defined as "the digit 0, the
           * digit 1, the character +, the character -", but the effect of a
           * scanset such as [a-zA-Z0-9] is implementation defined.  The V7
           * Unix scanf treats "a-z" as "the letters a through z", but treats
           * "a-a" as "the letter a, the character -, and the letter a". For
           * compatibility, the `-' is not considered to define a range if
           * the character following it is either a close bracket (required
           * by ANSI) or is not numerically greater than the character* we
           * just stored in the table (c).
           */

          n = fmt_char(fmt);
          if (n == ']' || n < c)
            {
              c = '-';
              break;            /* Resume the for (; ; ) */
            }

          fmt++;
          do
            {
              /* Fill in the range */

              c++;
              set[c / 8] |= (1 << (c % 8));     /* Take character c */
            }
          while (c < n);

          /* Alas, the V7 Unix scanf also treats formats such as [a-c-e] as
           * "the letters a through e".  This too is permitted by the
           * standard.
           */

          goto doswitch;

        default:                /* Just another character */
          c = n;
          break;
        }
    }

doexit:

  /* Default => accept */

  if (v)
    {
      for (i = 0; i < 32; i++)  /* Invert all */
        {
          set[i] ^= 0xff;
        }
    }

  return (fmt - 1);
}
#endif

/****************************************************************************
 * Name: vscanf_internal
 *
 * Description:
 *  Stream-oriented implementation that underlies scanf family:  scanf,
 *  fscanf, vfscanf, sscanf, vsscanf and bscanf.
 *
 ****************************************************************************/

static int vscanf_internal(FAR struct lib_instream_s *stream, FAR int *lastc,
                           FAR const IPTR char *fmt, bool varg,
                           union vabuf_u vabuf)
{
  int c;
  FAR char *tv;
  FAR const char *tc;
  int modifier;
  bool noassign;
  bool conv;
  int assigncount;
  int count;
  int ngetstart;
  int width;
  int fwidth;
  int base = 10;
  char tmp[MAXLN];

#ifdef CONFIG_HAVE_LONG_LONG
  FAR unsigned long long *plonglong = NULL;
#endif
  FAR unsigned long *plong = NULL;
  FAR unsigned int *pint = NULL;
  FAR unsigned short *pshort = NULL;
  FAR unsigned char *pchar = NULL;

#ifdef CONFIG_LIBC_SCANSET
  unsigned char set[32];        /* Bit field (256 / 8) */
#endif

  /* Parse the format, extracting values from the input buffer as needed */

  assigncount = 0;
  count       = 0;
  width       = 0;
  conv        = false;
  noassign    = false;
  modifier    = NO_MOD;
  ngetstart   = stream->nget;      /* for %n calculations */

  /* Make sure lastc is not NULL. */

  if (lastc == NULL)
    {
      lastc = &c;
    }

  /* Loop until all characters in the fmt string have been processed.  We may
   * have to continue loop after reaching the end the input data in order to
   * handle trailing %n format specifiers.
   */

  /* Get first character, we keep always the next character in c */

  c = lib_stream_getc(stream);

  while (fmt_char(fmt))
    {
      /* Skip over white spaces */

      if (isspace(fmt_char(fmt)))
        {
          while (isspace(c))
            {
              c = lib_stream_getc(stream);
            }
        }

      while (isspace(fmt_char(fmt)))
        {
          fmt++;
        }

      /* Check for a conversion specifier */

      if (fmt_char(fmt) == '%')
        {
          /* Check for qualifiers on the conversion specifier */

          fmt++;
          for (; fmt_char(fmt); fmt++)
            {
#ifdef CONFIG_LIBC_SCANSET
              if (strchr("diboupxXcseEfFgGaAn[%", fmt_char(fmt)))
#else
              if (strchr("diboupxXcseEfFgGaAn%", fmt_char(fmt)))
#endif
                {
                  if (fmt_char(fmt) != '%')
                    {
                      conv = true;
                    }
                  break;
                }

              if (fmt_char(fmt) == '*')
                {
                  noassign = true;
                }
              else if (fmt_char(fmt) == 'l' || fmt_char(fmt) == 'L')
                {
                  modifier = L_MOD;

                  if (*(fmt + 1) == 'l' || *(fmt + 1) == 'L')
                    {
                      modifier = LL_MOD;
                      fmt++;
                    }
                }
              else if (fmt_char(fmt) == 'z')
                {
                  if (sizeof(size_t) == sizeof(unsigned short))
                    {
                      modifier = H_MOD;
                    }
                  else if (sizeof(size_t) == sizeof(unsigned long))
                    {
                      modifier = L_MOD;
                    }
#if defined(CONFIG_HAVE_LONG_LONG) && ULLONG_MAX != ULONG_MAX
                  else if (sizeof(size_t) == sizeof(unsigned long long))
                    {
                      modifier = LL_MOD;
                    }
#endif
                  else
                    {
                      /* The only known cases that the default will be hit
                       * are (1) the eZ80 which has sizeof(size_t) = 3 which
                       * is the same as the sizeof(int).  And (2) if
                       * CONFIG_HAVE_LONG_LONG
                       * is not enabled and sizeof(size_t) is equal to
                       * sizeof(unsigned long long).  This latter case is an
                       * error.
                       * Treat as integer with no size qualifier.
                       */

                      continue;
                    }
                }
              else if (fmt_char(fmt) == 'j')
                {
                  /* Same as long long if available. Otherwise, long. */
#ifdef CONFIG_HAVE_LONG_LONG
                  modifier = LL_MOD;
#else
                  modifier = L_MOD;
#endif
                }
              else if (fmt_char(fmt) == 'h' || fmt_char(fmt) == 'H')
                {
                  modifier = H_MOD;

                  if (*(fmt + 1) == 'h' || *(fmt + 1) == 'H')
                    {
                      modifier = HH_MOD;
                      fmt++;
                    }
                }
              else if (fmt_char(fmt) >= '1' && fmt_char(fmt) <= '9')
                {
                  for (tc = fmt; isdigit(fmt_char(fmt)); fmt++)
                    ;
                  strlcpy(tmp, tc, fmt - tc + 1);
                  width = atoi(tmp);
                  fmt--;
                }
            }

          /* Process %s: String conversion */

          if (fmt_char(fmt) == 's')
            {
              /* Get a pointer to the char * value.  We need to do this even
               * of we have reached the end of the input data in order to
               * update the 'ap' variable.
               */

              tv = NULL;        /* To avoid warnings about begin uninitialized */
              if (!noassign)
                {
                  tv = next_arg_width(varg, vabuf, FAR char *, width);
                  tv[0] = '\0';
                }

              /* Skip over white space */

              while (isspace(c))
                {
                  c = lib_stream_getc(stream);
                }

              /* But we only perform the data conversion is we still have
               * bytes remaining in the input data stream.
               */

              if (c > 0)
                {
                  /* Use the actual field's width if 1) no fieldwidth
                   * specified or 2) the actual field's width is smaller
                   * than fieldwidth specified.
                   */

                  fwidth = 0;
                  while ((!width || (fwidth < width)) && (c > 0) &&
                         !isspace(c))
                    {
                      if (!noassign)
                        {
                          tv[fwidth] = c;
                        }

                      fwidth++;
                      c = lib_stream_getc(stream);
                    }

                  if (!noassign)
                    {
                      tv[fwidth] = '\0';
                      assigncount++;
                    }

                  count++;
                }
            }

#ifdef CONFIG_LIBC_SCANSET
          /* Process %[: Scanset conversion */

          if (fmt_char(fmt) == '[')
            {
              fmt = findscanset(fmt, set);      /* find scanset */

              /* Get a pointer to the char * value.  We need to do this even
               * if we have reached the end of the input data in order to
               * update the 'ap' variable.
               */

              tv = NULL;        /* To avoid warnings about begin uninitialized */
              if (!noassign)
                {
                  tv = next_arg_width(varg, vabuf, FAR char *, width);
                  tv[0] = '\0';
                }

              /* But we only perform the data conversion is we still have
               * bytes remaining in the input data stream.
               */

              if (c > 0)
                {
                  /* Use the actual field's width if 1) no fieldwidth
                   * specified or 2) the actual field's width is smaller
                   * than the fieldwidth specified.
                   */

                  fwidth = 0;
                  while ((!width || (fwidth < width)) && (c > 0)
                         && ((set[c / 8] & (1 << (c % 8))) != 0))
                    {
                      if (!noassign)
                        {
                          tv[fwidth] = c;
                        }

                      fwidth++;
                      c = lib_stream_getc(stream);
                    }

                  if (!fwidth)
                    {
                      *lastc = c;
                      return assigncount;
                    }

                  if (!noassign)
                    {
                      tv[fwidth] = '\0';
                      assigncount++;
                    }

                  count++;
                }
            }
#endif

          /* Process %c: Character conversion */

          else if (fmt_char(fmt) == 'c')
            {
              /* Get a pointer to the char * value.  We need to do this even
               * if we have reached the end of the input data in order to
               * update the 'ap' variable.
               */

              tv = NULL;        /* To avoid warnings about being uninitialized */
              if (!noassign)
                {
                  tv = next_arg_width(varg, vabuf, FAR char *, width);
                  tv[0] = '\0';
                }

              /* But we only perform the data conversion is we still have
               * bytes remaining in the input data stream.
               */

              if (c > 0)
                {
                  /* Was a field width specified? */

                  if (!width)
                    {
                      /* No, then width is this one single character */

                      width = 1;
                    }

                  /* Copy the character(s) (if we are making an assignment) */

                  fwidth = 0;
                  while ((fwidth < width) && c > 0)
                    {
                      if (!noassign)
                        {
                          tv[fwidth] = c;
                        }

                      fwidth++;
                      c = lib_stream_getc(stream);
                    }

                  if (fwidth != width)
                    {
                      *lastc = c;
                      return assigncount;
                    }

                  if (!noassign)
                    {
                      assigncount++;
                    }

                  count++;
                }
            }

          /* Process %d, %o, %b, %p, %x, %u: Various integer conversions */

          else if (strchr("dobpxXui", fmt_char(fmt)))
            {
              bool sign;

              /* Get a pointer to the integer value.  We need to do this even
               * if we have reached the end of the input data in order to
               * update the 'ap' variable.
               */

              if (!noassign)
                {
                  /* We have to check whether we need to return a long or an
                   * int.
                   */

                  switch (modifier)
                    {
                    case HH_MOD:
                      pchar = next_arg(varg, vabuf, FAR unsigned char *);
                      *pchar = 0;
                      break;

                    case H_MOD:
                      pshort = next_arg(varg, vabuf, FAR unsigned short *);
                      *pshort = 0;
                      break;

                    case NO_MOD:
                      pint = next_arg(varg, vabuf, FAR unsigned int *);
                      *pint = 0;
                      break;

                    default:
                    case L_MOD:
                      plong = next_arg(varg, vabuf, FAR unsigned long *);
                      *plong = 0;
                      break;

#ifdef CONFIG_HAVE_LONG_LONG
                    case LL_MOD:
                      plonglong = next_arg(varg, vabuf,
                                           FAR unsigned long long *);
                      *plonglong = 0;
                      break;
#endif
                    }
                }

              /* Skip over any white space before the integer string */

              while (isspace(c))
                {
                  c = lib_stream_getc(stream);
                }

              /* But we only perform the data conversion if we still have
               * bytes remaining in the input data stream.
               */

              if (c > 0)
                {
                  FAR char *endptr;
                  int prefix;
                  bool stopconv;
                  int errsave;
                  unsigned long tmplong = 0;
#ifdef CONFIG_HAVE_LONG_LONG
                  unsigned long long tmplonglong = 0;
#endif
                  /* Copy the real string into a temporary working buffer. */

                  if (!width || width > sizeof(tmp) - 1)
                    {
                      width = sizeof(tmp) - 1;
                    }

                  /* The base of the integer conversion depends on the
                   * specific conversion specification.
                   */

                  fwidth   = 0;
                  prefix   = 0;
                  stopconv = false;
                  sign     = false;

                  switch (fmt_char(fmt))
                    {
                    default:
                    case 'd':
                      sign = true;

                      /* FALLTHROUGH */

                    case 'u':
                      while (fwidth < width && !stopconv)
                        {
                          if (c == '-' || c == '+')
                            {
                              if (fwidth != 0)
                                {
                                  stopconv = true;
                                }
                            }
                          else if (!(c >= '0' && c <= '9'))
                            {
                              stopconv = true;
                            }

                          if (!stopconv)
                            {
                              tmp[fwidth++] = c;
                              c = lib_stream_getc(stream);
                            }
                        }

                      base = 10;
                      break;

                    case 'p':
                    case 'x':
                    case 'X':
                      while (fwidth < width && !stopconv)
                        {
                          if (c == '-' || c == '+')
                            {
                              if (fwidth != 0)
                                {
                                  stopconv = true;
                                }
                            }
                          else if (c == '0')
                            {
                              if (prefix == 0)
                                {
                                  prefix = 1;
                                }
                            }
                          else if (c == 'x' || c == 'X')
                            {
                              if (prefix == 1)
                                {
                                  prefix = 2;
                                }
                              else
                                {
                                  stopconv = true;
                                }
                            }
                          else if (!((c >= '0' && c <= '9') ||
                                     (c >= 'a' && c <= 'f') ||
                                     (c >= 'A' && c <= 'F')))
                            {
                              stopconv = true;
                            }
                          else
                            {
                              prefix = 2;
                            }

                          if (!stopconv)
                            {
                              tmp[fwidth++] = c;
                              c = lib_stream_getc(stream);
                            }
                        }

                      base = 16;
                      break;

                    case 'o':
                      while (fwidth < width && !stopconv)
                        {
                          if (c == '-' || c == '+')
                            {
                              if (fwidth != 0)
                                {
                                  stopconv = true;
                                }
                            }
                          else if (!(c >= '0' && c <= '7'))
                            {
                              stopconv = true;
                            }

                          if (!stopconv)
                            {
                              tmp[fwidth++] = c;
                              c = lib_stream_getc(stream);
                            }
                        }

                      base = 8;
                      break;

                    case 'b':  /* not official? */
                      while (fwidth < width && !stopconv)
                        {
                          if (c == '-' || c == '+')
                            {
                              if (fwidth != 0)
                                {
                                  stopconv = true;
                                }
                            }
                          else if (!(c >= '0' && c <= '1'))
                            {
                              stopconv = true;
                            }

                          if (!stopconv)
                            {
                              tmp[fwidth++] = c;
                              c = lib_stream_getc(stream);
                            }
                        }

                      base = 2;
                      break;

                    case 'i':
                      sign = true;
                      base = 10;

                      while (fwidth < width && !stopconv)
                        {
                          if (c == '-' || c == '+')
                            {
                              if (fwidth != 0)
                                {
                                  stopconv = true;
                                }
                            }
                          else if (c == '0')
                            {
                              if (prefix == 0)
                                {
                                  prefix = 1;
                                  base = 8;
                                }
                            }
                          else if (c == 'x' || c == 'X')
                            {
                              if (prefix == 1)
                                {
                                  prefix = 2;
                                  base = 16;
                                }
                              else
                                {
                                  stopconv = true;
                                }
                            }
                          else if (!((c >= '0' && c <= '7' && base >= 8) ||
                                     (c >= '8' && c <= '9' && base >= 10) ||
                                     (c >= 'a' && c <= 'f' && base == 16) ||
                                     (c >= 'A' && c <= 'F' && base == 16)))
                            {
                              stopconv = true;
                            }
                          else
                            {
                              prefix = 2;
                            }

                          if (!stopconv)
                            {
                              tmp[fwidth++] = c;
                              c = lib_stream_getc(stream);
                            }
                        }
                      break;
                    }

                  tmp[fwidth] = 0;

                  /* Perform the integer conversion */

                  /* Preserve the errno value */

                  errsave = get_errno();
                  set_errno(0);

                  switch (modifier)
                    {
#ifndef CONFIG_HAVE_LONG_LONG
                    case LL_MOD:
#endif
                    case HH_MOD:
                    case H_MOD:
                    case NO_MOD:
                    default:
                      if (sign)
                        {
                          tmplong = strtol(tmp, &endptr, base);
                        }
                      else
                        {
                          tmplong = strtoul(tmp, &endptr, base);
                        }
                      break;

#ifdef CONFIG_HAVE_LONG_LONG
                    case LL_MOD:
                      if (sign)
                        {
#  ifdef CONFIG_LIBC_LONG_LONG
                          tmplonglong = strtoll(tmp, &endptr, base);
#  else
                          tmplonglong = strtol(tmp, &endptr, base);
#  endif
                        }
                      else
                        {
#  ifdef CONFIG_LIBC_LONG_LONG
                          tmplonglong = strtoull(tmp, &endptr, base);
#  else
                          tmplonglong = strtoul(tmp, &endptr, base);
#  endif
                        }
                      break;
#endif
                    }

                  /* Check if the number was successfully converted */

                  if (tmp == endptr || get_errno() == ERANGE)
                    {
                      *lastc = c;
                      return assigncount;
                    }

                  set_errno(errsave);
                  if (!noassign)
                    {
                      /* We have to check whether we need to return a long or
                       * an int.
                       */

                      switch (modifier)
                        {
                        case HH_MOD:
                          *pchar = (unsigned char)tmplong;
                          break;

                        case H_MOD:
                          *pshort = (unsigned short)tmplong;
                          break;

                        case NO_MOD:
                          *pint = (unsigned int)tmplong;
                          break;

#ifndef CONFIG_HAVE_LONG_LONG
                        case L_MOD:
#endif
                        default:
                          *plong = tmplong;
                          break;

#ifdef CONFIG_HAVE_LONG_LONG
                        case LL_MOD:
                          *plonglong = tmplonglong;
                          break;
#endif
                        }

                      assigncount++;
                    }

                  count++;
                }
            }

          /* Process %a, %A, %f, %F, %e, %E, %g, and %G: Floating point
           * conversions.
           */

          else if (strchr("aAfFeEgG", fmt_char(fmt)) != NULL)
            {
#ifdef CONFIG_HAVE_DOUBLE
              FAR double *pd = NULL;
#endif

#ifdef CONFIG_HAVE_FLOAT
              FAR float *pf = NULL;
#endif

              /* Get a pointer to the double value.  We need to do this even
               * if we have reached the end of the input data in order to
               * upate the 'ap' variable.
               */

              if (!noassign)
                {
                  /* We have to check whether we need to return a float or a
                   * double.
                   */

#ifdef CONFIG_HAVE_DOUBLE
                  if (modifier >= L_MOD)
                    {
                      pd = next_arg(varg, vabuf, FAR double *);
                      *pd = 0.0;
                    }
                  else
#endif
#ifdef CONFIG_HAVE_FLOAT
                    {
                      pf = next_arg(varg, vabuf, FAR float *);
                      *pf = 0.0;
                    }
#endif
                }

#ifdef CONFIG_LIBC_FLOATINGPOINT

              /* Skip over any white space before the real string */

              while (isspace(c))
                {
                  c = lib_stream_getc(stream);
                }

              /* But we only perform the data conversion is we still have
               * bytes remaining in the input data stream.
               */

              if (c > 0)
                {
#  if defined(CONFIG_HAVE_DOUBLE) || defined(CONFIG_HAVE_FLOAT)
                  FAR char *endptr;
#  endif
                  bool expnt;
                  bool dot;
                  bool sign;
                  bool stopconv;
                  int errsave;
#  ifdef CONFIG_HAVE_DOUBLE
                  double dvalue;
#  endif
#  ifdef CONFIG_HAVE_FLOAT
                  float fvalue;
#  endif
                  /* Was a fieldwidth specified? */

                  if (!width || width > sizeof(tmp) - 1)
                    {
                      width = sizeof(tmp) - 1;
                    }

                  /* Copy the real string into a temporary working buffer. */

                  fwidth   = 0;
                  expnt    = false;
                  sign     = false;
                  dot      = false;
                  stopconv = false;

                  while (fwidth < width && !stopconv)
                    {
                      if (c == '-' || c == '+')
                        {
                          if (!sign)
                            {
                              sign = true;
                            }
                          else
                            {
                              stopconv = true;
                            }
                        }
                      else if (c == '.')
                        {
                          if (!dot)
                            {
                              dot = true;
                              sign = true;
                            }
                          else
                            {
                              stopconv = true;
                            }
                        }
                      else if (c == 'e' || c == 'E')
                        {
                          if (!expnt)
                            {
                              expnt = true;
                              sign = false;
                            }
                          else
                            {
                              stopconv = true;
                            }
                        }
                      else if (!(c >= '0' && c <= '9'))
                        {
                          stopconv = true;
                        }
                      else
                        {
                          sign = true;
                        }

                      if (!stopconv)
                        {
                          tmp[fwidth++] = c;
                          c = lib_stream_getc(stream);
                        }
                    }

                  tmp[fwidth] = 0;

                  /* Perform the floating point conversion */

                  /* Preserve the errno value */

                  errsave = get_errno();
                  set_errno(0);

#  ifdef CONFIG_HAVE_DOUBLE
                  if (modifier >= L_MOD)
                    {
                      /* Get the converted double value */

                      dvalue = strtod(tmp, &endptr);
                    }
                  else
#  endif
#  ifdef CONFIG_HAVE_FLOAT
                    {
                      fvalue = strtof(tmp, &endptr);
                    }
#  endif

                  /* Check if the number was successfully converted */

#  if defined(CONFIG_HAVE_DOUBLE) || defined(CONFIG_HAVE_FLOAT)
                  if (tmp == endptr || get_errno() == ERANGE)
                    {
                      *lastc = c;
                      return assigncount;
                    }
#endif

                  set_errno(errsave);

                  if (!noassign)
                    {
                      /* We have to check whether we need to return a float
                       * or a double.
                       */

#  ifdef CONFIG_HAVE_DOUBLE
                      if (modifier >= L_MOD)
                        {
                          /* Return the double value */

                          *pd = dvalue;
                        }
                      else
#  endif
                        {
                          /* Return the float value */

#  ifdef CONFIG_HAVE_FLOAT
                          *pf = fvalue;
#  endif
                        }

                      assigncount++;
                    }

                  count++;
                }
#endif
            }

          /* Process %n: Character count */

          else if (fmt_char(fmt) == 'n')
            {
              if (!noassign)
                {
                  size_t nchars = (size_t) (stream->nget - ngetstart);

                  if (!lib_stream_eof(c))
                    {
                      /* One more character already read */

                      nchars--;
                    }

                  /* Note %n does not count as a conversion */

                  switch (modifier)
                    {
                    case HH_MOD:
                      pchar = next_arg(varg, vabuf, FAR unsigned char *);
                      *pchar = (unsigned char)nchars;
                      break;

                    case H_MOD:
                      pshort = next_arg(varg, vabuf, FAR unsigned short *);
                      *pshort = (unsigned short)nchars;
                      break;

                    case NO_MOD:
                      pint = next_arg(varg, vabuf, FAR unsigned int *);
                      *pint = (unsigned int)nchars;
                      break;

                    default:
                    case L_MOD:
                      plong = next_arg(varg, vabuf, FAR unsigned long *);
                      *plong = (unsigned long)nchars;
                      break;

#ifdef CONFIG_HAVE_LONG_LONG
                    case LL_MOD:
                      plonglong = next_arg(varg, vabuf,
                                           FAR unsigned long long *);
                      *plonglong = (unsigned long long)nchars;
                      break;
#endif
                    }
                }

              count++;
            }
          else if (fmt_char(fmt) == '%')
            {
              if (c != '%')
                {
                  break;
                }
              else
                {
                  c = lib_stream_getc(stream);
                }
            }

          width    = 0;
          noassign = false;
          modifier = NO_MOD;

          fmt++;
        }

      /* It is not a conversion specifier */

      else if (c > 0)
        {
#if 0
          /* Skip over any leading spaces in the input buffer */

          while (isspace(c))
            {
              c = lib_stream_getc(stream);
            }
#endif

          /* Skip over matching characters in the buffer and format */

          if (fmt_char(fmt) != c)
            {
              break;
            }
          else
            {
              fmt++;
              c = lib_stream_getc(stream);
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

  *lastc = c;
  return (count || !conv) ? assigncount : EOF;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lib_vscanf
 *
 * Description:
 *  Stream-oriented implementation that underlies scanf family:  scanf,
 *  fscanf, vfscanf, sscanf, and vsscanf
 *
 ****************************************************************************/

int lib_vscanf(FAR struct lib_instream_s *stream, FAR int *lastc,
               FAR const IPTR char *fmt, va_list ap)
{
  union vabuf_u vabuf;
  int ret;

  va_copy(vabuf.ap, ap);
  ret = vscanf_internal(stream, lastc, fmt, true, vabuf);
  va_end(vabuf.ap);

  return ret;
}

/****************************************************************************
 * Name: lib_bscanf
 *
 * Description:
 *  Convert data into a structure according to standard formatting protocols.
 *  For string arrays, please use "%{length}s" or "%{length}c" to specify
 *  the length.
 *
 ****************************************************************************/

int lib_bscanf(FAR struct lib_instream_s *stream, FAR int *lastc,
               FAR const IPTR char *fmt, FAR void *data)
{
  union vabuf_u vabuf =
    {
      data
    };

  return vscanf_internal(stream, lastc, fmt, false, vabuf);
}
