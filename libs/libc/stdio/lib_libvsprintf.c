/****************************************************************************
 * libs/libc/stdio/lib_libvsprintf.c
 *
 *   Copyright (c) 2002, Alexander Popov (sasho@vip.bg)
 *   Copyright (c) 2002,2004,2005 Joerg Wunsch
 *   Copyright (c) 2005, Helmut Wallner
 *   Copyright (c) 2007, Dmitry Xmelkov
 *   All rights reserved.
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

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>

#include <nuttx/compiler.h>
#include <nuttx/streams.h>
#include <nuttx/allsyms.h>

#include "lib_dtoa_engine.h"
#include "lib_ultoa_invert.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CONFIG_LIBC_LONG_LONG is not a valid selection of the compiler does not
 * support long long types.
 */

#ifndef CONFIG_HAVE_LONG_LONG
#  undef CONFIG_LIBC_LONG_LONG
#endif

/* Order is relevant here and matches order in format string */

#define FL_ZFILL           0x0001
#define FL_PLUS            0x0002
#define FL_SPACE           0x0004
#define FL_LPAD            0x0008
#define FL_ALT             0x0010

#define FL_ARGNUMBER       0x0020
#define FL_ASTERISK        0x0040

#define FL_WIDTH           0x0080
#define FL_PREC            0x0100

#define FL_LONG            0x0200
#define FL_SHORT           0x0400
#define FL_REPD_TYPE       0x0800

#define FL_NEGATIVE        0x1000

/* The next 2 groups are Exclusive Or */

#define FL_ALTUPP          0x2000
#define FL_ALTHEX          0x4000

#define FL_FLTUPP          0x2000
#define FL_FLTEXP          0x4000
#define FL_FLTFIX          0x8000

#define TYPE_INT           1
#define TYPE_LONG          2
#define TYPE_LONG_LONG     3
#define TYPE_DOUBLE        4
#define TYPE_CHAR_POINTER  5

/* Support special access to CODE-space strings for Harvard architectures */

#ifdef CONFIG_ARCH_ROMGETC
#  define fmt_char(fmt)   up_romgetc((fmt)++)
#else
#  define fmt_char(fmt)   (*(fmt)++)
#endif

#define fmt_ungetc(fmt)   ((fmt)--)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct arg_s
{
  unsigned char type;
  union
  {
    unsigned int u;
    unsigned long ul;
#ifdef CONFIG_HAVE_LONG_LONG
    unsigned long long ull;
#endif
    double d;
    FAR char *cp;
  } value;
};

/****************************************************************************
 * Private Constant Data
 ****************************************************************************/

static const char g_nullstring[] = "(null)";

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int vsprintf_internal(FAR struct lib_outstream_s *stream,
                             FAR struct arg_s *arglist, int numargs,
                             FAR const IPTR char *fmt, va_list ap)
           printflike(4, 0);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_ALLSYMS
static int sprintf_internal(FAR struct lib_outstream_s *stream,
                            FAR const IPTR char *fmt, ...)
{
  va_list ap;
  int     n;

  /* Then let vsprintf_internal do the real work */

  va_start(ap, fmt);
  n = vsprintf_internal(stream, NULL, 0, fmt, ap);
  va_end(ap);

  return n;
}
#endif

static int vsprintf_internal(FAR struct lib_outstream_s *stream,
                             FAR struct arg_s *arglist, int numargs,
                             FAR const IPTR char *fmt, va_list ap)
{
  unsigned char c; /* Holds a char from the format string */
  uint16_t flags;
  int width;
  int prec;
  union
  {
#if defined (CONFIG_LIBC_LONG_LONG) || (ULONG_MAX > 4294967295UL)
    unsigned char __buf[22]; /* Size for -1 in octal, without '\0' */
#else
    unsigned char __buf[11]; /* Size for -1 in octal, without '\0' */
#endif
#ifdef CONFIG_LIBC_FLOATINGPOINT
    struct dtoa_s __dtoa;
#endif
  } u;

#define buf     (u.__buf)
#define _dtoa   (u.__dtoa)

  FAR const char *pnt;
  size_t size;
  unsigned char len;

#ifdef CONFIG_LIBC_NUMBERED_ARGS
  int total_len = 0;
  int argnumber = 0;
#endif

  for (; ; )
    {
      for (; ; )
        {
          c = fmt_char(fmt);
          if (c == '\0')
            {
              goto ret;
            }

          if (c == '%')
            {
              c = fmt_char(fmt);
              if (c != '%')
                {
                  break;
                }
            }

#ifdef CONFIG_LIBC_NUMBERED_ARGS
          if (stream != NULL)
            {
              lib_stream_put(stream, c);
            }
#else
          lib_stream_put(stream, c);
#endif
        }

      flags = 0;
      width = 0;
      prec  = 0;

      do
        {
          if (flags < FL_ASTERISK)
            {
              switch (c)
                {
                case '0':
                  flags |= FL_ZFILL;
                  continue;

                case '+':
                  flags |= FL_PLUS;

                  /* FALLTHROUGH */

                case ' ':
                  flags |= FL_SPACE;
                  continue;

                case '-':
                  flags |= FL_LPAD;
                  continue;

                case '#':
                  flags |= FL_ALT;
                  continue;
                }
            }

          if (flags < FL_LONG)
            {
#ifdef CONFIG_LIBC_NUMBERED_ARGS
              if (c == '$')
                {
                  if ((flags & FL_ARGNUMBER) == 0)
                    {
                      /* No other flag except FL_WIDTH or FL_ZFILL (leading
                       * zeros) and argument number must be at least 1
                       */

                      if ((flags & ~(FL_WIDTH | FL_ZFILL)) != 0 ||
                          width == 0)
                        {
                          goto ret;
                        }

                      /* It had been the argument number. */

                      argnumber = width;
                      width     = 0;
                      flags     = FL_ARGNUMBER;
                    }
                  else if ((flags & FL_ASTERISK) != 0)
                    {
                      int index;

                      flags &= ~FL_ASTERISK;
                      if ((flags & FL_PREC) == 0)
                        {
                          index = width;
                        }
                      else
                        {
                          index = prec;
                        }

                      if (index > 0 && index <= numargs)
                        {
                          if (stream == NULL)
                            {
                              arglist[index - 1].type = TYPE_INT;
                              if (index > total_len)
                                {
                                  total_len = index;
                                }
                            }
                          else
                            {
                              if ((flags & FL_PREC) == 0)
                                {
                                  width = (int)arglist[index - 1].value.u;
                                }
                              else
                                {
                                  prec = (int)arglist[index - 1].value.u;
                                }
                            }
                        }
                      else
                        {
                          goto ret;
                        }
                    }
                  else
                    {
                      goto ret;
                    }

                  continue;
                }
#endif

              if (c >= '0' && c <= '9')
                {
                  c -= '0';
                  if ((flags & FL_PREC) != 0)
                    {
                      prec = 10 * prec + c;
                      continue;
                    }

                  width = 10 * width + c;
                  flags |= FL_WIDTH;
                  continue;
                }

              if (c == '*')
                {
#ifdef CONFIG_LIBC_NUMBERED_ARGS
                  if ((flags & FL_ARGNUMBER) != 0)
                    {
                      flags |= FL_ASTERISK;
                      continue;
                    }
                  else if (stream == NULL)
                    {
                      continue; /* We do only parsing */
                    }
#endif

                  if ((flags & FL_PREC) != 0)
                    {
                      prec = va_arg(ap, int);
                      if (prec < 0)
                        {
                          prec = 0;
                        }
                    }
                  else
                    {
                      width = va_arg(ap, int);
                      flags |= FL_WIDTH;

                      if (width < 0)
                        {
                          width = -width;
                          flags |= FL_LPAD;
                        }
                    }

                  continue;
                }

              if (c == '.')
                {
                  if ((flags & FL_PREC) != 0)
                    {
                      goto ret;
                    }

                  flags |= FL_PREC;
                  continue;
                }
            }

          /* Note: On NuttX, ptrdiff_t == intptr_t == ssize_t. */

          if (c == 'z' || c == 't')
            {
              switch (sizeof(size_t))
                {
                  /* The only known cases that the default will be hit are
                   * (1) the eZ80 which has sizeof(size_t) = 3 which is the
                   * same as the sizeof(int).  And (2) if
                   * CONFIG_HAVE_LONG_LONG
                   * is not enabled and sizeof(size_t) is equal to
                   * sizeof(unsigned long long).  This latter case is an
                   * error.
                   */

                  default:
                    continue;  /* Treat as integer with no size qualifier. */

                  case sizeof(unsigned short):
                    c = 'h';
                    break;

                  case sizeof(unsigned long):
                    c = 'l';
                    break;

#if defined(CONFIG_HAVE_LONG_LONG) && ULLONG_MAX != ULONG_MAX
                  case sizeof(unsigned long long):
                    c = 'l';
                    flags |= FL_LONG;
                    flags &= ~FL_SHORT;
                    break;
#endif
                }
            }

          if (c == 'j')
            {
              /* Same as long long if available. Otherwise, long. */

#ifdef CONFIG_HAVE_LONG_LONG
              flags |= FL_REPD_TYPE;
#endif
              flags |= FL_LONG;
              flags &= ~FL_SHORT;
              continue;
            }

          if (c == 'l')
            {
              if ((flags & FL_LONG) != 0)
                {
                  flags |= FL_REPD_TYPE;
                }
              else
                {
                  flags |= FL_LONG;
                }

              flags &= ~FL_SHORT;
              continue;
            }

          if (c == 'h')
            {
              if ((flags & FL_SHORT) != 0)
                {
                  flags |= FL_REPD_TYPE;
                }
              else
                {
                  flags |= FL_SHORT;
                }

              flags &= ~FL_LONG;
              continue;
            }

          break;
        }
      while ((c = fmt_char(fmt)) != 0);

      /* Only a format character is valid.  */

#if 'F' != 'E'+1  ||  'G' != 'F'+1  ||  'f' != 'e'+1  ||  'g' != 'f'+1
#  error
#endif

      if (c == 'p')
        {
          /* Determine size of pointer and set flags accordingly */

          flags &= ~(FL_LONG | FL_REPD_TYPE);

#ifdef CONFIG_HAVE_LONG_LONG
          if (sizeof(void *) == sizeof(unsigned long long))
            {
              flags |= (FL_LONG | FL_REPD_TYPE);
            }
          else
#endif
          if (sizeof(void *) == sizeof(unsigned long))
            {
              flags |= FL_LONG;
            }
        }

#ifdef CONFIG_LIBC_NUMBERED_ARGS
      if ((flags & FL_ARGNUMBER) != 0)
        {
          if (argnumber > 0 && argnumber <= numargs)
            {
              if (stream == NULL)
                {
                  if ((c >= 'E' && c <= 'G')
                      || (c >= 'e' && c <= 'g'))
                    {
                      arglist[argnumber - 1].type = TYPE_DOUBLE;
                    }
                  else if (c == 'i' || c == 'd' || c == 'u' || c == 'p')
                    {
                      if ((flags & FL_LONG) == 0)
                        {
                          arglist[argnumber - 1].type = TYPE_INT;
                        }
                      else if ((flags & FL_REPD_TYPE) == 0)
                        {
                          arglist[argnumber - 1].type = TYPE_LONG;
                        }
                      else
                        {
                          arglist[argnumber - 1].type = TYPE_LONG_LONG;
                        }
                    }
                  else if (c == 'c')
                    {
                      arglist[argnumber - 1].type = TYPE_INT;
                    }
                  else if (c == 's')
                    {
                      arglist[argnumber - 1].type = TYPE_CHAR_POINTER;
                    }

                  if (argnumber > total_len)
                    {
                      total_len = argnumber;
                    }

                  continue; /* We do only parsing */
                }
            }
          else
            {
              goto ret;
            }
        }
      else if (stream == NULL)
        {
          continue; /* We do only parsing */
        }
#endif

#ifdef CONFIG_LIBC_FLOATINGPOINT
      if (c >= 'E' && c <= 'G')
        {
          flags |= FL_FLTUPP;
          c += 'e' - 'E';
          goto flt_oper;
        }
      else if (c >= 'e' && c <= 'g')
        {
          double value;
          int exp;              /* Exponent of master decimal digit */
          int n;
          uint8_t sign;         /* Sign character (or 0) */
          uint8_t ndigs;        /* Number of digits to convert */
          uint8_t ndecimal;     /* Digits after decimal (for 'f' format), 0 if
                                 * no limit */

          flags &= ~FL_FLTUPP;

flt_oper:
          ndigs = 0;
          if ((flags & FL_PREC) == 0)
            {
              prec = 6;
            }

          flags &= ~(FL_FLTEXP | FL_FLTFIX);

          if (c == 'e')
            {
              ndigs = prec + 1;
              ndecimal = 0;
              flags |= FL_FLTEXP;
            }
          else if (c == 'f')
            {
              ndigs = DTOA_MAX_DIG;
              ndecimal = prec;
              flags |= FL_FLTFIX;
            }
          else
            {
              ndigs = prec;
              ndecimal = 0;
            }

          if (ndigs > DTOA_MAX_DIG)
            {
              ndigs = DTOA_MAX_DIG;
            }

#ifdef CONFIG_LIBC_NUMBERED_ARGS
          if ((flags & FL_ARGNUMBER) != 0)
            {
              value = arglist[argnumber - 1].value.d;
            }
          else
            {
              value = va_arg(ap, double);
            }
#else
          value = va_arg(ap, double);
#endif

          ndigs = __dtoa_engine(value, &_dtoa, ndigs,
                                ndecimal);
          exp = _dtoa.exp;

          sign = 0;
          if ((_dtoa.flags & DTOA_MINUS) && !(_dtoa.flags & DTOA_NAN))
            {
              sign = '-';
            }
          else if ((flags & FL_PLUS) != 0)
            {
              sign = '+';
            }
          else if ((flags & FL_SPACE) != 0)
            {
              sign = ' ';
            }

          if (_dtoa.flags & (DTOA_NAN | DTOA_INF))
            {
              FAR const char *p;

              ndigs = sign ? 4 : 3;
              if (width > ndigs)
                {
                  width -= ndigs;
                  if ((flags & FL_LPAD) == 0)
                    {
                      do
                        {
                          lib_stream_put(stream, ' ');
                        }
                      while (--width);
                    }
                }
              else
                {
                  width = 0;
                }

              if (sign)
                {
                  lib_stream_put(stream, sign);
                }

              p = "inf";
              if (_dtoa.flags & DTOA_NAN)
                {
                  p = "nan";
                }

#  if ('I'-'i' != 'N'-'n') || ('I'-'i' != 'F'-'f') || ('I'-'i' != 'A'-'a')
#    error
#  endif
              while ((ndigs = *p) != 0)
                {
                  if ((flags & FL_FLTUPP) != 0)
                    {
                      ndigs += 'I' - 'i';
                    }

                  lib_stream_put(stream, ndigs);
                  p++;
                }

              goto tail;
            }

          if ((flags & (FL_FLTEXP | FL_FLTFIX)) == 0)
            {
              /* 'g(G)' format */

              prec = ndigs;

              /* Remove trailing zeros */

              while (ndigs > 0 && _dtoa.digits[ndigs - 1] == '0')
                {
                  ndigs--;
                }

              if (-4 <= exp && exp < prec)
                {
                  flags |= FL_FLTFIX;

                  if (exp < 0 || ndigs > exp)
                    {
                      prec = ndigs - (exp + 1);
                    }
                  else
                    {
                      prec = 0;
                    }
                }
              else
                {
                  /* Limit displayed precision to available precision */

                  prec = ndigs - 1;
                }
            }

          /* Conversion result length, width := free space length */

          if ((flags & FL_FLTFIX) != 0)
            {
              n = (exp > 0 ? exp + 1 : 1);
            }
          else
            {
              n = 5;              /* 1e+00 */
            }

          if (sign != 0)
            {
              n += 1;
            }

          if (prec != 0)
            {
              n += prec + 1;
            }
          else if ((flags & FL_ALT) != 0)
            {
              n += 1;
            }

          width = width > n ? width - n : 0;

          /* Output before first digit */

          if ((flags & (FL_LPAD | FL_ZFILL)) == 0)
            {
              while (width)
                {
                  lib_stream_put(stream, ' ');
                  width--;
                }
            }

          if (sign != 0)
            {
              lib_stream_put(stream, sign);
            }

          if ((flags & FL_LPAD) == 0)
            {
              while (width)
                {
                  lib_stream_put(stream, '0');
                  width--;
                }
            }

          if ((flags & FL_FLTFIX) != 0)
            {
              /* 'f' format */

              char out;

              /* At this point, we should have exp exponent of leftmost digit
               * in _dtoa.digits ndigs number of buffer digits to print prec
               * number of digits after decimal In the loop, 'n' walks over
               * the exponent value
               */

              n = exp > 0 ? exp : 0;    /* Exponent of left digit */
              do
                {
                  /* Insert decimal point at correct place */

                  if (n == -1)
                    {
                      lib_stream_put(stream, '.');
                    }

                  /* Pull digits from buffer when in-range, otherwise use 0 */

                  if (0 <= exp - n && exp - n < ndigs)
                    {
                      out = _dtoa.digits[exp - n];
                    }
                  else
                    {
                      out = '0';
                    }

                  if (--n < -prec)
                    {
                      if ((flags & FL_ALT) != 0 && n == -1)
                        {
                          lib_stream_put(stream, '.');
                        }

                      break;
                    }

                  lib_stream_put(stream, out);
                }
              while (1);

              if (n == exp && (_dtoa.digits[0] > '5' ||
                  (_dtoa.digits[0] == '5' && !(_dtoa.flags & DTOA_CARRY))))
                {
                  out = '1';
                }

              lib_stream_put(stream, out);
            }
          else
            {
              /* 'e(E)' format
               *
               * Mantissa
               */

              if (_dtoa.digits[0] != '1')
                {
                  _dtoa.flags &= ~DTOA_CARRY;
                }

              lib_stream_put(stream, _dtoa.digits[0]);
              if (prec > 0)
                {
                  uint8_t pos;

                  lib_stream_put(stream, '.');
                  for (pos = 1; pos < 1 + prec; pos++)
                    {
                      lib_stream_put(stream, pos < ndigs ?
                                     _dtoa.digits[pos] : '0');
                    }
                }
              else if ((flags & FL_ALT) != 0)
                {
                  lib_stream_put(stream, '.');
                }

              /* Exponent */

              lib_stream_put(stream, flags & FL_FLTUPP ? 'E' : 'e');
              ndigs = '+';
              if (exp < 0 || (exp == 0 && (_dtoa.flags & DTOA_CARRY) != 0))
                {
                  exp = -exp;
                  ndigs = '-';
                }

              lib_stream_put(stream, ndigs);
              c = __ultoa_invert(exp, (FAR char *)buf, 10) - (FAR char *)buf;
              while (c > 0)
                {
                  lib_stream_put(stream, buf[c - 1]);
                  c--;
                }
            }

          goto tail;
        }

#else /* !CONFIG_LIBC_FLOATINGPOINT */
      if ((c >= 'E' && c <= 'G') || (c >= 'e' && c <= 'g'))
        {
#  ifndef CONFIG_LIBC_NUMBERED_ARGS
          va_arg(ap, double);
#  endif
          pnt  = "*float*";
          size = sizeof("*float*") - 1;
          goto str_lpad;
        }
#endif

      switch (c)
        {
        case 'c':
#ifdef CONFIG_LIBC_NUMBERED_ARGS
          if ((flags & FL_ARGNUMBER) != 0)
            {
              buf[0] = (int)arglist[argnumber - 1].value.u;
            }
          else
            {
              buf[0] = va_arg(ap, int);
            }
#else
          buf[0] = va_arg(ap, int);
#endif
          pnt = (FAR char *)buf;
          size = 1;
          goto str_lpad;

        case 's':
        case 'S':
#ifdef CONFIG_LIBC_NUMBERED_ARGS
          if ((flags & FL_ARGNUMBER) != 0)
            {
              pnt = (FAR char *)arglist[argnumber - 1].value.cp;
            }
          else
            {
              pnt = va_arg(ap, FAR char *);
            }
#else
          pnt = va_arg(ap, FAR char *);
#endif
          if (pnt == NULL)
            {
              pnt = g_nullstring;
            }

          size = strnlen(pnt, (flags & FL_PREC) ? prec : ~0);

str_lpad:
          if ((flags & FL_LPAD) == 0)
            {
              while (size < width)
                {
                  lib_stream_put(stream, ' ');
                  width--;
                }
            }

          while (size)
            {
              lib_stream_put(stream, *pnt++);
              if (width != 0)
                {
                  width -= 1;
                }

              size -= 1;
            }

          goto tail;
        }

      if (c == 'd' || c == 'i')
        {
#ifndef CONFIG_HAVE_LONG_LONG
          long x;
#else
          long long x;

          if ((flags & FL_LONG) != 0 && (flags & FL_REPD_TYPE) != 0)
            {
#ifdef CONFIG_LIBC_NUMBERED_ARGS
              if ((flags & FL_ARGNUMBER) != 0)
                {
                  x = (long long)arglist[argnumber - 1].value.ull;
                }
              else
                {
                  x = va_arg(ap, long long);
                }
#else
                x = va_arg(ap, long long);
#endif
            }
          else
#endif
          if ((flags & FL_LONG) != 0)
            {
#ifdef CONFIG_LIBC_NUMBERED_ARGS
              if ((flags & FL_ARGNUMBER) != 0)
                {
                  x = (long)arglist[argnumber - 1].value.ul;
                }
              else
                {
                  x = va_arg(ap, long);
                }
#else
                x = va_arg(ap, long);
#endif
            }
          else
            {
#ifdef CONFIG_LIBC_NUMBERED_ARGS
              if ((flags & FL_ARGNUMBER) != 0)
                {
                  x = (int)arglist[argnumber - 1].value.u;
                }
              else
                {
                  x = va_arg(ap, int);
                }
#else
                x = va_arg(ap, int);
#endif
              if ((flags & FL_SHORT) != 0)
                {
                  if ((flags & FL_REPD_TYPE) == 0)
                    {
                      x = (short)x;
                    }
                  else
                    {
                      x = (signed char)x;
                    }
                }
            }

          flags &= ~(FL_NEGATIVE | FL_ALT);
          if (x < 0)
            {
              x = -x;
              flags |= FL_NEGATIVE;
            }

          if ((flags & FL_PREC) != 0 && prec == 0 && x == 0)
            {
              c = 0;
            }
          else
            {
#if !defined(CONFIG_LIBC_LONG_LONG) && defined(CONFIG_HAVE_LONG_LONG)
              DEBUGASSERT(x >= 0 && x <= ULONG_MAX);
#endif
              c = __ultoa_invert(x, (FAR char *)buf, 10) - (FAR char *)buf;
            }
        }
      else
        {
          int base;
#ifndef CONFIG_HAVE_LONG_LONG
          unsigned long x;
#else
          unsigned long long x;

          if ((flags & FL_LONG) != 0 && (flags & FL_REPD_TYPE) != 0)
            {
#ifdef CONFIG_LIBC_NUMBERED_ARGS
              if ((flags & FL_ARGNUMBER) != 0)
                {
                  x = arglist[argnumber - 1].value.ull;
                }
              else
                {
                  x = va_arg(ap, unsigned long long);
                }
#else
                x = va_arg(ap, unsigned long long);
#endif
            }
          else
#endif
          if ((flags & FL_LONG) != 0)
            {
#ifdef CONFIG_LIBC_NUMBERED_ARGS
              if ((flags & FL_ARGNUMBER) != 0)
                {
                  x = arglist[argnumber - 1].value.ul;
                }
              else
                {
                  x = va_arg(ap, unsigned long);
                }
#else
                x = va_arg(ap, unsigned long);
#endif
            }
          else
            {
#ifdef CONFIG_LIBC_NUMBERED_ARGS
              if ((flags & FL_ARGNUMBER) != 0)
                {
                  x = (unsigned int)arglist[argnumber - 1].value.u;
                }
              else
                {
                  x = va_arg(ap, unsigned int);
                }
#else
                x = va_arg(ap, unsigned int);
#endif
              if ((flags & FL_SHORT) != 0)
                {
                  if ((flags & FL_REPD_TYPE) == 0)
                    {
                      x = (unsigned short)x;
                    }
                  else
                    {
                      x = (unsigned char)x;
                    }
                }
            }

          flags &= ~(FL_PLUS | FL_SPACE);

          switch (c)
            {
            case 'u':
              flags &= ~FL_ALT;
              base = 10;
              break;

            case 'o':
              base = 8;
              break;

            case 'p':
              c = fmt_char(fmt);
              switch (c)
                {
                  case 'V':
                    {
                      FAR struct va_format *vaf = (FAR void *)(uintptr_t)x;
#ifdef va_copy
                      va_list copy;

                      va_copy(copy, *vaf->va);
                      lib_vsprintf(stream, vaf->fmt, copy);
                      va_end(copy);
#else
                      lib_vsprintf(stream, vaf->fmt, *vaf->va);
#endif
                      continue;
                    }

#ifdef CONFIG_ALLSYMS
                  case 'S':
                  case 's':
                    {
                      FAR const struct symtab_s *symbol;
                      FAR void *addr = (FAR void *)(uintptr_t)x;
                      size_t symbolsize;

                      symbol = allsyms_findbyvalue(addr, &symbolsize);
                      if (symbol != NULL)
                        {
                          pnt = symbol->sym_name;
                          while (*pnt != '\0')
                            {
                              lib_stream_put(stream, *pnt++);
                            }

                          if (c == 'S')
                            {
                              sprintf_internal(stream, "+%#tx/%#zx",
                                               addr - symbol->sym_value,
                                               symbolsize);
                            }

                          continue;
                        }

                      break;
                    }
#endif

                  default:
                    fmt_ungetc(fmt);
                    break;
                }

              flags |= FL_ALT;

              /* no break */

            case 'x':
              if ((flags & FL_ALT) != 0)
                {
                  flags |= FL_ALTHEX;
                }

              base = 16;
              break;

            case 'X':
              if ((flags & FL_ALT) != 0)
                {
                  flags |= (FL_ALTHEX | FL_ALTUPP);
                }

              base = 16 | XTOA_UPPER;
              break;

            default:
              lib_stream_put(stream, '%');
              lib_stream_put(stream, c);
              continue;
            }

          if ((flags & FL_PREC) != 0 && prec == 0 && x == 0)
            {
              c = 0;
            }
          else
            {
#if !defined(CONFIG_LIBC_LONG_LONG) && defined(CONFIG_HAVE_LONG_LONG)
              DEBUGASSERT(x <= ULONG_MAX);
#endif
              c = __ultoa_invert(x, (FAR char *)buf, base) - (FAR char *)buf;
            }

          flags &= ~FL_NEGATIVE;
        }

      len = c;

      if ((flags & FL_PREC) != 0)
        {
          flags &= ~FL_ZFILL;
          if (len < prec)
            {
              len = prec;
              if ((flags & FL_ALT) != 0 && (flags & FL_ALTHEX) == 0)
                {
                  flags &= ~FL_ALT;
                }
            }
        }

      if ((flags & FL_ALT) != 0)
        {
          if (buf[c - 1] == '0')
            {
              flags &= ~(FL_ALT | FL_ALTHEX | FL_ALTUPP);
            }
          else
            {
              len += 1;
              if ((flags & FL_ALTHEX) != 0)
                {
                  len += 1;
                }
            }
        }
      else if ((flags & (FL_NEGATIVE | FL_PLUS | FL_SPACE)) != 0)
        {
          len += 1;
        }

      if ((flags & FL_LPAD) == 0)
        {
          if ((flags & FL_ZFILL) != 0)
            {
              prec = c;
              if (len < width)
                {
                  prec += width - len;
                  len = width;
                }
            }

          while (len < width)
            {
              lib_stream_put(stream, ' ');
              len++;
            }
        }

      width = (len < width) ? width - len : 0;

      if ((flags & FL_ALT) != 0)
        {
          lib_stream_put(stream, '0');
          if ((flags & FL_ALTHEX) != 0)
            {
              lib_stream_put(stream, flags & FL_ALTUPP ? 'X' : 'x');
            }
        }
      else if ((flags & (FL_NEGATIVE | FL_PLUS | FL_SPACE)) != 0)
        {
          unsigned char z = ' ';
          if ((flags & FL_PLUS) != 0)
            {
              z = '+';
            }

          if ((flags & FL_NEGATIVE) != 0)
            {
              z = '-';
            }

          lib_stream_put(stream, z);
        }

      while (prec > c)
        {
          lib_stream_put(stream, '0');
          prec--;
        }

      while (c)
        {
          lib_stream_put(stream, buf[--c]);
        }

tail:

      /* Tail is possible.  */

      while (width)
        {
          lib_stream_put(stream, ' ');
          width--;
        }
    }

ret:
#ifdef CONFIG_LIBC_NUMBERED_ARGS
  return stream ? stream->nput : total_len;
#else
  return stream->nput;
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int lib_vsprintf(FAR struct lib_outstream_s *stream,
                 FAR const IPTR char *fmt, va_list ap)
{
#ifdef CONFIG_LIBC_NUMBERED_ARGS
  struct arg_s arglist[NL_ARGMAX];
  int numargs;
  int i;

  /* We do 2 passes of parsing and fill the arglist between the passes. */

  numargs = vsprintf_internal(NULL, arglist, NL_ARGMAX, fmt, ap);

  for (i = 0; i < numargs; i++)
    {
      switch (arglist[i].type)
        {
        case TYPE_LONG_LONG:
#ifdef CONFIG_HAVE_LONG_LONG
          arglist[i].value.ull = va_arg(ap, unsigned long long);
          break;
#endif

        case TYPE_LONG:
          arglist[i].value.ul = va_arg(ap, unsigned long);
          break;

        case TYPE_INT:
          arglist[i].value.u = va_arg(ap, unsigned int);
          break;

        case TYPE_DOUBLE:
          arglist[i].value.d = va_arg(ap, double);
          break;

        case TYPE_CHAR_POINTER:
          arglist[i].value.cp = va_arg(ap, FAR char *);
          break;
        }
    }

  return vsprintf_internal(stream, arglist, numargs, fmt, ap);
#else
  return vsprintf_internal(stream, NULL, 0, fmt, ap);
#endif
}
