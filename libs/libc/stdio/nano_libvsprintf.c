/****************************************************************************
 * libs/libc/stdio/nano_libvsprintf.c
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

#include <nuttx/streams.h>

#include "nano_dtoa_engine.h"
#include "nano_ultoa_invert.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration options for nano-printf PRINTF_LEVEL */

#define PRINTF_MIN 1
#define PRINTF_STD 2
#define PRINTF_FLT 3

/* This file can be compiled into more than one flavour.  The default
 * is to offer the usual modifiers and integer formatting support
 * (level 2).  Level 1 maintains a minimal version that just offers
 * integer formatting, but no modifier support whatsoever.  Level 3 is
 * intented for floating point support.
 */

#if defined(CONFIG_NANO_PRINTLEVEL)
#  define PRINTF_LEVEL CONFIG_NANO_PRINTLEVEL
#elif defined(CONFIG_LIBC_FLOATINGPOINT)
#  define PRINTF_LEVEL PRINTF_FLT
#else
#  define PRINTF_LEVEL PRINTF_STD
#endif

#if PRINTF_LEVEL == PRINTF_MIN || PRINTF_LEVEL == PRINTF_STD || \
    PRINTF_LEVEL == PRINTF_FLT

/* OK */

#else
#  error "Not a known printf level."
#endif

#ifdef putc
#  undef putc
#endif

#define putc(c,stream) (total_len++, (stream)->put(stream, c))

#if PRINTF_LEVEL <= PRINTF_MIN

#  define FL_ALTHEX       0x04
#  define FL_ALT          0x10
#  define FL_ALTLWR       0x20
#  define FL_NEGATIVE     0x40
#  define FL_LONG         0x80

#else /* i.e. PRINTF_LEVEL > PRINTF_MIN */

/* Order is relevant here and matches order in format string */

#  define FL_ZFILL        0x0001
#  define FL_PLUS         0x0002
#  define FL_SPACE        0x0004
#  define FL_LPAD         0x0008
#  define FL_ALT          0x0010

#  define FL_WIDTH        0x0020
#  define FL_PREC         0x0040

#  define FL_LONG         0x0080
#  define FL_LONGLONG     0x0100
#  define FL_SHORT        0x0200

#  define FL_NEGATIVE     0x0400

#  define FL_ALTUPP       0x0800
#  define FL_ALTHEX       0x1000

#  define FL_FLTUPP       0x2000
#  define FL_FLTEXP       0x4000
#  define FL_FLTFIX       0x8000

#endif

/* Support special access to CODE-space strings for Harvard architectures */

#ifdef CONFIG_ARCH_ROMGETC
#  define fmt_char(fmt)   up_romgetc((fmt)++)
#else
#  define fmt_char(fmt)   (*(fmt)++)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#if PRINTF_LEVEL <= PRINTF_MIN
int lib_vsprintf(FAR struct lib_outstream_s *stream,
                 FAR const IPTR char *fmt, va_list ap)
{
  unsigned char c;              /* Holds a char from the format string */
  unsigned char flags;
  unsigned char buf[11];        /* Size for -1 in octal, without '\0' */
  int total_len = 0;

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

          putc(c, stream);
        }

      for (flags = 0; !(flags & FL_LONG);       /* 'll' will detect as error */
           c = fmt_char(fmt))
        {
          if (c != '\0' && strchr(" +-.0123456789h", c) != NULL)
            {
              continue;
            }

          if (c == '#')
            {
              flags |= FL_ALT;
              continue;
            }

          if (c == 'l')
            {
              flags |= FL_LONG;
              continue;
            }

          break;
        }

      /* Only a format character is valid.  */

      if (c && strchr("EFGefg", c))
        {
          (void)va_arg(ap, double);
          putc('?', stream);
          continue;
        }

      {
        const char *pnt;

        switch (c)
          {
          case 'c':
            putc(va_arg(ap, int), stream);
            continue;

          case 'S':

            /* FALLTHROUGH */

          case 's':
            pnt = va_arg(ap, char *);
            while ((c = *pnt++) != 0)
              {
                putc(c, stream);
              }

            continue;
          }
      }

      if (c == 'd' || c == 'i')
        {
          long x = (flags & FL_LONG) ? va_arg(ap, long) : va_arg(ap, int);

          flags &= ~FL_ALT;
          if (x < 0)
            {
              x = -x;

              /* `putc ('-', stream)' will considarably inlarge stack size. So
               * flag is used.
               */

              flags |= FL_NEGATIVE;
            }

          c = __ultoa_invert(x, (char *)buf, 10) - (char *)buf;
        }
      else
        {
          int base;

          switch (c)
            {
            case 'u':
              flags &= ~FL_ALT;
              base   = 10;
              goto ultoa;

            case 'o':
              base   = 8;
              goto ultoa;

            case 'p':
              flags |= FL_ALT;

              /* no break */

            case 'x':
              flags |= (FL_ALTHEX | FL_ALTLWR);
              base   = 16;
              goto ultoa;

            case 'X':
              flags |= FL_ALTHEX;
              base   = 16 | XTOA_UPPER;

            ultoa:
              c = __ultoa_invert((flags & FL_LONG)
                                 ? va_arg(ap, unsigned long)
                                 : va_arg(ap, unsigned int),
                                 (char *)buf, base) - (char *)buf;
              break;

            default:
              goto ret;
            }
        }

      /* Integer number output.  */

      if ((flags & FL_NEGATIVE) != 0)
        {
          putc('-', stream);
        }

      if ((flags & FL_ALT) != 0 && buf[c - 1] != '0')
        {
          putc('0', stream);
          if ((flags & FL_ALTHEX) != 0)
            {
#  if  FL_ALTLWR != 'x' - 'X'
#    error
#  endif
              putc('X' + (flags & FL_ALTLWR), stream);
            }
        }

      do
        {
          putc(buf[--c], stream);
        }
      while (c);
    }

ret:
  return total_len;
}

#else /* i.e. PRINTF_LEVEL > PRINTF_MIN */

int lib_vsprintf(FAR struct lib_outstream_s *stream,
                 FAR const IPTR char *fmt, va_list ap)
{
  unsigned char c;              /* Holds a char from the format string */
  uint16_t flags;
  int width;
  int prec;
  union
  {
#  ifdef CONFIG_LIBC_LONG_LONG
    unsigned char __buf[22];  /* Size for -1 in octal, without '\0' */
#  else
    unsigned char __buf[11];  /* Size for -1 in octal, without '\0' */
#  endif
#  if PRINTF_LEVEL >= PRINTF_FLT
    struct dtoa_s __dtoa;
#  endif
  } u;
  const char *pnt;
  size_t size;
  unsigned char len;
  int total_len = 0;

#  define buf     (u.__buf)
#  define _dtoa   (u.__dtoa)

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

          putc(c, stream);
        }

      flags = 0;
      width = 0;
      prec  = 0;

      do
        {
          if (flags < FL_WIDTH)
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

          if (c == 'l')
            {
              if ((flags & FL_LONG) != 0)
                {
                  flags |= FL_LONGLONG;
                }

              flags |= FL_LONG;
              flags &= ~FL_SHORT;
              continue;
            }

          if (c == 'h')
            {
              flags |= FL_SHORT;
              flags &= ~FL_LONG;
              continue;
            }

          break;
        }
      while ((c = fmt_char(fmt)) != 0);

      /* Only a format character is valid.  */

#  if 'F' != 'E'+1  ||  'G' != 'F'+1  ||  'f' != 'e'+1  ||  'g' != 'f'+1
#    error
#  endif

#  if PRINTF_LEVEL >= PRINTF_FLT
      if (c >= 'E' && c <= 'G')
        {
          flags |= FL_FLTUPP;
          c += 'e' - 'E';
          goto flt_oper;
        }
      else if (c >= 'e' && c <= 'g')
        {
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

          ndigs = __dtoa_engine(va_arg(ap, double), &_dtoa, ndigs, ndecimal);
          exp   = _dtoa.exp;

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
              const char *p;

              ndigs = sign ? 4 : 3;
              if (width > ndigs)
                {
                  width -= ndigs;
                  if ((flags & FL_LPAD) == 0)
                    {
                      do
                        {
                          putc(' ', stream);
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
                  putc(sign, stream);
                }

              p = "inf";
              if (_dtoa.flags & DTOA_NAN)
                {
                  p = "nan";
                }

#    if ('I'-'i' != 'N'-'n') || ('I'-'i' != 'F'-'f') || ('I'-'i' != 'A'-'a')
#      error
#    endif
              while ((ndigs = *p) != 0)
                {
                  if ((flags & FL_FLTUPP) != 0)
                    {
                      ndigs += 'I' - 'i';
                    }

                  putc(ndigs, stream);
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
                  putc(' ', stream);
                  width--;
                }
            }

          if (sign != 0)
            {
              putc(sign, stream);
            }

          if ((flags & FL_LPAD) == 0)
            {
              while (width)
                {
                  putc('0', stream);
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
                      putc('.', stream);
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
                          putc('.', stream);
                        }

                      break;
                    }

                  putc(out, stream);
                }
              while (1);

              if (n == exp  && (_dtoa.digits[0] > '5' ||
                  (_dtoa.digits[0] == '5' && !(_dtoa.flags & DTOA_CARRY))))
                {
                  out = '1';
                }

              putc(out, stream);
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

              putc(_dtoa.digits[0], stream);
              if (prec > 0)
                {
                  uint8_t pos;

                  putc('.', stream);
                  for (pos = 1; pos < 1 + prec; pos++)
                    {
                      putc(pos < ndigs ? _dtoa.digits[pos] : '0', stream);
                    }
                }
              else if ((flags & FL_ALT) != 0)
                {
                  putc('.', stream);
                }

              /* Exponent */

              putc(flags & FL_FLTUPP ? 'E' : 'e', stream);
              ndigs = '+';
              if (exp < 0 || (exp == 0 && (_dtoa.flags & DTOA_CARRY) != 0))
                {
                  exp = -exp;
                  ndigs = '-';
                }

              putc(ndigs, stream);
              for (ndigs = '0'; exp >= 10; exp -= 10)
                {
                  ndigs += 1;
                }

              putc(ndigs, stream);
              putc('0' + exp, stream);
            }

          goto tail;
        }

#  else /* to: PRINTF_LEVEL >= PRINTF_FLT */
      if ((c >= 'E' && c <= 'G') || (c >= 'e' && c <= 'g'))
        {
          (void)va_arg(ap, double);
          pnt = "*float*";
          size = sizeof("*float*") - 1;
          goto str_lpad;
        }
#  endif

      switch (c)
        {

        case 'c':
          buf[0] = va_arg(ap, int);
          pnt = (char *)buf;
          size = 1;
          goto str_lpad;

        case 's':
        case 'S':
          pnt = va_arg(ap, char *);
          size = strnlen(pnt, (flags & FL_PREC) ? prec : ~0);

        str_lpad:
          if ((flags & FL_LPAD) == 0)
            {
              while (size < width)
                {
                  putc(' ', stream);
                  width--;
                }
            }

          while (size)
            {
              putc(*pnt++, stream);
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
#ifndef CONFIG_LIBC_LONG_LONG
          long x;
#else
          long long x;

          if ((flags & FL_LONGLONG) != 0)
            {
              x = va_arg(ap, long long);
            }
          else
#endif
          if ((flags & FL_LONG) != 0)
            {
              x = va_arg(ap, long);
            }
          else
            {
              x = va_arg(ap, int);
              if ((flags & FL_SHORT) != 0)
                {
                  x = (short)x;
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
              c = __ultoa_invert(x, (char *)buf, 10) - (char *)buf;
            }
        }
      else
        {
          int base;
#ifndef CONFIG_LIBC_LONG_LONG
          unsigned long x;
#else
          unsigned long long x;

          if ((flags & FL_LONGLONG) != 0)
            {
              x = va_arg(ap, unsigned long long);
            }
          else
#endif
          if ((flags & FL_LONG) != 0)
            {
              x = va_arg(ap, unsigned long);
            }
          else
            {
              x = va_arg(ap, unsigned int);
              if ((flags & FL_SHORT) != 0)
                {
                  x = (unsigned short)x;
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
              putc('%', stream);
              putc(c, stream);
              continue;
            }

          if ((flags & FL_PREC) != 0 && prec == 0 && x == 0)
            {
              c = 0;
            }
          else
            {
              c = __ultoa_invert(x, (char *)buf, base) - (char *)buf;
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
              putc(' ', stream);
              len++;
            }
        }

      width = (len < width) ? width - len : 0;

      if ((flags & FL_ALT) != 0)
        {
          putc('0', stream);
          if ((flags & FL_ALTHEX) != 0)
            {
              putc(flags & FL_ALTUPP ? 'X' : 'x', stream);
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

          putc(z, stream);
        }

      while (prec > c)
        {
          putc('0', stream);
          prec--;
        }

      while (c)
        {
          putc(buf[--c], stream);
        }

    tail:

    /* Tail is possible.  */

      while (width)
        {
          putc(' ', stream);
          width--;
        }
    }

ret:
  return total_len;
}

#endif /* PRINTF_LEVEL > PRINTF_MIN */
