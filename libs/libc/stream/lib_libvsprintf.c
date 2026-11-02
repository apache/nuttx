/****************************************************************************
 * libs/libc/stream/lib_libvsprintf.c
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: 2002, Alexander Popov (sasho@vip.bg)
 * SPDX-FileCopyrightText: 2002,2004,2005 Joerg Wunsch
 * SPDX-FileCopyrightText: 2005, Helmut Wallner
 * SPDX-FileCopyrightText: 2007, Dmitry Xmelkov
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

#include <nuttx/streams.h>
#ifdef CONFIG_ALLSYMS
#include <nuttx/allsyms.h>
#include <nuttx/symtab.h>
#endif

#include <assert.h>
#include <errno.h>
#include <limits.h>
#include <string.h>
#include <wchar.h>
#include <sys/param.h>

#include "lib_dtoa_engine.h"
#include "lib_ultoa_invert.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define stream_putc(c,stream)  (total_len++, lib_stream_putc(stream, c))
#define stream_puts(buf, len, stream) \
        (total_len += len, lib_stream_puts(stream, buf, len))
#define stream_repeat(c, len, stream) \
        (total_len += vsprintf_repeat(stream, c, len))

#define REPEAT_BUFLEN      16

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

/* L, as ll, means a long double in a floating point conversion */

#ifdef CONFIG_HAVE_LONG_DOUBLE
#  define FL_IS_LDOUBLE(f) (((f) & (FL_LONG | FL_REPD_TYPE)) == \
                            (FL_LONG | FL_REPD_TYPE))
#else
#  define FL_IS_LDOUBLE(f) 0
#endif

#define TYPE_INT           1
#define TYPE_LONG          2
#define TYPE_LONG_LONG     3
#define TYPE_DOUBLE        4
#define TYPE_CHAR_POINTER  5
#define TYPE_LONG_DOUBLE   6

/* Support special access to CODE-space strings for Harvard architectures */

#ifdef CONFIG_ARCH_ROMGETC
#  define fmt_char(fmt)   up_romgetc((fmt)++)
#else
#  define fmt_char(fmt)   (*(fmt)++)
#endif

#define fmt_ungetc(fmt)   ((fmt)--)

/* binary128 converts by hand, so that printf() does not pull in the
 * soft-float code of libgcc.  Any other long double uses a cast.
 */

#if defined(CONFIG_LIBC_FLOATINGPOINT) && defined(CONFIG_HAVE_LONG_DOUBLE) && \
    LDBL_MANT_DIG == 113 && DBL_MANT_DIG == 53
#  define HAVE_LDTOD_BY_HAND
#  ifdef CONFIG_ENDIAN_BIG
#    define LDTOD_HI 0
#    define LDTOD_LO 1
#  else
#    define LDTOD_HI 1
#    define LDTOD_LO 0
#  endif
#else
#  define vsprintf_ldtod(v) ((double)(v))
#endif

/* e, f, g and a in either case: 0x20 makes an ASCII letter lower case */

#define fmt_isfloat(c)    ((((c) | 0x20) >= 'e' && ((c) | 0x20) <= 'g') || \
                           ((c) | 0x20) == 'a')

/****************************************************************************
 * Private Types
 ****************************************************************************/

union arg_u
{
  unsigned int u;
  unsigned long ul;
  unsigned long long ull;
  double d;
  FAR char *cp;
};
struct arg_s
{
#ifdef CONFIG_LIBC_NUMBERED_ARGS
  unsigned char type[NL_ARGMAX];
#endif
  FAR union arg_u *value;
};

/****************************************************************************
 * Private Constant Data
 ****************************************************************************/

static const char g_nullstring[] = "(null)";
#ifdef CONFIG_LIBC_FLOATINGPOINT
static const char g_inf_lower[] = "inf";
static const char g_nan_lower[] = "nan";
static const char g_inf_upper[] = "INF";
static const char g_nan_upper[] = "NAN";
#endif
static const char g_spaces[REPEAT_BUFLEN] = "                ";
static const char g_zeroes[REPEAT_BUFLEN] = "0000000000000000";

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int vsprintf_internal(FAR struct lib_outstream_s *stream,
                             FAR struct arg_s *arglist, int numargs,
                             FAR const IPTR char *fmt, va_list ap)
           printf_like(4, 0);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int vsprintf_repeat(FAR struct lib_outstream_s *stream, int ch,
                           int len)
{
  FAR const char *buf = ch == '0' ? g_zeroes : g_spaces;
  int total_len = len;

  while (len > 0)
    {
      int n = MIN(len, REPEAT_BUFLEN);

      lib_stream_puts(stream, buf, n);
      len -= n;
    }

  return total_len;
}

#ifdef HAVE_LDTOD_BY_HAND
/* Round to nearest, ties to even, as the cast does.  All 64-bit shifts have
 * a constant count, so a 32-bit CPU needs no libgcc call either.
 */

static double vsprintf_ldtod(long double value)
{
  union
  {
    long double ld;
    uint64_t w[2];
  } in;

  union
  {
    double d;
    uint64_t u;
  } out;

  uint64_t frac;
  uint64_t rest;
  int exp;

  in.ld = value;
  exp   = (in.w[LDTOD_HI] >> 48) & 0x7fff;
  frac  = ((in.w[LDTOD_HI] & 0x0000ffffffffffffull) << 4) |
          (in.w[LDTOD_LO] >> 60);
  rest  = in.w[LDTOD_LO] << 4;
  out.u = in.w[LDTOD_HI] & 0x8000000000000000ull;

  if (exp == 0x7fff)
    {
      /* Infinity, or a NaN: its result stays a NaN and is quiet */

      out.u |= 0x7ff0000000000000ull | frac;
      if ((frac | rest) != 0)
        {
          out.u |= 0x0008000000000000ull;
        }

      return out.d;
    }

  exp -= 16383 - 1023;
  if (exp >= 0x7ff)
    {
      out.u |= 0x7ff0000000000000ull;
      return out.d;
    }

  if (exp <= 0)
    {
      if (exp < -52)
        {
          return out.d;
        }

      /* A subnormal result: shift the bits into rest and keep a sticky bit */

      frac |= 0x0010000000000000ull;
      do
        {
          rest = (frac << 63) | (rest >> 1) | (rest & 1);
          frac >>= 1;
        }
      while (exp++ < 0);

      exp = 0;
    }

  if ((rest >> 63) != 0 && ((rest << 1) != 0 || (frac & 1) != 0))
    {
      frac++;
    }

  /* A carry out of frac goes into the exponent, as it must */

  out.u += ((uint64_t)exp << 52) + frac;
  return out.d;
}
#endif

static int vsprintf_putnum(FAR struct lib_outstream_s *stream,
                           FAR char *buf, int len)
{
  int i;
  int j;

  for (i = 0, j = len - 1; i < j; i++, j--)
    {
      char tmp = buf[i];

      buf[i] = buf[j];
      buf[j] = tmp;
    }

  if (len > 0)
    {
      lib_stream_puts(stream, buf, len);
    }

  return len;
}

static int vsprintf_scan_literal(FAR struct lib_outstream_s *stream,
                                 FAR const IPTR char **fmtp,
                                 FAR unsigned char *conv)
{
  FAR const IPTR char *fmt = *fmtp;
  unsigned char c;
  int total_len = 0;

#ifndef CONFIG_ARCH_ROMGETC
  FAR const char *start = fmt;

  for (; ; )
    {
      c = fmt_char(fmt);
      if (c == '\0')
        {
#  ifdef CONFIG_LIBC_NUMBERED_ARGS
          if (stream != NULL)
#  endif
            {
              size_t litlen = (fmt - start) - 1;

              if (litlen > 0)
                {
                  total_len += litlen;
                  lib_stream_puts(stream, start, litlen);
                }
            }

          break;
        }

      if (c == '%')
        {
          size_t litlen = (fmt - start) - 1;

          c = fmt_char(fmt);
          if (c != '%')
            {
#  ifdef CONFIG_LIBC_NUMBERED_ARGS
              if (stream != NULL)
#  endif
                {
                  if (litlen > 0)
                    {
                      total_len += litlen;
                      lib_stream_puts(stream, start, litlen);
                    }
                }

              break;
            }

#  ifdef CONFIG_LIBC_NUMBERED_ARGS
          if (stream != NULL)
#  endif
            {
              total_len += litlen + 1;
              lib_stream_puts(stream, start, litlen + 1);
            }

          start = fmt;
        }
    }
#else
  for (; ; )
    {
      c = fmt_char(fmt);
      if (c == '\0')
        {
          break;
        }

      if (c == '%')
        {
          c = fmt_char(fmt);
          if (c != '%')
            {
              break;
            }
        }

#  ifdef CONFIG_LIBC_NUMBERED_ARGS
      if (stream != NULL)
#  endif
        {
          total_len++;
          lib_stream_putc(stream, c);
        }
    }
#endif

  *fmtp = fmt;
  *conv = c;

  return total_len;
}

static int vsprintf_put_field(FAR struct lib_outstream_s *stream,
                              FAR const char *pnt, size_t size,
                              FAR int *width, uint16_t flags)
{
  int total_len = 0;
  int w = *width;

  if ((flags & FL_LPAD) == 0)
    {
      if (w > 0 && size < w)
        {
          int pad = w - size;

          total_len += vsprintf_repeat(stream, ' ', pad);
          w -= pad;
        }
    }

  total_len += size;
  lib_stream_puts(stream, pnt, size);
  *width = w >= size ? w - size : 0;

  return total_len;
}

static int vsprintf_put_char(FAR struct lib_outstream_s *stream,
                             uint16_t flags, int ch, FAR int *width,
                             FAR char *buf)
{
  FAR const char *pnt;
  size_t size;

#ifdef CONFIG_LIBC_LOCALE
  if ((flags & FL_LONG) != 0)
    {
      mbstate_t ps;

      /* %lc: a wide character, written as its multibyte sequence. */

      memset(&ps, 0, sizeof(ps));
      size = wcrtomb(buf, (wchar_t)ch, &ps);
      if (size == (size_t)-1)
        {
          return -1;
        }
    }
  else
#endif
    {
      buf[0] = ch;
      size = 1;
    }

  pnt = buf;
  return vsprintf_put_field(stream, pnt, size, width, flags);
}

#ifdef CONFIG_LIBC_LOCALE
static int vsprintf_put_wstring(FAR struct lib_outstream_s *stream,
                                FAR const wchar_t *ws, uint16_t flags,
                                FAR int *width, int prec)
{
  char mb[MB_LEN_MAX];
  mbstate_t ps;
  size_t size;
  size_t n;
  int total_len = 0;
  int i;

  /* %ls: a wide string, written as multibyte characters.  The precision
   * limits the bytes, and a character that does not fit is not written in
   * part.  First count the bytes.
   */

  memset(&ps, 0, sizeof(ps));
  for (size = 0, i = 0; ws[i] != L'\0'; i++)
    {
      n = wcrtomb(mb, ws[i], &ps);
      if (n == (size_t)-1)
        {
          return -1;
        }

      if ((flags & FL_PREC) != 0 && size + n > (size_t)prec)
        {
          break;
        }

      size += n;
    }

  if ((flags & FL_LPAD) == 0)
    {
      if (*width > 0 && size < *width)
        {
          int pad = *width - size;

          total_len += vsprintf_repeat(stream, ' ', pad);
          *width -= pad;
        }
    }

  memset(&ps, 0, sizeof(ps));
  for (n = 0, i = 0; n < size; i++)
    {
      size_t m = wcrtomb(mb, ws[i], &ps);

      total_len += m;
      lib_stream_puts(stream, mb, m);
      n += m;
    }

  *width = *width >= size ? *width - size : 0;

  return total_len;
}
#endif

#ifdef CONFIG_LIBC_NUMBERED_ARGS
static int vsprintf_check_arg(FAR struct lib_outstream_s *stream,
                              FAR struct arg_s *arglist, int numargs,
                              int argnumber, uint16_t flags,
                              unsigned char c, FAR size_t *total_len)
{
  if ((flags & FL_ARGNUMBER) != 0)
    {
      if (argnumber <= 0 || argnumber > numargs)
        {
          return -1;
        }

      if (stream == NULL)
        {
          unsigned char type;

          if (fmt_isfloat(c))
            {
              type = FL_IS_LDOUBLE(flags) ? TYPE_LONG_DOUBLE : TYPE_DOUBLE;
            }
          else if (c == 'i' || c == 'd' || c == 'u' || c == 'p' ||
                   c == 'o' || c == 'x' || c == 'X')
            {
              if ((flags & FL_LONG) == 0)
                {
                  type = TYPE_INT;
                }
              else if ((flags & FL_REPD_TYPE) == 0)
                {
                  type = TYPE_LONG;
                }
              else
                {
                  type = TYPE_LONG_LONG;
                }
            }
          else if (c == 'c')
            {
              type = TYPE_INT;
            }
          else if (c == 's' || c == 'S' || c == 'n')
            {
              type = TYPE_CHAR_POINTER;
            }
          else
            {
              return -1;
            }

          arglist->type[argnumber - 1] = type;
          if (argnumber > *total_len)
            {
              *total_len = argnumber;
            }

          return 1; /* We do only parsing */
        }
    }
  else if (stream == NULL)
    {
      return 1; /* We do only parsing */
    }

  return 0;
}
#endif

static size_t vsprintf_put_integer(FAR struct lib_outstream_s *stream,
                                   uint16_t flags, FAR char *buf,
                                   int digits, FAR int *width, int prec)
{
  unsigned int len = digits;
  size_t total_len = 0;
  int w = *width;

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
      /* digits is 0 for a value of 0 with a precision of 0: then %#o has
       * a "0" from the flag, and a hexadecimal value has no prefix.
       */

      if (digits == 0 ? (flags & FL_ALTHEX) != 0 : buf[digits - 1] == '0')
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

  if (len > INT_MAX)
    {
      *width = 0;
      return INT_MAX + 1u;
    }

  if ((flags & FL_LPAD) == 0)
    {
      if ((flags & FL_ZFILL) != 0)
        {
          prec = digits;
          if (len < w)
            {
              prec += w - len;
              len = w;
            }
        }

      if (len < w)
        {
          total_len += vsprintf_repeat(stream, ' ', w - len);
          len = w;
        }
    }

  w = len < w ? w - len : 0;

  if ((flags & FL_ALT) != 0)
    {
      total_len++;
      lib_stream_putc(stream, '0');
      if ((flags & FL_ALTHEX) != 0)
        {
          total_len++;
          lib_stream_putc(stream, flags & FL_ALTUPP ? 'X' : 'x');
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

      total_len++;
      lib_stream_putc(stream, z);
    }

  if (prec > digits)
    {
      total_len += vsprintf_repeat(stream, '0', prec - digits);
      prec = digits;
    }

  total_len += vsprintf_putnum(stream, buf, digits);
  *width = w;

  return total_len;
}

#ifdef CONFIG_LIBC_FLOATINGPOINT
static int vsprintf_put_naninf(FAR struct lib_outstream_s *stream,
                               uint16_t flags, int dtoa_flags, int sign,
                               FAR int *width)
{
  FAR const char *p;
  int total_len = 0;
  int w = *width;
  int len = sign ? 4 : 3;

  if (w > len)
    {
      w -= len;
      if ((flags & FL_LPAD) == 0)
        {
          total_len += vsprintf_repeat(stream, ' ', w);
          w = 0;
        }
    }
  else
    {
      w = 0;
    }

  if (sign)
    {
      total_len++;
      lib_stream_putc(stream, sign);
    }

  if (dtoa_flags & DTOA_NAN)
    {
      p = (flags & FL_FLTUPP) != 0 ? g_nan_upper : g_nan_lower;
    }
  else
    {
      p = (flags & FL_FLTUPP) != 0 ? g_inf_upper : g_inf_lower;
    }

  total_len += 3;
  lib_stream_puts(stream, p, 3);
  *width = w;

  return total_len;
}

static void vsprintf_float_format(unsigned char c, int prec,
                                  FAR uint16_t *flags,
                                  FAR unsigned int *ndigs,
                                  FAR int *ndecimal)
{
  *ndigs = MIN(prec, DTOA_MAX_DIG);
  *ndecimal = 0;
  *flags &= ~(FL_FLTEXP | FL_FLTFIX);

  if (c == 'e')
    {
      if (*ndigs < DTOA_MAX_DIG)
        {
          (*ndigs)++;
        }

      *flags |= FL_FLTEXP;
    }
  else if (c == 'f')
    {
      *ndecimal = *ndigs;
      *ndigs    = DTOA_MAX_DIG;
      *flags |= FL_FLTFIX;
    }
  else if (*ndigs == 0)
    {
      *ndigs = 1;
    }
}

static void vsprintf_adjust_float_general(FAR uint16_t *flags,
                                          FAR int *prec,
                                          FAR unsigned int *ndigs, int exp,
                                          FAR const struct dtoa_s *dtoa)
{
  if ((*flags & (FL_FLTEXP | FL_FLTFIX)) != 0)
    {
      return;
    }

  /* 'g(G)' format */

  *prec = *ndigs;

  /* Remove trailing zeros */

  while (*ndigs > 0 && dtoa->digits[*ndigs - 1] == '0')
    {
      (*ndigs)--;
    }

  if (-4 <= exp && exp < *prec)
    {
      *flags |= FL_FLTFIX;

      if (exp < 0 || *ndigs > exp)
        {
          *prec = *ndigs - (exp + 1);
        }
      else
        {
          *prec = 0;
        }
    }
  else
    {
      /* Limit displayed precision to available precision */

      *prec = *ndigs - 1;
    }
}

static unsigned int vsprintf_float_length(uint16_t flags, int sign, int exp,
                                          int prec)
{
  unsigned int len;

  if ((flags & FL_FLTFIX) != 0)
    {
      len = exp > 0 ? exp + 1 : 1;
    }
  else
    {
      len = 5;              /* 1e+00 */
      if (exp >= 100 || exp <= -100)
        {
          len++;
        }
    }

  if (sign != 0)
    {
      len += 1;
    }

  if (prec != 0)
    {
      len += prec + 1u;
    }
  else if ((flags & FL_ALT) != 0)
    {
      len += 1;
    }

  return len;
}

static int vsprintf_put_float_prefix(FAR struct lib_outstream_s *stream,
                                     uint16_t flags, int sign,
                                     FAR int *width, int len)
{
  int total_len = 0;
  int w = *width;

  w = w > len ? w - len : 0;

  if ((flags & (FL_LPAD | FL_ZFILL)) == 0)
    {
      total_len += vsprintf_repeat(stream, ' ', w);
      w = 0;
    }

  if (sign != 0)
    {
      total_len++;
      lib_stream_putc(stream, sign);
    }

  if ((flags & FL_LPAD) == 0)
    {
      total_len += vsprintf_repeat(stream, '0', w);
      w = 0;
    }

  *width = w;

  return total_len;
}

static int vsprintf_put_float_fixed(FAR struct lib_outstream_s *stream,
                                    uint16_t flags,
                                    FAR const struct dtoa_s *dtoa, int exp,
                                    unsigned int ndigs, int prec)
{
  char out;
  int n;
  int total_len = 0;

  /* At this point, we should have exp exponent of leftmost digit in
   * dtoa->digits ndigs number of buffer digits to print prec number
   * of digits after decimal In the loop, 'n' walks over the exponent
   * value.
   */

  n = exp > 0 ? exp : 0;
  do
    {
      /* Insert decimal point at correct place */

      if (n == -1)
        {
          total_len++;
          lib_stream_putc(stream, '.');
        }

      /* Pull digits from buffer when in-range, otherwise use 0 */

      if (0 <= exp - n && exp - n < ndigs)
        {
          out = dtoa->digits[exp - n];
        }
      else
        {
          out = '0';
        }

      if (--n < -prec)
        {
          break;
        }

      if (out == '0')
        {
          int zeroes = 1;

          while (n != -1)
            {
              int next = n - 1;
              int offset = exp - n;

              if (next < -prec ||
                  (0 <= offset && offset < ndigs &&
                   dtoa->digits[offset] != '0'))
                {
                  break;
                }

              zeroes++;
              n = next;
            }

          total_len += vsprintf_repeat(stream, '0', zeroes);
        }
      else
        {
          total_len++;
          lib_stream_putc(stream, out);
        }
    }
  while (1);

  if (n == exp && (dtoa->digits[0] > '5' ||
      (dtoa->digits[0] == '5' && !(dtoa->flags & DTOA_CARRY))))
    {
      out = '1';
    }

  total_len++;
  lib_stream_putc(stream, out);

  if ((flags & FL_ALT) != 0 && n == -1)
    {
      total_len++;
      lib_stream_putc(stream, '.');
    }

  return total_len;
}

static int vsprintf_put_float_exp(FAR struct lib_outstream_s *stream,
                                  uint16_t flags, FAR struct dtoa_s *dtoa,
                                  int exp, unsigned int ndigs, int prec,
                                  FAR char *buf)
{
  int nbytes;
  int sign;
  int len;
  int total_len = 0;

  if (dtoa->digits[0] != '1')
    {
      dtoa->flags &= ~DTOA_CARRY;
    }

  total_len++;
  lib_stream_putc(stream, dtoa->digits[0]);
  if (prec > 0)
    {
      total_len++;
      lib_stream_putc(stream, '.');

      nbytes = MIN(prec, ndigs > 1 ? ndigs - 1 : 0);
      if (nbytes > 0)
        {
          total_len += nbytes;
          lib_stream_puts(stream, &dtoa->digits[1], nbytes);
        }

      if (prec > nbytes)
        {
          total_len += vsprintf_repeat(stream, '0', prec - nbytes);
        }
    }
  else if ((flags & FL_ALT) != 0)
    {
      total_len++;
      lib_stream_putc(stream, '.');
    }

  /* Exponent */

  total_len++;
  lib_stream_putc(stream, flags & FL_FLTUPP ? 'E' : 'e');
  sign = '+';
  if (exp < 0 || (exp == 0 && (dtoa->flags & DTOA_CARRY) != 0))
    {
      exp = -exp;
      sign = '-';
    }

  total_len++;
  lib_stream_putc(stream, sign);
  len = __ultoa_invert(exp, buf, 10) - buf;

  if (exp >= 0 && exp <= 9)
    {
      total_len++;
      lib_stream_putc(stream, '0');
    }

  total_len += vsprintf_putnum(stream, buf, len);

  return total_len;
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
    char __buf[22]; /* Size for -1 in octal, without '\0' */
#ifdef CONFIG_LIBC_FLOATINGPOINT
    struct dtoa_s __dtoa;
#endif
  } u;

#define buf     (u.__buf)
#define _dtoa   (u.__dtoa)

  FAR const char *pnt;
  size_t size;
  int star;
  int result;
  size_t total_len = 0;

#ifdef CONFIG_LIBC_NUMBERED_ARGS
  int argnumber = 0;
#else
  UNUSED(arglist);
  UNUSED(numargs);
#endif

  while (total_len <= INT_MAX)
    {
      total_len += vsprintf_scan_literal(stream, &fmt, &c);
      if (c == '\0')
        {
          goto ret;
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

                  /* XSI thousands grouping: the C locale has no
                   * separator, so ignore the flag.
                   */

                  case '\'':
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
                              arglist->type[index - 1] = TYPE_INT;
                              if (index > total_len)
                                {
                                  total_len = index;
                                }
                            }
                          else
                            {
                              star = arglist->value[index - 1].u;
                              goto set_star;
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
                  star = width;
                  if ((flags & FL_PREC) != 0)
                    {
                      star = prec;
                    }

                  if (star > (INT_MAX - 9) / 10)
                    {
                      star = INT_MAX;
                    }
                  else
                    {
                      star = 10 * star + (c - '0');
                    }

                  goto set_star;
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

                  star = va_arg(ap, int);

set_star:
                  if ((flags & FL_PREC) != 0)
                    {
                      prec = star < 0 ? 0 : star;
                    }
                  else
                    {
                      flags |= FL_WIDTH;
                      if (star < 0)
                        {
                          star = star == INT_MIN ? INT_MAX : -star;
                          flags |= FL_LPAD;
                        }

                      width = star;
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
              if (sizeof(size_t) == sizeof(unsigned short))
                {
                  c = 'h';
                }
              else if (sizeof(size_t) == sizeof(unsigned long))
                {
                  c = 'l';
                }
#if ULLONG_MAX != ULONG_MAX
              else if (sizeof(size_t) == sizeof(unsigned long long))
                {
                  c = 'l';
                  flags |= FL_LONG;
                  flags &= ~FL_SHORT;
                }
#endif
              else
                {
                  /* The only known case that the default will be hit is
                   * the eZ80 which has sizeof(size_t) = 3 which is the
                   * same as the sizeof(int).
                   * Treat as integer with no size qualifier.
                   */

                  continue;
                }
            }

          if (c == 'j')
            {
              /* Same as long long if available. Otherwise, long. */

              flags |= FL_REPD_TYPE;
              flags |= FL_LONG;
              flags &= ~FL_SHORT;
              continue;
            }

          if (c == 'L')
            {
              flags |= FL_LONG | FL_REPD_TYPE;
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

          if (sizeof(FAR void *) == sizeof(unsigned long long))
            {
              flags |= (FL_LONG | FL_REPD_TYPE);
            }
          else
          if (sizeof(FAR void *) == sizeof(unsigned long))
            {
              flags |= FL_LONG;
            }
        }

#ifdef CONFIG_LIBC_NUMBERED_ARGS
      result = vsprintf_check_arg(stream, arglist, numargs, argnumber,
                                  flags, c, &total_len);
      if (result < 0)
        {
          goto ret;
        }

      if (result > 0)
        {
          continue;
        }
#endif

#ifdef CONFIG_LIBC_FLOATINGPOINT
      if ((c >= 'E' && c <= 'G') || c == 'A')
        {
          flags |= FL_FLTUPP;
          c += 'e' - 'E';
          goto flt_oper;
        }
      else if ((c >= 'e' && c <= 'g') || c == 'a')
        {
          double value;
          int exp;              /* Exponent of master decimal digit */
          unsigned int flen;    /* Length of the conversion */
          uint8_t sign;         /* Sign character (or 0) */
          unsigned int ndigs;   /* Number of digits to convert */
          int ndecimal;         /* Digits after decimal (for 'f' format), 0 if
                                 * no limit */

          flags &= ~FL_FLTUPP;

flt_oper:
          if ((flags & FL_PREC) == 0)
            {
              prec = 6;
            }

          vsprintf_float_format(c, prec, &flags, &ndigs, &ndecimal);

#ifdef CONFIG_LIBC_NUMBERED_ARGS
          if ((flags & FL_ARGNUMBER) != 0)
            {
              value = arglist->value[argnumber - 1].d;
            }
          else
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
          if (FL_IS_LDOUBLE(flags))
            {
              /* The output has the precision of a double */

              value = vsprintf_ldtod(va_arg(ap, long double));
            }
          else
#endif
            {
              value = va_arg(ap, double);
            }

          /* %a has no output yet */

          if (c == 'a')
            {
              pnt  = "?";
              size = 1;
              total_len += vsprintf_put_field(stream, pnt, size, &width,
                                              flags);
              goto tail;
            }

          ndigs = __dtoa_engine(value, &_dtoa, ndigs,
                                ndecimal);
          exp = _dtoa.exp;

          sign = 0;
          if (_dtoa.flags & DTOA_MINUS)
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
              total_len += vsprintf_put_naninf(stream, flags, _dtoa.flags,
                                               sign, &width);

              goto tail;
            }

          vsprintf_adjust_float_general(&flags, &prec, &ndigs, exp, &_dtoa);

          /* Conversion result length, width := free space length */

          flen = vsprintf_float_length(flags, sign, exp, prec);

          if (flen > INT_MAX)
            {
              total_len = INT_MAX + 1u;
              goto ret;
            }

          total_len += vsprintf_put_float_prefix(stream, flags, sign,
                                                 &width, flen);

          if ((flags & FL_FLTFIX) != 0)
            {
              /* 'f' format */

              total_len += vsprintf_put_float_fixed(stream, flags, &_dtoa,
                                                    exp, ndigs, prec);
            }
          else
            {
              /* 'e(E)' format
               */

              total_len += vsprintf_put_float_exp(stream, flags, &_dtoa,
                                                  exp, ndigs, prec, buf);
            }

          goto tail;
        }

#else /* !CONFIG_LIBC_FLOATINGPOINT */
      if (fmt_isfloat(c))
        {
#  ifdef CONFIG_LIBC_NUMBERED_ARGS
          if ((flags & FL_ARGNUMBER) == 0)
#  endif
            {
#  ifdef CONFIG_HAVE_LONG_DOUBLE
              if (FL_IS_LDOUBLE(flags))
                {
                  va_arg(ap, long double);
                }
              else
#  endif
                {
                  va_arg(ap, double);
                }
            }

          pnt  = "?";
          size = 1;
          total_len += vsprintf_put_field(stream, pnt, size, &width,
                                          flags);
          goto tail;
        }
#endif

      switch (c)
        {
          case 'c':
#ifdef CONFIG_LIBC_NUMBERED_ARGS
            if ((flags & FL_ARGNUMBER) != 0)
              {
                result = vsprintf_put_char(stream, flags,
                                           arglist->value[argnumber - 1].u,
                                           &width, buf);
              }
            else
#endif
              {
                result = vsprintf_put_char(stream, flags, va_arg(ap, int),
                                           &width, buf);
              }

            if (result < 0)
              {
                return -1;
              }

            total_len += result;
            goto tail;

          case 'n':
          case 's':
          case 'S':
#ifdef CONFIG_LIBC_NUMBERED_ARGS
            if ((flags & FL_ARGNUMBER) != 0)
              {
                pnt = arglist->value[argnumber - 1].cp;
              }
            else
#endif
              {
                pnt = va_arg(ap, FAR char *);
              }

            /* %n only takes its pointer: a format never writes to memory */

            if (c == 'n')
              {
                pnt  = "?";
                size = 1;
                total_len += vsprintf_put_field(stream, pnt, size, &width,
                                                flags);
                goto tail;
              }

            if (pnt == NULL)
              {
                pnt = g_nullstring;
              }
#ifdef CONFIG_LIBC_LOCALE
            else if (c == 's' && (flags & FL_LONG) != 0)
              {
                result = vsprintf_put_wstring(stream,
                                              (FAR const wchar_t *)pnt,
                                              flags, &width, prec);
                if (result < 0)
                  {
                    return -1;
                  }

                total_len += result;
                goto tail;
              }
#endif

            if ((flags & FL_PREC) != 0)
              {
                size = strnlen(pnt, prec);
              }
            else
              {
                size = strlen(pnt);
              }

            total_len += vsprintf_put_field(stream, pnt, size, &width,
                                            flags);
            size = 0;

            goto tail;
        }

      if (c == 'd' || c == 'i')
        {
          long long x;

          if ((flags & FL_LONG) != 0 && (flags & FL_REPD_TYPE) != 0)
            {
#ifdef CONFIG_LIBC_NUMBERED_ARGS
              if ((flags & FL_ARGNUMBER) != 0)
                {
                  x = (long long)arglist->value[argnumber - 1].ull;
                }
              else
#endif
                {
                  x = va_arg(ap, long long);
                }
            }
          else
          if ((flags & FL_LONG) != 0)
            {
#ifdef CONFIG_LIBC_NUMBERED_ARGS
              if ((flags & FL_ARGNUMBER) != 0)
                {
                  x = (long)arglist->value[argnumber - 1].ul;
                }
              else
#endif
                {
                  x = va_arg(ap, long);
                }
            }
          else
            {
#ifdef CONFIG_LIBC_NUMBERED_ARGS
              if ((flags & FL_ARGNUMBER) != 0)
                {
                  x = (int)arglist->value[argnumber - 1].u;
                }
              else
#endif
                {
                  x = va_arg(ap, int);
                }

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
              x = -(unsigned long long)x;
              flags |= FL_NEGATIVE;
            }

          if ((flags & FL_PREC) != 0 && prec == 0 && x == 0)
            {
              c = 0;
            }
          else
            {
              c = __ultoa_invert(x, buf, 10) - buf;
            }
        }
      else if (c != 'u' && c != 'o' && c != 'p' && (c | 0x20) != 'x')
        {
          /* An unknown conversion, or the end of the format.  The size of
           * the argument is not known, so read no more arguments.  Print the
           * rest of the format as it is.
           */

          stream_putc('%', stream);
          while (c != '\0')
            {
              stream_putc(c, stream);
              c = fmt_char(fmt);
            }

          goto ret;
        }
      else
        {
          int base;
          unsigned long long x;

          if ((flags & FL_LONG) != 0 && (flags & FL_REPD_TYPE) != 0)
            {
#ifdef CONFIG_LIBC_NUMBERED_ARGS
              if ((flags & FL_ARGNUMBER) != 0)
                {
                  x = arglist->value[argnumber - 1].ull;
                }
              else
#endif
                {
                  x = va_arg(ap, unsigned long long);
                }
            }
          else
          if ((flags & FL_LONG) != 0)
            {
#ifdef CONFIG_LIBC_NUMBERED_ARGS
              if ((flags & FL_ARGNUMBER) != 0)
                {
                  x = arglist->value[argnumber - 1].ul;
                }
              else
#endif
                {
                  x = va_arg(ap, unsigned long);
                }
            }
          else
            {
#ifdef CONFIG_LIBC_NUMBERED_ARGS
              if ((flags & FL_ARGNUMBER) != 0)
                {
                  x = (unsigned int)arglist->value[argnumber - 1].u;
                }
              else
#endif
                {
                  x = va_arg(ap, unsigned int);
                }

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
#ifdef CONFIG_LIBC_PRINT_EXTENSION
                c = fmt_char(fmt);
                switch (c)
                  {
                    case 'B':
                      {
                        FAR struct va_format *vaf = (FAR void *)(uintptr_t)x;

                        total_len += lib_bsprintf(stream, vaf->fmt,
                                                  vaf->va);
                        continue;
                      }

                    case 'V':
                      {
                        FAR struct va_format *vaf = (FAR void *)(uintptr_t)x;
#  ifdef va_copy
                        va_list copy;

                        va_copy(copy, *vaf->va);
                        total_len += lib_vsprintf(stream, vaf->fmt, copy);
                        va_end(copy);
#  else
                        total_len += lib_vsprintf(stream, vaf->fmt,
                                                  *vaf->va);
#  endif
                        continue;
                      }

                    case 'S':
                    case 's':
                      {
#  ifdef CONFIG_ALLSYMS
                        FAR const struct symtab_s *symbol;
                        FAR void *addr = (FAR void *)(uintptr_t)x;
                        size_t symbolsize;

                        symbol = allsyms_findbyvalue(addr, &symbolsize);
                        if (symbol != NULL)
                          {
                            pnt = symbol->sym_name;
                            stream_puts(pnt, strlen(pnt), stream);

                            if (c == 'S')
                              {
                                total_len +=
                                lib_sprintf_internal(stream,
                                      "+%#tx/%#zx",
                                      addr - symbol->sym_value,
                                      symbolsize);
                              }

                            continue;
                          }
#  endif
                        break;
                      }

                    default:
                      fmt_ungetc(fmt);
                      break;
                  }
#endif

                flags |= FL_ALT;

                /* no break */

              case 'x':
                if ((flags & FL_ALT) != 0)
                  {
                    flags |= FL_ALTHEX;
                  }

                base = 16;
                break;

              default: /* 'X' */
                if ((flags & FL_ALT) != 0)
                  {
                    flags |= (FL_ALTHEX | FL_ALTUPP);
                  }

                base = 16 | XTOA_UPPER;
                break;
            }

          if ((flags & FL_PREC) != 0 && prec == 0 && x == 0)
            {
              c = 0;
            }
          else
            {
              c = __ultoa_invert(x, buf, base) - buf;
            }

          flags &= ~FL_NEGATIVE;
        }

      total_len += vsprintf_put_integer(stream, flags, buf, c, &width,
                                        prec);

tail:

      /* Tail is possible.  */

      if (width > 0)
        {
          stream_repeat(' ', width, stream);
        }
    }

ret:
  if (total_len > INT_MAX)
    {
      set_errno(EOVERFLOW);
      return -1;
    }

  return total_len;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int lib_vsprintf(FAR struct lib_outstream_s *stream,
                 FAR const IPTR char *fmt, va_list ap)
{
#ifdef CONFIG_LIBC_NUMBERED_ARGS
  /* We do 2 passes of parsing and fill the arglist between the passes. */

  struct arg_s arglist =
    {
      { 0 }
    };

  int numargs = vsprintf_internal(NULL, &arglist, NL_ARGMAX, fmt, ap);
  union arg_u argvalue[MAX(numargs, 1)];
  int i;

  arglist.value = argvalue;

  for (i = 0; i < numargs; i++)
    {
      switch (arglist.type[i])
        {
          case TYPE_LONG_LONG:
            arglist.value[i].ull = va_arg(ap, unsigned long long);
            break;

          case TYPE_LONG:
            arglist.value[i].ul = va_arg(ap, unsigned long);
            break;

          case TYPE_INT:
            arglist.value[i].u = va_arg(ap, unsigned int);
            break;

#ifdef CONFIG_HAVE_DOUBLE
          case TYPE_DOUBLE:
            arglist.value[i].d = va_arg(ap, double);
            break;
#endif

#ifdef CONFIG_HAVE_LONG_DOUBLE
          case TYPE_LONG_DOUBLE:
#  ifdef CONFIG_LIBC_FLOATINGPOINT
            arglist.value[i].d = vsprintf_ldtod(va_arg(ap, long double));
#  else
            va_arg(ap, long double);
#  endif
            break;
#endif

          case TYPE_CHAR_POINTER:
            arglist.value[i].cp = va_arg(ap, FAR char *);
            break;

          default:

            /* A number was not used, so its size is not known and the next
             * arguments cannot be found.  Use only the arguments before it.
             */

            numargs = i;
            break;
        }
    }

  return vsprintf_internal(stream, &arglist, numargs, fmt, ap);
#else
  return vsprintf_internal(stream, NULL, 0, fmt, ap);
#endif
}

/****************************************************************************
 * Name: lib_sprintf
 ****************************************************************************/

int lib_sprintf(FAR struct lib_outstream_s *stream, FAR const IPTR char *fmt,
                ...)
{
  va_list ap;
  int     n;

  /* Let lib_vsprintf do the real work */

  va_start(ap, fmt);
  n = lib_vsprintf(stream, fmt, ap);
  va_end(ap);
  return n;
}

/****************************************************************************
 * Name: lib_sprintf_internal
 *
 * Description:
 *   This function does not take numbered arguments in printf.
 *   Equivalent to lib_sprintf when CONFIG_LIBC_NUMBERED_ARGS is not enabled
 *
 ****************************************************************************/

int lib_sprintf_internal(FAR struct lib_outstream_s *stream,
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

/****************************************************************************
 * Name: lib_vsprintf_internal
 *
 * Description:
 *   This function does not take numbered arguments in printf.
 *   Equivalent to lib_sprintf when CONFIG_LIBC_NUMBERED_ARGS is not enabled
 *
 ****************************************************************************/

int lib_vsprintf_internal(FAR struct lib_outstream_s *stream,
                          FAR const IPTR char *fmt, va_list ap)
{
  return vsprintf_internal(stream, NULL, 0, fmt, ap);
}
