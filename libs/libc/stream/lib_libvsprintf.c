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
#ifdef CONFIG_LIBC_PRINTF_HEXFLOAT
#  include "lib_hexfloat.h"
#endif
#include "lib_ultoa_invert.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The union of the arguments holds a long double only if a path needs it */

#if defined(HAVE_HEXFLOAT_LDOUBLE) || defined(HAVE_DTOA_LDOUBLE)
#  define HAVE_ARG_LDOUBLE
#endif

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
#ifdef CONFIG_LIBC_PRINTF_LEFT_JUSTIFY
#  define FL_LPAD          0x0008
#else
#  define FL_LPAD          0      /* The compiler removes its code */
#endif
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

/* %b and %B: only an integer conversion has this bit.  A flag that is 0 lets
 * the compiler remove the code of its feature.
 */

#ifdef CONFIG_LIBC_PRINTF_C23
#  define FL_ALTBIN        0x8000
#  define fmt_isbinary(c)  (((c) | 0x20) == 'b')
#else
#  define FL_ALTBIN        0
#  define fmt_isbinary(c)  0
#endif

/* The # flag.  FL_ALT stays for the prefix of %p.  A flag that is 0 lets the
 * compiler remove the code of its feature.
 */

#ifdef CONFIG_LIBC_PRINTF_ALT_FORM
#  define FL_ALT_FORM      FL_ALT
#  define FL_POINTER       FL_ALT
#else
#  define FL_ALT_FORM      0
#  define FL_POINTER       (FL_ALT | FL_ALTHEX)
#endif

#ifdef CONFIG_LIBC_PRINTF_OCTAL
#  define HAVE_OCTAL_OUTPUT 1
#else
#  define HAVE_OCTAL_OUTPUT 0
#endif

/* Only %x, %X and %p have FL_ALT: then FL_ALTHEX needs no test */

#define FL_ALT_IS_HEX(f)   (FL_ALT_FORM == 0 || !HAVE_OCTAL_OUTPUT || \
                            ((f) & FL_ALTHEX) != 0)

/* An integer conversion with a feature that is off prints a question mark.
 * A flag that is 0 lets the compiler remove the code of its feature.
 */

#ifdef CONFIG_LIBC_PRINTF_INT_PRECISION
#  define FL_INT_PREC          FL_PREC
#  define FL_INT_PREC_UNAVAIL  0
#else
#  define FL_INT_PREC          0
#  define FL_INT_PREC_UNAVAIL  FL_PREC
#endif

#ifdef CONFIG_LIBC_PRINTF_SHORT
#  define FL_INT_SHORT         FL_SHORT
#  define FL_INT_SHORT_UNAVAIL 0
#else
#  define FL_INT_SHORT         0
#  define FL_INT_SHORT_UNAVAIL FL_SHORT
#endif

#define FL_INT_UNAVAIL       (FL_INT_PREC_UNAVAIL | FL_INT_SHORT_UNAVAIL)

/* Where long has the size of int, read it as an int: the code is smaller */

#define FL_LONG_READ       (sizeof(long) != sizeof(int) ? FL_LONG : 0)

/* A value longer than long has output only with this option */

#if defined(CONFIG_LIBC_PRINTF_LONG_LONG) || ULLONG_MAX == ULONG_MAX
#  define HAVE_LONG_LONG_OUTPUT 1
#else
#  define HAVE_LONG_LONG_OUTPUT 0
#endif

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

/* Digit i of the conversion.  The exact digits come one after the other */

#ifdef CONFIG_LIBC_PRINTF_EXACT
#  define dtoa_digit(d, i) ((d)->big != NULL ? \
                            __dtoa_big_digit((d)->big, (i)) : (d)->digits[i])
#else
#  define dtoa_digit(d, i) ((d)->digits[i])
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
#ifdef HAVE_ARG_LDOUBLE
  long double ld;
#endif
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
#if defined(CONFIG_LIBC_FLOATINGPOINT) || defined(CONFIG_LIBC_PRINTF_HEXFLOAT)
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

/* Not inline, so that the frame of vsprintf_internal has no digits */

#ifdef CONFIG_LIBC_FLOATINGPOINT
static noinline_function size_t
vsprintf_float(FAR struct lib_outstream_s *stream, double value,
               uint16_t flags, int width, int prec, int c);
#endif

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

#ifdef CONFIG_LIBC_PRINTF_PERCENT_N
/* Store the count of %n at the size of its length modifier */

static void vsprintf_store(FAR void *ptr, uint16_t flags, size_t count)
{
  if (ptr == NULL)
    {
      return;
    }

  if ((flags & FL_LONG) != 0)
    {
      if ((flags & FL_REPD_TYPE) != 0)
        {
          *(FAR long long *)ptr = count;
        }
      else
        {
          *(FAR long *)ptr = count;
        }
    }
  else if ((flags & FL_SHORT) != 0)
    {
      if ((flags & FL_REPD_TYPE) != 0)
        {
          *(FAR signed char *)ptr = count;
        }
      else
        {
          *(FAR short *)ptr = count;
        }
    }
  else
    {
      *(FAR int *)ptr = count;
    }
}
#endif

#ifdef CONFIG_LIBC_FLOATINGPOINT
static size_t vsprintf_putfloat(FAR struct lib_outstream_s *stream,
                                FAR struct dtoa_s *dtoa, unsigned int ndigs,
                                uint16_t flags, int width, int prec)
{
  size_t total_len = 0;
  int exp;              /* Exponent of master decimal digit */
  int n;
  unsigned int flen;    /* Length of the conversion */
  unsigned int len;
  uint8_t sign;         /* Sign character (or 0) */
  char buf[12];

  exp = dtoa->exp;

  sign = 0;
  if (dtoa->flags & DTOA_MINUS)
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

  if (dtoa->flags & (DTOA_NAN | DTOA_INF))
    {
      FAR const char *p;

      ndigs = sign ? 4 : 3;
      if (width > ndigs)
        {
          width -= ndigs;
          if ((flags & FL_LPAD) == 0)
            {
              stream_repeat(' ', width, stream);
              width = 0;
            }
        }
      else
        {
          width = 0;
        }

      if (sign)
        {
          stream_putc(sign, stream);
        }

      if (dtoa->flags & DTOA_NAN)
        {
          p = (flags & FL_FLTUPP) != 0 ? g_nan_upper : g_nan_lower;
        }
      else
        {
          p = (flags & FL_FLTUPP) != 0 ? g_inf_upper : g_inf_lower;
        }

      stream_puts(p, 3, stream);

      goto tail;
    }

  if ((flags & (FL_FLTEXP | FL_FLTFIX)) == 0)
    {
      /* 'g(G)' format: P is the precision, with 0 as 1 */

      if (prec == 0)
        {
          prec = 1;
        }

      if ((flags & FL_ALT_FORM) != 0)
        {
          /* Keep the trailing zeros: P digits in all */

          if (-4 <= exp && exp < prec)
            {
              flags |= FL_FLTFIX;
              if (exp < 0 && prec > INT_MAX + exp + 1)
                {
                  return INT_MAX + 1u;
                }

              prec -= exp + 1;
            }
          else
            {
              prec--;
            }

          goto flt_put;
        }

      /* Remove trailing zeros */

#ifdef CONFIG_LIBC_PRINTF_EXACT
      if (dtoa->big != NULL)
        {
          ndigs = dtoa->big->nz;
        }
      else
#endif
        {
          while (ndigs > 0 && dtoa->digits[ndigs - 1] == '0')
            {
              ndigs--;
            }
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

flt_put:

  /* Conversion result length, width := free space length */

  if ((flags & FL_FLTFIX) != 0)
    {
      flen = (exp > 0 ? exp + 1 : 1);
    }
  else
    {
      flen = 5;           /* 1e+00 */
      if (exp >= 100 || exp <= -100)
        {
          flen++;
        }

#ifdef HAVE_DTOA_LDOUBLE
      if (exp >= 1000 || exp <= -1000)
        {
          flen++;
        }
#endif
    }

  if (sign != 0)
    {
      flen += 1;
    }

  if (prec != 0)
    {
      flen += prec + 1u;
    }
  else if ((flags & FL_ALT_FORM) != 0)
    {
      flen += 1;
    }

  if (flen > INT_MAX)
    {
      return INT_MAX + 1u;
    }

  width = width > flen ? width - flen : 0;

  /* Output before first digit */

  if ((flags & (FL_LPAD | FL_ZFILL)) == 0)
    {
      stream_repeat(' ', width, stream);
      width = 0;
    }

  if (sign != 0)
    {
      stream_putc(sign, stream);
    }

  if ((flags & FL_LPAD) == 0)
    {
      stream_repeat('0', width, stream);
      width = 0;
    }

  if ((flags & FL_FLTFIX) != 0)
    {
      /* 'f' format */

      unsigned int uexp = exp;
      char out;

      /* At this point, we should have exp exponent of leftmost digit
       * in dtoa->digits ndigs number of buffer digits to print prec
       * number of digits after decimal In the loop, 'n' walks over
       * the exponent value
       */

      n = exp > 0 ? exp : 0;    /* Exponent of left digit */
      do
        {
          /* Insert decimal point at correct place */

          if (n == -1)
            {
              stream_putc('.', stream);
            }

          /* Pull digits from buffer when in-range, otherwise use 0 */

          if (uexp - n < ndigs)
            {
              out = dtoa_digit(dtoa, exp - n);
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
                  unsigned int offset = uexp - n;

                  if (next < -prec ||
                      (offset < ndigs &&
                       dtoa_digit(dtoa, offset) != '0'))
                    {
                      break;
                    }

                  zeroes++;
                  n = next;
                }

              stream_repeat('0', zeroes, stream);
            }
          else
            {
              stream_putc(out, stream);
            }
        }
      while (1);

      stream_putc(out, stream);

      if ((flags & FL_ALT_FORM) != 0 && n == -1)
        {
          stream_putc('.', stream);
        }
    }
  else
    {
      /* 'e(E)' format
       *
       * Mantissa
       */

      stream_putc(dtoa_digit(dtoa, 0), stream);
      if (prec > 0)
        {
          int nbytes;

          stream_putc('.', stream);

          nbytes = MIN(prec, ndigs > 1 ? ndigs - 1 : 0);
#ifdef CONFIG_LIBC_PRINTF_EXACT
          if (dtoa->big != NULL)
            {
              int i;

              for (i = 1; i <= nbytes; i++)
                {
                  stream_putc(__dtoa_big_digit(dtoa->big, i), stream);
                }
            }
          else
#endif
          if (nbytes > 0)
            {
              stream_puts(&dtoa->digits[1], nbytes, stream);
            }

          if (prec > nbytes)
            {
              stream_repeat('0', prec - nbytes, stream);
            }
        }
      else if ((flags & FL_ALT_FORM) != 0)
        {
          stream_putc('.', stream);
        }

      /* Exponent */

      stream_putc(flags & FL_FLTUPP ? 'E' : 'e', stream);
      ndigs = '+';
      if (exp < 0)
        {
          exp = -exp;
          ndigs = '-';
        }

      stream_putc(ndigs, stream);
      len = __ultoa_invert(exp, buf, 10) - buf;

      if (exp >= 0 && exp <= 9)
        {
          stream_putc('0', stream);
        }

      total_len += vsprintf_putnum(stream, buf, len);
    }

tail:
  if (FL_LPAD != 0 && width > 0)
    {
      stream_repeat(' ', width, stream);
    }

  return total_len;
}

#ifdef CONFIG_LIBC_PRINTF_EXACT
/* Only a value that the engine cannot prove comes here.  So only these calls
 * have the big numbers on the stack.
 */

static noinline_function size_t
vsprintf_float_big(FAR struct lib_outstream_s *stream, double value,
                   FAR struct dtoa_s *dtoa, uint16_t flags, int width,
                   int prec, int max_digits, int ndecimal)
{
  struct dtoa_big_s big;
  uint16_t limbs[DTOA_BIG_LIMBS];
  int32_t n;

  n = __dtoa_big_start(&big, limbs, value, dtoa, max_digits, ndecimal);
  return vsprintf_putfloat(stream, dtoa, n, flags, width, prec);
}
#endif

#ifdef HAVE_DTOA_LDOUBLE
/* Only %L calls this function, so only these calls have this stack */

static noinline_function size_t
vsprintf_ldouble(FAR struct lib_outstream_s *stream, long double value,
                 uint16_t flags, int width, int prec, int c)
{
  struct dtoa_s dtoa;
  struct dtoa_big_s big;
  uint16_t limbs[DTOA_BIG_LD_LIMBS];
  int32_t n;

  if ((flags & FL_PREC) == 0)
    {
      prec = 6;
    }

  flags &= ~(FL_FLTEXP | FL_FLTFIX);
  flags |= c == 'e' ? FL_FLTEXP : c == 'f' ? FL_FLTFIX : 0;

  n = __dtoa_big_start_ld(&big, limbs, value, &dtoa,
                          c == 'e' && prec < INT_MAX ? prec + 1 :
                          prec == 0 ? 1 : prec, c == 'f' ? prec : -1);
  return vsprintf_putfloat(stream, &dtoa, n, flags, width, prec);
}
#endif

static size_t vsprintf_float(FAR struct lib_outstream_s *stream,
                             double value, uint16_t flags, int width,
                             int prec, int c)
{
  struct dtoa_s dtoa;
  unsigned int ndigs;   /* Number of digits to convert */
  int ndecimal;         /* Digits after the point for %f, -1 for no limit */

  if ((flags & FL_PREC) == 0)
    {
      prec = 6;
    }

  flags &= ~(FL_FLTEXP | FL_FLTFIX);

  ndigs = prec;
  ndecimal = -1;

  if (c == 'e')
    {
      ndigs = prec < INT_MAX ? prec + 1 : prec;
      flags |= FL_FLTEXP;
    }
  else if (c == 'f')
    {
      ndecimal = prec;
      flags |= FL_FLTFIX;
    }
  else if (prec == 0)
    {
      ndigs = 1;
    }

  ndigs = __dtoa_engine(value, &dtoa, ndigs, ndecimal);

#ifdef CONFIG_LIBC_PRINTF_EXACT
  if ((int)ndigs < 0)
    {
      return vsprintf_float_big(stream, value, &dtoa, flags, width, prec,
                                c == 'e' && prec < INT_MAX ? prec + 1 :
                                prec == 0 ? 1 : prec, ndecimal);
    }
#endif

  return vsprintf_putfloat(stream, &dtoa, ndigs, flags, width, prec);
}
#endif

#ifdef CONFIG_LIBC_PRINTF_C23
/* Parse the rest of wN or wfN, and set the flags of the length that has this
 * size.  Return false for any other N, with fmt after the w.
 */

static bool vsprintf_wnlength(FAR const IPTR char **fmtp,
                              FAR uint16_t *flags)
{
  FAR const IPTR char *fmt = *fmtp;
  size_t size;
  int bits = 0;
  bool fast;
  int ch;

  ch = fmt_char(fmt);
  fast = ch == 'f';
  if (fast)
    {
      ch = fmt_char(fmt);
    }

  while (ch >= '0' && ch <= '9' && bits < 100)
    {
      bits = 10 * bits + (ch - '0');
      ch = fmt_char(fmt);
    }

  fmt_ungetc(fmt);

  switch (bits)
    {
      case 8:
        size = fast ? sizeof(int_fast8_t) : sizeof(int8_t);
        break;

      case 16:
        size = fast ? sizeof(int_fast16_t) : sizeof(int16_t);
        break;

      case 32:
        size = fast ? sizeof(int_fast32_t) : sizeof(int32_t);
        break;

      case 64:
        size = sizeof(int64_t);
        break;

      default:
        return false;
    }

  *fmtp = fmt;

  if (size == sizeof(signed char))
    {
      *flags |= FL_SHORT | FL_REPD_TYPE;
    }
  else if (size == sizeof(short) && size != sizeof(int))
    {
      *flags |= FL_SHORT;
    }
  else if (size == sizeof(long) && size != sizeof(int))
    {
      *flags |= FL_LONG;
    }
  else if (size != sizeof(int))
    {
      *flags |= FL_LONG | FL_REPD_TYPE;
    }

  return true;
}
#endif

#ifdef CONFIG_LIBC_PRINTF_HEXFLOAT
static size_t vsprintf_puthex(FAR struct lib_outstream_s *stream,
                              FAR struct hexfloat_s *hex, uint16_t flags,
                              int width, int prec, bool upper)
{
  size_t total_len = 0;
  size_t flen = 3;
  int nzero = 0;
  int exp;
  int nexp = 0;
  bool finite = (hex->flags & (HEXFLOAT_INF | HEXFLOAT_NAN)) == 0;
  char sign = 0;
  char buf[6];

  if ((hex->flags & HEXFLOAT_MINUS) != 0)
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

  if (finite)
    {
      /* 0x1p+0: the digits of the exponent come below */

      if (prec > hex->ndigs)
        {
          nzero = prec - hex->ndigs;
        }

      flen = 5u + (size_t)hex->ndigs + nzero;
      if (hex->ndigs + nzero != 0 || (flags & FL_ALT_FORM) != 0)
        {
          flen++;
        }

      exp = hex->exp < 0 ? -hex->exp : hex->exp;
      nexp = __ultoa_invert(exp, buf, 10) - buf;
      flen += nexp;
    }
  else
    {
      flags &= ~FL_ZFILL;
    }

  if (sign != 0)
    {
      flen++;
    }

  if (flen > INT_MAX)
    {
      return INT_MAX + 1u;
    }

  width = width > flen ? width - flen : 0;

  if ((flags & (FL_LPAD | FL_ZFILL)) == 0)
    {
      stream_repeat(' ', width, stream);
      width = 0;
    }

  if (sign != 0)
    {
      stream_putc(sign, stream);
    }

  if (finite)
    {
      stream_putc('0', stream);
      stream_putc(upper ? 'X' : 'x', stream);
    }

  if ((flags & FL_LPAD) == 0)
    {
      stream_repeat('0', width, stream);
      width = 0;
    }

  if (finite)
    {
      stream_putc(hex->digits[0], stream);
      if (hex->ndigs + nzero != 0 || (flags & FL_ALT_FORM) != 0)
        {
          stream_putc('.', stream);
        }

      if (hex->ndigs != 0)
        {
          stream_puts(&hex->digits[1], hex->ndigs, stream);
        }

      stream_repeat('0', nzero, stream);
      stream_putc(upper ? 'P' : 'p', stream);
      stream_putc(hex->exp < 0 ? '-' : '+', stream);
      total_len += vsprintf_putnum(stream, buf, nexp);
    }
  else if ((hex->flags & HEXFLOAT_NAN) != 0)
    {
      stream_puts(upper ? g_nan_upper : g_nan_lower, 3, stream);
    }
  else
    {
      stream_puts(upper ? g_inf_upper : g_inf_lower, 3, stream);
    }

  if (FL_LPAD != 0 && width > 0)
    {
      stream_repeat(' ', width, stream);
    }

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
#ifdef CONFIG_LIBC_PRINTF_C23
    char __buf[sizeof(ultoa_uint_t) * CHAR_BIT]; /* -1 in binary */
#else
    char __buf[22]; /* Size for -1 in octal, without '\0' */
#endif
#ifdef CONFIG_LIBC_PRINTF_HEXFLOAT
    struct hexfloat_s __hex;
#endif
  } u;

#define buf     (u.__buf)
#define _hex    (u.__hex)

  FAR const char *pnt;
  size_t size;
  int ch;
  int star;
  unsigned int len;
  size_t total_len = 0;

#ifdef CONFIG_LIBC_NUMBERED_ARGS
  int argnumber = 0;
#else
  UNUSED(arglist);
  UNUSED(numargs);
#endif

  while (total_len <= INT_MAX)
    {
#ifndef CONFIG_ARCH_ROMGETC
      FAR const char *start = fmt;

      for (; ; )
        {
          c = fmt_char(fmt);
          if (c == '\0')
            {
#ifdef CONFIG_LIBC_NUMBERED_ARGS
              if (stream != NULL)
#endif
                {
                  size_t litlen = (fmt - start) - 1;

                  if (litlen > 0)
                    {
                      stream_puts(start, litlen, stream);
                    }
                }

              goto ret;
            }

          if (c == '%')
            {
              size_t litlen = (fmt - start) - 1;

              c = fmt_char(fmt);
              if (c != '%')
                {
#ifdef CONFIG_LIBC_NUMBERED_ARGS
                  if (stream != NULL)
#endif
                    {
                      if (litlen > 0)
                        {
                          stream_puts(start, litlen, stream);
                        }
                    }

                  break;
                }

#ifdef CONFIG_LIBC_NUMBERED_ARGS
              if (stream != NULL)
#endif
                {
                  stream_puts(start, litlen + 1, stream);
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
              stream_putc(c, stream);
            }
#else
          stream_putc(c, stream);
#endif
        }
#endif

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
                    flags |= FL_ALT_FORM;
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
                      /* A negative precision means that there is none */

                      if (star < 0)
                        {
                          flags &= ~FL_PREC;
                          star = 0;
                        }

                      prec = star;
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

#ifdef CONFIG_LIBC_PRINTF_C23
          if (c == 'w')
            {
              if (vsprintf_wnlength(&fmt, &flags))
                {
                  continue;
                }

              break; /* An unknown size: the rest of the format is text */
            }
#endif

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
      if ((flags & FL_ARGNUMBER) != 0 && argnumber > numargs)
        {
          goto ret;
        }

      if (stream == NULL)
        {
          unsigned char type; /* We do only parsing */

          if (fmt_isfloat(c))
            {
              type = FL_IS_LDOUBLE(flags) ? TYPE_LONG_DOUBLE : TYPE_DOUBLE;
            }
          else if (c == 'i' || c == 'd' || c == 'u' || c == 'p' ||
                   c == 'o' || c == 'x' || c == 'X' || fmt_isbinary(c))
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
              goto ret; /* The second pass reads no argument after this */
            }

          if ((flags & FL_ARGNUMBER) != 0)
            {
              arglist->type[argnumber - 1] = type;
              if (argnumber > total_len)
                {
                  total_len = argnumber;
                }
            }

          continue;
        }
#endif

#ifdef CONFIG_LIBC_PRINTF_HEXFLOAT
      if ((c | 0x20) == 'a')
        {
          /* A negative precision means none */

          star = (flags & FL_PREC) != 0 ? prec : -1;

#  ifdef CONFIG_LIBC_NUMBERED_ARGS
          if ((flags & FL_ARGNUMBER) != 0)
            {
#    ifdef HAVE_HEXFLOAT_LDOUBLE
              if (FL_IS_LDOUBLE(flags))
                {
                  __hexfloat_engine_ld(arglist->value[argnumber - 1].ld,
                                       &_hex, star, c == 'A');
                }
              else
#    endif
                {
                  __hexfloat_engine(arglist->value[argnumber - 1].d,
                                    &_hex, star, c == 'A');
                }
            }
          else
#  endif
#  ifdef CONFIG_HAVE_LONG_DOUBLE
          if (FL_IS_LDOUBLE(flags))
            {
#    ifdef HAVE_HEXFLOAT_LDOUBLE
              __hexfloat_engine_ld(va_arg(ap, long double), &_hex, star,
                                   c == 'A');
#    else
              __hexfloat_engine(vsprintf_ldtod(va_arg(ap, long double)),
                                &_hex, star, c == 'A');
#    endif
            }
          else
#  endif
            {
              __hexfloat_engine(va_arg(ap, double), &_hex, star, c == 'A');
            }

          total_len += vsprintf_puthex(stream, &_hex, flags, width, star,
                                       c == 'A');
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

          flags &= ~FL_FLTUPP;

flt_oper:
#ifdef HAVE_DTOA_LDOUBLE
          if (FL_IS_LDOUBLE(flags))
            {
              long double ldvalue;

#  ifdef CONFIG_LIBC_NUMBERED_ARGS
              if ((flags & FL_ARGNUMBER) != 0)
                {
                  ldvalue = arglist->value[argnumber - 1].ld;
                }
              else
#  endif
                {
                  ldvalue = va_arg(ap, long double);
                }

              if (c == 'a')
                {
                  goto put_unavailable;
                }

              total_len += vsprintf_ldouble(stream, ldvalue, flags, width,
                                            prec, c);
              continue;
            }
#endif

#ifdef CONFIG_LIBC_NUMBERED_ARGS
          if ((flags & FL_ARGNUMBER) != 0)
            {
#  ifdef HAVE_ARG_LDOUBLE
              if (FL_IS_LDOUBLE(flags))
                {
                  value = vsprintf_ldtod(arglist->value[argnumber - 1].ld);
                }
              else
#  endif
                {
                  value = arglist->value[argnumber - 1].d;
                }
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
put_unavailable:
              pnt  = "?";
              size = 1;
              goto str_lpad;
            }

          total_len += vsprintf_float(stream, value, flags, width, prec, c);
          continue;
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

put_unavailable:
          pnt  = "?";
          size = 1;
          goto str_lpad;
        }
#endif

      switch (c)
        {
          case 'c':
#ifdef CONFIG_LIBC_NUMBERED_ARGS
            if ((flags & FL_ARGNUMBER) != 0)
              {
                ch = (int)arglist->value[argnumber - 1].u;
              }
            else
#endif
              {
                ch = va_arg(ap, int);
              }

#ifdef CONFIG_LIBC_LOCALE
            if ((flags & FL_LONG) != 0)
              {
                mbstate_t ps;

                /* %lc: a wide character, written as its multibyte
                 * sequence.
                 */

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
            goto str_lpad;

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

            /* %n prints nothing.  Only the option lets it store the count:
             * else a format never writes to memory.
             */

            if (c == 'n')
              {
#ifdef CONFIG_LIBC_PRINTF_PERCENT_N
                vsprintf_store((FAR void *)pnt, flags, total_len);
#endif
                continue;
              }

            if (pnt == NULL)
              {
                pnt = g_nullstring;
              }
#ifdef CONFIG_LIBC_LOCALE
            else if (c == 's' && (flags & FL_LONG) != 0)
              {
                FAR const wchar_t *ws = (FAR const wchar_t *)pnt;
                char mb[MB_LEN_MAX];
                mbstate_t ps;
                size_t n;
                int i;

                /* %ls: a wide string, written as multibyte characters.  The
                 * precision limits the bytes, and a character that does not
                 * fit is not written in part.  First count the bytes.
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
                    if (width > 0 && size < width)
                      {
                        int pad = width - size;

                        stream_repeat(' ', pad, stream);
                        width -= pad;
                      }
                  }

                memset(&ps, 0, sizeof(ps));
                for (n = 0, i = 0; n < size; i++)
                  {
                    size_t m = wcrtomb(mb, ws[i], &ps);

                    stream_puts(mb, m, stream);
                    n += m;
                  }

                width = width >= size ? width - size : 0;
                size = 0;
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

str_lpad:
            if ((flags & FL_LPAD) == 0)
              {
                if (width > 0 && size < width)
                  {
                    int pad = width - size;

                    stream_repeat(' ', pad, stream);
                    width -= pad;
                  }
              }

            stream_puts(pnt, size, stream);
            width = width >= size ? width - size : 0;
            size = 0;

            goto tail;
        }

      if (c == 'd' || c == 'i')
        {
          ultoa_int_t x;

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

              if (!HAVE_LONG_LONG_OUTPUT)
                {
                  goto put_unavailable;
                }
            }
          else
          if ((flags & FL_LONG_READ) != 0)
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

              if ((flags & FL_INT_SHORT) != 0)
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

          flags &= ~(FL_NEGATIVE | FL_ALT_FORM);
          if (x < 0)
            {
              x = -(ultoa_uint_t)x;
              flags |= FL_NEGATIVE;
            }

          if ((flags & FL_INT_PREC) != 0 && prec == 0 && x == 0)
            {
              c = 0;
            }
          else
            {
              c = __ultoa_invert(x, buf, 10) - buf;
            }
        }
      else if (c != 'u' && c != 'o' && c != 'p' && (c | 0x20) != 'x' &&
               !fmt_isbinary(c))
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
          ultoa_uint_t x;

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

              if (!HAVE_LONG_LONG_OUTPUT)
                {
                  goto put_unavailable;
                }
            }
          else
          if ((flags & FL_LONG_READ) != 0)
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

              if ((flags & FL_INT_SHORT) != 0)
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
                flags &= ~FL_ALT_FORM;
                base = 10;
                break;

              case 'o':
                if (!HAVE_OCTAL_OUTPUT)
                  {
                    goto put_unavailable;
                  }

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

                flags |= FL_POINTER;

                /* no break */

              case 'x':
                if ((flags & FL_ALT_FORM) != 0)
                  {
                    flags |= FL_ALTHEX;
                  }

                base = 16;
                break;

#ifdef CONFIG_LIBC_PRINTF_C23
              case 'B':
                if ((flags & FL_ALT_FORM) != 0)
                  {
                    flags |= FL_ALTUPP;
                  }

                /* FALLTHROUGH */

              case 'b':
                if ((flags & FL_ALT_FORM) != 0)
                  {
                    flags |= FL_ALTHEX | FL_ALTBIN;
                  }

                base = 2;
                break;
#endif

              default: /* 'X' */
                if ((flags & FL_ALT_FORM) != 0)
                  {
                    flags |= (FL_ALTHEX | FL_ALTUPP);
                  }

                base = 16 | XTOA_UPPER;
                break;
            }

          if ((flags & FL_INT_PREC) != 0 && prec == 0 && x == 0)
            {
              c = 0;
            }
          else
            {
              c = __ultoa_invert(x, buf, base) - buf;
            }

          flags &= ~FL_NEGATIVE;
        }

      if ((flags & FL_INT_UNAVAIL) != 0)
        {
          goto put_unavailable;
        }

      len = c;

      if ((flags & FL_INT_PREC) != 0)
        {
          flags &= ~FL_ZFILL;
          if (len < prec)
            {
              len = prec;
              if ((flags & FL_ALT_FORM) != 0 && !FL_ALT_IS_HEX(flags))
                {
                  flags &= ~FL_ALT;
                }
            }
        }

      if ((flags & FL_ALT) != 0)
        {
          /* c is 0 for a value of 0 with a precision of 0: then %#o has a
           * "0" from the flag, and a hexadecimal value has no prefix.
           */

          if (c == 0 ? FL_ALT_IS_HEX(flags) : buf[c - 1] == '0')
            {
              flags &= ~(FL_ALT | FL_ALTHEX | FL_ALTUPP);
            }
          else
            {
              len += 1;
              if (FL_ALT_IS_HEX(flags))
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
          total_len = INT_MAX + 1u;
          goto ret;
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

          if (len < width)
            {
              stream_repeat(' ', width - len, stream);
              len = width;
            }
        }

      width = (len < width) ? width - len : 0;

      if ((flags & FL_ALT) != 0)
        {
          stream_putc('0', stream);
          if (FL_ALT_IS_HEX(flags))
            {
              ch = (flags & FL_ALTBIN) != 0 ? 'b' : 'x';
              stream_putc(FL_ALT_FORM != 0 && (flags & FL_ALTUPP) != 0 ?
                          ch - 0x20 : ch, stream);
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

          stream_putc(z, stream);
        }

      if (prec > c)
        {
          stream_repeat('0', prec - c, stream);
          prec = c;
        }

      total_len += vsprintf_putnum(stream, buf, c);

tail:

      /* Tail is possible.  */

      if (FL_LPAD != 0 && width > 0)
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
#  ifdef HAVE_ARG_LDOUBLE
            arglist.value[i].ld = va_arg(ap, long double);
#  elif defined(CONFIG_LIBC_FLOATINGPOINT) || \
        defined(CONFIG_LIBC_PRINTF_HEXFLOAT)
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
