/****************************************************************************
 * libs/libc/stdlib/lib_strtold.c
 * Convert string to float and (long) double
 *
 * A pretty straight forward conversion of strtod():
 *
 *   Copyright © 2005-2020 Rich Felker, et al.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdio.h>
#include <stdint.h>
#include <ctype.h>
#include <stdlib.h>
#include <float.h>
#include <stdbool.h>
#include <errno.h>
#include <math.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/* These are predefined with GCC, but could be issues for other compilers. If
 * not defined, an arbitrary big number is put in for now.  These should be
 * added to nuttx/compiler for your compiler.
 */

#ifdef CONFIG_HAVE_LONG_DOUBLE
#  define long_double long double
#  define ldbl_max LDBL_MAX
#  define ldbl_min LDBL_MIN
#  define ldbl_min_exp LDBL_MIN_EXP
#  define ldbl_mant_dig LDBL_MANT_DIG
#  define ldbl_max_10_exp LDBL_MAX_10_EXP
#  define ldbl_min_10_exp LDBL_MIN_10_EXP
#elif defined(CONFIG_HAVE_DOUBLE)
#  define long_double double
#  define ldbl_max DBL_MAX
#  define ldbl_min DBL_MIN
#  define ldbl_min_exp LDBL_MIN_EXP
#  define ldbl_mant_dig DBL_MANT_DIG
#  define ldbl_max_10_exp DBL_MAX_10_EXP
#  define ldbl_min_10_exp DBL_MIN_10_EXP
#else
#  define long_double float
#  define ldbl_max FLT_MAX
#  define ldbl_min FLT_MIN
#  define ldbl_min_exp LDBL_MIN_EXP
#  define ldbl_mant_dig FLT_MANT_DIG
#  define ldbl_max_10_exp FLT_MAX_10_EXP
#  define ldbl_min_10_exp FLT_MIN_10_EXP
#endif

#ifdef CONFIG_HAVE_LONG_LONG
#  define long_long long long
#  define llong_min LLONG_MIN
#else
#  define long_long long
#  define llong_min LONG_MIN
#endif

#define shgetc(f) (*(f)++)
#define shunget(f) ((f)--)
#define ifexist(a,b) do { if ((a) != NULL) {*(a) = (b);} } while (0)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: scanexp
 *
 * Description:
 *   Gets the sum number of a string which the value of
 *   each character between 0 and 9
 *
 * Input Parameters:
 *   f    - The string
 *   flag - 1: The value of the f will be change, 0: not change
 *
 * Returned Value:
 *   The sum number
 *
 ****************************************************************************/

static long_long scanexp(FAR char **f, bool flag)
{
  FAR char *s = *f;
  int c;
  long_long y = 0;
  bool neg = 0;

  c = shgetc(s);

  if (c == '+' || c == '-')
    {
      neg = (c == '-');
      c = shgetc(s);
    }

  while (isdigit(c))
    {
      y = 10 * y + c - '0';
      c = shgetc(s);
    }

  shunget(s);
  if (flag)
    {
      ifexist(f, s);
    }

  return neg ? -y : y;
}

/****************************************************************************
 * Name: ifallzero
 *
 * Description:
 *   To find out if all the next number characters are all zero
 *
 * Input Parameters:
 *   f    - The string
 *   flag - 1:The value of the f will be change , 0: not change
 *
 * Returned Value:
 *   true  - Yes (like: 0000st)
 *   false - No  (like: 0001st)
 *
 ****************************************************************************/

static bool ifallzero(FAR char **f, bool flag)
{
  FAR char *s = *f;
  int c;

  c = shgetc(s);

  while (c == '0')
    {
      c = shgetc(s);
    }

  shunget(s);
  if (flag)
    {
      *f = s;
    }

  return !isdigit(c);
}

/****************************************************************************
 * Name: chtou
 *
 * Description:
 *   Determine whether c is a keyword then set the
 *   value of the number add base
 *
 * Input Parameters:
 *   c      - Hexadecimal: 0 - f(F), decimal: 0 - 9
 *   base   - Hexadecimal(16) decimal(10)
 *   number - IF c is a keyword, number += (the value of c)
 *
 * Returned Value:
 *   true - c is a keyword of hexadecimal or decimal
 *
 ****************************************************************************/

static bool chtou(char c, int base, FAR uint32_t *number)
{
  int tmp = base;

  if (isdigit(c))
    {
      tmp = c - '0';
    }
  else if (c >= 'a' && c <= 'f')
    {
      tmp = c - 'a' + 10;
    }
  else if (c >= 'A' && c <= 'F')
    {
      tmp = c - 'A' + 10;
    }

  if (tmp >= base)
    {
      return false;
    }

  *number = *number * base + tmp;

  return true;
}

/****************************************************************************
 * Name: scalbnx
 *
 * Description:
 *   Implement functionality similar to scalbnl
 *
 * Returned Value:
 *   Get a value of number * base ^ exp
 *
 ****************************************************************************/

static long_double scalbnx(long_double number,
                           long_double base, long_long exp)
{
  long_long e = exp;

  if (e < 0)
    {
      exp *= -1;
    }

  while (exp)
    {
      if (exp & 1)
        {
          if (e < 0)
            {
              number /= base;
            }
          else
            {
              number *= base;
            }
        }

      exp >>= 1;
      base *= base;
    }

  return number;
}

/****************************************************************************
 * Name: decfloat
 *
 * Description:
 *   Convert a decimal string to a long_double value
 *
 * Input Parameters:
 *   ptr    - A decimal string
 *   endptr - If have ,the part that holds all but the numbers
 *
 * Returned Value:
 *   A long_double number about ptr
 *
 ****************************************************************************/

static long_double decfloat(FAR char *ptr, FAR char **endptr)
{
  FAR char *f;
  FAR char *s;

  int c;
  int k;
  int gotrad;
  uint32_t x;
  long_double y;
  long_long num_digit;
  long_long num_decimal;

  const long_double zero = 0.;
  const long p10s[] =
    {
      10, 100, 1000, 10000, 100000,
      1000000, 10000000, 100000000
    };

  f = ptr;
  num_digit = 0;
  num_decimal = 0;

  /* Don't let leading zeros consume buffer space */

  ifallzero(&f, 1);
  c = shgetc(f);

  /* get the digit and decimal */

  for (; isdigit(c); c = shgetc(f))
    {
      num_digit++;
    }

  if (c == '.')
    {
      s = f;

      if (ifallzero(&f, 1))
        {
          c = shgetc(f);
          num_digit++;
        }
      else
        {
          f = s;
          c = shgetc(f);
          for (; isdigit(c); c = shgetc(f))
            {
              num_digit++;
              num_decimal--;
            }
        }
    }

  if (num_digit == 0)
    {
      shunget(f);
      ifexist(endptr, f);
      return zero;
    }

  if ((c | 32) == 'e')
    {
      num_decimal = scanexp(&f, 1) + num_decimal;
      if (num_decimal <= llong_min / 100)
        {
          ifexist(endptr, f);
          return zero;
        }
    }
  else
    {
      shunget(f);
    }

  ifexist(endptr, f);
  f = ptr;

  k = 0;
  x = 0;
  y = 0.;
  gotrad = 0;

  while (chtou(*f, 10, &x) || *f == '.')
    {
      if (*f == '.')
        {
          if (gotrad)
            {
              break;
            }

          c = shgetc(f);
          s = f;
          if (ifallzero(&s, 1))
            {
              f = s;
              break;
            }

          gotrad = 1;
          continue;
        }

      f++;
      if (++k == 9)
        {
          k = 0;
          y = 1000000000 * y + x;
          x = 0;
        }
    }

  if (num_digit < 9 && num_decimal == 0)
    {
      return x;
    }
  else if (num_digit + num_decimal > ldbl_max_10_exp)
    {
      errno = ERANGE;
    }
  else if (num_digit + num_decimal < ldbl_min_10_exp)
    {
      errno = ERANGE;
    }

  if (k % 9)
    {
      y = y * p10s[k % 9 - 1] + x;
    }

  y *= 1.;
  y = scalbnx(y, 10., num_decimal);
  return y;
}

/****************************************************************************
 * Name: hexfloat
 *
 * Description:
 *   Convert a hexadecimal string to a long_double value
 *
 * Input Parameters:
 *   ptr    - The hexadecimal string
 *   endptr - If have ,the part that holds all but the numbers
 *   bits   - LDBL_MANT_DIG
 *   emin   - LDBL_MIN_EXP - bits
 *
 * Returned Value:
 *   A long_double number about ptr
 *
 ****************************************************************************/

static long_double hexfloat(FAR char *ptr,
                            FAR char **endptr, int bits, int emin)
{
  FAR char *f = ptr;
  int d;
  int c;
  int gottail = 0;
  int gotrad  = 0;
  int gotdig  = 0;
  uint32_t x = 0;
  long_double y = 0;
  long_double scale = 1;
  long_double bias = 0;
  long_long rp = 0;
  long_long dc = 0;
  long_long e2 = 0;

  const long_double zero = 0.;

  c = shgetc(f);

  /* Skip leading zeros */

  for (; c == '0'; c = shgetc(f))
    {
      gotdig = 1;
    }

  if (c == '.')
    {
      gotrad = 1;
      c = shgetc(f);

      /* Count zeros after the radix point before significand */

      for (rp = 0; c == '0'; c = shgetc(f), rp--)
        {
          gotdig = 1;
        }
    }

  for (; c - '0' < 10 || (c | 32) - 'a' < 6 || c == '.'; c = shgetc(f))
    {
      if (c == '.')
        {
          if (gotrad)
            {
              break;
            }

          rp = dc;
          gotrad = 1;
        }
      else
        {
          gotdig = 1;
          if (c > '9')
            {
              d = (c | 32) + 10 - 'a';
            }
          else
            {
              d = c - '0';
            }

          if (dc < 8)
            {
              x = x * 16 + d;
            }
          else if (dc < ldbl_mant_dig / 4 + 1)
            {
              y += d * (scale /= 16);
            }
          else if (d && !gottail)
            {
              y += 0.5 * scale;
              gottail = 1;
            }

          dc++;
        }
    }

  if (!gotdig)
    {
      shunget(f);
      shunget(f);
      if (gotrad)
        {
          shunget(f);
        }

      ifexist(endptr, f);
      return zero;
    }

  if (!gotrad)
    {
      rp = dc;
    }

  while (dc < 8)
    {
      x *= 16, dc++;
    }

  if ((c | 32) == 'p')
    {
      e2 = scanexp(&f, 1);
      if (e2 == llong_min)
        {
          shunget(f);
          e2 = 0;
        }
    }
  else
    {
      shunget(f);
    }

  ifexist(endptr, f);
  e2 += 4 * rp - 32;
  if (!x)
    {
      return zero;
    }

  if (e2 > -emin)
    {
      errno = ERANGE;
      return ldbl_max * ldbl_max;
    }

  if (e2 < emin - 2 * ldbl_mant_dig)
    {
      errno = ERANGE;
      return ldbl_min * ldbl_min;
    }

  while (x < 0x80000000)
    {
      if (y >= 0.5)
        {
          x += x + 1;
          y += y - 1;
        }
      else
        {
          x += x;
          y += y;
        }

      e2--;
    }

  if (bits > 32 + e2 - emin)
    {
      bits = 32 + e2 - emin;
      if (bits < 0)
        {
          bits = 0;
        }
    }

  if (bits < ldbl_mant_dig)
    {
      bias = scalbnx(1, 2., 32 + ldbl_mant_dig - bits - 1);
    }

  if (bits < 32 && y && !(x & 1))
    {
      x++, y = 0;
    }

  y = bias + x + y;
  y -= bias;

  /* If it's a wrong number we will set errno and return it */

  if (!y)
    {
      errno = ERANGE;
    }

  return scalbnx(y, 2., e2);
}

/****************************************************************************
 * Name: strtox
 *
 * Description:
 *   Convert a string to a long_double value
 *
 * Input Parameters:
 *   str    - The string
 *   endptr - If have ,the part that holds all but the numbers
 *   flags  - 1: string -> float
 *            2: string -> double
 *            3: string- > long double
 *
 * Returned Value:
 *   A long_double number about str
 *
 ****************************************************************************/

static long_double strtox(FAR const char *str, FAR char **endptr, int flag)
{
  FAR char *s = (FAR char *)str;
  int negative = 0;
  long_double y = 0;
  int i = 0;

  int bits;
  int emin;

  switch (flag)
    {
      case 1:
        bits = FLT_MANT_DIG;
        emin = FLT_MIN_EXP - bits;
        break;
      case 2:
        bits = DBL_MANT_DIG,
        emin = DBL_MIN_EXP - bits;
        break;
      case 3:
        bits = LDBL_MANT_DIG,
        emin = LDBL_MIN_EXP - bits;
      default:
        return 0;
    }

  /* Skip leading whitespace */

  while (isspace(*s))
    {
      s++;
    }

  /* Handle optional sign */

  switch (*s)
    {
  case '-':
      negative = 1;  /* Fall through to increment position */

  case '+':
      s++;

  default:
      break;
    }

  for (i = 0; i < 8 && (*s | 32) == "infinity"[i]; i++)
    {
      s++;
    }

  if (i == 3 || i == 8)
    {
      ifexist(endptr, s);
      return negative ? -INFINITY : INFINITY;
    }

  s -= i;
  for (i = 0; i < 3 && (*s | 32) == "nan"[i]; i++)
    {
      s++;
    }

  if (i == 3)
    {
      ifexist(endptr, s);
      return NAN;
    }

  /* Process optional 0x prefix */

  s -= i;
  if (*s == '0' && (*(s + 1) | 32) == 'x')
    {
      s += 2;
      y = hexfloat(s, endptr, bits, emin);
    }
  else
    {
      y = decfloat(s, endptr);
    }

  return negative ? -y : y;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: strtof
 *
 * Description:
 *   Convert a string to a float value
 *
 * Input Parameters:
 *   str    - The string
 *   endptr - If have ,the part that holds all but the numbers
 *
 * Returned Value:
 *   A float number about str
 *
 ****************************************************************************/

float strtof(FAR const char *str, FAR char **endptr)
{
  return strtox(str, endptr, 1);
}

/****************************************************************************
 * Name: strtod
 *
 * Description:
 *   Convert a string to a double value
 *
 * Input Parameters:
 *   str    - The string
 *   endptr - If have ,the part that holds all but the numbers
 *
 * Returned Value:
 *   A double number about str
 *
 ****************************************************************************/

#ifdef CONFIG_HAVE_DOUBLE

double strtod(FAR const char *str, FAR char **endptr)
{
  return strtox(str, endptr, 2);
}

#endif /* CONFIG_HAVE_DOUBLE */

/****************************************************************************
 * Name: strtold
 *
 * Description:
 *   Convert a string to a long double value
 *
 * Input Parameters:
 *   str    - The string
 *   endptr - If have ,the part that holds all but the numbers
 *
 * Returned Value:
 *   A long double number about str
 *
 ****************************************************************************/

#ifdef CONFIG_HAVE_LONG_DOUBLE

long double strtold(FAR const char *str, FAR char **endptr)
{
  return strtox(str, endptr, 3);
}

#endif /* CONFIG_HAVE_LONG_DOUBLE */