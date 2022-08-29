/****************************************************************************
 * libs/libc/stdlib/lib_strtof.c
 * Convert string to float
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *
 * A pretty straight forward conversion fo strtod():
 *
 *   Copyright (C) 2002 Michael Ringgaard. All rights reserved.
 *   Copyright (C) 2006-2007 H. Peter Anvin.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the project nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdlib.h>
#include <ctype.h>
#include <errno.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/* These are predefined with GCC, but could be issues for other compilers. If
 * not defined, an arbitrary big number is put in for now.  These should be
 * added to nuttx/compiler for your compiler.
 */

#if !defined(__FLT_MIN_EXP__) || !defined(__FLT_MAX_EXP__)
#  ifdef CONFIG_CPP_HAVE_WARNING
#    warning "Size of exponent is unknown"
#  endif
#  undef  __FLT_MIN_EXP__
#  define __FLT_MIN_EXP__ (-125)
#  undef  __FLT_MAX_EXP__
#  define __FLT_MAX_EXP__ (128)
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline int is_real(float x)
{
  /* NOTE: Windows MSVC restrictions, MSVC doesn't allow division through a
   * zero literal, but allows it through non-const variable set to zero
   */

  float divzero = 0.0F;
  const float infinite = 1.0F / divzero;
  return (x < infinite) && (x >= -infinite);
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
 *   NOTE: This implementation is limited as compared to POSIX:
 *   - Hexadecimal input is not supported
 *   - INF, INFINITY, NAN, and NAN(...) are not supported
 *
 ****************************************************************************/

float strtof(FAR const char *str, FAR char **endptr)
{
  float number;
  int exponent;
  int negative;
  FAR char *p = (FAR char *) str;
  float p10;
  int n;
  int num_digits;
  int num_decimals;

  /* NOTE: Windows MSVC restrictions, MSVC doesn't allow division through a
   * zero literal, but allows it through non-const variable set to zero
   */

  float divzero = 0.0F;
  const float infinite = 1.0F / divzero;

  /* Skip leading whitespace */

  while (isspace(*p))
    {
      p++;
    }

  /* Handle optional sign */

  negative = 0;
  switch (*p)
    {
    case '-':
      negative = 1; /* Fall through to increment position */

      /* FALLTHROUGH */

    case '+':
      p++;

      /* FALLTHROUGH */

    default:
      break;
    }

  number       = 0.0F;
  exponent     = 0;
  num_digits   = 0;
  num_decimals = 0;

  /* Process string of digits */

  while (isdigit(*p))
    {
      number = number * 10.0F + (float)(*p - '0');
      p++;
      num_digits++;
    }

  /* Process decimal part */

  if (*p == '.')
    {
      p++;

      while (isdigit(*p))
        {
          number = number * 10.0F + (float)(*p - '0');
          p++;
          num_digits++;
          num_decimals++;
        }

      exponent -= num_decimals;
    }

  if (num_digits == 0)
    {
      set_errno(ERANGE);
      number = 0.0F;
      p = (FAR char *)str;
      goto errout;
    }

  /* Correct for sign */

  if (negative)
    {
      number = -number;
    }

  /* Process an exponent string */

  if (*p == 'e' || *p == 'E')
    {
      /* Handle optional sign */

      negative = 0;
      switch (*++p)
        {
        case '-':
          negative = 1;   /* Fall through to increment pos */

          /* FALLTHROUGH */

        case '+':
          p++;

          /* FALLTHROUGH */

        default:
          break;
        }

      /* Process string of digits */

      if (!isdigit(*p))
        {
          set_errno(ERANGE);
          number = 0.0F;
          p = (FAR char *)str;
          goto errout;
        }

      n = 0;
      while (isdigit(*p))
        {
          n = n * 10 + (*p - '0');
          p++;
        }

      if (negative)
        {
          exponent -= n;
        }
      else
        {
          exponent += n;
        }
    }

  if (exponent < __FLT_MIN_EXP__ ||
      exponent > __FLT_MAX_EXP__)
    {
      set_errno(ERANGE);
      number = infinite;
      goto errout;
    }

  /* Scale the result */

  p10 = 10.0F;
  n = exponent;
  if (n < 0)
    {
      n = -n;
    }

  while (n)
    {
      if (n & 1)
        {
          if (exponent < 0)
            {
              number /= p10;
            }
          else
            {
              number *= p10;
            }
        }

      n >>= 1;
      p10 *= p10;
    }

  if (!is_real(number))
    {
      set_errno(ERANGE);
    }

errout:
  if (endptr)
    {
      *endptr = p;
    }

  return number;
}
