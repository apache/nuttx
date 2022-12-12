/****************************************************************************
 * libs/libc/stdlib/lib_strtod.c
 * Convert string to double
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


#include <stdio.h>
#include <stdint.h>
#include <ctype.h>
#include <stdlib.h>
#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <syslog.h>
extern size_t up_check_stack(void);
#ifdef CONFIG_HAVE_DOUBLE
#  define DBL_B1B_DIG 3
#  define DBL_B1B_MAX 18, 446744073, 709551615
#  define KMAX 2048
#  define MASK (KMAX-1)



#define shgetc(f) *f++
#define shunget(f) f--
#define ifexist(A,B) if(*A) {**A = B;}

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/* These are predefined with GCC, but could be issues for other compilers. If
 * not defined, an arbitrary big number is put in for now.  These should be
 * added to nuttx/compiler for your compiler.
 */

#if !defined(__DBL_MIN_EXP__) || !defined(__DBL_MAX_EXP__)
#  ifdef CONFIG_CPP_HAVE_WARNING
#    warning "Size of exponent is unknown"
#  endif
#  undef  __DBL_MIN_EXP__
#  define __DBL_MIN_EXP__ (-1021)
#  undef  __DBL_MAX_EXP__
#  define __DBL_MAX_EXP__ (1024)
#endif

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
 *   flag:1 , the value of the f will be change ;0 ,not change
 *
 ****************************************************************************/
struct numberx
{
  uint32_t index1;
  uint32_t x[KMAX];
  uint32_t index2;
};
static long long scanexp (char** f , bool flag)
{
  char *s = *f;
  int c;
  long long y = 0;
  int neg = 0;

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
      *f = s;
    }

  return neg ? -y : y;
}

/****************************************************************************
 * Name: decdouble
 *
 * Description:
 *   Convert a decimal string to a double value
 *
 ****************************************************************************/

double decdouble (FAR char *ptr, FAR char ***endptr)
{
  FAR char *f = (FAR char *) ptr;
  

  int i;
  int j;
  int k;
  int a;
  int z;
  int c;
  int rp;
  int e2;
  int lnz;
  int gotdig = 0;
  int  gotrad = 0;
  long long dc = 0;
  long long lrp = 0;
  long long e10 = 0;
  
  int bits = DBL_MANT_DIG;
  struct numberx kindex;
  
  const double zero = 0.;
  const double inf = INFINITY;
  static const int p10s[] =
                     {
                       10, 100, 1000, 10000, 100000,
                       1000000, 10000000, 100000000
                     };

  const uint32_t th[] =
                   {
                      DBL_B1B_MAX
                   };

  int denormal = 0;
  double y;
  double frac = 0;
  double bias = 0;

  kindex.index1 =0x10;
  kindex.index2 =0x10;

  
  j = 0;
  k = 0;
  c = shgetc(f);

  /* Don't let leading zeros consume buffer space */
  
  for (; c == '0'; c = shgetc(f))
    {
      gotdig = 1;
    }
  
  if (c == '.')
    {
      gotrad = 1;
      for (c = shgetc(f); c == '0'; c = shgetc(f))
        {
          gotdig = 1, lrp--;
        }
    }
  
  /* Take the integer and decimal parts and put them in the array(x) */

  kindex.x[0] = 0;
  lnz = 0;
 
  for (; isdigit(c) || c == '.'; c = shgetc(f))
    {
    
      if (c == '.')
        {
          if (gotrad)
            {
              break;
            }

          gotrad = 1;
          rp = dc;
        }
      else if (k < KMAX - 3)
        {
          dc++;
          if (c != '0')
            {
              lnz = dc;
            }

          if (j)
            {
              kindex.x[k] = kindex.x[k] * 10 + c - '0';
            }
          else
            {
              kindex.x[k] = c - '0';
            }

          if (++j == 9)
            {
              k++;
              j = 0;
            }

          gotdig = 1;
        }
      else
        {
          dc++;
          if (c != '0')
            {
              lnz = (KMAX - 4) * 9;
              kindex.x[KMAX - 4] |= 1;
            }
        }
    }
   
  if (!gotrad)
    {
      lrp = dc;
    }

  if (!gotdig)
    {
      errno = EINVAL;
      ifexist(endptr, ptr); 
      return zero;
    }

  /* Take the exponential part */
  syslog(LOG_INFO, "stack used after the . :%d\n", up_check_stack());
  syslog(LOG_INFO,"the number of index1 is 0x%lx\n",kindex.index1);
  syslog(LOG_INFO,"the number of index2 is 0x%lx\n",kindex.index2);
  syslog(LOG_INFO, "stack used after the . :%d\n", up_check_stack());
      syslog(LOG_INFO,"the number of index1 is 0x%lx\n",kindex.index1);
      syslog(LOG_INFO,"the number of index2 is 0x%lx\n",kindex.index2);
  if (gotdig && (c | 32) == 'e')
    {
      syslog(LOG_INFO, "stack used after the . :%d\n", up_check_stack());
      syslog(LOG_INFO,"the number of index1 is 0x%lx\n",kindex.index1);
      syslog(LOG_INFO,"the number of index2 is 0x%lx\n",kindex.index2);
      e10 = scanexp(&f, 1);
      syslog(LOG_INFO," into decdouble get exp\n");
      syslog(LOG_INFO, "stack used in get f the exp:%d\n", up_check_stack());
      syslog(LOG_INFO,"the number of index1 is 0x%lx\n",kindex.index1);
      syslog(LOG_INFO,"the number of index2 is 0x%lx\n",kindex.index2);
      if (e10 <= LLONG_MIN / 100)
        {
          ifexist(endptr, ptr);
          return zero;
        }

      lrp += e10;
    }
  else if(c >= 0)
    {
      shunget(f);
    }

  if (!gotdig)
    {
      ifexist(endptr, ptr);
      return zero;
    }

  ifexist(endptr, f);
  syslog(LOG_INFO," into decdouble get f\n");
  syslog(LOG_INFO, "stack used in get f the for:%d\n", up_check_stack());
  syslog(LOG_INFO,"the number of index1 is 0x%lx\n",kindex.index1);
  syslog(LOG_INFO,"the number of index2 is 0x%lx\n",kindex.index2);
  if (!kindex.x[0])
    {
      return zero;
    }

  if (lrp == dc && dc < 10 && (bits > 30 || kindex.x[0] >> bits == 0))
    {
      return (double)kindex.x[0];
    }
  else if (lrp > __DBL_MAX_EXP__ / 2)
    {
      errno = ERANGE;
      return inf;
    }
  else if (lrp < __DBL_MIN_EXP__ - 2 * DBL_MANT_DIG)
    {
      errno = ERANGE;
      return zero;
    }

  /* Align incomplete final B1B digit */

  if (j)
    {
      for (; j < 9; j++)
        {
          kindex.x[k] *= 10;
        }

      k++;
      j = 0;
    }

  a  = 0;
  z  = k;
  e2 = 0;
  rp = lrp;

  /* Optimize small to mid-size integers (even in exp. notation) */

  if (lnz < 9 && lnz <= rp && rp < 18 && rp >= 0)
    {
      if (rp == 9)
        {
          return (double)kindex.x[0];
        }
      else if (rp < 9 && rp >= 0)
        {
          return (double)kindex.x[0] / p10s[8 - rp];
        }

      int bitlim = bits - 3 * (int)(rp - 9);
      if (bitlim > 30 || kindex.x[0] >> bitlim == 0)
        {
          return (double)kindex.x[0] * p10s[rp - 10];
        }
    }

  /* Drop trailing zeros */
syslog(LOG_INFO," into decdouble the middle\n");
syslog(LOG_INFO, "stack used in the middle the for:%d\n", up_check_stack());
syslog(LOG_INFO,"the number of index1 is 0x%lx\n",kindex.index1);
syslog(LOG_INFO,"the number of index2 is 0x%lx\n",kindex.index2);
  for (; !kindex.x[z - 1]; z--);

  /* Align radix point to B1B digit boundary */

  if (rp % 9)
    {
      int rpm9 = rp >= 0 ? rp % 9 : rp % 9 + 9;
      int p10 = p10s[8 - rpm9];
      uint32_t carry = 0;

      for (k = a; k != z; k++)
        {
          uint32_t tmp = kindex.x[k] % p10;
          kindex.x[k] = kindex.x[k] / p10 + carry;
          carry = 1000000000 / p10 * tmp;

          if (k == a && !kindex.x[k])
            {
              a = ((a + 1) & MASK);
              rp -= 9;
            }
        }

      if (carry)
        {
          kindex.x[z++] = carry;
        }

      rp += 9 - rpm9;
    }

  /* Upscale until desired number of bits are left of radix point */
syslog(LOG_INFO,"\n");
syslog(LOG_INFO," into decdouble before the for\n");
syslog(LOG_INFO, "stack used in the middle the for:%d\n", up_check_stack());
syslog(LOG_INFO,"the number of index1 is 0x%lx\n",kindex.index1);
syslog(LOG_INFO,"the number of index2 is 0x%lx\n",kindex.index2);
  while (rp < 9 * DBL_B1B_DIG || (rp == 9 * DBL_B1B_DIG && kindex.x[a] < th[0]))
    {
      uint32_t carry = 0;
      e2 -= 29;

      for (k = ((z - 1) & MASK); ; k = ((k - 1) & MASK))
        {
          uint64_t tmp = ((uint64_t)kindex.x[k] << 29) + carry;

          if (tmp > 1000000000)
            {
              carry = tmp / 1000000000;
              kindex.x[k] = tmp % 1000000000;
            }
          else
            {
              carry = 0;
              kindex.x[k] = tmp;
            }

          if (k == ((z - 1) & MASK) && k != a && !kindex.x[k])
            {
              z = k;
            }

          if (k == a)
            {
              break;
            }
        }

      if (carry)
        {
          rp += 9;
          a = ((a - 1) & MASK);
          if (a == z)
            {
              z = ((z - 1) & MASK);
              kindex.x[(z - 1) & MASK] |= kindex.x[z];
            }

          kindex.x[a] = carry;
        }
    }
syslog(LOG_INFO,"\n");
syslog(LOG_INFO," into decdouble end the while\n");
syslog(LOG_INFO, "stack used in the middle the for:%d\n", up_check_stack());
syslog(LOG_INFO,"the number of index1 is 0x%lx\n",kindex.index1);
syslog(LOG_INFO,"the number of index2 is 0x%lx\n",kindex.index2);
  /* Downscale until exactly number of bits are left of radix point */
  for (; ; )
    {
      uint32_t carry = 0;
      int sh = 1;
      
      for (i = 0; i < DBL_B1B_DIG; i++)
        {
          k = ((a + i) & MASK);

          if (k == z || kindex.x[k] < th[i])
            {
              i = DBL_B1B_DIG;
              break;
            }

          if (kindex.x[(a + i) & MASK] > th[i]) break;
        }

      if (i == DBL_B1B_DIG && rp == 9 * DBL_B1B_DIG)
        {
          break;
        }

      /* FIXME: find a way to compute optimal sh */

      if (rp > 9 + 9 * DBL_B1B_DIG)
        {
          sh = 9;
        }

      e2 += sh;
      for (k = a; k != z; k = ((k + 1) & MASK))
        {
          uint32_t tmp = kindex.x[k] & ((1 << sh) - 1);
          kindex.x[k] = (kindex.x[k] >> sh) + carry;
          carry = (1000000000 >> sh) * tmp;

          if (k == a && !kindex.x[k])
            {
              a = ((a + 1) & MASK);
              i--;
              rp -= 9;
            }
        }

      if (carry)
        {
          if (((z + 1) & MASK) != a)
            {
              kindex.x[z] = carry;
              z = ((z + 1) & MASK);
            }
          else kindex.x[(z - 1) & MASK] |= 1;
        }
    }
syslog(LOG_INFO,"\n");
syslog(LOG_INFO," into decdouble end the for\n");
syslog(LOG_INFO, "stack used in the middle the for:%d\n", up_check_stack());
syslog(LOG_INFO,"the number of index1 is 0x%lx\n",kindex.index1);
syslog(LOG_INFO,"the number of index2 is 0x%lx\n",kindex.index2);
  /* Assemble desired bits into floating point variable */

  for (y = i = 0; i < DBL_B1B_DIG; i++)
    {
      if (((a + i) & MASK) == z)
        {
          kindex.x[(z = ((z + 1) & MASK)) - 1] = 0;
        }

      y = 1000000000.0L * y + kindex.x[(a + i) & MASK];
    }

  y *= 1.0f;

  /* Limit precision for denormal results */

  if (bits > DBL_MANT_DIG + e2 - __DBL_MIN_EXP__)
    {
      bits = DBL_MANT_DIG + e2 - __DBL_MIN_EXP__;
      if (bits < 0)
        {
          bits = 0;
        }

      denormal = 1;
    }

  /* Calculate bias term to force rounding, move out lower bits */

  if (bits < DBL_MANT_DIG)
    {
      bias = copysign(scalbn(1, 2 * DBL_MANT_DIG - bits - 1), y);
      frac = fmod(y, scalbn(1, DBL_MANT_DIG - bits));
      y -= frac;
      y += bias;
    }

  /* Process tail of decimal input so it can affect rounding */
  
  if (((a + i) & MASK) != z)
    {
      uint32_t t = kindex.x[(a + i) & MASK];

      if (t < 500000000 && (t || ((a + i + 1) & MASK) != z))
        {
          frac += 0.25 * 1;
        }
      else if (t > 500000000)
        {
          frac += 0.75 * 1;
        }
      else if (t == 500000000)
        {
          if (((a + i + 1) & MASK) == z)
            {
              frac += 0.5 * 1;
            }
          else
            {
              frac += 0.75 * 1;
            }
        }

      if (DBL_MANT_DIG - bits >= 2 && !fmod(frac, 1))
        {
          frac++;
        }
    }

  y += frac;
  y -= bias;

  if (((e2 + DBL_MANT_DIG) & INT_MAX) > __DBL_MAX_EXP__ - 5)
    {
      if (fabsl(y) >= 2 / DBL_EPSILON)
        {
          if (denormal && bits == DBL_MANT_DIG + e2 - __DBL_MIN_EXP__)
            {
              denormal = 0;
            }

          y *= 0.5;
          e2++;
        }

      if (e2 + DBL_MANT_DIG > __DBL_MAX_EXP__ || (denormal && frac))
        {
          errno = ERANGE;
        }
    }
syslog(LOG_INFO,"\n");
syslog(LOG_INFO," go to scalbn\n");
syslog(LOG_INFO, "stack used in the middle the for:%d\n", up_check_stack());
syslog(LOG_INFO,"the number of index1 is 0x%lx\n",kindex.index1);
syslog(LOG_INFO,"the number of index2 is 0x%lx\n",kindex.index2);
  
  y=scalbn(y, e2);
  syslog(LOG_INFO," get out of decdouble \n");
  syslog(LOG_INFO,"the number of index1 is 0x%lx\n",kindex.index1);
  syslog(LOG_INFO,"the number of index2 is 0x%lx\n",kindex.index2);
  syslog(LOG_INFO, "stack used in decdouble:%d\n", up_check_stack());
  return y;
}

/****************************************************************************
 * Name: hexdouble
 *
 * Description:
 *   Convert a hexadecimal string to a double value
 *
 ****************************************************************************/

double hexdouble (FAR char *ptr, FAR char ***endptr)
{
  FAR char *f = (FAR char *)ptr;
  uint32_t x1 = 0;

  const double zero = 0.;
  const double inf = INFINITY;

  double y = 0.;
  int d;
  int c;
  int gottail = 0;
  int gotrad = 0;
  int gotdig = 0;
  long long rp = 0;
  long long dc = 0;
  long long e2 = 0;
  long double scale = 1;

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

  for (; c - '0' < 10U || (c | 32) - 'a' < 6U || c == '.'; c = shgetc(f))
    {
      if (c == '.')
        {
          if (gotrad) break;
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
              x1 = x1 * 16 + d;
            }
          else if (dc < DBL_MANT_DIG / 4 + 1)
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
      ifexist(endptr, ptr);
      return zero;
    }

  if (!gotrad)
    {
      rp = dc;
    }
  while (dc < 8)
    {
      x1 *= 16, dc++;
    }

  if ((c | 32) == 'p')
    {
      e2 = scanexp(&f, 1);
      if (e2 == LLONG_MIN)
        {
          return zero;
        }
    }
  else
    {
      shunget(f);
    }

    ifexist(endptr, f);
    e2 += 4 * rp - 32;

  if (! x1)
    {
      return  zero;
    }

  if (e2 > -__DBL_MIN_EXP__)
    {
      errno = ERANGE;
      return inf;
    }
  else if (e2 < __DBL_MIN_EXP__ - 2 * DBL_MANT_DIG)
    {
      errno = ERANGE;
      return zero;
    }

  while (x1 < 0x80000000)
    {
      if (y >= 0.5)
        {
          x1 += x1+ 1;
          y += y - 1;
        }
      else
        {
          x1 += x1;
          y += y;
        }

      e2--;
    }

    y = x1;

  if (!y) errno = ERANGE;
  return  scalbn(y, e2);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: strtod
 *
 * Description:
 *   Convert a string to a double value
 *
 *   NOTE: This implementation is limited as compared to POSIX:
 *   - INF, INFINITY, NAN, and NAN(...) are not supported
 *
 ****************************************************************************/

double strtod(FAR const char *str, FAR char **endptr)
{
  FAR char *s = (FAR char *) str;
  int negative = 0;

  /* Skip leading whitespace */

  while (isspace(*s))
    {
      s++;
    }
  syslog(LOG_INFO, "stack used1:%d\n", up_check_stack());
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

  /* Process optional 0x prefix */

  if (*s == '0' && *(s + 1) == 'x')
    {
      s += 2;
      return negative? -(hexdouble(s, &endptr)) : hexdouble(s, &endptr);
    }
  else
    {
      syslog(LOG_INFO,"enter decdoble\n");
      syslog(LOG_INFO, "stack used2:%d\n", up_check_stack());
      double a = decdouble(s, &endptr);
       syslog(LOG_INFO, "stack used3:%d\n", up_check_stack());
      return a;
      //return negative? -(decdouble(s, &endptr)) : decdouble(s, &endptr);
    }
}
#endif  /* CONFIG_HAVE_DOUBLE */
