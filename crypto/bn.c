/****************************************************************************
 * crypto/bn.c
 * This is free and unencumbered software released into the public domain.
 * Anyone is free to copy, modify, publish, use, compile, sell, or
 * distribute this software, either in source code form or as a compiled
 * binary, for any purpose, commercial or non-commercial, and by any
 * means.
 * In jurisdictions that recognize copyright laws, the author or authors
 * of this software dedicate any and all copyright interest in the
 * software to the public domain. We make this dedication for the benefit
 * of the public at large and to the detriment of our heirs and
 * successors. We intend this dedication to be an overt act of
 * relinquishment in perpetuity of all present and future rights to this
 * software under copyright law.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * For more information, please refer to <https://unlicense.org>
 ****************************************************************************/

/* Big number library - arithmetic on multiple-precision unsigned integers.
 *
 * This library is an implementation of arithmetic on arbitrarily large
 * integers.
 *
 * The difference between this and other implementations, is that the data
 * structure has optimal memory utilization (i.e. a 1024 bit integer takes up
 * 128 bytes RAM), and all memory is allocated statically: no dynamic
 * allocation for better or worse.
 *
 * Primary goals are correctness, clarity of code and clean, portable
 * implementation. Secondary goal is a memory footprint small enough to make
 * it suitable for use in embedded applications.
 *
 *
 * The current state is correct functionality and adequate performance.
 * There may well be room for performance-optimizations and improvements.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdio.h>
#include <stdbool.h>
#include <assert.h>
#include <crypto/bn.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Custom assert macro - easy to disable */

#define require(p, msg) assert(p && msg)

/****************************************************************************
 * Private Functions Prototype
 ****************************************************************************/

/* Functions for shifting number in-place. */

static void lshift_one_bit(FAR struct bn *a);
static void rshift_one_bit(FAR struct bn *a);
static void lshift_word(FAR struct bn *a, int nwords);
static void rshift_word(FAR struct bn *a, int nwords);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Private / Static functions. */

static void rshift_word(FAR struct bn *a, int nwords)
{
  /* Naive method: */

  int i;

  require(a, "a is null");
  require(nwords >= 0, "no negative shifts");

  if (nwords >= BN_ARRAY_SIZE)
    {
      for (i = 0; i < BN_ARRAY_SIZE; ++i)
        {
          a->array[i] = 0;
        }

      return;
    }

  for (i = 0; i < BN_ARRAY_SIZE - nwords; ++i)
    {
      a->array[i] = a->array[i + nwords];
    }

  for (; i < BN_ARRAY_SIZE; ++i)
    {
      a->array[i] = 0;
    }
}

static void lshift_word(FAR struct bn *a, int nwords)
{
  int i;

  require(a, "a is null");
  require(nwords >= 0, "no negative shifts");

  /* Shift whole words */

  for (i = (BN_ARRAY_SIZE - 1); i >= nwords; --i)
    {
      a->array[i] = a->array[i - nwords];
    }

  /* Zero pad shifted words. */

  for (; i >= 0; --i)
    {
      a->array[i] = 0;
    }
}

static void lshift_one_bit(FAR struct bn *a)
{
  int i;

  require(a, "a is null");

  for (i = (BN_ARRAY_SIZE - 1); i > 0; --i)
    {
      a->array[i] = (a->array[i] << 1) |
                    (a->array[i - 1] >> ((8 * WORD_SIZE) - 1));
    }

  a->array[0] <<= 1;
}

static void rshift_one_bit(FAR struct bn *a)
{
  int i;

  require(a, "a is null");

  for (i = 0; i < (BN_ARRAY_SIZE - 1); ++i)
    {
      a->array[i] = (a->array[i] >> 1) |
                    (a->array[i + 1] << ((8 * WORD_SIZE) - 1));
    }

  a->array[BN_ARRAY_SIZE - 1] >>= 1;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Public / Exported functions. */

void bignum_init(FAR struct bn *n)
{
  int i;

  require(n, "n is null");

  for (i = 0; i < BN_ARRAY_SIZE; ++i)
    {
      n->array[i] = 0;
    }
}

void bignum_from_int(FAR struct bn *n, DTYPE_TMP i)
{
  require(n, "n is null");

  bignum_init(n);

  /* Endianness issue if machine is not little-endian? */

#ifdef WORD_SIZE
#  if (WORD_SIZE == 1)
  n->array[0] = (i & 0x000000ff);
  n->array[1] = (i & 0x0000ff00) >> 8;
  n->array[2] = (i & 0x00ff0000) >> 16;
  n->array[3] = (i & 0xff000000) >> 24;
#  elif (WORD_SIZE == 2)
  n->array[0] = (i & 0x0000ffff);
  n->array[1] = (i & 0xffff0000) >> 16;
#  elif (WORD_SIZE == 4)
  n->array[0] = i;
  DTYPE_TMP num_32 = 32;
  DTYPE_TMP tmp = i >> num_32; /* bit-shift with U64 operands to force
                                * 64-bit results */
  n->array[1] = tmp;
#  endif
#endif
}

int bignum_to_int(FAR struct bn *n)
{
  int ret = 0;

  require(n, "n is null");

  /* Endianness issue if machine is not little-endian? */

#if (WORD_SIZE == 1)
  ret += n->array[0];
  ret += n->array[1] << 8;
  ret += n->array[2] << 16;
  ret += n->array[3] << 24;
#elif (WORD_SIZE == 2)
  ret += n->array[0];
  ret += n->array[1] << 16;
#elif (WORD_SIZE == 4)
  ret += n->array[0];
#endif

  return ret;
}

void bignum_from_string(FAR struct bn *n, FAR char *str, int nbytes)
{
  DTYPE tmp;                            /* DTYPE is defined in bn.h -
                                         * uint{8,16,32,64}_t */
  int i = nbytes - (2 * WORD_SIZE);     /* index into string */
  int j = 0;                            /* index into array */

  require(n, "n is null");
  require(str, "str is null");
  require(nbytes > 0, "nbytes must be positive");
  require((nbytes & 1) == 0,
          "string format must be in hex -> equal number of bytes");
  require((nbytes % (sizeof(DTYPE) * 2)) == 0,
  "string length must be a multiple of (sizeof(DTYPE) * 2) characters");

  bignum_init(n);

  /* reading last hex-byte "MSB" from string first -> big endian
   * MSB ~= most significant byte / block ? :)
   */

  while (i >= 0)
    {
      tmp = 0;
      sscanf(&str[i], SSCANF_FORMAT_STR, &tmp);
      n->array[j] = tmp;
      i -= (2 * WORD_SIZE); /* step WORD_SIZE hex-byte(s) back in
                             * the string. */
      j += 1;               /* step one element forward in the
                             * array. */
    }
}

void bignum_to_string(FAR struct bn *n, FAR char *str, int nbytes)
{
  int j = BN_ARRAY_SIZE - 1;    /* index into array - reading "MSB" first
                                 * -> big-endian */
  int i = 0;                    /* index into string representation. */

  require(n, "n is null");
  require(str, "str is null");
  require(nbytes > 0, "nbytes must be positive");
  require((nbytes & 1) == 0,
          "string format must be in hex -> equal number of bytes");

  /* reading last array-element "MSB" first -> big endian */

  while ((j >= 0) && (nbytes > (i + 1)))
    {
      sprintf(&str[i], SPRINTF_FORMAT_STR, n->array[j]);
      i += (2 * WORD_SIZE); /* step WORD_SIZE hex-byte(s) forward in the
                             * string. */
      j -= 1;               /* step one element back in the array. */
    }

  /* Count leading zeros: */

  j = 0;
  while (str[j] == '0')
    {
      j += 1;
    }

  /* Move string j places ahead, effectively skipping leading zeros */

  for (i = 0; i < (nbytes - j); ++i)
    {
      str[i] = str[i + j];
    }

  /* Zero-terminate string */

  str[i] = 0;
}

void bignum_dec(FAR struct bn *n)
{
  DTYPE tmp; /* copy of n */
  DTYPE res;
  int   i;

  require(n, "n is null");

  for (i = 0; i < BN_ARRAY_SIZE; ++i)
    {
      tmp = n->array[i];
      res = tmp - 1;
      n->array[i] = res;

      if (!(res > tmp))
        {
          break;
        }
    }
}

void bignum_inc(FAR struct bn *n)
{
  DTYPE     res;
  DTYPE_TMP tmp; /* copy of n */
  int       i;

  require(n, "n is null");

  for (i = 0; i < BN_ARRAY_SIZE; ++i)
    {
      tmp = n->array[i];
      res = tmp + 1;
      n->array[i] = res;

      if (res > tmp)
        {
          break;
        }
    }
}

void bignum_add(FAR struct bn *a, FAR struct bn *b, FAR struct bn *c)
{
  DTYPE_TMP tmp;
  int carry = 0;
  int i;

  require(a, "a is null");
  require(b, "b is null");
  require(c, "c is null");

  for (i = 0; i < BN_ARRAY_SIZE; ++i)
    {
      tmp = (DTYPE_TMP)a->array[i] + b->array[i] + carry;
      carry = (tmp > MAX_VAL);
      c->array[i] = (tmp & MAX_VAL);
    }
}

void bignum_sub(FAR struct bn *a, FAR struct bn *b, FAR struct bn *c)
{
  DTYPE_TMP res;
  DTYPE_TMP tmp1;
  DTYPE_TMP tmp2;
  int borrow = 0;
  int i;

  require(a, "a is null");
  require(b, "b is null");
  require(c, "c is null");

  for (i = 0; i < BN_ARRAY_SIZE; ++i)
    {
      tmp1 = (DTYPE_TMP)a->array[i] + (MAX_VAL + 1); /* + number_base */
      tmp2 = (DTYPE_TMP)b->array[i] + borrow;
      res  = (tmp1 - tmp2);

      /* "modulo number_base" == "% (number_base - 1)"
       * if number_base is 2^N
       */

      c->array[i] = (DTYPE)(res & MAX_VAL);
      borrow = (res <= MAX_VAL);
    }
}

void bignum_mul(FAR struct bn *a, FAR struct bn *b, FAR struct bn *c)
{
  struct bn row;
  struct bn tmp;
  int i;
  int j;

  require(a, "a is null");
  require(b, "b is null");
  require(c, "c is null");

  bignum_init(c);

  for (i = 0; i < BN_ARRAY_SIZE; ++i)
    {
      bignum_init(&row);

      for (j = 0; j < BN_ARRAY_SIZE; ++j)
        {
          if (i + j < BN_ARRAY_SIZE)
            {
              bignum_init(&tmp);
              DTYPE_TMP intermediate =
                ((DTYPE_TMP)a->array[i] * (DTYPE_TMP)b->array[j]);
              bignum_from_int(&tmp, intermediate);
              lshift_word(&tmp, i + j);
              bignum_add(&tmp, &row, &row);
            }
        }

      bignum_add(c, &row, c);
    }
}

void bignum_div(FAR struct bn *a, FAR struct bn *b, FAR struct bn *c)
{
  struct bn current;
  struct bn denom;
  struct bn tmp;
  const DTYPE_TMP half_max = 1 + (DTYPE_TMP)(MAX_VAL / 2);
  bool overflow = false;

  require(a, "a is null");
  require(b, "b is null");
  require(c, "c is null");

  bignum_from_int(&current, 1);                 /* int current = 1; */
  bignum_assign(&denom, b);                     /* denom = b */
  bignum_assign(&tmp, a);                       /* tmp   = a */

  while (bignum_cmp(&denom, a) != LARGER)     /* while (denom <= a) { */
    {
      if (denom.array[BN_ARRAY_SIZE - 1] >= half_max)
        {
          overflow = true;
          break;
        }

      lshift_one_bit(&current);                /*   current <<= 1; */
      lshift_one_bit(&denom);                  /*   denom <<= 1; */
    }

  if (!overflow)
    {
      rshift_one_bit(&denom);                  /* denom >>= 1; */
      rshift_one_bit(&current);                /* current >>= 1; */
    }

  bignum_init(c);                               /* int answer = 0; */

  while (!bignum_is_zero(&current))             /* while (current != 0) */
    {
      if (bignum_cmp(&tmp, &denom) != SMALLER)  /*   if (dividend >= denom) */
        {
          bignum_sub(&tmp, &denom, &tmp);       /*     dividend -= denom; */
          bignum_or(c, &current, c);            /*     answer |= current; */
        }

      rshift_one_bit(&current);                /*   current >>= 1; */
      rshift_one_bit(&denom);                  /*   denom >>= 1; */
    }

  /* return answer; */
}

void bignum_lshift(FAR struct bn *a, FAR struct bn *b, int nbits)
{
  const int nbits_pr_word = (WORD_SIZE * 8);
  int nwords = nbits / nbits_pr_word;

  require(a, "a is null");
  require(b, "b is null");
  require(nbits >= 0, "no negative shifts");

  bignum_assign(b, a);

  /* Handle shift in multiples of word-size */

  if (nwords != 0)
    {
      lshift_word(b, nwords);
      nbits -= (nwords * nbits_pr_word);
    }

  if (nbits != 0)
    {
      int i;
      for (i = (BN_ARRAY_SIZE - 1); i > 0; --i)
        {
          b->array[i] = (b->array[i] << nbits) |
                        (b->array[i - 1] >> ((8 * WORD_SIZE) - nbits));
        }

      b->array[i] <<= nbits;
    }
}

void bignum_rshift(FAR struct bn *a, FAR struct bn *b, int nbits)
{
  const int nbits_pr_word = (WORD_SIZE * 8);
  int nwords = nbits / nbits_pr_word;

  require(a, "a is null");
  require(b, "b is null");
  require(nbits >= 0, "no negative shifts");

  bignum_assign(b, a);

  /* Handle shift in multiples of word-size */

  if (nwords != 0)
    {
      rshift_word(b, nwords);
      nbits -= (nwords * nbits_pr_word);
    }

  if (nbits != 0)
    {
      int i;
      for (i = 0; i < (BN_ARRAY_SIZE - 1); ++i)
        {
          b->array[i] = (b->array[i] >> nbits) |
                        (b->array[i + 1] << ((8 * WORD_SIZE) - nbits));
        }

      b->array[i] >>= nbits;
    }
}

void bignum_mod(FAR struct bn *a, FAR struct bn *b, FAR struct bn *c)
{
  /* Take divmod and throw away div part */

  struct bn tmp;

  require(a, "a is null");
  require(b, "b is null");
  require(c, "c is null");

  bignum_divmod(a, b, &tmp, c);
}

void bignum_divmod(FAR struct bn *a, FAR struct bn *b,
                   FAR struct bn *c, FAR struct bn *d)
{
  /*  Puts a%b in d
   *  and a/b in c
   *
   *  mod(a,b) = a - ((a / b) * b)
   *
   *  example:
   *   mod(8, 3) = 8 - ((8 / 3) * 3) = 2
   */

  struct bn tmp;

  require(a, "a is null");
  require(b, "b is null");
  require(c, "c is null");

  /* c = (a / b) */

  bignum_div(a, b, c);

  /* tmp = (c * b) */

  bignum_mul(c, b, &tmp);

  /* c = a - tmp */

  bignum_sub(a, &tmp, d);
}

void bignum_and(FAR struct bn *a, FAR struct bn *b, FAR struct bn *c)
{
  int i;

  require(a, "a is null");
  require(b, "b is null");
  require(c, "c is null");

  for (i = 0; i < BN_ARRAY_SIZE; ++i)
    {
      c->array[i] = (a->array[i] & b->array[i]);
    }
}

void bignum_or(FAR struct bn *a, FAR struct bn *b, FAR struct bn *c)
{
  int i;

  require(a, "a is null");
  require(b, "b is null");
  require(c, "c is null");

  for (i = 0; i < BN_ARRAY_SIZE; ++i)
    {
      c->array[i] = (a->array[i] | b->array[i]);
    }
}

void bignum_xor(FAR struct bn *a, FAR struct bn *b, FAR struct bn *c)
{
  int i;

  require(a, "a is null");
  require(b, "b is null");
  require(c, "c is null");

  for (i = 0; i < BN_ARRAY_SIZE; ++i)
    {
      c->array[i] = (a->array[i] ^ b->array[i]);
    }
}

int bignum_cmp(FAR struct bn *a, FAR struct bn *b)
{
  int i = BN_ARRAY_SIZE;

  require(a, "a is null");
  require(b, "b is null");

  do
    {
      i -= 1; /* Decrement first, to start with last array element */
      if (a->array[i] > b->array[i])
        {
          return LARGER;
        }
      else if (a->array[i] < b->array[i])
        {
          return SMALLER;
        }
    }
  while (i != 0);

  return EQUAL;
}

int bignum_is_zero(FAR struct bn *n)
{
  int i;

  require(n, "n is null");

  for (i = 0; i < BN_ARRAY_SIZE; ++i)
    {
      if (n->array[i])
        {
          return 0;
        }
    }

  return 1;
}

void bignum_pow(FAR struct bn *a, FAR struct bn *b, FAR struct bn *c)
{
  struct bn tmp;

  require(a, "a is null");
  require(b, "b is null");
  require(c, "c is null");

  bignum_init(c);

  if (bignum_cmp(b, c) == EQUAL)
    {
      /* Return 1 when exponent is 0 -- n^0 = 1 */

      bignum_inc(c);
    }
  else
    {
      struct bn bcopy;
      bignum_assign(&bcopy, b);

      /* Copy a -> tmp */

      bignum_assign(&tmp, a);

      bignum_dec(&bcopy);

      /* Begin summing products: */

      while (!bignum_is_zero(&bcopy))
        {
          /* c = tmp * tmp */

          bignum_mul(&tmp, a, c);

          /* Decrement b by one */

          bignum_dec(&bcopy);

          bignum_assign(&tmp, c);
        }

      /* c = tmp */

      bignum_assign(c, &tmp);
    }
}

void bignum_isqrt(FAR struct bn *a, FAR struct bn *b)
{
  struct bn low;
  struct bn high;
  struct bn mid;
  struct bn tmp;

  require(a, "a is null");
  require(b, "b is null");

  bignum_init(&low);
  bignum_assign(&high, a);
  bignum_rshift(&high, &mid, 1);
  bignum_inc(&mid);

  while (bignum_cmp(&high, &low) > 0)
    {
      bignum_mul(&mid, &mid, &tmp);
      if (bignum_cmp(&tmp, a) > 0)
        {
          bignum_assign(&high, &mid);
          bignum_dec(&high);
        }
      else
        {
          bignum_assign(&low, &mid);
        }

      bignum_sub(&high, &low, &mid);
      rshift_one_bit(&mid);
      bignum_add(&low, &mid, &mid);
      bignum_inc(&mid);
    }

  bignum_assign(b, &low);
}

void bignum_assign(FAR struct bn *dst, FAR struct bn *src)
{
  int i;

  require(dst, "dst is null");
  require(src, "src is null");

  for (i = 0; i < BN_ARRAY_SIZE; ++i)
    {
      dst->array[i] = src->array[i];
    }
}

void pow_mod_faster(FAR struct bn *a, FAR struct bn *b,
                    FAR struct bn *n, FAR struct bn *res)
{
  struct bn tmpa;
  struct bn tmpb;
  struct bn tmp;
  bignum_assign(&tmpa, a);
  bignum_assign(&tmpb, b);

  bignum_from_int(res, 1); /* r = 1 */

  while (1)
    {
      if (tmpb.array[0] & 1)            /* if (b % 2) */
        {
          bignum_mul(res, &tmpa, &tmp); /*   r = r * a % m */
          bignum_mod(&tmp, n, res);
        }

      bignum_rshift(&tmpb, &tmp, 1); /* b /= 2 */
      bignum_assign(&tmpb, &tmp);

      if (bignum_is_zero(&tmpb))
        {
          break;
        }

      bignum_mul(&tmpa, &tmpa, &tmp);
      bignum_mod(&tmp, n, &tmpa);
    }
}
