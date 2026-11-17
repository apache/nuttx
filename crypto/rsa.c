/****************************************************************************
 * crypto/rsa.c
 *
 * SPDX-License-Identifier: MIT
 * SPDX-FileCopyrightText: 2016 Thomas Pornin <pornin@bolet.org>
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
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * RSA core operations derived from the "i31" implementation of BearSSL
 * (https://bearssl.org/, commit 7bea48e5e850).  Big integers are arrays
 * of 31-bit words; word 0 holds the encoded bit length.  All arithmetic is
 * constant-time with respect to the values of the operands.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include <nuttx/kmalloc.h>

#include <crypto/rsa.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RSA_MAX_FACTOR_BITS    ((RSA_MAX_BITS + 64) >> 1)

/* Four modulus-sized values are the minimum for the exponentiation.  The
 * work buffers come from the heap: callers include work queue threads
 * whose stacks cannot absorb 2 KB per operation.
 */

#define RSA_PUB_TLEN           (4 * (2 + ((RSA_MAX_BITS + 30) / 31)))

#define RSA_PRIV_U             (2 + ((RSA_MAX_FACTOR_BITS + 30) / 31))
#define RSA_PRIV_TLEN          (8 * RSA_PRIV_U)

#define MUL31(x, y)            ((uint64_t)(x) * (uint64_t)(y))
#define MUL31_LO(x, y)         (((uint32_t)(x) * (uint32_t)(y)) & 0x7fffffff)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Constant-time primitives.  Control words are 0 or 1. */

static inline uint32_t ct_not(uint32_t ctl)
{
  return ctl ^ 1;
}

static inline uint32_t ct_mux(uint32_t ctl, uint32_t x, uint32_t y)
{
  return y ^ (-ctl & (x ^ y));
}

static inline uint32_t ct_eq(uint32_t x, uint32_t y)
{
  uint32_t q = x ^ y;

  return ct_not((q | -q) >> 31);
}

static inline uint32_t ct_neq(uint32_t x, uint32_t y)
{
  uint32_t q = x ^ y;

  return (q | -q) >> 31;
}

static inline uint32_t ct_gt(uint32_t x, uint32_t y)
{
  uint32_t z = y - x;

  return (z ^ ((x ^ y) & (x ^ z))) >> 31;
}

static inline uint32_t ct_ge(uint32_t x, uint32_t y)
{
  return ct_not(ct_gt(y, x));
}

static inline uint32_t ct_lt(uint32_t x, uint32_t y)
{
  return ct_gt(y, x);
}

static inline int32_t ct_cmp(uint32_t x, uint32_t y)
{
  return (int32_t)ct_gt(x, y) | -(int32_t)ct_gt(y, x);
}

static inline uint32_t ct_bit_length(uint32_t x)
{
  uint32_t k;
  uint32_t c;

  k = ct_neq(x, 0);
  c = ct_gt(x, 0xffff);
  x = ct_mux(c, x >> 16, x);
  k += c << 4;
  c = ct_gt(x, 0x00ff);
  x = ct_mux(c, x >> 8, x);
  k += c << 3;
  c = ct_gt(x, 0x000f);
  x = ct_mux(c, x >> 4, x);
  k += c << 2;
  c = ct_gt(x, 0x0003);
  x = ct_mux(c, x >> 2, x);
  k += c << 1;
  k += ct_gt(x, 0x0001);
  return k;
}

static void ct_copy(uint32_t ctl, FAR void *dst, FAR const void *src,
                    size_t len)
{
  FAR const uint8_t *s = src;
  FAR uint8_t *d = dst;

  while (len-- > 0)
    {
      *d = ct_mux(ctl, *s++, *d);
      d++;
    }
}

/* Divide hi:lo by d; hi must be lower than or equal to d. */

static uint32_t ct_divrem(uint32_t hi, uint32_t lo, uint32_t d,
                          FAR uint32_t *r)
{
  uint32_t q = 0;
  uint32_t ctl;
  uint32_t hi2;
  uint32_t lo2;
  uint32_t ch;
  uint32_t cf;
  uint32_t w;
  int k;
  int j;

  ch = ct_eq(hi, d);
  hi = ct_mux(ch, 0, hi);
  for (k = 31; k > 0; k--)
    {
      j = 32 - k;
      w = (hi << j) | (lo >> k);
      ctl = ct_ge(w, d) | (hi >> k);
      hi2 = (w - d) >> j;
      lo2 = lo - (d << k);
      hi = ct_mux(ctl, hi2, hi);
      lo = ct_mux(ctl, lo2, lo);
      q |= ctl << k;
    }

  cf = ct_ge(lo, d) | hi;
  q |= cf;
  *r = ct_mux(cf, lo - d, lo);
  return q;
}

static inline uint32_t ct_div(uint32_t hi, uint32_t lo, uint32_t d)
{
  uint32_t r;

  return ct_divrem(hi, lo, d, &r);
}

static inline uint32_t ct_rem(uint32_t hi, uint32_t lo, uint32_t d)
{
  uint32_t r;

  ct_divrem(hi, lo, d, &r);
  return r;
}

static inline void enc32be(FAR uint8_t *buf, uint32_t x)
{
  buf[0] = (uint8_t)(x >> 24);
  buf[1] = (uint8_t)(x >> 16);
  buf[2] = (uint8_t)(x >> 8);
  buf[3] = (uint8_t)x;
}

/* i31 big integers */

static uint32_t i31_bit_length(FAR const uint32_t *x, size_t xlen)
{
  uint32_t twk = 0;
  uint32_t tw = 0;
  uint32_t c;
  uint32_t w;

  while (xlen-- > 0)
    {
      c = ct_eq(tw, 0);
      w = x[xlen];
      tw = ct_mux(c, w, tw);
      twk = ct_mux(c, (uint32_t)xlen, twk);
    }

  return (twk << 5) + ct_bit_length(tw);
}

static inline void i31_zero(FAR uint32_t *x, uint32_t bit_len)
{
  *x++ = bit_len;
  memset(x, 0, ((bit_len + 31) >> 5) * sizeof(*x));
}

static void i31_decode(FAR uint32_t *x, FAR const uint8_t *buf, size_t len)
{
  uint32_t acc = 0;
  uint32_t b;
  int acc_len = 0;
  size_t u = len;
  size_t v = 1;

  while (u-- > 0)
    {
      b = buf[u];
      acc |= b << acc_len;
      acc_len += 8;
      if (acc_len >= 31)
        {
          x[v++] = acc & 0x7fffffff;
          acc_len -= 31;
          acc = b >> (8 - acc_len);
        }
    }

  if (acc_len != 0)
    {
      x[v++] = acc;
    }

  x[0] = i31_bit_length(x + 1, v - 1);
}

/* Decode buf into x modulo m; returns 1 if the value is lower than m, 0
 * otherwise (x is then set to zero).
 */

static uint32_t i31_decode_mod(FAR uint32_t *x, FAR const uint8_t *buf,
                               size_t len, FAR const uint32_t *m)
{
  uint32_t r = 0;
  uint32_t acc;
  uint32_t xw;
  uint32_t cc;
  uint32_t b;
  size_t mlen;
  size_t tlen;
  size_t u;
  size_t v;
  int acc_len;
  int pass;

  mlen = (m[0] + 31) >> 5;
  tlen = mlen << 2;
  if (tlen < len)
    {
      tlen = len;
    }

  tlen += 4;
  for (pass = 0; pass < 2; pass++)
    {
      v = 1;
      acc = 0;
      acc_len = 0;
      for (u = 0; u < tlen; u++)
        {
          b = u < len ? buf[len - 1 - u] : 0;
          acc |= b << acc_len;
          acc_len += 8;
          if (acc_len >= 31)
            {
              xw = acc & 0x7fffffff;
              acc_len -= 31;
              acc = b >> (8 - acc_len);
              if (v <= mlen)
                {
                  if (pass)
                    {
                      x[v] = r & xw;
                    }
                  else
                    {
                      cc = (uint32_t)ct_cmp(xw, m[v]);
                      r = ct_mux(ct_eq(cc, 0), r, cc);
                    }
                }
              else if (!pass)
                {
                  r = ct_mux(ct_eq(xw, 0), r, 1);
                }

              v++;
            }
        }

      r >>= 1;
      r |= r << 1;
    }

  x[0] = m[0];
  return r & 1;
}

static void i31_encode(FAR uint8_t *dst, size_t len, FAR const uint32_t *x)
{
  FAR uint8_t *buf;
  uint32_t acc = 0;
  uint32_t w;
  uint32_t z;
  int acc_len = 0;
  size_t k = 1;
  size_t xlen;

  xlen = (x[0] + 31) >> 5;
  if (xlen == 0)
    {
      memset(dst, 0, len);
      return;
    }

  buf = dst + len;
  while (len != 0)
    {
      w = k <= xlen ? x[k] : 0;
      k++;
      if (acc_len == 0)
        {
          acc = w;
          acc_len = 31;
        }
      else
        {
          z = acc | (w << acc_len);
          acc_len--;
          acc = w >> (31 - acc_len);
          if (len >= 4)
            {
              buf -= 4;
              len -= 4;
              enc32be(buf, z);
            }
          else
            {
              while (len > 0)
                {
                  *--buf = (uint8_t)z;
                  z >>= 8;
                  len--;
                }

              return;
            }
        }
    }
}

/* Return -(1/x) mod 2^31, or 0 if x is even. */

static uint32_t i31_ninv31(uint32_t x)
{
  uint32_t y;

  y = 2 - x;
  y *= 2 - y * x;
  y *= 2 - y * x;
  y *= 2 - y * x;
  y *= 2 - y * x;
  return ct_mux(x & 1, -y, 0) & 0x7fffffff;
}

static uint32_t i31_add(FAR uint32_t *a, FAR const uint32_t *b, uint32_t ctl)
{
  size_t m = (a[0] + 63) >> 5;
  size_t u;
  uint32_t cc = 0;
  uint32_t naw;
  uint32_t aw;

  for (u = 1; u < m; u++)
    {
      aw = a[u];
      naw = aw + b[u] + cc;
      cc = naw >> 31;
      a[u] = ct_mux(ctl, naw & 0x7fffffff, aw);
    }

  return cc;
}

static uint32_t i31_sub(FAR uint32_t *a, FAR const uint32_t *b, uint32_t ctl)
{
  size_t m = (a[0] + 63) >> 5;
  size_t u;
  uint32_t cc = 0;
  uint32_t naw;
  uint32_t aw;

  for (u = 1; u < m; u++)
    {
      aw = a[u];
      naw = aw - b[u] - cc;
      cc = naw >> 31;
      a[u] = ct_mux(ctl, naw & 0x7fffffff, aw);
    }

  return cc;
}

static void i31_rshift(FAR uint32_t *x, int count)
{
  size_t len = (x[0] + 31) >> 5;
  size_t u;
  uint32_t r;
  uint32_t w;

  if (len == 0)
    {
      return;
    }

  r = x[1] >> count;
  for (u = 2; u <= len; u++)
    {
      w = x[u];
      x[u - 1] = ((w << (31 - count)) | r) & 0x7fffffff;
      r = w >> count;
    }

  x[len] = r;
}

/* Compute x = (x * 2^31 + z) mod m. */

static void i31_muladd_small(FAR uint32_t *x, uint32_t z,
                             FAR const uint32_t *m)
{
  uint32_t m_bitlen = m[0];
  uint32_t under;
  uint32_t over;
  uint32_t nxw;
  uint32_t a0;
  uint32_t a1;
  uint32_t b0;
  uint32_t hi;
  uint32_t tb;
  uint32_t cc;
  uint32_t mw;
  uint32_t zw;
  uint32_t g;
  uint32_t q;
  unsigned mblr;
  uint64_t zl;
  size_t mlen;
  size_t u;

  if (m_bitlen == 0)
    {
      return;
    }

  if (m_bitlen <= 31)
    {
      hi = x[1] >> 1;
      x[1] = ct_rem(hi, (x[1] << 31) | z, m[1]);
      return;
    }

  mlen = (m_bitlen + 31) >> 5;
  mblr = (unsigned)m_bitlen & 31;

  /* Estimate the quotient from the top words; the true quotient is then
   * q - 1, q or q + 1, corrected after the subtraction.
   */

  hi = x[mlen];
  if (mblr == 0)
    {
      a0 = x[mlen];
      memmove(x + 2, x + 1, (mlen - 1) * sizeof(*x));
      x[1] = z;
      a1 = x[mlen];
      b0 = m[mlen];
    }
  else
    {
      a0 = ((x[mlen] << (31 - mblr)) | (x[mlen - 1] >> mblr)) & 0x7fffffff;
      memmove(x + 2, x + 1, (mlen - 1) * sizeof(*x));
      x[1] = z;
      a1 = ((x[mlen] << (31 - mblr)) | (x[mlen - 1] >> mblr)) & 0x7fffffff;
      b0 = ((m[mlen] << (31 - mblr)) | (m[mlen - 1] >> mblr)) & 0x7fffffff;
    }

  g = ct_div(a0 >> 1, a1 | (a0 << 31), b0);
  q = ct_mux(ct_eq(a0, b0), 0x7fffffff, ct_mux(ct_eq(g, 0), 0, g - 1));

  cc = 0;
  tb = 1;
  for (u = 1; u <= mlen; u++)
    {
      mw = m[u];
      zl = MUL31(mw, q) + cc;
      cc = (uint32_t)(zl >> 31);
      zw = (uint32_t)zl & 0x7fffffff;
      nxw = x[u] - zw;
      cc += nxw >> 31;
      nxw &= 0x7fffffff;
      x[u] = nxw;
      tb = ct_mux(ct_eq(nxw, mw), tb, ct_gt(nxw, mw));
    }

  over = ct_gt(cc, hi);
  under = ~over & (tb | ct_lt(cc, hi));
  i31_add(x, m, over);
  i31_sub(x, m, under);
}

static void i31_decode_reduce(FAR uint32_t *x, FAR const uint8_t *buf,
                              size_t len, FAR const uint32_t *m)
{
  uint32_t m_ebitlen = m[0];
  uint32_t m_rbitlen;
  uint32_t acc = 0;
  uint32_t v;
  int acc_len = 0;
  size_t mblen;
  size_t k;

  if (m_ebitlen == 0)
    {
      x[0] = 0;
      return;
    }

  i31_zero(x, m_ebitlen);

  m_rbitlen = m_ebitlen >> 5;
  m_rbitlen = (m_ebitlen & 31) + (m_rbitlen << 5) - m_rbitlen;
  mblen = (m_rbitlen + 7) >> 3;
  k = mblen - 1;
  if (k >= len)
    {
      i31_decode(x, buf, len);
      x[0] = m_ebitlen;
      return;
    }

  i31_decode(x, buf, k);
  x[0] = m_ebitlen;

  while (k < len)
    {
      v = buf[k++];
      if (acc_len >= 23)
        {
          acc_len -= 23;
          acc <<= 8 - acc_len;
          acc |= v >> acc_len;
          i31_muladd_small(x, acc, m);
          acc = v & (0xff >> (8 - acc_len));
        }
      else
        {
          acc = (acc << 8) | v;
          acc_len += 8;
        }
    }

  if (acc_len != 0)
    {
      acc = (acc | (x[1] << acc_len)) & 0x7fffffff;
      i31_rshift(x, 31 - acc_len);
      i31_muladd_small(x, acc, m);
    }
}

static void i31_reduce(FAR uint32_t *x, FAR const uint32_t *a,
                       FAR const uint32_t *m)
{
  uint32_t m_bitlen = m[0];
  uint32_t a_bitlen;
  size_t mlen;
  size_t alen;
  size_t u;

  mlen = (m_bitlen + 31) >> 5;
  x[0] = m_bitlen;
  if (m_bitlen == 0)
    {
      return;
    }

  a_bitlen = a[0];
  alen = (a_bitlen + 31) >> 5;
  if (a_bitlen < m_bitlen)
    {
      memcpy(x + 1, a + 1, alen * sizeof(*a));
      for (u = alen; u < mlen; u++)
        {
          x[u + 1] = 0;
        }

      return;
    }

  memcpy(x + 1, a + 2 + (alen - mlen), (mlen - 1) * sizeof(*a));
  x[mlen] = 0;
  for (u = 1 + alen - mlen; u > 0; u--)
    {
      i31_muladd_small(x, a[u], m);
    }
}

static void i31_to_monty(FAR uint32_t *x, FAR const uint32_t *m)
{
  uint32_t k;

  for (k = (m[0] + 31) >> 5; k > 0; k--)
    {
      i31_muladd_small(x, 0, m);
    }
}

static void i31_from_monty(FAR uint32_t *x, FAR const uint32_t *m,
                           uint32_t m0i)
{
  size_t len = (m[0] + 31) >> 5;
  size_t u;
  size_t v;
  uint64_t cc;
  uint64_t z;
  uint32_t f;

  for (u = 0; u < len; u++)
    {
      f = MUL31_LO(x[1], m0i);
      cc = 0;
      for (v = 0; v < len; v++)
        {
          z = (uint64_t)x[v + 1] + MUL31(f, m[v + 1]) + cc;
          cc = z >> 31;
          if (v != 0)
            {
              x[v] = (uint32_t)z & 0x7fffffff;
            }
        }

      x[len] = (uint32_t)cc;
    }

  i31_sub(x, m, ct_not(i31_sub(x, m, 0)));
}

/* Compute d = x * y / 2^(31 * len) mod m (Montgomery multiplication). */

static void i31_montymul(FAR uint32_t *d, FAR const uint32_t *x,
                         FAR const uint32_t *y, FAR const uint32_t *m,
                         uint32_t m0i)
{
  size_t len = (m[0] + 31) >> 5;
  size_t u;
  size_t v;
  uint32_t dh = 0;
  uint32_t xu;
  uint32_t f;
  uint64_t r;
  uint64_t z;

  i31_zero(d, m[0]);
  for (u = 0; u < len; u++)
    {
      xu = x[u + 1];
      f = MUL31_LO(d[1] + MUL31_LO(xu, y[1]), m0i);
      r = 0;
      for (v = 0; v < len; v++)
        {
          z = (uint64_t)d[v + 1] + MUL31(xu, y[v + 1]) +
              MUL31(f, m[v + 1]) + r;
          r = z >> 31;
          d[v] = (uint32_t)z & 0x7fffffff;
        }

      dh += (uint32_t)r;
      d[len] = dh & 0x7fffffff;
      dh >>= 31;
    }

  d[0] = m[0];
  i31_sub(d, m, ct_neq(dh, 0) | ct_not(i31_sub(d, m, 0)));
}

/* Compute d += a * b (non-modular); d must not overlap a or b. */

static void i31_mulacc(FAR uint32_t *d, FAR const uint32_t *a,
                       FAR const uint32_t *b)
{
  size_t alen = (a[0] + 31) >> 5;
  size_t blen = (b[0] + 31) >> 5;
  size_t u;
  size_t v;
  uint64_t cc;
  uint64_t z;
  uint32_t dl;
  uint32_t dh;
  uint32_t f;

  dl = (a[0] & 31) + (b[0] & 31);
  dh = (a[0] >> 5) + (b[0] >> 5);
  d[0] = (dh << 5) + dl + (~(uint32_t)(dl - 31) >> 31);

  for (u = 0; u < blen; u++)
    {
      f = b[1 + u];
      cc = 0;
      for (v = 0; v < alen; v++)
        {
          z = (uint64_t)d[1 + u + v] + MUL31(f, a[1 + v]) + cc;
          cc = z >> 31;
          d[1 + u + v] = (uint32_t)z & 0x7fffffff;
        }

      d[1 + u + alen] = (uint32_t)cc;
    }
}

/* Compute x = x^e mod m with a fixed-window ladder sized to fit tmp.
 * Returns 0 if tmp cannot hold two modulus-sized temporaries.
 */

static uint32_t i31_modpow_opt(FAR uint32_t *x, FAR const uint8_t *e,
                               size_t elen, FAR const uint32_t *m,
                               uint32_t m0i, FAR uint32_t *tmp,
                               size_t twlen)
{
  FAR uint32_t *base;
  FAR uint32_t *t1;
  FAR uint32_t *t2;
  uint32_t acc = 0;
  uint32_t bits;
  uint32_t mask;
  int acc_len = 0;
  int win_len;
  int i;
  int k;
  size_t mwlen;
  size_t mlen;
  size_t u;
  size_t v;

  mwlen = (m[0] + 63) >> 5;
  mlen = mwlen * sizeof(m[0]);
  mwlen += mwlen & 1;
  t1 = tmp;
  t2 = tmp + mwlen;

  if (twlen < (mwlen << 1))
    {
      return 0;
    }

  for (win_len = 5; win_len > 1; win_len--)
    {
      if ((((uint32_t)1 << win_len) + 1) * mwlen <= twlen)
        {
          break;
        }
    }

  i31_to_monty(x, m);

  if (win_len == 1)
    {
      memcpy(t2, x, mlen);
    }
  else
    {
      memcpy(t2 + mwlen, x, mlen);
      base = t2 + mwlen;
      for (u = 2; u < ((unsigned)1 << win_len); u++)
        {
          i31_montymul(base + mwlen, base, x, m, m0i);
          base += mwlen;
        }
    }

  /* x = 1 in Montgomery representation */

  i31_zero(x, m[0]);
  x[(m[0] + 31) >> 5] = 1;
  i31_muladd_small(x, 0, m);

  while (acc_len > 0 || elen > 0)
    {
      k = win_len;
      if (acc_len < win_len)
        {
          if (elen > 0)
            {
              acc = (acc << 8) | *e++;
              elen--;
              acc_len += 8;
            }
          else
            {
              k = acc_len;
            }
        }

      bits = (acc >> (acc_len - k)) & (((uint32_t)1 << k) - 1);
      acc_len -= k;

      for (i = 0; i < k; i++)
        {
          i31_montymul(t1, x, x, m, m0i);
          memcpy(x, t1, mlen);
        }

      /* Constant-time window lookup */

      if (win_len > 1)
        {
          i31_zero(t2, m[0]);
          base = t2 + mwlen;
          for (u = 1; u < ((uint32_t)1 << k); u++)
            {
              mask = -ct_eq(u, bits);
              for (v = 1; v < mwlen; v++)
                {
                  t2[v] |= mask & base[v];
                }

              base += mwlen;
            }
        }

      i31_montymul(t1, x, t2, m, m0i);
      ct_copy(ct_neq(bits, 0), x, t1, mlen);
    }

  i31_from_monty(x, m, m0i);
  return 1;
}

static FAR const uint8_t *rsa_strip(FAR const uint8_t *buf,
                                    FAR size_t *len)
{
  while (*len > 0 && *buf == 0)
    {
      buf++;
      (*len)--;
    }

  return buf;
}

static inline bool rsa_present(FAR const uint8_t *buf, size_t len)
{
  return buf != NULL && len != 0;
}

/* Compute x = x^exp mod n; this serves both the public operation and the
 * private one when only d is known.
 */

static int rsa_modexp(FAR uint8_t *x, size_t xlen,
                      FAR const uint8_t *n, size_t nlen,
                      FAR const uint8_t *exp, size_t explen)
{
  FAR uint32_t *tmp;
  FAR uint32_t *m;
  FAR uint32_t *a;
  FAR uint32_t *t;
  size_t fwlen;
  uint32_t m0i;
  uint32_t r;
  long z;

  n = rsa_strip(n, &nlen);
  exp = rsa_strip(exp, &explen);
  if (nlen == 0 || nlen > RSA_MAX_BYTES || xlen != nlen ||
      explen == 0)
    {
      return -EINVAL;
    }

  tmp = kmm_malloc((1 + RSA_PUB_TLEN) * sizeof(*tmp));
  if (tmp == NULL)
    {
      return -ENOMEM;
    }

  z = (long)nlen << 3;
  fwlen = 1;
  while (z > 0)
    {
      z -= 31;
      fwlen++;
    }

  fwlen += fwlen & 1;

  m = tmp;
  a = m + fwlen;
  t = m + 2 * fwlen;

  i31_decode(m, n, nlen);
  m0i = i31_ninv31(m[1]);

  /* An even modulus gives m0i == 0, which is rejected here. */

  r = m0i & 1;
  r &= i31_decode_mod(a, x, xlen, m);
  r &= i31_modpow_opt(a, exp, explen, m, m0i, t, RSA_PUB_TLEN - 2 * fwlen);
  i31_encode(x, xlen, a);
  explicit_bzero(tmp, (1 + RSA_PUB_TLEN) * sizeof(*tmp));
  kmm_free(tmp);
  return r ? 0 : -EINVAL;
}

static int rsa_crt(FAR uint8_t *x, size_t xlen,
                   FAR const struct rsa_key_s *key)
{
  size_t plen = key->plen;
  size_t qlen = key->qlen;
  size_t fwlen;
  size_t u;
  FAR const uint8_t *p;
  FAR const uint8_t *q;
  FAR uint32_t *tmp;
  FAR uint32_t *mp;
  FAR uint32_t *mq;
  FAR uint32_t *s1;
  FAR uint32_t *s2;
  FAR uint32_t *t1;
  FAR uint32_t *t2;
  FAR uint32_t *t3;
  uint32_t p0i;
  uint32_t q0i;
  uint32_t wn;
  uint32_t wx;
  uint32_t r;
  long z;

  p = rsa_strip(key->p, &plen);
  q = rsa_strip(key->q, &qlen);
  if (plen == 0 || qlen == 0 || xlen == 0 ||
      xlen > RSA_MAX_BYTES)
    {
      return -EINVAL;
    }

  z = (long)(plen > qlen ? plen : qlen) << 3;
  fwlen = 1;
  while (z > 0)
    {
      z -= 31;
      fwlen++;
    }

  fwlen += fwlen & 1;
  if (6 * fwlen > RSA_PRIV_TLEN)
    {
      return -EINVAL;
    }

  tmp = kmm_malloc((1 + RSA_PRIV_TLEN) * sizeof(*tmp));
  if (tmp == NULL)
    {
      return -ENOMEM;
    }

  /* Rebuild n = p * q and require x < n.  The comparison computes the
   * borrow of x - n, which must be set for a value in range.
   */

  mq = tmp;
  i31_decode(mq, q, qlen);
  t1 = mq + fwlen;
  i31_decode(t1, p, plen);
  t2 = mq + 2 * fwlen;
  i31_zero(t2, mq[0]);
  i31_mulacc(t2, mq, t1);

  t3 = mq + 4 * fwlen;
  i31_encode((FAR uint8_t *)t3, xlen, t2);
  u = xlen;
  r = 0;
  while (u > 0)
    {
      u--;
      wn = ((FAR uint8_t *)t3)[u];
      wx = x[u];
      r = ((wx - (wn + r)) >> 8) & 1;
    }

  mp = mq + 2 * fwlen;
  memmove(mp, t1, fwlen * sizeof(*t1));

  /* s2 = x^dq mod q */

  q0i = i31_ninv31(mq[1]);
  s2 = mq + fwlen;
  i31_decode_reduce(s2, x, xlen, mq);
  r &= i31_modpow_opt(s2, key->dq, key->dqlen, mq, q0i,
                      mq + 3 * fwlen, RSA_PRIV_TLEN - 3 * fwlen);

  /* s1 = x^dp mod p */

  p0i = i31_ninv31(mp[1]);
  s1 = mq + 3 * fwlen;
  i31_decode_reduce(s1, x, xlen, mp);
  r &= i31_modpow_opt(s1, key->dp, key->dplen, mp, p0i,
                      mq + 4 * fwlen, RSA_PRIV_TLEN - 4 * fwlen);

  /* h = (s1 - s2) * qp mod p; p < q is tolerated, so s2 is reduced */

  t1 = mq + 4 * fwlen;
  t2 = mq + 5 * fwlen;
  i31_reduce(t2, s2, mp);
  i31_add(s1, mp, i31_sub(s1, t2, 1));
  i31_to_monty(s1, mp);
  i31_decode_reduce(t1, key->qp, key->qplen, mp);
  i31_montymul(t2, s1, t1, mp, p0i);

  /* x = s2 + q * h */

  t3 = s2;
  i31_mulacc(t3, mq, t2);
  i31_encode(x, xlen, t3);
  explicit_bzero(tmp, (1 + RSA_PRIV_TLEN) * sizeof(*tmp));
  kmm_free(tmp);

  /* Even p or q leaves p0i or q0i at zero. */

  return (p0i & q0i & r) ? 0 : -EINVAL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

size_t rsa_modulus_len(FAR const uint8_t *n, size_t nlen)
{
  if (n == NULL)
    {
      return 0;
    }

  rsa_strip(n, &nlen);
  return nlen;
}

int rsa_public(FAR uint8_t *x, size_t xlen, FAR const struct rsa_key_s *key)
{
  if (x == NULL || key == NULL || !rsa_present(key->n, key->nlen) ||
      !rsa_present(key->e, key->elen))
    {
      return -EINVAL;
    }

  return rsa_modexp(x, xlen, key->n, key->nlen, key->e, key->elen);
}

int rsa_private(FAR uint8_t *x, size_t xlen,
                FAR const struct rsa_key_s *key)
{
  if (x == NULL || key == NULL)
    {
      return -EINVAL;
    }

  if (rsa_present(key->p, key->plen) && rsa_present(key->q, key->qlen) &&
      rsa_present(key->dp, key->dplen) &&
      rsa_present(key->dq, key->dqlen) &&
      rsa_present(key->qp, key->qplen))
    {
      if (rsa_present(key->n, key->nlen) &&
          rsa_modulus_len(key->n, key->nlen) != xlen)
        {
          return -EINVAL;
        }

      return rsa_crt(x, xlen, key);
    }

  if (!rsa_present(key->n, key->nlen) || !rsa_present(key->d, key->dlen))
    {
      return -EINVAL;
    }

  return rsa_modexp(x, xlen, key->n, key->nlen, key->d, key->dlen);
}
