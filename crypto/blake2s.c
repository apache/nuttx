/****************************************************************************
 * crypto/blake2s.c
 *
 * This code is based on public-domain/CC0 BLAKE2 reference implementation
 * by Samual Neves, at https://github.com/BLAKE2/BLAKE2/tree/master/ref
 * Copyright 2012, Samuel Neves <sneves@dei.uc.pt>
 *
 * Copyright (C) 2017 Haltian Ltd. All rights reserved.
 * Authors: Jussi Kivilinna <jussi.kivilinna@haltian.com>
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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <debug.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/crypto/blake2s.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint32_t blake2s_IV[8] =
{
  0x6a09e667ul, 0xbb67ae85ul, 0x3c6ef372ul, 0xa54ff53aul, 0x510e527ful,
  0x9b05688cul, 0x1f83d9abul, 0x5be0cd19ul
};

static const uint8_t blake2s_sigma[10][16] =
{
  { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 },
  { 14, 10, 4, 8, 9, 15, 13, 6, 1, 12, 0, 2, 11, 7, 5, 3 },
  { 11, 8, 12, 0, 5, 2, 15, 13, 10, 14, 3, 6, 7, 1, 9, 4 },
  { 7, 9, 3, 1, 13, 12, 11, 14, 2, 6, 5, 10, 4, 0, 15, 8 },
  { 9, 0, 5, 7, 2, 4, 10, 15, 14, 1, 11, 12, 6, 8, 3, 13 },
  { 2, 12, 6, 10, 0, 11, 8, 3, 4, 13, 7, 5, 15, 14, 1, 9 },
  { 12, 5, 1, 15, 14, 13, 4, 10, 0, 7, 6, 3, 9, 2, 8, 11 },
  { 13, 11, 7, 14, 12, 1, 3, 9, 5, 0, 15, 4, 8, 6, 2, 10 },
  { 6, 15, 14, 9, 11, 3, 0, 8, 12, 2, 13, 7, 1, 4, 10, 5 },
  { 10, 2, 8, 4, 7, 6, 1, 5, 15, 11, 9, 14, 3, 12, 13, 0 }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t rotr32(const uint32_t w, const unsigned int c)
{
  return (w >> (c & 31)) | (w << ((32 - c) & 31));
}

static void blake2_memcpy(FAR void *dst, FAR const void *src, size_t len)
{
#ifdef BLAKE2_UNALIGNED
  FAR uint32_alias_t *idst = dst;
  FAR const uint32_alias_t *isrc = src;
  FAR uint8_t *bdst;
  FAR const uint8_t *bsrc;

  while (len >= sizeof(uint32_alias_t))
    {
      *idst = *isrc;
      idst++;
      isrc++;
      len -= sizeof(uint32_alias_t);
    }

  bdst = (FAR uint8_t *)idst;
  bsrc = (FAR const uint8_t *)isrc;
  while (len)
    {
      *bdst = *bsrc;
      bdst++;
      bsrc++;
      len--;
    }
#else
  memcpy(dst, set, len);
#endif
}

static void blake2_memset(FAR void *dst, int set, size_t len)
{
#ifdef BLAKE2_UNALIGNED
  FAR uint32_alias_t *idst = dst;
  FAR uint8_t *bdst;
  uint32_t mset;

  set &= 0xff;
  mset = (uint32_t)set * 0x01010101UL;

  while (len >= sizeof(uint32_alias_t))
    {
      *idst = mset;
      idst++;
      len -= sizeof(uint32_alias_t);
    }

  bdst = (FAR uint8_t *)idst;
  set &= 0xff;
  while (len)
    {
      *bdst = set;
      bdst++;
      len--;
    }
#else
  memset(dst, set, len);
#endif
}

static inline void secure_zero_memory(FAR void *v, size_t n)
{
  explicit_bzero(v, n);
}

/* Some helper functions, not necessarily useful */

static int blake2s_is_lastblock(FAR const blake2s_state *S)
{
  return S->f[0] != 0;
}

static void blake2s_set_lastblock(FAR blake2s_state *S)
{
  S->f[0] = (uint32_t)-1;
}

static void blake2s_increment_counter(FAR blake2s_state *S, const uint32_t inc)
{
  S->t[0] += inc;
  S->t[1] += (S->t[0] < inc);
}

static void blake2s_init0(FAR blake2s_state *S)
{
  size_t i;

  blake2_memset(S, 0, sizeof(*S) - sizeof(S->buf));

  for (i = 0; i < 8; ++i)
    S->h[i] = blake2s_IV[i];
}

static void blake2s_compress(FAR blake2s_state *S,
                             const uint8_t in[BLAKE2S_BLOCKBYTES])
{
  uint32_t m[16];
  uint32_t v[16];
  size_t i;
  unsigned int round;

  for (i = 0; i < 16; ++i)
    {
      m[i] = blake2_load32(in + i * sizeof(m[i]));
    }

  for (i = 0; i < 8; ++i)
    {
      v[i] = S->h[i];
    }

  v[8] = blake2s_IV[0];
  v[9] = blake2s_IV[1];
  v[10] = blake2s_IV[2];
  v[11] = blake2s_IV[3];
  v[12] = S->t[0] ^ blake2s_IV[4];
  v[13] = S->t[1] ^ blake2s_IV[5];
  v[14] = S->f[0] ^ blake2s_IV[6];
  v[15] = S->f[1] ^ blake2s_IV[7];

#define G(r,i,a,b,c,d)                      \
  do {                                      \
    a = a + b + m[blake2s_sigma[r][2*i+0]]; \
    d = rotr32(d ^ a, 16);                  \
    c = c + d;                              \
    b = rotr32(b ^ c, 12);                  \
    a = a + b + m[blake2s_sigma[r][2*i+1]]; \
    d = rotr32(d ^ a, 8);                   \
    c = c + d;                              \
    b = rotr32(b ^ c, 7);                   \
  } while(0)

#define ROUND(r)                    \
  do {                              \
    G(r,0,v[ 0],v[ 4],v[ 8],v[12]); \
    G(r,1,v[ 1],v[ 5],v[ 9],v[13]); \
    G(r,2,v[ 2],v[ 6],v[10],v[14]); \
    G(r,3,v[ 3],v[ 7],v[11],v[15]); \
    G(r,4,v[ 0],v[ 5],v[10],v[15]); \
    G(r,5,v[ 1],v[ 6],v[11],v[12]); \
    G(r,6,v[ 2],v[ 7],v[ 8],v[13]); \
    G(r,7,v[ 3],v[ 4],v[ 9],v[14]); \
  } while(0)

  /* Size vs performance trade-off. With unrolling, on ARMv7-M function text
   * is ~4 KiB and without ~1 KiB. Without unrolling we take ~25% performance
   * hit. */

#if 1
  /* Smaller, slightly slower. */

  for (round = 0; round < 10; round++)
    {
      ROUND(round);
    }
#else
  /* Larger, slightly faster. */

  (void)(round=0);
  ROUND(0);
  ROUND(1);
  ROUND(2);
  ROUND(3);
  ROUND(4);
  ROUND(5);
  ROUND(6);
  ROUND(7);
  ROUND(8);
  ROUND(9);
#endif

#undef G
#undef ROUND

  for (i = 0; i < 8; ++i)
    {
      S->h[i] = S->h[i] ^ v[i] ^ v[i + 8];
    }
}

#ifdef CONFIG_BLAKE2_SELFTEST
/* BLAKE2s self-test from RFC 7693 */

static void selftest_seq(FAR uint8_t *out, size_t len, uint32_t seed)
{
  size_t i;
  uint32_t t, a, b;

  a = 0xDEAD4BAD * seed; /* prime */
  b = 1;
  /* fill the buf */
  for (i = 0; i < len; i++)
    {
      t = a + b;
      a = b;
      b = t;
      out[i] = (t >> 24) & 0xFF;
    }
}

static int blake2s_selftest(void)
{
  /* Grand hash of hash results. */

  static const uint8_t blake2s_res[32] =
  {
    0x6a, 0x41, 0x1f, 0x08, 0xce, 0x25, 0xad, 0xcd, 0xfb, 0x02, 0xab, 0xa6,
    0x41, 0x45, 0x1c, 0xec, 0x53, 0xc5, 0x98, 0xb2, 0x4f, 0x4f, 0xc7, 0x87,
    0xfb, 0xdc, 0x88, 0x79, 0x7f, 0x4c, 0x1d, 0xfe
  };

  /* Parameter sets. */

  static const size_t b2s_md_len[4] = { 16, 20, 28, 32 };
  static const size_t b2s_in_len[6] = { 0, 3, 64, 65, 255, 1024 };
  size_t i, j, outlen, inlen;
  FAR uint8_t *in;
  uint8_t md[32], key[32];
  blake2s_state ctx;
  int ret = -1;

  in = malloc(1024);
  if (!in)
    {
      goto out;
    }

  /* 256-bit hash for testing. */

  if (blake2s_init(&ctx, 32))
    {
      goto out;
    }

  for (i = 0; i < 4; i++)
    {
      outlen = b2s_md_len[i];
      for (j = 0; j < 6; j++)
        {
          inlen = b2s_in_len[j];

          selftest_seq(in, inlen, inlen);     /* unkeyed hash */
          blake2s(md, outlen, in, inlen, NULL, 0);
          blake2s_update(&ctx, md, outlen);   /* hash the hash */

          selftest_seq(key, outlen, outlen);  /* keyed hash */
          blake2s(md, outlen, in, inlen, key, outlen);
          blake2s_update(&ctx, md, outlen);   /* hash the hash */
        }
    }

  /* Compute and compare the hash of hashes. */

  blake2s_final(&ctx, md, 32);
  for (i = 0; i < 32; i++)
    {
      if (md[i] != blake2s_res[i])
        goto out;
    }

  ret = 0;

out:
  free(in);
  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* init2 xors IV with input parameter block */

int blake2s_init_param(FAR blake2s_state *S, FAR const blake2s_param *P)
{
  FAR const unsigned char *p = (FAR const unsigned char *)(P);
  size_t i;
#ifdef CONFIG_BLAKE2_SELFTEST
  static bool selftest_done = false;
  int ret;

  if (!selftest_done)
    {
      selftest_done = true;
      ret = blake2s_selftest();
      DEBUGASSERT(ret == 0);
      if (ret)
        return -1;
    }
#endif

  blake2s_init0(S);

  /* IV XOR ParamBlock */

  for (i = 0; i < 8; ++i)
    {
      S->h[i] ^= blake2_load32(&p[i * 4]);
    }

  S->outlen = P->digest_length;
  return 0;
}

/* Sequential blake2s initialization */

int blake2s_init(FAR blake2s_state *S, size_t outlen)
{
  blake2s_param P[1];

  /* Move interval verification here? */

  if ((!outlen) || (outlen > BLAKE2S_OUTBYTES))
    {
      return -1;
    }

  P->digest_length = (uint8_t)outlen;
  P->key_length = 0;
  P->fanout = 1;
  P->depth = 1;
  blake2_store32(P->leaf_length, 0);
  blake2_store32(P->node_offset, 0);
  blake2_store16(P->xof_length, 0);
  P->node_depth = 0;
  P->inner_length = 0;
  /* memset(P->reserved, 0, sizeof(P->reserved)); */
  blake2_memset(P->salt, 0, sizeof(P->salt));
  blake2_memset(P->personal, 0, sizeof(P->personal));
  return blake2s_init_param(S, P);
}

int blake2s_init_key(FAR blake2s_state *S, size_t outlen, FAR const void *key,
                     size_t keylen)
{
  blake2s_param P[1];
  uint8_t block[BLAKE2S_BLOCKBYTES];

  if ((!outlen) || (outlen > BLAKE2S_OUTBYTES))
    {
      return -1;
    }

  if (!key || !keylen || keylen > BLAKE2S_KEYBYTES)
    {
      return -1;
    }

  P->digest_length = (uint8_t) outlen;
  P->key_length = (uint8_t) keylen;
  P->fanout = 1;
  P->depth = 1;
  blake2_store32(P->leaf_length, 0);
  blake2_store32(P->node_offset, 0);
  blake2_store16(P->xof_length, 0);
  P->node_depth = 0;
  P->inner_length = 0;
  /* memset(P->reserved, 0, sizeof(P->reserved)); */
  blake2_memset(P->salt, 0, sizeof(P->salt));
  blake2_memset(P->personal, 0, sizeof(P->personal));

  blake2s_init_param(S, P);

  blake2_memset(block, 0, BLAKE2S_BLOCKBYTES);
  blake2_memcpy(block, key, keylen);
  blake2s_update(S, block, BLAKE2S_BLOCKBYTES);
  secure_zero_memory(block, BLAKE2S_BLOCKBYTES); /* Burn the key from stack */

  return 0;
}

int blake2s_update(FAR blake2s_state *S, FAR const void *pin, size_t inlen)
{
  FAR const unsigned char * in = FAR (const unsigned char *)pin;
  size_t left, fill;

  if (inlen <= 0)
    {
      return 0;
    }

  left = S->buflen;
  fill = BLAKE2S_BLOCKBYTES - left;
  if (inlen > fill)
    {
      S->buflen = 0;
      if (fill)
        {
          blake2_memcpy(S->buf + left, in, fill); /* Fill buffer */
        }

      blake2s_increment_counter(S, BLAKE2S_BLOCKBYTES);
      blake2s_compress(S, S->buf); /* Compress */
      in += fill;
      inlen -= fill;
      while (inlen > BLAKE2S_BLOCKBYTES)
        {
          blake2s_increment_counter(S, BLAKE2S_BLOCKBYTES);
          blake2s_compress(S, in);
          in += BLAKE2S_BLOCKBYTES;
          inlen -= BLAKE2S_BLOCKBYTES;
        }
    }

  blake2_memcpy(S->buf + S->buflen, in, inlen);
  S->buflen += inlen;

  return 0;
}

int blake2s_final(FAR blake2s_state *S, FAR void *out, size_t outlen)
{
  FAR uint8_t *outbuf = out;
  uint32_t tmp = 0;
  size_t outwords;
  size_t padding;
  size_t i;

  if (out == NULL || outlen < S->outlen)
    {
      return -1;
    }

  if (blake2s_is_lastblock(S))
    {
      return -1;
    }

  blake2s_increment_counter(S, (uint32_t)S->buflen);
  blake2s_set_lastblock(S);
  padding = BLAKE2S_BLOCKBYTES - S->buflen;
  if (padding)
    {
      blake2_memset(S->buf + S->buflen, 0, padding);
    }
  blake2s_compress(S, S->buf);

  /* Output hash to out buffer */

  outwords = outlen / sizeof(uint32_t);
  outwords = (outwords < 8) ? outwords : 8;
  for (i = 0; i < outwords; ++i)
    {
      /* Store full words */

      blake2_store32(outbuf, S->h[i]);
      outlen -= sizeof(uint32_t);
      outbuf += sizeof(uint32_t);
    }

  if (outwords < 8 && outlen > 0 && outlen < sizeof(uint32_t))
    {
      /* Store partial word */

      blake2_store32(&tmp, S->h[i]);
      blake2_memcpy(outbuf, &tmp, outlen);
    }

  return 0;
}

int blake2s(FAR void *out, size_t outlen, FAR const void *in, size_t inlen,
            FAR const void *key, size_t keylen)
{
  blake2s_state S[1];

  /* Verify parameters */

  if (NULL == in && inlen > 0)
    {
      return -1;
    }

  if (NULL == out)
    {
      return -1;
    }

  if (NULL == key && keylen > 0)
    {
      return -1;
    }

  if (!outlen || outlen > BLAKE2S_OUTBYTES)
    {
      return -1;
    }

  if (keylen > BLAKE2S_KEYBYTES)
    {
      return -1;
    }

  if (keylen > 0)
    {
      if (blake2s_init_key(S, outlen, key, keylen) < 0)
        {
          return -1;
        }
    }
  else
    {
      if (blake2s_init(S, outlen) < 0)
        {
          return -1;
        }
    }

  blake2s_update(S, (const uint8_t *)in, inlen);
  blake2s_final(S, out, outlen);
  return 0;
}
