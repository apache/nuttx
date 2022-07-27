/****************************************************************************
 * crypto/gmac.c
 * $OpenBSD: gmac.c,v 1.10 2017/05/02 11:44:32 mikeb Exp $
 *
 * Copyright (c) 2010 Mike Belopuhov
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 *
 *
 * This code implements the Message Authentication part of the
 * Galois/Counter Mode (as being described in the RFC 4543) using
 * the AES cipher.  FIPS SP 800-38D describes the algorithm details.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/param.h>
#include <sys/systm.h>

#include <crypto/aes.h>
#include <crypto/gmac.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void ghash_gfmul(FAR uint32_t *, FAR uint32_t *, FAR uint32_t *);
void ghash_update_mi(FAR GHASH_CTX *, FAR uint8_t *, size_t);

/* Allow overriding with optimized MD function */

CODE void (*ghash_update)(FAR GHASH_CTX *,
                          FAR uint8_t *,
                          size_t) = ghash_update_mi;

/* Computes a block multiplication in the GF(2^128) */

void ghash_gfmul(FAR uint32_t *X, FAR uint32_t *Y, FAR uint32_t *product)
{
  uint32_t v[4];
  uint32_t z[4] =
  {
    0, 0, 0, 0
  };

  FAR uint8_t *x = (FAR uint8_t *)X;
  uint32_t mask;
  int i;

  v[0] = betoh32(Y[0]);
  v[1] = betoh32(Y[1]);
  v[2] = betoh32(Y[2]);
  v[3] = betoh32(Y[3]);

  for (i = 0; i < GMAC_BLOCK_LEN * 8; i++)
    {
      /* update Z */

      mask = !!(x[i >> 3] & (1 << (~i & 7)));
      mask = ~(mask - 1);
      z[0] ^= v[0] & mask;
      z[1] ^= v[1] & mask;
      z[2] ^= v[2] & mask;
      z[3] ^= v[3] & mask;

      /* update V */

      mask = ~((v[3] & 1) - 1);
      v[3] = (v[2] << 31) | (v[3] >> 1);
      v[2] = (v[1] << 31) | (v[2] >> 1);
      v[1] = (v[0] << 31) | (v[1] >> 1);
      v[0] = (v[0] >> 1) ^ (0xe1000000 & mask);
    }

  product[0] = htobe32(z[0]);
  product[1] = htobe32(z[1]);
  product[2] = htobe32(z[2]);
  product[3] = htobe32(z[3]);
}

void ghash_update_mi(FAR GHASH_CTX *ctx, FAR uint8_t *X, size_t len)
{
  FAR uint32_t *x = (FAR uint32_t *)X;
  FAR uint32_t *s = (FAR uint32_t *)ctx->S;
  FAR uint32_t *y = (FAR uint32_t *)ctx->Z;
  int i;

  for (i = 0; i < len / GMAC_BLOCK_LEN; i++)
    {
      s[0] = y[0] ^ x[0];
      s[1] = y[1] ^ x[1];
      s[2] = y[2] ^ x[2];
      s[3] = y[3] ^ x[3];

      ghash_gfmul((FAR uint32_t *)ctx->S, (FAR uint32_t *)ctx->H,
          (FAR uint32_t *)ctx->S);

      y = s;
      x += 4;
    }

  bcopy(ctx->S, ctx->Z, GMAC_BLOCK_LEN);
}

#define AESCTR_NONCESIZE 4

void aes_gmac_init(FAR void *xctx)
{
  FAR AES_GMAC_CTX *ctx = xctx;

  bzero(ctx->ghash.H, GMAC_BLOCK_LEN);
  bzero(ctx->ghash.S, GMAC_BLOCK_LEN);
  bzero(ctx->ghash.Z, GMAC_BLOCK_LEN);
  bzero(ctx->J, GMAC_BLOCK_LEN);
}

void aes_gmac_setkey(FAR void *xctx, FAR const uint8_t *key, uint16_t klen)
{
  FAR AES_GMAC_CTX *ctx = xctx;

  aes_setkey(&ctx->K, key, klen - AESCTR_NONCESIZE);

  /* copy out salt to the counter block */

  bcopy(key + klen - AESCTR_NONCESIZE, ctx->J, AESCTR_NONCESIZE);

  /* prepare a hash subkey */

  aes_encrypt(&ctx->K, ctx->ghash.H, ctx->ghash.H);
}

void aes_gmac_reinit(FAR void *xctx, FAR const uint8_t *iv, uint16_t ivlen)
{
  FAR AES_GMAC_CTX *ctx = xctx;

  /* copy out IV to the counter block */

  bcopy(iv, ctx->J + AESCTR_NONCESIZE, ivlen);
}

int aes_gmac_update(FAR void *xctx, FAR const uint8_t *data, uint16_t len)
{
  FAR AES_GMAC_CTX *ctx = xctx;
  uint32_t blk[4] =
  {
    0, 0, 0, 0
  };

  int plen;

  if (len > 0)
    {
      plen = len % GMAC_BLOCK_LEN;
      if (len >= GMAC_BLOCK_LEN)
        {
          (*ghash_update)(&ctx->ghash, (FAR uint8_t *)data,
            len - plen);
        }

      if (plen)
        {
          memcpy((FAR uint8_t *)blk, (FAR uint8_t *)data + (len - plen),
              plen);
          (*ghash_update)(&ctx->ghash, (FAR uint8_t *)blk,
              GMAC_BLOCK_LEN);
        }
    }

  return (0);
}

void aes_gmac_final(FAR uint8_t *digest, FAR void *xctx)
{
  FAR AES_GMAC_CTX *ctx = xctx;
  uint8_t keystream[GMAC_BLOCK_LEN];
  int i;

  /* do one round of GCTR */

  ctx->J[GMAC_BLOCK_LEN - 1] = 1;
  aes_encrypt(&ctx->K, ctx->J, keystream);
  for (i = 0; i < GMAC_DIGEST_LEN; i++)
    {
      digest[i] = ctx->ghash.S[i] ^ keystream[i];
    }

  explicit_bzero(keystream, sizeof(keystream));
}
