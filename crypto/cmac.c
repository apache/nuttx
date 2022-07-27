/****************************************************************************
 * crypto/cmac.c
 * $OpenBSD: cmac.c,v 1.3 2017/05/02 17:07:06 mikeb Exp $
 *
 * Copyright (c) 2008 Damien Bergamini <damien.bergamini@free.fr>
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
 ****************************************************************************/

/* This code implements the CMAC (Cipher-based Message Authentication)
 * algorithm described in FIPS SP800-38B using the AES-128 cipher.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/param.h>
#include <sys/systm.h>

#include <crypto/aes.h>
#include <crypto/cmac.h>

#define LSHIFT(v, r) do \
  { \
    int i; \
    for (i = 0; i < 15; i++) \
      (r)[i] = (v)[i] << 1 | (v)[i + 1] >> 7; \
    (r)[15] = (v)[15] << 1; \
  } while (0)

#define XOR(v, r) do \
  { \
    int i; \
    for (i = 0; i < 16; i++) \
      (r)[i] ^= (v)[i]; \
  } while (0)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void aes_cmac_init(FAR AES_CMAC_CTX *ctx)
{
  memset(ctx->X, 0, sizeof ctx->X);
  ctx->m_n = 0;
}

void aes_cmac_setkey(FAR AES_CMAC_CTX *ctx,
                     FAR const uint8_t *key)
{
  aes_setkey(&ctx->aesctx, key, 16);
}

void aes_cmac_update(FAR AES_CMAC_CTX *ctx,
                     FAR const uint8_t *data,
                     u_int len)
{
  u_int mlen;

  if (ctx->m_n > 0)
    {
      mlen = MIN(16 - ctx->m_n, len);
      memcpy(ctx->m_last + ctx->m_n, data, mlen);
      ctx->m_n += mlen;
      if (ctx->m_n < 16 || len == mlen)
        {
          return;
        }

      XOR(ctx->m_last, ctx->X);
      aes_encrypt(&ctx->aesctx, ctx->X, ctx->X);
      data += mlen;
      len -= mlen;
    }

  while (len > 16)
    {
      /* not last block */

      XOR(data, ctx->X);
      aes_encrypt(&ctx->aesctx, ctx->X, ctx->X);
      data += 16;
      len -= 16;
    }

  /* potential last block, save it */

  memcpy(ctx->m_last, data, len);
  ctx->m_n = len;
}

void aes_cmac_final(FAR uint8_t *digest,
                    FAR AES_CMAC_CTX *ctx)
{
  uint8_t K[16];

  /* generate subkey K1 */

  memset(K, 0, sizeof K);
  aes_encrypt(&ctx->aesctx, K, K);

  if (K[0] & 0x80)
    {
      LSHIFT(K, K);
      K[15] ^= 0x87;
    }
  else
    {
      LSHIFT(K, K);
    }

  if (ctx->m_n == 16)
    {
      /* last block was a complete block */

      XOR(K, ctx->m_last);
    }
  else
    {
      /* generate subkey K2 */

      if (K[0] & 0x80)
        {
          LSHIFT(K, K);
          K[15] ^= 0x87;
        }
      else
        {
          LSHIFT(K, K);
        }

      /* padding(m_last) */

      ctx->m_last[ctx->m_n] = 0x80;
      while (++ctx->m_n < 16)
        {
          ctx->m_last[ctx->m_n] = 0;
        }

      XOR(K, ctx->m_last);
    }

  XOR(ctx->m_last, ctx->X);
  aes_encrypt(&ctx->aesctx, ctx->X, digest);

  explicit_bzero(K, sizeof K);
}
