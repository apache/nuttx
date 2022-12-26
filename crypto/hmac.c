/****************************************************************************
 * crypto/hmac.c
 * $OpenBSD: hmac.c,v 1.4 2016/09/19 18:09:40 tedu Exp $
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

/* This code implements the HMAC algorithm described in RFC 2104 using
 * the MD5, SHA1 and SHA-256 hash functions.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <string.h>
#include <strings.h>
#include <sys/param.h>

#include <crypto/md5.h>
#include <crypto/sha1.h>
#include <crypto/sha2.h>
#include <crypto/hmac.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void hmca_md5_init(FAR HMAC_MD5_CTX *ctx,
                   FAR const uint8_t *key,
                   u_int key_len)
{
  uint8_t k_ipad[MD5_BLOCK_LENGTH];
  int i;

  if (key_len > MD5_BLOCK_LENGTH)
    {
      md5init(&ctx->ctx);
      md5update(&ctx->ctx, key, key_len);
      md5final(ctx->key, &ctx->ctx);
      ctx->key_len = MD5_DIGEST_LENGTH;
    }
  else
    {
      bcopy(key, ctx->key, key_len);
      ctx->key_len = key_len;
    }

  bzero(k_ipad, MD5_BLOCK_LENGTH);
  memcpy(k_ipad, ctx->key, ctx->key_len);
  for (i = 0; i < MD5_BLOCK_LENGTH; i++)
    {
      k_ipad[i] ^= 0x36;
    }

  md5init(&ctx->ctx);
  md5update(&ctx->ctx, k_ipad, MD5_BLOCK_LENGTH);

  explicit_bzero(k_ipad, sizeof k_ipad);
}

void hmac_md5_update(FAR HMAC_MD5_CTX *ctx,
                     FAR const uint8_t *data,
                     u_int len)
{
  md5update(&ctx->ctx, data, len);
}

void hmac_md5_final(FAR uint8_t *digest, FAR HMAC_MD5_CTX *ctx)
{
  uint8_t k_opad[MD5_BLOCK_LENGTH];
  int i;

  md5final(digest, &ctx->ctx);

  bzero(k_opad, MD5_BLOCK_LENGTH);
  memcpy(k_opad, ctx->key, ctx->key_len);
  for (i = 0; i < MD5_BLOCK_LENGTH; i++)
    {
      k_opad[i] ^= 0x5c;
    }

  md5init(&ctx->ctx);
  md5update(&ctx->ctx, k_opad, MD5_BLOCK_LENGTH);
  md5update(&ctx->ctx, digest, MD5_DIGEST_LENGTH);
  md5final(digest, &ctx->ctx);

  explicit_bzero(k_opad, sizeof k_opad);
}

void hmac_sha1_init(FAR HMAC_SHA1_CTX *ctx,
                    FAR const uint8_t *key,
                    u_int key_len)
{
  uint8_t k_ipad[SHA1_BLOCK_LENGTH];
  int i;

  if (key_len > SHA1_BLOCK_LENGTH)
    {
      sha1init(&ctx->ctx);
      sha1update(&ctx->ctx, key, key_len);
      sha1final(ctx->key, &ctx->ctx);
      ctx->key_len = SHA1_DIGEST_LENGTH;
    }
  else
    {
      bcopy(key, ctx->key, key_len);
      ctx->key_len = key_len;
    }

  bzero(k_ipad, SHA1_BLOCK_LENGTH);
  memcpy(k_ipad, ctx->key, ctx->key_len);
  for (i = 0; i < SHA1_BLOCK_LENGTH; i++)
    {
      k_ipad[i] ^= 0x36;
    }

  sha1init(&ctx->ctx);
  sha1update(&ctx->ctx, k_ipad, SHA1_BLOCK_LENGTH);

  explicit_bzero(k_ipad, sizeof k_ipad);
}

void hmac_sha1_update(FAR HMAC_SHA1_CTX *ctx,
                      FAR const uint8_t *data,
                      u_int len)
{
  sha1update(&ctx->ctx, data, len);
}

void hmca_sha1_final(FAR uint8_t *digest, FAR HMAC_SHA1_CTX *ctx)
{
  uint8_t k_opad[SHA1_BLOCK_LENGTH];
  int i;

  sha1final(digest, &ctx->ctx);

  bzero(k_opad, SHA1_BLOCK_LENGTH);
  memcpy(k_opad, ctx->key, ctx->key_len);
  for (i = 0; i < SHA1_BLOCK_LENGTH; i++)
    {
      k_opad[i] ^= 0x5c;
    }

  sha1init(&ctx->ctx);
  sha1update(&ctx->ctx, k_opad, SHA1_BLOCK_LENGTH);
  sha1update(&ctx->ctx, digest, SHA1_DIGEST_LENGTH);
  sha1final(digest, &ctx->ctx);

  explicit_bzero(k_opad, sizeof k_opad);
}

void hmac_sha256_init(FAR HMAC_SHA256_CTX *ctx,
                      FAR const uint8_t *key,
                      u_int key_len)
{
  uint8_t k_ipad[SHA256_BLOCK_LENGTH];
  int i;

  if (key_len > SHA256_BLOCK_LENGTH)
    {
      sha256init(&ctx->ctx);
      sha256update(&ctx->ctx, key, key_len);
      sha256final(ctx->key, &ctx->ctx);
      ctx->key_len = SHA256_DIGEST_LENGTH;
    }
  else
    {
      bcopy(key, ctx->key, key_len);
      ctx->key_len = key_len;
    }

  bzero(k_ipad, SHA256_BLOCK_LENGTH);
  memcpy(k_ipad, ctx->key, ctx->key_len);
  for (i = 0; i < SHA256_BLOCK_LENGTH; i++)
    {
      k_ipad[i] ^= 0x36;
    }

  sha256init(&ctx->ctx);
  sha256update(&ctx->ctx, k_ipad, SHA256_BLOCK_LENGTH);

  explicit_bzero(k_ipad, sizeof k_ipad);
}

void hmac_sha256_update(FAR HMAC_SHA256_CTX *ctx,
                        FAR const uint8_t *data,
                        u_int len)
{
  sha256update(&ctx->ctx, data, len);
}

void hmac_sha256_final(FAR uint8_t *digest,
                       FAR HMAC_SHA256_CTX *ctx)
{
  uint8_t k_opad[SHA256_BLOCK_LENGTH];
  int i;

  sha256final(digest, &ctx->ctx);

  bzero(k_opad, SHA256_BLOCK_LENGTH);
  memcpy(k_opad, ctx->key, ctx->key_len);
  for (i = 0; i < SHA256_BLOCK_LENGTH; i++)
    {
      k_opad[i] ^= 0x5c;
    }

  sha256init(&ctx->ctx);
  sha256update(&ctx->ctx, k_opad, SHA256_BLOCK_LENGTH);
  sha256update(&ctx->ctx, digest, SHA256_DIGEST_LENGTH);
  sha256final(digest, &ctx->ctx);

  explicit_bzero(k_opad, sizeof k_opad);
}
