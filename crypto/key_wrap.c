/****************************************************************************
 * crypto/key_wrap.c
 * $OpenBSD: key_wrap.c,v 1.5 2017/05/02 17:07:06 mikeb Exp $
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
 ****************************************************************************/

/* This code implements the AES Key Wrap algorithm described in RFC 3394. */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/param.h>
#include <sys/systm.h>

#include <crypto/aes.h>
#include <crypto/key_wrap.h>

static const uint8_t IV[8] =
{
  0xa6, 0xa6, 0xa6, 0xa6, 0xa6, 0xa6, 0xa6, 0xa6
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void aes_key_wrap_set_key(FAR aes_key_wrap_ctx *ctx,
                          FAR const uint8_t *K,
                          size_t k_len)
{
  aes_setkey(&ctx->ctx, K, k_len);
}

void aes_key_wrap_set_key_wrap_only(FAR aes_key_wrap_ctx *ctx,
                                    FAR const uint8_t *K,
                                    size_t k_len)
{
  aes_setkey(&ctx->ctx, K, k_len);
}

void aes_key_wrap(FAR aes_key_wrap_ctx *ctx,
                  FAR const uint8_t *P,
                  size_t n, FAR uint8_t *C)
{
  uint64_t B[2];
  uint64_t t;
  FAR uint8_t *A;
  FAR uint8_t *R;
  size_t i;
  int j;

  memmove(C + 8, P, n * 8); /* P and C may overlap */
  A = C;                    /* A points to C[0] */
  memcpy(A, IV, 8);         /* A = IV, an initial value */

  for (j = 0, t = 1; j <= 5; j++)
    {
      R = C + 8;
      for (i = 1; i <= n; i++, t++)
        {
          /* B = A | R[i] */

          memcpy(&B[0], A, 8);
          memcpy(&B[1], R, 8);

          /* B = AES(K, B) */

          aes_encrypt(&ctx->ctx, (FAR uint8_t *)B, (FAR uint8_t *)B);

          /* MSB(64, B) = MSB(64, B) ^ t */

          B[0] ^= htobe64(t);

          /* A = MSB(64, B) */

          memcpy(A, &B[0], 8);

          /* R[i] = LSB(64, B) */

          memcpy(R, &B[1], 8);

          R += 8;
        }
    }

  explicit_bzero(B, sizeof B);
}

int aes_key_unwrap(FAR aes_key_wrap_ctx *ctx,
                   FAR const uint8_t *C,
                   FAR uint8_t *P, size_t n)
{
  uint64_t B[2];
  uint64_t t;
  uint8_t A[8];
  FAR uint8_t *R;
  size_t i;
  int j;

  memcpy(A, C, 8);          /* A = C[0] */
  memmove(P, C + 8, n * 8); /* P and C may overlap */

  for (j = 5, t = 6 * n; j >= 0; j--)
    {
      R = P + (n - 1) * 8;
      for (i = n; i >= 1; i--, t--)
        {
          /* MSB(64, B) = A */

          memcpy(&B[0], A, 8);

          /* MSB(64, B) = MSB(64, B) ^ t */

          B[0] ^= htobe64(t);

          /* B = MSB(64, B) | R[i] */

          memcpy(&B[1], R, 8);

          /* B = AES-1(K, B) */

          aes_decrypt(&ctx->ctx, (FAR uint8_t *)B, (FAR uint8_t *)B);

          /* A = MSB(64, B) */

          memcpy(A, &B[0], 8);

          /* R[i] = LSB(64, B) */

          memcpy(R, &B[1], 8);

          R -= 8;
        }
    }

  explicit_bzero(B, sizeof B);

  /* check that A is an appropriate initial value */

  return timingsafe_bcmp(A, IV, 8) != 0;
}
