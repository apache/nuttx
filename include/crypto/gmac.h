/****************************************************************************
 * include/crypto/gmac.h
 * $OpenBSD: gmac.h,v 1.6 2017/05/02 11:44:32 mikeb Exp $
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
 ****************************************************************************/

#ifndef __INCLUDE_CRYPTO_GMAC_H
#define __INCLUDE_CRYPTO_GMAC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <crypto/aes.h>

#define GMAC_BLOCK_LEN 16
#define GMAC_DIGEST_LEN 16

typedef struct _GHASH_CTX
{
  uint8_t H[GMAC_BLOCK_LEN]; /* hash subkey */
  uint8_t S[GMAC_BLOCK_LEN]; /* state */
  uint8_t Z[GMAC_BLOCK_LEN]; /* initial state */
}
GHASH_CTX;

typedef struct _AES_GMAC_CTX
{
  GHASH_CTX ghash;
  AES_CTX K;
  uint8_t J[GMAC_BLOCK_LEN]; /* counter block */
}
AES_GMAC_CTX;

extern void (*ghash_update)(FAR GHASH_CTX *, FAR uint8_t *, size_t);

void aes_gmac_init(FAR void *);
void aes_gmac_setkey(FAR void *, FAR const uint8_t *, uint16_t);
void aes_gmac_reinit(FAR void *, FAR const uint8_t *, uint16_t);
int aes_gmac_update(FAR void *, FAR const uint8_t *, uint16_t);
void aes_gmac_final(FAR uint8_t *, FAR void *);

#endif /* __INCLUDE_CRYPTO_GMAC_H */
