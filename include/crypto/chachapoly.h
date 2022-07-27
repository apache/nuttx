/****************************************************************************
 * include/crypto/chachapoly.h
 * $OpenBSD: chachapoly.h,v 1.4 2020/07/22 13:54:30 tobhe Exp $
 * Copyright (c) 2015 Mike Belopuhov
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

#ifndef __INCLUDE_CRYPTO_CHACHAPOLY_H
#define __INCLUDE_CRYPTO_CHACHAPOLY_H

#define CHACHA20_KEYSIZE   32
#define CHACHA20_CTR       4
#define CHACHA20_SALT      4
#define CHACHA20_NONCE     8
#define CHACHA20_BLOCK_LEN 64

struct chacha20_ctx
{
  uint8_t block[CHACHA20_BLOCK_LEN];
  uint8_t nonce[CHACHA20_NONCE];
};

int chacha20_setkey(FAR void *, FAR uint8_t *, int);
void chacha20_reinit(caddr_t, FAR uint8_t *);
void chacha20_crypt(caddr_t, FAR uint8_t *);

#define POLY1305_KEYLEN 32
#define POLY1305_TAGLEN 16
#define POLY1305_BLOCK_LEN 16

struct poly1305_ctx
{
  /* r, h, pad, leftover */

  unsigned long state[5 + 5 + 4];
  size_t leftover;
  unsigned char buffer[POLY1305_BLOCK_LEN];
  unsigned char final;
};

typedef struct
{
  uint8_t key[POLY1305_KEYLEN];

  /* counter, salt */

  uint8_t nonce[CHACHA20_NONCE];
  struct chacha20_ctx chacha;
  struct poly1305_ctx poly;
}
CHACHA20_POLY1305_CTX;

void chacha20_poly1305_init(FAR void *);
void chacha20_poly1305_setkey(FAR void *, FAR const uint8_t *, uint16_t);
void chacha20_poly1305_reinit(FAR void *, FAR const uint8_t *, uint16_t);
int chacha20_poly1305_update(FAR void *, FAR const uint8_t *, uint16_t);
void chacha20_poly1305_final(FAR uint8_t *, FAR void *);

/* WireGuard crypto */

#define CHACHA20POLY1305_KEY_SIZE CHACHA20_KEYSIZE
#define CHACHA20POLY1305_AUTHTAG_SIZE POLY1305_TAGLEN
#define XCHACHA20POLY1305_NONCE_SIZE 24

void chacha20poly1305_encrypt(
  FAR uint8_t *, FAR const uint8_t *, const size_t,
  FAR const uint8_t *, const size_t, const uint64_t,
  FAR const uint8_t *);

int chacha20poly1305_decrypt(
  FAR uint8_t *, FAR const uint8_t *, const size_t,
  FAR const uint8_t *, const size_t, const uint64_t,
  FAR const uint8_t *);

void xchacha20poly1305_encrypt(
  FAR uint8_t *, FAR const uint8_t *, const size_t,
  FAR const uint8_t *, const size_t,
  FAR const uint8_t *,
  FAR const uint8_t *);

int xchacha20poly1305_decrypt(
  FAR uint8_t *, FAR const uint8_t *, const size_t,
  FAR const uint8_t *, const size_t,
  FAR const uint8_t *,
  FAR const uint8_t *);

#endif /* __INCLUDE_CRYPTO_CHACHAPOLY_H */
