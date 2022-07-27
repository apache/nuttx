/****************************************************************************
 * include/crypto/hmac.h
 * $OpenBSD: hmac.h,v 1.3 2012/12/05 23:20:15 deraadt Exp $
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

#ifndef __INCLUDE_CRYPTO_HMAC_H_
#define __INCLUDE_CRYPTO_HMAC_H_

typedef struct _HMAC_MD5_CTX
{
  MD5_CTX ctx;
  u_int8_t key[MD5_BLOCK_LENGTH];
  u_int key_len;
} HMAC_MD5_CTX;

typedef struct _HMAC_SHA1_CTX
{
  SHA1_CTX ctx;
  u_int8_t key[SHA1_BLOCK_LENGTH];
  u_int key_len;
} HMAC_SHA1_CTX;

typedef struct _HMAC_SHA256_CTX
{
  SHA2_CTX ctx;
  u_int8_t key[SHA256_BLOCK_LENGTH];
  u_int key_len;
} HMAC_SHA256_CTX;

void hmac_md5_init(FAR HMAC_MD5_CTX *, FAR const u_int8_t *, u_int);
void hmac_md5_update(FAR HMAC_MD5_CTX *, FAR const u_int8_t *, u_int);
void hmac_md5_final(FAR u_int8_t *, FAR HMAC_MD5_CTX *);
void hmac_sha1_init(FAR HMAC_SHA1_CTX *, FAR const u_int8_t *, u_int);
void hmac_sha1_update(FAR HMAC_SHA1_CTX *, FAR const u_int8_t *, u_int);
void hmac_sha1_final(FAR u_int8_t *, FAR HMAC_SHA1_CTX *);

void hmac_sha256_init(FAR HMAC_SHA256_CTX *, FAR const u_int8_t *, u_int);
void hmac_sha256_update(FAR HMAC_SHA256_CTX *, FAR const u_int8_t *, u_int);
void hmac_sha256_final(FAR u_int8_t *, FAR HMAC_SHA256_CTX *);

#endif /* __INCLUDE_CRYPTO_HMAC_H_ */
