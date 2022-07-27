/****************************************************************************
 * include/crypto/aes.h
 * $OpenBSD: aes.h,v 1.4 2020/07/22 13:54:30 tobhe Exp $
 *
 * Copyright (c) 2016 Thomas Pornin <pornin@bolet.org>
 * Copyright (c) 2016 Mike Belopuhov
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
 ****************************************************************************/

#ifndef __INCLUDE_CRYPTO_AES_H
#define __INCLUDE_CRYPTO_AES_H

#ifndef AES_MAXROUNDS
#  define AES_MAXROUNDS (14)
#endif

typedef struct aes_ctx
{
  uint32_t sk[60];
  uint32_t sk_exp[120];

  unsigned num_rounds;
} AES_CTX;

int aes_setkey(FAR AES_CTX *, FAR const uint8_t *, int);
void aes_encrypt(FAR AES_CTX *, FAR const uint8_t *, FAR uint8_t *);
void aes_decrypt(FAR AES_CTX *, FAR const uint8_t *, FAR uint8_t *);
void aes_encrypt_ecb(FAR AES_CTX *, FAR const uint8_t *, FAR uint8_t *,
                     size_t);
void aes_decrypt_ecb(FAR AES_CTX *, FAR const uint8_t *, FAR uint8_t *,
                     size_t);

int aes_keysetup_encrypt(FAR uint32_t *, FAR const uint8_t *, int);
int aes_keysetup_decrypt(FAR uint32_t *, FAR const uint8_t *, int);

#endif /* __INCLUDE_CRYPTO_AES_H */
