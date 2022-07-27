/****************************************************************************
 * include/crypto/blf.h
 * $OpenBSD: blf.h,v 1.7 2021/11/29 01:04:45 djm Exp $
 *
 * Blowfish - a fast block cipher designed by Bruce Schneier
 *
 * Copyright 1997 Niels Provos <provos@physnet.uni-hamburg.de>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************************/

#ifndef __INCLUDE_CRYPTO_BLF_H
#define __INCLUDE_CRYPTO_BLF_H

/* Schneier states the maximum key length to be 56 bytes.
 * The way how the subkeys are initialized by the key up
 * to (N+2)*4 i.e. 72 bytes are utilized.
 * Warning: For normal blowfish encryption only 56 bytes
 * of the key affect all cipherbits.
 */

#define BLF_N 16                          /* Number of Subkeys */
#define BLF_MAXKEYLEN ((BLF_N - 2) * 4)   /* 448 bits */
#define BLF_MAXUTILIZED ((BLF_N + 2) * 4) /* 576 bits */

/* Blowfish context */

typedef struct blowfishcontext
{
  uint32_t S[4][256];     /* S-Boxes */
  uint32_t P[BLF_N + 2];  /* Subkeys */
}
blf_ctx;

/* Raw access to customized Blowfish
 * blf_key is just:
 * Blowfish_initstate( state )
 * Blowfish_expand0state( state, key, keylen )
 */

void blowfish_encipher(FAR blf_ctx *, FAR uint32_t *);
void blowfish_decipher(FAR blf_ctx *, FAR uint32_t *);
void blowfish_initstate(FAR blf_ctx *);
void blowfish_expand0state(FAR blf_ctx *, FAR const uint8_t *, uint16_t);
void blowfish_expandstate(FAR blf_ctx *, FAR const uint8_t *,
                          uint16_t, FAR const uint8_t *, uint16_t);

/* Standard Blowfish */

void blf_key(FAR blf_ctx *, FAR const uint8_t *, uint16_t);
void blf_enc(FAR blf_ctx *, FAR uint32_t *, uint16_t);
void blf_dec(FAR blf_ctx *, FAR uint32_t *, uint16_t);

/* Converts uint8_t to uint32_t */

uint32_t blowfish_stream2word(FAR const uint8_t *, uint16_t,
                              FAR uint16_t *);

void blf_ecb_encrypt(FAR blf_ctx *, FAR uint8_t *, uint32_t);
void blf_ecb_decrypt(FAR blf_ctx *, FAR uint8_t *, uint32_t);

void blf_cbc_encrypt(FAR blf_ctx *, FAR uint8_t *, FAR uint8_t *,
                     uint32_t);
void blf_cbc_decrypt(FAR blf_ctx *, FAR uint8_t *, FAR uint8_t *,
                     uint32_t);
#endif /* __INCLUDE_CRYPTO_BLF_H */
