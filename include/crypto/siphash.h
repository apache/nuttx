/****************************************************************************
 * include/crypto/siphash.h
 * $OpenBSD: siphash.h,v 1.5 2015/02/20 11:51:03 tedu Exp $
 *
 * Copyright (c) 2013 Andre Oppermann <andre@FreeBSD.org>
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
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 *
 ****************************************************************************/

/* SipHash is a family of pseudorandom functions
 * (a.k.a. keyed hash functions)
 * optimized for speed on short messages returning a 64bit hash/digest value.
 *
 * The number of rounds is defined during the initialization:
 *  siphash24_init() for the fast and resonable strong version
 *  siphash48_init() for the strong version (half as fast)
 *
 * struct SIPHASH_CTX ctx;
 * siphash24_init(&ctx);
 * siphash_setkey(&ctx, "16bytes long key");
 * siphash_update(&ctx, pointer_to_string, length_of_string);
 * siphash_final(output, &ctx);
 */

#ifndef __INCLUDE_CRYPTO_SIPHASH_H_
#define __INCLUDE_CRYPTO_SIPHASH_H_

#define SIPHASH_BLOCK_LENGTH  8
#define SIPHASH_KEY_LENGTH    16
#define SIPHASH_DIGEST_LENGTH 8

typedef struct _SIPHASH_CTX
{
  uint64_t v[4];
  uint8_t buf[SIPHASH_BLOCK_LENGTH];
  uint32_t bytes;
} SIPHASH_CTX;

typedef struct
{
  uint64_t k0;
  uint64_t k1;
} SIPHASH_KEY;

void siphash_init(FAR SIPHASH_CTX *, FAR const SIPHASH_KEY *);
void siphash_update(FAR SIPHASH_CTX *, int, int, FAR const void *, size_t);
uint64_t siphash_end(FAR SIPHASH_CTX *, int, int);
void siphash_final(FAR void *, FAR SIPHASH_CTX *, int, int);
uint64_t siphash(FAR const SIPHASH_KEY *,
                 int, int, FAR const void *, size_t);

#define SipHash24_Init(_c, _k)        siphash_init((_c), (_k))
#define SipHash24_Update(_c, _p, _l)  siphash_update((_c), 2, 4, (_p), (_l))
#define SipHash24_End(_d)             siphash_end((_d), 2, 4)
#define SipHash24_Final(_d, _c)       siphash_final((_d), (_c), 2, 4)
#define SipHash24(_k, _p, _l)         siphash((_k), 2, 4, (_p), (_l))

#define SipHash48_Init(_c, _k)        siphash_init((_c), (_k))
#define SipHash48_Update(_c, _p, _l)  siphash_update((_c), 4, 8, (_p), (_l))
#define SipHash48_End(_d)             siphash_end((_d), 4, 8)
#define SipHash48_Final(_d, _c)       siphash_final((_d), (_c), 4, 8)
#define SipHash48(_k, _p, _l)         siphash((_k), 4, 8, (_p), (_l))

#endif /* _SIPHASH_H_ */
