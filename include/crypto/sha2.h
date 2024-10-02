/****************************************************************************
 * include/crypto/sha2.h
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: 2000-2001, Aaron D. Gifford
 * SPDX-FileContributor: Aaron D. Gifford <me@aarongifford.com>
 *
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
 * 3. Neither the name of the copyright holder nor the names of contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTOR(S) ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTOR(S) BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_CRYPTO_SHA2_H
#define __INCLUDE_CRYPTO_SHA2_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>

/* SHA-256/384/512 Various Length Definitions */

#define SHA224_BLOCK_LENGTH         64
#define SHA224_DIGEST_LENGTH        28
#define SHA224_DIGEST_STRING_LENGTH (SHA224_DIGEST_LENGTH * 2 + 1)
#define SHA256_BLOCK_LENGTH         64
#define SHA256_DIGEST_LENGTH        32
#define SHA256_DIGEST_STRING_LENGTH (SHA256_DIGEST_LENGTH * 2 + 1)
#define SHA384_BLOCK_LENGTH         128
#define SHA384_DIGEST_LENGTH        48
#define SHA384_DIGEST_STRING_LENGTH (SHA384_DIGEST_LENGTH * 2 + 1)
#define SHA512_BLOCK_LENGTH         128
#define SHA512_DIGEST_LENGTH        64
#define SHA512_DIGEST_STRING_LENGTH (SHA512_DIGEST_LENGTH * 2 + 1)

/* SHA-256/384/512 Context Structure */

typedef struct _SHA2_CTX
{
  union
  {
    uint32_t st32[8];
    uint64_t st64[8];
  } state;
  uint64_t bitcount[2];
  uint8_t buffer[SHA512_BLOCK_LENGTH];
} SHA2_CTX;

void sha224init(FAR SHA2_CTX *);
void sha224update(FAR SHA2_CTX *, FAR const void *, size_t);
void sha224final(FAR uint8_t *, FAR SHA2_CTX *);

void sha256init(FAR SHA2_CTX *);
void sha256update(FAR SHA2_CTX *, FAR const void *, size_t);
void sha256final(FAR uint8_t *, FAR SHA2_CTX *);

void sha384init(FAR SHA2_CTX *);
void sha384update(FAR SHA2_CTX *, FAR const void *, size_t);
void sha384final(FAR uint8_t *, FAR SHA2_CTX *);

void sha512init(FAR SHA2_CTX *);
void sha512update(FAR SHA2_CTX *, FAR const void *, size_t);
void sha512final(FAR uint8_t *, FAR SHA2_CTX *);

#endif /* __INCLUDE_CRYPTO_SHA2_H */
