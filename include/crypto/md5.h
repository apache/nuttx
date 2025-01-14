/****************************************************************************
 * include/crypto/md5.h
 *
 * SPDX-License-Identifier: NuttX-PublicDomain
 *
 * This code implements the MD5 message-digest algorithm.
 * The algorithm is due to Ron Rivest.  This code was
 * written by Colin Plumb in 1993, no copyright is claimed.
 * This code is in the public domain; do with it what you wish.
 *
 ****************************************************************************/

#ifndef __INCLUDE_CRYPTO_MD5_H
#define __INCLUDE_CRYPTO_MD5_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>

#define MD5_BLOCK_LENGTH    64
#define MD5_DIGEST_LENGTH   16

typedef struct MD5CONTEXT
{
  uint32_t state[4];                 /* state */
  uint64_t count;                    /* number of bits, mod 2^64 */
  uint8_t buffer[MD5_BLOCK_LENGTH];  /* input buffer */
} MD5_CTX;

void md5init(FAR MD5_CTX *);
void md5update(FAR MD5_CTX *, FAR const void *, size_t);
void md5final(FAR uint8_t *, FAR MD5_CTX *);
void md5transform(FAR uint32_t *, FAR const uint8_t *);

#endif /* __INCLUDE_CRYPTO_MD5_H */
