/****************************************************************************
 * include/crypto/sha1.h
 * $OpenBSD: sha1.h,v 1.6 2014/11/16 17:39:09 tedu Exp $
 * SHA-1 in C
 * By Steve Reid <steve@edmweb.com>
 * 100% Public Domain
 ****************************************************************************/

#ifndef __INCLUDE_CRYPTO_SHA1_H
#define __INCLUDE_CRYPTO_SHA1_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>

#define SHA1_BLOCK_LENGTH  64
#define SHA1_DIGEST_LENGTH 20

typedef struct
{
  uint32_t state[5];
  uint64_t count;
  unsigned char buffer[SHA1_BLOCK_LENGTH];
} SHA1_CTX;

void sha1init(FAR SHA1_CTX * context);
void sha1transform(FAR uint32_t *state,
                   FAR const unsigned char *buffer);
void sha1update(FAR SHA1_CTX *context,
                FAR const void *data,
                unsigned int len);
void sha1final(FAR unsigned char *digest,
               FAR SHA1_CTX *context);

#endif /* __INCLUDE_CRYPTO_SHA1_H */
