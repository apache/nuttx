/****************************************************************************
 * include/crypto/cast.h
 * $OpenBSD: cast.h,v 1.2 2002/03/14 01:26:51 millert Exp $
 *
 * CAST-128 in C
 * Written by Steve Reid <sreid@sea-to-sky.net>
 * 100% Public Domain - no warranty
 * Released 1997.10.11
 ****************************************************************************/

#ifndef __INCLUDE_CRYPTO_CAST_H
#define __INCLUDE_CRYPTO_CAST_H

typedef struct
{
  uint32_t xkey[32]; /* Key, after expansion */
  int rounds;        /* Number of rounds to use, 12 or 16 */
} cast_key;

void cast_setkey(FAR cast_key *key, FAR uint8_t *rawkey, int keybytes);
void cast_encrypt(FAR cast_key *key,
                  FAR uint8_t *inblock,
                  FAR uint8_t *outblock);
void cast_decrypt(FAR cast_key *key,
                  FAR uint8_t *inblock,
                  FAR uint8_t *outblock);

#endif /* __INCLUDE_CRYPTO_CAST_H */
