/****************************************************************************
 * include/crypto/poly1305.h
 *
 * SPDX-License-Identifier: NuttX-PublicDomain
 *
 * Public Domain poly1305 from Andrew Moon
 *
 ****************************************************************************/

#ifndef __INCLUDE_CRYPTO_POLY1305_H
#define __INCLUDE_CRYPTO_POLY1305_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>

#define poly1305_block_size 16

typedef struct poly1305_state
{
  unsigned long r[5];
  unsigned long h[5];
  unsigned long pad[4];
  size_t leftover;
  unsigned char buffer[poly1305_block_size];
  unsigned char final;
} poly1305_state;

void poly1305_begin(FAR poly1305_state *, FAR const unsigned char *);
void poly1305_update(FAR poly1305_state *,
                     FAR const unsigned char *, size_t);
void poly1305_finish(FAR poly1305_state *, FAR unsigned char *);

#endif /* __INCLUDE_CRYPTO_POLY1305_H */
