/****************************************************************************
 * include/crypto/poly1305.h
 * $OpenBSD: poly1305.h,v 1.2 2020/07/22 13:54:30 tobhe Exp $
 *
 * Public Domain poly1305 from Andrew Moon
 *
 * poly1305 implementation using 32 bit * 32 bit = 64 bit multiplication
 * and 64 bit addition from https://github.com/floodyberry/poly1305-donna
 ****************************************************************************/

#ifndef __INCLUDE_CRYPTO_POLY1305_H
#define __INCLUDE_CRYPTO_POLY1305_H

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

void poly1305_init(FAR poly1305_state *, FAR const unsigned char *);
void poly1305_update(FAR poly1305_state *,
                     FAR const unsigned char *, size_t);
void poly1305_finish(FAR poly1305_state *, FAR unsigned char *);

#endif /* __INCLUDE_CRYPTO_POLY1305_H */
