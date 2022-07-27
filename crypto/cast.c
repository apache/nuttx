/****************************************************************************
 * crypto/cast.c
 * $OpenBSD: cast.c,v 1.4 2012/04/25 04:12:27 matthew Exp $
 *
 * CAST-128 in C
 * Written by Steve Reid <sreid@sea-to-sky.net>
 * 100% Public Domain - no warranty
 * Released 1997.10.11
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <sys/systm.h>
#include <crypto/cast.h>
#include "castsb.h"

/* Macros to access 8-bit bytes out of a 32-bit word */

#define UINT8_TA(x) ( (uint8_t) (x>>24) )
#define UINT8_TB(x) ( (uint8_t) ((x>>16)&255) )
#define UINT8_TC(x) ( (uint8_t) ((x>>8)&255) )
#define UINT8_TD(x) ( (uint8_t) ((x)&255) )

/* Circular left shift */

#define ROL(x, n) ( ((x)<<(n)) | ((x)>>(32-(n))) )

/* CAST-128 uses three different round functions */

#define F1(l, r, i) \
  t = ROL(key->xkey[i] + r, key->xkey[i+16]); \
  l ^= ((cast_sbox1[UINT8_TA(t)] ^ cast_sbox2[UINT8_TB(t)]) - \
    cast_sbox3[UINT8_TC(t)]) + cast_sbox4[UINT8_TD(t)];

#define F2(l, r, i) \
  t = ROL(key->xkey[i] ^ r, key->xkey[i+16]); \
  l ^= ((cast_sbox1[UINT8_TA(t)] - cast_sbox2[UINT8_TB(t)]) + \
    cast_sbox3[UINT8_TC(t)]) ^ cast_sbox4[UINT8_TD(t)];

#define F3(l, r, i) \
  t = ROL(key->xkey[i] - r, key->xkey[i+16]); \
  l ^= ((cast_sbox1[UINT8_TA(t)] + cast_sbox2[UINT8_TB(t)]) ^ \
    cast_sbox3[UINT8_TC(t)]) - cast_sbox4[UINT8_TD(t)];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Encryption Function */

void cast_encrypt(FAR cast_key *key,
                  FAR uint8_t *inblock,
                  FAR uint8_t *outblock)
{
  uint32_t t;
  uint32_t l;
  uint32_t r;

  /* Get inblock into l,r */

  l = ((uint32_t)inblock[0] << 24) |
      ((uint32_t)inblock[1] << 16) |
      ((uint32_t)inblock[2] << 8) |
      (uint32_t)inblock[3];

  r = ((uint32_t)inblock[4] << 24) |
      ((uint32_t)inblock[5] << 16) |
      ((uint32_t)inblock[6] << 8) |
      (uint32_t)inblock[7];

  /* Do the work */

  F1(l, r, 0);
  F2(r, l, 1);
  F3(l, r, 2);
  F1(r, l, 3);
  F2(l, r, 4);
  F3(r, l, 5);
  F1(l, r, 6);
  F2(r, l, 7);
  F3(l, r, 8);
  F1(r, l, 9);
  F2(l, r, 10);
  F3(r, l, 11);

  /* Only do full 16 rounds if key length > 80 bits */

  if (key->rounds > 12)
    {
      F1(l, r, 12);
      F2(r, l, 13);
      F3(l, r, 14);
      F1(r, l, 15);
    }

  /* Put l,r into outblock */

  outblock[0] = UINT8_TA(r);
  outblock[1] = UINT8_TB(r);
  outblock[2] = UINT8_TC(r);
  outblock[3] = UINT8_TD(r);
  outblock[4] = UINT8_TA(l);
  outblock[5] = UINT8_TB(l);
  outblock[6] = UINT8_TC(l);
  outblock[7] = UINT8_TD(l);

  /* Wipe clean */

  t = l = r = 0;
}

/* Decryption Function */

void cast_decrypt(FAR cast_key *key,
                  FAR uint8_t *inblock,
                  FAR uint8_t *outblock)
{
  uint32_t t;
  uint32_t l;
  uint32_t r;

  /* Get inblock into l,r */

  r = ((uint32_t)inblock[0] << 24) |
      ((uint32_t)inblock[1] << 16) |
      ((uint32_t)inblock[2] << 8) |
      (uint32_t)inblock[3];

  l = ((uint32_t)inblock[4] << 24) |
      ((uint32_t)inblock[5] << 16) |
      ((uint32_t)inblock[6] << 8) |
      (uint32_t)inblock[7];

  /* Do the work */

  /* Only do full 16 rounds if key length > 80 bits */

  if (key->rounds > 12)
    {
      F1(r, l, 15);
      F3(l, r, 14);
      F2(r, l, 13);
      F1(l, r, 12);
    }

  F3(r, l, 11);
  F2(l, r, 10);
  F1(r, l,  9);
  F3(l, r,  8);
  F2(r, l,  7);
  F1(l, r,  6);
  F3(r, l,  5);
  F2(l, r,  4);
  F1(r, l,  3);
  F3(l, r,  2);
  F2(r, l,  1);
  F1(l, r,  0);

  /* Put l,r into outblock */

  outblock[0] = UINT8_TA(l);
  outblock[1] = UINT8_TB(l);
  outblock[2] = UINT8_TC(l);
  outblock[3] = UINT8_TD(l);
  outblock[4] = UINT8_TA(r);
  outblock[5] = UINT8_TB(r);
  outblock[6] = UINT8_TC(r);
  outblock[7] = UINT8_TD(r);

  /* Wipe clean */

  t = l = r = 0;
}

/* Key Schedule */

void cast_setkey(FAR cast_key *key, FAR uint8_t *rawkey, int keybytes)
{
  uint32_t t[4];
  uint32_t z[4];
  uint32_t x[4];
  int i;

  /* Set number of rounds to 12 or 16, depending on key length */

  key->rounds = (keybytes <= 10 ? 12 : 16);

  /* Copy key to workspace x */

  for (i = 0; i < 4; i++)
    {
      x[i] = 0;
      if ((i * 4 + 0) < keybytes)
        {
          x[i] = (uint32_t)rawkey[i * 4 + 0] << 24;
        }

      if ((i * 4 + 1) < keybytes)
        {
          x[i] |= (uint32_t)rawkey[i * 4 + 1] << 16;
        }

      if ((i * 4 + 2) < keybytes)
        {
          x[i] |= (uint32_t)rawkey[i * 4 + 2] << 8;
        }

      if ((i * 4 + 3) < keybytes)
        {
          x[i] |= (uint32_t)rawkey[i * 4 + 3];
        }
    }

  /* Generate 32 subkeys, four at a time */

  for (i = 0; i < 32; i += 4)
    {
      switch (i & 4)
        {
          case 0:
            t[0] = z[0] = x[0] ^ cast_sbox5[UINT8_TB(x[3])] ^
                cast_sbox6[UINT8_TD(x[3])] ^
                cast_sbox7[UINT8_TA(x[3])] ^
                cast_sbox8[UINT8_TC(x[3])] ^
                cast_sbox7[UINT8_TA(x[2])];
            t[1] = z[1] = x[2] ^ cast_sbox5[UINT8_TA(z[0])] ^
                cast_sbox6[UINT8_TC(z[0])] ^
                cast_sbox7[UINT8_TB(z[0])] ^
                cast_sbox8[UINT8_TD(z[0])] ^
                cast_sbox8[UINT8_TC(x[2])];
            t[2] = z[2] = x[3] ^ cast_sbox5[UINT8_TD(z[1])] ^
                cast_sbox6[UINT8_TC(z[1])] ^
                cast_sbox7[UINT8_TB(z[1])] ^
                cast_sbox8[UINT8_TA(z[1])] ^
                cast_sbox5[UINT8_TB(x[2])];
            t[3] = z[3] = x[1] ^ cast_sbox5[UINT8_TC(z[2])] ^
                cast_sbox6[UINT8_TB(z[2])] ^
                cast_sbox7[UINT8_TD(z[2])] ^
                cast_sbox8[UINT8_TA(z[2])] ^
                cast_sbox6[UINT8_TD(x[2])];
            break;
          case 4:
            t[0] = x[0] = z[2] ^ cast_sbox5[UINT8_TB(z[1])] ^
                cast_sbox6[UINT8_TD(z[1])] ^
                cast_sbox7[UINT8_TA(z[1])] ^
                cast_sbox8[UINT8_TC(z[1])] ^
                cast_sbox7[UINT8_TA(z[0])];
            t[1] = x[1] = z[0] ^ cast_sbox5[UINT8_TA(x[0])] ^
                cast_sbox6[UINT8_TC(x[0])] ^
                cast_sbox7[UINT8_TB(x[0])] ^
                cast_sbox8[UINT8_TD(x[0])] ^
                cast_sbox8[UINT8_TC(z[0])];
            t[2] = x[2] = z[1] ^ cast_sbox5[UINT8_TD(x[1])] ^
                cast_sbox6[UINT8_TC(x[1])] ^
                cast_sbox7[UINT8_TB(x[1])] ^
                cast_sbox8[UINT8_TA(x[1])] ^
                cast_sbox5[UINT8_TB(z[0])];
            t[3] = x[3] = z[3] ^ cast_sbox5[UINT8_TC(x[2])] ^
                cast_sbox6[UINT8_TB(x[2])] ^
                cast_sbox7[UINT8_TD(x[2])] ^
                cast_sbox8[UINT8_TA(x[2])] ^
                cast_sbox6[UINT8_TD(z[0])];
            break;
        }

      switch (i & 12)
        {
          case 0:
          case 12:
            key->xkey[i + 0] = cast_sbox5[UINT8_TA(t[2])] ^
                cast_sbox6[UINT8_TB(t[2])] ^
                cast_sbox7[UINT8_TD(t[1])] ^
                cast_sbox8[UINT8_TC(t[1])];
            key->xkey[i + 1] = cast_sbox5[UINT8_TC(t[2])] ^
                cast_sbox6[UINT8_TD(t[2])] ^
                cast_sbox7[UINT8_TB(t[1])] ^
                cast_sbox8[UINT8_TA(t[1])];
            key->xkey[i + 2] = cast_sbox5[UINT8_TA(t[3])] ^
                cast_sbox6[UINT8_TB(t[3])] ^
                cast_sbox7[UINT8_TD(t[0])] ^
                cast_sbox8[UINT8_TC(t[0])];
            key->xkey[i + 3] = cast_sbox5[UINT8_TC(t[3])] ^
                cast_sbox6[UINT8_TD(t[3])] ^
                cast_sbox7[UINT8_TB(t[0])] ^
                cast_sbox8[UINT8_TA(t[0])];
            break;
          case 4:
          case 8:
            key->xkey[i + 0] = cast_sbox5[UINT8_TD(t[0])] ^
                cast_sbox6[UINT8_TC(t[0])] ^
                cast_sbox7[UINT8_TA(t[3])] ^
                cast_sbox8[UINT8_TB(t[3])];
            key->xkey[i + 1] = cast_sbox5[UINT8_TB(t[0])] ^
                cast_sbox6[UINT8_TA(t[0])] ^
                cast_sbox7[UINT8_TC(t[3])] ^
                cast_sbox8[UINT8_TD(t[3])];
            key->xkey[i + 2] = cast_sbox5[UINT8_TD(t[1])] ^
                cast_sbox6[UINT8_TC(t[1])] ^
                cast_sbox7[UINT8_TA(t[2])] ^
                cast_sbox8[UINT8_TB(t[2])];
            key->xkey[i + 3] = cast_sbox5[UINT8_TB(t[1])] ^
                cast_sbox6[UINT8_TA(t[1])] ^
                cast_sbox7[UINT8_TC(t[2])] ^
                cast_sbox8[UINT8_TD(t[2])];
            break;
        }

      switch (i & 12)
        {
          case 0:
            key->xkey[i + 0] ^= cast_sbox5[UINT8_TC(z[0])];
            key->xkey[i + 1] ^= cast_sbox6[UINT8_TC(z[1])];
            key->xkey[i + 2] ^= cast_sbox7[UINT8_TB(z[2])];
            key->xkey[i + 3] ^= cast_sbox8[UINT8_TA(z[3])];
            break;
          case 4:
            key->xkey[i + 0] ^= cast_sbox5[UINT8_TA(x[2])];
            key->xkey[i + 1] ^= cast_sbox6[UINT8_TB(x[3])];
            key->xkey[i + 2] ^= cast_sbox7[UINT8_TD(x[0])];
            key->xkey[i + 3] ^= cast_sbox8[UINT8_TD(x[1])];
            break;
          case 8:
            key->xkey[i + 0] ^= cast_sbox5[UINT8_TB(z[2])];
            key->xkey[i + 1] ^= cast_sbox6[UINT8_TA(z[3])];
            key->xkey[i + 2] ^= cast_sbox7[UINT8_TC(z[0])];
            key->xkey[i + 3] ^= cast_sbox8[UINT8_TC(z[1])];
            break;
          case 12:
            key->xkey[i + 0] ^= cast_sbox5[UINT8_TD(x[0])];
            key->xkey[i + 1] ^= cast_sbox6[UINT8_TD(x[1])];
            key->xkey[i + 2] ^= cast_sbox7[UINT8_TA(x[2])];
            key->xkey[i + 3] ^= cast_sbox8[UINT8_TB(x[3])];
            break;
        }

      if (i >= 16)
        {
          key->xkey[i + 0] &= 31;
          key->xkey[i + 1] &= 31;
          key->xkey[i + 2] &= 31;
          key->xkey[i + 3] &= 31;
        }
    }

  /* Wipe clean */

  explicit_bzero(t, sizeof(t));
  explicit_bzero(x, sizeof(x));
  explicit_bzero(z, sizeof(z));
}

/* Made in Canada */
