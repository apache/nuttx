/****************************************************************************
 * crypto/sha2.c
 * $OpenBSD: sha2.c,v 1.19 2021/03/12 10:22:46 jsg Exp $
 * FILE: sha2.c
 * AUTHOR: Aaron D. Gifford <me@aarongifford.com>
 *
 * Copyright (c) 2000-2001, Aaron D. Gifford
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
 * $From: sha2.c,v 1.1 2001/11/08 00:01:51 adg Exp adg $
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <endian.h>
#include <string.h>
#include <sys/time.h>
#include <crypto/sha2.h>

/* UNROLLED TRANSFORM LOOP NOTE:
 * You can define SHA2_UNROLL_TRANSFORM to use the unrolled transform
 * loop version for the hash transform rounds (defined using macros
 * later in this file).  Either define on the command line, for example:
 *
 *   cc -DSHA2_UNROLL_TRANSFORM -o sha2 sha2.c sha2prog.c
 *
 * or define below:
 *
 *   #define SHA2_UNROLL_TRANSFORM
 *
 */

#ifndef SMALL_KERNEL
#  if defined(__amd64__) || defined(__i386__)
#    define SHA2_UNROLL_TRANSFORM
#  endif
#endif

/* SHA-256/384/512 Machine Architecture Definitions */

/* BYTE_ORDER NOTE:
 *
 * Please make sure that your system defines BYTE_ORDER.  If your
 * architecture is little-endian, make sure it also defines
 * LITTLE_ENDIAN and that the two (BYTE_ORDER and LITTLE_ENDIAN) are
 * equivalent.
 *
 * If your system does not define the above, then you can do so by
 * hand like this:
 *
 *   #define LITTLE_ENDIAN 1234
 *   #define BIG_ENDIAN    4321
 *
 * And for little-endian machines, add:
 *
 *   #define BYTE_ORDER LITTLE_ENDIAN
 *
 * Or for big-endian machines:
 *
 *   #define BYTE_ORDER BIG_ENDIAN
 *
 * The FreeBSD machine this was written on defines BYTE_ORDER
 * appropriately by including <sys/types.h> (which in turn includes
 * <machine/endian.h> where the appropriate definitions are actually
 * made).
 */

#if !defined(BYTE_ORDER) || \
    (BYTE_ORDER != LITTLE_ENDIAN && BYTE_ORDER != BIG_ENDIAN)
#  error Define BYTE_ORDER to be equal to either LITTLE_ENDIAN or BIG_ENDIAN
#endif

/* SHA-256/384/512 Various Length Definitions */

/* NOTE: Most of these are in sha2.h */

#define SHA256_SHORT_BLOCK_LENGTH (SHA256_BLOCK_LENGTH - 8)
#define SHA384_SHORT_BLOCK_LENGTH (SHA384_BLOCK_LENGTH - 16)
#define SHA512_SHORT_BLOCK_LENGTH (SHA512_BLOCK_LENGTH - 16)

/* Macro for incrementally adding the unsigned 64-bit integer n to the
 * unsigned 128-bit integer (represented using a two-element array of
 * 64-bit words):
 */

#define ADDINC128(w,n)         \
  do                           \
    {                          \
      (w)[0] += (uint64_t)(n); \
      if ((w)[0] < (n))        \
        {                      \
          (w)[1]++;            \
        }                      \
    }                          \
  while (0)

/* THE SIX LOGICAL FUNCTIONS */

/* Bit shifting and rotation (used by the six SHA-XYZ logical functions:
 *
 *   NOTE:  The naming of R and S appears backwards here (R is a SHIFT and
 *   S is a ROTATION) because the SHA-256/384/512 description document
 *   (see http://csrc.nist.gov/cryptval/shs/sha256-384-512.pdf) uses this
 *   same "backwards" definition.
 */

/* Shift-right (used in SHA-256, SHA-384, and SHA-512): */

#define R(b,x) ((x) >> (b))

/* 32-bit Rotate-right (used in SHA-256): */

#define S32(b,x) (((x) >> (b)) | ((x) << (32 - (b))))

/* 64-bit Rotate-right (used in SHA-384 and SHA-512): */

#define S64(b,x) (((x) >> (b)) | ((x) << (64 - (b))))

/* Two of six logical functions used in SHA-256, SHA-384, and SHA-512: */

#define CH(x,y,z) (((x) & (y)) ^ ((~(x)) & (z)))

#define MAJ(x,y,z) (((x) & (y)) ^ ((x) & (z)) ^ ((y) & (z)))

/* Four of six logical functions used in SHA-256: */

#define SIGMA0_256(x) (S32(2, (x)) ^ S32(13, (x)) ^ S32(22, (x)))
#define SIGMA1_256(x) (S32(6, (x)) ^ S32(11, (x)) ^ S32(25, (x)))
#define sigma0_256(x) (S32(7, (x)) ^ S32(18, (x)) ^ R(3, (x)))
#define sigma1_256(x) (S32(17, (x)) ^ S32(19, (x)) ^ R(10, (x)))

/* Four of six logical functions used in SHA-384 and SHA-512: */

#define SIGMA0_512(x) (S64(28, (x)) ^ S64(34, (x)) ^ S64(39, (x)))
#define SIGMA1_512(x) (S64(14, (x)) ^ S64(18, (x)) ^ S64(41, (x)))
#define sigma0_512(x) (S64(1, (x)) ^ S64( 8, (x)) ^ R(7, (x)))
#define sigma1_512(x) (S64(19, (x)) ^ S64(61, (x)) ^ R(6, (x)))

/* INTERNAL FUNCTION PROTOTYPES */

/* NOTE: These should not be accessed directly from outside this
 * library -- they are intended for private internal visibility/use
 * only.
 */

void sha512last(FAR SHA2_CTX *);
void sha256transform(FAR uint32_t *, FAR const uint8_t *);
void sha512transform(FAR uint64_t *, FAR const uint8_t *);

/* SHA-XYZ INITIAL HASH VALUES AND CONSTANTS */

/* Hash constant words K for SHA-256: */

const static uint32_t K256[64] =
{
  0x428a2f98ul, 0x71374491ul, 0xb5c0fbcful, 0xe9b5dba5ul,
  0x3956c25bul, 0x59f111f1ul, 0x923f82a4ul, 0xab1c5ed5ul,
  0xd807aa98ul, 0x12835b01ul, 0x243185beul, 0x550c7dc3ul,
  0x72be5d74ul, 0x80deb1feul, 0x9bdc06a7ul, 0xc19bf174ul,
  0xe49b69c1ul, 0xefbe4786ul, 0x0fc19dc6ul, 0x240ca1ccul,
  0x2de92c6ful, 0x4a7484aaul, 0x5cb0a9dcul, 0x76f988daul,
  0x983e5152ul, 0xa831c66dul, 0xb00327c8ul, 0xbf597fc7ul,
  0xc6e00bf3ul, 0xd5a79147ul, 0x06ca6351ul, 0x14292967ul,
  0x27b70a85ul, 0x2e1b2138ul, 0x4d2c6dfcul, 0x53380d13ul,
  0x650a7354ul, 0x766a0abbul, 0x81c2c92eul, 0x92722c85ul,
  0xa2bfe8a1ul, 0xa81a664bul, 0xc24b8b70ul, 0xc76c51a3ul,
  0xd192e819ul, 0xd6990624ul, 0xf40e3585ul, 0x106aa070ul,
  0x19a4c116ul, 0x1e376c08ul, 0x2748774cul, 0x34b0bcb5ul,
  0x391c0cb3ul, 0x4ed8aa4aul, 0x5b9cca4ful, 0x682e6ff3ul,
  0x748f82eeul, 0x78a5636ful, 0x84c87814ul, 0x8cc70208ul,
  0x90befffaul, 0xa4506cebul, 0xbef9a3f7ul, 0xc67178f2ul
};

/* Initial hash value H for SHA-256: */

const static uint32_t sha256_initial_hash_value[8] =
{
  0x6a09e667ul,
  0xbb67ae85ul,
  0x3c6ef372ul,
  0xa54ff53aul,
  0x510e527ful,
  0x9b05688cul,
  0x1f83d9abul,
  0x5be0cd19ul
};

/* Hash constant words K for SHA-384 and SHA-512: */

const static uint64_t K512[80] =
{
  0x428a2f98d728ae22ull, 0x7137449123ef65cdull,
  0xb5c0fbcfec4d3b2full, 0xe9b5dba58189dbbcull,
  0x3956c25bf348b538ull, 0x59f111f1b605d019ull,
  0x923f82a4af194f9bull, 0xab1c5ed5da6d8118ull,
  0xd807aa98a3030242ull, 0x12835b0145706fbeull,
  0x243185be4ee4b28cull, 0x550c7dc3d5ffb4e2ull,
  0x72be5d74f27b896full, 0x80deb1fe3b1696b1ull,
  0x9bdc06a725c71235ull, 0xc19bf174cf692694ull,
  0xe49b69c19ef14ad2ull, 0xefbe4786384f25e3ull,
  0x0fc19dc68b8cd5b5ull, 0x240ca1cc77ac9c65ull,
  0x2de92c6f592b0275ull, 0x4a7484aa6ea6e483ull,
  0x5cb0a9dcbd41fbd4ull, 0x76f988da831153b5ull,
  0x983e5152ee66dfabull, 0xa831c66d2db43210ull,
  0xb00327c898fb213full, 0xbf597fc7beef0ee4ull,
  0xc6e00bf33da88fc2ull, 0xd5a79147930aa725ull,
  0x06ca6351e003826full, 0x142929670a0e6e70ull,
  0x27b70a8546d22ffcull, 0x2e1b21385c26c926ull,
  0x4d2c6dfc5ac42aedull, 0x53380d139d95b3dfull,
  0x650a73548baf63deull, 0x766a0abb3c77b2a8ull,
  0x81c2c92e47edaee6ull, 0x92722c851482353bull,
  0xa2bfe8a14cf10364ull, 0xa81a664bbc423001ull,
  0xc24b8b70d0f89791ull, 0xc76c51a30654be30ull,
  0xd192e819d6ef5218ull, 0xd69906245565a910ull,
  0xf40e35855771202aull, 0x106aa07032bbd1b8ull,
  0x19a4c116b8d2d0c8ull, 0x1e376c085141ab53ull,
  0x2748774cdf8eeb99ull, 0x34b0bcb5e19b48a8ull,
  0x391c0cb3c5c95a63ull, 0x4ed8aa4ae3418acbull,
  0x5b9cca4f7763e373ull, 0x682e6ff3d6b2b8a3ull,
  0x748f82ee5defb2fcull, 0x78a5636f43172f60ull,
  0x84c87814a1f0ab72ull, 0x8cc702081a6439ecull,
  0x90befffa23631e28ull, 0xa4506cebde82bde9ull,
  0xbef9a3f7b2c67915ull, 0xc67178f2e372532bull,
  0xca273eceea26619cull, 0xd186b8c721c0c207ull,
  0xeada7dd6cde0eb1eull, 0xf57d4f7fee6ed178ull,
  0x06f067aa72176fbaull, 0x0a637dc5a2c898a6ull,
  0x113f9804bef90daeull, 0x1b710b35131c471bull,
  0x28db77f523047d84ull, 0x32caab7b40c72493ull,
  0x3c9ebe0a15c9bebcull, 0x431d67c49c100d4cull,
  0x4cc5d4becb3e42b6ull, 0x597f299cfc657e2aull,
  0x5fcb6fab3ad6faecull, 0x6c44198c4a475817ull
};

/* Initial hash value H for SHA-384 */

const static uint64_t sha384_initial_hash_value[8] =
{
  0xcbbb9d5dc1059ed8ull,
  0x629a292a367cd507ull,
  0x9159015a3070dd17ull,
  0x152fecd8f70e5939ull,
  0x67332667ffc00b31ull,
  0x8eb44a8768581511ull,
  0xdb0c2e0d64f98fa7ull,
  0x47b5481dbefa4fa4ull
};

/* Initial hash value H for SHA-512 */

const static uint64_t sha512_initial_hash_value[8] =
{
  0x6a09e667f3bcc908ull,
  0xbb67ae8584caa73bull,
  0x3c6ef372fe94f82bull,
  0xa54ff53a5f1d36f1ull,
  0x510e527fade682d1ull,
  0x9b05688c2b3e6c1full,
  0x1f83d9abfb41bd6bull,
  0x5be0cd19137e2179ull
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* SHA-256: */

void sha256init(FAR SHA2_CTX *context)
{
  memcpy(context->state.st32,
         sha256_initial_hash_value,
         SHA256_DIGEST_LENGTH);

  memset(context->buffer, 0, SHA256_BLOCK_LENGTH);
  context->bitcount[0] = 0;
}

#ifdef SHA2_UNROLL_TRANSFORM

/* Unrolled SHA-256 round macros: */

#define ROUND256_0_TO_15(a, b, c, d, e, f, g, h)                  \
  do                                                              \
    {                                                             \
      W256[j] = (uint32_t)data[3] | ((uint32_t)data[2] << 8) |    \
          ((uint32_t)data[1] << 16) | ((uint32_t)data[0] << 24);  \
      data += 4;                                                  \
      T1 = (h) + SIGMA1_256((e)) +                                \
          CH((e), (f), (g)) + K256[j] + W256[j];                  \
      (d) += T1;                                                  \
      (h) = T1 + SIGMA0_256((a)) + MAJ((a), (b), (c));            \
      j++;                                                        \
    }                                                             \
  while (0)

#define ROUND256(a, b, c, d, e, f, g, h)                           \
  do                                                               \
    {                                                              \
      s0 = W256[(j + 1) & 0x0f];                                   \
      s0 = sigma0_256(s0);                                         \
      s1 = W256[(j+14)&0x0f];                                      \
      s1 = sigma1_256(s1);                                         \
      T1 = (h) + SIGMA1_256((e)) + CH((e), (f), (g)) + K256[j] +   \
            (W256[j & 0x0f] += s1 + W256[(j + 9) & 0x0f] + s0);    \
      (d) += T1;                                                   \
      (h) = T1 + SIGMA0_256((a)) + MAJ((a), (b), (c));             \
      j++;                                                         \
    }                                                              \
while(0)

void sha256transform(FAR uint32_t *state, FAR const uint8_t *data)
{
  uint32_t a;
  uint32_t b;
  uint32_t c;
  uint32_t d;
  uint32_t e;
  uint32_t f;
  uint32_t g;
  uint32_t h;
  uint32_t s0;
  uint32_t s1;
  uint32_t T1;
  uint32_t W256[16];
  int j;

  /* Initialize registers with the prev. intermediate value */

  a = state[0];
  b = state[1];
  c = state[2];
  d = state[3];
  e = state[4];
  f = state[5];
  g = state[6];
  h = state[7];

  j = 0;
  do
    {
      /* Rounds 0 to 15 (unrolled): */

      ROUND256_0_TO_15(a, b, c, d, e, f, g, h);
      ROUND256_0_TO_15(h, a, b, c, d, e, f, g);
      ROUND256_0_TO_15(g, h, a, b, c, d, e, f);
      ROUND256_0_TO_15(f, g, h, a, b, c, d, e);
      ROUND256_0_TO_15(e, f, g, h, a, b, c, d);
      ROUND256_0_TO_15(d, e, f, g, h, a, b, c);
      ROUND256_0_TO_15(c, d, e, f, g, h, a, b);
      ROUND256_0_TO_15(b, c, d, e, f, g, h, a);
    }
  while (j < 16);

  /* Now for the remaining rounds to 64: */

  do
    {
      ROUND256(a, b, c, d, e, f, g, h);
      ROUND256(h, a, b, c, d, e, f, g);
      ROUND256(g, h, a, b, c, d, e, f);
      ROUND256(f, g, h, a, b, c, d, e);
      ROUND256(e, f, g, h, a, b, c, d);
      ROUND256(d, e, f, g, h, a, b, c);
      ROUND256(c, d, e, f, g, h, a, b);
      ROUND256(b, c, d, e, f, g, h, a);
    }
  while (j < 64);

  /* Compute the current intermediate hash value */

  state[0] += a;
  state[1] += b;
  state[2] += c;
  state[3] += d;
  state[4] += e;
  state[5] += f;
  state[6] += g;
  state[7] += h;

  /* Clean up */

  a = b = c = d = e = f = g = h = T1 = 0;
}

#else /* SHA2_UNROLL_TRANSFORM */

void sha256transform(FAR uint32_t *state, FAR const uint8_t *data)
{
  uint32_t a;
  uint32_t b;
  uint32_t c;
  uint32_t d;
  uint32_t e;
  uint32_t f;
  uint32_t g;
  uint32_t h;
  uint32_t s0;
  uint32_t s1;
  uint32_t T1;
  uint32_t T2;
  uint32_t W256[16];
  int j;

  /* Initialize registers with the prev. intermediate value */

  a = state[0];
  b = state[1];
  c = state[2];
  d = state[3];
  e = state[4];
  f = state[5];
  g = state[6];
  h = state[7];

  j = 0;
  do
    {
      W256[j] = (uint32_t)data[3] | ((uint32_t)data[2] << 8) |
          ((uint32_t)data[1] << 16) | ((uint32_t)data[0] << 24);
      data += 4;

      /* Apply the SHA-256 compression function to update a..h */

      T1 = h + SIGMA1_256(e) + CH(e, f, g) + K256[j] + W256[j];
      T2 = SIGMA0_256(a) + MAJ(a, b, c);
      h = g;
      g = f;
      f = e;
      e = d + T1;
      d = c;
      c = b;
      b = a;
      a = T1 + T2;

      j++;
    }
  while (j < 16);

  do
    {
      /* Part of the message block expansion: */

      s0 = W256[(j + 1) & 0x0f];
      s0 = sigma0_256(s0);
      s1 = W256[(j + 14) & 0x0f];
      s1 = sigma1_256(s1);

      /* Apply the SHA-256 compression function to update a..h */

      T1 = h + SIGMA1_256(e) + CH(e, f, g) + K256[j] +
            (W256[j & 0x0f] += s1 + W256[(j + 9) & 0x0f] + s0);
      T2 = SIGMA0_256(a) + MAJ(a, b, c);
      h = g;
      g = f;
      f = e;
      e = d + T1;
      d = c;
      c = b;
      b = a;
      a = T1 + T2;

      j++;
    }
  while (j < 64);

  /* Compute the current intermediate hash value */

  state[0] += a;
  state[1] += b;
  state[2] += c;
  state[3] += d;
  state[4] += e;
  state[5] += f;
  state[6] += g;
  state[7] += h;

  /* Clean up */

  a = b = c = d = e = f = g = h = T1 = T2 = 0;
}

#endif /* SHA2_UNROLL_TRANSFORM */

void sha256update(FAR SHA2_CTX *context,
                  FAR const void *dataptr,
                  size_t len)
{
  FAR const uint8_t *data = dataptr;
  size_t freespace;
  size_t usedspace;

  /* Calling with no data is valid (we do nothing) */

  if (len == 0)
    {
      return;
    }

  usedspace = (context->bitcount[0] >> 3) % SHA256_BLOCK_LENGTH;
  if (usedspace > 0)
    {
      /* Calculate how much free space is available in the buffer */

      freespace = SHA256_BLOCK_LENGTH - usedspace;

      if (len >= freespace)
        {
          /* Fill the buffer completely and process it */

          memcpy(&context->buffer[usedspace], data, freespace);
          context->bitcount[0] += freespace << 3;
          len -= freespace;
          data += freespace;
          sha256transform(context->state.st32, context->buffer);
        }
      else
        {
          /* The buffer is not yet full */

          memcpy(&context->buffer[usedspace], data, len);
          context->bitcount[0] += len << 3;

          /* Clean up: */

          usedspace = freespace = 0;
          return;
        }
    }

  while (len >= SHA256_BLOCK_LENGTH)
    {
      /* Process as many complete blocks as we can */

      sha256transform(context->state.st32, data);
      context->bitcount[0] += SHA256_BLOCK_LENGTH << 3;
      len -= SHA256_BLOCK_LENGTH;
      data += SHA256_BLOCK_LENGTH;
    }

  if (len > 0)
    {
      /* There's left-overs, so save 'em */

      memcpy(context->buffer, data, len);
      context->bitcount[0] += len << 3;
    }

  /* Clean up: */

  usedspace = freespace = 0;
}

void sha256final(FAR uint8_t *digest, FAR SHA2_CTX *context)
{
  unsigned int usedspace;

  usedspace = (context->bitcount[0] >> 3) % SHA256_BLOCK_LENGTH;
#if BYTE_ORDER == LITTLE_ENDIAN

  /* Convert FROM host byte order */

  context->bitcount[0] = swap64(context->bitcount[0]);
#endif

  if (usedspace > 0)
    {
      /* Begin padding with a 1 bit: */

      context->buffer[usedspace++] = 0x80;

      if (usedspace <= SHA256_SHORT_BLOCK_LENGTH)
        {
          /* Set-up for the last transform: */

          memset(&context->buffer[usedspace], 0,
              SHA256_SHORT_BLOCK_LENGTH - usedspace);
        }
      else
        {
          if (usedspace < SHA256_BLOCK_LENGTH)
            {
              memset(&context->buffer[usedspace], 0,
                  SHA256_BLOCK_LENGTH - usedspace);
            }

          /* Do second-to-last transform: */

          sha256transform(context->state.st32, context->buffer);

          /* And set-up for the last transform: */

          memset(context->buffer, 0,
              SHA256_SHORT_BLOCK_LENGTH);
        }
    }
  else
    {
      /* Set-up for the last transform: */

      memset(context->buffer, 0, SHA256_SHORT_BLOCK_LENGTH);

      /* Begin padding with a 1 bit: */

      *context->buffer = 0x80;
    }

  /* Set the bit count: */

  *(FAR uint64_t *)&context->buffer[SHA256_SHORT_BLOCK_LENGTH] =
  context->bitcount[0];

  /* Final transform: */

  sha256transform(context->state.st32, context->buffer);

#if BYTE_ORDER == LITTLE_ENDIAN
    {
      /* Convert TO host byte order */

      int j;

      for (j = 0; j < 8; j++)
        {
          context->state.st32[j] = swap32(context->state.st32[j]);
        }
    }
#endif

  memcpy(digest, context->state.st32, SHA256_DIGEST_LENGTH);

  /* Clean up state data: */

  explicit_bzero(context, sizeof(*context));
  usedspace = 0;
}

/* SHA-512: */

void sha512init(FAR SHA2_CTX *context)
{
  memcpy(context->state.st64, sha512_initial_hash_value,
         SHA512_DIGEST_LENGTH);
  memset(context->buffer, 0, SHA512_BLOCK_LENGTH);
  context->bitcount[0] = context->bitcount[1] =  0;
}

#ifdef SHA2_UNROLL_TRANSFORM

/* Unrolled SHA-512 round macros: */

#define ROUND512_0_TO_15(a, b, c, d, e, f, g, h)                          \
  do                                                                      \
    {                                                                     \
      W512[j] = (uint64_t)data[7] | ((uint64_t)data[6] << 8) |            \
          ((uint64_t)data[5] << 16) | ((uint64_t)data[4] << 24) |         \
          ((uint64_t)data[3] << 32) | ((uint64_t)data[2] << 40) |         \
          ((uint64_t)data[1] << 48) | ((uint64_t)data[0] << 56);          \
      data += 8;                                                          \
      T1 = (h) + SIGMA1_512((e)) + CH((e), (f), (g)) + K512[j] + W512[j]; \
      (d) += T1;                                                          \
      (h) = T1 + SIGMA0_512((a)) + MAJ((a), (b), (c));                    \
      j++;                                                                \
    }                                                                     \
  while (0)

#define ROUND512(a, b, c, d, e, f, g, h)                                  \
  do                                                                      \
    {                                                                     \
      s0 = W512[(j + 1) & 0x0f];                                          \
      s0 = sigma0_512(s0);                                                \
      s1 = W512[(j + 14) & 0x0f];                                         \
      s1 = sigma1_512(s1);                                                \
      T1 = (h) + SIGMA1_512((e)) + CH((e), (f), (g)) + K512[j] +          \
                  (W512[j & 0x0f] += s1 + W512[(j +9 ) & 0x0f] + s0);     \
      (d) += T1;                                                          \
      (h) = T1 + SIGMA0_512((a)) + MAJ((a), (b), (c));                    \
      j++;                                                                \
    }                                                                     \
  while(0)

void sha512transform(FAR uint64_t *state, FAR const uint8_t *data)
{
  uint64_t a;
  uint64_t b;
  uint64_t c;
  uint64_t d;
  uint64_t e;
  uint64_t f;
  uint64_t g;
  uint64_t h;
  uint64_t s0;
  uint64_t s1;
  uint64_t T1;
  uint64_t W512[16];
  int j;

  /* Initialize registers with the prev. intermediate value */

  a = state[0];
  b = state[1];
  c = state[2];
  d = state[3];
  e = state[4];
  f = state[5];
  g = state[6];
  h = state[7];

  j = 0;
  do
    {
      ROUND512_0_TO_15(a, b, c, d, e, f, g, h);
      ROUND512_0_TO_15(h, a, b, c, d, e, f, g);
      ROUND512_0_TO_15(g, h, a, b, c, d, e, f);
      ROUND512_0_TO_15(f, g, h, a, b, c, d, e);
      ROUND512_0_TO_15(e, f, g, h, a, b, c, d);
      ROUND512_0_TO_15(d, e, f, g, h, a, b, c);
      ROUND512_0_TO_15(c, d, e, f, g, h, a, b);
      ROUND512_0_TO_15(b, c, d, e, f, g, h, a);
    }
  while (j < 16);

  /* Now for the remaining rounds up to 79: */

  do
    {
      ROUND512(a, b, c, d, e, f, g, h);
      ROUND512(h, a, b, c, d, e, f, g);
      ROUND512(g, h, a, b, c, d, e, f);
      ROUND512(f, g, h, a, b, c, d, e);
      ROUND512(e, f, g, h, a, b, c, d);
      ROUND512(d, e, f, g, h, a, b, c);
      ROUND512(c, d, e, f, g, h, a, b);
      ROUND512(b, c, d, e, f, g, h, a);
    }
  while (j < 80);

  /* Compute the current intermediate hash value */

  state[0] += a;
  state[1] += b;
  state[2] += c;
  state[3] += d;
  state[4] += e;
  state[5] += f;
  state[6] += g;
  state[7] += h;

  /* Clean up */

  a = b = c = d = e = f = g = h = T1 = 0;
}

#else /* SHA2_UNROLL_TRANSFORM */

void sha512transform(FAR uint64_t *state, FAR const uint8_t *data)
{
  uint64_t a;
  uint64_t b;
  uint64_t c;
  uint64_t d;
  uint64_t e;
  uint64_t f;
  uint64_t g;
  uint64_t h;
  uint64_t s0;
  uint64_t s1;
  uint64_t T1;
  uint64_t T2;
  uint64_t W512[16];
  int j;

  /* Initialize registers with the prev. intermediate value */

  a = state[0];
  b = state[1];
  c = state[2];
  d = state[3];
  e = state[4];
  f = state[5];
  g = state[6];
  h = state[7];

  j = 0;
  do
    {
      W512[j] = (uint64_t)data[7] | ((uint64_t)data[6] << 8) |
          ((uint64_t)data[5] << 16) | ((uint64_t)data[4] << 24) |
          ((uint64_t)data[3] << 32) | ((uint64_t)data[2] << 40) |
          ((uint64_t)data[1] << 48) | ((uint64_t)data[0] << 56);
      data += 8;

      /* Apply the SHA-512 compression function to update a..h */

      T1 = h + SIGMA1_512(e) + CH(e, f, g) + K512[j] + W512[j];
      T2 = SIGMA0_512(a) + MAJ(a, b, c);
      h = g;
      g = f;
      f = e;
      e = d + T1;
      d = c;
      c = b;
      b = a;
      a = T1 + T2;

      j++;
    }
  while (j < 16);

  do
    {
      /* Part of the message block expansion: */

      s0 = W512[(j + 1) & 0x0f];
      s0 = sigma0_512(s0);
      s1 = W512[(j + 14) & 0x0f];
      s1 =  sigma1_512(s1);

      /* Apply the SHA-512 compression function to update a..h */

      T1 = h + SIGMA1_512(e) + CH(e, f, g) + K512[j] +
            (W512[j & 0x0f] += s1 + W512[(j + 9) & 0x0f] + s0);
      T2 = SIGMA0_512(a) + MAJ(a, b, c);
      h = g;
      g = f;
      f = e;
      e = d + T1;
      d = c;
      c = b;
      b = a;
      a = T1 + T2;

      j++;
    }
  while (j < 80);

  /* Compute the current intermediate hash value */

  state[0] += a;
  state[1] += b;
  state[2] += c;
  state[3] += d;
  state[4] += e;
  state[5] += f;
  state[6] += g;
  state[7] += h;

  /* Clean up */

  a = b = c = d = e = f = g = h = T1 = T2 = 0;
}

#endif /* SHA2_UNROLL_TRANSFORM */

void sha512update(FAR SHA2_CTX *context, FAR const void *dataptr, size_t len)
{
  FAR const uint8_t *data = dataptr;
  size_t freespace;
  size_t usedspace;

  /* Calling with no data is valid (we do nothing) */

  if (len == 0)
    {
      return;
    }

  usedspace = (context->bitcount[0] >> 3) % SHA512_BLOCK_LENGTH;
  if (usedspace > 0)
    {
      /* Calculate how much free space is available in the buffer */

      freespace = SHA512_BLOCK_LENGTH - usedspace;

      if (len >= freespace)
        {
          /* Fill the buffer completely and process it */

          memcpy(&context->buffer[usedspace], data, freespace);
          ADDINC128(context->bitcount, freespace << 3);
          len -= freespace;
          data += freespace;
          sha512transform(context->state.st64, context->buffer);
        }
      else
        {
          /* The buffer is not yet full */

          memcpy(&context->buffer[usedspace], data, len);
          ADDINC128(context->bitcount, len << 3);

          /* Clean up: */

          usedspace = freespace = 0;
          return;
        }
    }

  while (len >= SHA512_BLOCK_LENGTH)
    {
      /* Process as many complete blocks as we can */

      sha512transform(context->state.st64, data);
      ADDINC128(context->bitcount, SHA512_BLOCK_LENGTH << 3);
      len -= SHA512_BLOCK_LENGTH;
      data += SHA512_BLOCK_LENGTH;
    }

  if (len > 0)
    {
      /* There's left-overs, so save 'em */

      memcpy(context->buffer, data, len);
      ADDINC128(context->bitcount, len << 3);
    }

  /* Clean up: */

  usedspace = freespace = 0;
}

void sha512last(FAR SHA2_CTX *context)
{
  unsigned int usedspace;

  usedspace = (context->bitcount[0] >> 3) % SHA512_BLOCK_LENGTH;
#if BYTE_ORDER == LITTLE_ENDIAN

  /* Convert FROM host byte order */

  context->bitcount[0] = swap64(context->bitcount[0]);
  context->bitcount[1] = swap64(context->bitcount[1]);
#endif
  if (usedspace > 0)
    {
      /* Begin padding with a 1 bit: */

      context->buffer[usedspace++] = 0x80;

      if (usedspace <= SHA512_SHORT_BLOCK_LENGTH)
        {
          /* Set-up for the last transform: */

          memset(&context->buffer[usedspace], 0,
              SHA512_SHORT_BLOCK_LENGTH - usedspace);
        }
      else
        {
          if (usedspace < SHA512_BLOCK_LENGTH)
            {
              memset(&context->buffer[usedspace], 0,
                  SHA512_BLOCK_LENGTH - usedspace);
            }

          /* Do second-to-last transform: */

          sha512transform(context->state.st64, context->buffer);

          /* And set-up for the last transform: */

          memset(context->buffer, 0, SHA512_BLOCK_LENGTH - 2);
        }
    }
  else
    {
      /* Prepare for final transform: */

      memset(context->buffer, 0, SHA512_SHORT_BLOCK_LENGTH);

      /* Begin padding with a 1 bit: */

      *context->buffer = 0x80;
    }

  /* Store the length of input data (in bits): */

  *(FAR uint64_t *)&context->buffer[SHA512_SHORT_BLOCK_LENGTH] =
    context->bitcount[1];
  *(FAR uint64_t *)&context->buffer[SHA512_SHORT_BLOCK_LENGTH + 8] =
    context->bitcount[0];

  /* Final transform: */

  sha512transform(context->state.st64, context->buffer);
}

void sha512final(FAR uint8_t *digest, FAR SHA2_CTX *context)
{
  sha512last(context);

  /* Save the hash data for output: */

#if BYTE_ORDER == LITTLE_ENDIAN
    {
      /* Convert TO host byte order */

      int j;
      for (j = 0; j < 8; j++)
        {
          context->state.st64[j] = swap64(context->state.st64[j]);
        }
    }
#endif

  memcpy(digest, context->state.st64, SHA512_DIGEST_LENGTH);

  /* Zero out state data */

  explicit_bzero(context, sizeof(*context));
}

/* SHA-384: */

void sha384init(FAR SHA2_CTX *context)
{
  memcpy(context->state.st64, sha384_initial_hash_value,
      SHA512_DIGEST_LENGTH);
  memset(context->buffer, 0, SHA384_BLOCK_LENGTH);
  context->bitcount[0] = context->bitcount[1] = 0;
}

void sha384update(FAR SHA2_CTX *context, FAR const void *data, size_t len)
{
  sha512update(context, data, len);
}

void sha384final(FAR uint8_t *digest, FAR SHA2_CTX *context)
{
  sha512last(context);

  /* Save the hash data for output: */

#if BYTE_ORDER == LITTLE_ENDIAN
    {
      /* Convert TO host byte order */

      int j;
      for (j = 0; j < 6; j++)
        {
          context->state.st64[j] = swap64(context->state.st64[j]);
        }
    }
#endif

  memcpy(digest, context->state.st64, SHA384_DIGEST_LENGTH);

  /* Zero out state data */

  explicit_bzero(context, sizeof(*context));
}
