/****************************************************************************
 * tools/imxrt1180/sha256.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/* Straightforward implementation of the SHA-256 algorithm as specified in
 * FIPS 180-4.  Only used to compute the image digest embedded in the
 * RT1180 AHAB container header (see mkahab.c), so it favors clarity over
 * speed.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <string.h>

#include "sha256.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ROTR(x, n) (((x) >> (n)) | ((x) << (32 - (n))))

#define CH(x, y, z)  (((x) & (y)) ^ (~(x) & (z)))
#define MAJ(x, y, z) (((x) & (y)) ^ ((x) & (z)) ^ ((y) & (z)))

#define BSIG0(x) (ROTR(x, 2)  ^ ROTR(x, 13) ^ ROTR(x, 22))
#define BSIG1(x) (ROTR(x, 6)  ^ ROTR(x, 11) ^ ROTR(x, 25))
#define SSIG0(x) (ROTR(x, 7)  ^ ROTR(x, 18) ^ ((x) >> 3))
#define SSIG1(x) (ROTR(x, 17) ^ ROTR(x, 19) ^ ((x) >> 10))

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint32_t g_sha256_k[64] =
{
  0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5,
  0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
  0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
  0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
  0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
  0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
  0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
  0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
  0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
  0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
  0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3,
  0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
  0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5,
  0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
  0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
  0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void sha256_transform(struct sha256_ctx_s *ctx,
                              const uint8_t block[64])
{
  uint32_t w[64];
  uint32_t a;
  uint32_t b;
  uint32_t c;
  uint32_t d;
  uint32_t e;
  uint32_t f;
  uint32_t g;
  uint32_t h;
  uint32_t t1;
  uint32_t t2;
  int i;

  for (i = 0; i < 16; i++)
    {
      w[i] = ((uint32_t)block[i * 4]     << 24) |
             ((uint32_t)block[i * 4 + 1] << 16) |
             ((uint32_t)block[i * 4 + 2] << 8)  |
             ((uint32_t)block[i * 4 + 3]);
    }

  for (i = 16; i < 64; i++)
    {
      w[i] = SSIG1(w[i - 2]) + w[i - 7] + SSIG0(w[i - 15]) + w[i - 16];
    }

  a = ctx->state[0];
  b = ctx->state[1];
  c = ctx->state[2];
  d = ctx->state[3];
  e = ctx->state[4];
  f = ctx->state[5];
  g = ctx->state[6];
  h = ctx->state[7];

  for (i = 0; i < 64; i++)
    {
      t1 = h + BSIG1(e) + CH(e, f, g) + g_sha256_k[i] + w[i];
      t2 = BSIG0(a) + MAJ(a, b, c);
      h = g;
      g = f;
      f = e;
      e = d + t1;
      d = c;
      c = b;
      b = a;
      a = t1 + t2;
    }

  ctx->state[0] += a;
  ctx->state[1] += b;
  ctx->state[2] += c;
  ctx->state[3] += d;
  ctx->state[4] += e;
  ctx->state[5] += f;
  ctx->state[6] += g;
  ctx->state[7] += h;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void sha256_init(struct sha256_ctx_s *ctx)
{
  ctx->state[0] = 0x6a09e667;
  ctx->state[1] = 0xbb67ae85;
  ctx->state[2] = 0x3c6ef372;
  ctx->state[3] = 0xa54ff53a;
  ctx->state[4] = 0x510e527f;
  ctx->state[5] = 0x9b05688c;
  ctx->state[6] = 0x1f83d9ab;
  ctx->state[7] = 0x5be0cd19;
  ctx->bitcount = 0;
}

void sha256_update(struct sha256_ctx_s *ctx, const void *data, size_t len)
{
  const uint8_t *p = data;
  size_t buf_used = (size_t)((ctx->bitcount / 8) % 64);

  ctx->bitcount += (uint64_t)len * 8;

  while (len > 0)
    {
      size_t n = 64 - buf_used;

      if (n > len)
        {
          n = len;
        }

      memcpy(ctx->buf + buf_used, p, n);
      buf_used += n;
      p += n;
      len -= n;

      if (buf_used == 64)
        {
          sha256_transform(ctx, ctx->buf);
          buf_used = 0;
        }
    }
}

void sha256_final(struct sha256_ctx_s *ctx,
                   uint8_t digest[SHA256_DIGEST_SIZE])
{
  size_t buf_used = (size_t)((ctx->bitcount / 8) % 64);
  uint64_t bitcount = ctx->bitcount;
  uint8_t pad = 0x80;
  int i;

  sha256_update(ctx, &pad, 1);

  buf_used = (size_t)((ctx->bitcount / 8) % 64);
  while (buf_used != 56)
    {
      uint8_t zero = 0;

      sha256_update(ctx, &zero, 1);
      buf_used = (size_t)((ctx->bitcount / 8) % 64);
    }

  {
    uint8_t lenbytes[8];

    for (i = 0; i < 8; i++)
      {
        lenbytes[i] = (uint8_t)(bitcount >> (56 - i * 8));
      }

    /* Append length directly without going through sha256_update()'s
     * bitcount accounting (the length field itself is not counted).
     */

    memcpy(ctx->buf + 56, lenbytes, 8);
    sha256_transform(ctx, ctx->buf);
  }

  for (i = 0; i < 8; i++)
    {
      digest[i * 4]     = (uint8_t)(ctx->state[i] >> 24);
      digest[i * 4 + 1] = (uint8_t)(ctx->state[i] >> 16);
      digest[i * 4 + 2] = (uint8_t)(ctx->state[i] >> 8);
      digest[i * 4 + 3] = (uint8_t)(ctx->state[i]);
    }
}

void sha256_buffer(const void *data, size_t len,
                    uint8_t digest[SHA256_DIGEST_SIZE])
{
  struct sha256_ctx_s ctx;

  sha256_init(&ctx);
  sha256_update(&ctx, data, len);
  sha256_final(&ctx, digest);
}
