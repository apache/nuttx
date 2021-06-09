/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_sha.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_ESP32C3_SHA_ACCELERATOR

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <debug.h>
#include <semaphore.h>

#include "riscv_arch.h"
#include "hardware/esp32c3_sha.h"
#include "hardware/esp32c3_system.h"

#include "esp32c3_sha.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PUT_UINT32_BE(n,b,i)                      \
{                                                 \
  (b)[(i)] = (unsigned char) ((n) >> 24);         \
  (b)[(i) + 1] = (unsigned char) ((n) >> 16);     \
  (b)[(i) + 2] = (unsigned char) ((n) >>  8);     \
  (b)[(i) + 3] = (unsigned char) ((n));           \
}

#define GET_UINT64_BE(n,b,i)                    \
{                                               \
  (n) = ((uint64_t) (b)[(i)] << 56)             \
    | ((uint64_t) (b)[(i) + 1] << 48)           \
    | ((uint64_t) (b)[(i) + 2] << 40)           \
    | ((uint64_t) (b)[(i) + 3] << 32)           \
    | ((uint64_t) (b)[(i) + 4] << 24)           \
    | ((uint64_t) (b)[(i) + 5] << 16)           \
    | ((uint64_t) (b)[(i) + 6] <<  8)           \
    | ((uint64_t) (b)[(i) + 7]);                \
}

#define PUT_UINT64_BE(n,b,i)                      \
{                                                 \
  (b)[(i)] = (uint8_t) ((n) >> 56);               \
  (b)[(i) + 1] = (uint8_t) ((n) >> 48);           \
  (b)[(i) + 2] = (uint8_t) ((n) >> 40);           \
  (b)[(i) + 3] = (uint8_t) ((n) >> 32);           \
  (b)[(i) + 4] = (uint8_t) ((n) >> 24);           \
  (b)[(i) + 5] = (uint8_t) ((n) >> 16);           \
  (b)[(i) + 6] = (uint8_t) ((n) >>  8);           \
  (b)[(i) + 7] = (uint8_t) ((n));                 \
}

#define SHR(x,n)  ((x) >> (n))
#define ROTR(x,n) (SHR((x),(n)) | ((x) << (64 - (n))))

#define S0(x) (ROTR(x, 1) ^ ROTR(x, 8) ^  SHR(x, 7))
#define S1(x) (ROTR(x,19) ^ ROTR(x,61) ^  SHR(x, 6))

#define S2(x) (ROTR(x,28) ^ ROTR(x,34) ^ ROTR(x,39))
#define S3(x) (ROTR(x,14) ^ ROTR(x,18) ^ ROTR(x,41))

#define F0(x,y,z) (((x) & (y)) | ((z) & ((x) | (y))))
#define F1(x,y,z) ((z) ^ ((x) & ((y) ^ (z))))

#define P(a,b,c,d,e,f,g,h,x,K)                            \
  do                                                      \
  {                                                       \
    temp1 = (h) + S3(e) + F1((e),(f),(g)) + (K) + (x);    \
    temp2 = S2(a) + F0((a),(b),(c));                      \
    (d) += temp1;                                         \
    (h) = temp1 + temp2;                                  \
  } while(0)

#define SHA1_BLK_SIZE                    (20)
#define SHA2_BLK_SIZE                    (32)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_sha_inited;
static sem_t g_sha_sem = SEM_INITIALIZER(1);
static const unsigned char esp32c3_sha_padding[64] =
{
  0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

static const uint64_t K[80] =
{
  UINT64_C(0x428a2f98d728ae22),  UINT64_C(0x7137449123ef65cd),
  UINT64_C(0xb5c0fbcfec4d3b2f),  UINT64_C(0xe9b5dba58189dbbc),
  UINT64_C(0x3956c25bf348b538),  UINT64_C(0x59f111f1b605d019),
  UINT64_C(0x923f82a4af194f9b),  UINT64_C(0xab1c5ed5da6d8118),
  UINT64_C(0xd807aa98a3030242),  UINT64_C(0x12835b0145706fbe),
  UINT64_C(0x243185be4ee4b28c),  UINT64_C(0x550c7dc3d5ffb4e2),
  UINT64_C(0x72be5d74f27b896f),  UINT64_C(0x80deb1fe3b1696b1),
  UINT64_C(0x9bdc06a725c71235),  UINT64_C(0xc19bf174cf692694),
  UINT64_C(0xe49b69c19ef14ad2),  UINT64_C(0xefbe4786384f25e3),
  UINT64_C(0x0fc19dc68b8cd5b5),  UINT64_C(0x240ca1cc77ac9c65),
  UINT64_C(0x2de92c6f592b0275),  UINT64_C(0x4a7484aa6ea6e483),
  UINT64_C(0x5cb0a9dcbd41fbd4),  UINT64_C(0x76f988da831153b5),
  UINT64_C(0x983e5152ee66dfab),  UINT64_C(0xa831c66d2db43210),
  UINT64_C(0xb00327c898fb213f),  UINT64_C(0xbf597fc7beef0ee4),
  UINT64_C(0xc6e00bf33da88fc2),  UINT64_C(0xd5a79147930aa725),
  UINT64_C(0x06ca6351e003826f),  UINT64_C(0x142929670a0e6e70),
  UINT64_C(0x27b70a8546d22ffc),  UINT64_C(0x2e1b21385c26c926),
  UINT64_C(0x4d2c6dfc5ac42aed),  UINT64_C(0x53380d139d95b3df),
  UINT64_C(0x650a73548baf63de),  UINT64_C(0x766a0abb3c77b2a8),
  UINT64_C(0x81c2c92e47edaee6),  UINT64_C(0x92722c851482353b),
  UINT64_C(0xa2bfe8a14cf10364),  UINT64_C(0xa81a664bbc423001),
  UINT64_C(0xc24b8b70d0f89791),  UINT64_C(0xc76c51a30654be30),
  UINT64_C(0xd192e819d6ef5218),  UINT64_C(0xd69906245565a910),
  UINT64_C(0xf40e35855771202a),  UINT64_C(0x106aa07032bbd1b8),
  UINT64_C(0x19a4c116b8d2d0c8),  UINT64_C(0x1e376c085141ab53),
  UINT64_C(0x2748774cdf8eeb99),  UINT64_C(0x34b0bcb5e19b48a8),
  UINT64_C(0x391c0cb3c5c95a63),  UINT64_C(0x4ed8aa4ae3418acb),
  UINT64_C(0x5b9cca4f7763e373),  UINT64_C(0x682e6ff3d6b2b8a3),
  UINT64_C(0x748f82ee5defb2fc),  UINT64_C(0x78a5636f43172f60),
  UINT64_C(0x84c87814a1f0ab72),  UINT64_C(0x8cc702081a6439ec),
  UINT64_C(0x90befffa23631e28),  UINT64_C(0xa4506cebde82bde9),
  UINT64_C(0xbef9a3f7b2c67915),  UINT64_C(0xc67178f2e372532b),
  UINT64_C(0xca273eceea26619c),  UINT64_C(0xd186b8c721c0c207),
  UINT64_C(0xeada7dd6cde0eb1e),  UINT64_C(0xf57d4f7fee6ed178),
  UINT64_C(0x06f067aa72176fba),  UINT64_C(0x0a637dc5a2c898a6),
  UINT64_C(0x113f9804bef90dae),  UINT64_C(0x1b710b35131c471b),
  UINT64_C(0x28db77f523047d84),  UINT64_C(0x32caab7b40c72493),
  UINT64_C(0x3c9ebe0a15c9bebc),  UINT64_C(0x431d67c49c100d4c),
  UINT64_C(0x4cc5d4becb3e42b6),  UINT64_C(0x597f299cfc657e2a),
  UINT64_C(0x5fcb6fab3ad6faec),  UINT64_C(0x6c44198c4a475817)
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_sha1_block
 *
 * Description:
 *   Performs SHA1 on multiple blocks at a time.
 *
 * Input Parameters:
 *   ctx      - The SHA1 context
 *   data     - Input message to be hashed on single block
 *   len      - Length of the input message on single block
 *   buf      - Input message to be hashed on multiple blocks
 *   buf_len  - Length of the input message on multiple blocks
 *
 * Returned Value:
 *   OK is returned on success.
 *
 ****************************************************************************/

static int esp32c3_sha1_block(struct esp32c3_sha1_context_s *ctx,
                              const uint8_t *data, size_t len,
                              uint8_t *buf, size_t buf_len)
{
  uint32_t *data_words = NULL;
  size_t blk_len = 0;
  size_t blk_word_len = 0;
  int num_block = 0;
  int i;
  int j;

  blk_len = 64;
  blk_word_len =  blk_len / 4;
  num_block = len / blk_len;

  putreg32(ctx->mode, SHA_MODE_REG);

  if (buf_len != 0)
    {
      data_words = (uint32_t *)buf;

      while (getreg32(SHA_BUSY_REG))
        {
        }

      for (i = 0; i < blk_word_len; i++)
        {
          putreg32(data_words[i], SHA_M_0_REG + i * 4);
        }

      if (ctx->first_block)
        {
          putreg32(1, SHA_START_REG);
        }
      else
        {
          putreg32(1, SHA_CONTINUE_REG);
        }

      ctx->first_block = false;
    }

  for (j = 0; j < num_block; j++)
    {
      data_words = (uint32_t *)(data + blk_len * j);

      while (getreg32(SHA_BUSY_REG))
        {
        }

      for (i = 0; i < blk_word_len; i++)
        {
          putreg32(data_words[i], SHA_M_0_REG + i * 4);
        }

      if (ctx->first_block)
        {
          putreg32(1, SHA_START_REG);
        }
      else
        {
          putreg32(1, SHA_CONTINUE_REG);
        }

      ctx->first_block = false;
    }

  while (getreg32(SHA_BUSY_REG))
    {
    }

  for (i = 0; i < 5; i ++)
    {
      ctx->state[i] = getreg32(SHA_H_0_REG + i * 4);
    }

  return OK;
}

/****************************************************************************
 * Name: esp32c3_sha256_block
 *
 * Description:
 *   Performs SHA256 on multiple blocks at a time.
 *
 * Input Parameters:
 *   ctx      - The SHA256 context
 *   data     - Input message to be hashed on single block
 *   len      - Length of the input message on single block
 *   buf      - Input message to be hashed on multiple blocks
 *   buf_len  - Length of the input message on multiple blocks
 *
 * Returned Value:
 *   OK is returned on success.
 *
 ****************************************************************************/

static int esp32c3_sha256_block(struct esp32c3_sha256_context_s *ctx,
                                const uint8_t *data, size_t len,
                                uint8_t *buf, size_t buf_len)
{
  uint32_t *data_words = NULL;
  size_t blk_len = 0;
  size_t blk_word_len = 0;
  int num_block = 0;
  int i;
  int j;

  blk_len = 64;
  blk_word_len =  blk_len / 4;
  num_block = len / blk_len;

  putreg32(ctx->mode, SHA_MODE_REG);

  if (buf_len != 0)
    {
      data_words = (uint32_t *)buf;

      while (getreg32(SHA_BUSY_REG))
        {
        }

      for (i = 0; i < blk_word_len; i++)
        {
          putreg32(data_words[i], SHA_M_0_REG + i * 4);
        }

      if (ctx->first_block)
        {
          putreg32(1, SHA_START_REG);
        }
      else
        {
          putreg32(1, SHA_CONTINUE_REG);
        }

      ctx->first_block = false;
    }

  for (j = 0; j < num_block; j++)
    {
      data_words = (uint32_t *)(data + blk_len * j);

      while (getreg32(SHA_BUSY_REG))
        {
        }

      for (i = 0; i < blk_word_len; i++)
        {
          putreg32(data_words[i], SHA_M_0_REG + i * 4);
        }

      if (ctx->first_block)
        {
          putreg32(1, SHA_START_REG);
        }
      else
        {
          putreg32(1, SHA_CONTINUE_REG);
        }

      ctx->first_block = false;
    }

  while (getreg32(SHA_BUSY_REG))
    {
    }

  if (ctx->mode == ESP32C3_SHA2_256)
    {
      num_block = 8;
    }
  else
    {
      num_block = 7;
    }

  for (i = 0; i < num_block; i ++)
    {
      ctx->state[i] = getreg32(SHA_H_0_REG + i * 4);
    }

  return OK;
}

/****************************************************************************
 * Name: esp32c3_sha512_block
 *
 * Description:
 *   Performs SHA512 on multiple blocks at a time.
 *
 * Input Parameters:
 *   ctx      - The SHA512 context
 *   data     - Input message to be hashed
 *
 * Returned Value:
 *   OK is returned on success.
 *
 ****************************************************************************/

static int esp32c3_sha512_block(struct esp32c3_sha512_context_s *ctx,
                                const uint8_t *data)
{
  int i;
  int j;
  uint64_t temp1, temp2, W[80];
  uint64_t A, B, C, D, E, F, G, H;

  for (i = 0; i < 16; i++)
    {
      GET_UINT64_BE(W[i], data, i << 3);
    }

  for (; i < 80; i++)
    {
      W[i] = S1(W[i -  2]) + W[i -  7] + S0(W[i - 15]) + W[i - 16];
    }

  A = ctx->state[0];
  B = ctx->state[1];
  C = ctx->state[2];
  D = ctx->state[3];
  E = ctx->state[4];
  F = ctx->state[5];
  G = ctx->state[6];
  H = ctx->state[7];
  i = 0;
  j = 0;

  do
    {
      P(A, B, C, D, E, F, G, H, W[i++], K[j++]);
      P(H, A, B, C, D, E, F, G, W[i++], K[j++]);
      P(G, H, A, B, C, D, E, F, W[i++], K[j++]);
      P(F, G, H, A, B, C, D, E, W[i++], K[j++]);
      P(E, F, G, H, A, B, C, D, W[i++], K[j++]);
      P(D, E, F, G, H, A, B, C, W[i++], K[j++]);
      P(C, D, E, F, G, H, A, B, W[i++], K[j++]);
      P(B, C, D, E, F, G, H, A, W[i++], K[j++]);
    }
  while (i < 80);

  ctx->state[0] += A;
  ctx->state[1] += B;
  ctx->state[2] += C;
  ctx->state[3] += D;
  ctx->state[4] += E;
  ctx->state[5] += F;
  ctx->state[6] += G;
  ctx->state[7] += H;

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_sha1_init
 *
 * Description:
 *   Initializes a SHA-1 context.
 *
 * Input Parameters:
 *   ctx      - The SHA-1 context to initialize
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32c3_sha1_init(struct esp32c3_sha1_context_s *ctx)
{
  memset(ctx, 0, sizeof(struct esp32c3_sha1_context_s));
}

/****************************************************************************
 * Name: esp32c3_sha1_starts
 *
 * Description:
 *   Starts a SHA-1 checksum calculation.
 *
 * Input Parameters:
 *   ctx      - The SHA-1 context to initialize
 *
 * Returned Value:
 *   OK is returned on success.
 *
 ****************************************************************************/

int esp32c3_sha1_starts(struct esp32c3_sha1_context_s *ctx)
{
  memset(ctx, 0, sizeof(struct esp32c3_sha1_context_s));
  ctx->mode = ESP32C3_SHA1_1;

  return OK;
}

/****************************************************************************
 * Name: esp32c3_sha1_update
 *
 * Description:
 *   Feeds an input buffer into an ongoing SHA-1 checksum calculation.
 *
 * Input Parameters:
 *   ctx      - The SHA-1 context to use
 *   input    - The buffer holding the input data
 *   ilen     - The length of the input data in Bytes
 *
 * Returned Value:
 *   OK is returned on success.
 *   Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

int esp32c3_sha1_update(struct esp32c3_sha1_context_s *ctx,
                        const unsigned char *input,
                        size_t ilen)
{
  int ret;
  size_t fill;
  uint32_t left;
  uint32_t len;
  uint32_t local_len = 0;
  int i;

  if (!ilen || (input == NULL))
    {
      return OK;
    }

  left = ctx->total[0] & 0x3f;
  fill = 64 - left;

  ctx->total[0] += ilen;
  ctx->total[0] &= UINT32_MAX;

  if (ctx->total[0] < ilen)
    {
      ctx->total[1]++;
    }

  if (left && ilen >= fill)
    {
      memcpy((void *) (ctx->buffer + left), input, fill);

      input    += fill;
      ilen     -= fill;
      left      = 0;
      local_len = 64;
    }

  len = (ilen / 64) * 64;
  if (len || local_len)
    {
      ret = nxsem_wait(&g_sha_sem);
      if (ret < 0)
        {
          return ret;
        }

      if (ctx->sha_state == ESP32C3_SHA_STATE_INIT)
        {
          ctx->first_block = true;

          ctx->sha_state = ESP32C3_SHA_STATE_IN_PROCESS;
        }
      else if (ctx->sha_state == ESP32C3_SHA_STATE_IN_PROCESS)
        {
          ctx->first_block = false;
          for (i = 0; i < 5; i++)
            {
              putreg32(ctx->state[i], SHA_H_0_REG + i * 4);
            }
        }

      ret = esp32c3_sha1_block(ctx, input, len, ctx->buffer, local_len);
      ret |= nxsem_post(&g_sha_sem);

      if (ret != 0)
        {
          return ret;
        }
    }

  if (ilen > 0)
    {
      memcpy((void *) (ctx->buffer + left), input + len, ilen - len);
    }

  return OK;
}

/****************************************************************************
 * Name: esp32c3_sha1_finish
 *
 * Description:
 *   Finishes the SHA-1 operation,
 *   and writes the result to the output buffer.
 *
 * Input Parameters:
 *   ctx      - The SHA-1 context to use
 *   output   - The SHA-1 checksum result
 *
 * Returned Value:
 *   OK is returned on success.
 *   Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

int esp32c3_sha1_finish(struct esp32c3_sha1_context_s *ctx,
                        unsigned char output[20])
{
  int ret;
  uint32_t last;
  uint32_t padn;
  uint32_t high;
  uint32_t low;
  unsigned char msglen[8];

  high = (ctx->total[0] >> 29) | (ctx->total[1] <<  3);
  low  = (ctx->total[0] <<  3);

  PUT_UINT32_BE(high, msglen, 0);
  PUT_UINT32_BE(low,  msglen, 4);

  last = ctx->total[0] & 0x3f;
  padn = (last < 56) ? (56 - last) : (120 - last);

  ret = esp32c3_sha1_update(ctx, esp32c3_sha_padding, padn);
  if (ret != 0)
    {
      return ret;
    }

  ret = esp32c3_sha1_update(ctx, msglen, 8);
  if (ret != 0)
    {
      return ret;
    }

  memcpy(output, ctx->state, SHA1_BLK_SIZE);

  return ret;
}

/****************************************************************************
 * Name: esp32c3_sha1_free
 *
 * Description:
 *   Clears a SHA-1 context.
 *
 * Input Parameters:
 *   ctx      - The SHA-1 context to clear
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32c3_sha1_free(struct esp32c3_sha1_context_s *ctx)
{
  if (ctx == NULL)
    {
      return;
    }

  memset(ctx, 0, sizeof(struct esp32c3_sha1_context_s));
}

/****************************************************************************
 * Name: esp32c3_sha256_init
 *
 * Description:
 *   Initializes a SHA-256 context.
 *
 * Input Parameters:
 *   ctx      - The SHA-256 context to initialize
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32c3_sha256_init(struct esp32c3_sha256_context_s *ctx)
{
  memset(ctx, 0, sizeof(struct esp32c3_sha256_context_s));
}

/****************************************************************************
 * Name: esp32c3_sha256_starts
 *
 * Description:
 *   Starts a SHA-224 or SHA-256 checksum calculation.
 *
 * Input Parameters:
 *   ctx      - The SHA-256 context to initialize
 *   is224    - Determines which function to use
 *
 * Returned Value:
 *   OK is returned on success.
 *
 ****************************************************************************/

int esp32c3_sha256_starts(struct esp32c3_sha256_context_s *ctx, bool is224)
{
  memset(ctx, 0, sizeof(struct esp32c3_sha256_context_s));

  if (is224)
    {
      ctx->mode = ESP32C3_SHA2_224;
    }
  else
    {
      ctx->mode = ESP32C3_SHA2_256;
    }

  return OK;
}

/****************************************************************************
 * Name: esp32c3_sha256_update
 *
 * Description:
 *   Feeds an input buffer into an ongoing SHA-224 or SHA-256
 *   checksum calculation.
 *
 * Input Parameters:
 *   ctx      - The SHA-256 context to use
 *   input    - The buffer holding the input data
 *   ilen     - The length of the input data in Bytes
 *
 * Returned Value:
 *   OK is returned on success.
 *   Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

int esp32c3_sha256_update(struct esp32c3_sha256_context_s *ctx,
                          const unsigned char *input,
                          size_t ilen)
{
  int ret = 0;
  size_t fill;
  uint32_t left;
  uint32_t len;
  uint32_t local_len = 0;
  int i;

  if (ilen == 0)
    {
      return OK;
    }

  left = ctx->total[0] & 0x3f;
  fill = 64 - left;

  ctx->total[0] += ilen;
  ctx->total[0] &= UINT32_MAX;

  if (ctx->total[0] < ilen)
    {
      ctx->total[1]++;
    }

  /* Check if any data pending from previous call to this API */

  if (left && ilen >= fill)
    {
      memcpy((void *) (ctx->buffer + left), input, fill);

      input    += fill;
      ilen     -= fill;
      left      = 0;
      local_len = 64;
    }

  len = (ilen / 64) * 64;

  if (len || local_len)
    {
      ret = nxsem_wait(&g_sha_sem);
      if (ret < 0)
        {
          return ret;
        }

      if (ctx->sha_state == ESP32C3_SHA_STATE_INIT)
        {
          ctx->first_block = true;
          ctx->sha_state = ESP32C3_SHA_STATE_IN_PROCESS;
        }
      else if (ctx->sha_state == ESP32C3_SHA_STATE_IN_PROCESS)
        {
          ctx->first_block = false;
          int block_num = (ctx->mode == ESP32C3_SHA2_224) ? 7 : 8;
          for (i = 0; i < block_num; i++)
            {
              putreg32(ctx->state[i], SHA_H_0_REG + i * 4);
            }
        }

      ret = esp32c3_sha256_block(ctx, input, len,  ctx->buffer, local_len);
      ret |= nxsem_post(&g_sha_sem);

      if (ret != 0)
        {
          return ret;
        }
    }

  if (ilen > 0)
    {
      memcpy((void *) (ctx->buffer + left), input + len, ilen - len);
    }

  return OK;
}

/****************************************************************************
 * Name: esp32c3_sha256_finish
 *
 * Description:
 *   Finishes the SHA-224 or SHA-256 operation, and writes the result to
 *   the output buffer.
 *
 * Input Parameters:
 *   ctx      - The SHA-256 context to use
 *   output   - The SHA-256 checksum result
 *
 * Returned Value:
 *   OK is returned on success.
 *   Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

int esp32c3_sha256_finish(struct esp32c3_sha256_context_s *ctx,
                          unsigned char output[32])
{
  int ret;
  uint32_t last;
  uint32_t padn;
  uint32_t high;
  uint64_t low;
  unsigned char msglen[8];

  high = (ctx->total[0] >> 29) | (ctx->total[1] << 3);
  low  = (ctx->total[0] << 3);

  PUT_UINT32_BE(high, msglen, 0);
  PUT_UINT32_BE(low,  msglen, 4);

  last = ctx->total[0] & 0x3f;
  padn = (last < 56) ? (56 - last) : (120 - last);

  ret = esp32c3_sha256_update(ctx, esp32c3_sha_padding, padn);
  if (ret != 0)
    {
      return ret;
    }

  ret = esp32c3_sha256_update(ctx, msglen, 8);
  if (ret != 0)
    {
      return ret;
    }

  memcpy(output, ctx->state, SHA2_BLK_SIZE);

  return ret;
}

/****************************************************************************
 * Name: esp32c3_sha256_free
 *
 * Description:
 *   Clears a SHA-256 context.
 *
 * Input Parameters:
 *   ctx      - The SHA-256 context to clear
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32c3_sha256_free(struct esp32c3_sha256_context_s *ctx)
{
  if (ctx == NULL)
    {
      return;
    }

  memset(ctx, 0, sizeof(struct esp32c3_sha256_context_s));
}

/****************************************************************************
 * Name: esp32c3_sha512_init
 *
 * Description:
 *   Initializes a SHA-512 context.
 *
 * Input Parameters:
 *   ctx      - The SHA-512 context to initialize
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32c3_sha512_init(struct esp32c3_sha512_context_s *ctx)
{
  memset(ctx, 0, sizeof(struct esp32c3_sha512_context_s));
}

/****************************************************************************
 * Name: esp32c3_sha512_starts
 *
 * Description:
 *   Starts a SHA-384 or SHA-512 checksum calculation.
 *
 * Input Parameters:
 *   ctx      - The SHA-512 context to initialize
 *   is384    - Determines which function to use
 *
 * Returned Value:
 *   OK is returned on success.
 *
 ****************************************************************************/

int esp32c3_sha512_starts(struct esp32c3_sha512_context_s *ctx, bool is384)
{
  int ret = 0;

  if (is384)
    {
      ctx->mode = ESP32C3_SHA3_384;

      ctx->total[0] = 0;
      ctx->total[1] = 0;

      ctx->state[0] = UINT64_C(0xcbbb9d5dc1059ed8);
      ctx->state[1] = UINT64_C(0x629a292a367cd507);
      ctx->state[2] = UINT64_C(0x9159015a3070dd17);
      ctx->state[3] = UINT64_C(0x152fecd8f70e5939);
      ctx->state[4] = UINT64_C(0x67332667ffc00b31);
      ctx->state[5] = UINT64_C(0x8eb44a8768581511);
      ctx->state[6] = UINT64_C(0xdb0c2e0d64f98fa7);
      ctx->state[7] = UINT64_C(0x47b5481dbefa4fa4);
    }
  else
    {
      ctx->mode = ESP32C3_SHA3_512;

      ctx->total[0] = 0;
      ctx->total[1] = 0;

      ctx->state[0] = UINT64_C(0x6a09e667f3bcc908);
      ctx->state[1] = UINT64_C(0xbb67ae8584caa73b);
      ctx->state[2] = UINT64_C(0x3c6ef372fe94f82b);
      ctx->state[3] = UINT64_C(0xa54ff53a5f1d36f1);
      ctx->state[4] = UINT64_C(0x510e527fade682d1);
      ctx->state[5] = UINT64_C(0x9b05688c2b3e6c1f);
      ctx->state[6] = UINT64_C(0x1f83d9abfb41bd6b);
      ctx->state[7] = UINT64_C(0x5be0cd19137e2179);
    }

  return ret;
}

/****************************************************************************
 * Name: esp32c3_sha512_update
 *
 * Description:
 *   Feeds an input buffer into an ongoing SHA-384 or SHA-512
 *   checksum calculation.
 *
 * Input Parameters:
 *   ctx      - The SHA-512 context to use
 *   input    - The buffer holding the input data
 *   ilen     - The length of the input data in Bytes
 *
 * Returned Value:
 *   OK is returned on success.
 *
 ****************************************************************************/

int esp32c3_sha512_update(struct esp32c3_sha512_context_s *ctx,
                          const unsigned char *input,
                          size_t ilen)
{
  size_t fill;
  uint32_t left;
  const uint8_t *input_buffer = (const uint8_t *)input;

  left = (uint32_t) (ctx->total[0] & 0x7f);
  fill = 128 - left;

  ctx->total[0] += (uint64_t)ilen;

  if (ctx->total[0] < (uint64_t)ilen)
    {
      ctx->total[1]++;
    }

  if (left && ilen >= fill)
    {
      memcpy(ctx->buffer + left, input_buffer, fill);

      esp32c3_sha512_block(ctx, ctx->buffer);

      input_buffer += fill;
      ilen         -= fill;
      left          = 0;
    }

  while (ilen >= 128)
    {
      esp32c3_sha512_block(ctx, input_buffer);

      input_buffer += 128;
      ilen  -= 128;
    }

  if (ilen > 0)
    {
      memcpy((void *) (ctx->buffer + left), input_buffer, ilen);
    }

  return OK;
}

/****************************************************************************
 * Name: esp32c3_sha512_finish
 *
 * Description:
 *   Finishes the SHA-384 or SHA-512 operation, and writes the result to
 *   the output buffer.
 *
 * Input Parameters:
 *   ctx      - The SHA-512 context to use
 *   output   - The SHA-512 checksum result
 *
 * Returned Value:
 *   OK is returned on success.
 *
 ****************************************************************************/

int esp32c3_sha512_finish(struct esp32c3_sha512_context_s *ctx,
                          unsigned char output[64])
{
  uint8_t used;
  uint64_t high;
  uint64_t low;
  uint8_t *output_buffer = (uint8_t *)output;

  used = ctx->total[0] & 0x7f;
  ctx->buffer[used++] = 0x80;

  if (used <= 112)
    {
      memset(ctx->buffer + used, 0, 112 - used);
    }
  else
    {
      memset(ctx->buffer + used, 0, 128 - used);
      esp32c3_sha512_block(ctx, ctx->buffer);
      memset(ctx->buffer, 0, 112);
    }

  high = (ctx->total[0] >> 61) | (ctx->total[1] <<  3);
  low  = (ctx->total[0] <<  3);

  PUT_UINT64_BE(high, ctx->buffer, 112);
  PUT_UINT64_BE(low,  ctx->buffer, 120);

  esp32c3_sha512_block(ctx, ctx->buffer);

  PUT_UINT64_BE(ctx->state[0], output_buffer,  0);
  PUT_UINT64_BE(ctx->state[1], output_buffer,  8);
  PUT_UINT64_BE(ctx->state[2], output_buffer, 16);
  PUT_UINT64_BE(ctx->state[3], output_buffer, 24);
  PUT_UINT64_BE(ctx->state[4], output_buffer, 32);
  PUT_UINT64_BE(ctx->state[5], output_buffer, 40);

  if (ctx->mode == ESP32C3_SHA3_512)
    {
      PUT_UINT64_BE(ctx->state[6], output_buffer, 48);
      PUT_UINT64_BE(ctx->state[7], output_buffer, 56);
    }

  return OK;
}

/****************************************************************************
 * Name: esp32c3_sha512_free
 *
 * Description:
 *   Clears a SHA-512 context.
 *
 * Input Parameters:
 *   ctx      - The SHA-512 context to clear
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32c3_sha512_free(struct esp32c3_sha512_context_s *ctx)
{
  if (ctx == NULL)
    {
      return;
    }

  memset(ctx, 0, sizeof(struct esp32c3_sha1_context_s));
}

/****************************************************************************
 * Name: esp32c3_sha_init
 *
 * Description:
 *   Initialize ESP32-C3 SHA hardware.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK is returned on success.
 *   Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

int esp32c3_sha_init(void)
{
  if (!g_sha_inited)
    {
      modifyreg32(SYSTEM_PERIP_CLK_EN1_REG, 0, SYSTEM_CRYPTO_SHA_CLK_EN);
      modifyreg32(SYSTEM_PERIP_RST_EN1_REG, SYSTEM_CRYPTO_SHA_RST, 0);
      g_sha_inited = true;
    }
  else
    {
      return -EBUSY;
    }

  return OK;
}

#endif

/****************************************************************************
 * Test Functions
 ****************************************************************************/

#ifdef CONFIG_ESP32C3_SHA_ACCELERATOR_TEST

/****************************************************************************
 * Name: esp32c3_sha1_self_test
 *
 * Description:
 *   Checkup routine
 *
 * Input Parameters:
 *   verbose        - The result output or not
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int esp32c3_sha1_self_test(bool verbose)
{
  int i;
  int j;
  int buflen;
  int ret = 0;
  unsigned char buf[1024];
  unsigned char sha1sum[20];
  struct esp32c3_sha1_context_s ctx;

  /* FIPS-180-1 test vectors */

  const unsigned char sha1_test_buf[3][57] =
    {
      {
        "abc"
      },

      {
        "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq"
      },

      {
        ""
      }
    };

  const size_t sha1_test_buflen[3] =
    {
      3, 56, 1000
    };

  const unsigned char sha1_test_sum[3][20] =
  {
    {
      0xa9, 0x99, 0x3e, 0x36, 0x47, 0x06, 0x81, 0x6a, 0xba, 0x3e,
      0x25, 0x71, 0x78, 0x50, 0xc2, 0x6c, 0x9c, 0xd0, 0xd8, 0x9d
    },

    {
      0x84, 0x98, 0x3e, 0x44, 0x1c, 0x3b, 0xd2, 0x6e, 0xba, 0xae,
      0x4a, 0xa1, 0xf9, 0x51, 0x29, 0xe5, 0xe5, 0x46, 0x70, 0xf1
    },

    {
      0x34, 0xaa, 0x97, 0x3c, 0xd4, 0xc4, 0xda, 0xa4, 0xf6, 0x1e,
      0xeb, 0x2b, 0xdb, 0xad, 0x27, 0x31, 0x65, 0x34, 0x01, 0x6f
    }
  };

  esp32c3_sha1_init(&ctx);

  for (i = 0; i < 3; i++)
    {
      if (verbose)
        {
          syslog(LOG_INFO, "  SHA-1 test #%d: ", i + 1);
        }

      ret = esp32c3_sha1_starts(&ctx);
      if (ret != 0)
        {
          goto fail;
        }

      if (i == 2)
        {
          memset(buf, 'a', buflen = 1000);

          for (j = 0; j < 1000; j++)
            {
              ret = esp32c3_sha1_update(&ctx, buf, buflen);
              if (ret != 0)
                {
                  goto fail;
                }
            }
        }
      else
        {
          ret = esp32c3_sha1_update(&ctx,
                                    sha1_test_buf[i],
                                    sha1_test_buflen[i]);
          if (ret != 0)
            {
              goto fail;
            }
        }

      ret = esp32c3_sha1_finish(&ctx, sha1sum);
      if (ret != 0)
        {
          goto fail;
        }

      if (memcmp(sha1sum, sha1_test_sum[i], 20) != 0)
        {
          ret = 1;
          goto fail;
        }

      if (verbose)
        {
          syslog(LOG_INFO, "passed\n");
        }
    }

  if (verbose)
    {
      syslog(LOG_INFO, "\n");
    }

  goto exit;

fail:
  if (verbose)
    {
      syslog(LOG_INFO, "failed\n");
    }

exit:
  esp32c3_sha1_free(&ctx);

  return (ret);
}

/****************************************************************************
 * Name: esp32c3_sha256_self_test
 *
 * Description:
 *   Checkup routine
 *
 * Input Parameters:
 *   verbose        - The result output or not
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int esp32c3_sha256_self_test(bool verbose)
{
  int i;
  int j;
  int k;
  int buflen;
  int ret = 0;
  unsigned char *buf;
  unsigned char sha256sum[32];
  struct esp32c3_sha256_context_s ctx;

  /* FIPS-180-2 test vectors */

  const unsigned char sha256_test_buf[3][57] =
    {
      {
        "abc"
      },

      {
        "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq"
      },

      {
        ""
      }
    };

  const size_t sha256_test_buflen[3] =
    {
      3, 56, 1000
    };

  const unsigned char sha256_test_sum[6][32] =
    {
      /* SHA-224 test vectors */

      {
        0x23, 0x09, 0x7d, 0x22, 0x34, 0x05, 0xd8, 0x22,
        0x86, 0x42, 0xa4, 0x77, 0xbd, 0xa2, 0x55, 0xb3,
        0x2a, 0xad, 0xbc, 0xe4, 0xbd, 0xa0, 0xb3, 0xf7,
        0xe3, 0x6c, 0x9d, 0xa7
      },

      {
        0x75, 0x38, 0x8b, 0x16, 0x51, 0x27, 0x76, 0xcc,
        0x5d, 0xba, 0x5d, 0xa1, 0xfd, 0x89, 0x01, 0x50,
        0xb0, 0xc6, 0x45, 0x5c, 0xb4, 0xf5, 0x8b, 0x19,
        0x52, 0x52, 0x25, 0x25
      },

      {
        0x20, 0x79, 0x46, 0x55, 0x98, 0x0c, 0x91, 0xd8,
        0xbb, 0xb4, 0xc1, 0xea, 0x97, 0x61, 0x8a, 0x4b,
        0xf0, 0x3f, 0x42, 0x58, 0x19, 0x48, 0xb2, 0xee,
        0x4e, 0xe7, 0xad, 0x67
      },

      /* SHA-256 test vectors */

      {
        0xba, 0x78, 0x16, 0xbf, 0x8f, 0x01, 0xcf, 0xea,
        0x41, 0x41, 0x40, 0xde, 0x5d, 0xae, 0x22, 0x23,
        0xb0, 0x03, 0x61, 0xa3, 0x96, 0x17, 0x7a, 0x9c,
        0xb4, 0x10, 0xff, 0x61, 0xf2, 0x00, 0x15, 0xad
      },

      {
        0x24, 0x8d, 0x6a, 0x61, 0xd2, 0x06, 0x38, 0xb8,
        0xe5, 0xc0, 0x26, 0x93, 0x0c, 0x3e, 0x60, 0x39,
        0xa3, 0x3c, 0xe4, 0x59, 0x64, 0xff, 0x21, 0x67,
        0xf6, 0xec, 0xed, 0xd4, 0x19, 0xdb, 0x06, 0xc1
      },

      {
        0xcd, 0xc7, 0x6e, 0x5c, 0x99, 0x14, 0xfb, 0x92,
        0x81, 0xa1, 0xc7, 0xe2, 0x84, 0xd7, 0x3e, 0x67,
        0xf1, 0x80, 0x9a, 0x48, 0xa4, 0x97, 0x20, 0x0e,
        0x04, 0x6d, 0x39, 0xcc, 0xc7, 0x11, 0x2c, 0xd0
      }
    };

  buf = calloc(1024, sizeof(unsigned char));
  if (NULL == buf)
    {
      if (verbose)
        {
          syslog(LOG_INFO, "Buffer allocation failed\n");
        }

      return (1);
    }

  esp32c3_sha256_init(&ctx);

  for (i = 0; i < 6; i++)
    {
      j = i % 3;
      k = i < 3;

      if (verbose)
        {
          syslog(LOG_INFO, "  SHA-%d test #%d: ", 256 - k * 32, j + 1);
        }

      ret = esp32c3_sha256_starts(&ctx, k);
      if (ret != 0)
        {
          goto fail;
        }

      if (j == 2)
        {
          memset(buf, 'a', buflen = 1000);

          for (j = 0; j < 1000; j++)
            {
              ret = esp32c3_sha256_update(&ctx, buf, buflen);
              if (ret != 0)
                {
                  goto fail;
                }
            }
        }
      else
        {
          ret = esp32c3_sha256_update(&ctx,
                                      sha256_test_buf[j],
                                      sha256_test_buflen[j]);
          if (ret != 0)
            {
              goto fail;
            }
        }

      ret = esp32c3_sha256_finish(&ctx, sha256sum);
      if (ret != 0)
        {
          goto fail;
        }

      if (memcmp(sha256sum, sha256_test_sum[i], 32 - k * 4) != 0)
        {
          ret = 1;
          goto fail;
        }

      if (verbose)
        {
          syslog(LOG_INFO, "passed\n");
        }
    }

  if (verbose)
    {
      syslog(LOG_INFO, "\n");
    }

  goto exit;

fail:
  if (verbose)
    {
      syslog(LOG_INFO, "failed\n");
    }

exit:
  esp32c3_sha256_free(&ctx);
  free(buf);

  return (ret);
}

/****************************************************************************
 * Name: esp32c3_sha512_self_test
 *
 * Description:
 *   Checkup routine
 *
 * Input Parameters:
 *   verbose        - The result output or not
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int esp32c3_sha512_self_test(bool verbose)
{
  int i;
  int j;
  int k;
  int buflen;
  int ret = 0;
  unsigned char *buf;
  unsigned char sha512sum[64];
  struct esp32c3_sha512_context_s ctx;

  /* FIPS-180-2 test vectors */

  const unsigned char sha512_test_buf[3][113] =
    {
      {
        "abc"
      },

      {
        "abcdefghbcdefghicdefghijdefghijkefghijklfghijklmghijklmn"
        "hijklmnoijklmnopjklmnopqklmnopqrlmnopqrsmnopqrstnopqrstu"
      },

      {
        ""
      }
    };

  const size_t sha512_test_buflen[3] =
    {
      3, 112, 1000
    };

  const unsigned char sha512_test_sum[6][64] =
    {
      /* SHA-384 test vectors */

      {
        0xcb, 0x00, 0x75, 0x3f, 0x45, 0xa3, 0x5e, 0x8b,
        0xb5, 0xa0, 0x3d, 0x69, 0x9a, 0xc6, 0x50, 0x07,
        0x27, 0x2c, 0x32, 0xab, 0x0e, 0xde, 0xd1, 0x63,
        0x1a, 0x8b, 0x60, 0x5a, 0x43, 0xff, 0x5b, 0xed,
        0x80, 0x86, 0x07, 0x2b, 0xa1, 0xe7, 0xcc, 0x23,
        0x58, 0xba, 0xec, 0xa1, 0x34, 0xc8, 0x25, 0xa7
      },

      {
        0x09, 0x33, 0x0c, 0x33, 0xf7, 0x11, 0x47, 0xe8,
        0x3d, 0x19, 0x2f, 0xc7, 0x82, 0xcd, 0x1b, 0x47,
        0x53, 0x11, 0x1b, 0x17, 0x3b, 0x3b, 0x05, 0xd2,
        0x2f, 0xa0, 0x80, 0x86, 0xe3, 0xb0, 0xf7, 0x12,
        0xfc, 0xc7, 0xc7, 0x1a, 0x55, 0x7e, 0x2d, 0xb9,
        0x66, 0xc3, 0xe9, 0xfa, 0x91, 0x74, 0x60, 0x39
      },

      {
        0x9d, 0x0e, 0x18, 0x09, 0x71, 0x64, 0x74, 0xcb,
        0x08, 0x6e, 0x83, 0x4e, 0x31, 0x0a, 0x4a, 0x1c,
        0xed, 0x14, 0x9e, 0x9c, 0x00, 0xf2, 0x48, 0x52,
        0x79, 0x72, 0xce, 0xc5, 0x70, 0x4c, 0x2a, 0x5b,
        0x07, 0xb8, 0xb3, 0xdc, 0x38, 0xec, 0xc4, 0xeb,
        0xae, 0x97, 0xdd, 0xd8, 0x7f, 0x3d, 0x89, 0x85
      },

      /* SHA-512 test vectors */

      {
        0xdd, 0xaf, 0x35, 0xa1, 0x93, 0x61, 0x7a, 0xba,
        0xcc, 0x41, 0x73, 0x49, 0xae, 0x20, 0x41, 0x31,
        0x12, 0xe6, 0xfa, 0x4e, 0x89, 0xa9, 0x7e, 0xa2,
        0x0a, 0x9e, 0xee, 0xe6, 0x4b, 0x55, 0xd3, 0x9a,
        0x21, 0x92, 0x99, 0x2a, 0x27, 0x4f, 0xc1, 0xa8,
        0x36, 0xba, 0x3c, 0x23, 0xa3, 0xfe, 0xeb, 0xbd,
        0x45, 0x4d, 0x44, 0x23, 0x64, 0x3c, 0xe8, 0x0e,
        0x2a, 0x9a, 0xc9, 0x4f, 0xa5, 0x4c, 0xa4, 0x9f
      },

      {
        0x8e, 0x95, 0x9b, 0x75, 0xda, 0xe3, 0x13, 0xda,
        0x8c, 0xf4, 0xf7, 0x28, 0x14, 0xfc, 0x14, 0x3f,
        0x8f, 0x77, 0x79, 0xc6, 0xeb, 0x9f, 0x7f, 0xa1,
        0x72, 0x99, 0xae, 0xad, 0xb6, 0x88, 0x90, 0x18,
        0x50, 0x1d, 0x28, 0x9e, 0x49, 0x00, 0xf7, 0xe4,
        0x33, 0x1b, 0x99, 0xde, 0xc4, 0xb5, 0x43, 0x3a,
        0xc7, 0xd3, 0x29, 0xee, 0xb6, 0xdd, 0x26, 0x54,
        0x5e, 0x96, 0xe5, 0x5b, 0x87, 0x4b, 0xe9, 0x09
      },

      {
        0xe7, 0x18, 0x48, 0x3d, 0x0c, 0xe7, 0x69, 0x64,
        0x4e, 0x2e, 0x42, 0xc7, 0xbc, 0x15, 0xb4, 0x63,
        0x8e, 0x1f, 0x98, 0xb1, 0x3b, 0x20, 0x44, 0x28,
        0x56, 0x32, 0xa8, 0x03, 0xaf, 0xa9, 0x73, 0xeb,
        0xde, 0x0f, 0xf2, 0x44, 0x87, 0x7e, 0xa6, 0x0a,
        0x4c, 0xb0, 0x43, 0x2c, 0xe5, 0x77, 0xc3, 0x1b,
        0xeb, 0x00, 0x9c, 0x5c, 0x2c, 0x49, 0xaa, 0x2e,
        0x4e, 0xad, 0xb2, 0x17, 0xad, 0x8c, 0xc0, 0x9b
      }
    };

  buf = calloc(1024, sizeof(unsigned char));
  if (NULL == buf)
    {
      if (verbose)
        {
          syslog(LOG_INFO, "Buffer allocation failed\n");
        }

      return (1);
    }

  esp32c3_sha512_init(&ctx);

  for (i = 0; i < 6; i++)
    {
      j = i % 3;
      k = i < 3;

      if (verbose)
        {
          syslog(LOG_INFO, "  SHA-%d test #%d: ", 512 - k * 128, j + 1);
        }

      ret = esp32c3_sha512_starts(&ctx, k);
      if (ret != 0)
        {
          goto fail;
        }

      if (j == 2)
        {
          memset(buf, 'a', buflen = 1000);

          for (j = 0; j < 1000; j++)
            {
              ret = esp32c3_sha512_update(&ctx, buf, buflen);
              if (ret != 0)
                {
                  goto fail;
                }
            }
        }
      else
        {
          ret = esp32c3_sha512_update(&ctx,
                                      sha512_test_buf[j],
                                      sha512_test_buflen[j]);
          if (ret != 0)
            {
              goto fail;
            }
        }

      ret = esp32c3_sha512_finish(&ctx, sha512sum);
      if (ret != 0)
        {
          goto fail;
        }

      if (memcmp(sha512sum, sha512_test_sum[i], 64 - k * 16) != 0)
        {
          ret = 1;
          goto fail;
        }

      if (verbose)
        {
          syslog(LOG_INFO, "passed\n");
        }
    }

  if (verbose)
    {
      syslog(LOG_INFO, "\n");
    }

  goto exit;

fail:
  if (verbose)
    {
      syslog(LOG_INFO, "failed\n");
    }

exit:
  esp32c3_sha512_free(&ctx);
  free(buf);

  return (ret);
}

/****************************************************************************
 * Name: esp32c3_sha_main
 ****************************************************************************/

int esp32c3_sha_main(int argc, char *argv[])
{
  int ret = 0;

  syslog(LOG_INFO, "----- BEGIN TEST -----\n");

  esp32c3_sha_init();

  ret = esp32c3_sha1_self_test(true);
  if (ret)
    {
      goto test_end;
    }

  ret = esp32c3_sha256_self_test(true);
  if (ret)
    {
      goto test_end;
    }

  ret = esp32c3_sha512_self_test(true);
  if (ret)
    {
      goto test_end;
    }

test_end:
  syslog(LOG_INFO, "----- END TEST -----\n");

  syslog(LOG_INFO, "\n");

  syslog(LOG_INFO, "----- RESULT: %s -----\n",
         !ret ? "SUCCESS" : "FAILED");

  return OK;
}

#endif
