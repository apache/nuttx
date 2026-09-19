/****************************************************************************
 * arch/arm64/src/common/arm64_aes.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/crypto/crypto.h>

#include "arm64_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ID_AA64ISAR0_EL1.AES, bits [7:4]. Non-zero means AESE/AESD and the
 * MixColumns pair are implemented.
 */

#define ID_AA64ISAR0_AES_SHIFT 4
#define ID_AA64ISAR0_AES_MASK  0xful

#define AES_BLOCK_SIZE 16
#define AES_MAX_ROUNDS 14
#define AES_MAX_WORDS  (4 * (AES_MAX_ROUNDS + 1))

/* The counter occupies the last four bytes of the block, as in
 * crypto/xform.c, so both agree on what a CTR stream looks like.
 */

#define AES_CTR_OFFSET 12

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct arm64_aes_s
{
  uint32_t ek[AES_MAX_WORDS];
  uint32_t dk[AES_MAX_WORDS];
  unsigned rounds;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool arm64_aes_present(void)
{
  uint64_t isar0;

  __asm__ volatile ("mrs %0, id_aa64isar0_el1" : "=r" (isar0));

  return ((isar0 >> ID_AA64ISAR0_AES_SHIFT) & ID_AA64ISAR0_AES_MASK) != 0;
}

/* Only SubWord is accelerated; the rest of the schedule is ordinary
 * arithmetic, so it stays in C.
 */

static uint32_t arm64_aes_subword(uint32_t in)
{
  uint8_t block[AES_BLOCK_SIZE];
  uint32_t out;

  /* A zero round key leaves AESE doing SubBytes and ShiftRows only, and
   * ShiftRows cannot disturb a state whose four columns are identical.
   */

  memcpy(block + 0, &in, 4);
  memcpy(block + 4, &in, 4);
  memcpy(block + 8, &in, 4);
  memcpy(block + 12, &in, 4);

  __asm__ volatile (
    "ld1   {v0.16b}, [%0]     \n"
    "movi  v1.16b, #0         \n"
    "aese  v0.16b, v1.16b     \n"
    "st1   {v0.16b}, [%0]     \n"
    :: "r" (block) : "v0", "v1", "memory");

  memcpy(&out, block, sizeof(out));
  return out;
}

static void arm64_aes_encrypt(FAR const struct arm64_aes_s *ctx,
                              FAR const uint8_t *in, FAR uint8_t *out)
{
  FAR const uint8_t *rk = (FAR const uint8_t *)ctx->ek;
  unsigned n = ctx->rounds - 1;

  __asm__ volatile (
    "ld1   {v0.16b}, [%[in]]              \n"
    "1:                                   \n"
    "ld1   {v1.16b}, [%[rk]], #16         \n"
    "aese  v0.16b, v1.16b                 \n"
    "aesmc v0.16b, v0.16b                 \n"
    "subs  %w[n], %w[n], #1               \n"
    "b.ne  1b                             \n"
    "ld1   {v1.16b}, [%[rk]], #16         \n"
    "aese  v0.16b, v1.16b                 \n"
    "ld1   {v1.16b}, [%[rk]]              \n"
    "eor   v0.16b, v0.16b, v1.16b         \n"
    "st1   {v0.16b}, [%[out]]             \n"
    : [rk] "+r" (rk), [n] "+r" (n)
    : [in] "r" (in), [out] "r" (out)
    : "v0", "v1", "cc", "memory");
}

static void arm64_aes_decrypt(FAR const struct arm64_aes_s *ctx,
                              FAR const uint8_t *in, FAR uint8_t *out)
{
  FAR const uint8_t *rk = (FAR const uint8_t *)&ctx->dk[4 * ctx->rounds];
  unsigned n = ctx->rounds - 1;

  __asm__ volatile (
    "ld1    {v0.16b}, [%[in]]             \n"
    "1:                                   \n"
    "ld1    {v1.16b}, [%[rk]]             \n"
    "sub    %[rk], %[rk], #16             \n"
    "aesd   v0.16b, v1.16b                \n"
    "aesimc v0.16b, v0.16b                \n"
    "subs   %w[n], %w[n], #1              \n"
    "b.ne   1b                            \n"
    "ld1    {v1.16b}, [%[rk]]             \n"
    "sub    %[rk], %[rk], #16             \n"
    "aesd   v0.16b, v1.16b                \n"
    "ld1    {v1.16b}, [%[rk]]             \n"
    "eor    v0.16b, v0.16b, v1.16b        \n"
    "st1    {v0.16b}, [%[out]]            \n"
    : [rk] "+r" (rk), [n] "+r" (n)
    : [in] "r" (in), [out] "r" (out)
    : "v0", "v1", "cc", "memory");
}

static int arm64_aes_setkey(FAR struct arm64_aes_s *ctx,
                            FAR const uint8_t *key, size_t len,
                            bool decrypt)
{
  static const uint8_t rcon[] =
    {
      0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1b, 0x36
    };

  unsigned nk;
  unsigned words;
  unsigned i;

  switch (len)
    {
      case 16:
        ctx->rounds = 10;
        break;

      case 24:
        ctx->rounds = 12;
        break;

      case 32:
        ctx->rounds = 14;
        break;

      default:
        return -EINVAL;
    }

  nk = (unsigned)len / 4;
  words = 4 * (ctx->rounds + 1);

  memcpy(ctx->ek, key, len);

  for (i = nk; i < words; i++)
    {
      uint32_t t = ctx->ek[i - 1];

      if (i % nk == 0)
        {
          t = (t >> 8) | (t << 24);
          t = arm64_aes_subword(t) ^ (uint32_t)rcon[i / nk - 1];
        }
      else if (nk > 6 && i % nk == 4)
        {
          t = arm64_aes_subword(t);
        }

      ctx->ek[i] = ctx->ek[i - nk] ^ t;
    }

  if (!decrypt)
    {
      return OK;
    }

  /* The equivalent inverse cipher wants InvMixColumns applied to every round
   * key except the first and the last, so decryption is the same shape as
   * encryption rather than a separate schedule.
   */

  memcpy(ctx->dk, ctx->ek, (size_t)words * 4);

  for (i = 1; i < ctx->rounds; i++)
    {
      FAR uint8_t *rk = (FAR uint8_t *)&ctx->dk[4 * i];

      __asm__ volatile (
        "ld1    {v0.16b}, [%0]   \n"
        "aesimc v0.16b, v0.16b   \n"
        "st1    {v0.16b}, [%0]   \n"
        :: "r" (rk) : "v0", "memory");
    }

  return OK;
}

static void arm64_aes_xor_block(FAR uint8_t *dst, FAR const uint8_t *src)
{
  int i;

  for (i = 0; i < AES_BLOCK_SIZE; i++)
    {
      dst[i] ^= src[i];
    }
}

static void arm64_aes_ctr_inc(FAR uint8_t *block)
{
  int i;

  for (i = AES_BLOCK_SIZE - 1; i >= AES_CTR_OFFSET; i--)
    {
      if (++block[i] != 0)
        {
          break;
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aes_cypher
 *
 * Description:
 *   Encrypt or decrypt a whole number of AES blocks in ECB, CBC or CTR mode
 *   using the Armv8 Cryptography Extension.
 *
 * Returned Value:
 *   OK, -ENOTSUP on a core without the extension, or -EINVAL for a bad mode,
 *   key length or size.
 *
 ****************************************************************************/

int aes_cypher(FAR void *out, FAR const void *in, size_t size,
               FAR const void *iv, FAR const void *key, size_t keysize,
               int mode, int encrypt)
{
  struct arm64_aes_s ctx;
  uint8_t chain[AES_BLOCK_SIZE];
  FAR const uint8_t *src = in;
  FAR uint8_t *dst = out;
  int ret;

  if (!arm64_aes_present())
    {
      return -ENOTSUP;
    }

  if (size % AES_BLOCK_SIZE != 0)
    {
      return -EINVAL;
    }

  switch (mode & AES_MODE_MASK)
    {
      case AES_MODE_ECB:
      case AES_MODE_CBC:
      case AES_MODE_CTR:
        break;

      default:
        return -EINVAL;
    }

  /* CTR runs the cipher forwards in both directions, so it never needs the
   * inverse schedule.
   */

  ret = arm64_aes_setkey(&ctx, key, keysize,
                         !encrypt && (mode & AES_MODE_MASK) != AES_MODE_CTR);
  if (ret < 0)
    {
      return ret;
    }

  if (iv != NULL)
    {
      memcpy(chain, iv, AES_BLOCK_SIZE);
    }
  else
    {
      memset(chain, 0, AES_BLOCK_SIZE);
    }

  while (size != 0)
    {
      switch (mode & AES_MODE_MASK)
        {
          case AES_MODE_ECB:
            if (encrypt)
              {
                arm64_aes_encrypt(&ctx, src, dst);
              }
            else
              {
                arm64_aes_decrypt(&ctx, src, dst);
              }
            break;

          case AES_MODE_CBC:
            if (encrypt)
              {
                memcpy(dst, src, AES_BLOCK_SIZE);
                arm64_aes_xor_block(dst, chain);
                arm64_aes_encrypt(&ctx, dst, dst);
                memcpy(chain, dst, AES_BLOCK_SIZE);
              }
            else
              {
                uint8_t prev[AES_BLOCK_SIZE];

                memcpy(prev, src, AES_BLOCK_SIZE);
                arm64_aes_decrypt(&ctx, src, dst);
                arm64_aes_xor_block(dst, chain);
                memcpy(chain, prev, AES_BLOCK_SIZE);
              }
            break;

          case AES_MODE_CTR:
            {
              uint8_t stream[AES_BLOCK_SIZE];

              arm64_aes_encrypt(&ctx, chain, stream);
              memcpy(dst, src, AES_BLOCK_SIZE);
              arm64_aes_xor_block(dst, stream);
              arm64_aes_ctr_inc(chain);
              explicit_bzero(stream, sizeof(stream));
            }
            break;
        }

      src  += AES_BLOCK_SIZE;
      dst  += AES_BLOCK_SIZE;
      size -= AES_BLOCK_SIZE;
    }

  explicit_bzero(&ctx, sizeof(ctx));
  explicit_bzero(chain, sizeof(chain));
  return OK;
}
