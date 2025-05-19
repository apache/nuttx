/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_sha.c
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

#ifdef CONFIG_ESPRESSIF_SHA_ACCELERATOR

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <debug.h>
#include <nuttx/mutex.h>

#include "riscv_internal.h"

#include "esp_sha.h"

#include "esp_private/periph_ctrl.h"
#include "soc/periph_defs.h"
#include "hal/sha_hal.h"
#include "soc/soc_caps.h"
#include "rom/cache.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if CONFIG_SOC_SHA_SUPPORT_PARALLEL_ENG
#  error "Parallel engine feature is not supported."
#endif

#define PUT_UINT32_BE(n,b,i)                      \
{                                                 \
  (b)[(i)] = (unsigned char) ((n) >> 24);         \
  (b)[(i) + 1] = (unsigned char) ((n) >> 16);     \
  (b)[(i) + 2] = (unsigned char) ((n) >>  8);     \
  (b)[(i) + 3] = (unsigned char) ((n));           \
}

#define SHA1_BLK_SIZE                    (20)
#define SHA2_BLK_SIZE                    (32)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_sha_inited;
static mutex_t g_sha_lock = NXMUTEX_INITIALIZER;
static const unsigned char esp_sha_padding[64] =
{
  0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_sha_hash_block
 *
 * Description:
 *   Performs SHA on multiple blocks at a time.
 *
 * Input Parameters:
 *   type        - Type of SHA
 *   first_block - Pointer of the ctx struct about is block first
 *   state       - Pointer of the ctx struct about state
 *   data        - Input message to be hashed on single block
 *   len         - Length of the input message on single block
 *   buf         - Input message to be hashed on multiple blocks
 *   buf_len     - Length of the input message on multiple blocks
 *
 * Returned Value:
 *   OK is returned on success.
 *
 ****************************************************************************/

static int esp_sha_hash_block(enum esp_sha_type_e type,
                              bool *first_block, uint32_t *state,
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

  if (buf_len != 0)
    {
      sha_hal_hash_block(type, buf, blk_word_len, (*first_block));
      (*first_block) = false;
    }

  for (j = 0; j < num_block; j++)
    {
      data_words = (uint32_t *)(data + blk_len * j);
      sha_hal_hash_block(type, data_words, blk_word_len, (*first_block));
      (*first_block) = false;
    }

  sha_hal_read_digest(type, state);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_sha1_init
 *
 * Description:
 *   Initializes a SHA-1 context.
 *
 * Input Parameters:
 *   ctx - The SHA-1 context to initialize
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_sha1_init(struct esp_sha1_context_s *ctx)
{
  memset(ctx, 0, sizeof(struct esp_sha1_context_s));
}

/****************************************************************************
 * Name: esp_sha1_starts
 *
 * Description:
 *   Starts a SHA-1 checksum calculation.
 *
 * Input Parameters:
 *   ctx - The SHA-1 context to initialize
 *
 * Returned Value:
 *   OK is returned on success.
 *
 ****************************************************************************/

int esp_sha1_starts(struct esp_sha1_context_s *ctx)
{
  memset(ctx, 0, sizeof(struct esp_sha1_context_s));
  ctx->mode = ESP_SHA1_1;

  return OK;
}

/****************************************************************************
 * Name: esp_sha1_update
 *
 * Description:
 *   Feeds an input buffer into an ongoing SHA-1 checksum calculation.
 *
 * Input Parameters:
 *   ctx   - The SHA-1 context to use
 *   input - The buffer holding the input data
 *   ilen  - The length of the input data in Bytes
 *
 * Returned Value:
 *   OK is returned on success.
 *   Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

int esp_sha1_update(struct esp_sha1_context_s *ctx,
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
      ret = nxmutex_lock(&g_sha_lock);
      if (ret < 0)
        {
          return ret;
        }

      if (ctx->sha_state == ESP_SHA_STATE_INIT)
        {
          ctx->first_block = true;

          ctx->sha_state = ESP_SHA_STATE_IN_PROCESS;
        }
      else if (ctx->sha_state == ESP_SHA_STATE_IN_PROCESS)
        {
          ctx->first_block = false;
          sha_hal_write_digest(ctx->mode, ctx->state);
        }

      ret = esp_sha_hash_block(ctx->mode, &ctx->first_block, ctx->state,
                               input, len, ctx->buffer, local_len);
      ret |= nxmutex_unlock(&g_sha_lock);

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
 * Name: esp_sha1_finish
 *
 * Description:
 *   Finishes the SHA-1 operation,
 *   and writes the result to the output buffer.
 *
 * Input Parameters:
 *   ctx    - The SHA-1 context to use
 *   output - The SHA-1 checksum result
 *
 * Returned Value:
 *   OK is returned on success.
 *   Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

int esp_sha1_finish(struct esp_sha1_context_s *ctx,
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

  ret = esp_sha1_update(ctx, esp_sha_padding, padn);
  if (ret != 0)
    {
      return ret;
    }

  ret = esp_sha1_update(ctx, msglen, 8);
  if (ret != 0)
    {
      return ret;
    }

  memcpy(output, ctx->state, SHA1_BLK_SIZE);

  return ret;
}

/****************************************************************************
 * Name: esp_sha1_free
 *
 * Description:
 *   Clears a SHA-1 context.
 *
 * Input Parameters:
 *   ctx - The SHA-1 context to clear
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_sha1_free(struct esp_sha1_context_s *ctx)
{
  if (ctx == NULL)
    {
      return;
    }

  memset(ctx, 0, sizeof(struct esp_sha1_context_s));
}

/****************************************************************************
 * Name: esp_sha256_init
 *
 * Description:
 *   Initializes a SHA-256 context.
 *
 * Input Parameters:
 *   ctx - The SHA-256 context to initialize
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_sha256_init(struct esp_sha256_context_s *ctx)
{
  memset(ctx, 0, sizeof(struct esp_sha256_context_s));
}

/****************************************************************************
 * Name: esp_sha256_starts
 *
 * Description:
 *   Starts a SHA-224 or SHA-256 checksum calculation.
 *
 * Input Parameters:
 *   ctx   - The SHA-256 context to initialize
 *   is224 - Determines which function to use
 *
 * Returned Value:
 *   OK is returned on success.
 *
 ****************************************************************************/

int esp_sha256_starts(struct esp_sha256_context_s *ctx, bool is224)
{
  memset(ctx, 0, sizeof(struct esp_sha256_context_s));

  if (is224)
    {
      ctx->mode = ESP_SHA2_224;
    }
  else
    {
      ctx->mode = ESP_SHA2_256;
    }

  return OK;
}

/****************************************************************************
 * Name: esp_sha256_update
 *
 * Description:
 *   Feeds an input buffer into an ongoing SHA-224 or SHA-256
 *   checksum calculation.
 *
 * Input Parameters:
 *   ctx   - The SHA-256 context to use
 *   input - The buffer holding the input data
 *   ilen  - The length of the input data in Bytes
 *
 * Returned Value:
 *   OK is returned on success.
 *   Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

int esp_sha256_update(struct esp_sha256_context_s *ctx,
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
      ret = nxmutex_lock(&g_sha_lock);
      if (ret < 0)
        {
          return ret;
        }

      if (ctx->sha_state == ESP_SHA_STATE_INIT)
        {
          ctx->first_block = true;
          ctx->sha_state = ESP_SHA_STATE_IN_PROCESS;
        }
      else if (ctx->sha_state == ESP_SHA_STATE_IN_PROCESS)
        {
          ctx->first_block = false;
          sha_hal_write_digest(ctx->mode, ctx->state);
        }

      ret = esp_sha_hash_block(ctx->mode, &ctx->first_block, ctx->state,
                               input, len, ctx->buffer, local_len);
      ret |= nxmutex_unlock(&g_sha_lock);

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
 * Name: esp_sha256_finish
 *
 * Description:
 *   Finishes the SHA-224 or SHA-256 operation, and writes the result to
 *   the output buffer.
 *
 * Input Parameters:
 *   ctx    - The SHA-256 context to use
 *   output - The SHA-256 checksum result
 *
 * Returned Value:
 *   OK is returned on success.
 *   Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

int esp_sha256_finish(struct esp_sha256_context_s *ctx,
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

  ret = esp_sha256_update(ctx, esp_sha_padding, padn);
  if (ret != 0)
    {
      return ret;
    }

  ret = esp_sha256_update(ctx, msglen, 8);
  if (ret != 0)
    {
      return ret;
    }

  memcpy(output, ctx->state, SHA2_BLK_SIZE);

  return ret;
}

/****************************************************************************
 * Name: esp_sha256_free
 *
 * Description:
 *   Clears a SHA-256 context.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_sha256_free(struct esp_sha256_context_s *ctx)
{
  if (ctx == NULL)
    {
      return;
    }

  memset(ctx, 0, sizeof(struct esp_sha256_context_s));
}

/****************************************************************************
 * Name: esp_sha_init
 *
 * Description:
 *   Initialize ESP device SHA hardware.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK is returned on success.
 *   Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

int esp_sha_init(void)
{
  if (!g_sha_inited)
    {
      periph_module_enable(PERIPH_SHA_MODULE);
      g_sha_inited = true;
    }
  else
    {
      return -EBUSY;
    }

  return OK;
}

#endif
