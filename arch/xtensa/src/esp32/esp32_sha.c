/****************************************************************************
 * arch/xtensa/src/esp32/esp32_sha.c
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

#ifdef CONFIG_ESP32_SHA_ACCELERATOR

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <debug.h>
#include <nuttx/mutex.h>

#include "xtensa.h"
#include "hardware/esp32_sha.h"
#include "periph_ctrl.h"
#include "esp32_sha.h"
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_sha_inited;
uint32_t sha_busy_reg[4] =
                        {
                          SHA1_BUSY_REG,
                          SHA256_BUSY_REG,
                          SHA384_BUSY_REG,
                          SHA512_BUSY_REG
                        };
uint32_t sha_start_reg[4] =
                        {
                          SHA1_START_REG,
                          SHA256_START_REG,
                          SHA384_START_REG,
                          SHA512_START_REG
                        };
uint32_t sha_continue_reg[4] =
                        {
                          SHA1_CONTINUE_REG,
                          SHA256_CONTINUE_REG,
                          SHA384_CONTINUE_REG,
                          SHA512_CONTINUE_REG
                        };
uint32_t sha_load_reg[4] =
                        {
                          SHA1_LOAD_REG,
                          SHA256_LOAD_REG,
                          SHA384_LOAD_REG,
                          SHA512_LOAD_REG
                        };
/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_sha_block
 *
 * Description:
 *   Performs SHA on multiple blocks at a time.
 *
 * Input Parameters:
 *   ctx      - The SHA context
 *   data     - Input message to be hashed on single block
 *   len      - Length of the input message on single block
 *   buf      - Input message to be hashed on multiple blocks
 *   buf_len  - Length of the input message on multiple blocks
 *
 * Returned Value:
 *   OK is returned on success.
 *
 ****************************************************************************/

static int esp32_sha_block(struct esp32_sha_context_s *ctx,
                              const unsigned char *buffer)
{
  uint32_t *data_words = NULL;
  int i;

  data_words = (uint32_t *)buffer;

  while (getreg32(sha_busy_reg[ctx->mode]))
    {
    }

  for (i = 0; i < ctx->block_size / 32; i++)
    {
      data_words[i] = ((data_words[i] >> 24) & 0xff) |
                     ((data_words[i] >> 8) & 0xff00) |
                     ((data_words[i] << 8) & 0xff0000) |
                     ((data_words[i] << 24) & 0xff000000);
      putreg32(data_words[i], SHA_TEXT_0_REG + i * 4);
    }

  if (ctx->first_block)
    {
      putreg32(1, sha_start_reg[ctx->mode]);
    }
  else
    {
      putreg32(1, sha_continue_reg[ctx->mode]);
    }

  ctx->first_block = false;

  while (getreg32(sha_busy_reg[ctx->mode]))
    {
    }

  if (ctx->final_block)
    {
      putreg32(1, sha_load_reg[ctx->mode]);
      while (getreg32(sha_busy_reg[ctx->mode]))
        {
        }
    }

  for (i = 0; i < ctx->output_size / 32; i++)
    {
      ctx->state[i] = getreg32(SHA_TEXT_0_REG + i * 4);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_sha1_starts
 *
 * Description:
 *   Starts a SHA-1 checksum calculation.
 *
 * Input Parameters:
 *   ctx      - The SHA context to initialize
 *
 * Returned Value:
 *   OK is returned on success.
 *
 ****************************************************************************/

int esp32_sha1_starts(struct esp32_sha_context_s *ctx)
{
  memset(ctx, 0, sizeof(struct esp32_sha_context_s));
  ctx->mode = ESP32_SHA1_1;
  ctx->output_size = 160;
  ctx->block_size = 512;

  return OK;
}

/****************************************************************************
 * Name: esp32_sha_update
 *
 * Description:
 *   Feeds an input buffer into an ongoing SHA checksum calculation.
 *
 * Input Parameters:
 *   ctx      - The SHA context to use
 *   input    - The buffer holding the input data
 *   ilen     - The length of the input data in Bytes
 *
 * Returned Value:
 *   OK is returned on success.
 *   Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

int esp32_sha_update(struct esp32_sha_context_s *ctx,
                     const unsigned char *input,
                     size_t ilen)
{
  unsigned int i;
  unsigned int j;

  j = (uint32_t)((ctx->count[0] >> 3) & ((ctx->block_size >> 3) - 1));
  ctx->count[0] += (ilen << 3);
  if (ctx->count[0] < (ilen << 3))
    {
      if (ctx->mode == ESP32_SHA1_1 || ctx->mode == ESP32_SHA2_256)
          return ERROR;

      ctx->count[1]++;
    }

  if ((j + ilen) > ((ctx->block_size >> 3) - 1))
    {
      memcpy(&ctx->buffer[j], input, (i = (ctx->block_size >> 3) - j));
      if (ctx->sha_state == ESP32_SHA_STATE_INIT)
        {
          ctx->first_block = true;
          ctx->sha_state = ESP32_SHA_STATE_IN_PROCESS;
        }
      else if (ctx->sha_state == ESP32_SHA_STATE_IN_PROCESS)
        {
          ctx->first_block = false;
        }

      esp32_sha_block(ctx, ctx->buffer);
      for (; i + ((ctx->block_size >> 3) - 1) < ilen;
            i += (ctx->block_size >> 3))
          esp32_sha_block(ctx, &input[i]);

      j = 0;
    }
  else
    {
      i = 0;
    }

  memcpy(&ctx->buffer[j], &input[i], ilen - i);
  return OK;
}

/****************************************************************************
 * Name: esp32_sha_finish
 *
 * Description:
 *   Finishes the SHA operation,
 *   and writes the result to the output buffer.
 *
 * Input Parameters:
 *   ctx      - The SHA context to use
 *   output   - The SHA-1 checksum result
 *
 * Returned Value:
 *   OK is returned on success.
 *   Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

int esp32_sha_finish(struct esp32_sha_context_s *ctx,
                     unsigned char output[64])
{
  unsigned int i;
  unsigned char finalcount[16];

  if (ctx->mode == ESP32_SHA1_1 || ctx->mode == ESP32_SHA2_256)
      for (i = 0; i < 8; i++)
          finalcount[i] = (unsigned char)((ctx->count[0] >>
              ((8 - 1 - i) * 8)) & 255);

  /* SHA384 and SHA512 use 1024 bits to store the message length */

  else
    {
      for (i = 0; i < 8; i++)
          finalcount[i] = (unsigned char)((ctx->count[1] >>
              ((8 - 1 - i) * 8)) & 255);
      for (i = 8; i < 16; i++)
          finalcount[i] = (unsigned char)((ctx->count[0] >>
              ((16 - 1 - i) * 8)) & 255);
    }

  esp32_sha_update(ctx, "\200", 1);

  while ((ctx->count[0] & (ctx->block_size - 8)) !=
          ctx->block_size - (ctx->block_size >> 3))
      esp32_sha_update(ctx, "\0", 1);

  ctx->final_block = true;

  if (ctx->mode == ESP32_SHA1_1 || ctx->mode == ESP32_SHA2_256)
      esp32_sha_update(ctx, finalcount, 8);

  /* SHA384 and SHA512 use 1024 bits to store the message length */

  else
      esp32_sha_update(ctx, finalcount, 16);

  for (i = 0; i <  ctx->output_size / 8; i++)
      output[i] = (unsigned char)((ctx->state[i >> 2] >>
        ((3 - (i & 3)) * 8)) & 255);

  explicit_bzero(&finalcount, sizeof(finalcount));
  explicit_bzero(ctx, sizeof(*ctx));

  return 0;
}

/****************************************************************************
 * Name: esp32_sha1_free
 *
 * Description:
 *   Clears a SHA context.
 *
 * Input Parameters:
 *   ctx      - The SHA context to clear
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32_sha1_free(struct esp32_sha_context_s *ctx)
{
  if (ctx == NULL)
      return;

  memset(ctx, 0, sizeof(struct esp32_sha_context_s));
}

/****************************************************************************
 * Name: esp32_sha256_starts
 *
 * Description:
 *   Starts a SHA-256 checksum calculation.
 *
 * Input Parameters:
 *   ctx      - The SHA context to initialize
 *
 * Returned Value:
 *   OK is returned on success.
 *
 ****************************************************************************/

int esp32_sha256_starts(struct esp32_sha_context_s *ctx)
{
  memset(ctx, 0, sizeof(struct esp32_sha_context_s));
  ctx->mode = ESP32_SHA2_256;
  ctx->output_size = 256;
  ctx->block_size = 512;

  return OK;
}

/****************************************************************************
 * Name: esp32_sha384_starts
 *
 * Description:
 *   Starts a SHA-384 checksum calculation.
 *
 * Input Parameters:
 *   ctx      - The SHA context to initialize
 *
 * Returned Value:
 *   OK is returned on success.
 *
 ****************************************************************************/

int esp32_sha384_starts(struct esp32_sha_context_s *ctx)
{
  memset(ctx, 0, sizeof(struct esp32_sha_context_s));
  ctx->mode = ESP32_SHA3_384;
  ctx->output_size = 384;
  ctx->block_size = 1024;

  return OK;
}

/****************************************************************************
 * Name: esp32_sha512_starts
 *
 * Description:
 *   Starts a SHA-512 checksum calculation.
 *
 * Input Parameters:
 *   ctx      - The SHA context to initialize
 *
 * Returned Value:
 *   OK is returned on success.
 *
 ****************************************************************************/

int esp32_sha512_starts(struct esp32_sha_context_s *ctx)
{
  memset(ctx, 0, sizeof(struct esp32_sha_context_s));
  ctx->mode = ESP32_SHA3_512;
  ctx->output_size = 512;
  ctx->block_size = 1024;

  return OK;
}

/****************************************************************************
 * Name: esp32_sha_init
 *
 * Description:
 *   Initialize ESP32 SHA hardware.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK is returned on success.
 *   Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

int esp32_sha_init(void)
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

