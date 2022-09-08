/****************************************************************************
 * arch/arm/src/lpc43xx/lpc43_aes.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/crypto/crypto.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include <hardware/lpc43_aes.h>

#define AES_BLOCK_SIZE 16

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct lpc43_aes_s *g_aes;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int aes_init(const void *iv,
                    const void *key, uint32_t keysize,
                    int mode, int encrypt)
{
  unsigned int cmd = 0;
  unsigned int ret = 0;

  if (g_aes == NULL)
    {
      return -ENOSYS;
    }

  /* The LPC43 aes engine can load two keys from otp and one random
   * generated key. This behavior doesn't fit current api.  So if
   * key == NULL, we will usr keysize as identifier of the special key.
   */

  if (keysize != 16 && key)
    {
      return -EINVAL;
    }

  if (mode != AES_MODE_ECB && mode != AES_MODE_CBC)
    {
      return -EINVAL;
    }

  if (encrypt == CYPHER_ENCRYPT)
    {
      cmd = mode == AES_MODE_ECB ?
                    AES_API_CMD_ENCODE_ECB : AES_API_CMD_ENCODE_CBC;
    }
  else
    {
      cmd = mode == AES_MODE_ECB ?
                    AES_API_CMD_DECODE_ECB : AES_API_CMD_DECODE_CBC;
    }

  g_aes->aes_init();

  if (key != NULL)
    {
      g_aes->aes_load_key_sw(key);
    }
  else
    {
      switch (keysize)
        {
          case 0:
            g_aes->aes_load_key1();
            break;

          case 1:
            g_aes->aes_load_key2();
            break;

          case 2:
            g_aes->aes_load_key_rng();
            break;
        }
    }

  g_aes->aes_load_iv_sw((const unsigned char *)iv);

  ret = g_aes->aes_set_mode(cmd);
  switch (ret)
    {
      case AES_API_ERR_WRONG_CMD:
      case AES_API_ERR_NOT_SUPPORTED:
      case AES_API_ERR_KEY_ALREADY_PROGRAMMED:
        ret = -EINVAL;
        break;

      case AES_API_ERR_DMA_CHANNEL_CFG:
      case AES_API_ERR_DMA_MUX_CFG:
      case AES_API_ERR_DMA_BUSY:
        ret = -EBUSY;
        break;
    }

  return 0;
}

static int aes_update(const void *out,
                      uint32_t *outl, const void *in,
                      uint32_t inl)
{
  if (g_aes == NULL)
    {
      return -ENOSYS;
    }

  if ((inl & (AES_BLOCK_SIZE - 1)) != 0)
    {
      return -EINVAL;
    }

  if (inl > *outl)
    {
      return -EINVAL;
    }

  return g_aes->aes_operate((unsigned char *)out,
                            (unsigned char *)in, inl / 16);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int aes_cypher(void *out, const void *in, size_t size, const void *iv,
               const void *key, size_t keysize, int mode, int encrypt)
{
  unsigned int ret = 0;
  uint32_t outl = size;

  g_aes = (struct lpc43_g_aes *) * ((uint32_t *)LPC43_ROM_AES_DRIVER_TABLE);

  ret = aes_init(iv, key, keysize, mode, encrypt);

  if (ret != OK)
    {
      return ret;
    }

  return aes_update(out, &outl, in, size);
}
