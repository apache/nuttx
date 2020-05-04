/****************************************************************************
 * arch/arm/src/lpc43xx/lpc43_aes.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author:  Alexander Vasiljev <alexvasiljev@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
#include "arm_arch.h"

#include "chip.h"
#include <chip/lpc43_aes.h>

#define AES_BLOCK_SIZE 16

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct lpc43_aes_s *g_aes;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int aes_init(FAR const void *iv, FAR const void *key, uint32_t keysize,
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
      cmd = mode == AES_MODE_ECB ? AES_API_CMD_ENCODE_ECB : AES_API_CMD_ENCODE_CBC;
    }
  else
    {
      cmd = mode == AES_MODE_ECB ? AES_API_CMD_DECODE_ECB : AES_API_CMD_DECODE_CBC;
    }

  g_aes->aes_Init();

  if (key != NULL)
    {
      g_aes->aes_LoadKeySW(key);
    }
  else
    {
      switch (keysize)
        {
          case 0:
            g_aes->aes_LoadKey1();
            break;

          case 1:
            g_aes->aes_LoadKey2();
            break;

          case 2:
            g_aes->aes_LoadKeyRNG();
            break;
        }
    }

  g_aes->aes_LoadIV_SW((const unsigned char*)iv);

  ret = g_aes->aes_SetMode(cmd);
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

static int aes_update(FAR const void *out, uint32_t *outl, FAR const void *in,
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

  return g_aes->aes_Operate((unsigned char*)out,
                            (unsigned char*)in, inl / 16);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int aes_cypher(void *out, const void *in, uint32_t size, const void *iv,
               const void *key, uint32_t keysize, int mode, int encrypt)
{
  unsigned int ret = 0;
  uint32_t outl = size;

  g_aes = (struct lpc43_g_aes*)*((uint32_t*)LPC43_ROM_AES_DRIVER_TABLE);

  ret = aes_init(iv, key, keysize, mode, encrypt);

  if (ret != OK)
    {
      return ret;
    }

  return aes_update(out, &outl, in, size);
}
