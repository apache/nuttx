/****************************************************************************
 * arch/arm/src/sam34/sam_aes.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author:  Max Nekludov <macscomp@gmail.com>
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
#include <nuttx/semaphore.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "arm_arch.h"

#include "chip.h"
#include "sam_periphclks.h"
#include "sam_aes.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AES_BLOCK_SIZE 16

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static sem_t g_samaes_lock;
static bool  g_samaes_initdone = false;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void samaes_lock(void)
{
  nxsem_wait(&g_samaes_lock);
}

static void samaes_unlock(void)
{
  nxsem_post(&g_samaes_lock);
}

static void samaes_memcpy(FAR void *out, FAR const void *in, size_t size)
{
  size_t i;
  size_t wcount = size / 4;

  for (i = 0; i < wcount;
       i++, out = (FAR uint8_t *)out + 4, in = (FAR uint8_t *)in + 4)
    {
      *(FAR uint32_t *)out = *(FAR uint32_t *)in;
    }
}

static void samaes_encryptblock(FAR void *out, FAR const void *in)
{
  samaes_memcpy((void *)SAM_AES_IDATAR, in, AES_BLOCK_SIZE);

  putreg32(AES_CR_START, SAM_AES_CR);

  while (!(getreg32(SAM_AES_ISR) & AES_ISR_DATRDY));

  if (out)
    {
      samaes_memcpy(out, (void *)SAM_AES_ODATAR, AES_BLOCK_SIZE);
    }
}

static int samaes_setup_mr(uint32_t keysize, int mode, int encrypt)
{
  uint32_t regval = AES_MR_SMOD_MANUAL_START | AES_MR_CKEY;

  if (encrypt)
    {
      regval |= AES_MR_CIPHER_ENCRYPT;
    }
  else
    {
      regval |= AES_MR_CIPHER_DECRYPT;
    }

  switch (keysize)
    {
    case 16:
      regval |= AES_MR_KEYSIZE_AES128;
      break;

    case 24:
      regval |= AES_MR_KEYSIZE_AES192;
      break;

    case 32:
      regval |= AES_MR_KEYSIZE_AES256;
      break;

    default:
      return -EINVAL;
    }

  switch (mode)
    {
    case AES_MODE_ECB:
      regval |= AES_MR_OPMOD_ECB;
      break;

    case AES_MODE_CBC:
      regval |= AES_MR_OPMOD_CBC;
      break;

    case AES_MODE_CTR:
      regval |= AES_MR_OPMOD_CTR;
      break;

    case AES_MODE_CFB:
      regval |= AES_MR_OPMOD_CFB;
      break;

    default:
      return -EINVAL;
    }

  putreg32(regval, SAM_AES_MR);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

static int samaes_initialize(void)
{
  nxsem_init(&g_samaes_lock, 0, 1);
  sam_aes_enableclk();
  putreg32(AES_CR_SWRST, SAM_AES_CR);
  return OK;
}

int aes_cypher(FAR void *out, FAR const void *in, uint32_t size,
               FAR const void *iv, FAR const void *key, uint32_t keysize,
               int mode, int encrypt)
{
  int ret = OK;

  if (!g_samaes_initdone)
    {
      ret = samaes_initialize();
      if (ret != OK)
        {
          return ret;
        }

      g_samaes_initdone = true;
    }

  if (size % 16)
    {
      return -EINVAL;
    }

  samaes_lock();

  ret = samaes_setup_mr(keysize, mode & AES_MODE_MASK, encrypt);
  if (ret < 0)
    {
      samaes_unlock();
      return ret;
    }

  samaes_memcpy((FAR void *)SAM_AES_KEYWR, key, keysize);
  if (iv != NULL)
    {
      samaes_memcpy((FAR void *)SAM_AES_IVR, iv, AES_BLOCK_SIZE);
    }

  while (size)
    {
      if ((mode & AES_MODE_MAC) == 0)
        {
          samaes_encryptblock(out, in);
          out = (FAR char *)out + AES_BLOCK_SIZE;
        }
      else if (size == AES_BLOCK_SIZE)
        {
          samaes_encryptblock(out, in);
        }
      else
        {
          samaes_encryptblock(NULL, in);
        }

      in    = (FAR char *)in + AES_BLOCK_SIZE;
      size -= AES_BLOCK_SIZE;
    }

  samaes_unlock();
  return ret;
}
