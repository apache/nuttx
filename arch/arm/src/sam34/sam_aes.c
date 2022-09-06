/****************************************************************************
 * arch/arm/src/sam34/sam_aes.c
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
#include <nuttx/mutex.h>
#include <arch/board/board.h>

#include "arm_internal.h"
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

static mutex_t g_samaes_lock = NXMUTEX_INITIALIZER;
static bool    g_samaes_initdone = false;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void samaes_memcpy(void *out, const void *in, size_t size)
{
  size_t i;
  size_t wcount = size / 4;

  for (i = 0; i < wcount;
       i++, out = (uint8_t *)out + 4, in = (uint8_t *)in + 4)
    {
      *(uint32_t *)out = *(uint32_t *)in;
    }
}

static void samaes_encryptblock(void *out, const void *in)
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
  sam_aes_enableclk();
  putreg32(AES_CR_SWRST, SAM_AES_CR);
  return OK;
}

int aes_cypher(void *out, const void *in, size_t size,
               const void *iv, const void *key, size_t keysize,
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

  nxmutex_lock(&g_samaes_lock);

  ret = samaes_setup_mr(keysize, mode & AES_MODE_MASK, encrypt);
  if (ret < 0)
    {
      nxmutex_unlock(&g_samaes_lock);
      return ret;
    }

  samaes_memcpy((void *)SAM_AES_KEYWR, key, keysize);
  if (iv != NULL)
    {
      samaes_memcpy((void *)SAM_AES_IVR, iv, AES_BLOCK_SIZE);
    }

  while (size)
    {
      if ((mode & AES_MODE_MAC) == 0)
        {
          samaes_encryptblock(out, in);
          out = (char *)out + AES_BLOCK_SIZE;
        }
      else if (size == AES_BLOCK_SIZE)
        {
          samaes_encryptblock(out, in);
        }
      else
        {
          samaes_encryptblock(NULL, in);
        }

      in    = (char *)in + AES_BLOCK_SIZE;
      size -= AES_BLOCK_SIZE;
    }

  nxmutex_unlock(&g_samaes_lock);
  return ret;
}
