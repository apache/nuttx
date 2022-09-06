/****************************************************************************
 * arch/arm/src/stm32f0l0g0/stm32_aes.c
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
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32_rcc.h"
#include "stm32_aes.h"

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

static void stm32aes_enable(bool on);
static void stm32aes_ccfc(void);
static void stm32aes_setkey(const void *key, size_t key_len);
static void stm32aes_setiv(const void *iv);
static void stm32aes_encryptblock(void *block_out,
                                  const void *block_in);
static int  stm32aes_setup_cr(int mode, int encrypt);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mutex_t g_stm32aes_lock = NXMUTEX_INITIALIZER;
static bool    g_stm32aes_initdone;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void stm32aes_enable(bool on)
{
  uint32_t regval;

  regval = getreg32(STM32_AES_CR);
  if (on)
    {
      regval |= AES_CR_EN;
    }
  else
    {
      regval &= ~AES_CR_EN;
    }

  putreg32(regval, STM32_AES_CR);
}

/* Clear AES_SR_CCF status register bit */

static void stm32aes_ccfc(void)
{
  uint32_t regval;

  regval  = getreg32(STM32_AES_CR);
  regval |= AES_CR_CCFC;
  putreg32(regval, STM32_AES_CR);
}

/* TODO: Handle other AES key lengths or fail if length is not valid */

static void stm32aes_setkey(const void *key, size_t key_len)
{
  uint32_t *in = (uint32_t *)key;

  putreg32(__builtin_bswap32(*in), STM32_AES_KEYR3);
  in++;
  putreg32(__builtin_bswap32(*in), STM32_AES_KEYR2);
  in++;
  putreg32(__builtin_bswap32(*in), STM32_AES_KEYR1);
  in++;
  putreg32(__builtin_bswap32(*in), STM32_AES_KEYR0);
}

static void stm32aes_setiv(const void *iv)
{
  uint32_t *in = (uint32_t *)iv;

  putreg32(__builtin_bswap32(*in), STM32_AES_IVR3);
  in++;
  putreg32(__builtin_bswap32(*in), STM32_AES_IVR2);
  in++;
  putreg32(__builtin_bswap32(*in), STM32_AES_IVR1);
  in++;
  putreg32(__builtin_bswap32(*in), STM32_AES_IVR0);
}

static void stm32aes_encryptblock(void *block_out,
                                  const void *block_in)
{
  uint32_t *in  = (uint32_t *)block_in;
  uint32_t *out = (uint32_t *)block_out;

  putreg32(*in, STM32_AES_DINR);
  in++;
  putreg32(*in, STM32_AES_DINR);
  in++;
  putreg32(*in, STM32_AES_DINR);
  in++;
  putreg32(*in, STM32_AES_DINR);

  while (!(getreg32(STM32_AES_SR) & AES_SR_CCF));
  stm32aes_ccfc();

  *out = getreg32(STM32_AES_DOUTR);
  out++;
  *out = getreg32(STM32_AES_DOUTR);
  out++;
  *out = getreg32(STM32_AES_DOUTR);
  out++;
  *out = getreg32(STM32_AES_DOUTR);
}

static int stm32aes_setup_cr(int mode, int encrypt)
{
  uint32_t regval = 0;

  regval |= AES_CR_DATATYPE_BE;

  switch (mode)
  {
  case AES_MODE_ECB:
    regval |= AES_CR_CHMOD_ECB;
    break;

  case AES_MODE_CBC:
    regval |= AES_CR_CHMOD_CBC;
    break;

  case AES_MODE_CTR:
    regval |= AES_CR_CHMOD_CTR;
    break;

  default:
    return -EINVAL;
  }

  if (encrypt)
    {
      regval |= AES_CR_MODE_ENCRYPT;
    }
  else
    {
      if (mode == AES_MODE_CTR)
        {
          regval |= AES_CR_MODE_DECRYPT;
        }
      else
        {
          regval |= AES_CR_MODE_DECRYPT_KEYDERIV;
        }
    }

  putreg32(regval, STM32_AES_CR);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stm32_aesreset(void)
{
  irqstate_t flags;
  uint32_t regval;

  flags = enter_critical_section();

  regval  = getreg32(STM32_RCC_AHBRSTR);
  regval |= RCC_AHBRSTR_AESRST;
  putreg32(regval, STM32_RCC_AHBRSTR);
  regval &= ~RCC_AHBRSTR_AESRST;
  putreg32(regval, STM32_RCC_AHBRSTR);

  leave_critical_section(flags);

  return OK;
}

int stm32_aesinitialize(void)
{
  uint32_t regval;

  regval  = getreg32(STM32_RCC_AHBENR);
  regval |= RCC_AHBENR_AESEN;
  putreg32(regval, STM32_RCC_AHBENR);

  stm32aes_enable(false);

  return OK;
}

int stm32_aesuninitialize(void)
{
  uint32_t regval;

  stm32aes_enable(false);

  regval  = getreg32(STM32_RCC_AHBENR);
  regval &= ~RCC_AHBENR_AESEN;
  putreg32(regval, STM32_RCC_AHBENR);

  return OK;
}

int aes_cypher(void *out, const void *in, size_t size,
               const void *iv, const void *key, size_t keysize,
               int mode, int encrypt)
{
  int ret = OK;

  /* Ensure initialization was done */

  if (!g_stm32aes_initdone)
    {
      ret = stm32_aesinitialize();
      if (ret < 0)
        {
          return ret; /* AES init failed */
        }

      g_stm32aes_initdone = true;
    }

  if ((size & (AES_BLOCK_SIZE - 1)) != 0)
    {
      return -EINVAL;
    }

  if (keysize != 16)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&g_stm32aes_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* AES must be disabled before changing mode, key or IV. */

  stm32aes_enable(false);
  ret = stm32aes_setup_cr(mode, encrypt);
  if (ret < 0)
    {
      goto out;
    }

  stm32aes_setkey(key, keysize);
  if (iv != NULL)
    {
      stm32aes_setiv(iv);
    }

  stm32aes_enable(true);
  while (size)
    {
      stm32aes_encryptblock(out, in);
      out   = (uint8_t *)out + AES_BLOCK_SIZE;
      in    = (uint8_t *)in  + AES_BLOCK_SIZE;
      size -= AES_BLOCK_SIZE;
    }

  stm32aes_enable(false);

out:
  nxmutex_unlock(&g_stm32aes_lock);
  return ret;
}
