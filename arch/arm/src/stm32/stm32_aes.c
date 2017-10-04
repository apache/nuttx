/****************************************************************************
 * arch/arm/src/stm32/stm32_aes.c
 *
 *   Copyright (C) 2015 Haltian Ltd. All rights reserved.
 *   Author:  Juha Niskanen <juha.niskanen@haltian.com>
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
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/crypto/crypto.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "up_internal.h"
#include "up_arch.h"

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

static void aes_enable(bool on);
static void aes_ccfc(void);
static void aes_setkey(const void *key, size_t key_len);
static void aes_setiv(const void *iv);
static void aes_encryptblock(void *block_out, const void *block_in);
static int aes_setup_cr(int mode, int encrypt);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static sem_t aes_lock;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void aes_enable(bool on)
{
  uint32_t regval;

  regval = getreg32(STM32_AES_CR);
  if (on)
    regval |= AES_CR_EN;
  else
    regval &= ~AES_CR_EN;
  putreg32(regval, STM32_AES_CR);
}

/* Clear AES_SR_CCF status register bit */

static void aes_ccfc(void)
{
  uint32_t regval;

  regval = getreg32(STM32_AES_CR);
  regval |= AES_CR_CCFC;
  putreg32(regval, STM32_AES_CR);
}

static void aes_setkey(const void *key, size_t key_len)
{
  uint32_t *in = (uint32_t *)key;

  (void)key_len;

  putreg32(__builtin_bswap32(*in), STM32_AES_KEYR3);
  in++;
  putreg32(__builtin_bswap32(*in), STM32_AES_KEYR2);
  in++;
  putreg32(__builtin_bswap32(*in), STM32_AES_KEYR1);
  in++;
  putreg32(__builtin_bswap32(*in), STM32_AES_KEYR0);
}

static void aes_setiv(const void *iv)
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

static void aes_encryptblock(void *block_out, const void *block_in)
{
  uint32_t *in = (uint32_t *)block_in;
  uint32_t *out = (uint32_t *)block_out;

  putreg32(*in, STM32_AES_DINR);
  in++;
  putreg32(*in, STM32_AES_DINR);
  in++;
  putreg32(*in, STM32_AES_DINR);
  in++;
  putreg32(*in, STM32_AES_DINR);

  while (!(getreg32(STM32_AES_SR) & AES_SR_CCF))
    ;
  aes_ccfc();

  *out = getreg32(STM32_AES_DOUTR);
  out++;
  *out = getreg32(STM32_AES_DOUTR);
  out++;
  *out = getreg32(STM32_AES_DOUTR);
  out++;
  *out = getreg32(STM32_AES_DOUTR);
}

static int aes_setup_cr(int mode, int encrypt)
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

int aes_cypher(void *out, const void *in, uint32_t size, const void *iv,
               const void *key, uint32_t keysize, int mode, int encrypt)
{
  int ret = OK;

  if ((size & (AES_BLOCK_SIZE-1)) != 0)
    {
      return -EINVAL;
    }

  if (keysize != 16)
    {
      return -EINVAL;
    }

  ret = nxsem_wait(&aes_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* AES must be disabled before changing mode, key or IV. */

  aes_enable(false);
  ret = aes_setup_cr(mode, encrypt);
  if (ret < 0)
    {
      goto out;
    }

  aes_setkey(key, keysize);
  if (iv)
    {
      aes_setiv(iv);
    }

  aes_enable(true);
  while (size)
    {
      aes_encryptblock(out, in);
      out = (uint8_t *)out + AES_BLOCK_SIZE;
      in  = (uint8_t *)in  + AES_BLOCK_SIZE;
      size -= AES_BLOCK_SIZE;
    }

  aes_enable(false);

out:
  nxsem_post(&aes_lock);
  return ret;
}

int up_aesreset(void)
{
  irqstate_t flags;
  uint32_t regval;

  flags = enter_critical_section();

  regval = getreg32(STM32_RCC_AHBRSTR);
  regval |= RCC_AHBRSTR_AESRST;
  putreg32(regval, STM32_RCC_AHBRSTR);
  regval &= ~RCC_AHBRSTR_AESRST;
  putreg32(regval, STM32_RCC_AHBRSTR);

  leave_critical_section(flags);

  return OK;
}

int up_aesinitialize(void)
{
  uint32_t regval;

  nxsem_init(&aes_lock, 0, 1);

  regval = getreg32(STM32_RCC_AHBENR);
  regval |= RCC_AHBENR_AESEN;
  putreg32(regval, STM32_RCC_AHBENR);

  aes_enable(false);

  return OK;
}

int up_aesuninitialize(void)
{
  uint32_t regval;

  aes_enable(false);

  regval = getreg32(STM32_RCC_AHBENR);
  regval &= ~RCC_AHBENR_AESEN;
  putreg32(regval, STM32_RCC_AHBENR);

  nxsem_destroy(&aes_lock);

  return OK;
}
