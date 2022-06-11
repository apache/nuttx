/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_aes.c
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

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <semaphore.h>

#include <nuttx/semaphore.h>
#include <nuttx/crypto/crypto.h>

#include "riscv_internal.h"
#include "esp32c3.h"
#include "esp32c3_aes.h"

#include "hardware/esp32c3_aes.h"
#include "hardware/esp32c3_system.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AES_BLK_SIZE                    (16)

#define AES_MODE_DECRYPT                (BIT(2))

#define AES_IDLE_STATE                  (0)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_aes_inited;
static sem_t g_aes_sem = SEM_INITIALIZER(1);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aes_hw_setkey
 *
 * Description:
 *   Set AES hardware key and encryption/decryption mode
 *
 * Input Parameters:
 *   aes     - AES object data pointer
 *   encrypt - True: encryption mode; False: decryption mode
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void aes_hw_setkey(struct esp32c3_aes_s *aes, bool encrypt)
{
  int i;
  uint32_t cryptbits = encrypt ? 0 : AES_MODE_DECRYPT;
  uint32_t keybits = (aes->keybits / 64) - 2;
  uint32_t keywords = aes->keybits / 32;

  putreg32(cryptbits | keybits, AES_MODE_REG);

  for (i = 0; i < keywords; ++i)
    {
      putreg32(aes->key[i], AES_KEY_0_REG + i * 4);
    }
}

/****************************************************************************
 * Name: aes_hw_cypher
 *
 * Description:
 *   Process AES hardware encryption/decryption.
 *
 * Input Parameters:
 *   s - Input data pointer
 *   d - Output buffer pointer
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void aes_hw_cypher(const uint8_t *s, uint8_t *d)
{
  uint32_t buffer[AES_BLK_SIZE / 4];

  memcpy(buffer, s, AES_BLK_SIZE);

  putreg32(buffer[0], AES_TEXT_IN_0_REG + 0);
  putreg32(buffer[1], AES_TEXT_IN_0_REG + 4);
  putreg32(buffer[2], AES_TEXT_IN_0_REG + 8);
  putreg32(buffer[3], AES_TEXT_IN_0_REG + 12);

  putreg32(AES_TRIGGER_M, AES_TRIGGER_REG);

  while (getreg32(AES_STATE_REG) != AES_IDLE_STATE)
    {
    }

  buffer[0] = getreg32(AES_TEXT_OUT_0_REG + 0);
  buffer[1] = getreg32(AES_TEXT_OUT_0_REG + 4);
  buffer[2] = getreg32(AES_TEXT_OUT_0_REG + 8);
  buffer[3] = getreg32(AES_TEXT_OUT_0_REG + 12);

  memcpy(d, buffer, AES_BLK_SIZE);
}

/****************************************************************************
 * Name: gf128mul_x_ble
 *
 * Description:
 *   GF(2^128) multiplication function.
 *
 * Input Parameters:
 *   d - Result buffer
 *   s - Input data buffer
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void gf128mul_x_ble(uint8_t *d, const uint8_t *s)
{
  uint64_t a, b, ra, rb;

  memcpy(&a, s, 8);
  memcpy(&b, s + 8, 8);

  ra = (a <<  1) ^ (0x0087 >> (8 - ((b >> 63) << 3)));
  rb = (a >> 63) | (b << 1);

  memcpy(d, &ra, 8);
  memcpy(d + 8, &rb, 8);
}

/****************************************************************************
 * Name: esp32c3_aes_ecb_cypher
 *
 * Description:
 *   Process AES ECB encryption/decryption.
 *
 * Input Parameters:
 *   aes     - AES object data pointer
 *   encrypt - True: encryption mode; False: decryption mode
 *   input   - Input data pointer
 *   output  - Output buffer pointer
 *   size    - Data size in bytes
 *
 * Returned Value:
 *   OK is returned on success. Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

int esp32c3_aes_ecb_cypher(struct esp32c3_aes_s *aes, bool encrypt,
                           const void *input, void *output, uint32_t size)
{
  int ret;
  uint32_t i;
  const uint8_t *s = (const uint8_t *)input;
  uint8_t *d = (uint8_t *)output;

  DEBUGASSERT(aes && input && output);
  DEBUGASSERT(size && ((size % AES_BLK_SIZE) == 0));

  ret = nxsem_wait(&g_aes_sem);
  if (ret < 0)
    {
      return ret;
    }

  aes_hw_setkey(aes, encrypt);

  for (i = 0; i < size; i += AES_BLK_SIZE)
    {
      aes_hw_cypher(s, d);

      s += AES_BLK_SIZE;
      d += AES_BLK_SIZE;
    }

  ret = nxsem_post(&g_aes_sem);
  if (ret < 0)
    {
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: esp32c3_aes_cbc_cypher
 *
 * Description:
 *   Process AES CBC encryption/decryption.
 *
 * Input Parameters:
 *   aes     - AES object data pointer
 *   encrypt - True: encryption mode; False: decryption mode
 *   ivptr   - Initialization vector pointer
 *   input   - Input data pointer
 *   output  - Output buffer pointer
 *   size    - Data size in bytes
 *
 * Returned Value:
 *   OK is returned on success. Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

int esp32c3_aes_cbc_cypher(struct esp32c3_aes_s *aes, bool encrypt,
                           void *ivptr, const void *input, void *output,
                           uint32_t size)
{
  int ret;
  uint32_t i;
  uint32_t j;
  const uint8_t *s = (const uint8_t *)input;
  uint8_t *d = (uint8_t *)output;
  uint8_t *iv = (uint8_t *)ivptr;

  DEBUGASSERT(aes && input && output && ivptr);
  DEBUGASSERT(size && ((size % AES_BLK_SIZE) == 0));

  ret = nxsem_wait(&g_aes_sem);
  if (ret < 0)
    {
      return ret;
    }

  aes_hw_setkey(aes, encrypt);

  for (i = 0; i < size; i += AES_BLK_SIZE)
    {
      if (encrypt)
        {
          for (j = 0; j < AES_BLK_SIZE; j++)
            {
              d[j] = s[j] ^ iv[j];
            }

          aes_hw_cypher(d, d);

          memcpy(iv, d, AES_BLK_SIZE);
        }
      else
        {
          aes_hw_cypher(s, d);

          for (j = 0; j < AES_BLK_SIZE; j++)
            {
              d[j] = d[j] ^ iv[j];
            }

          memcpy(iv, s, AES_BLK_SIZE);
        }

      s += AES_BLK_SIZE;
      d += AES_BLK_SIZE;
    }

  ret = nxsem_post(&g_aes_sem);
  if (ret < 0)
    {
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: esp32c3_aes_ctr_cypher
 *
 * Description:
 *   Process AES CTR encryption/decryption.
 *
 * Input Parameters:
 *   aes      - AES object data pointer
 *   offptr   - Offset buffer pointer
 *   cntptr   - Counter buffer pointer
 *   cacheptr - Counter calculation buffer pointer
 *   input    - Input data pointer
 *   output   - Output buffer pointer
 *   size     - Data size in bytes
 *
 * Returned Value:
 *   OK is returned on success. Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

int esp32c3_aes_ctr_cypher(struct esp32c3_aes_s *aes, uint32_t *offptr,
                           void *cntptr, void *cacheptr, const void *input,
                           void *output, uint32_t size)
{
  int ret;
  uint32_t i;
  uint32_t j;
  uint32_t n;
  uint8_t *cnt = (uint8_t *)cntptr;
  uint8_t *cache = (uint8_t *)cacheptr;
  const uint8_t *s = (const uint8_t *)input;
  uint8_t *d = (uint8_t *)output;

  DEBUGASSERT(aes && offptr && cntptr && cacheptr && input && output);
  DEBUGASSERT(size);

  ret = nxsem_wait(&g_aes_sem);
  if (ret < 0)
    {
      return ret;
    }

  aes_hw_setkey(aes, true);

  n = *offptr;
  for (i = 0; i < size; i++)
    {
      if (n == 0)
        {
          aes_hw_cypher(cnt, cache);
          for (j = AES_BLK_SIZE - 1; j > 0; j--)
            {
              cnt[j]++;
              if (cnt[j] != 0)
                {
                  break;
                }
            }
        }

      d[i] = s[i] ^ cache[n];

      n = (n + 1) & (AES_BLK_SIZE - 1);
    }

  *offptr = n;

  ret = nxsem_post(&g_aes_sem);
  if (ret < 0)
    {
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: esp32c3_aes_xts_cypher
 *
 * Description:
 *   Process AES XTS encryption/decryption.
 *
 * Input Parameters:
 *   aes     - AES object data pointer
 *   encrypt - True: encryption mode; False: decryption mode
 *   unitptr - Unit data buffer pointer
 *   input   - Input data pointer
 *   output  - Output buffer pointer
 *   size    - Data size in bytes
 *
 * Returned Value:
 *   OK is returned on success. Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

int esp32c3_aes_xts_cypher(struct esp32c3_aes_xts_s *aes, bool encrypt,
                           void *unitptr, const void *input, void *output,
                           uint32_t size)
{
  int ret;
  uint32_t i;
  uint32_t j;
  uint32_t blks;
  uint32_t rst;
  uint8_t *t;
  uint8_t *prev_output;
  uint8_t tweak[AES_BLK_SIZE];
  uint8_t prev_tweak[AES_BLK_SIZE];
  uint8_t tmp[AES_BLK_SIZE];
  uint8_t *unit = (uint8_t *)unitptr;
  const uint8_t *s = (const uint8_t *)input;
  uint8_t *d = (uint8_t *)output;

  DEBUGASSERT(aes && unitptr && input && output);

  /* NIST SP 80-38E disallows data units larger than 2**20 blocks. */

  DEBUGASSERT((size >= AES_BLK_SIZE) &&
              (size <= ((1 << 20) * AES_BLK_SIZE)));

  ret = nxsem_wait(&g_aes_sem);
  if (ret < 0)
    {
      return ret;
    }

  blks = size / AES_BLK_SIZE;
  rst  = size % AES_BLK_SIZE;

  aes_hw_setkey(&aes->tweak, true);
  aes_hw_cypher(unit, tweak);

  for (i = 0; i < blks; i++)
    {
      if (rst && (encrypt == false) && (blks == 1))
        {
          memcpy(prev_tweak, tweak, AES_BLK_SIZE);
          gf128mul_x_ble(tweak, tweak);
        }

      for (j = 0; j < AES_BLK_SIZE; j++)
        {
          tmp[j] = s[j] ^ tweak[j];
        }

      aes_hw_setkey(&aes->crypt, encrypt);
      aes_hw_cypher(tmp, tmp);

      for (j = 0; j < AES_BLK_SIZE; j++)
        {
          d[j] = tmp[j] ^ tweak[j];
        }

      gf128mul_x_ble(tweak, tweak);

      s += AES_BLK_SIZE;
      d += AES_BLK_SIZE;
    }

  if (rst)
    {
      t = encrypt ? tweak : prev_tweak;
      prev_output = d - AES_BLK_SIZE;

      for (i = 0; i < rst; i++)
        {
          d[i] = prev_output[i];
          tmp[i] = s[i] ^ t[i];
        }

      for (; i < AES_BLK_SIZE; i++)
        {
          tmp[i] = prev_output[i] ^ t[i];
        }

      aes_hw_setkey(&aes->crypt, encrypt);
      aes_hw_cypher(tmp, tmp);

      for (i = 0; i < AES_BLK_SIZE; i++)
        {
          prev_output[i] = tmp[i] ^ t[i];
        }
    }

  ret = nxsem_post(&g_aes_sem);
  if (ret < 0)
    {
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: esp32c3_aes_setkey
 *
 * Description:
 *   Configurate AES key.
 *
 * Input Parameters:
 *   aes     - AES object data pointer
 *   keyptr  - Key data pointer
 *   keybits - Key data bits
 *
 * Returned Value:
 *   OK is returned on success. Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

int esp32c3_aes_setkey(struct esp32c3_aes_s *aes, const void *keyptr,
                       uint16_t keybits)
{
  DEBUGASSERT(aes && keyptr);

  if ((keybits != 128) && (keybits != 256))
    {
      return -EINVAL;
    }

  aes->keybits = keybits;
  memcpy(aes->key, keyptr, keybits / 8);

  return OK;
}

/****************************************************************************
 * Name: esp32c3_aes_xts_setkey
 *
 * Description:
 *   Configurate AES XTS key.
 *
 * Input Parameters:
 *   aes     - AES object data pointer
 *   keyptr  - Key data pointer
 *   keybits - Key data bits
 *
 * Returned Value:
 *   OK is returned on success. Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

int esp32c3_aes_xts_setkey(struct esp32c3_aes_xts_s *aes, const void *keyptr,
                           uint16_t keybits)
{
  const uint8_t *key = (const uint8_t *)keyptr;
  uint16_t half_keybits = keybits / 2;

  DEBUGASSERT(aes && keyptr);

  if ((keybits != 256) && (keybits != 512))
    {
      return -EINVAL;
    }

  aes->crypt.keybits = half_keybits;
  memcpy(aes->crypt.key, key, half_keybits / 8);

  aes->tweak.keybits = half_keybits;
  memcpy(aes->tweak.key, key + half_keybits / 8, half_keybits / 8);

  return OK;
}

/****************************************************************************
 * Name: esp32c3_aes_init
 *
 * Description:
 *   Initialize ESP32-C3 AES hardware.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK is returned on success. Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

int esp32c3_aes_init(void)
{
  if (!g_aes_inited)
    {
      setbits(SYSTEM_CRYPTO_AES_CLK_EN, SYSTEM_PERIP_CLK_EN1_REG);
      resetbits(SYSTEM_CRYPTO_AES_RST, SYSTEM_PERIP_RST_EN1_REG);
      g_aes_inited = true;
    }

  return OK;
}

/****************************************************************************
 * Name: aes_cypher
 ****************************************************************************/

#ifdef CONFIG_CRYPTO_AES

int aes_cypher(void *out, const void *in, size_t size,
               const void *iv, const void *key, size_t keysize,
               int mode, int encrypt)
{
  int ret;
  uint8_t iv_buf[AES_BLK_SIZE];
  uint8_t cache_buf[AES_BLK_SIZE];
  uint32_t nc_off;
  struct esp32c3_aes_s aes;

  if ((size & (AES_BLK_SIZE - 1)) != 0)
    {
      return -EINVAL;
    }

  if (keysize != 16)
    {
      return -EINVAL;
    }

  if ((mode != AES_MODE_ECB) &&
      (mode != AES_MODE_CBC) &&
      (mode != AES_MODE_CTR))
    {
      return -EINVAL;
    }

  ret = esp32c3_aes_init();
  if (ret < 0)
    {
      return ret;
    }

  ret = esp32c3_aes_setkey(&aes, key, keysize * 8);
  if (ret < 0)
    {
      return ret;
    }

  switch (mode)
    {
      case AES_MODE_ECB:
        ret = esp32c3_aes_ecb_cypher(&aes, encrypt, in, out, size);
        break;
      case AES_MODE_CBC:
        memcpy(iv_buf, iv, AES_BLK_SIZE);
        ret = esp32c3_aes_cbc_cypher(&aes, encrypt, iv_buf, in, out, size);
        break;
      case AES_MODE_CTR:
        nc_off = 0;
        memcpy(iv_buf, iv, AES_BLK_SIZE);
        ret = esp32c3_aes_ctr_cypher(&aes, &nc_off, iv_buf, cache_buf,
                                   in, out, size);
      default :
        ret = -EINVAL;
        break;
    }

  return ret;
}

#endif

