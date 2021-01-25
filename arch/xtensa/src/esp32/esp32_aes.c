/****************************************************************************
 * arch/xtensa/src/esp32/esp32_aes.c
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
#include <debug.h>
#include <semaphore.h>

#include <nuttx/semaphore.h>
#include <nuttx/crypto/crypto.h>

#include "xtensa.h"
#include "hardware/esp32_aes.h"
#include "hardware/esp32_dport.h"
#include "esp32_aes.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AES_BLK_SIZE                    (16)

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

static void aes_hw_setkey(struct esp32_aes_s *aes, bool encrypt)
{
  int i;
  uint32_t cryptbits = encrypt ? 0 : AES_MODE_DECRYPT;
  uint32_t keybits = (aes->keybits / 64) - 2;
  uint32_t keywords = aes->keybits / 32;

  putreg32(cryptbits | keybits, AES_MODE_REG);

  for (i = 0; i < keywords; ++i)
    {
      putreg32(aes->key[i], AES_KEY_BASE + i * 4);
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

  putreg32(buffer[0], AES_TEXT_BASE + 0);
  putreg32(buffer[1], AES_TEXT_BASE + 4);
  putreg32(buffer[2], AES_TEXT_BASE + 8);
  putreg32(buffer[3], AES_TEXT_BASE + 12);

  putreg32(AES_START_OPT, AES_START_REG);

  while (getreg32(AES_IDLE_REG) != AES_IDLE_STATE)
    {
    }

  buffer[0] = getreg32(AES_TEXT_BASE + 0);
  buffer[1] = getreg32(AES_TEXT_BASE + 4);
  buffer[2] = getreg32(AES_TEXT_BASE + 8);
  buffer[3] = getreg32(AES_TEXT_BASE + 12);

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
 * Name: esp32_aes_ecb_cypher
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

int esp32_aes_ecb_cypher(struct esp32_aes_s *aes, bool encrypt,
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
 * Name: esp32_aes_cbc_cypher
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

int esp32_aes_cbc_cypher(struct esp32_aes_s *aes, bool encrypt, void *ivptr,
                         const void *input, void *output, uint32_t size)
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
 * Name: esp32_aes_ctr_cypher
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

int esp32_aes_ctr_cypher(struct esp32_aes_s *aes, uint32_t *offptr,
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
 * Name: esp32_aes_xts_cypher
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

int esp32_aes_xts_cypher(struct esp32_aes_xts_s *aes, bool encrypt,
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
 * Name: esp32_aes_setkey
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

int esp32_aes_setkey(struct esp32_aes_s *aes, const void *keyptr,
                     uint16_t keybits)
{
  DEBUGASSERT(aes && keyptr);

  if ((keybits != 128) && (keybits != 192) && (keybits != 256))
    {
      return -EINVAL;
    }

  aes->keybits = keybits;
  memcpy(aes->key, keyptr, keybits / 8);

  return OK;
}

/****************************************************************************
 * Name: esp32_aes_xts_setkey
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

int esp32_aes_xts_setkey(struct esp32_aes_xts_s *aes, const void *keyptr,
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
 * Name: esp32_aes_init
 *
 * Description:
 *   Initialize ESP32 AES hardware.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK is returned on success. Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

int esp32_aes_init(void)
{
  if (!g_aes_inited)
    {
      modifyreg32(DPORT_PERI_CLK_EN_REG, 0, DPORT_PERI_CLK_EN_AES);
      modifyreg32(DPORT_PERI_RST_EN_REG, DPORT_PERI_RST_EN_AES, 0);
      g_aes_inited = true;
    }

  return OK;
}

/****************************************************************************
 * Name: aes_cypher
 ****************************************************************************/

#ifdef CONFIG_CRYPTO_AES

int aes_cypher(FAR void *out, FAR const void *in, uint32_t size,
               FAR const void *iv, FAR const void *key, uint32_t keysize,
               int mode, int encrypt)
{
  int ret;
  uint8_t iv_buf[AES_BLK_SIZE];
  uint8_t cache_buf[AES_BLK_SIZE];
  uint32_t nc_off;
  struct esp32_aes_s aes;

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

  ret = esp32_aes_init();
  if (ret < 0)
    {
      return ret;
    }

  ret = esp32_aes_setkey(&aes, key, keysize * 8);
  if (ret < 0)
    {
      return ret;
    }

  switch (mode)
    {
      case AES_MODE_ECB:
        ret = esp32_aes_ecb_cypher(&aes, encrypt, in, out, size);
        break;
      case AES_MODE_CBC:
        memcpy(iv_buf, iv, AES_BLK_SIZE);
        ret = esp32_aes_cbc_cypher(&aes, encrypt, iv_buf, in, out, size);
        break;
      case AES_MODE_CTR:
        nc_off = 0;
        memcpy(iv_buf, iv, AES_BLK_SIZE);
        ret = esp32_aes_ctr_cypher(&aes, &nc_off, iv_buf, cache_buf,
                                   in, out, size);
      default :
        ret = -EINVAL;
        break;
    }

  return ret;
}

#endif

/****************************************************************************
 * Test Functions
 ****************************************************************************/

#ifdef CONFIG_ESP32_AES_ACCELERATOR_TEST

/****************************************************************************
 * Name: esp32_aes_ecb_test
 ****************************************************************************/

static void esp32_aes_ecb_test(void)
{
  int ret;
  int i;
  int keybits;
  uint8_t encrypt_buf[16];
  uint8_t decrypt_buf[16];
  struct esp32_aes_s aes;
  const int size = 16;

  const uint32_t input[8] =
    {
      0x740fdb34, 0x002defca, 0xb042437b, 0xc2f42cf9,
      0xc64444be, 0x32365bc1, 0xb613cfa2, 0x15ce0d23
    };

  const uint32_t key[16] =
    {
      0x8ffdc2c5, 0x14d6c69d, 0x9cb7608f, 0x899b2472,
      0xbf9e4372, 0x855290d0, 0xc62753da, 0xdeedeab7
    };

  const uint32_t result[3][4] =
    {
      /* keybits = 128 */

      {
        0xc810df2a, 0x8ae67e6e, 0x50c5e32c, 0xd535f3e4
      },

      /* keybits = 192 */

      {
        0x00d2f88e, 0x4e859ec6, 0x394e0af7, 0x965326d8
      },

      /* keybits = 256 */

      {
        0xa0714c2b, 0x356adb1f, 0xe905c243, 0x35195a7c
      }
    };

  esp32_aes_init();

  for (i = 0; i < 3; i++)
    {
      keybits = i * 64 + 128;

      ret = esp32_aes_setkey(&aes, key, keybits);
      DEBUGASSERT(ret == 0);

      ret = esp32_aes_ecb_cypher(&aes, 1, input, encrypt_buf, size);
      DEBUGASSERT(ret == 0);

      if (memcmp(encrypt_buf, result[i], size))
        {
          DEBUGASSERT(0);
        }

      ret = esp32_aes_ecb_cypher(&aes, 0, encrypt_buf, decrypt_buf, size);
      DEBUGASSERT(ret == 0);

      if (memcmp(decrypt_buf, input, size))
        {
          DEBUGASSERT(0);
        }

      syslog(LOG_INFO, "ESP32 AES ECB key=%d bits test: PASS\n", keybits);
    }
}

/****************************************************************************
 * Name: esp32_aes_cbc_test
 ****************************************************************************/

static void esp32_aes_cbc_test(void)
{
  int ret;
  int i;
  int keybits;
  uint8_t encrypt_buf[32];
  uint8_t decrypt_buf[32];
  uint8_t iv_buf[16];
  struct esp32_aes_s aes;
  const int size = 32;

  const uint32_t input[8] =
    {
      0x740fdb34, 0x002defca, 0xb042437b, 0xc2f42cf9,
      0xc64444be, 0x32365bc1, 0xb613cfa2, 0x15ce0d23
    };

  const uint32_t key[16] =
    {
      0x8ffdc2c5, 0x14d6c69d, 0x9cb7608f, 0x899b2472,
      0xbf9e4372, 0x855290d0, 0xc62753da, 0xdeedeab7
    };

  const uint32_t iv[4] =
    {
      0xf53a50f2, 0x8aaf711d, 0x953bbbfa, 0x228d53cb
    };

  const uint32_t result[3][8] =
    {
      /* keybits = 128 */

      {
        0x04e27d12, 0x1a91e508, 0x01092431, 0x9d572184,
        0xa39979e1, 0x5543e1bc, 0x7173b71d, 0x4e3be064
      },

      /* keybits = 192 */

      {
        0x9b894bd8, 0x7dc31ec6, 0xde40c3d5, 0xc2ed0679,
        0xa8a857fc, 0x815db8ca, 0x33f18ab8, 0x752c1b8e
      },

      /* keybits = 256 */

      {
        0x6f36b8fe, 0x33bc1f37, 0x24fe659c, 0x0370def0,
        0xb9a852f8, 0x64a79ae2, 0xd59f5045, 0x648a0f44
      }
    };

  for (i = 0; i < 3; i++)
    {
      keybits = i * 64 + 128;

      ret = esp32_aes_setkey(&aes, key, keybits);
      DEBUGASSERT(ret == 0);

      memcpy(iv_buf, iv, 16);
      ret = esp32_aes_cbc_cypher(&aes, 1, iv_buf, input, encrypt_buf, size);
      DEBUGASSERT(ret == 0);

      if (memcmp(encrypt_buf, result[i], size))
        {
          DEBUGASSERT(0);
        }

      memcpy(iv_buf, iv, 16);
      ret = esp32_aes_cbc_cypher(&aes, 0, iv_buf, encrypt_buf,
                                 decrypt_buf, size);
      DEBUGASSERT(ret == 0);

      if (memcmp(decrypt_buf, input, size))
        {
          DEBUGASSERT(0);
        }

      syslog(LOG_INFO, "ESP32 AES CBC key=%d bits test: PASS\n", keybits);
    }
}

/****************************************************************************
 * Name: esp32_aes_cbc_test
 ****************************************************************************/

static void esp32_aes_ctr_test(void)
{
  int ret;
  int i;
  int keybits;
  uint8_t encrypt_buf[32];
  uint8_t decrypt_buf[32];
  uint8_t cnt_buf[16];
  uint8_t cache_buf[16];
  uint32_t nc_off;
  struct esp32_aes_s aes;
  const int size = 32;

  const uint32_t input[8] =
    {
      0x740fdb34, 0x002defca, 0xb042437b, 0xc2f42cf9,
      0xc64444be, 0x32365bc1, 0xb613cfa2, 0x15ce0d23
    };

  const uint32_t key[16] =
    {
      0x8ffdc2c5, 0x14d6c69d, 0x9cb7608f, 0x899b2472,
      0xbf9e4372, 0x855290d0, 0xc62753da, 0xdeedeab7
    };

  const uint32_t cnt[4] =
    {
      0xf53a50f2, 0x8aaf711d, 0x953bbbfa, 0x228d53cb
    };

  const uint32_t result[3][8] =
    {
      /* keybits = 128 */

      {
        0x5f922338, 0x5aff403d, 0x45fede3f, 0x616568c6,
        0x3cd0ffc7, 0xa26cb704, 0x0aaa8b6a, 0x1d0b5e1c
      },

      /* keybits = 192 */

      {
        0xe1052003, 0x429823e2, 0x547e3f33, 0xbe55c832,
        0x037f9f57, 0x1b3f025f, 0xc4c9a836, 0x164e2730
      },

      /* keybits = 256 */

      {
        0x70af4473, 0x597d2126, 0xd598ed09, 0x3fea540c,
        0xfb5c743c, 0x0c1a39ca, 0xcbcf2d17, 0x341a7a0c
      }
    };

  for (i = 0; i < 3; i++)
    {
      keybits = i * 64 + 128;

      ret = esp32_aes_setkey(&aes, key, keybits);
      DEBUGASSERT(ret == 0);

      nc_off = 0;
      memcpy(cnt_buf, cnt, 16);
      ret = esp32_aes_ctr_cypher(&aes, &nc_off, cnt_buf, cache_buf,
                                 input, encrypt_buf, size);
      DEBUGASSERT(ret == 0);

      if (memcmp(encrypt_buf, result[i], size))
        {
          DEBUGASSERT(0);
        }

      nc_off = 0;
      memcpy(cnt_buf, cnt, 16);
      ret = esp32_aes_ctr_cypher(&aes, &nc_off, cnt_buf, cache_buf,
                                 encrypt_buf, decrypt_buf, size);
      DEBUGASSERT(ret == 0);

      if (memcmp(decrypt_buf, input, size))
        {
          DEBUGASSERT(0);
        }

      syslog(LOG_INFO, "ESP32 AES CTR key=%d bits test: PASS\n", keybits);
    }
}

/****************************************************************************
 * Name: esp32_aes_xts_test
 ****************************************************************************/

static void esp32_aes_xts_test(void)
{
  int ret;
  int i;
  int keybits;
  uint8_t encrypt_buf[32];
  uint8_t decrypt_buf[32];
  uint8_t unit_buf[16];
  struct esp32_aes_xts_s aes;
  int size;

  const uint32_t input[8] =
    {
      0x740fdb34, 0x002defca, 0xb042437b, 0xc2f42cf9,
      0xc64444be, 0x32365bc1, 0xb613cfa2, 0x15ce0d23
    };

  const uint32_t key[16] =
    {
      0x8ffdc2c5, 0x14d6c69d, 0x9cb7608f, 0x899b2472,
      0xbf9e4372, 0x855290d0, 0xc62753da, 0xdeedeab7,
      0x7ac6c53b, 0xc94f0b81, 0xdd673fc9, 0x8c1b71a6,
      0x1f99b728, 0x5e7af2eb, 0xcc7274a3, 0xf0005b23
    };

  const uint32_t unit[4] =
    {
      0xf53a50f2, 0x8aaf711d, 0x953bbbfa, 0x228d53cb
    };

  const uint32_t result_in32[2][8] =
    {
      /* keybits = 256 */

      {
        0xf70e05fd, 0x2791be41, 0x926ec006, 0xc76068f4,
        0x01fd0843, 0xdf5e576a, 0xa4b1833d, 0x90502608
      },

      /* keybits = 512 */

      {
        0x164b4185, 0x4cb1cce7, 0xf285e523, 0x06a5923a,
        0xae4fcb7b, 0x59ce9dc6, 0xed64546f, 0x5889cb17
      }
    };

  const uint32_t result_in30[2][8] =
    {
      /* keybits = 256 */

      {
        0x26991fb6, 0x72e4a7bc, 0x97041d61, 0x9ec889af,
        0xf70e05fd, 0x2791be41, 0x926ec006, 0x000068f4
      },

      /* keybits = 512 */

      {
        0x4b42dd86, 0xeee792c0, 0x1516ff95, 0x1f5fd9e6,
        0x164b4185, 0x4cb1cce7, 0xf285e523, 0x0000923a
      }
    };

  for (i = 0; i < 2; i++)
    {
      keybits = i * 256 + 256;

      ret = esp32_aes_xts_setkey(&aes, key, keybits);
      DEBUGASSERT(ret == 0);

      /* Encrypt/Decrypt 32 bytes */

      size = 32;

      memcpy(unit_buf, unit, 16);
      ret = esp32_aes_xts_cypher(&aes, true, unit_buf, input,
                                 encrypt_buf, size);
      DEBUGASSERT(ret == 0);

      if (memcmp(encrypt_buf, result_in32[i], size))
        {
          DEBUGASSERT(0);
        }

      memcpy(unit_buf, unit, 16);
      ret = esp32_aes_xts_cypher(&aes, false, unit_buf, encrypt_buf,
                                 decrypt_buf, size);
      DEBUGASSERT(ret == 0);

      if (memcmp(decrypt_buf, input, size))
        {
          DEBUGASSERT(0);
        }

      /* Encrypt/Decrypt 30 bytes */

      size = 30;

      memcpy(unit_buf, unit, 16);
      ret = esp32_aes_xts_cypher(&aes, true, unit_buf, input,
                                 encrypt_buf, size);
      DEBUGASSERT(ret == 0);

      if (memcmp(encrypt_buf, result_in30[i], size))
        {
          DEBUGASSERT(0);
        }

      memcpy(unit_buf, unit, 16);
      ret = esp32_aes_xts_cypher(&aes, false, unit_buf, encrypt_buf,
                                 decrypt_buf, size);
      DEBUGASSERT(ret == 0);

      if (memcmp(decrypt_buf, input, size))
        {
          DEBUGASSERT(0);
        }

      syslog(LOG_INFO, "ESP32 AES XTS key=%d bits test: PASS\n", keybits);
    }
}

/****************************************************************************
 * Name: esp32_aes_main
 ****************************************************************************/

int esp32_aes_main(int argc, char *argv[])
{
  esp32_aes_init();

  esp32_aes_ecb_test();
  esp32_aes_cbc_test();
  esp32_aes_ctr_test();
  esp32_aes_xts_test();

  syslog(LOG_INFO, "\nESP32 AES hardware accelerate test done.\n");

  return 0;
}

#endif
