/****************************************************************************
 * arch/arm/src/tlsr82/tlsr82_aes.c
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

#include <stdint.h>

#include <nuttx/irq.h>
#include <nuttx/crypto/crypto.h>
#include <nuttx/semaphore.h>
#include <assert.h>
#include <debug.h>
#include <semaphore.h>

#include "hardware/tlsr82_aes.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AES_BLK_SHIFT                   (4)
#define AES_BLK_SIZE                    (1 << AES_BLK_SHIFT)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private data
 ****************************************************************************/

static sem_t g_aes_excl_sem = SEM_INITIALIZER(1);

/****************************************************************************
 * Public data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tlsr82_aes_encrypt
 *
 * Description:
 *   This function servers to perform aes_128 encryption for 16-Byte input
 *   data with specific 16-Byte key.
 *
 * Input Parameters:
 *   key    - the pointer to the 16-Byte key
 *   data   - the pointer to the 16-Byte plain text
 *   result - the pointer to the encryption result cipher text
 *
 * Returned Value:
 *   0: Success, not 0: Failure
 *
 ****************************************************************************/

int tlsr82_aes_encrypt(const uint8_t *key, const uint8_t *data,
                       uint8_t *result)
{
  uint32_t tmp;
  int i;

  /* Trigger encrypt operation */

  AES_CTRL_REG &= ~AES_CTRL_CODEC_TRIG;

  /* Set the aes key */

  for (i = 0; i < 16; i++)
    {
      AES_KEY_REG(i) = key[i];
    }

  /* Feed data to hardware */

  while (AES_CTRL_REG & AES_CTRL_DATA_FEED)
    {
      tmp = data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24);
      AES_DATA_REG = tmp;
      data += 4;
    }

  /* Wait for the encrypt finished */

  while ((AES_CTRL_REG & AES_CTRL_CODEC_FINISHED) == 0);

  /* Asign the result */

  for (i = 0; i < 4; i++)
    {
      tmp  = AES_DATA_REG;
      *result++ = tmp & 0xff;
      *result++ = (tmp >> 8) & 0xff;
      *result++ = (tmp >> 16) & 0xff;
      *result++ = (tmp >> 24) & 0xff;
    }

  return 0;
}

/****************************************************************************
 * Name: tlsr82_aes_decrypt
 *
 * Description:
 *   This function servers to perform aes_128 decryption for 16-Byte input
 *   data with specific 16-Byte key.
 *
 * Input Parameters:
 *   key    - the pointer to the 16-Byte key
 *   data   - the pointer to the 16-Byte cipher text
 *   result - the pointer to the decryption result plain text
 *
 * Returned Value:
 *   0: Success, not 0: Failure
 *
 ****************************************************************************/

int tlsr82_aes_decrypt(const uint8_t *key, const uint8_t *data,
                       uint8_t *result)
{
  uint32_t tmp = 0;
  int i = 0;

  /* Trigger decrypt operation */

  AES_CTRL_REG |= AES_CTRL_CODEC_TRIG;

  /* Set the key */

  for (i = 0; i < 16; i++)
    {
      AES_KEY_REG(i) = key[i];
    }

  /* Feed data to hardware */

  while (AES_CTRL_REG & AES_CTRL_DATA_FEED)
    {
      tmp = data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24);
      AES_DATA_REG = tmp;
      data += 4;
    }

  /* Wait for decrypt finished */

  while ((AES_CTRL_REG & AES_CTRL_CODEC_FINISHED) == 0);

  /* Assign the result */

  for (i = 0; i < 4; i++)
    {
      tmp       = AES_DATA_REG;
      *result++ = tmp & 0xff;
      *result++ = (tmp >> 8) & 0xff;
      *result++ = (tmp >> 16) & 0xff;
      *result++ = (tmp >> 24) & 0xff;
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int aes_cypher(void *out, const void *in, size_t size, const void *iv,
               const void *key, size_t keysize, int mode, int encrypt)
{
  int ret = OK;
  int i;
  int nblk;

  const uint8_t *inbuf = (const uint8_t *)in;
  uint8_t *outbuf = (uint8_t *)out;

  /* Input check, error condition:
   * 1. size == 0 or size is not aligned with AES_BLK_SIZE
   * 2. not ECB mode, tlsr82 only support ECB mode
   * 3. not aes-128, tlsr82 only support aes-128
   */

  if (size == 0 || (size & (AES_BLK_SIZE - 1)) != 0)
    {
      return -EINVAL;
    }

  if (mode != AES_MODE_ECB)
    {
      return -EINVAL;
    }

  if (keysize != 16)
    {
      return -EINVAL;
    }

  ret = nxsem_wait(&g_aes_excl_sem);
  if (ret < 0)
    {
      return ret;
    }

  /* encrypt or decrypt */

  nblk = size >> AES_BLK_SHIFT;
  if (encrypt)
    {
      for (i = 0; i < nblk; i++)
        {
          tlsr82_aes_encrypt(key, inbuf, outbuf);
          inbuf  += AES_BLK_SIZE;
          outbuf += AES_BLK_SIZE;
        }
    }
  else
    {
      for (i = 0; i < nblk; i++)
        {
          tlsr82_aes_decrypt(key, inbuf, outbuf);
          inbuf  += AES_BLK_SIZE;
          outbuf += AES_BLK_SIZE;
        }
    }

  ret = nxsem_post(&g_aes_excl_sem);
  if (ret < 0)
    {
      return ret;
    }

  return ret;
}
