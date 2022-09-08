/****************************************************************************
 * arch/arm/src/stm32f0l0g0/stm32_crypto.c
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

#include <errno.h>
#include <stdint.h>

#include <crypto/cryptodev.h>
#include <crypto/xform.h>
#include <nuttx/crypto/crypto.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t g_stm32_sesnum = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: authcompute
 *
 * Description:
 *   Calculate the hash.
 *
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_newsession
 *
 * Description:
 *   create new session for crypto.
 *
 ****************************************************************************/

static int stm32_newsession(uint32_t *sid, struct cryptoini *cri)
{
  if (sid == NULL || cri == NULL)
    {
      return -EINVAL;
    }

  switch (cri->cri_alg)
    {
      case CRYPTO_AES_CBC:
        *sid = g_stm32_sesnum++;
        break;
      case CRYPTO_AES_CTR:
        if ((cri->cri_klen / 8 - 4) != 16)
          {
            /* stm32 aes-ctr key bits just support 128 */

            return -EINVAL;
          }

        *sid = g_stm32_sesnum++;
        break;
      default :
        return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: stm32_freesession
 *
 * Description:
 *   free session.
 *
 ****************************************************************************/

static int stm32_freesession(uint64_t tid)
{
  return 0;
}

/****************************************************************************
 * Name: stm32_process
 *
 * Description:
 *   process session to use hardware algorithm.
 *
 ****************************************************************************/

static int stm32_process(struct cryptop *crp)
{
  struct cryptodesc *crd;
  uint8_t iv[AESCTR_BLOCKSIZE];

  for (crd = crp->crp_desc; crd; crd = crd->crd_next)
    {
      switch (crd->crd_alg)
        {
          case CRYPTO_AES_CBC:
            return aes_cypher(crp->crp_dst, crp->crp_buf, crd->crd_len,
                              crd->crd_iv, crd->crd_key, 16,
                              AES_MODE_CBC, crd->crd_flags & CRD_F_ENCRYPT);
          case CRYPTO_AES_CTR:

            memcpy(iv, crd->crd_key + crd->crd_klen / 8 - AESCTR_NONCESIZE,
                   AESCTR_NONCESIZE);
            memcpy(iv + AESCTR_NONCESIZE, crd->crd_iv, AESCTR_IVSIZE);
            memset(iv + AESCTR_NONCESIZE + AESCTR_IVSIZE , 0, 4);

            return aes_cypher(crp->crp_dst, crp->crp_buf, crd->crd_len,
                              iv, crd->crd_key, crd->crd_klen / 8 - 4,
                              AES_MODE_CTR, crd->crd_flags & CRD_F_ENCRYPT);
          default:
            return -EINVAL;
        }
    }
}

/****************************************************************************
 * Name: hwcr_init
 *
 * Description:
 *   register the hardware crypto driver.
 *
 ****************************************************************************/

void hwcr_init(void)
{
  int hwcr_id;
  int algs[CRYPTO_ALGORITHM_MAX + 1];

  hwcr_id = crypto_get_driverid(0);
  DEBUGASSERT(hwcr_id >= 0);

  memset(algs, 0, sizeof(algs));

  algs[CRYPTO_AES_CBC] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_AES_CTR] = CRYPTO_ALG_FLAG_SUPPORTED;

  crypto_register(hwcr_id, algs, stm32_newsession,
                  stm32_freesession, stm32_process);
}
