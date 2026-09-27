/****************************************************************************
 * arch/arm64/src/common/arm64_crypto.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <assert.h>
#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <crypto/cryptodev.h>
#include <crypto/xform.h>
#include <nuttx/crypto/crypto.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int arm64_crypto_newsession(FAR uint32_t *sid,
                                   FAR struct cryptoini *cri)
{
  if (sid == NULL || cri == NULL)
    {
      return -EINVAL;
    }

  switch (cri->cri_alg)
    {
      case CRYPTO_AES_CBC:
        break;

      case CRYPTO_AES_CTR:

        /* cri_klen counts the trailing nonce, which is not key material. */

        if ((cri->cri_klen / 8 - AESCTR_NONCESIZE) > 32)
          {
            return -EINVAL;
          }
        break;

      default:
        return -EINVAL;
    }

  return OK;
}

static int arm64_crypto_freesession(uint64_t tid)
{
  return OK;
}

static int arm64_crypto_process(FAR struct cryptop *crp)
{
  FAR struct cryptodesc *crd;
  uint8_t iv[AESCTR_BLOCKSIZE];

  for (crd = crp->crp_desc; crd != NULL; crd = crd->crd_next)
    {
      switch (crd->crd_alg)
        {
          case CRYPTO_AES_CBC:
            return aes_cypher(crp->crp_dst, crp->crp_buf, crd->crd_len,
                              crd->crd_iv, crd->crd_key, crd->crd_klen / 8,
                              AES_MODE_CBC,
                              crd->crd_flags & CRD_F_ENCRYPT);

          case CRYPTO_AES_CTR:
            memcpy(iv, crd->crd_key + crd->crd_klen / 8 - AESCTR_NONCESIZE,
                   AESCTR_NONCESIZE);
            memcpy(iv + AESCTR_NONCESIZE, crd->crd_iv, AESCTR_IVSIZE);
            memset(iv + AESCTR_NONCESIZE + AESCTR_IVSIZE, 0,
                   AESCTR_BLOCKSIZE - AESCTR_NONCESIZE - AESCTR_IVSIZE);
            return aes_cypher(crp->crp_dst, crp->crp_buf, crd->crd_len,
                              iv, crd->crd_key,
                              crd->crd_klen / 8 - AESCTR_NONCESIZE,
                              AES_MODE_CTR,
                              crd->crd_flags & CRD_F_ENCRYPT);

          default:
            return -EINVAL;
        }
    }

  return -EINVAL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hwcr_init
 *
 * Description:
 *   Register the Armv8 AES instructions with /dev/crypto.
 *
 ****************************************************************************/

void hwcr_init(void)
{
  int algs[CRYPTO_ALGORITHM_MAX + 1];
  int hwcr_id;

  hwcr_id = crypto_get_driverid(0);
  DEBUGASSERT(hwcr_id >= 0);

  memset(algs, 0, sizeof(algs));
  algs[CRYPTO_AES_CBC] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_AES_CTR] = CRYPTO_ALG_FLAG_SUPPORTED;

  crypto_register(hwcr_id, algs, arm64_crypto_newsession,
                  arm64_crypto_freesession, arm64_crypto_process);
}
