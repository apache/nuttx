/****************************************************************************
 * arch/arm/src/lpc43xx/lpc43_crypto.c
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

static uint32_t g_lpc43_sesnum;

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
 * Name: lpc43_newsession
 *
 * Description:
 *   create new session for crypto.
 *
 ****************************************************************************/

static int lpc43_newsession(uint32_t *sid, struct cryptoini *cri)
{
  if (sid == NULL || cri == NULL || cri->cri_alg != CRYPTO_AES_CBC)
    {
      return -EINVAL;
    }

  sid = g_lpc43_sesnum++;
  return OK;
}

/****************************************************************************
 * Name: lpc43_freesession
 *
 * Description:
 *   free session.
 *
 ****************************************************************************/

static int lpc43_freesession(uint64_t tid)
{
  return 0;
}

/****************************************************************************
 * Name: lpc43_process
 *
 * Description:
 *   process session to use hardware algorithm.
 *
 ****************************************************************************/

static int lpc43_process(struct cryptop *crp)
{
  struct cryptodesc *crd;

  for (crd = crp->crp_desc; crd; crd = crd->crd_next)
    {
      switch (crd->crd_alg)
        {
          case CRYPTO_AES_CBC:
            return aes_cypher(crp->crp_dst, crp->crp_buf, crd->crd_len,
                              crd->crd_iv, crd->crd_key, 16,
                              AES_MODE_CBC, crd->crd_flags & CRD_F_ENCRYPT);
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

  crypto_register(hwcr_id, algs, lpc43_newsession,
                  lpc43_freesession, lpc43_process);
}
