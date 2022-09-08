/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_crypto.c
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
#include <sys/queue.h>

#include <crypto/cryptodev.h>
#include <crypto/xform.h>
#include <nuttx/kmalloc.h>
#include <nuttx/crypto/crypto.h>

#include "esp32c3_sha.h"
#include "esp32c3_aes.h"

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

static void sha1_init(void *ctx);
static int sha1_update(void *ctx, const uint8_t *in, uint16_t len);
static void sha1_final(uint8_t *out, void *ctx);
static void sha256_init(void *ctx);
static int sha256_update(void *ctx, const uint8_t *in, uint16_t len);
static void sha256_final(uint8_t *out, void *ctx);
static int esp32c3_freesession(uint64_t tid);

/****************************************************************************
 * Private Data
 ****************************************************************************/

SLIST_HEAD(esp32c3_crypto_list, esp32c3_crypto_data);
static struct esp32c3_crypto_list *g_esp32c3_sessions = NULL;
static uint32_t g_esp32c3_sesnum = 0;

const struct auth_hash g_auth_hash_hmac_sha1_esp32c3 =
{
  CRYPTO_SHA1_HMAC, "HMAC-SHA1",
  20, 20, 12, sizeof(struct esp32c3_sha1_context_s),
  HMAC_SHA1_BLOCK_LEN,
  sha1_init, NULL, NULL,
  sha1_update,
  sha1_final
};

const struct auth_hash g_auth_hash_hmac_sha256_esp32c3 =
{
  CRYPTO_SHA2_256_HMAC, "HMAC-SHA2-256",
  32, 32, 16, sizeof(struct esp32c3_sha256_context_s),
  HMAC_SHA2_256_BLOCK_LEN,
  sha256_init, NULL, NULL,
  sha256_update,
  sha256_final
};

struct esp32c3_crypto_data
{
  int alg; /* Algorithm */
  union
  {
    struct
    {
      uint8_t *ictx;
      uint8_t *octx;
      uint32_t klen;
      const struct auth_hash *axf;
    } HWCR_AUTH;
  } HWCR_UN;

#define hw_ictx   HWCR_UN.HWCR_AUTH.ictx
#define hw_octx   HWCR_UN.HWCR_AUTH.octx
#define hw_klen   HWCR_UN.HWCR_AUTH.klen
#define hw_axf    HWCR_UN.HWCR_AUTH.axf

  SLIST_ENTRY(esp32c3_crypto_data) next;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void sha1_init(void *ctx)
{
  esp32c3_sha1_starts(ctx);
}

static int sha1_update(void *ctx, const uint8_t *in, uint16_t len)
{
  return esp32c3_sha1_update((struct esp32c3_sha1_context_s *)ctx,
                             (const unsigned char *)in,
                             (size_t)len);
}

static void sha1_final(uint8_t *out, void *ctx)
{
  esp32c3_sha1_finish((struct esp32c3_sha1_context_s *)ctx,
                      (unsigned char *)out);
}

static void sha256_init(void *ctx)
{
  esp32c3_sha256_starts(ctx, false);
}

static int sha256_update(void *ctx, const uint8_t *in, uint16_t len)
{
  return esp32c3_sha256_update((struct esp32c3_sha256_context_s *)ctx,
                               (const unsigned char *)in,
                               (size_t)len);
}

static void sha256_final(uint8_t *out, void *ctx)
{
  esp32c3_sha256_finish((struct esp32c3_sha256_context_s *)ctx,
                        (unsigned char *)out);
}

/****************************************************************************
 * Name: authcompute
 *
 * Description:
 *   Calculate the hash.
 *
 ****************************************************************************/

static int authcompute(struct cryptop *crp, struct cryptodesc *crd,
                       struct esp32c3_crypto_data *data,
                       caddr_t buf)
{
  unsigned char aalg[AALG_MAX_RESULT_LEN];
  const struct auth_hash *axf;
  int err = 0;

  if (data->hw_ictx == 0)
    {
      return -EINVAL;
    }

  axf = data->hw_axf;
  err = axf->update(data->hw_ictx, (uint8_t *)buf, crd->crd_len);
  if (err)
    {
      return err;
    }

  if (crd->crd_flags & CRD_F_ESN)
    {
      axf->update(data->hw_ictx, crd->crd_esn, 4);
    }

  switch (data->alg)
    {
      case CRYPTO_SHA1_HMAC:
      case CRYPTO_SHA2_256_HMAC:
        if (data->hw_octx == NULL)
          {
            return -EINVAL;
          }

          axf->final(aalg, data->hw_ictx);
          axf->update(data->hw_octx, aalg, axf->hashsize);
          axf->final(aalg, data->hw_octx);

        break;
    }

  /* Inject the authentication data */

  bcopy(aalg, crp->crp_mac, axf->hashsize);
  return 0;
}

/****************************************************************************
 * Name: esp32c3_newsession
 *
 * Description:
 *   create new session for crypto.
 *
 ****************************************************************************/

static int esp32c3_newsession(uint32_t *sid, struct cryptoini *cri)
{
  struct esp32c3_crypto_list *session;
  struct esp32c3_crypto_data *prev = NULL;
  struct esp32c3_crypto_data *data;
  const struct auth_hash *axf;
  int i;
  int k;

  if (sid == NULL || cri == NULL)
    {
      return -EINVAL;
    }

  for (i = 0; i < g_esp32c3_sesnum; i++)
    {
      if (SLIST_EMPTY(&g_esp32c3_sessions[i]))
        {
          break;
        }
    }

  if (i >= g_esp32c3_sesnum)
    {
      if (g_esp32c3_sessions == NULL)
        {
          g_esp32c3_sesnum = 1;
        }
      else
        {
          g_esp32c3_sesnum *= 2;
        }

      session = kmm_calloc(g_esp32c3_sesnum,
                           sizeof(struct esp32c3_crypto_list));
      if (session == NULL)
        {
          g_esp32c3_sesnum /= 2;
          return -ENOBUFS;
        }

      if (g_esp32c3_sessions != NULL)
        {
          bcopy(g_esp32c3_sessions, session, (g_esp32c3_sesnum / 2) *
                sizeof(struct esp32c3_crypto_list));
          kmm_free(g_esp32c3_sessions);
        }

      g_esp32c3_sessions = session;
    }

  session = &g_esp32c3_sessions[i];
  *sid = i;

  while (cri)
    {
      data = kmm_malloc(sizeof(struct esp32c3_crypto_data));
      if (data == NULL)
        {
          esp32c3_freesession(i);
          return -ENOBUFS;
        }

      switch (cri->cri_alg)
        {
          case CRYPTO_AES_CBC:
              break;
          case CRYPTO_AES_CTR:
            if ((cri->cri_klen / 8 - 4) != 16 &&
                (cri->cri_klen / 8 -4) != 32)
              {
                /* esp32c3 aes-ctr key bits just support 128 & 256 */

                esp32c3_freesession(i);
                kmm_free(data);
                return -EINVAL;
              }

            break;
          case CRYPTO_SHA1_HMAC:
            axf = &g_auth_hash_hmac_sha1_esp32c3;
            goto common;
          case CRYPTO_SHA2_256_HMAC:
            axf = &g_auth_hash_hmac_sha256_esp32c3;
            goto common;
          common:
            data->hw_ictx = kmm_malloc(axf->ctxsize);
            if (data->hw_ictx == NULL)
              {
                kmm_free(data);
                return -ENOBUFS;
              }

            data->hw_octx = kmm_malloc(axf->ctxsize);
            if (data->hw_octx == NULL)
              {
                kmm_free(data->hw_ictx);
                kmm_free(data);
                return -ENOBUFS;
              }

            for (k = 0; k < cri->cri_klen / 8; k++)
              {
                cri->cri_key[k] ^= HMAC_IPAD_VAL;
              }

            axf->init(data->hw_ictx);
            axf->update(data->hw_ictx, (uint8_t *)cri->cri_key,
                        cri->cri_klen / 8);
            axf->update(data->hw_ictx, hmac_ipad_buffer,
                        axf->blocksize - (cri->cri_klen / 8));

            for (k = 0; k < cri->cri_klen / 8; k++)
              {
                cri->cri_key[k] ^= (HMAC_IPAD_VAL ^ HMAC_OPAD_VAL);
              }

            axf->init(data->hw_octx);
            axf->update(data->hw_octx, (uint8_t *)cri->cri_key,
                        cri->cri_klen / 8);
            axf->update(data->hw_octx, hmac_opad_buffer,
                        axf->blocksize - (cri->cri_klen / 8));

            for (k = 0; k < cri->cri_klen / 8; k++)
              {
                cri->cri_key[k] ^= HMAC_OPAD_VAL;
              }

            data->hw_axf = axf;
            break;
          default :
            esp32c3_freesession(i);
            kmm_free(data);
            return -EINVAL;
        }

      if (prev == NULL)
        {
          SLIST_INSERT_HEAD(session, data, next);
        }
      else
        {
          SLIST_INSERT_AFTER(prev, data, next);
        }

      data->alg = cri->cri_alg;
      cri = cri->cri_next;
      prev = data;
    }

  return OK;
}

/****************************************************************************
 * Name: esp32c3_freesession
 *
 * Description:
 *   free session.
 *
 ****************************************************************************/

static int esp32c3_freesession(uint64_t tid)
{
  struct esp32c3_crypto_list *session;
  struct esp32c3_crypto_data *data;
  const struct auth_hash *axf;

  uint32_t sid = ((uint32_t)tid) & 0xffffffff;

  if (sid > g_esp32c3_sesnum || SLIST_EMPTY(&g_esp32c3_sessions[sid]))
    {
      return -EINVAL;
    }

  session = &g_esp32c3_sessions[sid];

  while (!SLIST_EMPTY(session))
    {
      data = SLIST_FIRST(session);
      switch (data->alg)
        {
          case CRYPTO_SHA1_HMAC:
          case CRYPTO_SHA2_256_HMAC:
            axf = data->hw_axf;
            if (data->hw_ictx)
              {
                explicit_bzero(data->hw_ictx, axf->ctxsize);
                kmm_free(data->hw_ictx);
              }

            if (data->hw_octx)
              {
                explicit_bzero(data->hw_octx, axf->ctxsize);
                kmm_free(data->hw_octx);
              }

            break;
        }

      SLIST_REMOVE_HEAD(session, next);
      kmm_free(data);
    }

  return 0;
}

/****************************************************************************
 * Name: esp32c3_process
 *
 * Description:
 *   process session to use hardware algorithm.
 *
 ****************************************************************************/

static int esp32c3_process(struct cryptop *crp)
{
  struct cryptodesc *crd;
  struct esp32c3_crypto_list *session;
  struct esp32c3_crypto_data *data;
  uint8_t iv[AESCTR_BLOCKSIZE];
  uint32_t lid;
  int err = 0;

  lid = crp->crp_sid & 0xffffffff;

  /* Go through crypto descriptors, processing as we go */

  session = &g_esp32c3_sessions[lid];
  for (crd = crp->crp_desc; crd; crd = crd->crd_next)
    {
      SLIST_FOREACH(data, session, next)
        {
          if (data->alg == crd->crd_alg)
            {
              break;
            }
        }

      if (data == NULL)
        {
          crp->crp_etype = EINVAL;
          return -EINVAL;
        }

      switch (data->alg)
        {
          case CRYPTO_AES_CBC:
            err = esp32c3_aes_cypher(crp->crp_dst, crp->crp_buf,
                                     crd->crd_len,
                                     crd->crd_iv, crd->crd_key, 16,
                                     AES_MODE_CBC,
                                     crd->crd_flags & CRD_F_ENCRYPT);
            if (err < 0)
              {
                return err;
              }

            break;
          case CRYPTO_AES_CTR:
            memcpy(iv, crd->crd_key + crd->crd_klen / 8 - 4, 4);
            memcpy(iv + 4, crd->crd_iv, 8);
            iv[15] = 0x1;
            err = esp32c3_aes_cypher(crp->crp_dst, crp->crp_buf,
                                     crd->crd_len,
                                     iv, crd->crd_key,
                                     crd->crd_klen / 8 - 4,
                                     AES_MODE_CTR ,
                                     crd->crd_flags & CRD_F_ENCRYPT);
            if (err < 0)
              {
                return err;
              }

            break;
          case CRYPTO_SHA1_HMAC:
          case CRYPTO_SHA2_256_HMAC:
            if ((crp->crp_etype = authcompute(crp, crd, data,
                 crp->crp_buf)) != 0)
              {
                return crp->crp_etype;
              }

            break;
          default:
            return -EINVAL;
        }
    }

  return OK;
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
  algs[CRYPTO_SHA1_HMAC] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_SHA2_256_HMAC] = CRYPTO_ALG_FLAG_SUPPORTED;

  esp32c3_sha_init();
  crypto_register(hwcr_id, algs, esp32c3_newsession,
                  esp32c3_freesession, esp32c3_process);
}
