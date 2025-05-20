/****************************************************************************
 * arch/xtensa/src/common/espressif/esp_crypto.c
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

#include <errno.h>
#include <stddef.h>
#include <sys/queue.h>

#include <crypto/cryptodev.h>
#include <crypto/xform.h>
#include <nuttx/kmalloc.h>
#include <nuttx/crypto/crypto.h>

#include "esp_sha.h"

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

static void sha1_init(void *ctx);
static int sha1_update(void *ctx, const uint8_t *in, size_t len);
static void sha1_final(uint8_t *out, void *ctx);
static void sha256_init(void *ctx);
static int sha256_update(void *ctx, const uint8_t *in, size_t len);
static void sha256_final(uint8_t *out, void *ctx);
static void sha384_init(void *ctx);
static void sha512_init(void *ctx);
static int sha512_update(void *ctx, const uint8_t *in, size_t len);
static void sha512_final(uint8_t *out, void *ctx);
static int esp_freesession(uint64_t tid);

/****************************************************************************
 * Private Data
 ****************************************************************************/

SLIST_HEAD(esp_crypto_list, esp_crypto_data);
static struct esp_crypto_list *g_esp_sessions = NULL;
static uint32_t g_esp_sesnum = 0;

const struct auth_hash g_auth_hash_hmac_sha1_esp =
{
  CRYPTO_SHA1_HMAC, "HMAC-SHA1",
  20, 20, 12, sizeof(struct esp_sha1_context_s),
  HMAC_SHA1_BLOCK_LEN,
  sha1_init, NULL, NULL,
  sha1_update,
  sha1_final
};

const struct auth_hash g_auth_hash_hmac_sha256_esp =
{
  CRYPTO_SHA2_256_HMAC, "HMAC-SHA2-256",
  32, 32, 16, sizeof(struct esp_sha256_context_s),
  HMAC_SHA2_256_BLOCK_LEN,
  sha256_init, NULL, NULL,
  sha256_update,
  sha256_final
};

const struct auth_hash g_auth_hash_hmac_sha384_esp =
{
  CRYPTO_SHA2_384_HMAC, "HMAC-SHA3-384",
  0, 48, 12, sizeof(struct esp_sha512_context_s),
  0,
  sha384_init, NULL, NULL,
  sha512_update,
  sha512_final
};

const struct auth_hash g_auth_hash_hmac_sha512_esp =
{
  CRYPTO_SHA2_512_HMAC, "HMAC-SHA3-512",
  0, 64, 12, sizeof(struct esp_sha512_context_s),
  0,
  sha512_init, NULL, NULL,
  sha512_update,
  sha512_final
};

struct esp_crypto_data
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

  SLIST_ENTRY(esp_crypto_data) next;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sha1_init
 *
 * Description:
 *   Starts a SHA-1 checksum calculation.
 *
 * Input Parameters:
 *   ctx - The SHA-1 context to initialize
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sha1_init(void *ctx)
{
  esp_sha1_starts(ctx);
}

/****************************************************************************
 * Name: sha1_update
 *
 * Description:
 *   Feeds an input buffer into an ongoing SHA-1 checksum calculation.
 *
 * Input Parameters:
 *   ctx - The SHA-1 context to use
 *   in  - The buffer holding the input data
 *   len - The length of the input data in Bytes
 *
 * Returned Value:
 *   OK is returned on success.
 *   Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

static int sha1_update(void *ctx, const uint8_t *in, size_t len)
{
  return esp_sha1_update((struct esp_sha1_context_s *)ctx,
                             (const unsigned char *)in,
                             (size_t)len);
  return OK;
}

/****************************************************************************
 * Name: sha1_final
 *
 * Description:
 *   Finishes the SHA-1 operation,
 *   and writes the result to the output buffer.
 *
 * Input Parameters:
 *   out - The SHA-1 checksum result
 *   ctx - The SHA-1 context to use
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sha1_final(uint8_t *out, void *ctx)
{
  esp_sha1_finish((struct esp_sha1_context_s *)ctx,
                      (unsigned char *)out);
}

/****************************************************************************
 * Name: sha256_init
 *
 * Description:
 *   Initializes a SHA-256 context.
 *
 * Input Parameters:
 *   ctx - The SHA-256 context to initialize
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void sha256_init(void *ctx)
{
  esp_sha256_starts(ctx, false);
}

/****************************************************************************
 * Name: sha256_update
 *
 * Description:
 *   Feeds an input buffer into an ongoing SHA-224 or SHA-256
 *   checksum calculation.
 *
 * Input Parameters:
 *   ctx - The SHA-256 context to use
 *   in  - The buffer holding the input data
 *   len - The length of the input data in Bytes
 *
 * Returned Value:
 *   OK is returned on success.
 *   Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

static int sha256_update(void *ctx, const uint8_t *in, size_t len)
{
  return esp_sha256_update((struct esp_sha256_context_s *)ctx,
                               (const unsigned char *)in,
                               (size_t)len);
  return OK;
}

/****************************************************************************
 * Name: sha256_final
 *
 * Description:
 *   Finishes the SHA-224 or SHA-256 operation, and writes the result to
 *   the output buffer.
 *
 * Input Parameters:
 *   out - The SHA-256 checksum result
 *   ctx - The SHA-256 context to use
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void sha256_final(uint8_t *out, void *ctx)
{
  esp_sha256_finish((struct esp_sha256_context_s *)ctx,
                        (unsigned char *)out);
}

/****************************************************************************
 * Name: sha384_init
 *
 * Description:
 *   Initializes a SHA-384 context.
 *
 * Input Parameters:
 *   ctx - The SHA-512 context to initialize
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void sha384_init(void *ctx)
{
  esp_sha512_starts(ctx, true);
}

/****************************************************************************
 * Name: sha384_init
 *
 * Description:
 *   Initializes a SHA-512 context.
 *
 * Input Parameters:
 *   ctx - The SHA-512 context to initialize
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void sha512_init(void *ctx)
{
  esp_sha512_starts(ctx, false);
}

/****************************************************************************
 * Name: sha512_update
 *
 * Description:
 *   Feeds an input buffer into an ongoing SHA-384 or SHA-512
 *   checksum calculation.
 *
 * Input Parameters:
 *   ctx - The SHA-256 context to use
 *   in  - The buffer holding the input data
 *   len - The length of the input data in Bytes
 *
 * Returned Value:
 *   OK is returned on success.
 *   Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

static int sha512_update(void *ctx, const uint8_t *in, size_t len)
{
  return esp_sha512_update((struct esp_sha512_context_s *)ctx,
                               (const unsigned char *)in,
                               (size_t)len);
  return OK;
}

/****************************************************************************
 * Name: sha256_final
 *
 * Description:
 *   Finishes the SHA-384 or SHA-512 operation, and writes the result to
 *   the output buffer.
 *
 * Input Parameters:
 *   out - The SHA-256 checksum result
 *   ctx - The SHA-256 context to use
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void sha512_final(uint8_t *out, void *ctx)
{
  esp_sha512_finish((struct esp_sha512_context_s *)ctx,
                        (unsigned char *)out);
}

/****************************************************************************
 * Name: authcompute
 *
 * Description:
 *   Calculate the hash.
 *
 * Input Parameters:
 *   crp  - Pointer of cryptop struct
 *   crd  - Pointer of description struct
 *   data - Internal struct for crypt
 *   buf  - Buffer to compute
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int authcompute(struct cryptop *crp, struct cryptodesc *crd,
                       struct esp_crypto_data *data,
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
  return OK;
}

/****************************************************************************
 * Name: esp_newsession
 *
 * Description:
 *   Create new session for crypto.
 *
 * Input Parameters:
 *   sid - Session id
 *   cri - Pointer of cryptop struct
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int esp_newsession(uint32_t *sid, struct cryptoini *cri)
{
  struct esp_crypto_list *session;
  struct esp_crypto_data *prev = NULL;
  struct esp_crypto_data *data;
  const struct auth_hash *axf;
  int i;
  int k;

  if (sid == NULL || cri == NULL)
    {
      return -EINVAL;
    }

  for (i = 0; i < g_esp_sesnum; i++)
    {
      if (SLIST_EMPTY(&g_esp_sessions[i]))
        {
          break;
        }
    }

  if (i >= g_esp_sesnum)
    {
      if (g_esp_sessions == NULL)
        {
          g_esp_sesnum = 1;
        }
      else
        {
          g_esp_sesnum *= 2;
        }

      session = kmm_calloc(g_esp_sesnum,
                           sizeof(struct esp_crypto_list));
      if (session == NULL)
        {
          g_esp_sesnum /= 2;
          return -ENOBUFS;
        }

      if (g_esp_sessions != NULL)
        {
          bcopy(g_esp_sessions, session, (g_esp_sesnum / 2) *
                sizeof(struct esp_crypto_list));
          kmm_free(g_esp_sessions);
        }

      g_esp_sessions = session;
    }

  session = &g_esp_sessions[i];
  *sid = i;

  while (cri)
    {
      data = kmm_malloc(sizeof(struct esp_crypto_data));
      if (data == NULL)
        {
          esp_freesession(i);
          return -ENOBUFS;
        }

      switch (cri->cri_alg)
        {
          case CRYPTO_SHA1_HMAC:
            axf = &g_auth_hash_hmac_sha1_esp;
            goto common;
          case CRYPTO_SHA2_256_HMAC:
            axf = &g_auth_hash_hmac_sha256_esp;
            goto common;
          case CRYPTO_SHA2_384_HMAC:
            axf = &g_auth_hash_hmac_sha384_esp;
            goto common;
          case CRYPTO_SHA2_512_HMAC:
            axf = &g_auth_hash_hmac_sha512_esp;
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
          default:

            esp_freesession(i);
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
 * Name: esp_freesession
 *
 * Description:
 *   Free session.
 *
 * Input Parameters:
 *   tid - Session id
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int esp_freesession(uint64_t tid)
{
  struct esp_crypto_list *session;
  struct esp_crypto_data *data;
  const struct auth_hash *axf;

  uint32_t sid = ((uint32_t)tid) & 0xffffffff;

  if (sid > g_esp_sesnum || SLIST_EMPTY(&g_esp_sessions[sid]))
    {
      return -EINVAL;
    }

  session = &g_esp_sessions[sid];

  while (!SLIST_EMPTY(session))
    {
      data = SLIST_FIRST(session);
      switch (data->alg)
        {
          case CRYPTO_SHA1_HMAC:
          case CRYPTO_SHA2_256_HMAC:
          case CRYPTO_SHA2_384_HMAC:
          case CRYPTO_SHA2_512_HMAC:
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

  return OK;
}

/****************************************************************************
 * Name: esp_process
 *
 * Description:
 *   Process session to use hardware algorithm.
 *
 * Input Parameters:
 *   crp - Pointer of cryptop struct
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int esp_process(struct cryptop *crp)
{
  struct cryptodesc *crd;
  struct esp_crypto_list *session;
  struct esp_crypto_data *data;
  uint32_t lid;
  int err = 0;

  lid = crp->crp_sid & 0xffffffff;

  /* Go through crypto descriptors, processing as we go */

  session = &g_esp_sessions[lid];
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
          case CRYPTO_SHA1_HMAC:
          case CRYPTO_SHA2_256_HMAC:
          case CRYPTO_SHA2_384_HMAC:
          case CRYPTO_SHA2_512_HMAC:
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
 *   Register the hardware crypto driver.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void hwcr_init(void)
{
  int hwcr_id;
  int algs[CRYPTO_ALGORITHM_MAX + 1];

  hwcr_id = crypto_get_driverid(0);
  DEBUGASSERT(hwcr_id >= 0);

  memset(algs, 0, sizeof(algs));

  algs[CRYPTO_SHA1_HMAC] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_SHA2_256_HMAC] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_SHA2_384_HMAC] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_SHA2_512_HMAC] = CRYPTO_ALG_FLAG_SUPPORTED;

  esp_sha_init();
  crypto_register(hwcr_id, algs, esp_newsession,
                  esp_freesession, esp_process);
}
