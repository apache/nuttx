/****************************************************************************
 * arch/xtensa/src/esp32/esp32_crypto.c
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

#include "esp32_sha.h"
#include <stdio.h>

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

static void sha1_init(void *ctx);
static void sha256_init(void *ctx);
static void sha384_init(void *ctx);
static void sha512_init(void *ctx);
static int sha_update(void *ctx, const uint8_t *in, size_t len);
static void sha_final(uint8_t *out, void *ctx);
static int esp32_freesession(uint64_t sid);

/****************************************************************************
 * Private Data
 ****************************************************************************/

SLIST_HEAD(esp32_crypto_list, esp32_crypto_data);
static struct esp32_crypto_list *g_esp32_sessions = NULL;
static uint32_t g_esp32_sesnum = 0;

const struct auth_hash g_auth_hash_sha1_esp32 =
{
  CRYPTO_SHA1, "SHA1",
  0, 20, 12, sizeof(struct esp32_sha_context_s),
  0,
  sha1_init, NULL, NULL,
  sha_update,
  sha_final
};

const struct auth_hash g_auth_hash_sha2_256_esp32 =
{
  CRYPTO_SHA2_256, "SHA256",
  0, 32, 12, sizeof(struct esp32_sha_context_s),
  0,
  sha256_init, NULL, NULL,
  sha_update,
  sha_final
};

const struct auth_hash g_auth_hash_sha2_384_esp32 =
{
  CRYPTO_SHA2_384, "SHA384",
  0, 48, 12, sizeof(struct esp32_sha_context_s),
  0,
  sha384_init, NULL, NULL,
  sha_update,
  sha_final
};

const struct auth_hash g_auth_hash_sha2_512_esp32 =
{
  CRYPTO_SHA2_512, "SHA512",
  0, 64, 12, sizeof(struct esp32_sha_context_s),
  0,
  sha512_init, NULL, NULL,
  sha_update,
  sha_final
};

struct esp32_crypto_data
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

  SLIST_ENTRY(esp32_crypto_data) next;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sha1_init
 *
 * Description:
 *   Initialize the SHA-1 context.
 *
 * Input Parameters:
 *   ctx      - The SHA-1 context to be initialized
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sha1_init(void *ctx)
{
  esp32_sha1_starts(ctx);
}

/****************************************************************************
 * Name: sha256_init
 *
 * Description:
 *   Initialize the SHA-256 context.
 *
 * Input Parameters:
 *   ctx      - The SHA-256 context to be initialized
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sha256_init(void *ctx)
{
  esp32_sha256_starts(ctx);
}

/****************************************************************************
 * Name: sha384_init
 *
 * Description:
 *   Initialize the SHA-384 context.
 *
 * Input Parameters:
 *   ctx      - The SHA-384 context to be initialized
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sha384_init(void *ctx)
{
  esp32_sha384_starts(ctx);
}

/****************************************************************************
 * Name: sha512_init
 *
 * Description:
 *   Initialize the SHA-512 context.
 *
 * Input Parameters:
 *   ctx      - The SHA-512 context to be initialized
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sha512_init(void *ctx)
{
  esp32_sha512_starts(ctx);
}

/****************************************************************************
 * Name: sha_update
 *
 * Description:
 *   Feeds an input buffer into an ongoing SHA checksum calculation.
 *
 * Input Parameters:
 *   ctx      - The SHA context to use
 *   in       - The buffer holding the input data
 *   len      - The length of the input data in Bytes
 *
 * Returned Value:
 *   OK is returned on success.
 *   Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

static int sha_update(void *ctx, const uint8_t *in, size_t len)
{
  return esp32_sha_update((struct esp32_sha_context_s *)ctx,
                          (const unsigned char *)in,
                          (size_t)len);
}

/****************************************************************************
 * Name: sha_final
 *
 * Description:
 *   Finishes the SHA operation, and writes the result to the output buffer.
 *
 * Input Parameters:
 *   out      - The SHA checksum result
 *   ctx      - The SHA context to use
 *
 * Returned Value:
 *   OK is returned on success.
 *   Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

static void sha_final(uint8_t *out, void *ctx)
{
  esp32_sha_finish((struct esp32_sha_context_s *)ctx,
                   (unsigned char *)out);
}

/****************************************************************************
 * Name: hash
 *
 * Description:
 *   Calculate the hash.
 *
 * Input Parameters:
 *   crp      - The description of the crypto operation
 *   crd      - Boundaries of the crypto operation
 *   data     - ESP32 specific crypto operation data
 *   buf      - Input data to be hashed
 *
 * Returned Value:
 *   OK is returned on success.
 *   Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

static int hash(struct cryptop *crp,
                struct cryptodesc *crd,
                struct esp32_crypto_data *data,
                caddr_t buf)
{
  const struct auth_hash *axf;

  if (data->hw_ictx == 0)
    {
      return -EINVAL;
    }

  axf = data->hw_axf;

  if (crd->crd_flags & CRD_F_UPDATE)
    {
      return axf->update(data->hw_ictx, (uint8_t *)buf, crd->crd_len);
    }
  else
    {
      axf->final((uint8_t *)crp->crp_mac, data->hw_ictx);
    }

  return 0;
}

/****************************************************************************
 * Name: esp32_newsession
 *
 * Description:
 *   create new session for crypto.
 *
 * Input Parameters:
 *   sid      - Session id
 *   cri      - Initialization structure
 *
 * Returned Value:
 *   OK is returned on success.
 *   Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

static int esp32_newsession(uint32_t *sid, struct cryptoini *cri)
{
  struct esp32_crypto_list *session;
  struct esp32_crypto_data *prev = NULL;
  struct esp32_crypto_data *data;
  const struct auth_hash *axf;
  int i;
  int k;

  if (sid == NULL || cri == NULL)
    {
      return -EINVAL;
    }

  for (i = 0; i < g_esp32_sesnum; i++)
    {
      if (SLIST_EMPTY(&g_esp32_sessions[i]))
        {
          break;
        }
    }

  if (i >= g_esp32_sesnum)
    {
      if (g_esp32_sessions == NULL)
        {
          g_esp32_sesnum = 1;
        }
      else
        {
          g_esp32_sesnum *= 2;
        }

      session = kmm_calloc(g_esp32_sesnum,
                           sizeof(struct esp32_crypto_list));
      if (session == NULL)
        {
          g_esp32_sesnum /= 2;
          return -ENOBUFS;
        }

      if (g_esp32_sessions != NULL)
        {
          bcopy(g_esp32_sessions, session, (g_esp32_sesnum / 2) *
                sizeof(struct esp32_crypto_list));
          kmm_free(g_esp32_sessions);
        }

      g_esp32_sessions = session;
    }

  session = &g_esp32_sessions[i];
  *sid = i;

  while (cri)
    {
      data = kmm_malloc(sizeof(struct esp32_crypto_data));

      if (data == NULL)
        {
          esp32_freesession(i);
          return -ENOBUFS;
        }

      switch (cri->cri_alg)
        {
          case CRYPTO_SHA1:
            axf = &g_auth_hash_sha1_esp32;
            goto common;
          case CRYPTO_SHA2_256:
            axf = &g_auth_hash_sha2_256_esp32;
            goto common;
          case CRYPTO_SHA2_384:
            axf = &g_auth_hash_sha2_384_esp32;
            goto common;
          case CRYPTO_SHA2_512:
            axf = &g_auth_hash_sha2_512_esp32;
            goto common;
            common:
              data->hw_ictx = kmm_malloc(axf->ctxsize);
              if (data->hw_ictx == NULL)
                {
                  kmm_free(data);
                  return -ENOBUFS;
                }

              axf->init(data->hw_ictx);
              data->hw_axf = axf;
            break;

          default:
            esp32_freesession(i);
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
 * Name: esp32_freesession
 *
 * Description:
 *   free session.
 *
 * Input Parameters:
 *   sid      - Session id
 *
 * Returned Value:
 *   OK is returned on success.
 *   Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

static int esp32_freesession(uint64_t sid)
{
  struct esp32_crypto_list *session;
  struct esp32_crypto_data *data;
  const struct auth_hash *axf;

  if (sid > g_esp32_sesnum || SLIST_EMPTY(&g_esp32_sessions[sid]))
    {
      return -EINVAL;
    }

  session = &g_esp32_sessions[sid];

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
 * Name: esp32_process
 *
 * Description:
 *   process session to use hardware algorithm.
 *
 * Input Parameters:
 *   crp      - The description of the crypto operation
 *
 * Returned Value:
 *   OK is returned on success.
 *   Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

static int esp32_process(struct cryptop *crp)
{
  struct cryptodesc *crd;
  struct esp32_crypto_list *session;
  struct esp32_crypto_data *data;
  uint8_t iv[AESCTR_BLOCKSIZE];
  uint32_t lid;
  int err = 0;
  lid = crp->crp_sid & 0xffffffff;

  /* Go through crypto descriptors, processing as we go */

  session = &g_esp32_sessions[lid];
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
          case CRYPTO_SHA1:
          case CRYPTO_SHA2_256:
          case CRYPTO_SHA2_384:
          case CRYPTO_SHA2_512:
            if ((crp->crp_etype = hash(crp, crd, data,
                crp->crp_buf)) != 0)
              {
                return 0;
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

  algs[CRYPTO_SHA1] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_SHA2_256] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_SHA2_384] = CRYPTO_ALG_FLAG_SUPPORTED;
  algs[CRYPTO_SHA2_512] = CRYPTO_ALG_FLAG_SUPPORTED;

  esp32_sha_init();
  crypto_register(hwcr_id, algs, esp32_newsession,
                  esp32_freesession, esp32_process);
}
