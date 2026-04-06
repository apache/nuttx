/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_crypto.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with this
 * work for additional information regarding copyright ownership.  The ASF
 * licenses this file to you under the Apache License, Version 2.0 (the
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

#ifdef CONFIG_CRYPTO_CRYPTODEV_HARDWARE

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <sys/queue.h>

#include <crypto/cryptodev.h>
#include <crypto/xform.h>
#include <nuttx/crypto/crypto.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>

#include "arm_internal.h"
#include "hardware/rp23xx_sha256.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RP23XX_SHA256_BLOCK_SIZE   64
#define RP23XX_SHA256_PADDING_SIZE 9
#define RP23XX_SHA256_RESULT_SIZE  32

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rp23xx_sha256_context_s
{
  bool started;
  uint8_t cache_used;
  size_t total_data_size;
  union
  {
    uint32_t word;
    uint8_t bytes[4];
  } cache;
};

struct rp23xx_crypto_data
{
  int alg;
  union
  {
    struct
    {
      FAR struct rp23xx_sha256_context_s *ictx;
      FAR const struct auth_hash *axf;
    } auth;
  } u;

#define hw_ictx u.auth.ictx
#define hw_axf  u.auth.axf

  SLIST_ENTRY(rp23xx_crypto_data) next;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void sha256_init(FAR void *ctx);
static int sha256_update(FAR void *ctx, FAR const uint8_t *in, size_t len);
static void sha256_final(FAR uint8_t *out, FAR void *ctx);
static int rp23xx_freesession(uint64_t tid);

/****************************************************************************
 * Private Data
 ****************************************************************************/

SLIST_HEAD(rp23xx_crypto_list, rp23xx_crypto_data);
static FAR struct rp23xx_crypto_list *g_rp23xx_sessions;
static uint32_t g_rp23xx_sesnum;
static mutex_t g_rp23xx_sha_lock = NXMUTEX_INITIALIZER;
static FAR struct rp23xx_sha256_context_s *g_rp23xx_active_ctx;

static const struct auth_hash g_auth_hash_sha256_rp23xx =
{
  CRYPTO_SHA2_256, "SHA256",
  0, 32, 12, sizeof(struct rp23xx_sha256_context_s),
  0,
  sha256_init, NULL, NULL,
  sha256_update,
  sha256_final
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline bool rp23xx_sha256_is_ready(void)
{
  return (getreg32(RP23XX_SHA256_CSR) & RP23XX_SHA256_CSR_WDATA_RDY) != 0;
}

static inline bool rp23xx_sha256_is_valid(void)
{
  return (getreg32(RP23XX_SHA256_CSR) & RP23XX_SHA256_CSR_SUM_VLD) != 0;
}

static inline bool rp23xx_sha256_err_not_ready(void)
{
  return (getreg32(RP23XX_SHA256_CSR) &
          RP23XX_SHA256_CSR_ERR_WDATA_NOT_RDY) != 0;
}

static inline void rp23xx_sha256_wait_ready(void)
{
  while (!rp23xx_sha256_is_ready())
    {
    }
}

static inline void rp23xx_sha256_wait_valid(void)
{
  while (!rp23xx_sha256_is_valid())
    {
    }
}

static inline void rp23xx_sha256_start_engine(void)
{
  uint32_t regval;

  /* The CSR contains a mix of write-one-to-clear and self-clearing control
   * bits, so avoid atomic alias writes here. Preserve the writable config
   * fields, clear any stale not-ready error, force big-endian SHA word
   * ordering via BSWAP, and pulse START in one direct write.
   */

  regval = getreg32(RP23XX_SHA256_CSR);
  regval &= (RP23XX_SHA256_CSR_BSWAP | RP23XX_SHA256_CSR_DMA_SIZE_MASK);
  regval |= (RP23XX_SHA256_CSR_ERR_WDATA_NOT_RDY |
             RP23XX_SHA256_CSR_BSWAP |
             RP23XX_SHA256_CSR_START);
  putreg32(regval, RP23XX_SHA256_CSR);
}

static void rp23xx_sha256_get_result(FAR uint8_t *out)
{
  int i;

  for (i = 0; i < RP23XX_SHA256_RESULT_SIZE / 4; i++)
    {
      uint32_t data = getreg32(RP23XX_SHA256_SUM(i));
      data = __builtin_bswap32(data);
      memcpy(out + i * sizeof(uint32_t), &data, sizeof(uint32_t));
    }
}

static int rp23xx_sha256_try_start(FAR struct rp23xx_sha256_context_s *ctx)
{
  if (g_rp23xx_active_ctx != NULL && g_rp23xx_active_ctx != ctx)
    {
      return -EBUSY;
    }

  rp23xx_sha256_start_engine();
  ctx->started = true;
  g_rp23xx_active_ctx = ctx;
  return OK;
}

static void rp23xx_sha256_write_to_hardware(
    FAR struct rp23xx_sha256_context_s *ctx,
    FAR const uint8_t *data, size_t data_size_bytes)
{
  /* Mirror the Pico SDK's non-DMA flow: use aligned word writes when
   * possible, otherwise accumulate bytes into a 32-bit cache and write full
   * words once available.
   */

  if (ctx->cache_used == 0 && (((uintptr_t)data) & 3) == 0)
    {
      FAR const uint32_t *data32 = (FAR const uint32_t *)data;

      while (data_size_bytes >= sizeof(uint32_t))
        {
          rp23xx_sha256_wait_ready();
          putreg32(*data32++, RP23XX_SHA256_WDATA);
          data_size_bytes -= sizeof(uint32_t);
        }

      data = (FAR const uint8_t *)data32;
    }

  while (data_size_bytes-- > 0)
    {
      ctx->cache.bytes[ctx->cache_used++] = *data++;
      if (ctx->cache_used == sizeof(uint32_t))
        {
          ctx->cache_used = 0;
          rp23xx_sha256_wait_ready();
          putreg32(ctx->cache.word, RP23XX_SHA256_WDATA);
        }
    }
}

static int rp23xx_sha256_update_internal(
    FAR struct rp23xx_sha256_context_s *ctx,
    FAR const uint8_t *data, size_t data_size_bytes)
{
  size_t bytes_left;

  if (data_size_bytes == 0 || data == NULL)
    {
      return OK;
    }

  if (!ctx->started)
    {
      int ret = rp23xx_sha256_try_start(ctx);

      if (ret < 0)
        {
          return ret;
        }
    }
  else if (g_rp23xx_active_ctx != ctx)
    {
      return -EBUSY;
    }

  /* Keep the same block-boundary handling as the Pico SDK: finish the
   * current 64-byte hardware block first, otherwise the peripheral reports
   * WDATA-not-ready if we run past the boundary without waiting.
   */

  bytes_left = ((ctx->total_data_size +
                (RP23XX_SHA256_BLOCK_SIZE - 1)) &
                ~(RP23XX_SHA256_BLOCK_SIZE - 1)) - ctx->total_data_size;
  if (bytes_left > data_size_bytes)
    {
      bytes_left = data_size_bytes;
    }

  if (bytes_left > 0)
    {
      rp23xx_sha256_write_to_hardware(ctx, data, bytes_left);
      ctx->total_data_size += bytes_left;
      data += bytes_left;
      data_size_bytes -= bytes_left;
    }

  if (data_size_bytes > 0)
    {
      rp23xx_sha256_write_to_hardware(ctx, data, data_size_bytes);
      ctx->total_data_size += data_size_bytes;
    }

  if (rp23xx_sha256_err_not_ready())
    {
      return -EIO;
    }

  return OK;
}

static int rp23xx_sha256_add_zero_bytes(
    FAR struct rp23xx_sha256_context_s *ctx, size_t len)
{
  static const uint32_t zero = 0;
  int ret;

  while (len > 0)
    {
      size_t chunk = len > sizeof(zero) ? sizeof(zero) : len;

      ret = rp23xx_sha256_update_internal(ctx, (FAR const uint8_t *)&zero,
                                          chunk);
      if (ret < 0)
        {
          return ret;
        }

      len -= chunk;
    }

  return OK;
}

static int rp23xx_sha256_write_padding(
    FAR struct rp23xx_sha256_context_s *ctx)
{
  uint64_t size;
  size_t user_data_size;
  size_t padding_size_bytes;
  uint8_t one_bit = 0x80;
  int ret;

  size = (ctx->total_data_size + RP23XX_SHA256_PADDING_SIZE +
          (RP23XX_SHA256_BLOCK_SIZE - 1)) &
         ~(RP23XX_SHA256_BLOCK_SIZE - 1);
  user_data_size = ctx->total_data_size;
  padding_size_bytes = size - ctx->total_data_size;

  /* Keep padding byte-for-byte consistent with the Pico SDK flow. */

  ret = rp23xx_sha256_update_internal(ctx, &one_bit, 1);
  if (ret < 0)
    {
      return ret;
    }

  ret = rp23xx_sha256_add_zero_bytes(
      ctx, padding_size_bytes - RP23XX_SHA256_PADDING_SIZE);
  if (ret < 0)
    {
      return ret;
    }

  size = __builtin_bswap64((uint64_t)user_data_size * 8);
  return rp23xx_sha256_update_internal(ctx, (FAR const uint8_t *)&size,
                                       sizeof(size));
}

static void sha256_init(FAR void *ctx)
{
  memset(ctx, 0, sizeof(struct rp23xx_sha256_context_s));
}

static int sha256_update(FAR void *ctx, FAR const uint8_t *in, size_t len)
{
  FAR struct rp23xx_sha256_context_s *sha =
    (FAR struct rp23xx_sha256_context_s *)ctx;
  int ret;

  ret = nxmutex_lock(&g_rp23xx_sha_lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = rp23xx_sha256_update_internal(sha, in, len);
  nxmutex_unlock(&g_rp23xx_sha_lock);
  return ret;
}

static void sha256_final(FAR uint8_t *out, FAR void *ctx)
{
  FAR struct rp23xx_sha256_context_s *sha =
    (FAR struct rp23xx_sha256_context_s *)ctx;
  int ret;

  if (nxmutex_lock(&g_rp23xx_sha_lock) < 0)
    {
      return;
    }

  if (!sha->started)
    {
      ret = rp23xx_sha256_try_start(sha);
      if (ret < 0)
        {
          goto out;
        }
    }

  if (g_rp23xx_active_ctx != sha)
    {
      goto out;
    }

  ret = rp23xx_sha256_write_padding(sha);
  if (ret < 0)
    {
      goto out;
    }

  rp23xx_sha256_wait_valid();
  if (out != NULL)
    {
      rp23xx_sha256_get_result(out);
    }

out:
  memset(sha, 0, sizeof(*sha));
  if (g_rp23xx_active_ctx == sha)
    {
      g_rp23xx_active_ctx = NULL;
    }

  nxmutex_unlock(&g_rp23xx_sha_lock);
}

static int rp23xx_hash(FAR struct cryptop *crp,
                       FAR struct cryptodesc *crd,
                       FAR struct rp23xx_crypto_data *data,
                       caddr_t buf)
{
  FAR const struct auth_hash *axf = data->hw_axf;

  if (data->hw_ictx == NULL)
    {
      return -EINVAL;
    }

  if (crd->crd_flags & CRD_F_UPDATE)
    {
      return axf->update(data->hw_ictx,
                         (FAR uint8_t *)buf + crd->crd_skip,
                         crd->crd_len);
    }

  axf->final((FAR uint8_t *)crp->crp_mac, data->hw_ictx);
  return OK;
}

static int rp23xx_newsession(uint32_t *sid, FAR struct cryptoini *cri)
{
  FAR struct rp23xx_crypto_list *session;
  FAR struct rp23xx_crypto_data *data;
  FAR const struct auth_hash *axf;
  int i;

  if (sid == NULL || cri == NULL)
    {
      return -EINVAL;
    }

  for (i = 0; i < g_rp23xx_sesnum; i++)
    {
      if (SLIST_EMPTY(&g_rp23xx_sessions[i]))
        {
          break;
        }
    }

  if (i >= g_rp23xx_sesnum)
    {
      FAR struct rp23xx_crypto_list *new_sessions;
      uint32_t new_count =
        g_rp23xx_sessions == NULL ? 1 : g_rp23xx_sesnum * 2;

      new_sessions = kmm_calloc(new_count, sizeof(*new_sessions));
      if (new_sessions == NULL)
        {
          return -ENOBUFS;
        }

      if (g_rp23xx_sessions != NULL)
        {
          memcpy(new_sessions, g_rp23xx_sessions,
                 g_rp23xx_sesnum * sizeof(*new_sessions));
          kmm_free(g_rp23xx_sessions);
        }

      g_rp23xx_sessions = new_sessions;
      g_rp23xx_sesnum = new_count;
    }

  session = &g_rp23xx_sessions[i];
  *sid = i;

  while (cri != NULL)
    {
      data = kmm_zalloc(sizeof(*data));
      if (data == NULL)
        {
          rp23xx_freesession(i);
          return -ENOBUFS;
        }

      switch (cri->cri_alg)
        {
          case CRYPTO_SHA2_256:
            axf = &g_auth_hash_sha256_rp23xx;
            data->hw_ictx = kmm_zalloc(axf->ctxsize);
            if (data->hw_ictx == NULL)
              {
                kmm_free(data);
                rp23xx_freesession(i);
                return -ENOBUFS;
              }

            axf->init(data->hw_ictx);
            data->hw_axf = axf;
            data->alg = cri->cri_alg;
            SLIST_INSERT_HEAD(session, data, next);
            break;

          default:
            kmm_free(data);
            rp23xx_freesession(i);
            return -EINVAL;
        }

      cri = cri->cri_next;
    }

  return OK;
}

static int rp23xx_freesession(uint64_t tid)
{
  uint32_t sid = tid & 0xffffffff;
  FAR struct rp23xx_crypto_data *data;
  FAR struct rp23xx_crypto_data *next;

  if (sid >= g_rp23xx_sesnum)
    {
      return -EINVAL;
    }

  data = SLIST_FIRST(&g_rp23xx_sessions[sid]);
  while (data != NULL)
    {
      next = SLIST_NEXT(data, next);

      if (data->hw_ictx != NULL)
        {
          nxmutex_lock(&g_rp23xx_sha_lock);
          if (g_rp23xx_active_ctx == data->hw_ictx)
            {
              g_rp23xx_active_ctx = NULL;
            }

          nxmutex_unlock(&g_rp23xx_sha_lock);
          kmm_free(data->hw_ictx);
        }

      kmm_free(data);
      data = next;
    }

  SLIST_INIT(&g_rp23xx_sessions[sid]);
  return OK;
}

static int rp23xx_process(FAR struct cryptop *crp)
{
  uint32_t lid;
  FAR struct cryptodesc *crd;
  FAR struct rp23xx_crypto_list *session;
  FAR struct rp23xx_crypto_data *data;

  lid = crp->crp_sid & 0xffffffff;
  if (lid >= g_rp23xx_sesnum)
    {
      return -EINVAL;
    }

  session = &g_rp23xx_sessions[lid];

  for (crd = crp->crp_desc; crd != NULL; crd = crd->crd_next)
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
          case CRYPTO_SHA2_256:
            crp->crp_etype = rp23xx_hash(crp, crd, data, crp->crp_buf);
            if (crp->crp_etype != 0)
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
 * Public Functions
 ****************************************************************************/

void hwcr_init(void)
{
  int hwcr_id;
  int algs[CRYPTO_ALGORITHM_MAX + 1];

  hwcr_id = crypto_get_driverid(0);
  DEBUGASSERT(hwcr_id >= 0);

  memset(algs, 0, sizeof(algs));
  algs[CRYPTO_SHA2_256] = CRYPTO_ALG_FLAG_SUPPORTED;

  crypto_register(hwcr_id, algs, rp23xx_newsession,
                  rp23xx_freesession, rp23xx_process);
}

#endif /* CONFIG_CRYPTO_CRYPTODEV_HARDWARE */
