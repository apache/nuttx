/****************************************************************************
 * drivers/crypto/rpmsg_crypto_server.c
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

#include <debug.h>
#include <errno.h>
#include <inttypes.h>
#include <string.h>
#include <sys/endian.h>
#include <sys/param.h>

#include <crypto/cryptodev.h>
#include <crypto/virtio_crypto.h>
#include <nuttx/nuttx.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mm/mempool.h>
#include <nuttx/rpmsg/rpmsg.h>
#include <nuttx/crypto/crypto.h>

#include "rpmsg_crypto.h"

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#if CONFIG_CRYPTO_RPMSG_DATA_POOL_COUNT > 0
#  define rpmsg_crypto_data_free(p)                                        \
     do                                                                    \
       {                                                                   \
         explicit_bzero(p, sizeof(struct rpmsg_crypto_server_data_s));     \
         mempool_release(&g_rpmsg_crypto_data_pool, p);                    \
       }                                                                   \
     while (0)
#else
#  define rpmsg_crypto_data_free(p)                                        \
     do                                                                    \
       {                                                                   \
         explicit_bzero(p, sizeof(struct rpmsg_crypto_server_data_s));     \
         kmm_free(p);                                                      \
       }                                                                   \
     while (0)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rpmsg_crypto_server_data_s
{
  uint64_t tid;

  /* Operation data - symmetric or asymmetric, never both */

  union
  {
    struct
    {
      struct cryptop crp;
      struct cryptodesc crda;
      struct cryptodesc crde;
    };

    struct
    {
      struct cryptkop krp;
      unsigned char keybuf[CONFIG_CRYPTO_RPMSG_RSA_KEYBUF_SIZE];
    };
  };

  /* Async callback context */

  FAR struct rpmsg_endpoint *ept;     /* rpmsg endpoint for response */
  FAR void *rsp;                      /* tx response buffer */
  FAR void *rxbuf;                    /* held rx buffer */
  uint32_t rsplen;                    /* response length */
};

struct rpmsg_crypto_server_s
{
  struct rpmsg_endpoint ept;          /* Rpmsg end point */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Functions handle the messages from the client cpu */

static int rpmsg_crypto_newsession_handler(FAR struct rpmsg_endpoint *ept,
                                           FAR void *data, size_t len,
                                           uint32_t src, FAR void *priv);
static int rpmsg_crypto_process_handler(FAR struct rpmsg_endpoint *ept,
                                        FAR void *data, size_t len,
                                        uint32_t src, FAR void *priv);
static int rpmsg_crypto_freesession_handler(FAR struct rpmsg_endpoint *ept,
                                            FAR void *data, size_t len,
                                            uint32_t src, FAR void *priv);
static int rpmsg_crypto_kprocess_handler(FAR struct rpmsg_endpoint *ept,
                                         FAR void *data, size_t len,
                                         uint32_t src, FAR void *priv);
static int rpmsg_crypto_keyprocess_handler(FAR struct rpmsg_endpoint *ept,
                                           FAR void *data, size_t len,
                                           uint32_t src, FAR void *priv);
static int rpmsg_crypto_cancel_handler(FAR struct rpmsg_endpoint *ept,
                                       FAR void *data, size_t len,
                                       uint32_t src, FAR void *priv);

/* Async completion callbacks */

static int rpmsg_crypto_done_cb(FAR struct cryptop *crp);
static int rpmsg_crypto_kdone_cb(FAR struct cryptkop *krp);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if CONFIG_CRYPTO_RPMSG_DATA_POOL_COUNT > 0
MEMPOOL_DEFINE(g_rpmsg_crypto_data_pool,
               sizeof(struct rpmsg_crypto_server_data_s),
               CONFIG_CRYPTO_RPMSG_DATA_POOL_COUNT,
               CONFIG_CRYPTO_RPMSG_DATA_POOL_COUNT, 0);
#endif

static const rpmsg_ept_cb g_rpmsg_crypto_handler[] =
{
  [RPMSG_CRYPTO_NEWSESSION]  = rpmsg_crypto_newsession_handler,
  [RPMSG_CRYPTO_PROCESS]     = rpmsg_crypto_process_handler,
  [RPMSG_CRYPTO_FREESESSION] = rpmsg_crypto_freesession_handler,
  [RPMSG_CRYPTO_KPROCESS]    = rpmsg_crypto_kprocess_handler,
  [RPMSG_CRYPTO_KEYPROCESS]  = rpmsg_crypto_keyprocess_handler,
  [RPMSG_CRYPTO_CANCEL]      = rpmsg_crypto_cancel_handler,
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpmsg_crypto_data_alloc
 *
 * Description:
 *   Allocate and zero a per-call server data context, from the mempool
 *   when enabled, otherwise the heap.
 *
 ****************************************************************************/

static FAR struct rpmsg_crypto_server_data_s *
rpmsg_crypto_data_alloc(void)
{
  FAR struct rpmsg_crypto_server_data_s *p;

#if CONFIG_CRYPTO_RPMSG_DATA_POOL_COUNT > 0
  p = mempool_allocate(&g_rpmsg_crypto_data_pool, 0);
  if (p != NULL)
    {
      memset(p, 0, sizeof(*p));
    }
#else
  p = kmm_zalloc(sizeof(struct rpmsg_crypto_server_data_s));
#endif

  return p;
}

/****************************************************************************
 * Name: rpmsg_crypto_server_get_tx_payload_buffer
 *
 * Description:
 *   Get a tx payload buffer for the response.
 *
 ****************************************************************************/

static FAR void *
rpmsg_crypto_server_get_tx_payload_buffer(FAR struct rpmsg_endpoint *ept,
                                          FAR uint32_t *len)
{
  return rpmsg_get_tx_payload_buffer(ept, len, true);
}

/****************************************************************************
 * Name: rpmsg_crypto_done_cb
 *
 * Description:
 *   Completion callback for symmetric crypto operations. Sends the pre-
 *   filled response, releases the held rx buffer, and clears rsp as a
 *   guard against unexpected double completion from the backend.
 *
 ****************************************************************************/

static int rpmsg_crypto_done_cb(FAR struct cryptop *crp)
{
  FAR struct rpmsg_crypto_server_data_s *rcsdata = crp->crp_opaque;
  FAR struct rpmsg_crypto_process_s *rsp = rcsdata->rsp;

  rsp->header.result = crp->crp_etype;
  rpmsg_send_nocopy(rcsdata->ept, rsp, rcsdata->rsplen);
  rpmsg_release_rx_buffer(rcsdata->ept, rcsdata->rxbuf);
  rcsdata->rsp = NULL;
  return OK;
}

/****************************************************************************
 * Name: rpmsg_crypto_kdone_cb
 *
 * Description:
 *   Completion callback for asymmetric crypto operations. Sends the pre-
 *   filled response, releases the held rx buffer, and frees the per-call
 *   heap context. DEBUGASSERT guards against backend double completion
 *   which would otherwise free the context twice.
 *
 ****************************************************************************/

static int rpmsg_crypto_kdone_cb(FAR struct cryptkop *krp)
{
  FAR struct rpmsg_crypto_server_data_s *rcsdata = krp->krp_opaque;
  FAR struct rpmsg_crypto_kprocess_s *rsp = rcsdata->rsp;

  DEBUGASSERT(rsp != NULL);

  rsp->header.result = krp->krp_status;
  rpmsg_send_nocopy(rcsdata->ept, rsp, rcsdata->rsplen);
  rpmsg_release_rx_buffer(rcsdata->ept, rcsdata->rxbuf);
  rcsdata->rsp = NULL;
  rpmsg_crypto_data_free(rcsdata);
  return OK;
}

/****************************************************************************
 * Name: rpmsg_crypto_dispatch
 *
 * Description:
 *   Common dispatch helper for symmetric crypto operations: holds the rx
 *   buffer, saves async context, sets the completion callback, and
 *   dispatches to hardware via crypto_invoke.
 *
 ****************************************************************************/

static int rpmsg_crypto_dispatch(
    FAR struct rpmsg_crypto_server_data_s *rcsdata,
    FAR struct rpmsg_endpoint *ept,
    FAR struct rpmsg_crypto_process_s *msg,
    FAR struct rpmsg_crypto_process_s *rsp, uint32_t rsplen)
{
  rcsdata->ept = ept;
  rcsdata->rxbuf = msg;
  rcsdata->rsplen = rsplen;

  if (msg->data.header.flag & CRYPTO_F_CBIMM)
    {
      /* Async path: hold rx buffer, set callback, let ISR send response */

      rcsdata->rsp = rsp;
      rcsdata->crp.crp_flags |= CRYPTO_F_CBIMM;
      rcsdata->crp.crp_callback = rpmsg_crypto_done_cb;
      rcsdata->crp.crp_opaque = rcsdata;
      rpmsg_hold_rx_buffer(ept, msg);
      crypto_invoke(&rcsdata->crp);
    }
  else
    {
      /* Sync path: invoke and send response directly */

      rcsdata->crp.crp_flags &= ~CRYPTO_F_CBIMM;
      crypto_invoke(&rcsdata->crp);
      rsp->header.result = rcsdata->crp.crp_etype;
      rpmsg_send_nocopy(ept, rsp, rsplen);
    }

  return 0;
}

/****************************************************************************
 * Name: rpmsg_crypto_kdispatch
 *
 * Description:
 *   Common dispatch helper for asymmetric crypto operations.
 *
 ****************************************************************************/

static int rpmsg_crypto_kdispatch(
    FAR struct rpmsg_crypto_server_data_s *rcsdata,
    FAR struct rpmsg_endpoint *ept, FAR void *msg,
    FAR void *rsp, uint32_t rsplen)
{
  rcsdata->ept = ept;
  rcsdata->rxbuf = msg;
  rcsdata->rsplen = rsplen;

  if (rcsdata->krp.krp_flags & CRYPTO_F_CBIMM)
    {
      /* Async path: hold rx buffer, set callback, let ISR send response */

      int ret;

      rcsdata->rsp = rsp;
      rcsdata->krp.krp_callback = rpmsg_crypto_kdone_cb;
      rcsdata->krp.krp_opaque = rcsdata;
      rpmsg_hold_rx_buffer(ept, msg);
      ret = crypto_kinvoke(&rcsdata->krp);
      if (ret != 0)
        {
          ((FAR struct rpmsg_crypto_kprocess_s *)rsp)->header.result
              = ret;
          rpmsg_send_nocopy(ept, rsp, rsplen);
          rpmsg_release_rx_buffer(ept, msg);
          rpmsg_crypto_data_free(rcsdata);
        }
    }
  else
    {
      /* Sync path: invoke and send response directly */

      crypto_kinvoke(&rcsdata->krp);
      ((FAR struct rpmsg_crypto_kprocess_s *)rsp)->header.result =
          rcsdata->krp.krp_status;
      rpmsg_send_nocopy(ept, rsp, rsplen);
      rpmsg_crypto_data_free(rcsdata);
    }

  return 0;
}

/****************************************************************************
 * Name: rpmsg_crypto_newsession_handler
 ****************************************************************************/

static int rpmsg_crypto_newsession_handler(FAR struct rpmsg_endpoint *ept,
                                           FAR void *data, size_t len,
                                           uint32_t src, FAR void *priv)
{
  FAR struct rpmsg_crypto_newsession_s *msg = data;
  FAR struct rpmsg_crypto_server_data_s *rcsdata;
  struct cryptoini cria;
  struct cryptoini crie;
  int keylen = 0;
  int txform = 0;
  int thash = 0;

  rcsdata = rpmsg_crypto_data_alloc();
  if (rcsdata == NULL)
    {
      msg->header.result = -ENOMEM;
      goto out;
    }

  bzero(&crie, sizeof(crie));
  bzero(&cria, sizeof(cria));

  if (msg->ctrl.header.opcode == VIRTIO_CRYPTO_CIPHER_CREATE_SESSION)
    {
      txform = virtio_crypto_get_cipher_alg(msg->ctrl.header.algo);
      if (txform < 0)
        {
          msg->header.result = txform;
          rpmsg_crypto_data_free(rcsdata);
          goto out;
        }
      else if (txform == CRYPTO_AES_CBC)
        {
          if (msg->ctrl.op_flf.sym.op_flf.cipher.key_len == 24)
            {
              txform = CRYPTO_AES_192_CBC;
            }
          else if (msg->ctrl.op_flf.sym.op_flf.cipher.key_len == 32)
            {
              txform = CRYPTO_AES_256_CBC;
            }
        }

      keylen = msg->ctrl.op_flf.sym.op_flf.cipher.key_len;
    }
  else if (msg->ctrl.header.opcode == VIRTIO_CRYPTO_HASH_CREATE_SESSION)
    {
      thash = virtio_crypto_get_hash_alg(msg->ctrl.header.algo);
      if (thash < 0)
        {
          msg->header.result = thash;
          rpmsg_crypto_data_free(rcsdata);
          goto out;
        }
    }
  else if (msg->ctrl.header.opcode == VIRTIO_CRYPTO_MAC_CREATE_SESSION)
    {
      keylen = msg->ctrl.op_flf.mac.auth_key_len;

      thash = virtio_crypto_get_mac_alg(msg->ctrl.header.algo);
      if (thash < 0)
        {
          msg->header.result = thash;
          rpmsg_crypto_data_free(rcsdata);
          goto out;
        }
      else if (thash == CRYPTO_AES_128_CMAC)
        {
          txform = CRYPTO_AES_CMAC;
          if (keylen == 16)
            {
              thash = CRYPTO_AES_128_CMAC;
            }
          else if (keylen == 32)
            {
              thash = CRYPTO_AES_256_CMAC;
            }
        }
    }
  else if (msg->ctrl.header.opcode == VIRTIO_CRYPTO_AEAD_CREATE_SESSION)
    {
      txform = virtio_crypto_get_aead_alg(msg->ctrl.header.algo);
      if (txform < 0)
        {
          msg->header.result = txform;
          rpmsg_crypto_data_free(rcsdata);
          goto out;
        }
      else if (txform == CRYPTO_AES_GCM_16)
        {
          keylen = msg->ctrl.op_flf.aead.key_len;
          if (keylen == 16)
            {
              thash = CRYPTO_AES_128_GMAC;
            }
          else if (keylen == 24)
            {
              thash = CRYPTO_AES_192_GMAC;
            }
          else if (keylen == 32)
            {
              thash = CRYPTO_AES_256_GMAC;
            }
        }
    }

  if (txform)
    {
      crie.cri_alg = txform;
      crie.cri_klen = keylen * 8;
      crie.cri_key = msg->buf;
      rcsdata->crde.crd_alg = txform;
      if (msg->ctrl.header.flag & VIRTIO_CRYPTO_KEYID)
        {
          crie.cri_flags |= CRD_F_KEYID;
        }

      if (thash)
        {
          crie.cri_next = &cria;
        }
    }

  if (thash)
    {
      cria.cri_alg = thash;
      cria.cri_sid = -1;
      cria.cri_klen = keylen * 8;
      cria.cri_key = msg->buf;
      rcsdata->crda.crd_alg = thash;
      if (msg->ctrl.header.flag & VIRTIO_CRYPTO_KEYID)
        {
          cria.cri_flags |= CRD_F_KEYID;
        }
    }

  msg->header.result = crypto_newsession(&rcsdata->tid, txform ?
                                         &crie : &cria, 1);
  if (msg->header.result != 0)
    {
      rpmsg_crypto_data_free(rcsdata);
      goto out;
    }

  msg->header.tid = (uintptr_t)rcsdata;
out:
  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int
rpmsg_crypto_process_endec(FAR struct rpmsg_endpoint *ept,
                           FAR struct rpmsg_crypto_process_s *msg,
                           int msglen)
{
  FAR struct rpmsg_crypto_server_data_s *rcsdata =
    (FAR struct rpmsg_crypto_server_data_s *)(uintptr_t)msg->header.tid;
  FAR struct cryptodesc *crde = &rcsdata->crde;
  FAR struct cryptop *crp = &rcsdata->crp;
  FAR struct rpmsg_crypto_process_s *rsp;
  uint32_t ivlen = msg->data.op_flf.sym.op_type_flf.cipher.iv_len;
  uint32_t space;

  rsp = rpmsg_crypto_server_get_tx_payload_buffer(ept, &space);
  if (rsp == NULL)
    {
      return -ENOMEM;
    }

  if (msg->data.header.opcode == VIRTIO_CRYPTO_CIPHER_ENCRYPT)
    {
      crde->crd_flags |= CRD_F_ENCRYPT;
    }
  else
    {
      crde->crd_flags &= ~CRD_F_ENCRYPT;
    }

  crp->crp_sid = rcsdata->tid;
  crp->crp_reqid = msg->header.reqid;
  crp->crp_desc = crde;

  if (ivlen)
    {
      crp->crp_ivlen = ivlen;
      crde->crd_flags &= ~CRD_F_UPDATE;
    }
  else
    {
      crp->crp_ivlen = 0;
      crde->crd_flags |= CRD_F_UPDATE;
    }

  crp->crp_ilen = msglen - sizeof(*msg) - crp->crp_ivlen;
  crde->crd_len = crp->crp_ilen;
  crp->crp_olen = crp->crp_ilen;
  crp->crp_iv = msg->buf;
  crp->crp_buf = msg->buf + crp->crp_ivlen;
  crp->crp_dst = rsp->buf;
  rsp->header = msg->header;
  rsp->data = msg->data;
  return rpmsg_crypto_dispatch(rcsdata, ept, msg, rsp,
                               sizeof(*rsp) + crp->crp_olen);
}

static int
rpmsg_crypto_process_hash(FAR struct rpmsg_endpoint *ept,
                          FAR struct rpmsg_crypto_process_s *msg,
                          int msglen)
{
  FAR struct rpmsg_crypto_server_data_s *rcsdata =
    (FAR struct rpmsg_crypto_server_data_s *)(uintptr_t)msg->header.tid;
  FAR struct cryptodesc *crda = &rcsdata->crda;
  FAR struct cryptop *crp = &rcsdata->crp;
  FAR struct rpmsg_crypto_process_s *rsp;
  uint32_t result_len = msg->data.op_flf.hash.hash_result_len;
  uint32_t space;

  crda->crd_alg = virtio_crypto_get_hash_alg(msg->data.header.algo);
  if (crda->crd_alg < 0)
    {
      return -EINVAL;
    }

  crp->crp_sid = rcsdata->tid;
  crp->crp_reqid = msg->header.reqid;
  crp->crp_desc = crda;

  if (result_len)
    {
      /* output result */

      rsp = rpmsg_crypto_server_get_tx_payload_buffer(ept, &space);
      if (rsp == NULL)
        {
          return -ENOMEM;
        }

      crda->crd_flags = 0;
      rsp->header = msg->header;
      rsp->data = msg->data;
      crp->crp_olen = result_len;
      crp->crp_mac = rsp->buf;
      return rpmsg_crypto_dispatch(rcsdata, ept, msg, rsp,
                                   sizeof(*rsp) + crp->crp_olen);
    }

  crda->crd_flags |= CRD_F_UPDATE;
  crp->crp_ilen = msglen - sizeof(*msg);
  crda->crd_len = crp->crp_ilen;
  crp->crp_callback = NULL;
  crp->crp_buf = msg->buf;
  return crypto_invoke(crp);
}

static int
rpmsg_crypto_process_mac(FAR struct rpmsg_endpoint *ept,
                         FAR struct rpmsg_crypto_process_s *msg,
                         int msglen)
{
  FAR struct rpmsg_crypto_server_data_s *rcsdata =
    (FAR struct rpmsg_crypto_server_data_s *)(uintptr_t)msg->header.tid;
  FAR struct cryptodesc *crda = &rcsdata->crda;
  FAR struct cryptodesc *crde = &rcsdata->crde;
  FAR struct cryptop *crp = &rcsdata->crp;
  FAR struct rpmsg_crypto_process_s *rsp;
  uint32_t result_len = msg->data.op_flf.mac.hdr.hash_result_len;
  uint32_t space;

  if (crda->crd_alg == CRYPTO_AES_128_CMAC ||
      crda->crd_alg == CRYPTO_AES_256_CMAC)
    {
      crde->crd_alg = CRYPTO_AES_CMAC;
      crda->crd_next = crde;
    }

  crp->crp_sid = rcsdata->tid;
  crp->crp_reqid = msg->header.reqid;
  crp->crp_desc = crda;

  /* PBKDF2 is carried over the MAC service as a plain HMAC session plus a
   * non-zero iteration count; the backend runs the PBKDF2 loop when
   * crp_iter > 0 and a plain HMAC otherwise. The algorithm stays HMAC so
   * that it matches the session created at newsession time.
   */

  crp->crp_iter = msg->data.op_flf.mac.hdr.iterations;

  if (result_len)
    {
      /* output result */

      rsp = rpmsg_crypto_server_get_tx_payload_buffer(ept, &space);
      if (rsp == NULL)
        {
          return -ENOMEM;
        }

      crda->crd_flags = 0;
      rsp->header = msg->header;
      rsp->data = msg->data;
      crp->crp_olen = result_len;
      crp->crp_mac = rsp->buf;

      if (crp->crp_iter > 0)
        {
          /* PBKDF2 is a single-shot operation: the salt (input) and the
           * derived-key output are carried in the same request, unlike
           * plain HMAC which streams data over separate update/final
           * messages. Feed the salt from this message rather than
           * discarding it, so the backend derivation receives it.
           */

          crp->crp_ilen = msglen - sizeof(*msg);
          crda->crd_len = crp->crp_ilen;
          crp->crp_buf = msg->buf;
        }
      else
        {
          crp->crp_buf = NULL;
          crp->crp_ilen = 0;
        }

      return rpmsg_crypto_dispatch(rcsdata, ept, msg, rsp,
                                   sizeof(*rsp) + crp->crp_olen);
    }

  crda->crd_flags |= CRD_F_UPDATE;
  crp->crp_ilen = msglen - sizeof(*msg);
  crda->crd_len = crp->crp_ilen;
  crde->crd_len = crp->crp_ilen;
  crp->crp_callback = NULL;
  crp->crp_buf = msg->buf;
  return crypto_invoke(crp);
}

static int rpmsg_crypto_process_aead_update_aad(
                FAR struct rpmsg_endpoint *ept,
                FAR struct rpmsg_crypto_server_data_s *rcsdata,
                FAR struct rpmsg_crypto_process_s *msg)
{
  FAR struct cryptodesc *crda = &rcsdata->crda;
  FAR struct cryptodesc *crde = &rcsdata->crde;
  FAR struct cryptop *crp = &rcsdata->crp;
  FAR char *buf = msg->buf;
  uint32_t ivlen = msg->data.op_flf.aead.iv_len;
  uint32_t aadlen = msg->data.op_flf.aead.aad_len;

  /* Set encrypt/decrypt flag and stage flag */

  if (msg->data.header.opcode == VIRTIO_CRYPTO_AEAD_ENCRYPT)
    {
      crda->crd_flags = CRD_F_ENCRYPT;
      crde->crd_flags = CRD_F_ENCRYPT;
    }
  else
    {
      crda->crd_flags = 0;
      crde->crd_flags = 0;
    }

  /* UPDATE_AAD - process IV + AAD only */

  crp->crp_iv = NULL;
  crp->crp_ivlen = ivlen;
  if (ivlen > 0)
    {
      crp->crp_iv = buf;
      buf += ivlen;
    }

  crp->crp_aadlen = aadlen;
  crp->crp_aad = buf;
  crp->crp_ilen = 0;
  crp->crp_buf = NULL;
  crp->crp_dst = NULL;
  crp->crp_mac = NULL;
  crda->crd_len = 0;
  crda->crd_flags |= CRD_F_UPDATE_AAD;
  msg->header.result = crypto_invoke(crp);
  if (msg->header.result == 0)
    {
      msg->header.result = crp->crp_etype;
    }

  return 0;
}

static int rpmsg_crypto_process_aead_update(
                FAR struct rpmsg_endpoint *ept,
                FAR struct rpmsg_crypto_server_data_s *rcsdata,
                FAR struct rpmsg_crypto_process_s *msg)
{
  FAR struct cryptodesc *crda = &rcsdata->crda;
  FAR struct cryptodesc *crde = &rcsdata->crde;
  FAR struct cryptop *crp = &rcsdata->crp;
  FAR struct rpmsg_crypto_process_s *rsp;
  uint32_t datalen = msg->data.op_flf.aead.src_data_len;
  uint32_t space;
  FAR char *buf = msg->buf;

  /* Get tx payload buffer for response */

  rsp = rpmsg_crypto_server_get_tx_payload_buffer(ept, &space);
  if (rsp == NULL)
    {
      return -ENOMEM;
    }

  /* Set encrypt/decrypt flag and stage flag */

  if (msg->data.header.opcode == VIRTIO_CRYPTO_AEAD_ENCRYPT)
    {
      crda->crd_flags = CRD_F_ENCRYPT;
      crde->crd_flags = CRD_F_ENCRYPT;
    }
  else
    {
      crda->crd_flags = 0;
      crde->crd_flags = 0;
    }

  /* UPDATE - process data only */

  crp->crp_iv = NULL;
  crp->crp_aadlen = 0;
  crp->crp_aad = NULL;
  crp->crp_ilen = msg->data.op_flf.aead.dst_data_len;
  crda->crd_len = datalen;
  crde->crd_len = datalen;
  crp->crp_buf = buf;
  crp->crp_olen = msg->data.op_flf.aead.dst_data_len;
  crp->crp_dst = rsp->buf;
  crp->crp_mac = NULL;
  crda->crd_flags |= CRD_F_UPDATE;
  crde->crd_flags |= CRD_F_UPDATE;

  rsp->header = msg->header;
  rsp->data = msg->data;
  return rpmsg_crypto_dispatch(rcsdata, ept, msg, rsp,
                               sizeof(*rsp) + datalen);
}

static int
rpmsg_crypto_process_aead_final(
                FAR struct rpmsg_endpoint *ept,
                FAR struct rpmsg_crypto_server_data_s *rcsdata,
                FAR struct rpmsg_crypto_process_s *msg)
{
  FAR struct cryptodesc *crda =
        (FAR struct cryptodesc *)rcsdata->crp.crp_desc;
  FAR struct cryptodesc *crde = &rcsdata->crde;
  FAR struct cryptop *crp = &rcsdata->crp;
  FAR struct rpmsg_crypto_process_s *rsp;
  uint32_t taglen = msg->data.op_flf.aead.tag_len;
  uint32_t space;

  /* Set encrypt/decrypt flag */

  if (msg->data.header.opcode == VIRTIO_CRYPTO_AEAD_ENCRYPT)
    {
      crda->crd_flags = CRD_F_ENCRYPT;
      crde->crd_flags = CRD_F_ENCRYPT;
    }
  else
    {
      crda->crd_flags = 0;
      crde->crd_flags = 0;
    }

  /* FINAL - get tag result */

  rsp = rpmsg_crypto_server_get_tx_payload_buffer(ept, &space);
  if (rsp == NULL)
    {
      return -ENOMEM;
    }

  rsp->header = msg->header;
  rsp->data = msg->data;

  /* No IV and AAD in final stage */

  crp->crp_ivlen = 0;
  crp->crp_iv = NULL;
  crp->crp_aadlen = 0;
  crp->crp_aad = NULL;

  /* no data input/output, only tag */

  crp->crp_ilen = 0;
  crda->crd_len = 0;
  crde->crd_len = 0;
  crp->crp_buf = NULL;
  crp->crp_olen = 0;
  crp->crp_dst = NULL;
  crp->crp_mac = rsp->buf;
  return rpmsg_crypto_dispatch(rcsdata, ept, msg, rsp,
                               sizeof(*rsp) + taglen);
}

static int
rpmsg_crypto_process_aead(FAR struct rpmsg_endpoint *ept,
                          FAR struct rpmsg_crypto_process_s *msg,
                          int msglen)
{
  FAR struct rpmsg_crypto_server_data_s *rcsdata;
  FAR struct cryptodesc *crda;
  FAR struct cryptodesc *crde;
  uint32_t flags;

  if (msg == NULL)
    {
      rpmsgerr("rpmsg_crypto_process_aead: msg is NULL\n");
      return -EINVAL;
    }

  rcsdata =
    (FAR struct rpmsg_crypto_server_data_s *)(uintptr_t)msg->header.tid;
  if (rcsdata == NULL)
    {
      rpmsgerr("rpmsg_crypto_process_aead: invalid tid\n");
      return -EINVAL;
    }

  crda = &rcsdata->crda;
  crde = &rcsdata->crde;
  flags = msg->data.header.flag;

  rcsdata->crp.crp_sid = rcsdata->tid;
  rcsdata->crp.crp_reqid = msg->header.reqid;
  rcsdata->crp.crp_desc = crda;
  crda->crd_next = crde;

  if (flags & CRD_F_UPDATE_AAD)
    {
      return rpmsg_crypto_process_aead_update_aad(ept, rcsdata, msg);
    }
  else if (flags & CRD_F_UPDATE)
    {
      return rpmsg_crypto_process_aead_update(ept, rcsdata, msg);
    }
  else
    {
      return rpmsg_crypto_process_aead_final(ept, rcsdata, msg);
    }
}

static void rpmsg_crypto_read_binary(FAR unsigned char **p,
                                     FAR unsigned char *mpi,
                                     size_t len)
{
  uint32_t *left;
  uint32_t *right;
  uint32_t tmp;

  memset(mpi, 0, len);

  if (len != 0)
    {
      memcpy(mpi, *p, len);
    }

  for (left = (uint32_t *)mpi,
       right = (uint32_t *)(mpi + len - sizeof(*right));
       left <= right; left++, right--)
    {
      tmp    = betoh32(*left);
      *left  = betoh32(*right);
      *right = tmp;
    }

  *p += len;
}

static int rpmsg_crypto_read_mpi(FAR unsigned char **pos,
                                 FAR unsigned char *end,
                                 FAR unsigned char *buf,
                                 FAR struct crparam *mpi)
{
  size_t len;
  int ret;

  ret = asn1_get_tag(pos, end, &len, ASN1_INTEGER);
  if (ret != 0)
    {
      return ret;
    }

  /* Strip DER sign-extension: a leading 0x00 is only padding when the
   * next byte has MSB set.
   */

  if (len > 1 && **pos == 0 && ((*pos)[1] & 0x80) != 0)
    {
      (*pos)++;
      len--;
    }

  rpmsg_crypto_read_binary(pos, buf, len);
  if (len != 1 || *buf != 0)
    {
      mpi->crp_p = (caddr_t)buf;
      mpi->crp_nbits = len * 8;
    }

  return ALIGN_UP(len, 4); /* round up to aligned size */
}

static int
rpmsg_crypto_rsa_parse_private_key(FAR char *msg, int msglen,
                                   FAR unsigned char *buf,
                                   FAR struct cryptkop *krp)
{
  FAR unsigned char *end = NULL;
  FAR unsigned char *pos = NULL;
  FAR struct crparam *n = NULL;
  FAR struct crparam *e = NULL;
  FAR struct crparam *d = NULL;
  FAR struct crparam *p = NULL;
  FAR struct crparam *q = NULL;
  FAR struct crparam *dp = NULL;
  FAR struct crparam *dq = NULL;
  FAR struct crparam *qp = NULL;
  size_t buflen = 0;
  size_t len;
  int ver;
  int ret;

  pos = (FAR unsigned char *)msg;
  end = pos + msglen;

  n = &krp->krp_param[1];
  e = &krp->krp_param[2];
  d = &krp->krp_param[3];
  p = &krp->krp_param[4];
  q = &krp->krp_param[5];
  dp = &krp->krp_param[6];
  dq = &krp->krp_param[7];
  qp = &krp->krp_param[8];

  /* Skip leading zeros */

  while (pos < end && *pos == 0)
    {
      pos++;
    }

  ret = asn1_get_tag(&pos, end, &len, ASN1_CONSTRUCTED | ASN1_SEQUENCE);
  if (ret != 0)
    {
      return ret;
    }

  ret = asn1_get_int(&pos, end, &ver);
  if (ret != 0)
    {
      return ret;
    }

  /* Import N */

  buflen += rpmsg_crypto_read_mpi(&pos, end, buf + buflen, n);

  /* Import E */

  buflen += rpmsg_crypto_read_mpi(&pos, end, buf + buflen, e);

  /* Import D */

  buflen += rpmsg_crypto_read_mpi(&pos, end, buf + buflen, d);

  /* Import P */

  buflen += rpmsg_crypto_read_mpi(&pos, end, buf + buflen, p);

  /* Import Q */

  buflen += rpmsg_crypto_read_mpi(&pos, end, buf + buflen, q);

  /* Import DP */

  buflen += rpmsg_crypto_read_mpi(&pos, end, buf + buflen, dp);

  /* Import DQ */

  buflen += rpmsg_crypto_read_mpi(&pos, end, buf + buflen, dq);

  /* Import QP */

  buflen += rpmsg_crypto_read_mpi(&pos, end, buf + buflen, qp);
  return buflen;
}

static int
rpmsg_crypto_rsa_parse_public_key(FAR char *msg, int msglen,
                                  FAR unsigned char *buf,
                                  FAR struct cryptkop *krp)
{
  FAR unsigned char *end = NULL;
  FAR unsigned char *pos = NULL;
  FAR struct crparam *n = NULL;
  FAR struct crparam *e = NULL;
  size_t buflen = 0;
  size_t len;
  int ret;

  pos = (FAR unsigned char *)msg;
  end = pos + msglen;

  /* Skip leading zeros */

  while (pos < end && *pos == 0)
    {
      pos++;
    }

  ret = asn1_get_tag(&pos, end, &len, ASN1_CONSTRUCTED | ASN1_SEQUENCE);
  if (ret != 0)
    {
      return ret;
    }

  n = &krp->krp_param[1];
  e = &krp->krp_param[2];

  /* Import N */

  buflen += rpmsg_crypto_read_mpi(&pos, end, buf + buflen, n);

  /* Import E */

  buflen += rpmsg_crypto_read_mpi(&pos, end, buf + buflen, e);
  return buflen;
}

static int
rpmsg_crypto_kprocess_rsa(FAR struct rpmsg_endpoint *ept,
                          FAR struct rpmsg_crypto_kprocess_s *msg,
                          size_t len, uint32_t src, FAR void *priv)
{
  FAR struct rpmsg_crypto_kprocess_s *rsp = NULL;
  FAR struct rpmsg_crypto_server_data_s *rcsdata;
  FAR struct crparam *output = NULL;
  FAR struct crparam *input = NULL;
  FAR struct cryptkop *krp;
  uint32_t data_len = 0;
  uint32_t space;
  uint32_t keylen;
  int ret = 0;

  /* Allocate kprocess context from pool for async lifetime */

  rcsdata = rpmsg_crypto_data_alloc();
  if (rcsdata == NULL)
    {
      msg->header.result = -ENOMEM;
      return rpmsg_send(ept, &msg->header, sizeof(msg->header));
    }

  rsp = rpmsg_crypto_server_get_tx_payload_buffer(ept, &space);
  if (rsp == NULL)
    {
      rpmsg_crypto_data_free(rcsdata);
      msg->header.result = -ENOMEM;
      return rpmsg_send(ept, &msg->header, sizeof(msg->header));
    }

  rsp->header = msg->header;
  rsp->ctrl = msg->ctrl;
  rsp->data = msg->data;

  keylen = msg->ctrl.op_flf.akcipher.key_len;
  if (keylen > sizeof(rcsdata->keybuf))
    {
      rpmsgerr("rsa key length %" PRIu32 " exceeds buffer size %zu\n",
               keylen, sizeof(rcsdata->keybuf));
      ret = -EOVERFLOW;
      goto err;
    }

  krp = &rcsdata->krp;
  krp->krp_padding = msg->ctrl.op_flf.akcipher.algo_flf.rsa.padding_algo;
  krp->krp_hash = msg->ctrl.op_flf.akcipher.algo_flf.rsa.hash_algo;

  /* Import key */

  if (msg->ctrl.op_flf.akcipher.key_type ==
                         VIRTIO_CRYPTO_AKCIPHER_KEY_TYPE_PRIVATE)
    {
      ret = rpmsg_crypto_rsa_parse_private_key(msg->buf, keylen,
                                               rcsdata->keybuf, krp);
      krp->krp_keytype = CRYPTO_KEY_TYPE_PRIVATE;
    }
  else if (msg->ctrl.op_flf.akcipher.key_type ==
                         VIRTIO_CRYPTO_AKCIPHER_KEY_TYPE_PUBLIC)
    {
      ret = rpmsg_crypto_rsa_parse_public_key(msg->buf, keylen,
                                              rcsdata->keybuf, krp);
      krp->krp_keytype = CRYPTO_KEY_TYPE_PUBLIC;
    }

  if (ret < 0)
    {
      rpmsgerr("rpmsg crypto rsa parse key failed\n");
      goto err;
    }

  input = &krp->krp_param[0];
  input->crp_p = msg->buf + keylen;
  input->crp_nbits = msg->data.op_flf.akcipher.src_data_len * 8;

  /* padding processing data */

  krp->krp_flags = msg->data.header.flag;
  krp->krp_reqid = msg->header.reqid;
  switch (msg->data.header.opcode)
    {
      case VIRTIO_CRYPTO_AKCIPHER_DECRYPT:
        if (krp->krp_keytype == CRYPTO_KEY_TYPE_PRIVATE)
          {
            krp->krp_op = CRK_MOD_EXP_CRT;
            krp->krp_iparams = 9;
          }
        else
          {
            krp->krp_op = CRK_MOD_EXP;
            krp->krp_iparams = 3;
          }

        krp->krp_oparams = 1;
        krp->krp_optype = CRYPTO_OP_DECRYPT;
        output = &krp->krp_param[krp->krp_iparams +
                                 krp->krp_oparams - 1];
        output->crp_p = rsp->buf;
        output->crp_nbits = msg->data.op_flf.akcipher.dst_data_len * 8;
        data_len += msg->data.op_flf.akcipher.dst_data_len;
        break;
      case VIRTIO_CRYPTO_AKCIPHER_ENCRYPT:
        krp->krp_op = CRK_MOD_EXP;
        krp->krp_iparams = 3;
        krp->krp_oparams = 1;
        krp->krp_optype = CRYPTO_OP_ENCRYPT;
        output = &krp->krp_param[krp->krp_iparams +
                                 krp->krp_oparams - 1];
        output->crp_p = rsp->buf;
        output->crp_nbits = msg->data.op_flf.akcipher.dst_data_len * 8;
        data_len += msg->data.op_flf.akcipher.dst_data_len;
        break;
      case VIRTIO_CRYPTO_AKCIPHER_SIGN:
        if (krp->krp_padding == CRYPTO_RSA_PKCS1_PADDING)
          {
            krp->krp_op = CRK_RSA_PKCS15_SIGN;
          }
        else if (krp->krp_padding == CRYPTO_RSA_OAEP_PADDING)
          {
            krp->krp_op = CRK_RSA_PSS_SIGN;
          }
        else
          {
            ret = -EINVAL;
            goto err;
          }

        krp->krp_iparams = 9;
        krp->krp_oparams = 1;
        krp->krp_optype = CRYPTO_OP_SIGN;
        output = &krp->krp_param[krp->krp_iparams +
                                 krp->krp_oparams - 1];
        output->crp_p = rsp->buf;
        output->crp_nbits = msg->data.op_flf.akcipher.dst_data_len * 8;
        data_len += msg->data.op_flf.akcipher.dst_data_len;
        break;
      case VIRTIO_CRYPTO_AKCIPHER_VERIFY:
        if (krp->krp_padding == CRYPTO_RSA_PKCS1_PADDING)
          {
            krp->krp_op = CRK_RSA_PKCS15_VERIFY;
            krp->krp_iparams = 5;
            krp->krp_oparams = 0;
            output = &krp->krp_param[krp->krp_iparams +
                                     krp->krp_oparams - 1];
          }
        else if (krp->krp_padding == CRYPTO_RSA_OAEP_PADDING)
          {
            krp->krp_op = CRK_RSA_PSS_VERIFY;
            krp->krp_iparams = 4;
            krp->krp_oparams = 0;
            output = &krp->krp_param[krp->krp_iparams +
                                     krp->krp_oparams - 1];
          }
        else
          {
            ret = -EINVAL;
            goto err;
          }

        krp->krp_optype = CRYPTO_OP_VERIFY;
        output->crp_p = msg->buf + keylen + input->crp_nbits / 8;
        output->crp_nbits = msg->data.op_flf.akcipher.dst_data_len * 8;
        break;
      default:
        ret = -EINVAL;
        goto err;
    }

  return rpmsg_crypto_kdispatch(rcsdata, ept, msg, rsp,
                                sizeof(*rsp) + data_len);

err:
  rsp->header = msg->header;
  rsp->header.result = ret;
  rpmsg_crypto_data_free(rcsdata);
  return rpmsg_send_nocopy(ept, rsp, sizeof(*rsp));
}

static int
rpmsg_crypto_kprocess_ecdsa(FAR struct rpmsg_endpoint *ept,
                            FAR struct rpmsg_crypto_kprocess_s *msg,
                            size_t len, uint32_t src, FAR void *priv)
{
  FAR struct rpmsg_crypto_server_data_s *rcsdata;
  FAR struct rpmsg_crypto_kprocess_s *rsp;
  FAR struct crparam *dst;
  FAR struct crparam *d;
  FAR struct crparam *x;
  FAR struct crparam *y;
  FAR struct crparam *r;
  FAR struct crparam *s;
  FAR struct cryptkop *krp;
  uint32_t data_len = 0;
  uint32_t siglen;
  uint32_t keylen;
  uint32_t space;
  int ret;

  /* Allocate kprocess context from pool for async lifetime */

  rcsdata = rpmsg_crypto_data_alloc();
  if (rcsdata == NULL)
    {
      msg->header.result = -ENOMEM;
      return rpmsg_send(ept, &msg->header, sizeof(msg->header));
    }

  rsp = rpmsg_crypto_server_get_tx_payload_buffer(ept, &space);
  if (rsp == NULL)
    {
      rpmsg_crypto_data_free(rcsdata);
      msg->header.result = -ENOMEM;
      return rpmsg_send(ept, &msg->header, sizeof(msg->header));
    }

  rsp->header = msg->header;
  rsp->ctrl = msg->ctrl;
  rsp->data = msg->data;

  krp = &rcsdata->krp;
  krp->krp_flags = msg->data.header.flag;
  krp->krp_reqid = msg->header.reqid;
  switch (msg->data.header.opcode)
    {
      case VIRTIO_CRYPTO_AKCIPHER_SIGN:
        d = &krp->krp_param[0];
        r = &krp->krp_param[2];
        s = &krp->krp_param[3];
        dst = &krp->krp_param[1];
        krp->krp_op = CRK_ECDSA_SECP256R1_SIGN;
        krp->krp_optype = CRYPTO_OP_SIGN;
        krp->krp_keytype = CRYPTO_KEY_TYPE_PRIVATE;
        krp->krp_iparams = 2;
        krp->krp_oparams = 2;

        /* import private key */

        keylen = msg->ctrl.op_flf.akcipher.key_len;
        d->crp_p = (caddr_t)msg->buf;
        d->crp_nbits = keylen * 8;

        /* import source data */

        dst->crp_p = (caddr_t)msg->buf + keylen;
        dst->crp_nbits = msg->data.op_flf.akcipher.src_data_len * 8;

        siglen = msg->data.op_flf.akcipher.dst_data_len;
        r->crp_p = (caddr_t)rsp->buf;
        r->crp_nbits = siglen / 2 * 8;
        s->crp_p = (caddr_t)rsp->buf + siglen / 2;
        s->crp_nbits = siglen / 2 * 8;
        data_len += siglen;
        break;
      case VIRTIO_CRYPTO_AKCIPHER_VERIFY:
        x = &krp->krp_param[0];
        y = &krp->krp_param[1];
        r = &krp->krp_param[3];
        s = &krp->krp_param[4];
        dst = &krp->krp_param[5];
        krp->krp_op = CRK_ECDSA_SECP256R1_VERIFY;
        krp->krp_optype = CRYPTO_OP_VERIFY;
        krp->krp_keytype = CRYPTO_KEY_TYPE_PUBLIC;
        krp->krp_iparams = 6;
        krp->krp_oparams = 0;

        /* import public key */

        keylen = msg->ctrl.op_flf.akcipher.key_len;
        x->crp_p = (caddr_t)msg->buf;
        x->crp_nbits = keylen / 2 * 8;
        y->crp_p = (caddr_t)msg->buf + keylen / 2;
        y->crp_nbits = keylen / 2 * 8;

        /* import signature */

        siglen = msg->data.op_flf.akcipher.src_data_len;
        r->crp_p = (caddr_t)msg->buf + keylen;
        r->crp_nbits = siglen / 2 * 8;
        s->crp_p = (caddr_t)msg->buf + keylen + siglen / 2;
        s->crp_nbits = siglen / 2 * 8;

        /* import source data */

        dst->crp_p = (caddr_t)msg->buf + keylen + siglen;
        dst->crp_nbits = msg->data.op_flf.akcipher.dst_data_len * 8;
        break;
      case VIRTIO_CRYPTO_AKCIPHER_GENKEY:
        d = &krp->krp_param[0];
        x = &krp->krp_param[1];
        y = &krp->krp_param[2];

        krp->krp_op = CRK_ECDSA_SECP256R1_GENKEY;
        keylen = msg->data.op_flf.akcipher.src_data_len;
        d->crp_p = rsp->buf;
        d->crp_nbits = keylen * 8;
        data_len += keylen;

        keylen = msg->data.op_flf.akcipher.dst_data_len;
        x->crp_p = rsp->buf + data_len;
        x->crp_nbits = keylen / 2 * 8;
        data_len += keylen / 2;
        y->crp_p = rsp->buf + data_len;
        y->crp_nbits = keylen / 2 * 8;
        data_len += keylen / 2;
        break;
      default:
        ret = -EINVAL;
        goto err;
    }

  return rpmsg_crypto_kdispatch(rcsdata, ept, msg, rsp,
                                sizeof(*rsp) + data_len);

err:
  rsp->header.result = ret;
  rpmsg_crypto_data_free(rcsdata);
  return rpmsg_send_nocopy(ept, rsp, sizeof(*rsp));
}

/****************************************************************************
 * Name: rpmsg_crypto_process_handler
 ****************************************************************************/

static int rpmsg_crypto_process_handler(FAR struct rpmsg_endpoint *ept,
                                        FAR void *data, size_t len,
                                        uint32_t src, FAR void *priv)
{
  FAR struct rpmsg_crypto_process_s *msg = data;
  int ret = -EINVAL;

  if (msg == NULL || len < sizeof(struct rpmsg_crypto_process_s))
    {
      return -EINVAL;
    }

  if (msg->data.header.opcode == VIRTIO_CRYPTO_CIPHER_ENCRYPT ||
      msg->data.header.opcode == VIRTIO_CRYPTO_CIPHER_DECRYPT)
    {
      ret = rpmsg_crypto_process_endec(ept, msg, len);
    }
  else if (msg->data.header.opcode == VIRTIO_CRYPTO_HASH)
    {
      ret = rpmsg_crypto_process_hash(ept, msg, len);
    }
  else if (msg->data.header.opcode == VIRTIO_CRYPTO_MAC)
    {
      ret = rpmsg_crypto_process_mac(ept, msg, len);
    }
  else if (msg->data.header.opcode == VIRTIO_CRYPTO_AEAD_ENCRYPT ||
           msg->data.header.opcode == VIRTIO_CRYPTO_AEAD_DECRYPT)
    {
      ret = rpmsg_crypto_process_aead(ept, msg, len);
    }

  return ret;
}

/****************************************************************************
 * Name: rpmsg_crypto_freesession_handler
 ****************************************************************************/

static int rpmsg_crypto_freesession_handler(FAR struct rpmsg_endpoint *ept,
                                            FAR void *data, size_t len,
                                            uint32_t src, FAR void *priv)
{
  FAR struct rpmsg_crypto_freesession_s *msg = data;
  FAR struct rpmsg_crypto_server_data_s *rcsdata =
    (FAR struct rpmsg_crypto_server_data_s *)(uintptr_t)msg->header.tid;

  msg->header.result = crypto_freesession(rcsdata->tid);
  rpmsg_crypto_data_free(rcsdata);
  return rpmsg_send(ept, msg, sizeof(*msg));
}

static int
rpmsg_crypto_kprocess_ecdh(FAR struct rpmsg_endpoint *ept,
                           FAR struct rpmsg_crypto_kprocess_s *msg,
                           size_t len, uint32_t src, FAR void *priv)
{
  FAR struct rpmsg_crypto_server_data_s *rcsdata;
  FAR struct rpmsg_crypto_kprocess_s *rsp;
  FAR struct crparam *x;
  FAR struct crparam *y;
  FAR struct crparam *d;
  FAR struct cryptkop *krp;
  uint32_t data_len = 0;
  uint32_t peerlen;
  uint32_t keylen;
  uint32_t space;
  int ret;

  rcsdata = rpmsg_crypto_data_alloc();
  if (rcsdata == NULL)
    {
      msg->header.result = -ENOMEM;
      return rpmsg_send(ept, &msg->header, sizeof(msg->header));
    }

  rsp = rpmsg_crypto_server_get_tx_payload_buffer(ept, &space);
  if (rsp == NULL)
    {
      rpmsg_crypto_data_free(rcsdata);
      msg->header.result = -ENOMEM;
      return rpmsg_send(ept, &msg->header, sizeof(msg->header));
    }

  rsp->header = msg->header;
  rsp->ctrl = msg->ctrl;
  rsp->data = msg->data;

  krp = &rcsdata->krp;
  krp->krp_flags = msg->data.header.flag;
  krp->krp_reqid = msg->header.reqid;
  switch (msg->data.header.opcode)
    {
      case VIRTIO_CRYPTO_AKCIPHER_GENKEY:

        /* out param[0]=d, param[1]=x, param[2]=y into rsp->buf */

        d = &krp->krp_param[0];
        x = &krp->krp_param[1];
        y = &krp->krp_param[2];

        krp->krp_op = CRK_ECDH_GEN_PUBLIC;
        krp->krp_iparams = 0;
        krp->krp_oparams = 4;

        /* Private key length from src, public key (x || y) from dst. */

        keylen = msg->data.op_flf.akcipher.src_data_len;
        d->crp_p = rsp->buf;
        d->crp_nbits = keylen * 8;
        data_len += keylen;

        keylen = msg->data.op_flf.akcipher.dst_data_len;
        x->crp_p = rsp->buf + data_len;
        x->crp_nbits = keylen / 2 * 8;
        data_len += keylen / 2;
        y->crp_p = rsp->buf + data_len;
        y->crp_nbits = keylen / 2 * 8;
        data_len += keylen / 2;
        break;
      case VIRTIO_CRYPTO_AKCIPHER_DERIVE:

        /* in param[0]=priv, param[1]=peer.x, param[2]=peer.y;
         * out param[3]=shared secret into rsp->buf.
         */

        d = &krp->krp_param[0];
        x = &krp->krp_param[1];
        y = &krp->krp_param[2];

        krp->krp_op = CRK_ECDH_COMPUTE_SHARED;
        krp->krp_iparams = 3;
        krp->krp_oparams = 1;

        /* Private key (d) length from ctrl.key_len; peer public key
         * (x || y) length from src_data_len (split in half). Buffer
         * layout is d || x || y. Mirrors the ECDSA verify handling so
         * no equal-segment assumption is needed.
         */

        keylen = msg->ctrl.op_flf.akcipher.key_len;
        peerlen = msg->data.op_flf.akcipher.src_data_len;

        /* Validate lengths step by step to avoid both unsigned underflow
         * (len < sizeof(*msg) would wrap the subtraction to a huge value)
         * and overflow of keylen + peerlen. Each comparison keeps its
         * left operand within the remaining payload space.
         */

        if (len < sizeof(*msg) ||
            keylen > len - sizeof(*msg) ||
            peerlen > len - sizeof(*msg) - keylen)
          {
            ret = -EMSGSIZE;
            goto err;
          }

        d->crp_p = (caddr_t)msg->buf;
        d->crp_nbits = keylen * 8;
        x->crp_p = (caddr_t)msg->buf + keylen;
        x->crp_nbits = peerlen / 2 * 8;
        y->crp_p = (caddr_t)msg->buf + keylen + peerlen / 2;
        y->crp_nbits = peerlen / 2 * 8;

        krp->krp_param[3].crp_p = rsp->buf;
        krp->krp_param[3].crp_nbits =
                        msg->data.op_flf.akcipher.dst_data_len * 8;
        data_len += msg->data.op_flf.akcipher.dst_data_len;
        break;
      default:
        ret = -EINVAL;
        goto err;
    }

  return rpmsg_crypto_kdispatch(rcsdata, ept, msg, rsp,
                                sizeof(*rsp) + data_len);

err:
  rsp->header = msg->header;
  rsp->header.result = ret;
  rpmsg_crypto_data_free(rcsdata);
  return rpmsg_send_nocopy(ept, rsp, sizeof(*rsp));
}

/****************************************************************************
 * Name: rpmsg_crypto_kprocess_handler
 ****************************************************************************/

static int rpmsg_crypto_kprocess_handler(FAR struct rpmsg_endpoint *ept,
                                         FAR void *data, size_t len,
                                         uint32_t src, FAR void *priv)
{
  FAR struct rpmsg_crypto_kprocess_s *msg = data;

  switch (msg->ctrl.header.algo)
    {
      case VIRTIO_CRYPTO_AKCIPHER_RSA:
        return rpmsg_crypto_kprocess_rsa(ept, msg, len, src, priv);
      case VIRTIO_CRYPTO_AKCIPHER_ECDSA:
        return rpmsg_crypto_kprocess_ecdsa(ept, msg, len, src, priv);
      case VIRTIO_CRYPTO_AKCIPHER_ECDH:
        return rpmsg_crypto_kprocess_ecdh(ept, msg, len, src, priv);
      default:
        msg->header.result = -ENOTSUP;
        return rpmsg_send(ept, &msg->header, sizeof(msg->header));
    }
}

/****************************************************************************
 * Name: rpmsg_crypto_keyprocess_handler
 ****************************************************************************/

static int rpmsg_crypto_keyprocess_handler(FAR struct rpmsg_endpoint *ept,
                                           FAR void *data, size_t len,
                                           uint32_t src, FAR void *priv)
{
  FAR struct rpmsg_crypto_keyprocess_s *msg = data;
  FAR struct rpmsg_crypto_keyprocess_s *rsp;
  struct cryptkop krp;
  uint32_t space;

  rsp = rpmsg_crypto_server_get_tx_payload_buffer(ept, &space);
  if (rsp == NULL)
    {
      msg->header.result = -ENOMEM;
      return rpmsg_send(ept, &msg->header, sizeof(msg->header));
    }

  rsp->header = msg->header;
  rsp->data = msg->data;
  memset(&krp, 0, sizeof(krp));
  krp.krp_op = msg->data.cmd;
  krp.krp_flags = msg->data.flag;
  krp.krp_reqid = msg->header.reqid;
  switch (msg->data.cmd)
    {
      case CRK_ALLOCATE_KEY:
        krp.krp_iparams = 0;
        krp.krp_oparams = 1;
        krp.krp_param[0].crp_nbits = msg->data.dst_data_len * 8;
        krp.krp_param[0].crp_p = rsp->buf;
        break;
      case CRK_VALIDATE_KEYID:
      case CRK_DELETE_KEY:
      case CRK_SAVE_KEY:
      case CRK_LOAD_KEY:
      case CRK_UNLOAD_KEY:
        krp.krp_iparams = 1;
        krp.krp_oparams = 0;
        krp.krp_param[0].crp_nbits = msg->data.name_len * 8;
        krp.krp_param[0].crp_p = msg->buf;
        break;
      case CRK_IMPORT_KEY:
      case CRK_GENERATE_AES_KEY:
      case CRK_GENERATE_SECP256R1_KEY:
        krp.krp_iparams = 2;
        krp.krp_oparams = 0;
        krp.krp_param[0].crp_nbits = msg->data.name_len * 8;
        krp.krp_param[0].crp_p = msg->buf;
        krp.krp_param[1].crp_nbits = msg->data.src_data_len * 8;
        krp.krp_param[1].crp_p = msg->buf + msg->data.name_len;
        break;
      case CRK_EXPORT_KEY:
        krp.krp_iparams = 1;
        krp.krp_oparams = 1;
        krp.krp_param[0].crp_nbits = msg->data.name_len * 8;
        krp.krp_param[0].crp_p = msg->buf;
        krp.krp_param[1].crp_nbits = msg->data.dst_data_len * 8;
        krp.krp_param[1].crp_p = rsp->buf;
        break;
    }

  rsp->header.result = crypto_kinvoke(&krp);
  if (rsp->header.result == 0)
    {
      rsp->header.result = krp.krp_status;
      if (krp.krp_op == CRK_EXPORT_KEY)
        {
          rsp->data.dst_data_len = krp.krp_param[1].crp_nbits / 8;
        }
    }

  return rpmsg_send_nocopy(ept, rsp, sizeof(*rsp) + rsp->data.dst_data_len);
}

/****************************************************************************
 * Name: rpmsg_crypto_cancel_handler
 *
 * Description:
 *   Forward a cancel to the backend so a still-queued operation can be
 *   dropped before it consumes hardware. A symmetric cancel carries the
 *   session (header.tid), so it is dispatched via crypto_invoke using the
 *   backend sid held in the session's rcsdata; an asymmetric cancel carries
 *   no session and is routed by op via crypto_kinvoke. Both are matched by
 *   reqid; backends that cannot abort ignore it. Fire-and-forget.
 *
 ****************************************************************************/

static int rpmsg_crypto_cancel_handler(FAR struct rpmsg_endpoint *ept,
                                       FAR void *data, size_t len,
                                       uint32_t src, FAR void *priv)
{
  FAR struct rpmsg_crypto_cancel_s *msg = data;
  FAR struct rpmsg_crypto_server_data_s *rcsdata =
    (FAR struct rpmsg_crypto_server_data_s *)(uintptr_t)msg->header.tid;
  struct cryptodesc crd;
  struct cryptop crp;
  struct cryptkop krp;

  if (msg->header.tid != 0)
    {
      memset(&crp, 0, sizeof(crp));
      memset(&crd, 0, sizeof(crd));
      crp.crp_sid = rcsdata->tid;
      crp.crp_reqid = msg->header.reqid;
      crp.crp_flags = CRYPTO_F_CANCEL;
      crp.crp_desc = &crd;

      crypto_invoke(&crp);
    }
  else
    {
      memset(&krp, 0, sizeof(krp));
      krp.krp_op = msg->op;
      krp.krp_flags = CRYPTO_F_CANCEL;
      krp.krp_reqid = msg->header.reqid;

      crypto_kinvoke(&krp);
    }

  return OK;
}

/****************************************************************************
 * Name: rpmsg_crypto_ept_cb
 ****************************************************************************/

static int rpmsg_crypto_ept_cb(FAR struct rpmsg_endpoint *ept,
                               FAR void *data, size_t len, uint32_t src,
                               FAR void *priv)
{
  FAR struct rpmsg_crypto_header_s *header = data;
  uint32_t command = header->command;

  if (command < nitems(g_rpmsg_crypto_handler))
    {
      return g_rpmsg_crypto_handler[command](ept, data, len, src, priv);
    }

  rpmsgerr("Rpmsg Crypto Server: Failed to find command handler\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: rpmsgdev_ns_match
 ****************************************************************************/

static bool rpmsg_crypto_ns_match(FAR struct rpmsg_device *rdev,
                                  FAR void *priv, FAR const char *name,
                                  uint32_t dest)
{
  return !strncmp(name, RPMSG_CRYPTO_NAME_PREFIX,
                        RPMSG_CRYPTO_NAME_PREFIX_LEN);
}

/****************************************************************************
 * Name: rpmsg_crypto_ept_release
 ****************************************************************************/

static void rpmsg_crypto_ept_release(FAR struct rpmsg_endpoint *ept)
{
  FAR struct rpmsg_crypto_server_s *server = ept->priv;

  kmm_free(server);
}

/****************************************************************************
 * Name: rpmsg_crypto_ns_bind
 ****************************************************************************/

static void rpmsg_crypto_ns_bind(FAR struct rpmsg_device *rdev,
                                 FAR void *priv, FAR const char *name,
                                 uint32_t dest)
{
  FAR struct rpmsg_crypto_server_s *server;
  int ret;

  server = kmm_zalloc(sizeof(*server));
  if (server == NULL)
    {
      return;
    }

  server->ept.priv = server;
  server->ept.release_cb = rpmsg_crypto_ept_release;
  ret = rpmsg_create_ept(&server->ept, rdev, name,
                         RPMSG_ADDR_ANY, dest,
                         rpmsg_crypto_ept_cb, rpmsg_destroy_ept);
  if (ret < 0)
    {
      kmm_free(server);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpmsg_crypto_server_init
 *
 * Description:
 *   Rpmsg-crypto server initialize function, the server cpu should call
 *   this function.
 *
 * Parameters:
 *   None
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

int rpmsg_crypto_server_init(void)
{
  return rpmsg_register_callback(NULL,
                                 NULL,
                                 NULL,
                                 rpmsg_crypto_ns_match,
                                 rpmsg_crypto_ns_bind);
}
