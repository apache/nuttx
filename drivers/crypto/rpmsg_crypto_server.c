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

  /* Operation data */

  struct cryptop crp;
  struct cryptodesc crda;
  struct cryptodesc crde;


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
static int rpmsg_crypto_cancel_handler(FAR struct rpmsg_endpoint *ept,
                                       FAR void *data, size_t len,
                                       uint32_t src, FAR void *priv);

/* Async completion callbacks */

static int rpmsg_crypto_done_cb(FAR struct cryptop *crp);

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
