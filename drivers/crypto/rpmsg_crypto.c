/****************************************************************************
 * drivers/crypto/rpmsg_crypto.c
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
#include <inttypes.h>
#include <stdio.h>
#include <sys/param.h>

#include <crypto/cryptodev.h>
#include <crypto/virtio_crypto.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/nuttx.h>
#include <nuttx/rpmsg/rpmsg.h>
#include <nuttx/crypto/crypto.h>

#include "rpmsg_crypto.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rpmsg_crypto_session_s
{
  bool      used;    /* The session is used */
  sem_t     sem;     /* Semaphore used for rpmsg */
  uint64_t  tid;     /* The session id */
  int       result;  /* The return value of the remote call */
  FAR void *data;    /* The return data buffer of the remote call */
  uint32_t  offset;  /* CBIMM: accumulated output offset */
};

struct rpmsg_crypto_s
{
  struct rpmsg_endpoint ept;          /* Rpmsg endpoint */
  FAR const char       *remotecpu;    /* The server cpu name */
  sem_t                 wait;         /* Wait sem, used for preventing any
                                       * operation until the connection
                                       * between two cpu established.
                                       */
  mutex_t               lock;         /* Protect session alloc/free */
  uint8_t               devid;        /* The device id registered */
  uint8_t               keydevid;     /* The key management id registered */
  uint32_t              sesnum;       /* The number of sessions */
  FAR struct rpmsg_crypto_session_s
                       *sessions;     /* The session data slots */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

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

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Rpmsg crypto response handler table */

static const rpmsg_ept_cb g_rpmsg_crypto_handler[] =
{
  [RPMSG_CRYPTO_NEWSESSION]  = rpmsg_crypto_newsession_handler,
  [RPMSG_CRYPTO_PROCESS]     = rpmsg_crypto_process_handler,
  [RPMSG_CRYPTO_FREESESSION] = rpmsg_crypto_freesession_handler,
  [RPMSG_CRYPTO_KPROCESS]    = rpmsg_crypto_kprocess_handler,
  [RPMSG_CRYPTO_KEYPROCESS]  = rpmsg_crypto_keyprocess_handler,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpmsg_crypto_get_tx_payload_buffer
 *
 * Description:
 *   Get the rpmsg crypto tx payload, the buffer is from the rpmsg
 *   share memory that can be accessed by local and remote cpu.
 *
 * Parameters:
 *   priv  - The rpmsg-crypto handle
 *   len   - The got memory size
 *
 * Returned Values:
 *   NULL     - failure
 *   not NULL - success
 *
 ****************************************************************************/

static FAR void *
rpmsg_crypto_get_tx_payload_buffer(FAR struct rpmsg_crypto_s *priv,
                                   FAR uint32_t *len)
{
  int sval = 0;

  nxsem_get_value(&priv->wait, &sval);
  if (sval <= 0)
    {
#if CONFIG_CRYPTO_RPMSG_CONNECT_TIMEOUT_MS < 0
      if (rpmsg_wait(&priv->ept, &priv->wait) < 0)
#else
      if (rpmsg_tickwait(&priv->ept, &priv->wait,
                         MSEC2TICK(
                           CONFIG_CRYPTO_RPMSG_CONNECT_TIMEOUT_MS)) < 0)
#endif
        {
          return NULL;
        }

      rpmsg_post(&priv->ept, &priv->wait);
    }

  return rpmsg_get_tx_payload_buffer(&priv->ept, len, true);
}

/****************************************************************************
 * Name: rpmsg_crypto_send_recv
 *
 * Description:
 *   Send and receive the rpmsg crypto data reserved by
 *   rpmsg_get_tx_payload_buffer().
 *
 * Parameters:
 *   priv    - rpmsg crypto handle
 *   command - the command, RPMSG_CRYPTO_NEWSESSION, RPMSG_CRYPTO_PROCESS,
 *                          RPMSG_CRYPTO_FREESESSION
 *   msg     - the message header
 *   len     - length of the payload
 *   data    - the data
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int rpmsg_crypto_send_recv(FAR struct rpmsg_crypto_s *priv,
                                  FAR struct rpmsg_crypto_session_s *session,
                                  FAR struct rpmsg_crypto_header_s *msg,
                                  uint32_t command, uint32_t reqid,
                                  int len, bool waiting)
{
  int ret;

  msg->command = command;
  msg->result  = -ENXIO;
  msg->cookie  = (uintptr_t)session;
  msg->reqid   = reqid;

  ret = rpmsg_send_nocopy(&priv->ept, msg, len);
  if (ret < 0)
    {
      rpmsg_release_tx_buffer(&priv->ept, msg);
      return ret;
    }

  if (waiting && rpmsg_wait(&priv->ept, &session->sem) >= 0)
    {
      ret = session->result;
    }

  return ret;
}

/****************************************************************************
 * Name: rpmsg_crypto_cancel
 *
 * Description:
 *   Send a fire-and-forget cancel for an in-flight op. It needs no session
 *   and expects no response, so it does not take a session slot or wait: it
 *   just grabs a tx buffer and sends. A symmetric cancel carries the session
 *   (tid) for sid-based routing on the server; an asymmetric cancel carries
 *   the op. Matched by the reqid in the header; backends that cannot abort
 *   ignore it.
 *
 ****************************************************************************/

static int rpmsg_crypto_cancel(FAR struct rpmsg_crypto_s *rdev,
                               uint32_t reqid, uint64_t tid, uint32_t op)
{
  FAR struct rpmsg_crypto_cancel_s *msg;
  uint32_t space;
  int ret;

  if (rdev == NULL)
    {
      return -EINVAL;
    }

  msg = rpmsg_crypto_get_tx_payload_buffer(rdev, &space);
  if (msg == NULL)
    {
      return -ENOMEM;
    }

  memset(msg, 0, sizeof(*msg));
  msg->header.command = RPMSG_CRYPTO_CANCEL;
  msg->header.reqid   = reqid;
  msg->header.tid     = tid;
  msg->op             = op;

  ret = rpmsg_send_nocopy(&rdev->ept, msg, sizeof(*msg));
  if (ret < 0)
    {
      rpmsg_release_tx_buffer(&rdev->ept, msg);
    }

  return ret;
}

static int rpmsg_crypto_get_session(FAR struct rpmsg_crypto_s *rdev)
{
  uint32_t i;

  nxmutex_lock(&rdev->lock);

  for (i = 0; i < rdev->sesnum; i++)
    {
      if (!rdev->sessions[i].used)
        {
          break;
        }
    }

  if (i == rdev->sesnum)
    {
      nxmutex_unlock(&rdev->lock);
      return -ENOMEM;
    }

  rdev->sessions[i].used = true;
  rdev->sessions[i].data = NULL;
  nxsem_init(&rdev->sessions[i].sem, 0, 0);
  nxmutex_unlock(&rdev->lock);
  return i;
}

static int rpmsg_crypto_put_session(FAR struct rpmsg_crypto_s *rdev,
                                    uint32_t sid)
{
  if (sid >= rdev->sesnum)
    {
      return -EINVAL;
    }

  nxmutex_lock(&rdev->lock);
  nxsem_destroy(&rdev->sessions[sid].sem);
  rdev->sessions[sid].used = false;
  nxmutex_unlock(&rdev->lock);
  return OK;
}

static int
rpmsg_crypto_clean_session(FAR struct rpmsg_crypto_session_s *session)
{
  nxsem_destroy(&session->sem);
  session->used = false;
  return OK;
}

static int
rpmsg_crypto_fill_session_request(FAR struct cryptoini *cri,
                                  FAR struct rpmsg_crypto_newsession_s *msg)
{
  FAR struct virtio_crypto_op_ctrl_req_s *ctrl = &msg->ctrl;
  int service = virtio_crypto_get_service(cri->cri_alg);
  int algo = virtio_crypto_get_alg(cri->cri_alg);

  if (service < 0)
    {
      return service;
    }

  if (algo < 0)
    {
      return algo;
    }

  memset(ctrl, 0, sizeof(*ctrl));

  /* Pad ctrl header for creating a new session */

  ctrl->header.algo = algo;
  if (cri->cri_flags & CRD_F_KEYID)
    {
      ctrl->header.flag |= VIRTIO_CRYPTO_KEYID;
    }

  switch (service)
    {
      case VIRTIO_CRYPTO_SERVICE_CIPHER:
        ctrl->header.opcode = VIRTIO_CRYPTO_CIPHER_CREATE_SESSION;
        ctrl->op_flf.sym.op_type = VIRTIO_CRYPTO_SYM_OP_CIPHER;
        ctrl->op_flf.sym.op_flf.cipher.algo = algo;
        ctrl->op_flf.sym.op_flf.cipher.key_len = cri->cri_klen / 8;
        ctrl->op_flf.sym.op_flf.cipher.op =
          cri->cri_op == COP_ENCRYPT ?
          VIRTIO_CRYPTO_OP_ENCRYPT : VIRTIO_CRYPTO_OP_DECRYPT;
        break;
      case VIRTIO_CRYPTO_SERVICE_HASH:
        ctrl->header.opcode = VIRTIO_CRYPTO_HASH_CREATE_SESSION;
        ctrl->op_flf.hash.algo = algo;
        ctrl->op_flf.hash.hash_result_len =
          virtio_crypto_get_hash_result_len(algo);
        break;
      case VIRTIO_CRYPTO_SERVICE_MAC:
        ctrl->header.opcode = VIRTIO_CRYPTO_MAC_CREATE_SESSION;
        ctrl->op_flf.mac.algo = algo;
        ctrl->op_flf.mac.hash_result_len =
          virtio_crypto_get_hash_result_len(algo);
        ctrl->op_flf.mac.auth_key_len = cri->cri_klen / 8;
        break;
      case VIRTIO_CRYPTO_SERVICE_AEAD:
        ctrl->header.opcode = VIRTIO_CRYPTO_AEAD_CREATE_SESSION;
        ctrl->op_flf.aead.algo = algo;
        ctrl->op_flf.aead.key_len = cri->cri_klen / 8;
        ctrl->op_flf.aead.tag_len = virtio_crypto_get_tag_len(service);
        ctrl->op_flf.aead.op = cri->cri_op == COP_ENCRYPT ?
          VIRTIO_CRYPTO_OP_ENCRYPT : VIRTIO_CRYPTO_OP_DECRYPT;
        break;
      default:
        return -EINVAL;
    }

  return OK;
}

static int rpmsg_crypto_newsession(FAR uint32_t *sid,
                                   FAR struct cryptoini *cri)
{
  FAR struct rpmsg_crypto_s *rdev;
  FAR struct rpmsg_crypto_newsession_s *msg;
  uint32_t len = sizeof(*msg);
  uint32_t space;
  int ret;

  if (sid == NULL || cri == NULL)
    {
      return -EINVAL;
    }

  rdev = (FAR struct rpmsg_crypto_s *)crypto_driver_get_priv(*sid);
  if (rdev == NULL)
    {
      rpmsgerr("Invalid context\n");
      return -EINVAL;
    }

  ret = rpmsg_crypto_get_session(rdev);
  if (ret < 0)
    {
      return ret;
    }

  *sid = ret;
  msg = rpmsg_crypto_get_tx_payload_buffer(rdev, &space);
  if (msg == NULL)
    {
      ret = -ENOMEM;
      goto fail;
    }

  /* create newsession msg */

  ret = rpmsg_crypto_fill_session_request(cri, msg);
  if (ret < 0)
    {
      rpmsgerr("Invalid parameters in newsession\n");
      rpmsg_release_tx_buffer(&rdev->ept, msg);
      goto fail;
    }

  if (cri->cri_klen > 0)
    {
      memcpy(msg->buf, cri->cri_key, cri->cri_klen / 8);
      len += cri->cri_klen / 8;
    }

  ret = rpmsg_crypto_send_recv(rdev, &rdev->sessions[*sid],
                               &msg->header, RPMSG_CRYPTO_NEWSESSION, 0,
                               len, true);
  if (ret < 0)
    {
      goto fail;
    }

  return rdev->sessions[*sid].result;

fail:
  rpmsg_crypto_put_session(rdev, *sid);
  return ret;
}

static int rpmsg_crypto_fill_data_request(
                        FAR struct cryptop *crp,
                        FAR struct rpmsg_crypto_process_s *msg)
{
  FAR struct virtio_crypto_op_data_req_s *data = &msg->data;
  FAR struct virtio_crypto_op_header_s *header = &data->header;
  FAR struct cryptodesc *crda = crp->crp_desc;
  FAR struct cryptodesc *crde = crda->crd_next;
  int service = virtio_crypto_get_service(crda->crd_alg);
  int algo = virtio_crypto_get_alg(crda->crd_alg);

  if (service < 0)
    {
      return service;
    }

  if (algo < 0)
    {
      return algo;
    }

  memset(data, 0, sizeof(struct virtio_crypto_op_data_req_s));
  header->algo = algo;
  switch (service)
    {
      case VIRTIO_CRYPTO_SERVICE_CIPHER:
        header->opcode = crda->crd_flags & CRD_F_ENCRYPT ?
          VIRTIO_CRYPTO_CIPHER_ENCRYPT : VIRTIO_CRYPTO_CIPHER_DECRYPT;
        data->op_flf.sym.op_type = VIRTIO_CRYPTO_SYM_OP_CIPHER;
        data->op_flf.sym.op_type_flf.cipher.src_data_len = crp->crp_ilen;
        data->op_flf.sym.op_type_flf.cipher.dst_data_len = crp->crp_olen;
        crp->crp_opaque = crp->crp_dst;
        break;
      case VIRTIO_CRYPTO_SERVICE_HASH:
        header->opcode = VIRTIO_CRYPTO_HASH;
        data->op_flf.hash.src_data_len = crp->crp_ilen;
        break;
      case VIRTIO_CRYPTO_SERVICE_MAC:
        header->opcode = VIRTIO_CRYPTO_MAC;
        data->op_flf.mac.hdr.src_data_len = crp->crp_ilen;
        data->op_flf.mac.hdr.iterations = crp->crp_iter;
        break;
      case VIRTIO_CRYPTO_SERVICE_AEAD:
        header->opcode = crde->crd_flags & CRD_F_ENCRYPT ?
          VIRTIO_CRYPTO_AEAD_ENCRYPT : VIRTIO_CRYPTO_AEAD_DECRYPT;
        data->op_flf.aead.iv_len = crp->crp_ivlen;
        data->op_flf.aead.tag_len = virtio_crypto_get_tag_len(service);
        break;
      default:
        return -ENOTSUP;
    }

  header->flag = crp->crp_flags;
  return OK;
}

static FAR struct rpmsg_crypto_process_s *
rpmsg_crypto_get_process_msg(FAR struct rpmsg_crypto_s * rdev,
                             FAR struct rpmsg_crypto_process_s *src,
                             FAR uint32_t *space)
{
  FAR struct rpmsg_crypto_process_s *msg;

  msg = rpmsg_crypto_get_tx_payload_buffer(rdev, space);
  if (msg == NULL)
    {
      return NULL;
    }

  if (*space <= sizeof(*msg))
    {
      rpmsg_release_tx_buffer(&rdev->ept, msg);
      return NULL;
    }

  msg->header = src->header;
  msg->data = src->data;
  return msg;
}

static int
rpmsg_crypto_process_endec(FAR struct rpmsg_crypto_s *rdev,
                           FAR struct rpmsg_crypto_process_s *info,
                           FAR struct rpmsg_crypto_session_s *session,
                           FAR struct cryptop *crp)
{
  FAR struct rpmsg_crypto_process_s *msg;
  FAR struct rpmsg_crypto_process_s *rsp;
  FAR char *buf;
  uint32_t header_size;
  uint32_t data_size;
  uint32_t offset = 0;
  uint32_t space;
  bool need_iv = !(crp->crp_desc->crd_flags & CRD_F_UPDATE);
  bool async = crp->crp_flags & CRYPTO_F_CBIMM;
  int remaining = crp->crp_ilen;
  int blksize = virtio_crypto_get_blocksize(crp->crp_desc->crd_alg);
  int ret;

  if (blksize < 0)
    {
      return blksize;
    }

  if (crp->crp_dst == NULL)
    {
      return -EINVAL;
    }

  while (remaining > 0)
    {
      msg = rpmsg_crypto_get_process_msg(rdev, info, &space);
      if (msg == NULL)
        {
          return -ENOMEM;
        }

      buf = msg->buf;
      header_size = sizeof(*msg);
      data_size = space - sizeof(*msg);
      if (remaining == crp->crp_ilen && need_iv)
        {
          if (data_size <= crp->crp_ivlen)
            {
              rpmsg_release_tx_buffer(&rdev->ept, msg);
              return -ENOMEM;
            }

          msg->data.op_flf.sym.op_type_flf.cipher.iv_len = crp->crp_ivlen;
          memcpy(buf, crp->crp_iv, crp->crp_ivlen);
          header_size += crp->crp_ivlen;
          data_size -= crp->crp_ivlen;
          buf += crp->crp_ivlen;
        }
      else
        {
          msg->data.op_flf.sym.op_type_flf.cipher.iv_len = 0;
        }

      data_size = MIN(data_size, remaining);
      if (data_size < remaining)
        {
          data_size = ALIGN_DOWN(data_size, blksize);
        }

      if (data_size == 0)
        {
          rpmsg_release_tx_buffer(&rdev->ept, msg);
          return -ENOMEM;
        }

      memcpy(buf, crp->crp_buf + offset, data_size);
      ret = rpmsg_crypto_send_recv(rdev, session, &msg->header,
                                   RPMSG_CRYPTO_PROCESS, crp->crp_reqid,
                                   header_size + data_size, !async);
      if (ret < 0)
        {
          return ret;
        }

      if (!async && session->data != NULL)
        {
          rsp = session->data;
          memcpy(crp->crp_dst + offset, rsp->buf, data_size);
          rpmsg_release_rx_buffer(&rdev->ept, rsp);
          session->data = NULL;
        }

      remaining -= data_size;
      offset += data_size;
    }

  return async ? OK : session->result;
}

static int
rpmsg_crypto_process_hash_mac(FAR struct rpmsg_crypto_s *rdev,
                              FAR struct rpmsg_crypto_process_s *info,
                              FAR struct rpmsg_crypto_session_s *session,
                              FAR struct cryptop *crp)
{
  FAR struct rpmsg_crypto_process_s *msg;
  FAR struct rpmsg_crypto_process_s *rsp;
  uint32_t flags = crp->crp_desc->crd_flags;
  uint32_t offset = 0;
  uint32_t data_size;
  uint32_t space;
  bool async = crp->crp_flags & CRYPTO_F_CBIMM;
  int service = virtio_crypto_get_service(crp->crp_desc->crd_alg);
  int remaining = crp->crp_ilen;
  int ret;

  do
    {
      msg = rpmsg_crypto_get_process_msg(rdev, info, &space);
      if (msg == NULL)
        {
          return -ENOMEM;
        }

      data_size = MIN(space - sizeof(*msg), remaining);
      if (data_size > 0)
        {
          memcpy(msg->buf, crp->crp_buf + offset, data_size);
        }

      offset += data_size;
      remaining -= data_size;
      if (remaining == 0 && !(flags & CRD_F_UPDATE))
        {
          if (service == VIRTIO_CRYPTO_SERVICE_HASH)
            {
              msg->data.op_flf.hash.hash_result_len = crp->crp_olen;
            }
          else if (service == VIRTIO_CRYPTO_SERVICE_MAC)
            {
              msg->data.op_flf.mac.hdr.hash_result_len = crp->crp_olen;
            }
        }

      ret = rpmsg_crypto_send_recv(rdev, session, &msg->header,
                                   RPMSG_CRYPTO_PROCESS, crp->crp_reqid,
                                   sizeof(*msg) + data_size,
                                   remaining == 0 &&
                                   !(flags & CRD_F_UPDATE) && !async);
      if (ret < 0)
        {
          return ret;
        }

      if (!async && session->data != NULL)
        {
          rsp = session->data;
          memcpy(crp->crp_mac, rsp->buf, crp->crp_olen);
          rpmsg_release_rx_buffer(&rdev->ept, rsp);
          session->data = NULL;
        }
    }
  while (remaining > 0);

  return async ? OK : session->result;
}

static int
rpmsg_crypto_process_aead_update_aad(
              FAR struct rpmsg_crypto_s *rdev,
              FAR struct rpmsg_crypto_process_s *info,
              FAR struct rpmsg_crypto_session_s *session,
              FAR struct cryptop *crp)
{
  FAR struct rpmsg_crypto_process_s *msg;
  uint32_t flags = crp->crp_desc->crd_flags;
  uint32_t remaining = crp->crp_aadlen;
  uint32_t ivlen = crp->crp_ivlen;
  uint32_t offset = 0;
  uint32_t datalen;
  uint32_t space;
  uint32_t len;
  FAR char *buf;
  int ret;

  do
    {
      msg = rpmsg_crypto_get_process_msg(rdev, info, &space);
      if (msg == NULL)
        {
          return -ENOMEM;
        }

      buf = msg->buf;
      len = sizeof(*msg);
      msg->data.header.flag = flags;
      msg->data.op_flf.aead.dst_data_len = crp->crp_olen;

      /* First packet includes IV (ivlen > 0), then reset to 0 */

      msg->data.op_flf.aead.iv_len = ivlen;
      if (ivlen > 0)
        {
          memcpy(buf, crp->crp_iv, ivlen);
          len += ivlen;
          buf += ivlen;
          ivlen = 0;
        }

      /* Copy AAD data */

      if (space <= len)
        {
          rpmsg_release_tx_buffer(&rdev->ept, msg);
          return -ENOMEM;
        }

      datalen = MIN(space - len, remaining);
      if (datalen > 0)
        {
          memcpy(buf, crp->crp_aad + offset, datalen);
          len += datalen;
          offset += datalen;
          remaining -= datalen;
        }

      msg->data.op_flf.aead.aad_len = datalen;
      msg->data.op_flf.aead.src_data_len = 0;

      /* Send packet without waiting - AAD produces no output */

      ret = rpmsg_crypto_send_recv(rdev, session, &msg->header,
                                   RPMSG_CRYPTO_PROCESS, crp->crp_reqid,
                                   len, false);
      if (ret < 0)
        {
          return ret;
        }
    }
  while (remaining > 0);

  return session->result;
}

static int
rpmsg_crypto_process_aead_update(FAR struct rpmsg_crypto_s *rdev,
                                 FAR struct rpmsg_crypto_process_s *info,
                                 FAR struct rpmsg_crypto_session_s *session,
                                 FAR struct cryptop *crp)
{
  FAR struct rpmsg_crypto_process_s *msg;
  FAR struct rpmsg_crypto_process_s *rsp;
  FAR char *buf;
  uint32_t remaining = crp->crp_ilen;
  uint32_t flags = crp->crp_desc->crd_flags;
  uint32_t offset = 0;
  uint32_t datalen;
  uint32_t space;
  uint32_t len;
  bool async = crp->crp_flags & CRYPTO_F_CBIMM;
  int blksize;
  int ret;

  blksize = virtio_crypto_get_blocksize(crp->crp_desc->crd_next->crd_alg);
  if (blksize < 0)
    {
      return blksize;
    }

  /* Process data in packets if needed */

  while (remaining > 0)
    {
      msg = rpmsg_crypto_get_process_msg(rdev, info, &space);
      if (msg == NULL)
        {
          return -ENOMEM;
        }

      buf = msg->buf;
      len = sizeof(*msg);
      msg->data.header.flag = flags;
      msg->data.op_flf.aead.dst_data_len = crp->crp_olen;
      msg->data.op_flf.aead.aad_len = 0;

      /* Copy data */

      if (space <= len)
        {
          rpmsg_release_tx_buffer(&rdev->ept, msg);
          return -ENOMEM;
        }

      datalen = MIN(space - len, remaining);
      if (datalen < remaining)
        {
          datalen = ALIGN_DOWN(datalen, blksize);
        }

      if (datalen == 0)
        {
          rpmsg_release_tx_buffer(&rdev->ept, msg);
          return -ENOMEM;
        }

      memcpy(buf, crp->crp_buf + offset, datalen);
      len += datalen;
      offset += datalen;
      remaining -= datalen;

      msg->data.op_flf.aead.src_data_len = datalen;

      ret = rpmsg_crypto_send_recv(rdev, session, &msg->header,
                                   RPMSG_CRYPTO_PROCESS, crp->crp_reqid,
                                   len, !async);
      if (ret < 0)
        {
          return ret;
        }

      if (!async && session->data != NULL)
        {
          rsp = session->data;
          memcpy(crp->crp_dst + offset - datalen, rsp->buf, datalen);
          rpmsg_release_rx_buffer(&rdev->ept, rsp);
          session->data = NULL;
        }
    }

  return async ? OK : session->result;
}

static int
rpmsg_crypto_process_aead_final(FAR struct rpmsg_crypto_s *rdev,
                                FAR struct rpmsg_crypto_process_s *info,
                                FAR struct rpmsg_crypto_session_s *session,
                                FAR struct cryptop *crp)
{
  FAR struct rpmsg_crypto_process_s *msg;
  FAR struct rpmsg_crypto_process_s *rsp;
  bool async = crp->crp_flags & CRYPTO_F_CBIMM;
  uint32_t flags = crp->crp_desc->crd_flags;
  uint32_t space;
  int ret;

  msg = rpmsg_crypto_get_process_msg(rdev, info, &space);
  if (msg == NULL)
    {
      return -ENOMEM;
    }

  /* Pass the flags to server for proper handling */

  msg->data.header.flag = flags;
  msg->data.op_flf.aead.dst_data_len = 0;
  msg->data.op_flf.aead.iv_len = 0;
  msg->data.op_flf.aead.aad_len = 0;
  msg->data.op_flf.aead.src_data_len = 0;

  /* Both encrypt and decrypt receive tag from server */

  ret = rpmsg_crypto_send_recv(rdev, session, &msg->header,
                               RPMSG_CRYPTO_PROCESS, crp->crp_reqid,
                               sizeof(*msg), !async);
  if (ret < 0)
    {
      return ret;
    }

  if (!async && session->data != NULL)
    {
      rsp = session->data;
      memcpy(crp->crp_mac, rsp->buf, crp->crp_olen);
      rpmsg_release_rx_buffer(&rdev->ept, rsp);
      session->data = NULL;
    }

  return async ? OK : session->result;
}

static int
rpmsg_crypto_process_aead(FAR struct rpmsg_crypto_s *rdev,
                          FAR struct rpmsg_crypto_process_s *info,
                          FAR struct rpmsg_crypto_session_s *session,
                          FAR struct cryptop *crp)
{
  uint32_t flags = crp->crp_desc->crd_flags;

  if (flags & CRD_F_UPDATE_AAD)
    {
      return rpmsg_crypto_process_aead_update_aad(rdev, info, session, crp);
    }
  else if (flags & CRD_F_UPDATE)
    {
      return rpmsg_crypto_process_aead_update(rdev, info, session, crp);
    }
  else
    {
      return rpmsg_crypto_process_aead_final(rdev, info, session, crp);
    }
}

static int rpmsg_crypto_process(FAR struct cryptop *crp)
{
  FAR struct rpmsg_crypto_s *rdev;
  FAR struct rpmsg_crypto_session_s *session;
  struct rpmsg_crypto_process_s info;
  uint32_t hid = (uint32_t)(crp->crp_sid >> 32);
  uint32_t sid = (uint32_t)crp->crp_sid;
  FAR void *opaque = crp->crp_opaque;
  int ret;

  memset(&info, 0, sizeof(info));
  rdev = (FAR struct rpmsg_crypto_s *)crypto_driver_get_priv(hid);
  if (rdev == NULL)
    {
      rpmsgerr("Invalid context\n");
      return -EINVAL;
    }

  if (sid >= rdev->sesnum || !rdev->sessions[sid].used)
    {
      rpmsgerr("Invalid sid\n");
      return -EINVAL;
    }

  session = &rdev->sessions[sid];

  if (crp->crp_flags & CRYPTO_F_CANCEL)
    {
      return rpmsg_crypto_cancel(rdev, crp->crp_reqid,
                                 session->tid, 0);
    }

  ret = rpmsg_crypto_fill_data_request(crp, &info);
  if (ret < 0)
    {
      rpmsgerr("Failed to fill data request\n");
      return ret;
    }

  if (crp->crp_flags & CRYPTO_F_CBIMM)
    {
      session->data = opaque;
      session->offset = 0;
    }
  else
    {
      session->data = NULL;
    }

  info.header.tid = session->tid;
  session->result = 0;
  if (info.data.header.opcode == VIRTIO_CRYPTO_CIPHER_ENCRYPT ||
      info.data.header.opcode == VIRTIO_CRYPTO_CIPHER_DECRYPT)
    {
      ret = rpmsg_crypto_process_endec(rdev, &info, session, crp);
    }
  else if (info.data.header.opcode == VIRTIO_CRYPTO_HASH ||
           info.data.header.opcode == VIRTIO_CRYPTO_MAC)
    {
      ret = rpmsg_crypto_process_hash_mac(rdev, &info, session, crp);
    }
  else if (info.data.header.opcode == VIRTIO_CRYPTO_AEAD_ENCRYPT ||
           info.data.header.opcode == VIRTIO_CRYPTO_AEAD_DECRYPT)
    {
      ret = rpmsg_crypto_process_aead(rdev, &info, session, crp);
    }
  else
    {
      return -ENOTSUP;
    }

  crp->crp_etype = session->result;
  return ret == session->result ? 0 : ret;
}

static int rpmsg_crypto_freesession(uint64_t tid)
{
  FAR struct rpmsg_crypto_freesession_s *msg;
  FAR struct rpmsg_crypto_session_s *session;
  FAR struct rpmsg_crypto_s *rdev;
  uint32_t hid = (uint32_t)(tid >> 32);
  uint32_t sid = (uint32_t)tid;
  uint32_t space;
  int ret;

  rdev = (FAR struct rpmsg_crypto_s *)crypto_driver_get_priv(hid);
  if (rdev == NULL)
    {
      rpmsgerr("Invalid context\n");
      return -EINVAL;
    }

  if (sid >= rdev->sesnum)
    {
      rpmsgerr("Invalid sid\n");
      return -EINVAL;
    }

  session = &rdev->sessions[sid];
  msg = rpmsg_crypto_get_tx_payload_buffer(rdev, &space);
  if (msg == NULL)
    {
      return -ENOMEM;
    }

  msg->ctrl.header.opcode = VIRTIO_CRYPTO_CIPHER_DESTROY_SESSION;
  msg->header.tid = session->tid;
  ret = rpmsg_crypto_send_recv(rdev, session, &msg->header,
                               RPMSG_CRYPTO_FREESESSION, 0,
                               sizeof(*msg), true);
  rpmsg_crypto_put_session(rdev, sid);
  return ret;
}

static int rpmsg_crypto_rsa_process(FAR struct cryptkop *krp)
{
  FAR struct virtio_crypto_op_ctrl_req_s *ctrl;
  FAR struct virtio_crypto_op_data_req_s *data;
  FAR struct rpmsg_crypto_session_s *session;
  FAR struct rpmsg_crypto_kprocess_s *msg;
  FAR struct rpmsg_crypto_s *rdev;
  FAR struct crparam *dst;
  FAR struct crparam *src;
  FAR struct crparam *n;
  FAR struct crparam *e;
  FAR struct crparam *d;
  FAR struct crparam *p;
  FAR struct crparam *q;
  FAR struct crparam *dp;
  FAR struct crparam *dq;
  FAR struct crparam *u;
  unsigned char keybuf[CONFIG_CRYPTO_RPMSG_RSA_KEYBUF_SIZE];
  uint32_t data_len = 0;
  uint32_t keybufsize;
  uint32_t keybits;
  uint32_t space;
  uint32_t sid;
  ssize_t keylen;
  int ret;

  rdev = (FAR struct rpmsg_crypto_s *)crypto_driver_get_priv(krp->krp_hid);
  if (rdev == NULL)
    {
      rpmsgerr("Invalid context\n");
      return -EINVAL;
    }

  msg = rpmsg_crypto_get_tx_payload_buffer(rdev, &space);
  if (msg == NULL)
    {
      return -ENOMEM;
    }

  memset(msg, 0, sizeof(*msg));
  ctrl = &msg->ctrl;
  data = &msg->data;
  keybits = krp->krp_param[1].crp_nbits;
  keybufsize = VIRTIO_CRYPTO_RSA_DER_MAX_BYTES(
                keybits, krp->krp_keytype == CRYPTO_KEY_TYPE_PRIVATE);
  keybufsize = ALIGN_UP(keybufsize, 4);
  if (keybufsize > CONFIG_CRYPTO_RPMSG_RSA_KEYBUF_SIZE)
    {
      rpmsgerr("keybufsize %" PRIu32 " exceeds buffer %d\n",
               keybufsize, CONFIG_CRYPTO_RPMSG_RSA_KEYBUF_SIZE);
      rpmsg_release_tx_buffer(&rdev->ept, msg);
      return -ENOSPC;
    }

  memset(keybuf, 0, keybufsize);
  ret = rpmsg_crypto_get_session(rdev);
  if (ret < 0)
    {
      rpmsg_release_tx_buffer(&rdev->ept, msg);
      return ret;
    }

  sid = ret;
  session = &rdev->sessions[sid];

  /* Pad ctrl header for passing key parameters */

  ctrl->header.algo = virtio_crypto_asym_get_alg(krp->krp_op);
  ctrl->header.opcode = VIRTIO_CRYPTO_AKCIPHER_CREATE_SESSION;
  ctrl->op_flf.akcipher.algo = ctrl->header.algo;
  ctrl->op_flf.akcipher.key_type =
                        krp->krp_keytype == CRYPTO_KEY_TYPE_PRIVATE ?
                        VIRTIO_CRYPTO_AKCIPHER_KEY_TYPE_PRIVATE :
                        VIRTIO_CRYPTO_AKCIPHER_KEY_TYPE_PUBLIC;
  ctrl->op_flf.akcipher.algo_flf.rsa.padding_algo = krp->krp_padding;
  ctrl->op_flf.akcipher.algo_flf.rsa.hash_algo = krp->krp_hash;

  if (krp->krp_keytype == CRYPTO_KEY_TYPE_PRIVATE)
    {
      n = &krp->krp_param[1];
      e = &krp->krp_param[2];
      d = &krp->krp_param[3];
      p = &krp->krp_param[4];
      q = &krp->krp_param[5];
      dp = &krp->krp_param[6];
      dq = &krp->krp_param[7];
      u = &krp->krp_param[8];
      keylen = virtio_crypto_write_rsa_key_der(
          (FAR unsigned char *)n->crp_p, n->crp_nbits / 8,
          (FAR unsigned char *)e->crp_p, e->crp_nbits / 8,
          (FAR unsigned char *)d->crp_p, d->crp_nbits / 8,
          (FAR unsigned char *)p->crp_p, p->crp_nbits / 8,
          (FAR unsigned char *)q->crp_p, q->crp_nbits / 8,
          (FAR unsigned char *)dp->crp_p, dq->crp_nbits / 8,
          (FAR unsigned char *)dq->crp_p, dp->crp_nbits / 8,
          (FAR unsigned char *)u->crp_p, u->crp_nbits / 8,
          (FAR unsigned char *)keybuf, keybufsize);
    }
  else
    {
      n = &krp->krp_param[1];
      e = &krp->krp_param[2];
      keylen = virtio_crypto_write_rsa_pubkey_der(
                (FAR unsigned char *)n->crp_p, n->crp_nbits / 8,
                (FAR unsigned char *)e->crp_p, e->crp_nbits / 8,
                (FAR unsigned char *)keybuf, keybufsize);
    }

  if (keylen < 0)
    {
      rpmsg_release_tx_buffer(&rdev->ept, msg);
      rpmsg_crypto_put_session(rdev, sid);
      return keylen;
    }

  keylen = ALIGN_UP(keylen, 4); /* keysize align to 4 bytes */
  if (keylen > keybufsize)
    {
      rpmsg_release_tx_buffer(&rdev->ept, msg);
      rpmsg_crypto_put_session(rdev, sid);
      return -EINVAL;
    }

  ctrl->op_flf.akcipher.key_len = keylen;

  /* Pad data header for passing data */

  src = &krp->krp_param[0];
  dst = &krp->krp_param[krp->krp_iparams + krp->krp_oparams - 1];

  /* Calculate actual total data length and check before copying */

  data_len = keylen + src->crp_nbits / 8;
  if (krp->krp_optype == CRYPTO_OP_VERIFY)
    {
      data_len += dst->crp_nbits / 8;
    }

  if (sizeof(*msg) + data_len > space)
    {
      rpmsg_crypto_put_session(rdev, sid);
      rpmsg_release_tx_buffer(&rdev->ept, msg);
      return -ENOMEM;
    }

  memcpy(msg->buf, keybuf + keybufsize - keylen, keylen);
  data->header.algo = virtio_crypto_asym_get_alg(krp->krp_op);
  data->header.flag = krp->krp_flags;
  memcpy(msg->buf + keylen, src->crp_p, src->crp_nbits / 8);
  switch (krp->krp_optype)
    {
      case CRYPTO_OP_DECRYPT:
        data->header.opcode = VIRTIO_CRYPTO_AKCIPHER_DECRYPT;
        data->op_flf.akcipher.src_data_len = src->crp_nbits / 8;
        data->op_flf.akcipher.dst_data_len = dst->crp_nbits / 8;
        break;
      case CRYPTO_OP_ENCRYPT:
        data->header.opcode = VIRTIO_CRYPTO_AKCIPHER_ENCRYPT;
        data->op_flf.akcipher.src_data_len = src->crp_nbits / 8;
        data->op_flf.akcipher.dst_data_len = dst->crp_nbits / 8;
        break;
      case CRYPTO_OP_SIGN:
        data->header.opcode = VIRTIO_CRYPTO_AKCIPHER_SIGN;
        data->op_flf.akcipher.src_data_len = src->crp_nbits / 8;
        data->op_flf.akcipher.dst_data_len = dst->crp_nbits / 8;
        break;
      case CRYPTO_OP_VERIFY:
        data->header.opcode = VIRTIO_CRYPTO_AKCIPHER_VERIFY;
        data->op_flf.akcipher.src_data_len = src->crp_nbits / 8;
        data->op_flf.akcipher.dst_data_len = dst->crp_nbits / 8;
        memcpy(msg->buf + keylen + src->crp_nbits / 8,
               dst->crp_p, dst->crp_nbits / 8);
        break;
    }

  session->data = krp;

  ret = rpmsg_crypto_send_recv(rdev, session, &msg->header,
                               RPMSG_CRYPTO_KPROCESS, krp->krp_reqid,
                               sizeof(*msg) + data_len,
                               !(krp->krp_flags & CRYPTO_F_CBIMM));
  if (ret < 0)
    {
      rpmsg_crypto_put_session(rdev, sid);
      return ret;
    }

  if (!(krp->krp_flags & CRYPTO_F_CBIMM))
    {
      rpmsg_crypto_put_session(rdev, sid);
    }

  return OK;
}

static int rpmsg_crypto_ecdsa_process(FAR struct cryptkop *krp)
{
  FAR struct virtio_crypto_op_ctrl_req_s *ctrl;
  FAR struct virtio_crypto_op_data_req_s *data;
  FAR struct rpmsg_crypto_session_s *session;
  FAR struct rpmsg_crypto_kprocess_s *msg;
  FAR struct rpmsg_crypto_s *rdev;
  FAR struct crparam *src;
  FAR struct crparam *d;
  FAR struct crparam *x;
  FAR struct crparam *y;
  FAR struct crparam *r;
  FAR struct crparam *s;
  uint32_t data_len = 0;
  uint32_t space;
  uint32_t sid;
  int ret;

  rdev = (FAR struct rpmsg_crypto_s *)crypto_driver_get_priv(krp->krp_hid);
  if (rdev == NULL)
    {
      rpmsgerr("Invalid context\n");
      return -EINVAL;
    }

  msg = rpmsg_crypto_get_tx_payload_buffer(rdev, &space);
  if (msg == NULL)
    {
      return -ENOMEM;
    }

  memset(msg, 0, sizeof(*msg));
  ctrl = &msg->ctrl;
  data = &msg->data;
  ret = rpmsg_crypto_get_session(rdev);
  if (ret < 0)
    {
      rpmsg_release_tx_buffer(&rdev->ept, msg);
      return ret;
    }

  sid = ret;
  session = &rdev->sessions[sid];

  /* Pad ctrl header for passing key parameters */

  ctrl->header.algo = virtio_crypto_asym_get_alg(krp->krp_op);
  ctrl->header.opcode = VIRTIO_CRYPTO_AKCIPHER_CREATE_SESSION;
  ctrl->op_flf.akcipher.algo = ctrl->header.algo;
  ctrl->op_flf.akcipher.key_type =
                        krp->krp_keytype == CRYPTO_KEY_TYPE_PRIVATE ?
                        VIRTIO_CRYPTO_AKCIPHER_KEY_TYPE_PRIVATE :
                        VIRTIO_CRYPTO_AKCIPHER_KEY_TYPE_PUBLIC;
  ctrl->op_flf.akcipher.algo_flf.ecdsa.curve_id =
                        VIRTIO_CRYPTO_CURVE_NIST_P256;

  data->header.algo = ctrl->header.algo;
  data->header.flag = krp->krp_flags;
  switch (krp->krp_optype)
    {
      case CRYPTO_OP_SIGN:
        d = &krp->krp_param[0];
        r = &krp->krp_param[2];
        s = &krp->krp_param[3];
        src = &krp->krp_param[1];

        /* Import private key */

        memcpy(msg->buf, d->crp_p, d->crp_nbits / 8);
        ctrl->op_flf.akcipher.key_len = d->crp_nbits / 8;
        data_len += d->crp_nbits / 8;

        /* Import source data */

        memcpy(msg->buf + ctrl->op_flf.akcipher.key_len,
               src->crp_p, src->crp_nbits / 8);
        data->op_flf.akcipher.src_data_len = src->crp_nbits / 8;
        data_len += src->crp_nbits / 8;

        data->op_flf.akcipher.dst_data_len = (r->crp_nbits +
                                              s->crp_nbits) / 8;
        data->header.opcode = VIRTIO_CRYPTO_AKCIPHER_SIGN;
        break;
      case CRYPTO_OP_VERIFY:
        x = &krp->krp_param[0];
        y = &krp->krp_param[1];
        r = &krp->krp_param[3];
        s = &krp->krp_param[4];
        src = &krp->krp_param[5];

        /* Import public key */

        memcpy(msg->buf, x->crp_p, x->crp_nbits / 8);
        data_len += x->crp_nbits / 8;
        memcpy(msg->buf + data_len, y->crp_p, y->crp_nbits / 8);
        data_len += y->crp_nbits / 8;
        ctrl->op_flf.akcipher.key_len = data_len;

        /* Import signature */

        memcpy(msg->buf + data_len, r->crp_p, r->crp_nbits / 8);
        data_len += r->crp_nbits / 8;
        memcpy(msg->buf + data_len, s->crp_p, s->crp_nbits / 8);
        data_len += s->crp_nbits / 8;
        data->op_flf.akcipher.src_data_len = (r->crp_nbits +
                                              s->crp_nbits) / 8;

        /* Import source data */

        memcpy(msg->buf + data_len, src->crp_p, src->crp_nbits / 8);
        data_len += src->crp_nbits / 8;

        data->op_flf.akcipher.dst_data_len = src->crp_nbits / 8;
        data->header.opcode = VIRTIO_CRYPTO_AKCIPHER_VERIFY;
        break;
      case CRYPTO_OP_GENKEY:
        d = &krp->krp_param[0];
        x = &krp->krp_param[1];
        y = &krp->krp_param[2];

        data->header.opcode = VIRTIO_CRYPTO_AKCIPHER_GENKEY;
        data->op_flf.akcipher.src_data_len = d->crp_nbits / 8;
        data->op_flf.akcipher.dst_data_len = (x->crp_nbits +
                                              y->crp_nbits) / 8;
        break;
      default:
        rpmsg_release_tx_buffer(&rdev->ept, msg);
        rpmsg_crypto_put_session(rdev, sid);
        return -EINVAL;
    }

  session->data = krp;

  ret = rpmsg_crypto_send_recv(rdev, session, &msg->header,
                               RPMSG_CRYPTO_KPROCESS, krp->krp_reqid,
                               sizeof(*msg) + data_len,
                               !(krp->krp_flags & CRYPTO_F_CBIMM));
  if (ret < 0)
    {
      rpmsg_crypto_put_session(rdev, sid);
      return ret;
    }

  if (!(krp->krp_flags & CRYPTO_F_CBIMM))
    {
      rpmsg_crypto_put_session(rdev, sid);
    }

  return OK;
}

static int rpmsg_crypto_ecdh_process(FAR struct cryptkop *krp)
{
  FAR struct virtio_crypto_op_ctrl_req_s *ctrl;
  FAR struct virtio_crypto_op_data_req_s *data;
  FAR struct rpmsg_crypto_session_s *session;
  FAR struct rpmsg_crypto_kprocess_s *msg;
  FAR struct rpmsg_crypto_s *rdev;
  FAR struct crparam *x;
  FAR struct crparam *y;
  FAR struct crparam *d;
  uint32_t data_len = 0;
  uint32_t space;
  uint32_t sid;
  int ret;

  rdev = (FAR struct rpmsg_crypto_s *)crypto_driver_get_priv(krp->krp_hid);
  if (rdev == NULL)
    {
      rpmsgerr("Invalid context\n");
      return -EINVAL;
    }

  msg = rpmsg_crypto_get_tx_payload_buffer(rdev, &space);
  if (msg == NULL)
    {
      return -ENOMEM;
    }

  memset(msg, 0, sizeof(*msg));
  ctrl = &msg->ctrl;
  data = &msg->data;
  ret = rpmsg_crypto_get_session(rdev);
  if (ret < 0)
    {
      rpmsg_release_tx_buffer(&rdev->ept, msg);
      return ret;
    }

  sid = ret;
  session = &rdev->sessions[sid];

  ctrl->header.algo = virtio_crypto_asym_get_alg(krp->krp_op);
  ctrl->op_flf.akcipher.algo = ctrl->header.algo;
  ctrl->op_flf.akcipher.algo_flf.ecdsa.curve_id =
                        VIRTIO_CRYPTO_CURVE_NIST_P256;

  data->header.algo = ctrl->header.algo;
  data->header.flag = krp->krp_flags;
  switch (krp->krp_op)
    {
      case CRK_ECDH_GEN_PUBLIC:

        /* out param[0]=d, param[1]=x, param[2]=y */

        d = &krp->krp_param[0];
        x = &krp->krp_param[1];
        y = &krp->krp_param[2];

        data->header.opcode = VIRTIO_CRYPTO_AKCIPHER_GENKEY;
        data->op_flf.akcipher.src_data_len = d->crp_nbits / 8;
        data->op_flf.akcipher.dst_data_len = (x->crp_nbits +
                                              y->crp_nbits) / 8;
        break;
      case CRK_ECDH_COMPUTE_SHARED:

        /* in param[0]=priv, param[1]=peer.x, param[2]=peer.y;
         * out param[3]=shared secret.
         */

        d = &krp->krp_param[0];
        x = &krp->krp_param[1];
        y = &krp->krp_param[2];

        if (d->crp_p == NULL || x->crp_p == NULL || y->crp_p == NULL)
          {
            rpmsg_release_tx_buffer(&rdev->ept, msg);
            rpmsg_crypto_put_session(rdev, sid);
            return -EINVAL;
          }

        if ((d->crp_nbits + x->crp_nbits + y->crp_nbits) / 8 >
            space - sizeof(*msg))
          {
            rpmsg_release_tx_buffer(&rdev->ept, msg);
            rpmsg_crypto_put_session(rdev, sid);
            return -EMSGSIZE;
          }

        /* Private key length carried in ctrl.key_len; peer public key
         * (x || y) length carried in src_data_len. This avoids assuming
         * the three segments are equally sized (which only holds for
         * prime-field curves) and mirrors the ECDSA verify layout.
         */

        memcpy(msg->buf, d->crp_p, d->crp_nbits / 8);
        data_len += d->crp_nbits / 8;
        ctrl->op_flf.akcipher.key_len = d->crp_nbits / 8;

        memcpy(msg->buf + data_len, x->crp_p, x->crp_nbits / 8);
        data_len += x->crp_nbits / 8;
        memcpy(msg->buf + data_len, y->crp_p, y->crp_nbits / 8);
        data_len += y->crp_nbits / 8;

        data->header.opcode = VIRTIO_CRYPTO_AKCIPHER_DERIVE;
        data->op_flf.akcipher.src_data_len = (x->crp_nbits +
                                              y->crp_nbits) / 8;
        data->op_flf.akcipher.dst_data_len =
                        krp->krp_param[3].crp_nbits / 8;
        break;
      default:
        rpmsg_release_tx_buffer(&rdev->ept, msg);
        rpmsg_crypto_put_session(rdev, sid);
        return -EINVAL;
    }

  session->data = krp;

  ret = rpmsg_crypto_send_recv(rdev, session, &msg->header,
                               RPMSG_CRYPTO_KPROCESS, krp->krp_reqid,
                               sizeof(*msg) + data_len,
                               !(krp->krp_flags & CRYPTO_F_CBIMM));
  if (ret < 0)
    {
      rpmsg_crypto_put_session(rdev, sid);
      return ret;
    }

  if (!(krp->krp_flags & CRYPTO_F_CBIMM))
    {
      rpmsg_crypto_put_session(rdev, sid);
    }

  return OK;
}

static int rpmsg_crypto_kprocess(FAR struct cryptkop *krp)
{
  if (krp->krp_flags & CRYPTO_F_CANCEL)
    {
      return rpmsg_crypto_cancel(
        crypto_driver_get_priv(krp->krp_hid),
        krp->krp_reqid, 0, krp->krp_op);
    }

  switch (virtio_crypto_asym_get_alg(krp->krp_op))
    {
      case VIRTIO_CRYPTO_AKCIPHER_RSA:
        return rpmsg_crypto_rsa_process(krp);
      case VIRTIO_CRYPTO_AKCIPHER_ECDSA:
        return rpmsg_crypto_ecdsa_process(krp);
      case VIRTIO_CRYPTO_AKCIPHER_ECDH:
        return rpmsg_crypto_ecdh_process(krp);
    }

  return -EINVAL;
}

static int rpmsg_crypto_keyprocess(FAR struct cryptkop *krp)
{
  FAR struct rpmsg_crypto_session_s *session;
  FAR struct rpmsg_crypto_keyprocess_s *msg;
  FAR struct rpmsg_crypto_s *rdev;
  uint32_t data_len = 0;
  uint32_t space;
  uint32_t sid;
  int ret;

  if (krp->krp_flags & CRYPTO_F_CANCEL)
    {
      return rpmsg_crypto_cancel(
        crypto_driver_get_priv(krp->krp_hid),
        krp->krp_reqid, 0, krp->krp_op);
    }

  rdev = (FAR struct rpmsg_crypto_s *)crypto_driver_get_priv(krp->krp_hid);
  if (rdev == NULL)
    {
      rpmsgerr("Invalid context\n");
      return -EINVAL;
    }

  msg = rpmsg_crypto_get_tx_payload_buffer(rdev, &space);
  if (msg == NULL)
    {
      return -ENOMEM;
    }

  ret = rpmsg_crypto_get_session(rdev);
  if (ret < 0)
    {
      rpmsg_release_tx_buffer(&rdev->ept, msg);
      return ret;
    }

  sid = ret;
  session = &rdev->sessions[sid];
  memset(msg, 0, sizeof(*msg));
  msg->data.cmd = krp->krp_op;
  msg->data.flag = krp->krp_flags;
  switch (msg->data.cmd)
    {
      case CRK_ALLOCATE_KEY:
        msg->data.dst_data_len = krp->krp_param[0].crp_nbits / 8;
        break;
      case CRK_VALIDATE_KEYID:
      case CRK_DELETE_KEY:
      case CRK_SAVE_KEY:
      case CRK_LOAD_KEY:
      case CRK_UNLOAD_KEY:
        msg->data.name_len = krp->krp_param[0].crp_nbits / 8;
        break;
      case CRK_IMPORT_KEY:
      case CRK_GENERATE_AES_KEY:
      case CRK_GENERATE_SECP256R1_KEY:
        msg->data.name_len = krp->krp_param[0].crp_nbits / 8;
        msg->data.src_data_len = krp->krp_param[1].crp_nbits / 8;
        break;
      case CRK_EXPORT_KEY:
        msg->data.name_len = krp->krp_param[0].crp_nbits / 8;
        msg->data.dst_data_len = krp->krp_param[1].crp_nbits / 8;
        break;
    }

  if (msg->data.name_len)
    {
      memcpy(msg->buf, krp->krp_param[0].crp_p, msg->data.name_len);
      data_len += msg->data.name_len;
    }

  if (msg->data.src_data_len)
    {
      memcpy(msg->buf + data_len, krp->krp_param[1].crp_p,
                                  msg->data.src_data_len);
      data_len += msg->data.src_data_len;
    }

  session->data = krp;

  ret = rpmsg_crypto_send_recv(rdev, session, &msg->header,
                               RPMSG_CRYPTO_KEYPROCESS, krp->krp_reqid,
                               sizeof(*msg) + data_len,
                               !(krp->krp_flags & CRYPTO_F_CBIMM));
  if (ret < 0)
    {
      rpmsg_crypto_put_session(rdev, sid);
      return ret;
    }

  if (!(krp->krp_flags & CRYPTO_F_CBIMM))
    {
      rpmsg_crypto_put_session(rdev, sid);
    }

  return OK;
}

static int rpmsg_crypto_newsession_handler(FAR struct rpmsg_endpoint *ept,
                                           FAR void *data, size_t len,
                                           uint32_t src, FAR void *priv)
{
  FAR struct rpmsg_crypto_header_s *header = data;
  FAR struct rpmsg_crypto_session_s *session =
    (FAR struct rpmsg_crypto_session_s *)(uintptr_t)header->cookie;

  session->result = header->result;
  if (session->result >= 0)
    {
      session->tid = header->tid;
    }

  return rpmsg_post(ept, &session->sem);
}

static int rpmsg_crypto_process_handler(FAR struct rpmsg_endpoint *ept,
                                        FAR void *data, size_t len,
                                        uint32_t src, FAR void *priv)
{
  FAR struct rpmsg_crypto_header_s *header = data;
  FAR struct rpmsg_crypto_session_s *session =
    (FAR struct rpmsg_crypto_session_s *)(uintptr_t)header->cookie;
  FAR struct rpmsg_crypto_process_s *msg = data;
  FAR struct cryptop *crp = session->data;

  session->result = header->result;
  if (crp != NULL && (crp->crp_flags & CRYPTO_F_CBIMM))
    {
      crp->crp_etype = header->result;
      if (session->result == 0)
        {
          len -= sizeof(*msg);
          if (crp->crp_dst && len > 0)
            {
              memcpy(crp->crp_dst + session->offset, msg->buf, len);
            }
          else if (crp->crp_mac && len > 0)
            {
              memcpy(crp->crp_mac, msg->buf, len);
            }

          session->offset += len;
        }

      if (session->result != 0 || session->offset >= crp->crp_olen)
        {
          session->data = NULL;
          if (crp->crp_callback)
            {
              crp->crp_callback(crp);
            }
        }

      return OK;
    }

  if (session->result == 0)
    {
      rpmsg_hold_rx_buffer(ept, data);
      session->data = data;
    }
  else
    {
      session->data = NULL;
    }

  return rpmsg_post(ept, &session->sem);
}

static int rpmsg_crypto_freesession_handler(FAR struct rpmsg_endpoint *ept,
                                            FAR void *data, size_t len,
                                            uint32_t src, FAR void *priv)
{
  FAR struct rpmsg_crypto_header_s *header = data;
  FAR struct rpmsg_crypto_session_s *session =
    (FAR struct rpmsg_crypto_session_s *)(uintptr_t)header->cookie;

  session->result = header->result;
  return rpmsg_post(ept, &session->sem);
}

static int rpmsg_crypto_kprocess_handler(FAR struct rpmsg_endpoint *ept,
                                         FAR void *data, size_t len,
                                         uint32_t src, FAR void *priv)
{
  FAR struct rpmsg_crypto_header_s *header = data;
  FAR struct rpmsg_crypto_session_s *session =
    (FAR struct rpmsg_crypto_session_s *)(uintptr_t)header->cookie;
  FAR struct rpmsg_crypto_kprocess_s *msg = data;
  FAR struct cryptkop *krp = session->data;

  len -= sizeof(*msg);
  session->result = header->result;
  krp->krp_status = header->result;
  if (session->result == 0 && msg->data.op_flf.akcipher.dst_data_len > 0)
    {
      switch (virtio_crypto_asym_get_alg(krp->krp_op))
        {
          case VIRTIO_CRYPTO_AKCIPHER_RSA:
            memcpy(
              krp->krp_param[krp->krp_iparams + krp->krp_oparams - 1].crp_p,
              msg->buf, len);
            break;
          case VIRTIO_CRYPTO_AKCIPHER_ECDSA:
            if (krp->krp_optype == CRYPTO_OP_SIGN)
              {
                memcpy(krp->krp_param[2].crp_p,
                       msg->buf, krp->krp_param[2].crp_nbits / 8);
                memcpy(krp->krp_param[3].crp_p,
                       msg->buf + krp->krp_param[2].crp_nbits / 8,
                                  krp->krp_param[3].crp_nbits / 8);
              }
            else if (krp->krp_optype == CRYPTO_OP_GENKEY)
              {
                memcpy(krp->krp_param[0].crp_p,
                       msg->buf, krp->krp_param[0].crp_nbits / 8);
                memcpy(krp->krp_param[1].crp_p,
                       msg->buf + krp->krp_param[0].crp_nbits / 8,
                                  krp->krp_param[1].crp_nbits / 8);
                memcpy(krp->krp_param[2].crp_p,
                       msg->buf + krp->krp_param[0].crp_nbits / 8 +
                                  krp->krp_param[1].crp_nbits / 8,
                                  krp->krp_param[2].crp_nbits / 8);
              }

            break;
          case VIRTIO_CRYPTO_AKCIPHER_ECDH:
            if (krp->krp_op == CRK_ECDH_GEN_PUBLIC)
              {
                /* d || x || y into param[0], param[1], param[2] */

                memcpy(krp->krp_param[0].crp_p,
                       msg->buf, krp->krp_param[0].crp_nbits / 8);
                memcpy(krp->krp_param[1].crp_p,
                       msg->buf + krp->krp_param[0].crp_nbits / 8,
                                  krp->krp_param[1].crp_nbits / 8);
                memcpy(krp->krp_param[2].crp_p,
                       msg->buf + krp->krp_param[0].crp_nbits / 8 +
                                  krp->krp_param[1].crp_nbits / 8,
                                  krp->krp_param[2].crp_nbits / 8);
              }
            else
              {
                /* shared secret into param[3] */

                memcpy(krp->krp_param[3].crp_p,
                       msg->buf, krp->krp_param[3].crp_nbits / 8);
              }

            break;
          default:
            rpmsgerr("Unsupported asymmetric algorithm\n");
            session->result = -ENOTSUP;
            break;
        }
    }

  if ((krp->krp_flags & CRYPTO_F_CBIMM) && krp->krp_callback)
    {
      rpmsg_crypto_clean_session(session);
      return krp->krp_callback(krp);
    }

  return rpmsg_post(ept, &session->sem);
}

static int rpmsg_crypto_keyprocess_handler(FAR struct rpmsg_endpoint *ept,
                                           FAR void *data, size_t len,
                                           uint32_t src, FAR void *priv)
{
  FAR struct rpmsg_crypto_header_s *header = data;
  FAR struct rpmsg_crypto_session_s *session =
    (FAR struct rpmsg_crypto_session_s *)(uintptr_t)header->cookie;
  FAR struct rpmsg_crypto_keyprocess_s *msg = data;
  FAR struct cryptkop *krp = session->data;

  len -= sizeof(*msg);
  session->result = header->result;
  krp->krp_status = header->result;
  if (session->result == 0 && msg->data.dst_data_len > 0)
    {
      memcpy(krp->krp_param[krp->krp_iparams].crp_p, msg->buf, len);
      if (krp->krp_op == CRK_EXPORT_KEY)
        {
          krp->krp_param[1].crp_nbits = len * 8;
        }
    }

  if ((krp->krp_flags & CRYPTO_F_CBIMM) && krp->krp_callback)
    {
      rpmsg_crypto_clean_session(session);
      return krp->krp_callback(krp);
    }

  return rpmsg_post(ept, &session->sem);
}

/****************************************************************************
 * Name: rpmsg_crypto_ept_cb
 *
 * Description:
 *   Rpmsg crypto end point callback function, this function will be called
 *   when receive the remote cpu message.
 *
 * Parameters:
 *   ept  - The rpmsg-crypto end point
 *   data - The received data
 *   len  - The received data length
 *   src  - unknown
 *   priv - unknown
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
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

  rpmsgerr("Failed to find command handler\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: rpmsg_crypto_ns_bound
 *
 * Description:
 *   Rpmsg crypto end point service bound callback function , called when
 *   remote end point address is received.
 *
 * Parameters:
 *   ept  - The rpmsg-crypto end point
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void rpmsg_crypto_ns_bound(FAR struct rpmsg_endpoint *ept)
{
  FAR struct rpmsg_crypto_s *priv = ept->priv;

  rpmsg_post(&priv->ept, &priv->wait);
}

/****************************************************************************
 * Name: rpmsg_crypto_created
 *
 * Description:
 *   Rpmsg crypto create function, this function will be called by rptun to
 *   create a rpmsg-crypto end point.
 *
 * Parameters:
 *   rdev  - The rpmsg-crypto end point
 *   priv_ - Rpmsg-crypto handle
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void rpmsg_crypto_created(FAR struct rpmsg_device *rdev,
                                 FAR void *priv_)
{
  FAR struct rpmsg_crypto_s *priv = priv_;

  if (strcmp(priv->remotecpu, rpmsg_get_cpuname(rdev)) == 0)
    {
      priv->ept.priv = priv;
      priv->ept.ns_bound_cb = rpmsg_crypto_ns_bound;
      rpmsg_set_priority(&priv->ept, RPMSG_PRIO_RT);
      rpmsg_create_ept(&priv->ept, rdev, RPMSG_CRYPTO_NAME_PREFIX,
                       RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                       rpmsg_crypto_ept_cb, NULL);
    }
}

/****************************************************************************
 * Name: rpmsg_crypto_destroy
 *
 * Description:
 *   Rpmsg crypto destroy function, this function will be called by rptun to
 *   destroy rpmsg-crypto end point.
 *
 * Parameters:
 *   rdev     - The rpmsg-crypto end point
 *   priv_    - Rpmsg-device handle
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void rpmsg_crypto_destroy(FAR struct rpmsg_device *rdev,
                                 FAR void *priv_)
{
  FAR struct rpmsg_crypto_s *priv = priv_;

  if (strcmp(priv->remotecpu, rpmsg_get_cpuname(rdev)) == 0)
    {
      rpmsg_destroy_ept(&priv->ept);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpmsg_crypto_register
 *
 * Description:
 *   Rpmsg-crypto client initialize function, the client cpu should call
 *   this function in the board initialize process.
 *
 * Parameters:
 *   remotecpu  - the server cpu name
 *
 * Returned Values:
 *   OK on success; A negated errno value is returned on any failure.
 *
 ****************************************************************************/

int rpmsg_crypto_register(FAR const char *remotecpu)
{
  FAR struct rpmsg_crypto_s *rdev;
  int algs[CRYPTO_ALGORITHM_MAX + 1];
  int kalgs[CRK_ALGORITHM_MAX + 1];
  int keyalgs[CRYPTO_ALGORITHM_MAX + 1];
  int ret;
  int i;

  /* Arguments check */

  if (remotecpu == NULL)
    {
      return -EINVAL;
    }

  /* Initialize the rpmsg crypto */

  rdev = kmm_zalloc(sizeof(struct rpmsg_crypto_s));
  if (rdev == NULL)
    {
      rpmsgerr("rpmsg crypto failed to allocate memory\n");
      return -ENOMEM;
    }

  /* Pre-allocate session slots to avoid realloc during concurrent use */

  rdev->sessions = kmm_zalloc(CONFIG_CRYPTO_RPMSG_SESSIONS_NUM *
                              sizeof(struct rpmsg_crypto_session_s));
  if (rdev->sessions == NULL)
    {
      kmm_free(rdev);
      return -ENOMEM;
    }

  rdev->sesnum = CONFIG_CRYPTO_RPMSG_SESSIONS_NUM;
  rdev->remotecpu = remotecpu;
  nxsem_init(&rdev->wait, 0, 0);
  nxmutex_init(&rdev->lock);

  /* Register the rpmsg callback */

  ret = rpmsg_register_callback(rdev,
                                rpmsg_crypto_created,
                                rpmsg_crypto_destroy,
                                NULL,
                                NULL);
  if (ret < 0)
    {
      rpmsgerr("rpmsg register callback failed, ret=%d\n", ret);
      goto fail_alloc;
    }

  /* Register NuttX driver for cipher */

  rdev->devid = crypto_get_driverid(CRYPTOCAP_F_REMOTE);

  crypto_driver_set_priv(rdev->devid, rdev);

  for (i = 0; i < CRYPTO_ALGORITHM_MAX + 1; i++)
    {
      algs[i] = CRYPTO_ALG_FLAG_SUPPORTED;
    }

  ret = crypto_register(rdev->devid, algs, rpmsg_crypto_newsession,
                        rpmsg_crypto_freesession, rpmsg_crypto_process);

  if (ret < 0)
    {
      rpmsgerr("rpmsg crypto cipher register failed, ret=%d\n", ret);
      goto fail_register;
    }

  /* Register NuttX driver for asymmetric algorithms */

  memset(kalgs, 0, sizeof(kalgs));
  kalgs[CRK_MOD_EXP] = CRYPTO_ALG_FLAG_SUPPORTED;
  kalgs[CRK_MOD_EXP_CRT] = CRYPTO_ALG_FLAG_SUPPORTED;
  kalgs[CRK_RSA_PKCS15_SIGN] = CRYPTO_ALG_FLAG_SUPPORTED;
  kalgs[CRK_RSA_PKCS15_VERIFY] = CRYPTO_ALG_FLAG_SUPPORTED;
  kalgs[CRK_RSA_PSS_SIGN] = CRYPTO_ALG_FLAG_SUPPORTED;
  kalgs[CRK_RSA_PSS_VERIFY] = CRYPTO_ALG_FLAG_SUPPORTED;
  kalgs[CRK_ECDSA_SECP256R1_SIGN] = CRYPTO_ALG_FLAG_SUPPORTED;
  kalgs[CRK_ECDSA_SECP256R1_VERIFY] = CRYPTO_ALG_FLAG_SUPPORTED;
  kalgs[CRK_ECDSA_SECP256R1_GENKEY] = CRYPTO_ALG_FLAG_SUPPORTED;
  kalgs[CRK_ECDH_GEN_PUBLIC] = CRYPTO_ALG_FLAG_SUPPORTED;
  kalgs[CRK_ECDH_COMPUTE_SHARED] = CRYPTO_ALG_FLAG_SUPPORTED;

  ret = crypto_kregister(rdev->devid, kalgs, rpmsg_crypto_kprocess);
  if (ret < 0)
    {
      for (i = 0; i < CRYPTO_ALGORITHM_MAX; i++)
        {
          crypto_unregister(rdev->devid, i);
        }

      rpmsgerr("rpmsg crypto kprocess register failed, ret=%d\n", ret);
      goto fail_register;
    }

  /* Register NuttX driver for key management */

  rdev->keydevid = crypto_get_driverid(CRYPTOCAP_F_KEY_MGMT |
                                       CRYPTOCAP_F_REMOTE);

  crypto_driver_set_priv(rdev->keydevid, rdev);

  keyalgs[CRK_ALLOCATE_KEY] = CRYPTO_ALG_FLAG_SUPPORTED;
  keyalgs[CRK_VALIDATE_KEYID] = CRYPTO_ALG_FLAG_SUPPORTED;
  keyalgs[CRK_IMPORT_KEY] = CRYPTO_ALG_FLAG_SUPPORTED;
  keyalgs[CRK_DELETE_KEY] = CRYPTO_ALG_FLAG_SUPPORTED;
  keyalgs[CRK_EXPORT_KEY] = CRYPTO_ALG_FLAG_SUPPORTED;
  keyalgs[CRK_GENERATE_AES_KEY] = CRYPTO_ALG_FLAG_SUPPORTED;
  keyalgs[CRK_GENERATE_SECP256R1_KEY] = CRYPTO_ALG_FLAG_SUPPORTED;
  keyalgs[CRK_SAVE_KEY] = CRYPTO_ALG_FLAG_SUPPORTED;
  keyalgs[CRK_LOAD_KEY] = CRYPTO_ALG_FLAG_SUPPORTED;
  keyalgs[CRK_UNLOAD_KEY] = CRYPTO_ALG_FLAG_SUPPORTED;

  ret = crypto_kregister(rdev->keydevid, keyalgs, rpmsg_crypto_keyprocess);

  if (ret < 0)
    {
      rpmsgerr("rpmsg crypto key register failed, ret=%d\n", ret);
      goto fail_register;
    }

  return ret;

fail_register:
  rpmsg_unregister_callback(rdev,
                            rpmsg_crypto_created,
                            rpmsg_crypto_destroy,
                            NULL,
                            NULL);
fail_alloc:
  nxmutex_destroy(&rdev->lock);
  nxsem_destroy(&rdev->wait);
  kmm_free(rdev->sessions);
  kmm_free(rdev);
  return ret;
}
