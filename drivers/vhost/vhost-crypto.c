/****************************************************************************
 * drivers/vhost/vhost-crypto.c
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

#include <nuttx/debug.h>
#include <errno.h>
#include <string.h>

#include <crypto/asn1.h>
#include <crypto/cryptodev.h>
#include <crypto/virtio_crypto.h>
#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/vhost/vhost.h>
#include <nuttx/virtio/virtio-crypto.h>
#include <nuttx/wqueue.h>

#include "vhost-crypto.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Cap on frontend-controlled max_dataqueues, to bound stack arrays. */

#define VHOST_CRYPTO_MAX_DATAQUEUES    8

/* Sanity bound for akcipher src/dst data length (peer-controlled). */

#define VHOST_CRYPTO_AKCIPHER_LEN_MAX  4096

/* Max descriptors per request: header, iv, src, aad, dst, mac, inhdr. */

#define VHOST_CRYPTO_MAX_SEGS          8

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Session data, reference rpmsg_crypto_server_data_s */

struct vhost_crypto_session_s
{
  uint64_t tid;       /* crypto_newsession returned handle */

  /* Session data is exclusive to either the symmetric (cipher/hash/
   * MAC/AEAD) or the akcipher (RSA/ECDSA) path, decided once at
   * CREATE_SESSION and never mixed for the lifetime of the session.
   */

  union
  {
    struct
    {
      struct cryptop    crp;    /* Symmetric/hash/MAC/AEAD op descriptor */
      struct cryptodesc crda;   /* Auth descriptor (hash/MAC/AEAD) */
      struct cryptodesc crde;   /* Encrypt descriptor (cipher/AEAD) */
    } sym;

    struct
    {
      struct cryptkop   krp;        /* Akcipher op descriptor */
      FAR void         *akkey;      /* DER key buf */
      uint32_t          akkeylen;   /* DER key length */
      uint32_t          curve_id;   /* VIRTIO_CRYPTO_CURVE_* (ECDSA/ECDH) */

      /* DER scratch for ASN1 key encoding, reused across requests */

      unsigned char     keybuf[CONFIG_DRIVERS_VHOST_CRYPTO_RSA_KEYBUF_SIZE];
    } ak;
  } u;
};

/* One descriptor chain collected up-front; process_* consume in order. */

struct vhost_crypto_req_s
{
  struct virtqueue_buf vb[VHOST_CRYPTO_MAX_SEGS];
  size_t               cnt;
  size_t               pos;
};

struct vhost_crypto_priv_s
{
  FAR struct vhost_device *hdev;       /* Vhost device pointer */
  struct work_s            work;       /* Deferred work for virtqueue processing */
  uint32_t                 max_dataqueues;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  vhost_crypto_probe(FAR struct vhost_device *hdev);
static void vhost_crypto_remove(FAR struct vhost_device *hdev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct vhost_driver g_vhost_crypto_driver =
{
  VIRTIO_ID_CRYPTO,      /* device id */
  vhost_crypto_probe,    /* probe */
  vhost_crypto_remove,   /* remove */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vhost_crypto_next_buf
 ****************************************************************************/

static FAR void *vhost_crypto_next_buf(FAR struct vhost_crypto_req_s *req,
                                       FAR uint32_t *len)
{
  if (req->pos >= req->cnt)
    {
      return NULL;
    }

  *len = req->vb[req->pos].len;
  return req->vb[req->pos++].buf;
}

/****************************************************************************
 * Name: vhost_crypto_create_session_sym
 *
 * Description:
 *   Handle CREATE_SESSION for cipher/hash/MAC/AEAD (symmetric) sessions.
 *
 ****************************************************************************/

static int vhost_crypto_create_session_sym(
    FAR struct vhost_crypto_session_s *sess,
    FAR struct virtio_crypto_op_ctrl_req_s *ctrl,
    FAR void *key, uint32_t keylen)
{
  struct cryptoini cria;
  struct cryptoini crie;
  int txform = 0;
  int thash = 0;

  memset(&crie, 0, sizeof(crie));
  memset(&cria, 0, sizeof(cria));

  switch (ctrl->header.opcode)
    {
      case VIRTIO_CRYPTO_CIPHER_CREATE_SESSION:
        txform = virtio_crypto_get_cipher_alg(ctrl->header.algo);
        if (txform < 0)
          {
            return txform;
          }

        /* Handle AES-CBC key length variants */

        if (txform == CRYPTO_AES_CBC)
          {
            if (ctrl->op_flf.sym.op_flf.cipher.key_len == 24)
              {
                txform = CRYPTO_AES_192_CBC;
              }
            else if (ctrl->op_flf.sym.op_flf.cipher.key_len == 32)
              {
                txform = CRYPTO_AES_256_CBC;
              }
          }

        keylen = ctrl->op_flf.sym.op_flf.cipher.key_len;
        break;

      case VIRTIO_CRYPTO_HASH_CREATE_SESSION:
        thash = virtio_crypto_get_hash_alg(ctrl->header.algo);
        if (thash < 0)
          {
            return thash;
          }

        break;

      case VIRTIO_CRYPTO_MAC_CREATE_SESSION:
        keylen = ctrl->op_flf.mac.auth_key_len;

        thash = virtio_crypto_get_mac_alg(ctrl->header.algo);
        if (thash < 0)
          {
            return thash;
          }

        if (thash == CRYPTO_AES_128_CMAC)
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

        break;

      case VIRTIO_CRYPTO_AEAD_CREATE_SESSION:
        txform = virtio_crypto_get_aead_alg(ctrl->header.algo);
        if (txform < 0)
          {
            return txform;
          }

        if (txform == CRYPTO_AES_GCM_16)
          {
            keylen = ctrl->op_flf.aead.key_len;
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
        else if (txform == CRYPTO_CHACHA20_POLY1305)
          {
            thash = CRYPTO_CHACHA20_POLY1305_MAC;
          }

        break;

      default:
        return -ENOTSUP;
    }

  /* Build cryptoini chain */

  if (txform)
    {
      crie.cri_alg = txform;
      crie.cri_klen = keylen * 8;
      crie.cri_key = key;
      sess->u.sym.crde.crd_alg = txform;

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
      cria.cri_key = key;
      sess->u.sym.crda.crd_alg = thash;
    }

  return crypto_newsession(&sess->tid, txform ? &crie : &cria, 1);
}

/****************************************************************************
 * Name: vhost_crypto_create_session_akcipher
 *
 * Description:
 *   Handle CREATE_SESSION for akcipher (asymmetric) sessions. The key is
 *   just stashed on the session here; RSA/ECDSA parse it lazily on first
 *   use (see vhost_crypto_process_rsa/ecdsa).
 *
 ****************************************************************************/

static int vhost_crypto_create_session_akcipher(
    FAR struct vhost_crypto_session_s *sess,
    FAR struct virtio_crypto_op_ctrl_req_s *ctrl,
    FAR void *key, uint32_t keylen)
{
  sess->u.ak.krp.krp_padding =
    ctrl->op_flf.akcipher.algo_flf.rsa.padding_algo;
  sess->u.ak.krp.krp_hash = ctrl->op_flf.akcipher.algo_flf.rsa.hash_algo;
  sess->u.ak.krp.krp_keytype =
    ctrl->op_flf.akcipher.key_type ==
    VIRTIO_CRYPTO_AKCIPHER_KEY_TYPE_PRIVATE ?
    CRYPTO_KEY_TYPE_PRIVATE : CRYPTO_KEY_TYPE_PUBLIC;

  if (ctrl->header.algo == VIRTIO_CRYPTO_AKCIPHER_ECDSA ||
      ctrl->header.algo == VIRTIO_CRYPTO_AKCIPHER_ECDH)
    {
      sess->u.ak.curve_id = ctrl->op_flf.akcipher.algo_flf.ecdsa.curve_id;
      if (virtio_crypto_curve_get_len(sess->u.ak.curve_id) < 0)
        {
          return -ENOTSUP;
        }
    }

  if (key != NULL && keylen > 0)
    {
      sess->u.ak.akkey = kmm_malloc(keylen);
      if (sess->u.ak.akkey == NULL)
        {
          return -ENOMEM;
        }

      memcpy(sess->u.ak.akkey, key, keylen);
      sess->u.ak.akkeylen = keylen;
    }

  return 0;
}

/****************************************************************************
 * Name: vhost_crypto_create_session
 *
 * Description:
 *   Handle CREATE_SESSION request from controlq.
 *   Supports cipher/hash/MAC/AEAD/akcipher session types.
 *
 ****************************************************************************/

static int vhost_crypto_create_session(
    FAR struct virtio_crypto_op_ctrl_req_s *ctrl,
    FAR void *key, uint32_t keylen,
    FAR struct virtio_crypto_create_session_input_s *input)
{
  FAR struct vhost_crypto_session_s *sess;
  int ret;

  sess = kmm_zalloc(sizeof(struct vhost_crypto_session_s));
  if (sess == NULL)
    {
      input->status = VIRTIO_CRYPTO_ERR;
      return -ENOMEM;
    }

  if (ctrl->header.opcode == VIRTIO_CRYPTO_AKCIPHER_CREATE_SESSION)
    {
      ret = vhost_crypto_create_session_akcipher(sess, ctrl, key, keylen);
    }
  else
    {
      ret = vhost_crypto_create_session_sym(sess, ctrl, key, keylen);
    }

  if (ret != 0)
    {
      kmm_free(sess);
      input->status = VIRTIO_CRYPTO_ERR;
      return ret;
    }

  input->session_id = (uintptr_t)sess;
  input->status = VIRTIO_CRYPTO_OK;
  return 0;
}

/****************************************************************************
 * Name: vhost_crypto_destroy_session
 ****************************************************************************/

static int vhost_crypto_destroy_session(
    FAR struct virtio_crypto_op_ctrl_req_s *ctrl,
    FAR void *result)
{
  FAR struct virtio_crypto_destroy_session_input_s *input = result;
  FAR struct vhost_crypto_session_s *sess;
  uint64_t session_id;

  session_id = ctrl->op_flf.destroy.session_id;
  sess = (FAR struct vhost_crypto_session_s *)(uintptr_t)session_id;

  if (sess == NULL)
    {
      input->status = VIRTIO_CRYPTO_INVSESS;
      return -EINVAL;
    }

  if (ctrl->header.opcode == VIRTIO_CRYPTO_AKCIPHER_DESTROY_SESSION)
    {
      if (sess->u.ak.akkey != NULL)
        {
          kmm_free(sess->u.ak.akkey);
        }
    }
  else
    {
      crypto_freesession(sess->tid);
    }

  kmm_free(sess);
  input->status = VIRTIO_CRYPTO_OK;
  return 0;
}

/****************************************************************************
 * Name: vhost_crypto_controlq_work
 *
 * Description:
 *   Process all pending requests on the control virtqueue.
 *
 ****************************************************************************/

static void vhost_crypto_controlq_work(
    FAR struct vhost_crypto_priv_s *priv)
{
  FAR struct virtio_crypto_op_ctrl_req_s *ctrl;
  struct vhost_crypto_req_s req;
  FAR struct virtqueue *vq;
  FAR void *result;
  FAR void *key;
  uint32_t resultlen;
  uint32_t keylen;
  int idx;

  vq = priv->hdev->vrings_info[priv->max_dataqueues].vq;

  /* Disable notifications while draining; re-enable and recheck below. */

  virtqueue_disable_cb(vq);

again:
  for (; ; )
    {
      idx = vhost_get_vq_buffers(vq, req.vb, VHOST_CRYPTO_MAX_SEGS,
                                 &req.cnt);
      if (idx < 0)
        {
          break;
        }

      /* Chain layout: ctrl_req [key] result, so result is always last
       * and the key, if any, sits between the two.
       */

      if (req.cnt < 2)
        {
          virtqueue_add_consumed_buffer(vq, idx, 0);
          continue;
        }

      ctrl = req.vb[0].buf;
      result = req.vb[req.cnt - 1].buf;
      resultlen = req.vb[req.cnt - 1].len;

      switch (ctrl->header.opcode)
        {
          case VIRTIO_CRYPTO_CIPHER_CREATE_SESSION:
          case VIRTIO_CRYPTO_HASH_CREATE_SESSION:
          case VIRTIO_CRYPTO_MAC_CREATE_SESSION:
          case VIRTIO_CRYPTO_AEAD_CREATE_SESSION:
          case VIRTIO_CRYPTO_AKCIPHER_CREATE_SESSION:

            key = NULL;
            keylen = 0;
            if (req.cnt > 2)
              {
                key = req.vb[1].buf;
                keylen = req.vb[1].len;
              }

            vhost_crypto_create_session(ctrl, key, keylen, result);

            break;

          case VIRTIO_CRYPTO_CIPHER_DESTROY_SESSION:
          case VIRTIO_CRYPTO_HASH_DESTROY_SESSION:
          case VIRTIO_CRYPTO_MAC_DESTROY_SESSION:
          case VIRTIO_CRYPTO_AEAD_DESTROY_SESSION:
          case VIRTIO_CRYPTO_AKCIPHER_DESTROY_SESSION:

            vhost_crypto_destroy_session(ctrl, result);

            break;

          default:

            /* Unknown opcode: layout is opcode-specific, so only write
             * the 1-byte status field common to every *_input_s struct.
             */

            vrterr("vhost-crypto: unknown controlq opcode 0x%x\n",
                   ctrl->header.opcode);
            if (resultlen >= 1)
              {
                *(FAR uint8_t *)result = VIRTIO_CRYPTO_NOTSUPP;
              }

            break;
        }

      virtqueue_add_consumed_buffer(vq, idx, resultlen);
    }

  virtqueue_kick(vq);

  /* Drain again if a request slipped in before re-enabling callbacks. */

  if (virtqueue_enable_cb(vq))
    {
      virtqueue_disable_cb(vq);
      goto again;
    }
}

/****************************************************************************
 * Name: vhost_crypto_process_cipher
 *
 * Description:
 *   Process cipher encrypt/decrypt request.
 *
 ****************************************************************************/

static int vhost_crypto_process_cipher(
    FAR struct vhost_crypto_session_s *sess,
    FAR struct virtio_crypto_op_data_req_s *data,
    FAR struct vhost_crypto_req_s *req)
{
  FAR struct cryptodesc *crde = &sess->u.sym.crde;
  FAR struct cryptop *crp = &sess->u.sym.crp;
  FAR void *buf;
  uint32_t len;

  crde->crd_alg = virtio_crypto_get_cipher_alg(data->header.algo);
  if (crde->crd_alg < 0)
    {
      return crde->crd_alg;
    }

  if (data->header.opcode == VIRTIO_CRYPTO_CIPHER_ENCRYPT)
    {
      crde->crd_flags |= CRD_F_ENCRYPT;
    }
  else
    {
      crde->crd_flags &= ~CRD_F_ENCRYPT;
    }

  crp->crp_sid = sess->tid;
  crp->crp_desc = crde;

  if (data->op_flf.sym.op_type_flf.cipher.iv_len)
    {
      crp->crp_ivlen = data->op_flf.sym.op_type_flf.cipher.iv_len;

      /* Get IV buffer */

      buf = vhost_crypto_next_buf(req, &len);
      if (buf == NULL)
        {
          return -EINVAL;
        }

      crp->crp_iv = buf;
    }
  else
    {
      crp->crp_ivlen = 0;
    }

  /* UPDATE flag comes from the frontend's flag, not derived from ivlen. */

  if (data->header.flag & CRD_F_UPDATE)
    {
      crde->crd_flags |= CRD_F_UPDATE;
    }
  else
    {
      crde->crd_flags &= ~CRD_F_UPDATE;
    }

  /* Get input data buffer */

  buf = vhost_crypto_next_buf(req, &len);
  if (buf == NULL)
    {
      return -EINVAL;
    }

  crp->crp_buf = buf;
  crp->crp_ilen = len;
  crde->crd_len = len;
  crp->crp_olen = len;

  /* Get output data buffer */

  buf = vhost_crypto_next_buf(req, &len);
  if (buf == NULL)
    {
      return -EINVAL;
    }

  crp->crp_dst = buf;

  return crypto_invoke(crp);
}

/****************************************************************************
 * Name: vhost_crypto_process_hash
 *
 * Description:
 *   Process hash request.
 *
 ****************************************************************************/

static int vhost_crypto_process_hash(
    FAR struct vhost_crypto_session_s *sess,
    FAR struct virtio_crypto_op_data_req_s *data,
    FAR struct vhost_crypto_req_s *req)
{
  FAR struct cryptodesc *crda = &sess->u.sym.crda;
  FAR struct cryptop *crp = &sess->u.sym.crp;
  FAR void *buf;
  uint32_t len;

  crda->crd_alg = virtio_crypto_get_hash_alg(data->header.algo);
  if (crda->crd_alg < 0)
    {
      return crda->crd_alg;
    }

  crp->crp_sid = sess->tid;
  crp->crp_desc = crda;

  /* Get src data buffer if present */

  if (data->op_flf.hash.src_data_len > 0)
    {
      buf = vhost_crypto_next_buf(req, &len);
      if (buf == NULL)
        {
          return -EINVAL;
        }

      crp->crp_buf = buf;
      crp->crp_ilen = len;
      crda->crd_len = len;
    }
  else
    {
      crp->crp_buf = NULL;
      crp->crp_ilen = 0;
      crda->crd_len = 0;
    }

  if (data->op_flf.hash.hash_result_len)
    {
      crda->crd_flags = 0;
      crp->crp_olen = data->op_flf.hash.hash_result_len;

      /* Get output buffer for hash result */

      buf = vhost_crypto_next_buf(req, &len);
      if (buf == NULL)
        {
          return -EINVAL;
        }

      crp->crp_mac = buf;
    }
  else
    {
      crda->crd_flags |= CRD_F_UPDATE;
    }

  return crypto_invoke(crp);
}

/****************************************************************************
 * Name: vhost_crypto_process_mac
 *
 * Description:
 *   Process MAC request.
 *
 ****************************************************************************/

static int vhost_crypto_process_mac(
    FAR struct vhost_crypto_session_s *sess,
    FAR struct virtio_crypto_op_data_req_s *data,
    FAR struct vhost_crypto_req_s *req)
{
  FAR struct cryptodesc *crda = &sess->u.sym.crda;
  FAR struct cryptodesc *crde = &sess->u.sym.crde;
  FAR struct cryptop *crp = &sess->u.sym.crp;
  FAR void *buf;
  uint32_t len;

  if (crda->crd_alg == CRYPTO_AES_128_CMAC ||
      crda->crd_alg == CRYPTO_AES_256_CMAC)
    {
      crda->crd_next = crde;
    }

  crp->crp_sid = sess->tid;
  crp->crp_desc = crda;
  crp->crp_iter = data->op_flf.mac.hdr.iterations;

  /* Get src data buffer if present */

  if (data->op_flf.mac.hdr.src_data_len > 0)
    {
      buf = vhost_crypto_next_buf(req, &len);
      if (buf == NULL)
        {
          return -EINVAL;
        }

      crp->crp_buf = buf;
      crp->crp_ilen = len;
      crda->crd_len = len;
      crde->crd_len = len;
    }
  else
    {
      crp->crp_buf = NULL;
      crp->crp_ilen = 0;
      crda->crd_len = 0;
      crde->crd_len = 0;
    }

  if (data->op_flf.mac.hdr.hash_result_len)
    {
      crda->crd_flags = 0;
      crp->crp_olen = data->op_flf.mac.hdr.hash_result_len;

      /* Get output buffer for MAC result */

      buf = vhost_crypto_next_buf(req, &len);
      if (buf == NULL)
        {
          return -EINVAL;
        }

      crp->crp_mac = buf;
    }
  else
    {
      crda->crd_flags |= CRD_F_UPDATE;
    }

  return crypto_invoke(crp);
}

/****************************************************************************
 * Name: vhost_crypto_process_aead
 *
 * Description:
 *   Process AEAD request (AES-GCM etc).
 *   Three-stage model: UPDATE_AAD → UPDATE → FINAL
 *
 ****************************************************************************/

static int vhost_crypto_process_aead_update_aad(
    FAR struct vhost_crypto_session_s *sess,
    FAR struct virtio_crypto_op_data_req_s *data,
    FAR struct vhost_crypto_req_s *req)
{
  FAR struct cryptodesc *crda = &sess->u.sym.crda;
  FAR struct cryptop *crp = &sess->u.sym.crp;
  FAR void *buf;
  uint32_t len;
  int ret;

  crp->crp_iv = NULL;
  crp->crp_ivlen = data->op_flf.aead.iv_len;

  if (crp->crp_ivlen > 0)
    {
      buf = vhost_crypto_next_buf(req, &len);
      if (buf == NULL)
        {
          return -EINVAL;
        }

      crp->crp_iv = buf;
    }

  /* Skip src data buffer (unused in UPDATE_AAD) */

  if (vhost_crypto_next_buf(req, &len) == NULL)
    {
      return -EINVAL;
    }

  /* Get AAD buffer */

  buf = vhost_crypto_next_buf(req, &len);
  if (buf == NULL)
    {
      return -EINVAL;
    }

  crp->crp_aadlen = data->op_flf.aead.aad_len;
  crp->crp_aad = buf;
  crp->crp_ilen = 0;
  crp->crp_buf = NULL;
  crp->crp_dst = NULL;
  crp->crp_mac = NULL;
  crda->crd_len = 0;
  crda->crd_flags = (crda->crd_flags & CRD_F_ENCRYPT) | CRD_F_UPDATE_AAD;

  ret = crypto_invoke(crp);
  if (ret == 0)
    {
      ret = crp->crp_etype;
    }

  /* Skip mac buffer (unused in UPDATE_AAD) */

  if (vhost_crypto_next_buf(req, &len) == NULL)
    {
      return -EINVAL;
    }

  return ret;
}

static int vhost_crypto_process_aead_update(
    FAR struct vhost_crypto_session_s *sess,
    FAR struct virtio_crypto_op_data_req_s *data,
    FAR struct vhost_crypto_req_s *req)
{
  FAR struct cryptodesc *crda = &sess->u.sym.crda;
  FAR struct cryptodesc *crde = &sess->u.sym.crde;
  FAR struct cryptop *crp = &sess->u.sym.crp;
  FAR void *buf;
  uint32_t datalen;
  uint32_t len;
  int ret;

  datalen = data->op_flf.aead.src_data_len;

  crp->crp_iv = NULL;
  crp->crp_aadlen = 0;
  crp->crp_aad = NULL;

  /* Get input data */

  buf = vhost_crypto_next_buf(req, &len);
  if (buf == NULL)
    {
      return -EINVAL;
    }

  crp->crp_ilen = datalen;
  crda->crd_len = datalen;
  crde->crd_len = datalen;
  crp->crp_buf = buf;
  crp->crp_olen = datalen;

  /* Get output buffer */

  buf = vhost_crypto_next_buf(req, &len);
  if (buf == NULL)
    {
      return -EINVAL;
    }

  crp->crp_dst = buf;
  crp->crp_mac = NULL;
  crda->crd_flags = (crda->crd_flags & CRD_F_ENCRYPT) | CRD_F_UPDATE;

  ret = crypto_invoke(crp);
  if (ret == 0)
    {
      ret = crp->crp_etype;
    }

  return ret;
}

static int vhost_crypto_process_aead_final(
    FAR struct vhost_crypto_session_s *sess,
    FAR struct vhost_crypto_req_s *req)
{
  FAR struct cryptodesc *crda = &sess->u.sym.crda;
  FAR struct cryptodesc *crde = &sess->u.sym.crde;
  FAR struct cryptop *crp = &sess->u.sym.crp;
  FAR void *buf;
  uint32_t len;
  int ret;

  crp->crp_ivlen = 0;
  crp->crp_iv = NULL;
  crp->crp_aadlen = 0;
  crp->crp_aad = NULL;
  crp->crp_ilen = 0;
  crda->crd_len = 0;
  crde->crd_len = 0;
  crda->crd_flags &= CRD_F_ENCRYPT;
  crp->crp_buf = NULL;
  crp->crp_olen = 0;
  crp->crp_dst = NULL;

  /* Skip src data buffer (unused in FINAL) */

  if (vhost_crypto_next_buf(req, &len) == NULL)
    {
      return -EINVAL;
    }

  /* Get tag output buffer */

  buf = vhost_crypto_next_buf(req, &len);
  if (buf == NULL)
    {
      return -EINVAL;
    }

  crp->crp_mac = buf;

  ret = crypto_invoke(crp);
  if (ret == 0)
    {
      ret = crp->crp_etype;
    }

  return ret;
}

/****************************************************************************
 * Name: vhost_crypto_process_aead
 *
 * Description:
 *   Dispatch AEAD request to the UPDATE_AAD/UPDATE/FINAL stage handler.
 *
 ****************************************************************************/

static int vhost_crypto_process_aead(
    FAR struct vhost_crypto_session_s *sess,
    FAR struct virtio_crypto_op_data_req_s *data,
    FAR struct vhost_crypto_req_s *req)
{
  FAR struct cryptodesc *crda = &sess->u.sym.crda;
  FAR struct cryptodesc *crde = &sess->u.sym.crde;
  FAR struct cryptop *crp = &sess->u.sym.crp;
  uint32_t flags = data->header.flag;

  crp->crp_sid = sess->tid;
  crp->crp_desc = crda;
  crda->crd_next = crde;

  /* swcr_combined() takes the direction from the cipher descriptor. */

  if (data->header.opcode == VIRTIO_CRYPTO_AEAD_ENCRYPT)
    {
      crda->crd_flags = CRD_F_ENCRYPT;
      crde->crd_flags = CRD_F_ENCRYPT;
    }
  else
    {
      crda->crd_flags = 0;
      crde->crd_flags = 0;
    }

  if (flags & CRD_F_UPDATE_AAD)
    {
      return vhost_crypto_process_aead_update_aad(sess, data, req);
    }
  else if (flags & CRD_F_UPDATE)
    {
      return vhost_crypto_process_aead_update(sess, data, req);
    }
  else
    {
      return vhost_crypto_process_aead_final(sess, req);
    }
}

/****************************************************************************
 * Name: vhost_crypto_rsa_parse_private_key
 *
 * Description:
 *   Parse DER-encoded RSA private key into krp params.
 *
 ****************************************************************************/

static int vhost_crypto_rsa_parse_private_key(
    FAR struct vhost_crypto_session_s *sess,
    FAR unsigned char *keybuf,
    FAR struct cryptkop *krp)
{
  FAR unsigned char *pos = sess->u.ak.akkey;
  FAR unsigned char *end = pos + sess->u.ak.akkeylen;
  size_t buflen = 0;
  size_t tlen;
  int ver;
  int ret;

  while (pos < end && *pos == 0)
    {
      pos++;
    }

  ret = asn1_get_tag(&pos, end, &tlen,
                     ASN1_CONSTRUCTED | ASN1_SEQUENCE);
  if (ret != 0)
    {
      return ret;
    }

  ret = asn1_get_int(&pos, end, &ver);
  if (ret != 0)
    {
      return ret;
    }

  buflen += virtio_crypto_read_mpi(&pos, end,
              keybuf + buflen, &krp->krp_param[1]);
  buflen += virtio_crypto_read_mpi(&pos, end,
              keybuf + buflen, &krp->krp_param[2]);
  buflen += virtio_crypto_read_mpi(&pos, end,
              keybuf + buflen, &krp->krp_param[3]);
  buflen += virtio_crypto_read_mpi(&pos, end,
              keybuf + buflen, &krp->krp_param[4]);
  buflen += virtio_crypto_read_mpi(&pos, end,
              keybuf + buflen, &krp->krp_param[5]);
  buflen += virtio_crypto_read_mpi(&pos, end,
              keybuf + buflen, &krp->krp_param[6]);
  buflen += virtio_crypto_read_mpi(&pos, end,
              keybuf + buflen, &krp->krp_param[7]);
  buflen += virtio_crypto_read_mpi(&pos, end,
              keybuf + buflen, &krp->krp_param[8]);
  return 0;
}

/****************************************************************************
 * Name: vhost_crypto_rsa_parse_public_key
 *
 * Description:
 *   Parse DER-encoded RSA public key into krp params.
 *
 ****************************************************************************/

static int vhost_crypto_rsa_parse_public_key(
    FAR struct vhost_crypto_session_s *sess,
    FAR unsigned char *keybuf,
    FAR struct cryptkop *krp)
{
  FAR unsigned char *pos = sess->u.ak.akkey;
  FAR unsigned char *end = pos + sess->u.ak.akkeylen;
  size_t buflen = 0;
  size_t tlen;
  int ret;

  while (pos < end && *pos == 0)
    {
      pos++;
    }

  ret = asn1_get_tag(&pos, end, &tlen,
                     ASN1_CONSTRUCTED | ASN1_SEQUENCE);
  if (ret != 0)
    {
      return ret;
    }

  buflen += virtio_crypto_read_mpi(&pos, end,
              keybuf + buflen, &krp->krp_param[1]);
  buflen += virtio_crypto_read_mpi(&pos, end,
              keybuf + buflen, &krp->krp_param[2]);
  return 0;
}

/****************************************************************************
 * Name: vhost_crypto_process_rsa
 *
 * Description:
 *   Process RSA akcipher request (encrypt/decrypt/sign/verify).
 *
 ****************************************************************************/

static int vhost_crypto_process_rsa(
    FAR struct vhost_crypto_session_s *sess,
    FAR struct virtio_crypto_op_data_req_s *data,
    FAR struct vhost_crypto_req_s *req)
{
  FAR struct cryptkop *krp = &sess->u.ak.krp;
  FAR struct crparam *output;
  FAR struct crparam *input;
  FAR void *src_buf;
  FAR void *dst_buf;
  uint32_t src_len;
  uint32_t dst_len;
  uint32_t len;
  int ret;

  src_len = data->op_flf.akcipher.src_data_len;
  dst_len = data->op_flf.akcipher.dst_data_len;

  if (src_len > VHOST_CRYPTO_AKCIPHER_LEN_MAX ||
      dst_len > VHOST_CRYPTO_AKCIPHER_LEN_MAX)
    {
      return -EINVAL;
    }

  memset(krp->krp_param, 0, sizeof(krp->krp_param));

  if (krp->krp_keytype == CRYPTO_KEY_TYPE_PRIVATE)
    {
      ret = vhost_crypto_rsa_parse_private_key(sess, sess->u.ak.keybuf, krp);
    }
  else
    {
      ret = vhost_crypto_rsa_parse_public_key(sess, sess->u.ak.keybuf, krp);
    }

  if (ret != 0)
    {
      return ret;
    }

  /* Descriptors map shared memory directly, so pass the buffers as-is */

  src_buf = vhost_crypto_next_buf(req, &len);
  if (src_len > 0 && (src_buf == NULL || len < src_len))
    {
      return -EINVAL;
    }

  input = &krp->krp_param[0];
  input->crp_p = src_buf;
  input->crp_nbits = src_len * 8;

  /* Every opcode needs dst: output for non-VERIFY, signature for VERIFY */

  dst_buf = vhost_crypto_next_buf(req, &len);
  if (dst_len > 0 && (dst_buf == NULL || len < dst_len))
    {
      return -EINVAL;
    }

  switch (data->header.opcode)
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
        break;

      case VIRTIO_CRYPTO_AKCIPHER_ENCRYPT:
        krp->krp_op = CRK_MOD_EXP;
        krp->krp_iparams = 3;
        krp->krp_oparams = 1;
        krp->krp_optype = CRYPTO_OP_ENCRYPT;
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
            return -EINVAL;
          }

        krp->krp_iparams = 9;
        krp->krp_oparams = 1;
        krp->krp_optype = CRYPTO_OP_SIGN;
        break;

      case VIRTIO_CRYPTO_AKCIPHER_VERIFY:
        if (krp->krp_padding == CRYPTO_RSA_PKCS1_PADDING)
          {
            krp->krp_op = CRK_RSA_PKCS15_VERIFY;
            krp->krp_iparams = 5;
            krp->krp_oparams = 0;
          }
        else if (krp->krp_padding == CRYPTO_RSA_OAEP_PADDING)
          {
            krp->krp_op = CRK_RSA_PSS_VERIFY;
            krp->krp_iparams = 4;
            krp->krp_oparams = 0;
          }
        else
          {
            return -EINVAL;
          }

        krp->krp_optype = CRYPTO_OP_VERIFY;
        break;

      default:
        return -ENOTSUP;
    }

  output = &krp->krp_param[krp->krp_iparams + krp->krp_oparams - 1];
  output->crp_p = dst_buf;
  output->crp_nbits = dst_len * 8;

  ret = crypto_kinvoke(krp);
  if (ret == 0)
    {
      ret = krp->krp_status;
    }

  return ret;
}

/****************************************************************************
 * Name: vhost_crypto_ecdsa_get_op
 *
 * Description:
 *   Map a negotiated curve_id + operation type to the CRK_* op code.
 *   Only secp256r1 has a backing cryptodev implementation today; other
 *   curves are rejected instead of silently treated as P256.
 *
 ****************************************************************************/

static int vhost_crypto_ecdsa_get_op(uint32_t curve_id, uint32_t optype)
{
  if (curve_id != VIRTIO_CRYPTO_CURVE_NIST_P256)
    {
      return -ENOTSUP;
    }

  switch (optype)
    {
      case CRYPTO_OP_SIGN:
        return CRK_ECDSA_SECP256R1_SIGN;
      case CRYPTO_OP_VERIFY:
        return CRK_ECDSA_SECP256R1_VERIFY;
      case CRYPTO_OP_GENKEY:
        return CRK_ECDSA_SECP256R1_GENKEY;
      default:
        return -EINVAL;
    }
}

/****************************************************************************
 * Name: vhost_crypto_process_ecdsa
 *
 * Description:
 *   Process ECDSA akcipher request (sign/verify/genkey).
 *
 ****************************************************************************/

static int vhost_crypto_process_ecdsa(
    FAR struct vhost_crypto_session_s *sess,
    FAR struct virtio_crypto_op_data_req_s *data,
    FAR struct vhost_crypto_req_s *req)
{
  FAR struct cryptkop *krp = &sess->u.ak.krp;
  FAR struct crparam *dst;
  FAR struct crparam *d;
  FAR struct crparam *x;
  FAR struct crparam *y;
  FAR struct crparam *r;
  FAR struct crparam *s;
  FAR void *src_buf = NULL;
  FAR void *dst_buf;
  uint32_t curve_id = sess->u.ak.curve_id;
  uint32_t src_len;
  uint32_t dst_len;
  uint32_t keylen;
  uint32_t len;
  int curve_len;
  int krp_op;
  int ret;

  src_len = data->op_flf.akcipher.src_data_len;
  dst_len = data->op_flf.akcipher.dst_data_len;
  keylen = sess->u.ak.akkeylen;

  if (src_len > VHOST_CRYPTO_AKCIPHER_LEN_MAX ||
      dst_len > VHOST_CRYPTO_AKCIPHER_LEN_MAX)
    {
      return -EINVAL;
    }

  curve_len = virtio_crypto_curve_get_len(curve_id);
  if (curve_len < 0)
    {
      return curve_len;
    }

  memset(krp->krp_param, 0, sizeof(krp->krp_param));

  /* Descriptors map shared memory directly, so pass the buffers as-is */

  if (src_len > 0)
    {
      src_buf = vhost_crypto_next_buf(req, &len);
      if (src_buf == NULL || len < src_len)
        {
          return -EINVAL;
        }
    }

  switch (data->header.opcode)
    {
      case VIRTIO_CRYPTO_AKCIPHER_SIGN:
        d = &krp->krp_param[0];
        dst = &krp->krp_param[1];
        r = &krp->krp_param[2];
        s = &krp->krp_param[3];
        krp_op = vhost_crypto_ecdsa_get_op(curve_id, CRYPTO_OP_SIGN);
        if (krp_op < 0)
          {
            return krp_op;
          }

        krp->krp_op = krp_op;
        krp->krp_optype = CRYPTO_OP_SIGN;
        krp->krp_keytype = CRYPTO_KEY_TYPE_PRIVATE;
        krp->krp_iparams = 2;
        krp->krp_oparams = 2;

        /* Private key */

        d->crp_p = sess->u.ak.akkey;
        d->crp_nbits = keylen * 8;

        /* Hash to sign */

        dst->crp_p = src_buf;
        dst->crp_nbits = src_len * 8;

        /* Signature is r || s, each fixed curve_len bytes */

        if (dst_len != curve_len * 2)
          {
            return -EINVAL;
          }

        dst_buf = vhost_crypto_next_buf(req, &len);
        if (dst_buf == NULL || len < dst_len)
          {
            return -EINVAL;
          }

        r->crp_p = dst_buf;
        r->crp_nbits = curve_len * 8;
        s->crp_p = (FAR char *)dst_buf + curve_len;
        s->crp_nbits = curve_len * 8;
        break;

      case VIRTIO_CRYPTO_AKCIPHER_VERIFY:
        x = &krp->krp_param[0];
        y = &krp->krp_param[1];
        r = &krp->krp_param[3];
        s = &krp->krp_param[4];
        dst = &krp->krp_param[5];
        krp_op = vhost_crypto_ecdsa_get_op(curve_id, CRYPTO_OP_VERIFY);
        if (krp_op < 0)
          {
            return krp_op;
          }

        krp->krp_op = krp_op;
        krp->krp_optype = CRYPTO_OP_VERIFY;
        krp->krp_keytype = CRYPTO_KEY_TYPE_PUBLIC;
        krp->krp_iparams = 6;
        krp->krp_oparams = 0;

        /* Public key (x || y) */

        x->crp_p = sess->u.ak.akkey;
        x->crp_nbits = keylen / 2 * 8;
        y->crp_p = (FAR char *)sess->u.ak.akkey + keylen / 2;
        y->crp_nbits = keylen / 2 * 8;

        /* Signature is r || s, each fixed curve_len bytes */

        if (src_len != curve_len * 2)
          {
            return -EINVAL;
          }

        r->crp_p = src_buf;
        r->crp_nbits = curve_len * 8;
        s->crp_p = (FAR char *)src_buf + curve_len;
        s->crp_nbits = curve_len * 8;

        dst_buf = vhost_crypto_next_buf(req, &len);
        if (dst_len > 0 && (dst_buf == NULL || len < dst_len))
          {
            return -EINVAL;
          }

        dst->crp_p = dst_buf;
        dst->crp_nbits = dst_len * 8;
        break;

      case VIRTIO_CRYPTO_AKCIPHER_GENKEY:
        d = &krp->krp_param[0];
        x = &krp->krp_param[1];
        y = &krp->krp_param[2];
        krp_op = vhost_crypto_ecdsa_get_op(curve_id, CRYPTO_OP_GENKEY);
        if (krp_op < 0)
          {
            return krp_op;
          }

        krp->krp_op = krp_op;
        krp->krp_iparams = 0;
        krp->krp_oparams = 3;

        /* GENKEY produces d, x, y, each fixed curve_len bytes */

        if (dst_len != curve_len * 3)
          {
            return -EINVAL;
          }

        dst_buf = vhost_crypto_next_buf(req, &len);
        if (dst_buf == NULL || len < dst_len)
          {
            return -EINVAL;
          }

        d->crp_p = dst_buf;
        d->crp_nbits = curve_len * 8;
        x->crp_p = (FAR char *)dst_buf + curve_len;
        x->crp_nbits = curve_len * 8;
        y->crp_p = (FAR char *)dst_buf + curve_len * 2;
        y->crp_nbits = curve_len * 8;
        break;

      default:
        return -ENOTSUP;
    }

  ret = crypto_kinvoke(krp);
  if (ret == 0)
    {
      ret = krp->krp_status;
    }

  return ret;
}

/****************************************************************************
 * Name: vhost_crypto_process_ecdh
 *
 * Description:
 *   Process ECDH GENKEY (dst = d || x || y) and DERIVE
 *   (src = d || peer.x || peer.y, dst = shared secret).
 *
 ****************************************************************************/

static int vhost_crypto_process_ecdh(
    FAR struct vhost_crypto_session_s *sess,
    FAR struct virtio_crypto_op_data_req_s *data,
    FAR struct vhost_crypto_req_s *req)
{
  FAR struct cryptkop *krp = &sess->u.ak.krp;
  FAR uint8_t *src_buf;
  FAR uint8_t *dst_buf;
  uint32_t src_len = data->op_flf.akcipher.src_data_len;
  uint32_t dst_len = data->op_flf.akcipher.dst_data_len;
  uint32_t len;
  int curve_len;
  int ret;

  curve_len = virtio_crypto_curve_get_len(sess->u.ak.curve_id);
  if (curve_len < 0)
    {
      return curve_len;
    }

  if (sess->u.ak.curve_id != VIRTIO_CRYPTO_CURVE_NIST_P256)
    {
      return -ENOTSUP;
    }

  memset(krp->krp_param, 0, sizeof(krp->krp_param));

  switch (data->header.opcode)
    {
      case VIRTIO_CRYPTO_AKCIPHER_GENKEY:
        if (dst_len != curve_len * 3)
          {
            return -EINVAL;
          }

        dst_buf = vhost_crypto_next_buf(req, &len);
        if (dst_buf == NULL || len < dst_len)
          {
            return -EINVAL;
          }

        krp->krp_op = CRK_ECDH_GEN_PUBLIC;
        krp->krp_iparams = 0;
        krp->krp_oparams = 3;
        krp->krp_param[0].crp_p = (caddr_t)dst_buf;
        krp->krp_param[0].crp_nbits = curve_len * 8;
        krp->krp_param[1].crp_p = (caddr_t)(dst_buf + curve_len);
        krp->krp_param[1].crp_nbits = curve_len * 8;
        krp->krp_param[2].crp_p = (caddr_t)(dst_buf + curve_len * 2);
        krp->krp_param[2].crp_nbits = curve_len * 8;
        break;

      case VIRTIO_CRYPTO_AKCIPHER_DERIVE:
        if (src_len != curve_len * 3 || dst_len != curve_len)
          {
            return -EINVAL;
          }

        src_buf = vhost_crypto_next_buf(req, &len);
        if (src_buf == NULL || len < src_len)
          {
            return -EINVAL;
          }

        dst_buf = vhost_crypto_next_buf(req, &len);
        if (dst_buf == NULL || len < dst_len)
          {
            return -EINVAL;
          }

        krp->krp_op = CRK_ECDH_COMPUTE_SHARED;
        krp->krp_iparams = 3;
        krp->krp_oparams = 1;
        krp->krp_param[0].crp_p = (caddr_t)src_buf;
        krp->krp_param[0].crp_nbits = curve_len * 8;
        krp->krp_param[1].crp_p = (caddr_t)(src_buf + curve_len);
        krp->krp_param[1].crp_nbits = curve_len * 8;
        krp->krp_param[2].crp_p = (caddr_t)(src_buf + curve_len * 2);
        krp->krp_param[2].crp_nbits = curve_len * 8;
        krp->krp_param[3].crp_p = (caddr_t)dst_buf;
        krp->krp_param[3].crp_nbits = curve_len * 8;
        break;

      default:
        return -ENOTSUP;
    }

  ret = crypto_kinvoke(krp);
  if (ret == 0)
    {
      ret = krp->krp_status;
    }

  return ret;
}

/****************************************************************************
 * Name: vhost_crypto_process_akcipher
 *
 * Description:
 *   Dispatch akcipher request to RSA or ECDSA handler.
 *
 ****************************************************************************/

static int vhost_crypto_process_akcipher(
    FAR struct vhost_crypto_session_s *sess,
    FAR struct virtio_crypto_op_data_req_s *data,
    FAR struct vhost_crypto_req_s *req)
{
  switch (data->header.algo)
    {
      case VIRTIO_CRYPTO_AKCIPHER_RSA:
        return vhost_crypto_process_rsa(sess, data, req);
      case VIRTIO_CRYPTO_AKCIPHER_ECDSA:
        return vhost_crypto_process_ecdsa(sess, data, req);
      case VIRTIO_CRYPTO_AKCIPHER_ECDH:
        return vhost_crypto_process_ecdh(sess, data, req);
      default:
        return -ENOTSUP;
    }
}

/****************************************************************************
 * Name: vhost_crypto_dataq_work
 *
 * Description:
 *   Process all pending requests on a data virtqueue.
 *
 ****************************************************************************/

static void vhost_crypto_dataq_work(
    FAR struct vhost_crypto_priv_s *priv, int qidx)
{
  FAR struct virtio_crypto_op_data_req_s *data;
  FAR struct vhost_crypto_session_s *sess;
  FAR struct virtio_crypto_inhdr_s *inhdr;
  struct vhost_crypto_req_s req;
  FAR struct virtqueue *vq;
  int service;
  int idx;
  int ret;

  vq = priv->hdev->vrings_info[qidx].vq;

  /* Disable notifications while draining; re-enable and recheck below. */

  virtqueue_disable_cb(vq);

again:
  for (; ; )
    {
      idx = vhost_get_vq_buffers(vq, req.vb, VHOST_CRYPTO_MAX_SEGS,
                                 &req.cnt);
      if (idx < 0)
        {
          break;
        }

      /* A chain without inhdr gives no way to report status; drop it. */

      if (req.cnt < 2)
        {
          virtqueue_add_consumed_buffer(vq, idx, 0);
          continue;
        }

      /* inhdr is always the last descriptor; process_* never consume it. */

      data = req.vb[0].buf;
      req.pos = 1;
      req.cnt--;

      sess = (FAR struct vhost_crypto_session_s *)
             (uintptr_t)data->header.session_id;
      service = (data->header.opcode >> 8) & 0xff;
      switch (service)
        {
          case VIRTIO_CRYPTO_SERVICE_CIPHER:
            ret = vhost_crypto_process_cipher(sess, data, &req);
            break;

          case VIRTIO_CRYPTO_SERVICE_HASH:
            ret = vhost_crypto_process_hash(sess, data, &req);
            break;

          case VIRTIO_CRYPTO_SERVICE_MAC:
            ret = vhost_crypto_process_mac(sess, data, &req);
            break;

          case VIRTIO_CRYPTO_SERVICE_AEAD:
            ret = vhost_crypto_process_aead(sess, data, &req);
            break;

          case VIRTIO_CRYPTO_SERVICE_AKCIPHER:
            ret = vhost_crypto_process_akcipher(sess, data, &req);
            break;

          default:
            ret = -ENOTSUP;
            break;
        }

      inhdr = req.vb[req.cnt].buf;
      switch (ret)
        {
          case 0:
            inhdr->status = VIRTIO_CRYPTO_OK;
            break;

          case -ENOTSUP:
            inhdr->status = VIRTIO_CRYPTO_NOTSUPP;
            break;

          case -EBADF:
            inhdr->status = VIRTIO_CRYPTO_INVSESS;
            break;

          default:
            inhdr->status = VIRTIO_CRYPTO_ERR;
            break;
        }

      virtqueue_add_consumed_buffer(vq, idx, req.vb[req.cnt].len);
    }

  virtqueue_kick(vq);

  /* Drain again if a request slipped in before re-enabling callbacks. */

  if (virtqueue_enable_cb(vq))
    {
      virtqueue_disable_cb(vq);
      goto again;
    }
}

/****************************************************************************
 * Name: vhost_crypto_work
 *
 * Description:
 *   Work handler that processes controlq and dataq requests.  Scheduled by
 *   the virtqueue callback via the high-priority work queue.
 *
 ****************************************************************************/

static void vhost_crypto_work(FAR void *arg)
{
  FAR struct vhost_crypto_priv_s *priv = arg;
  uint32_t i;

  /* Process controlq first */

  vhost_crypto_controlq_work(priv);

  /* Process all data queues */

  for (i = 0; i < priv->max_dataqueues; i++)
    {
      vhost_crypto_dataq_work(priv, i);
    }
}

/****************************************************************************
 * Name: vhost_crypto_callback
 *
 * Description:
 *   Virtqueue callback - schedule deferred work to process pending requests.
 *
 ****************************************************************************/

static void vhost_crypto_callback(FAR struct virtqueue *vq)
{
  FAR struct vhost_crypto_priv_s *priv = vq->vq_dev->priv;

  /* work_available() coalesces notifications while work is pending. */

  if (work_available(&priv->work))
    {
      work_queue(HPWORK, &priv->work, vhost_crypto_work, priv, 0);
    }
}

/****************************************************************************
 * Name: vhost_crypto_probe
 *
 * Description:
 *   Initialize the vhost crypto backend driver.
 *
 ****************************************************************************/

static int vhost_crypto_probe(FAR struct vhost_device *hdev)
{
  FAR struct vhost_crypto_priv_s *priv;
  FAR const char *vqnames[VHOST_CRYPTO_MAX_DATAQUEUES + 1];
  vq_callback callbacks[VHOST_CRYPTO_MAX_DATAQUEUES + 1];
  uint64_t features;
  uint32_t num_vqs;
  uint32_t i;
  int ret;

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      vhosterr("No enough memory\n");
      return -ENOMEM;
    }

  priv->hdev = hdev;
  hdev->priv = priv;

  /* Negotiate features: get-then-set, as in other vhost backends. */

  vhost_get_features(hdev, &features);
  vhost_set_features(hdev, features &
                     ((1ULL << VIRTIO_CRYPTO_F_REVISION_1) |
                      (1ULL << VIRTIO_CRYPTO_F_CIPHER_STATELESS_MODE) |
                      (1ULL << VIRTIO_CRYPTO_F_HASH_STATELESS_MODE) |
                      (1ULL << VIRTIO_CRYPTO_F_MAC_STATELESS_MODE) |
                      (1ULL << VIRTIO_CRYPTO_F_AEAD_STATELESS_MODE) |
                      (1ULL << VIRTIO_CRYPTO_F_AKCIPHER_STATELESS_MODE)));

  /* Read max_dataqueues from frontend config */

  vhost_read_config_member(hdev, struct virtio_crypto_config_s,
                           max_dataqueues, &priv->max_dataqueues);
  if (priv->max_dataqueues < 1)
    {
      priv->max_dataqueues = 1;
    }
  else if (priv->max_dataqueues > VHOST_CRYPTO_MAX_DATAQUEUES)
    {
      priv->max_dataqueues = VHOST_CRYPTO_MAX_DATAQUEUES;
    }

  /* max_cipher_key_len/max_auth_key_len are not written here -- they
   * are compile-time values baked into crypto_config in
   * drivers/rptun/rptun_ivshmem.c, read by the frontend before probe.
   */

  /* Total virtqueues = N dataq + 1 controlq */

  num_vqs = priv->max_dataqueues + 1;

  for (i = 0; i < priv->max_dataqueues; i++)
    {
      vqnames[i] = "vhost_crypto_dataq";
      callbacks[i] = vhost_crypto_callback;
    }

  vqnames[priv->max_dataqueues] = "vhost_crypto_ctrlq";
  callbacks[priv->max_dataqueues] = vhost_crypto_callback;

  ret = vhost_create_virtqueues(hdev, 0, num_vqs, vqnames,
                                callbacks, NULL);

  if (ret < 0)
    {
      vhosterr("vhost_create_virtqueues failed, ret=%d\n", ret);
      goto errout;
    }

  for (i = 0; i < num_vqs; i++)
    {
      virtqueue_enable_cb(hdev->vrings_info[i].vq);
    }

  return 0;

errout:
  kmm_free(priv);
  return ret;
}

/****************************************************************************
 * Name: vhost_crypto_remove
 *
 * Description:
 *   Remove the vhost crypto backend driver.
 *
 ****************************************************************************/

static void vhost_crypto_remove(FAR struct vhost_device *hdev)
{
  FAR struct vhost_crypto_priv_s *priv = hdev->priv;
  uint32_t i;

  /* Disable callbacks on all virtqueues */

  for (i = 0; i < priv->max_dataqueues + 1; i++)
    {
      virtqueue_disable_cb(hdev->vrings_info[i].vq);
    }

  work_cancel_sync(HPWORK, &priv->work);
  vhost_delete_virtqueues(hdev);
  kmm_free(priv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vhost_register_crypto_driver
 ****************************************************************************/

int vhost_register_crypto_driver(void)
{
  return vhost_register_driver(&g_vhost_crypto_driver);
}
