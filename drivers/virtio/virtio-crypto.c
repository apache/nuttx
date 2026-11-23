/****************************************************************************
 * drivers/virtio/virtio-crypto.c
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

#include <crypto/virtio_crypto.h>
#include <nuttx/spinlock.h>
#include <nuttx/virtio/virtio.h>
#include <nuttx/virtio/virtio-crypto.h>

#include "virtio-crypto.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VIRTIO_CRYPTO_RSA_KEYBITS_MAX  8192

/* Largest number of virtqueue slots a single request uses. Bounded by the
 * AEAD path: header, iv, src, aad, dst, inhdr.
 */

#define VIRTIO_CRYPTO_MAX_SEGS         8

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* A request built in two phases: declare (out/in) then map, so a bounced
 * buffer is never touched after it has been mapped.
 */

struct virtio_crypto_req_s
{
  FAR struct virtio_device *vdev;

  /* Declared buffers, out segments first then in segments. */

  FAR void            *buf[VIRTIO_CRYPTO_MAX_SEGS];
  enum virtio_dma_map  dir[VIRTIO_CRYPTO_MAX_SEGS];

  /* Mapped view handed to the virtqueue. */

  struct virtqueue_buf vb[VIRTIO_CRYPTO_MAX_SEGS];

  int n_out;      /* Declared out segments */
  int n_in;       /* Declared in segments */
  int n_mapped;   /* Segments currently mapped */
};

struct virtio_crypto_s
{
  FAR struct virtio_device *vdev;

  /* To protect the vq operations for the controlq and dataq */

  spinlock_t lock;

  /* Maximum number of data queues supported by the device */

  uint32_t max_data_queues;

  /* Number of queue currently used by the driver */

  uint32_t curr_queue;

  /* Specifies the services mask which the device supports,
   * see VIRTIO_CRYPTO_SERVICE_*
   */

  uint32_t crypto_services;

  /* Detailed algorithms mask */

  uint32_t cipher_algo_l;
  uint32_t cipher_algo_h;
  uint32_t hash_algo;
  uint32_t mac_algo_l;
  uint32_t mac_algo_h;
  uint32_t aead_algo;
  uint32_t akcipher_algo;

  /* Maximum length of cipher key */

  uint32_t max_cipher_key_len;

  /* Maximum length of authenticated key */

  uint32_t max_auth_key_len;

  /* Maximum size of per request */

  uint64_t max_size;

  uint8_t dev_id;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Crypto driver operation function */

static int virtio_crypto_newsession(FAR uint32_t *sid,
                                    FAR struct cryptoini *cri);
static int virtio_crypto_process(FAR struct cryptop *crp);
static int virtio_crypto_kprocess(FAR struct cryptkop *krp);
static int virtio_crypto_freesession(uint64_t tid);

static int virtio_crypto_fill_crtlq_request(FAR struct cryptoini *cri,
                        FAR struct virtio_crypto_op_ctrl_req_s *ctrl);
static int
virtio_crypto_send_recv(FAR struct virtio_crypto_s *vcrypto,
                        FAR struct virtqueue *vq,
                        FAR struct virtqueue_buf *vb,
                        int num_out, int num_in);
static int
virtio_crypto_process_endec(FAR struct virtio_crypto_s *vcrypto,
                            FAR struct cryptop *crp);
static int
virtio_crypto_process_hash_mac(FAR struct virtio_crypto_s *vcrypto,
                               FAR struct cryptop *crp);
static int
virtio_crypto_process_aead(FAR struct virtio_crypto_s *vcrypto,
                           FAR struct cryptop *crp);

static void virtio_crypto_notify(FAR struct virtqueue *vq);
static int  virtio_crypto_create_vqs(FAR struct virtio_crypto_s *vcrypto);

/* Virtio driver functions */

static int virtio_crypto_probe(FAR struct virtio_device *vdev);
static void virtio_crypto_remove(FAR struct virtio_device *vdev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct virtio_driver g_virtio_crypto_driver =
{
  VIRTIO_ID_CRYPTO,                                /* device id */
  virtio_crypto_probe,                             /* probe */
  virtio_crypto_remove,                            /* remove */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: virtio_crypto_req_init
 *
 * Description:
 *   Bind a request to a device. Must be called before declaring segments.
 *
 ****************************************************************************/

static void virtio_crypto_req_init(FAR struct virtio_crypto_req_s *req,
                                   FAR struct virtio_device *vdev)
{
  req->vdev    = vdev;
  req->n_out    = 0;
  req->n_in     = 0;
  req->n_mapped = 0;
}

/****************************************************************************
 * Name: virtio_crypto_req_out
 *
 * Description:
 *   Declare a buffer the device reads from. Must precede any in segment.
 *
 ****************************************************************************/

static void virtio_crypto_req_out(FAR struct virtio_crypto_req_s *req,
                                  FAR void *buf, size_t len)
{
  DEBUGASSERT(req->n_in == 0);
  DEBUGASSERT(req->n_out < VIRTIO_CRYPTO_MAX_SEGS);

  req->buf[req->n_out] = buf;
  req->dir[req->n_out] = VIRTIO_DMA_TO_DEVICE;
  req->vb[req->n_out].len = len;
  req->n_out++;
}

/****************************************************************************
 * Name: virtio_crypto_req_in
 *
 * Description:
 *   Declare a buffer the device writes to.
 *
 ****************************************************************************/

static void virtio_crypto_req_in(FAR struct virtio_crypto_req_s *req,
                                 FAR void *buf, size_t len)
{
  int i = req->n_out + req->n_in;

  DEBUGASSERT(i < VIRTIO_CRYPTO_MAX_SEGS);

  req->buf[i] = buf;
  req->dir[i] = VIRTIO_DMA_FROM_DEVICE;
  req->vb[i].len = len;
  req->n_in++;
}

/****************************************************************************
 * Name: virtio_crypto_req_unmap
 *
 * Description:
 *   Release mappings from virtio_crypto_req_map(); must run before the
 *   results are examined, since bouncing also happens here.
 *
 ****************************************************************************/

static void virtio_crypto_req_unmap(FAR struct virtio_crypto_req_s *req)
{
  int i;

  for (i = req->n_mapped - 1; i >= 0; i--)
    {
      virtio_unmap_buf(req->vdev, req->buf[i], req->vb[i].len,
                       req->dir[i], req->vb[i].buf);
    }

  req->n_mapped = 0;
  req->n_out    = 0;
  req->n_in     = 0;
}

/****************************************************************************
 * Name: virtio_crypto_req_map
 *
 * Description:
 *   Map every declared segment, rolling back on failure.
 *
 ****************************************************************************/

static int virtio_crypto_req_map(FAR struct virtio_crypto_req_s *req)
{
  int total = req->n_out + req->n_in;
  int ret;
  int i;

  for (i = 0; i < total; i++)
    {
      ret = virtio_map_buf(req->vdev, req->buf[i], req->vb[i].len,
                           req->dir[i], &req->vb[i].buf);
      if (ret < 0)
        {
          req->n_mapped = i;
          virtio_crypto_req_unmap(req);
          return ret;
        }
    }

  req->n_mapped = total;
  return OK;
}

static int
virtio_crypto_send_recv(FAR struct virtio_crypto_s *vcrypto,
                        FAR struct virtqueue *vq,
                        FAR struct virtqueue_buf *vb,
                        int num_out, int num_in)
{
  irqstate_t flags;
  sem_t sem;
  int ret;

  nxsem_init(&sem, 0, 0);

  flags = spin_lock_irqsave(&vcrypto->lock);
  ret = virtqueue_add_buffer(vq, vb, num_out, num_in, &sem);
  if (ret >= 0)
    {
      virtqueue_kick(vq);
    }

  spin_unlock_irqrestore(&vcrypto->lock, flags);

  if (ret < 0)
    {
      nxsem_destroy(&sem);
      return ret;
    }

  nxsem_wait_uninterruptible(&sem);
  nxsem_destroy(&sem);
  return OK;
}

static int virtio_crypto_fill_crtlq_request(
             FAR struct cryptoini *cri,
             FAR struct virtio_crypto_op_ctrl_req_s *ctrl)
{
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

  memset(ctrl, 0, sizeof(struct virtio_crypto_op_ctrl_req_s));

  /* Pad ctrl header for creating a new session */

  ctrl->header.algo = algo;

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
        ctrl->op_flf.aead.tag_len = virtio_crypto_get_hash_result_len(algo);
        ctrl->op_flf.aead.op = cri->cri_op == COP_ENCRYPT ?
                VIRTIO_CRYPTO_OP_ENCRYPT : VIRTIO_CRYPTO_OP_DECRYPT;
        break;
      default:
        return -EINVAL;
    }

  return OK;
}

static int virtio_crypto_newsession(FAR uint32_t *sid,
                                    FAR struct cryptoini *cri)
{
  struct virtio_crypto_create_session_input_s input;
  struct virtio_crypto_op_ctrl_req_s ctrl;
  struct virtio_crypto_req_s req;
  FAR struct virtio_crypto_s *vcrypto;
  FAR struct virtio_device *vdev;
  FAR struct virtqueue *vq;
  uint32_t keylen;
  int ret;

  vcrypto = (struct virtio_crypto_s *)crypto_driver_get_priv(*sid);
  if (vcrypto == NULL)
    {
      vrterr("Virtio Crypto newsession: Invalid context\n");
      return -EINVAL;
    }

  vdev = vcrypto->vdev;
  vq = vdev->vrings_info[vcrypto->max_data_queues].vq;

  keylen = cri->cri_klen / 8;

  /* Change Nuttx Crypto para into Virtio para */

  ret = virtio_crypto_fill_crtlq_request(cri, &ctrl);
  if (ret < 0)
    {
      vrterr("Virtio Crypto: Invalid parameters in newsession\n");
      return ret;
    }

  input.status = VIRTIO_CRYPTO_ERR;

  virtio_crypto_req_init(&req, vdev);
  virtio_crypto_req_out(&req, &ctrl, sizeof(ctrl));
  if (keylen > 0)
    {
      virtio_crypto_req_out(&req, cri->cri_key, keylen);
    }

  virtio_crypto_req_in(&req, &input, sizeof(input));

  ret = virtio_crypto_req_map(&req);
  if (ret < 0)
    {
      return ret;
    }

  ret = virtio_crypto_send_recv(vcrypto, vq, req.vb, req.n_out, req.n_in);

  virtio_crypto_req_unmap(&req);
  if (ret < 0)
    {
      return ret;
    }

  if (input.status != VIRTIO_CRYPTO_OK)
    {
      vrterr("virtio_crypto_s: Create session failed status: %u\n",
                                                     input.status);
      return virtio_crypto_get_errcode(input.status);
    }

  *sid = input.session_id;
  return OK;
}

static int virtio_crypto_freesession(uint64_t tid)
{
  struct virtio_crypto_destroy_session_input_s input;
  struct virtio_crypto_op_ctrl_req_s ctrl;
  struct virtio_crypto_req_s req;
  FAR struct virtio_crypto_s *vcrypto;
  FAR struct virtio_device *vdev;
  FAR struct virtqueue *vq;
  uint32_t hid = (uint32_t)(tid >> 32);
  int ret;

  vcrypto = crypto_driver_get_priv(hid);
  if (vcrypto == NULL)
    {
      vrterr("Virtio Crypto freesession: Invalid context\n");
      return -EINVAL;
    }

  vdev = vcrypto->vdev;
  vq = vdev->vrings_info[vcrypto->max_data_queues].vq;

  memset(&ctrl, 0, sizeof(ctrl));
  input.status = VIRTIO_CRYPTO_ERR;

  /* Pad ctrl header */

  ctrl.header.opcode = VIRTIO_CRYPTO_CIPHER_DESTROY_SESSION;
  ctrl.op_flf.destroy.session_id = tid & 0xffffffff;

  virtio_crypto_req_init(&req, vdev);
  virtio_crypto_req_out(&req, &ctrl, sizeof(ctrl));
  virtio_crypto_req_in(&req, &input, sizeof(input));

  ret = virtio_crypto_req_map(&req);
  if (ret < 0)
    {
      return ret;
    }

  ret = virtio_crypto_send_recv(vcrypto, vq, req.vb, req.n_out, req.n_in);

  virtio_crypto_req_unmap(&req);
  if (ret == 0 && input.status != VIRTIO_CRYPTO_OK)
    {
      vrterr("virtio_crypto_s: Close session failed status: %u,"
             "session_id: 0x%lu\n", input.status,
             ctrl.op_flf.destroy.session_id);

      ret = virtio_crypto_get_errcode(input.status);
    }

  return ret;
}

static int
virtio_crypto_process_endec(FAR struct virtio_crypto_s *vcrypto,
                            FAR struct cryptop *crp)
{
  FAR struct virtio_device *vdev = vcrypto->vdev;
  FAR struct virtqueue *vq = vdev->vrings_info[0].vq;
  struct virtio_crypto_op_data_req_s data;
  struct virtio_crypto_inhdr_s inhdr;
  struct virtio_crypto_req_s req;
  uint32_t data_size = crp->crp_ilen;
  bool need_iv = !(crp->crp_desc->crd_flags & CRD_F_UPDATE) &&
                 crp->crp_ivlen > 0;
  int ret;

  memset(&data, 0, sizeof(data));
  data.header.algo =
    virtio_crypto_get_alg(crp->crp_desc->crd_alg);
  data.header.session_id = crp->crp_sid & 0xffffffff;
  data.header.opcode =
    crp->crp_desc->crd_flags & CRD_F_ENCRYPT ?
    VIRTIO_CRYPTO_CIPHER_ENCRYPT : VIRTIO_CRYPTO_CIPHER_DECRYPT;
  data.op_flf.sym.op_type = VIRTIO_CRYPTO_SYM_OP_CIPHER;
  data.op_flf.sym.op_type_flf.cipher.iv_len =
    need_iv ? crp->crp_ivlen : 0;
  data.op_flf.sym.op_type_flf.cipher.src_data_len = data_size;
  data.op_flf.sym.op_type_flf.cipher.dst_data_len = data_size;
  data.header.flag = crp->crp_desc->crd_flags & CRD_F_UPDATE;
  inhdr.status = VIRTIO_CRYPTO_ERR;

  virtio_crypto_req_init(&req, vdev);
  virtio_crypto_req_out(&req, &data, sizeof(data));
  if (need_iv)
    {
      virtio_crypto_req_out(&req, crp->crp_iv, crp->crp_ivlen);
    }

  virtio_crypto_req_out(&req, crp->crp_buf, data_size);
  virtio_crypto_req_in(&req, crp->crp_dst, data_size);
  virtio_crypto_req_in(&req, &inhdr, sizeof(inhdr));

  ret = virtio_crypto_req_map(&req);
  if (ret < 0)
    {
      return ret;
    }

  ret = virtio_crypto_send_recv(vcrypto, vq, req.vb, req.n_out, req.n_in);

  virtio_crypto_req_unmap(&req);
  if (ret < 0)
    {
      return ret;
    }

  if (inhdr.status != VIRTIO_CRYPTO_OK)
    {
      ret = virtio_crypto_get_errcode(inhdr.status);
    }

  return ret;
}

static int
virtio_crypto_process_hash_mac(FAR struct virtio_crypto_s *vcrypto,
                               FAR struct cryptop *crp)
{
  FAR struct virtio_device *vdev = vcrypto->vdev;
  FAR struct virtqueue *vq = vdev->vrings_info[0].vq;
  struct virtio_crypto_op_data_req_s data;
  struct virtio_crypto_inhdr_s inhdr;
  struct virtio_crypto_req_s req;
  uint32_t flags = crp->crp_desc->crd_flags;
  uint32_t data_size = crp->crp_ilen;
  size_t result_len = 0;
  int service;
  int ret;

  service = virtio_crypto_get_service(crp->crp_desc->crd_alg);
  if (!(flags & CRD_F_UPDATE))
    {
      result_len = crp->crp_olen;
    }

  memset(&data, 0, sizeof(data));
  data.header.algo =
    virtio_crypto_get_alg(crp->crp_desc->crd_alg);
  data.header.session_id = crp->crp_sid & 0xffffffff;
  data.header.opcode = service == VIRTIO_CRYPTO_SERVICE_MAC ?
    VIRTIO_CRYPTO_MAC : VIRTIO_CRYPTO_HASH;

  if (service == VIRTIO_CRYPTO_SERVICE_MAC)
    {
      data.op_flf.mac.hdr.src_data_len = data_size;
      data.op_flf.mac.hdr.hash_result_len = result_len;
      data.op_flf.mac.hdr.iterations = crp->crp_iter;
    }
  else
    {
      data.op_flf.hash.src_data_len = data_size;
      data.op_flf.hash.hash_result_len = result_len;
    }

  inhdr.status = VIRTIO_CRYPTO_ERR;

  virtio_crypto_req_init(&req, vdev);
  virtio_crypto_req_out(&req, &data, sizeof(data));
  if (data_size > 0)
    {
      virtio_crypto_req_out(&req, crp->crp_buf, data_size);
    }

  if (result_len > 0)
    {
      virtio_crypto_req_in(&req, crp->crp_mac, result_len);
    }

  virtio_crypto_req_in(&req, &inhdr, sizeof(inhdr));

  ret = virtio_crypto_req_map(&req);
  if (ret < 0)
    {
      return ret;
    }

  ret = virtio_crypto_send_recv(vcrypto, vq, req.vb, req.n_out, req.n_in);

  virtio_crypto_req_unmap(&req);
  if (ret < 0)
    {
      return ret;
    }

  if (inhdr.status != VIRTIO_CRYPTO_OK)
    {
      ret = virtio_crypto_get_errcode(inhdr.status);
    }

  return ret;
}

/* Segment layouts below must match vhost-crypto's AEAD stage handlers. */

static int
virtio_crypto_process_aead_update_aad(FAR struct virtio_crypto_s *vcrypto,
                                      FAR struct cryptop *crp)
{
  FAR struct virtqueue *vq = vcrypto->vdev->vrings_info[0].vq;
  FAR struct cryptodesc *crda = crp->crp_desc;
  FAR struct cryptodesc *crde = crda->crd_next;
  struct virtio_crypto_op_data_req_s data;
  struct virtio_crypto_inhdr_s inhdr;
  struct virtio_crypto_req_s req;
  int ret;

  memset(&data, 0, sizeof(data));
  data.header.algo = virtio_crypto_get_alg(crda->crd_alg);
  data.header.session_id = crp->crp_sid & 0xffffffff;
  data.header.opcode =
    (crde ? crde->crd_flags : crda->crd_flags) & CRD_F_ENCRYPT ?
    VIRTIO_CRYPTO_AEAD_ENCRYPT : VIRTIO_CRYPTO_AEAD_DECRYPT;
  data.header.flag = CRD_F_UPDATE_AAD;
  data.op_flf.aead.iv_len = crp->crp_ivlen;
  data.op_flf.aead.aad_len = crp->crp_aadlen;
  inhdr.status = VIRTIO_CRYPTO_ERR;

  /* [iv] src(empty) aad dst(empty) inhdr */

  virtio_crypto_req_init(&req, vcrypto->vdev);
  virtio_crypto_req_out(&req, &data, sizeof(data));
  if (crp->crp_ivlen > 0)
    {
      virtio_crypto_req_out(&req, crp->crp_iv, crp->crp_ivlen);
    }

  virtio_crypto_req_out(&req, NULL, 0);
  virtio_crypto_req_out(&req, crp->crp_aad, crp->crp_aadlen);
  virtio_crypto_req_in(&req, NULL, 0);
  virtio_crypto_req_in(&req, &inhdr, sizeof(inhdr));

  ret = virtio_crypto_req_map(&req);
  if (ret < 0)
    {
      return ret;
    }

  ret = virtio_crypto_send_recv(vcrypto, vq, req.vb, req.n_out, req.n_in);

  virtio_crypto_req_unmap(&req);
  if (ret < 0)
    {
      return ret;
    }

  if (inhdr.status != VIRTIO_CRYPTO_OK)
    {
      ret = virtio_crypto_get_errcode(inhdr.status);
    }

  return ret;
}

static int
virtio_crypto_process_aead_update(FAR struct virtio_crypto_s *vcrypto,
                                  FAR struct cryptop *crp)
{
  FAR struct virtqueue *vq = vcrypto->vdev->vrings_info[0].vq;
  FAR struct cryptodesc *crda = crp->crp_desc;
  FAR struct cryptodesc *crde = crda->crd_next;
  struct virtio_crypto_op_data_req_s data;
  struct virtio_crypto_inhdr_s inhdr;
  struct virtio_crypto_req_s req;
  int ret;

  if (crp->crp_dst == NULL)
    {
      return -EINVAL;
    }

  memset(&data, 0, sizeof(data));
  data.header.algo = virtio_crypto_get_alg(crda->crd_alg);
  data.header.session_id = crp->crp_sid & 0xffffffff;
  data.header.opcode =
    (crde ? crde->crd_flags : crda->crd_flags) & CRD_F_ENCRYPT ?
    VIRTIO_CRYPTO_AEAD_ENCRYPT : VIRTIO_CRYPTO_AEAD_DECRYPT;
  data.header.flag = CRD_F_UPDATE;
  data.op_flf.aead.src_data_len = crp->crp_ilen;
  data.op_flf.aead.dst_data_len = crp->crp_ilen;
  inhdr.status = VIRTIO_CRYPTO_ERR;

  /* src dst inhdr */

  virtio_crypto_req_init(&req, vcrypto->vdev);
  virtio_crypto_req_out(&req, &data, sizeof(data));
  virtio_crypto_req_out(&req, crp->crp_buf, crp->crp_ilen);
  virtio_crypto_req_in(&req, crp->crp_dst, crp->crp_ilen);
  virtio_crypto_req_in(&req, &inhdr, sizeof(inhdr));

  ret = virtio_crypto_req_map(&req);
  if (ret < 0)
    {
      return ret;
    }

  ret = virtio_crypto_send_recv(vcrypto, vq, req.vb, req.n_out, req.n_in);

  virtio_crypto_req_unmap(&req);
  if (ret < 0)
    {
      return ret;
    }

  if (inhdr.status != VIRTIO_CRYPTO_OK)
    {
      ret = virtio_crypto_get_errcode(inhdr.status);
    }

  return ret;
}

static int
virtio_crypto_process_aead_final(FAR struct virtio_crypto_s *vcrypto,
                                 FAR struct cryptop *crp)
{
  FAR struct virtqueue *vq = vcrypto->vdev->vrings_info[0].vq;
  FAR struct cryptodesc *crda = crp->crp_desc;
  FAR struct cryptodesc *crde = crda->crd_next;
  struct virtio_crypto_op_data_req_s data;
  struct virtio_crypto_inhdr_s inhdr;
  struct virtio_crypto_req_s req;
  int ret;

  if (crp->crp_mac == NULL)
    {
      return -EINVAL;
    }

  memset(&data, 0, sizeof(data));
  data.header.algo = virtio_crypto_get_alg(crda->crd_alg);
  data.header.session_id = crp->crp_sid & 0xffffffff;
  data.header.opcode =
    (crde ? crde->crd_flags : crda->crd_flags) & CRD_F_ENCRYPT ?
    VIRTIO_CRYPTO_AEAD_ENCRYPT : VIRTIO_CRYPTO_AEAD_DECRYPT;
  data.header.flag = 0;
  data.op_flf.aead.tag_len = crp->crp_olen;
  inhdr.status = VIRTIO_CRYPTO_ERR;

  /* src(empty) tag inhdr */

  virtio_crypto_req_init(&req, vcrypto->vdev);
  virtio_crypto_req_out(&req, &data, sizeof(data));
  virtio_crypto_req_out(&req, NULL, 0);
  virtio_crypto_req_in(&req, crp->crp_mac, crp->crp_olen);
  virtio_crypto_req_in(&req, &inhdr, sizeof(inhdr));

  ret = virtio_crypto_req_map(&req);
  if (ret < 0)
    {
      return ret;
    }

  ret = virtio_crypto_send_recv(vcrypto, vq, req.vb, req.n_out, req.n_in);

  virtio_crypto_req_unmap(&req);
  if (ret < 0)
    {
      return ret;
    }

  if (inhdr.status != VIRTIO_CRYPTO_OK)
    {
      ret = virtio_crypto_get_errcode(inhdr.status);
    }

  return ret;
}

static int
virtio_crypto_process_aead(FAR struct virtio_crypto_s *vcrypto,
                           FAR struct cryptop *crp)
{
  uint32_t flags = crp->crp_desc->crd_flags;

  if (flags & CRD_F_UPDATE_AAD)
    {
      return virtio_crypto_process_aead_update_aad(vcrypto, crp);
    }
  else if (flags & CRD_F_UPDATE)
    {
      return virtio_crypto_process_aead_update(vcrypto, crp);
    }
  else
    {
      return virtio_crypto_process_aead_final(vcrypto, crp);
    }
}

static int virtio_crypto_process(FAR struct cryptop *crp)
{
  FAR struct virtio_crypto_s *vcrypto;
  uint32_t hid = (uint32_t)(crp->crp_sid >> 32);
  int service;

  vcrypto = crypto_driver_get_priv(hid);
  if (vcrypto == NULL)
    {
      vrterr("Virtio Crypto process: Invalid context\n");
      return -EINVAL;
    }

  service = virtio_crypto_get_service(crp->crp_desc->crd_alg);
  switch (service)
    {
      case VIRTIO_CRYPTO_SERVICE_CIPHER:
        return virtio_crypto_process_endec(vcrypto, crp);
      case VIRTIO_CRYPTO_SERVICE_HASH:
      case VIRTIO_CRYPTO_SERVICE_MAC:
        return virtio_crypto_process_hash_mac(vcrypto, crp);
      case VIRTIO_CRYPTO_SERVICE_AEAD:
        return virtio_crypto_process_aead(vcrypto, crp);
      default:
        return -ENOTSUP;
    }
}

static int virtio_crypto_rsa_get_session(FAR struct virtio_crypto_s *vcrypto,
                                         FAR struct cryptkop *krp,
                                         FAR uint32_t *sid)
{
  struct virtio_crypto_create_session_input_s input;
  struct virtio_crypto_op_ctrl_req_s ctrl;
  struct virtio_crypto_req_s req;
  FAR struct virtio_device *vdev = vcrypto->vdev;
  FAR unsigned char *keybuf;
  FAR struct virtqueue *vq;
  FAR struct crparam *n;
  FAR struct crparam *e;
  FAR struct crparam *d;
  FAR struct crparam *p;
  FAR struct crparam *q;
  FAR struct crparam *dp;
  FAR struct crparam *dq;
  FAR struct crparam *u;
  uint32_t keybufsize;
  uint32_t keybits;
  ssize_t keylen;
  bool is_private;
  int ret;

  vq = vdev->vrings_info[vcrypto->max_data_queues].vq;

  is_private = krp->krp_keytype == CRYPTO_KEY_TYPE_PRIVATE;
  keybits = krp->krp_param[1].crp_nbits;
  if (keybits == 0 || keybits > VIRTIO_CRYPTO_RSA_KEYBITS_MAX)
    {
      return -EINVAL;
    }

  keybufsize = VIRTIO_CRYPTO_RSA_DER_MAX_BYTES(keybits, is_private);

  /* Private RSA-8192 DER is ~5.7 KB, too large for the stack. */

  keybuf = kmm_malloc(keybufsize);
  if (keybuf == NULL)
    {
      return -ENOMEM;
    }

  /* Pad ctrl header for creating a new session */

  memset(&ctrl, 0, sizeof(ctrl));
  ctrl.header.algo = virtio_crypto_asym_get_alg(krp->krp_op);
  ctrl.header.opcode = VIRTIO_CRYPTO_AKCIPHER_CREATE_SESSION;
  ctrl.op_flf.akcipher.algo = ctrl.header.algo;
  ctrl.op_flf.akcipher.key_type = is_private ?
                        VIRTIO_CRYPTO_AKCIPHER_KEY_TYPE_PRIVATE :
                        VIRTIO_CRYPTO_AKCIPHER_KEY_TYPE_PUBLIC;
  ctrl.op_flf.akcipher.algo_flf.rsa.padding_algo = krp->krp_padding;
  ctrl.op_flf.akcipher.algo_flf.rsa.hash_algo = krp->krp_hash;

  if (ctrl.op_flf.akcipher.key_type ==
                  VIRTIO_CRYPTO_AKCIPHER_KEY_TYPE_PRIVATE)
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
          (FAR unsigned char *)dp->crp_p, dp->crp_nbits / 8,
          (FAR unsigned char *)dq->crp_p, dq->crp_nbits / 8,
          (FAR unsigned char *)u->crp_p, u->crp_nbits / 8,
          keybuf, keybufsize);
    }
  else
    {
      n = &krp->krp_param[1];
      e = &krp->krp_param[2];
      keylen = virtio_crypto_write_rsa_pubkey_der(
                (FAR unsigned char *)n->crp_p, n->crp_nbits / 8,
                (FAR unsigned char *)e->crp_p, e->crp_nbits / 8,
                keybuf, keybufsize);
    }

  if (keylen < 0)
    {
      ret = keylen;
      goto out;
    }

  ctrl.op_flf.akcipher.key_len = keylen;

  input.status = VIRTIO_CRYPTO_ERR;

  virtio_crypto_req_init(&req, vdev);
  virtio_crypto_req_out(&req, &ctrl, sizeof(ctrl));

  /* keybufsize is worst-case; send only the tail actually filled in. */

  virtio_crypto_req_out(&req, keybuf + keybufsize - keylen, keylen);
  virtio_crypto_req_in(&req, &input, sizeof(input));

  ret = virtio_crypto_req_map(&req);
  if (ret < 0)
    {
      goto out;
    }

  ret = virtio_crypto_send_recv(vcrypto, vq, req.vb, req.n_out, req.n_in);

  virtio_crypto_req_unmap(&req);
  if (ret < 0)
    {
      goto out;
    }

  if (input.status != VIRTIO_CRYPTO_OK)
    {
      vrterr("virtio_crypto_s: Create session failed status: %u\n",
                                                     input.status);
      ret = virtio_crypto_get_errcode(input.status);
    }
  else
    {
      *sid = input.session_id;
      ret = OK;
    }

out:
  kmm_free(keybuf);
  return ret;
}

static int
virtio_crypto_ecdsa_get_session(FAR struct virtio_crypto_s *vcrypto,
                                FAR struct cryptkop *krp,
                                FAR uint32_t *sid)
{
  struct virtio_crypto_create_session_input_s input;
  struct virtio_crypto_op_ctrl_req_s ctrl;
  struct virtio_crypto_req_s req;
  FAR struct virtio_device *vdev = vcrypto->vdev;
  FAR struct virtqueue *vq;
  FAR uint8_t *keybuf = NULL;
  uint32_t keylen = 0;
  int32_t curve_id;
  int curve_len;
  int ret;

  vq = vdev->vrings_info[vcrypto->max_data_queues].vq;

  curve_id = virtio_crypto_ecdsa_get_curve(krp->krp_op);
  if (curve_id < 0)
    {
      return curve_id;
    }

  curve_len = virtio_crypto_curve_get_len(curve_id);
  if (curve_len < 0)
    {
      return curve_len;
    }

  switch (krp->krp_optype)
    {
      case CRYPTO_OP_SIGN:
        keylen = krp->krp_param[0].crp_nbits / 8;
        break;
      case CRYPTO_OP_VERIFY:
        keylen = krp->krp_param[0].crp_nbits / 8 +
                 krp->krp_param[1].crp_nbits / 8;
        break;
      case CRYPTO_OP_GENKEY:

        /* Key generation has no input key; session carries no key. */

        keylen = 0;
        break;
      default:
        return -EINVAL;
    }

  /* Bound keylen against the curve's field length before allocating. */

  if (keylen > (uint32_t)curve_len * 2)
    {
      return -EINVAL;
    }

  if (keylen > 0)
    {
      keybuf = kmm_malloc(keylen);
      if (keybuf == NULL)
        {
          return -ENOMEM;
        }
    }

  memset(&ctrl, 0, sizeof(ctrl));
  ctrl.header.algo = VIRTIO_CRYPTO_AKCIPHER_ECDSA;
  ctrl.header.opcode = VIRTIO_CRYPTO_AKCIPHER_CREATE_SESSION;
  ctrl.op_flf.akcipher.algo = VIRTIO_CRYPTO_AKCIPHER_ECDSA;
  ctrl.op_flf.akcipher.key_type =
    krp->krp_keytype == CRYPTO_KEY_TYPE_PRIVATE ?
    VIRTIO_CRYPTO_AKCIPHER_KEY_TYPE_PRIVATE :
    VIRTIO_CRYPTO_AKCIPHER_KEY_TYPE_PUBLIC;
  ctrl.op_flf.akcipher.algo_flf.ecdsa.curve_id = curve_id;
  ctrl.op_flf.akcipher.key_len = keylen;

  if (krp->krp_optype == CRYPTO_OP_SIGN)
    {
      memcpy(keybuf, krp->krp_param[0].crp_p, keylen);
    }
  else if (krp->krp_optype == CRYPTO_OP_VERIFY)
    {
      memcpy(keybuf, krp->krp_param[0].crp_p,
             krp->krp_param[0].crp_nbits / 8);
      memcpy(keybuf + krp->krp_param[0].crp_nbits / 8,
             krp->krp_param[1].crp_p,
             krp->krp_param[1].crp_nbits / 8);
    }

  input.status = VIRTIO_CRYPTO_ERR;

  virtio_crypto_req_init(&req, vdev);
  virtio_crypto_req_out(&req, &ctrl, sizeof(ctrl));
  if (keylen > 0)
    {
      virtio_crypto_req_out(&req, keybuf, keylen);
    }

  virtio_crypto_req_in(&req, &input, sizeof(input));

  ret = virtio_crypto_req_map(&req);
  if (ret < 0)
    {
      goto out;
    }

  ret = virtio_crypto_send_recv(vcrypto, vq, req.vb, req.n_out, req.n_in);

  virtio_crypto_req_unmap(&req);
  if (ret < 0)
    {
      goto out;
    }

  if (input.status != VIRTIO_CRYPTO_OK)
    {
      ret = virtio_crypto_get_errcode(input.status);
      goto out;
    }

  *sid = input.session_id;
  ret = OK;

out:
  kmm_free(keybuf);
  return ret;
}

static int
virtio_crypto_ecdsa_process(FAR struct virtio_crypto_s *vcrypto,
                            FAR struct cryptkop *krp,
                            uint32_t session_id)
{
  struct virtio_crypto_op_data_req_s data;
  struct virtio_crypto_inhdr_s inhdr;
  struct virtio_crypto_req_s req;
  FAR struct virtio_device *vdev = vcrypto->vdev;
  FAR struct virtqueue *vq = vdev->vrings_info[0].vq;
  FAR uint8_t *src = NULL;
  FAR uint8_t *dst = NULL;
  uint32_t src_len = 0;
  uint32_t dst_len = 0;
  uint32_t rlen;
  uint32_t slen;
  uint32_t mlen;
  uint32_t dlen;
  uint32_t xlen;
  uint32_t ylen;
  int32_t curve_id;
  int curve_len;
  int ret;

  curve_id = virtio_crypto_ecdsa_get_curve(krp->krp_op);
  if (curve_id < 0)
    {
      return curve_id;
    }

  curve_len = virtio_crypto_curve_get_len(curve_id);
  if (curve_len < 0)
    {
      return curve_len;
    }

  switch (krp->krp_optype)
    {
      case CRYPTO_OP_SIGN:
        src_len = krp->krp_param[1].crp_nbits / 8;
        dst_len = krp->krp_param[2].crp_nbits / 8 +
                  krp->krp_param[3].crp_nbits / 8;
        break;
      case CRYPTO_OP_VERIFY:
        src_len = krp->krp_param[3].crp_nbits / 8 +
                  krp->krp_param[4].crp_nbits / 8;
        dst_len = krp->krp_param[5].crp_nbits / 8;
        break;
      case CRYPTO_OP_GENKEY:
        src_len = 0;
        dst_len = krp->krp_param[0].crp_nbits / 8 +
                  krp->krp_param[1].crp_nbits / 8 +
                  krp->krp_param[2].crp_nbits / 8;
        break;
      default:
        return -EINVAL;
    }

  /* Bound src_len/dst_len against the curve's field length before
   * allocating. A signature/shared-key request touches at most a
   * handful of curve-sized components (r||s, d||x||y, ...), so 4x
   * the field length is a generous but still meaningful upper bound.
   */

  if (src_len > (uint32_t)curve_len * 4 ||
      dst_len > (uint32_t)curve_len * 4)
    {
      return -EINVAL;
    }

  if (src_len > 0)
    {
      src = kmm_malloc(src_len);
      if (src == NULL)
        {
          return -ENOMEM;
        }
    }

  if (dst_len > 0)
    {
      dst = kmm_malloc(dst_len);
      if (dst == NULL)
        {
          ret = -ENOMEM;
          goto out;
        }
    }

  memset(&data, 0, sizeof(data));
  data.header.algo = VIRTIO_CRYPTO_AKCIPHER_ECDSA;
  data.header.session_id = session_id & 0xffffffff;
  data.header.flag = krp->krp_flags;
  inhdr.status = VIRTIO_CRYPTO_ERR;

  switch (krp->krp_optype)
    {
      case CRYPTO_OP_SIGN:
        data.header.opcode = VIRTIO_CRYPTO_AKCIPHER_SIGN;
        data.op_flf.akcipher.src_data_len = src_len;
        data.op_flf.akcipher.dst_data_len = dst_len;
        memcpy(src, krp->krp_param[1].crp_p, src_len);
        break;
      case CRYPTO_OP_VERIFY:
        rlen = krp->krp_param[3].crp_nbits / 8;
        slen = krp->krp_param[4].crp_nbits / 8;
        mlen = krp->krp_param[5].crp_nbits / 8;
        data.header.opcode = VIRTIO_CRYPTO_AKCIPHER_VERIFY;
        data.op_flf.akcipher.src_data_len = rlen + slen;
        data.op_flf.akcipher.dst_data_len = mlen;
        memcpy(src, krp->krp_param[3].crp_p, rlen);
        memcpy(src + rlen, krp->krp_param[4].crp_p, slen);
        memcpy(dst, krp->krp_param[5].crp_p, mlen);
        break;
      case CRYPTO_OP_GENKEY:
        data.header.opcode = VIRTIO_CRYPTO_AKCIPHER_GENKEY;
        data.op_flf.akcipher.src_data_len = 0;
        data.op_flf.akcipher.dst_data_len = dst_len;
        break;
      default:
        break;
    }

  virtio_crypto_req_init(&req, vdev);
  virtio_crypto_req_out(&req, &data, sizeof(data));
  if (src_len > 0)
    {
      virtio_crypto_req_out(&req, src, src_len);
    }

  if (dst_len > 0)
    {
      /* VERIFY feeds dst in; SIGN/GENKEY get their result written back. */

      if (krp->krp_optype == CRYPTO_OP_VERIFY)
        {
          virtio_crypto_req_out(&req, dst, dst_len);
        }
      else
        {
          virtio_crypto_req_in(&req, dst, dst_len);
        }
    }

  virtio_crypto_req_in(&req, &inhdr, sizeof(inhdr));

  ret = virtio_crypto_req_map(&req);
  if (ret < 0)
    {
      goto out;
    }

  ret = virtio_crypto_send_recv(vcrypto, vq, req.vb, req.n_out, req.n_in);

  virtio_crypto_req_unmap(&req);
  if (ret < 0)
    {
      goto out;
    }

  if (inhdr.status != VIRTIO_CRYPTO_OK)
    {
      ret = virtio_crypto_get_errcode(inhdr.status);
      goto out;
    }

  switch (krp->krp_optype)
    {
      case CRYPTO_OP_SIGN:
        rlen = krp->krp_param[2].crp_nbits / 8;
        slen = krp->krp_param[3].crp_nbits / 8;
        memcpy(krp->krp_param[2].crp_p, dst, rlen);
        memcpy(krp->krp_param[3].crp_p, dst + rlen, slen);
        break;
      case CRYPTO_OP_GENKEY:
        dlen = krp->krp_param[0].crp_nbits / 8;
        xlen = krp->krp_param[1].crp_nbits / 8;
        ylen = krp->krp_param[2].crp_nbits / 8;
        memcpy(krp->krp_param[0].crp_p, dst, dlen);
        memcpy(krp->krp_param[1].crp_p, dst + dlen, xlen);
        memcpy(krp->krp_param[2].crp_p, dst + dlen + xlen, ylen);
        break;
      default:
        break;
    }

  ret = OK;

out:
  kmm_free(src);
  kmm_free(dst);
  return ret;
}

static int
virtio_crypto_ecdh_get_session(FAR struct virtio_crypto_s *vcrypto,
                               FAR struct cryptkop *krp,
                               FAR uint32_t *sid)
{
  struct virtio_crypto_create_session_input_s input;
  struct virtio_crypto_op_ctrl_req_s ctrl;
  struct virtio_crypto_req_s req;
  FAR struct virtio_device *vdev = vcrypto->vdev;
  FAR struct virtqueue *vq;
  int32_t curve_id;
  int ret;

  vq = vdev->vrings_info[vcrypto->max_data_queues].vq;

  curve_id = virtio_crypto_ecdsa_get_curve(krp->krp_op);
  if (curve_id < 0)
    {
      return curve_id;
    }

  /* ECDH keys travel with each request; the session only fixes the curve */

  memset(&ctrl, 0, sizeof(ctrl));
  ctrl.header.algo = VIRTIO_CRYPTO_AKCIPHER_ECDH;
  ctrl.header.opcode = VIRTIO_CRYPTO_AKCIPHER_CREATE_SESSION;
  ctrl.op_flf.akcipher.algo = VIRTIO_CRYPTO_AKCIPHER_ECDH;
  ctrl.op_flf.akcipher.key_type = VIRTIO_CRYPTO_AKCIPHER_KEY_TYPE_PRIVATE;
  ctrl.op_flf.akcipher.algo_flf.ecdsa.curve_id = curve_id;
  input.status = VIRTIO_CRYPTO_ERR;

  virtio_crypto_req_init(&req, vdev);
  virtio_crypto_req_out(&req, &ctrl, sizeof(ctrl));
  virtio_crypto_req_in(&req, &input, sizeof(input));

  ret = virtio_crypto_req_map(&req);
  if (ret < 0)
    {
      return ret;
    }

  ret = virtio_crypto_send_recv(vcrypto, vq, req.vb, req.n_out, req.n_in);

  virtio_crypto_req_unmap(&req);
  if (ret < 0)
    {
      return ret;
    }

  if (input.status != VIRTIO_CRYPTO_OK)
    {
      return virtio_crypto_get_errcode(input.status);
    }

  *sid = input.session_id;
  return OK;
}

static int
virtio_crypto_ecdh_process(FAR struct virtio_crypto_s *vcrypto,
                           FAR struct cryptkop *krp,
                           uint32_t session_id)
{
  struct virtio_crypto_op_data_req_s data;
  struct virtio_crypto_inhdr_s inhdr;
  struct virtio_crypto_req_s req;
  FAR struct virtio_device *vdev = vcrypto->vdev;
  FAR struct virtqueue *vq = vdev->vrings_info[0].vq;
  FAR struct crparam *d = &krp->krp_param[0];
  FAR struct crparam *x = &krp->krp_param[1];
  FAR struct crparam *y = &krp->krp_param[2];
  FAR uint8_t *buf;
  uint32_t dlen = d->crp_nbits / 8;
  uint32_t xlen = x->crp_nbits / 8;
  uint32_t ylen = y->crp_nbits / 8;
  uint32_t klen = 0;
  int curve_len;
  int ret;

  curve_len = virtio_crypto_curve_get_len(
                virtio_crypto_ecdsa_get_curve(krp->krp_op));
  if (curve_len < 0)
    {
      return curve_len;
    }

  if (krp->krp_op == CRK_ECDH_COMPUTE_SHARED)
    {
      klen = krp->krp_param[3].crp_nbits / 8;
      if (klen != curve_len)
        {
          return -EINVAL;
        }
    }

  if (dlen != curve_len || xlen != curve_len || ylen != curve_len)
    {
      return -EINVAL;
    }

  /* d || x || y: GEN_PUBLIC output, COMPUTE_SHARED input */

  buf = kmm_malloc(dlen + xlen + ylen);
  if (buf == NULL)
    {
      return -ENOMEM;
    }

  memset(&data, 0, sizeof(data));
  data.header.algo = VIRTIO_CRYPTO_AKCIPHER_ECDH;
  data.header.session_id = session_id & 0xffffffff;
  data.header.flag = krp->krp_flags;
  inhdr.status = VIRTIO_CRYPTO_ERR;

  virtio_crypto_req_init(&req, vdev);
  virtio_crypto_req_out(&req, &data, sizeof(data));

  switch (krp->krp_op)
    {
      case CRK_ECDH_GEN_PUBLIC:
        data.header.opcode = VIRTIO_CRYPTO_AKCIPHER_GENKEY;
        data.op_flf.akcipher.dst_data_len = dlen + xlen + ylen;
        virtio_crypto_req_in(&req, buf, dlen + xlen + ylen);
        break;
      case CRK_ECDH_COMPUTE_SHARED:
        memcpy(buf, d->crp_p, dlen);
        memcpy(buf + dlen, x->crp_p, xlen);
        memcpy(buf + dlen + xlen, y->crp_p, ylen);
        data.header.opcode = VIRTIO_CRYPTO_AKCIPHER_DERIVE;
        data.op_flf.akcipher.src_data_len = dlen + xlen + ylen;
        data.op_flf.akcipher.dst_data_len = klen;
        virtio_crypto_req_out(&req, buf, dlen + xlen + ylen);
        virtio_crypto_req_in(&req, krp->krp_param[3].crp_p, klen);
        break;
      default:
        ret = -EINVAL;
        goto out;
    }

  virtio_crypto_req_in(&req, &inhdr, sizeof(inhdr));

  ret = virtio_crypto_req_map(&req);
  if (ret < 0)
    {
      goto out;
    }

  ret = virtio_crypto_send_recv(vcrypto, vq, req.vb, req.n_out, req.n_in);

  virtio_crypto_req_unmap(&req);
  if (ret < 0)
    {
      goto out;
    }

  if (inhdr.status != VIRTIO_CRYPTO_OK)
    {
      ret = virtio_crypto_get_errcode(inhdr.status);
      goto out;
    }

  if (krp->krp_op == CRK_ECDH_GEN_PUBLIC)
    {
      memcpy(d->crp_p, buf, dlen);
      memcpy(x->crp_p, buf + dlen, xlen);
      memcpy(y->crp_p, buf + dlen + xlen, ylen);
    }

out:
  kmm_free(buf);
  return ret;
}

static int
virtio_crypto_asym_get_session(FAR struct virtio_crypto_s *vcrypto,
                               FAR struct cryptkop *krp,
                               FAR uint32_t *sid)
{
  switch (virtio_crypto_asym_get_alg(krp->krp_op))
    {
      case VIRTIO_CRYPTO_AKCIPHER_RSA:
        return virtio_crypto_rsa_get_session(vcrypto, krp, sid);
      case VIRTIO_CRYPTO_AKCIPHER_ECDSA:
        return virtio_crypto_ecdsa_get_session(vcrypto, krp, sid);
      case VIRTIO_CRYPTO_AKCIPHER_ECDH:
        return virtio_crypto_ecdh_get_session(vcrypto, krp, sid);
    }

  return -EINVAL;
}

static void
virtio_crypto_asym_free_session(FAR struct virtio_crypto_s *vcrypto,
                                FAR struct cryptkop *krp,
                                uint32_t session_id)
{
  FAR struct virtio_device *vdev = vcrypto->vdev;
  FAR struct virtqueue *vq;
  struct virtio_crypto_destroy_session_input_s input;
  struct virtio_crypto_op_ctrl_req_s ctrl;
  struct virtio_crypto_req_s req;
  int ret;

  vq = vdev->vrings_info[vcrypto->max_data_queues].vq;

  memset(&ctrl, 0, sizeof(ctrl));
  ctrl.header.opcode = VIRTIO_CRYPTO_AKCIPHER_DESTROY_SESSION;
  ctrl.op_flf.destroy.session_id = session_id;
  input.status = VIRTIO_CRYPTO_ERR;

  virtio_crypto_req_init(&req, vdev);
  virtio_crypto_req_out(&req, &ctrl, sizeof(ctrl));
  virtio_crypto_req_in(&req, &input, sizeof(input));

  if (virtio_crypto_req_map(&req) < 0)
    {
      return;
    }

  ret = virtio_crypto_send_recv(vcrypto, vq, req.vb, req.n_out, req.n_in);

  virtio_crypto_req_unmap(&req);
  if (ret == 0 && input.status != VIRTIO_CRYPTO_OK)
    {
      vrterr("virtio_crypto_s: Close session failed status: %u,"
             "session_id: 0x%lu\n", input.status,
             ctrl.op_flf.destroy.session_id);
    }
}

static int virtio_crypto_rsa_process(FAR struct virtio_crypto_s *vcrypto,
                                     FAR struct cryptkop *krp,
                                     uint32_t session_id)
{
  FAR struct virtio_device *vdev = vcrypto->vdev;
  FAR struct virtqueue *vq = vdev->vrings_info[0].vq;
  FAR struct crparam *src_param;
  struct virtio_crypto_op_data_req_s data;
  struct virtio_crypto_inhdr_s inhdr;
  struct virtio_crypto_req_s req;
  FAR struct crparam *dst_param;
  int iparams = krp->krp_iparams;
  int oparams = krp->krp_oparams;
  uint32_t src_len;
  uint32_t dst_len;
  int ret;

  src_param = &krp->krp_param[0];
  dst_param = &krp->krp_param[iparams + oparams - 1];
  src_len = src_param->crp_nbits / 8;
  dst_len = dst_param->crp_nbits / 8;

  memset(&data, 0, sizeof(data));
  data.header.session_id = session_id;
  data.header.algo = virtio_crypto_asym_get_alg(krp->krp_op);

  switch (krp->krp_optype)
    {
      case CRYPTO_OP_DECRYPT:
        data.header.opcode = VIRTIO_CRYPTO_AKCIPHER_DECRYPT;
        break;
      case CRYPTO_OP_ENCRYPT:
        data.header.opcode = VIRTIO_CRYPTO_AKCIPHER_ENCRYPT;
        break;
      case CRYPTO_OP_SIGN:
        data.header.opcode = VIRTIO_CRYPTO_AKCIPHER_SIGN;
        break;
      case CRYPTO_OP_VERIFY:
        data.header.opcode = VIRTIO_CRYPTO_AKCIPHER_VERIFY;
        break;
      default:
        return -EINVAL;
    }

  data.op_flf.akcipher.src_data_len = src_len;
  data.op_flf.akcipher.dst_data_len = dst_len;
  inhdr.status = VIRTIO_CRYPTO_ERR;

  virtio_crypto_req_init(&req, vdev);
  virtio_crypto_req_out(&req, &data, sizeof(data));
  virtio_crypto_req_out(&req, src_param->crp_p, src_len);
  if (krp->krp_optype == CRYPTO_OP_VERIFY)
    {
      virtio_crypto_req_out(&req, dst_param->crp_p, dst_len);
    }
  else
    {
      virtio_crypto_req_in(&req, dst_param->crp_p, dst_len);
    }

  virtio_crypto_req_in(&req, &inhdr, sizeof(inhdr));

  ret = virtio_crypto_req_map(&req);
  if (ret < 0)
    {
      return ret;
    }

  ret = virtio_crypto_send_recv(vcrypto, vq, req.vb, req.n_out, req.n_in);

  virtio_crypto_req_unmap(&req);

  if (ret == 0 && inhdr.status != VIRTIO_CRYPTO_OK)
    {
      vrterr("virtio_crypto_s: Akcipher process failed status: %u,"
             "session_id: 0x%" PRIx32 "\n", inhdr.status, session_id);
      ret = virtio_crypto_get_errcode(inhdr.status);
    }

  return ret;
}

static int virtio_crypto_asym_process(FAR struct virtio_crypto_s *vcrypto,
                                      FAR struct cryptkop *krp,
                                      uint32_t session_id)
{
  switch (virtio_crypto_asym_get_alg(krp->krp_op))
    {
      case VIRTIO_CRYPTO_AKCIPHER_RSA:
        return virtio_crypto_rsa_process(vcrypto, krp, session_id);
      case VIRTIO_CRYPTO_AKCIPHER_ECDSA:
        return virtio_crypto_ecdsa_process(vcrypto, krp, session_id);
      case VIRTIO_CRYPTO_AKCIPHER_ECDH:
        return virtio_crypto_ecdh_process(vcrypto, krp, session_id);
    }

  return -EINVAL;
}

static int virtio_crypto_kprocess(FAR struct cryptkop *krp)
{
  FAR struct virtio_crypto_s *vcrypto;
  uint32_t hid;
  uint32_t sid = 0;
  int ret;

  if (krp == NULL)
    {
      vrterr("Virtio Crypto kprocess: Invalid parameters\n");
      return -EINVAL;
    }

  hid = (uint32_t)krp->krp_hid;
  vcrypto = crypto_driver_get_priv(hid);
  if (vcrypto == NULL)
    {
      vrterr("Virtio Crypto kprocess: Invalid context\n");
      return -EINVAL;
    }

  /* create session */

  ret = virtio_crypto_asym_get_session(vcrypto, krp, &sid);
  if (ret < 0)
    {
      return ret;
    }

  /* process akcipher data */

  ret = virtio_crypto_asym_process(vcrypto, krp, sid);
  if (ret < 0)
    {
      vrterr("Virtio Crypto kprocess failed, ret = %d\n", ret);
    }

  /* free session */

  virtio_crypto_asym_free_session(vcrypto, krp, sid);
  return ret;
}

static void virtio_crypto_notify(FAR struct virtqueue *vq)
{
  FAR struct virtio_crypto_s *vcrypto = vq->vq_dev->priv;
  FAR sem_t *sem;

  for (; ; )
    {
      sem = virtqueue_get_buffer_lock(vq, 0, NULL, &vcrypto->lock);
      if (sem == NULL)
        {
          break;
        }

      nxsem_post(sem);
    }
}

static int virtio_crypto_create_vqs(struct virtio_crypto_s *vcrypto)
{
  FAR struct virtio_device *vdev = vcrypto->vdev;
  FAR const char *vqnames[vcrypto->max_data_queues + 1];
  vq_callback callback[vcrypto->max_data_queues + 1];
  uint32_t i;
  int ret;

  /* We expect 1 data virtqueue, followed by
   * possible N-1 data queues used in multiqueue mode,
   * followed by control vq.
   */

  /* Allocate/initialize parameters for data virtqueues */

  for (i = 0; i < vcrypto->max_data_queues; i++)
    {
      vqnames[i]  = "virtio_crypto_dataq";
      callback[i] = virtio_crypto_notify;
    }

  /* Parameters for control virtqueue */

  vqnames[vcrypto->max_data_queues] = "virtio_crypto_ctrlq";
  callback[vcrypto->max_data_queues] = virtio_crypto_notify;

  ret = virtio_create_virtqueues(
          vdev, 0, vcrypto->max_data_queues + 1, vqnames, callback, NULL);
  if (ret < 0)
    {
      vrterr("virtio_device_create_virtqueue failed, ret=%d\n", ret);
    }

  return ret;
}

static int virtio_crypto_probe(FAR struct virtio_device *vdev)
{
  FAR struct virtio_crypto_s *vcrypto;
  int algs[CRYPTO_ALGORITHM_MAX + 1];
  int kalgs[CRK_ALGORITHM_MAX + 1];
  int ret;
  int i;

  vcrypto = kmm_zalloc(sizeof(struct virtio_crypto_s));
  if (vcrypto == NULL)
    {
      vrterr("Virtio Crypto: Probe failed to allocate memory\n");
      return -ENOMEM;
    }

  virtio_read_config_member(vdev, struct virtio_crypto_config_s,
      max_dataqueues, &vcrypto->max_data_queues);
  if (vcrypto->max_data_queues < 1)
    {
      vcrypto->max_data_queues = 1;
    }

  virtio_read_config_member(vdev, struct virtio_crypto_config_s,
      max_cipher_key_len, &vcrypto->max_cipher_key_len);
  virtio_read_config_member(vdev, struct virtio_crypto_config_s,
      max_auth_key_len, &vcrypto->max_auth_key_len);
  virtio_read_config_member(vdev, struct virtio_crypto_config_s,
      max_size, &vcrypto->max_size);
  virtio_read_config_member(vdev, struct virtio_crypto_config_s,
      crypto_services, &vcrypto->crypto_services);
  virtio_read_config_member(vdev, struct virtio_crypto_config_s,
      cipher_algo_l, &vcrypto->cipher_algo_l);
  virtio_read_config_member(vdev, struct virtio_crypto_config_s,
      cipher_algo_h, &vcrypto->cipher_algo_h);
  virtio_read_config_member(vdev, struct virtio_crypto_config_s,
      hash_algo, &vcrypto->hash_algo);
  virtio_read_config_member(vdev, struct virtio_crypto_config_s,
      mac_algo_l, &vcrypto->mac_algo_l);
  virtio_read_config_member(vdev, struct virtio_crypto_config_s,
      mac_algo_h, &vcrypto->mac_algo_h);
  if (vcrypto->crypto_services & (1 << VIRTIO_CRYPTO_SERVICE_AKCIPHER))
    {
      virtio_read_config_member(vdev, struct virtio_crypto_config_s,
          akcipher_algo, &vcrypto->akcipher_algo);
    }

  vcrypto->vdev = vdev;
  vdev->priv = vcrypto;

  /* Use single data queue as default */

  vcrypto->curr_queue = 1;

  vrtinfo("max_queues: %x, max_cipher_key_len: %u,\
           max_auth_key_len: %u, max_size 0x%lx\n",
           vcrypto->max_data_queues, vcrypto->max_cipher_key_len,
           vcrypto->max_auth_key_len, vcrypto->max_size);

  /* Call openamp api to initialize the virtio device */

  virtio_set_status(vdev, VIRTIO_CONFIG_STATUS_DRIVER);
  virtio_negotiate_features(vdev,
                           (1UL << VIRTIO_CRYPTO_F_REVISION_1) |
                           (1UL << VIRTIO_CRYPTO_F_CIPHER_STATELESS_MODE) |
                           (1UL << VIRTIO_CRYPTO_F_HASH_STATELESS_MODE) |
                           (1UL << VIRTIO_CRYPTO_F_MAC_STATELESS_MODE) |
                           (1UL << VIRTIO_CRYPTO_F_AEAD_STATELESS_MODE) |
                           (1UL << VIRTIO_CRYPTO_F_AKCIPHER_STATELESS_MODE),
                           NULL);
  virtio_set_status(vdev, VIRTIO_CONFIG_FEATURES_OK);

  ret = virtio_crypto_create_vqs(vcrypto);
  if (ret < 0)
    {
      vrterr("virtio_device_create_virtqueue failed, ret=%d\n", ret);
      goto err;
    }

  virtio_set_status(vdev, VIRTIO_CONFIG_STATUS_DRIVER_OK);

  spin_lock_init(&vcrypto->lock);

  /* Register NuttX driver */

  vcrypto->dev_id = crypto_get_driverid(0);

  crypto_driver_set_priv(vcrypto->dev_id, vcrypto);

  /* All algorithms are supported by default, handled by the host. */

  for (i = 0; i <= CRYPTO_ALGORITHM_MAX; i++)
    {
      algs[i] = CRYPTO_ALG_FLAG_SUPPORTED;
    }

  ret = crypto_register(vcrypto->dev_id, algs, virtio_crypto_newsession,
                        virtio_crypto_freesession, virtio_crypto_process);
  if (ret < 0)
    {
      goto err;
    }

  for (i = 0; i <= CRK_ALGORITHM_MAX; i++)
    {
      kalgs[i] = CRYPTO_ALG_FLAG_SUPPORTED;
    }

  ret = crypto_kregister(vcrypto->dev_id, kalgs, virtio_crypto_kprocess);
  if (ret < 0)
    {
      for (i = 0; i <= CRYPTO_ALGORITHM_MAX; i++)
        {
          crypto_unregister(vcrypto->dev_id, i);
        }

      goto err;
    }

  return ret;

err:
  virtio_delete_virtqueues(vdev);
  virtio_reset_device(vdev);
  kmm_free(vcrypto);
  return ret;
}

static void virtio_crypto_remove(FAR struct virtio_device *vdev)
{
  FAR struct virtio_crypto_s *vcrypto = vdev->priv;
  int i;

  for (i = 0; i <= CRYPTO_ALGORITHM_MAX; i++)
    {
      crypto_unregister(vcrypto->dev_id, i);
    }

  virtio_delete_virtqueues(vdev);
  virtio_reset_device(vdev);
  kmm_free(vcrypto);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: virtio_register_crypto_driver
 ****************************************************************************/

int virtio_register_crypto_driver(void)
{
  return virtio_register_driver(&g_virtio_crypto_driver);
}
