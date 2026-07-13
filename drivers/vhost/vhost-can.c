/****************************************************************************
 * drivers/vhost/vhost-can.c
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

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>
#include <poll.h>
#include <string.h>
#include <sys/endian.h>
#include <sys/param.h>
#include <sys/socket.h>

#include <nuttx/can.h>
#include <nuttx/kmalloc.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/spinlock.h>
#include <nuttx/vhost/vhost.h>
#include <nuttx/virtio/virtio.h>
#include <nuttx/virtio/virtio-can.h>
#include <nuttx/wqueue.h>

#include "vhost-can.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VHOST_CAN_TXQ    0
#define VHOST_CAN_RXQ    1
#define VHOST_CAN_CTLQ   2
#define VHOST_CAN_NUM_VQ 3

/* Longest guest descriptor chain popped in one request. */

#define VHOST_CAN_NREQ   8

/* Wire header length, and the payload offset shared by both SocketCAN
 * frame layouts.
 */

#define VHOST_CAN_HDRLEN offsetof(struct virtio_can_rx_s, sdu)
#define VHOST_CAN_DOFF   offsetof(struct canfd_frame, data)

static_assert(offsetof(struct can_frame, data) == VHOST_CAN_DOFF,
              "SocketCAN frame layouts must share the payload offset");
static_assert(offsetof(struct can_frame, can_dlc) ==
              offsetof(struct canfd_frame, len),
              "SocketCAN frame layouts must share the length offset");

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct vhost_can_priv_s
{
  FAR struct vhost_device *hdev;
  FAR struct net_driver_s *dev;
  struct socket            sock;
  struct pollfd            pfd;
  struct work_s            txwork;
  struct work_s            ctlwork;
  struct work_s            rxwork;
  spinlock_t               vqlock[VHOST_CAN_NUM_VQ];

  /* Guest RX buffer held between drains; rxhead is -1 when none. */

  struct virtqueue_buf     rxvb[VHOST_CAN_NREQ];
  size_t                   rxvbcnt;
  int                      rxhead;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  vhost_can_probe(FAR struct vhost_device *hdev);
static void vhost_can_remove(FAR struct vhost_device *hdev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct vhost_driver g_vhost_can_driver =
{
  LIST_INITIAL_VALUE(g_vhost_can_driver.node),
  VIRTIO_ID_CAN,    /* Device id */
  vhost_can_probe,  /* Probe */
  vhost_can_remove, /* Remove */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vhost_can_stream_read
 *
 * Description:
 *   Copy bytes out of a descriptor chain treated as one byte stream; the
 *   guest may split a message anywhere.
 *
 ****************************************************************************/

static inline_function size_t
vhost_can_stream_read(FAR struct virtqueue_buf *vb, size_t n,
                      size_t off, FAR void *dst, size_t len)
{
  FAR uint8_t *out = dst;
  size_t done = 0;
  size_t chunk;
  size_t i;

  for (i = 0; i < n && done < len; i++)
    {
      if (off >= vb[i].len)
        {
          off -= vb[i].len;
          continue;
        }

      chunk = vb[i].len - off;
      if (chunk > len - done)
        {
          chunk = len - done;
        }

      memcpy(out + done, (FAR uint8_t *)vb[i].buf + off, chunk);
      done += chunk;
      off = 0;
    }

  return done;
}

/****************************************************************************
 * Name: vhost_can_stream_write
 *
 * Description:
 *   Copy bytes into a descriptor chain treated as one byte stream; the
 *   guest may split a message anywhere.
 *
 ****************************************************************************/

static inline_function size_t
vhost_can_stream_write(FAR struct virtqueue_buf *vb, size_t n,
                       size_t off, FAR const void *src, size_t len)
{
  FAR const uint8_t *in = src;
  size_t done = 0;
  size_t chunk;
  size_t i;

  for (i = 0; i < n && done < len; i++)
    {
      if (off >= vb[i].len)
        {
          off -= vb[i].len;
          continue;
        }

      chunk = vb[i].len - off;
      if (chunk > len - done)
        {
          chunk = len - done;
        }

      memcpy((FAR uint8_t *)vb[i].buf + off, in + done, chunk);
      done += chunk;
      off = 0;
    }

  return done;
}

/****************************************************************************
 * Name: vhost_can_wire2frame
 *
 * Description:
 *   Decode a VirtIO CAN tx request stream into a SocketCAN frame.  A
 *   lone contiguous buffer with a full payload carries the frame built
 *   in place; otherwise the frame is laid over the tail of out so its
 *   payload coincides with the gathered sdu.  Returns the frame length
 *   and sets frame, or 0 for a request the device must reject: an
 *   unsupported frame type, an out of range can_id or sdu length, or a
 *   stopped controller.
 *
 ****************************************************************************/

static size_t vhost_can_wire2frame(FAR struct vhost_can_priv_s *priv,
                                   FAR struct virtio_can_tx_out_s *out,
                                   FAR struct virtqueue_buf *vb, size_t n,
                                   FAR void **frame)
{
  FAR struct canfd_frame *cfd;
  uint64_t features;
  uint32_t flags;
  uint32_t canid;
  uint16_t len;
  size_t framelen;
  size_t sdulen;

  if (!IFF_IS_UP(priv->dev->d_flags))
    {
      return 0;
    }

  if (vhost_can_stream_read(vb, n, 0, out,
                            offsetof(struct virtio_can_tx_out_s, sdu)) <
      offsetof(struct virtio_can_tx_out_s, sdu))
    {
      return 0;
    }

  if (le16toh(out->msg_type) != VIRTIO_CAN_TX)
    {
      return 0;
    }

  flags = le32toh(out->flags);
  canid = le32toh(out->can_id);
  len   = le16toh(out->length);

  /* The negotiated set lives in the shared resource table; the cached
   * vdev->features field is not maintained for the device role.
   */

  if (vhost_get_features(priv->hdev, &features) < 0)
    {
      return 0;
    }

  /* Reject a frame type that was not negotiated, and an out of range
   * can_id or sdu length.  The spec forbids scheduling any of these.
   */

  if ((features & (1ULL << ((flags & VIRTIO_CAN_FLAGS_FD) != 0 ?
                            VIRTIO_CAN_F_CAN_FD :
                            VIRTIO_CAN_F_CAN_CLASSIC))) == 0)
    {
      return 0;
    }

  if ((flags & VIRTIO_CAN_FLAGS_RTR) != 0 &&
      ((flags & VIRTIO_CAN_FLAGS_FD) != 0 ||
       (features & (1ULL << VIRTIO_CAN_F_RTR_FRAMES)) == 0))
    {
      return 0;
    }

  if (canid > ((flags & VIRTIO_CAN_FLAGS_EXTENDED) != 0 ?
               CAN_EFF_MASK : CAN_SFF_MASK))
    {
      return 0;
    }

  if (len > ((flags & VIRTIO_CAN_FLAGS_FD) != 0 ?
             CANFD_MAX_DLEN : CAN_MAX_DLEN))
    {
      return 0;
    }

  if (flags & VIRTIO_CAN_FLAGS_EXTENDED)
    {
      canid |= CAN_EFF_FLAG;
    }

  if (flags & VIRTIO_CAN_FLAGS_RTR)
    {
      canid |= CAN_RTR_FLAG;
    }

  framelen = (flags & VIRTIO_CAN_FLAGS_FD) != 0 ?
             sizeof(struct canfd_frame) : sizeof(struct can_frame);

  /* A lone aligned buffer holding the whole frame area takes the frame
   * in place, and the sdu is not copied at all; otherwise the sdu is
   * gathered into out and the frame laid over its tail.  Either way the
   * SocketCAN header lands on the parsed tail of the wire header, and
   * both frame layouts share its byte positions, so the FD layout
   * writes it for either.
   */

  cfd = (FAR struct canfd_frame *)((FAR uint8_t *)vb[0].buf +
                                   offsetof(struct virtio_can_tx_out_s,
                                            sdu) - VHOST_CAN_DOFF);
  if (n != 1 || ((uintptr_t)cfd & 3) != 0 ||
      vb[0].len < offsetof(struct virtio_can_tx_out_s, sdu) -
                  VHOST_CAN_DOFF + framelen)
    {
      sdulen = vhost_can_stream_read(vb, n,
                                     offsetof(struct virtio_can_tx_out_s,
                                              sdu), out->sdu, len);
      memset(out->sdu + sdulen, 0, framelen - VHOST_CAN_DOFF - sdulen);
      cfd = (FAR struct canfd_frame *)((FAR uint8_t *)out +
                                       offsetof(struct virtio_can_tx_out_s,
                                                sdu) - VHOST_CAN_DOFF);
    }

  cfd->can_id = canid;
  cfd->len    = len;
  cfd->flags  = 0;
  cfd->__res0 = 0;
  cfd->__res1 = 0;
  *frame = cfd;
  return framelen;
}

/****************************************************************************
 * Name: vhost_can_rxhdr
 *
 * Description:
 *   Fill a VirtIO CAN rx message header.
 *
 ****************************************************************************/

static void vhost_can_rxhdr(FAR struct virtio_can_rx_s *rx, uint32_t canid,
                            uint8_t dlen)
{
  uint32_t flags = dlen > CAN_MAX_DLEN ? VIRTIO_CAN_FLAGS_FD : 0;

  if (canid & CAN_EFF_FLAG)
    {
      flags |= VIRTIO_CAN_FLAGS_EXTENDED;
    }

  if (canid & CAN_RTR_FLAG)
    {
      flags |= VIRTIO_CAN_FLAGS_RTR;
    }

  rx->msg_type             = htole16(VIRTIO_CAN_RX);
  rx->length               = htole16(dlen);
  rx->reserved_classic_dlc = 0;
  rx->padding              = 0;
  rx->reserved_xl_priority = 0;
  rx->flags                = htole32(flags);
  rx->can_id               = htole32(canid & CAN_EFF_MASK);
}

/****************************************************************************
 * Name: vhost_can_frame2wire
 *
 * Description:
 *   Encode a SocketCAN frame into the writable rx stream.  Returns the
 *   bytes written, or 0 when the stream is too small.
 *
 ****************************************************************************/

static size_t vhost_can_frame2wire(FAR struct virtqueue_buf *vb, size_t n,
                                   FAR const void *data, size_t len)
{
  struct virtio_can_rx_s rx;
  FAR const uint8_t *sdu;
  size_t total = 0;
  uint32_t canid;
  uint8_t  dlen;
  size_t i;

  for (i = 0; i < n; i++)
    {
      total += vb[i].len;
    }

  if (total < VHOST_CAN_HDRLEN)
    {
      return 0;
    }

  if (len >= sizeof(struct canfd_frame))
    {
      FAR const struct canfd_frame *cfd = data;

      canid = cfd->can_id;
      dlen  = MIN(cfd->len, CANFD_MAX_DLEN);
      sdu   = cfd->data;
    }
  else
    {
      FAR const struct can_frame *cf = data;

      canid = cf->can_id;
      dlen  = MIN(cf->can_dlc, CAN_MAX_DLEN);
      sdu   = cf->data;
    }

  dlen = MIN(dlen, total - VHOST_CAN_HDRLEN);
  vhost_can_rxhdr(&rx, canid, dlen);
  vhost_can_stream_write(vb, n, 0, &rx, VHOST_CAN_HDRLEN);
  vhost_can_stream_write(vb, n, VHOST_CAN_HDRLEN, sdu, dlen);
  return VIRTIO_CAN_RX_SIZE(dlen);
}

/****************************************************************************
 * Name: vhost_can_getreq
 *
 * Description:
 *   Pop the next request chain from a guest virtqueue.  On empty the
 *   callback is re-armed and the queue checked once more so a racing
 *   buffer is not missed.  Returns the chain head, or negative when the
 *   queue is drained; reqlen receives the total chain length.
 *
 ****************************************************************************/

static int vhost_can_getreq(FAR struct virtqueue *vq, FAR spinlock_t *lock,
                            FAR struct virtqueue_buf *vb, size_t n,
                            FAR size_t *vbcnt, FAR size_t *reqlen)
{
  size_t len = 0;
  size_t i;
  int head;

  *vbcnt = 0;

  head = vhost_get_vq_buffers_lock(vq, vb, n, vbcnt, lock);
  if (head < 0 && virtqueue_enable_cb_lock(vq, lock) != 0)
    {
      head = vhost_get_vq_buffers_lock(vq, vb, n, vbcnt, lock);
      if (head >= 0)
        {
          virtqueue_disable_cb_lock(vq, lock);
        }
    }

  if (head >= 0)
    {
      for (i = 0; i < *vbcnt; i++)
        {
          len += vb[i].len;
        }
    }

  *reqlen = len;
  return head;
}

/****************************************************************************
 * Name: vhost_can_txq
 ****************************************************************************/

static void vhost_can_txq(FAR void *arg)
{
  FAR struct vhost_can_priv_s *priv = arg;
  FAR struct virtqueue *vq = priv->hdev->vrings_info[VHOST_CAN_TXQ].vq;
  FAR spinlock_t *lock = &priv->vqlock[VHOST_CAN_TXQ];
  bool kick = false;

  for (; ; )
    {
      FAR struct virtio_can_tx_in_s *in;
      struct virtqueue_buf vb[VHOST_CAN_NREQ];
      struct virtio_can_tx_out_s out aligned_data(4);
      FAR void *frame;
      size_t framelen;
      size_t vbcnt;
      size_t reqlen;
      int head;

      head = vhost_can_getreq(vq, lock, vb, nitems(vb), &vbcnt, &reqlen);
      if (head < 0)
        {
          break;
        }

      /* The device-writable result is the last descriptor.  A request the
       * device must reject is answered with NOT_OK and not transmitted; a
       * malformed one without a result descriptor is consumed as is.
       */

      if (vbcnt >= 2 && vb[vbcnt - 1].len >= sizeof(*in))
        {
          in = vb[vbcnt - 1].buf;
          framelen = vhost_can_wire2frame(priv, &out, vb, vbcnt - 1,
                                          &frame);
          if (framelen == 0)
            {
              in->result = VIRTIO_CAN_RESULT_NOT_OK;
            }
          else
            {
              in->result = psock_sendto(&priv->sock, frame, framelen, 0,
                                        NULL, 0) >= 0 ?
                           VIRTIO_CAN_RESULT_OK : VIRTIO_CAN_RESULT_NOT_OK;
            }
        }

      virtqueue_add_consumed_buffer_lock(vq, head, reqlen, lock);
      kick = true;
    }

  if (kick)
    {
      virtqueue_kick_lock(vq, lock);
    }
}

/****************************************************************************
 * Name: vhost_can_start
 *
 * Description:
 *   Start the controller: drop stale frames queued while stopped,
 *   register the socket poll that drives RX, and bring the bridged
 *   interface up.
 *
 ****************************************************************************/

static int vhost_can_start(FAR struct vhost_can_priv_s *priv)
{
  struct canfd_frame frame;
  int ret;

  if (IFF_IS_UP(priv->dev->d_flags))
    {
      return OK;
    }

  /* A stopped controller receives nothing. */

  while (psock_recvfrom(&priv->sock, &frame, sizeof(frame),
                        MSG_DONTWAIT, NULL, NULL) > 0)
    {
    }

  ret = psock_poll(&priv->sock, &priv->pfd, true);
  if (ret < 0)
    {
      return ret;
    }

  ret = netdev_ifup(priv->dev);
  if (ret < 0)
    {
      psock_poll(&priv->sock, &priv->pfd, false);
    }

  return ret;
}

/****************************************************************************
 * Name: vhost_can_stop
 *
 * Description:
 *   Stop the controller: unregister the socket poll and bring the
 *   bridged interface down.
 *
 ****************************************************************************/

static void vhost_can_stop(FAR struct vhost_can_priv_s *priv)
{
  psock_poll(&priv->sock, &priv->pfd, false);
  netdev_ifdown(priv->dev);
}

/****************************************************************************
 * Name: vhost_can_ctlq
 ****************************************************************************/

static void vhost_can_ctlq(FAR void *arg)
{
  FAR struct vhost_can_priv_s *priv = arg;
  FAR struct virtqueue *vq = priv->hdev->vrings_info[VHOST_CAN_CTLQ].vq;
  FAR spinlock_t *lock = &priv->vqlock[VHOST_CAN_CTLQ];
  bool kick = false;

  for (; ; )
    {
      FAR struct virtio_can_control_in_s *in;
      struct virtqueue_buf vb[VHOST_CAN_NREQ];
      size_t vbcnt;
      size_t reqlen;
      int head;

      head = vhost_can_getreq(vq, lock, vb, nitems(vb), &vbcnt, &reqlen);
      if (head < 0)
        {
          break;
        }

      if (vbcnt >= 2 && vb[vbcnt - 1].len >= sizeof(*in))
        {
          struct virtio_can_control_out_s out;

          in = vb[vbcnt - 1].buf;
          in->result = VIRTIO_CAN_RESULT_NOT_OK;

          if (vhost_can_stream_read(vb, vbcnt - 1, 0, &out, sizeof(out)) ==
              sizeof(out))
            {
              switch (le16toh(out.msg_type))
                {
                  case VIRTIO_CAN_SET_CTRL_MODE_START:
                    if (vhost_can_start(priv) >= 0)
                      {
                        in->result = VIRTIO_CAN_RESULT_OK;
                      }
                    break;

                  case VIRTIO_CAN_SET_CTRL_MODE_STOP:
                    vhost_can_stop(priv);
                    in->result = VIRTIO_CAN_RESULT_OK;
                    break;

                  default:
                    break;
                }
            }
        }

      virtqueue_add_consumed_buffer_lock(vq, head, reqlen, lock);
      kick = true;
    }

  if (kick)
    {
      virtqueue_kick_lock(vq, lock);
    }
}

/****************************************************************************
 * Name: vhost_can_txkick
 ****************************************************************************/

static void vhost_can_txkick(FAR struct virtqueue *vq)
{
  FAR struct vhost_can_priv_s *priv = vq->vq_dev->priv;

  /* Doorbells stay suppressed until the drain re-arms the callback. */

  virtqueue_disable_cb_lock(vq, &priv->vqlock[VHOST_CAN_TXQ]);

  if (work_available(&priv->txwork))
    {
      work_queue(HPWORK, &priv->txwork, vhost_can_txq, priv, 0);
    }
}

/****************************************************************************
 * Name: vhost_can_ctlkick
 ****************************************************************************/

static void vhost_can_ctlkick(FAR struct virtqueue *vq)
{
  FAR struct vhost_can_priv_s *priv = vq->vq_dev->priv;

  /* Doorbells stay suppressed until the drain re-arms the callback. */

  virtqueue_disable_cb_lock(vq, &priv->vqlock[VHOST_CAN_CTLQ]);

  if (work_available(&priv->ctlwork))
    {
      work_queue(HPWORK, &priv->ctlwork, vhost_can_ctlq, priv, 0);
    }
}

/****************************************************************************
 * Name: vhost_can_rxq
 *
 * Description:
 *   Drain the bus socket into the guest RX virtqueue.  A guest buffer is
 *   popped before a frame is read, so backpressure keeps the frames in
 *   the socket buffer.
 *
 ****************************************************************************/

static void vhost_can_rxq(FAR void *arg)
{
  FAR struct vhost_can_priv_s *priv = arg;
  FAR struct virtqueue *vq = priv->hdev->vrings_info[VHOST_CAN_RXQ].vq;
  FAR spinlock_t *lock = &priv->vqlock[VHOST_CAN_RXQ];
  struct canfd_frame buf;
  bool kick = false;
  ssize_t nread;

  priv->pfd.revents = 0;

  for (; ; )
    {
      FAR struct canfd_frame *frame = &buf;
      FAR uint8_t *p;
      uint8_t dlen;
      size_t reqlen;
      size_t used;

      if (priv->rxhead < 0)
        {
          priv->rxhead = vhost_can_getreq(vq, lock, priv->rxvb,
                                          nitems(priv->rxvb),
                                          &priv->rxvbcnt, &reqlen);
          if (priv->rxhead < 0)
            {
              break;
            }
        }

      /* A lone aligned buffer with room for the largest frame receives
       * it in place, positioned so the payload lands on the sdu; a
       * fragmented, short or unaligned chain bounces through buf.
       */

      p = (FAR uint8_t *)priv->rxvb[0].buf +
          VHOST_CAN_HDRLEN - VHOST_CAN_DOFF;
      if (priv->rxvbcnt == 1 && ((uintptr_t)p & 3) == 0 &&
          priv->rxvb[0].len >= VHOST_CAN_HDRLEN - VHOST_CAN_DOFF +
                               sizeof(buf))
        {
          frame = (FAR struct canfd_frame *)p;
        }

      nread = psock_recvfrom(&priv->sock, frame, sizeof(buf),
                             MSG_DONTWAIT, NULL, NULL);
      if (nread < (ssize_t)sizeof(struct can_frame))
        {
          break;
        }

      /* The wire format has no error frame encoding: drop them rather
       * than hand the guest a data frame with a mangled id.  The popped
       * buffer is kept for the next frame.
       */

      if ((frame->can_id & CAN_ERR_FLAG) != 0)
        {
          continue;
        }

      /* The popped buffer is exclusively owned here, so the encode runs
       * outside the lock.  The wire header overwrites the received
       * SocketCAN header, which is already parsed; a too-small bounced
       * chain is consumed with length 0, dropping the frame.
       */

      if (frame != &buf)
        {
          dlen = MIN(frame->len, nread >= (ssize_t)sizeof(buf) ?
                                 CANFD_MAX_DLEN : CAN_MAX_DLEN);
          vhost_can_rxhdr(priv->rxvb[0].buf, frame->can_id, dlen);
          used = VIRTIO_CAN_RX_SIZE(dlen);
        }
      else
        {
          used = vhost_can_frame2wire(priv->rxvb, priv->rxvbcnt,
                                      frame, nread);
        }

      virtqueue_add_consumed_buffer_lock(vq, priv->rxhead, used, lock);
      priv->rxhead = -1;
      kick = true;
    }

  if (kick)
    {
      virtqueue_kick_lock(vq, lock);
    }
}

/****************************************************************************
 * Name: vhost_can_rxready
 *
 * Description:
 *   Wake the RX drain.
 *
 ****************************************************************************/

static void vhost_can_rxready(FAR struct vhost_can_priv_s *priv)
{
  if (work_available(&priv->rxwork))
    {
      work_queue(HPWORK, &priv->rxwork, vhost_can_rxq, priv, 0);
    }
}

/****************************************************************************
 * Name: vhost_can_poll_cb
 ****************************************************************************/

static void vhost_can_poll_cb(FAR struct pollfd *pfds)
{
  vhost_can_rxready(pfds->arg);
}

/****************************************************************************
 * Name: vhost_can_rxkick
 ****************************************************************************/

static void vhost_can_rxkick(FAR struct virtqueue *vq)
{
  FAR struct vhost_can_priv_s *priv = vq->vq_dev->priv;

  /* Doorbells stay suppressed until the drain re-arms the callback. */

  virtqueue_disable_cb_lock(vq, &priv->vqlock[VHOST_CAN_RXQ]);
  vhost_can_rxready(priv);
}

/****************************************************************************
 * Name: vhost_can_probe
 ****************************************************************************/

static int vhost_can_probe(FAR struct vhost_device *hdev)
{
  FAR struct vhost_can_priv_s *priv;
  FAR const char *vqnames[VHOST_CAN_NUM_VQ];
  vq_callback callback[VHOST_CAN_NUM_VQ];
  char ifname[VIRTIO_CAN_IFNAME_SIZE];
  FAR struct net_driver_s *dev;
  struct sockaddr_can addr;
  int on = 1;
  int ret;
  int i;

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->hdev   = hdev;
  priv->rxhead = -1;
  hdev->priv   = priv;
  for (i = 0; i < VHOST_CAN_NUM_VQ; i++)
    {
      spin_lock_init(&priv->vqlock[i]);
    }

  /* The name comes from the shared config, so a peer that leaves it
   * unterminated cannot run off the end of it here.
   */

  vhost_read_config_member(hdev, struct vhost_can_config_s, ifname, &ifname);
  ifname[sizeof(ifname) - 1] = '\0';
  if (ifname[0] == '\0')
    {
      ret = -ENODEV;
      goto err_free;
    }

  ret = psock_socket(PF_CAN, SOCK_RAW, CAN_RAW, &priv->sock);
  if (ret < 0)
    {
      goto err_free;
    }

  dev = netdev_findbyname(ifname);
  if (dev == NULL)
    {
      ret = -ENODEV;
      goto err_close;
    }

  priv->dev = dev;
  psock_setsockopt(&priv->sock, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS,
                   &on, sizeof(on));
  psock_setsockopt(&priv->sock, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
                   &on, sizeof(on));

  memset(&addr, 0, sizeof(addr));
  addr.can_family  = AF_CAN;
  addr.can_ifindex = dev->d_ifindex;
  ret = psock_bind(&priv->sock, (FAR struct sockaddr *)&addr, sizeof(addr));
  if (ret < 0)
    {
      goto err_close;
    }

  vqnames[VHOST_CAN_TXQ]  = "virtio_can_tx";
  vqnames[VHOST_CAN_RXQ]  = "virtio_can_rx";
  vqnames[VHOST_CAN_CTLQ] = "virtio_can_ctl";
  callback[VHOST_CAN_TXQ]  = vhost_can_txkick;
  callback[VHOST_CAN_RXQ]  = vhost_can_rxkick;
  callback[VHOST_CAN_CTLQ] = vhost_can_ctlkick;

  /* The socket poll callback registered on controller start drives RX;
   * there is no RX thread.
   */

  priv->pfd.events = POLLIN;
  priv->pfd.arg    = priv;
  priv->pfd.cb     = vhost_can_poll_cb;

  ret = vhost_create_virtqueues(hdev, 0, VHOST_CAN_NUM_VQ, vqnames,
                                callback, NULL);
  if (ret < 0)
    {
      goto err_close;
    }

  /* The drain arms the queue callbacks and answers requests the guest
   * queued before the device was probed.
   */

  vhost_can_txq(priv);
  vhost_can_ctlq(priv);
  return OK;

err_close:
  psock_close(&priv->sock);
err_free:
  kmm_free(priv);
  return ret;
}

/****************************************************************************
 * Name: vhost_can_remove
 ****************************************************************************/

static void vhost_can_remove(FAR struct vhost_device *hdev)
{
  FAR struct vhost_can_priv_s *priv = hdev->priv;

  virtqueue_disable_cb(hdev->vrings_info[VHOST_CAN_TXQ].vq);
  virtqueue_disable_cb(hdev->vrings_info[VHOST_CAN_RXQ].vq);
  virtqueue_disable_cb(hdev->vrings_info[VHOST_CAN_CTLQ].vq);
  vhost_can_stop(priv);
  work_cancel_sync(HPWORK, &priv->txwork);
  work_cancel_sync(HPWORK, &priv->ctlwork);
  work_cancel_sync(HPWORK, &priv->rxwork);
  vhost_delete_virtqueues(hdev);
  psock_close(&priv->sock);
  kmm_free(priv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vhost_register_can_driver
 ****************************************************************************/

int vhost_register_can_driver(void)
{
  return vhost_register_driver(&g_vhost_can_driver);
}
