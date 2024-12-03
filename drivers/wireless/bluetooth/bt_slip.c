/****************************************************************************
 * drivers/wireless/bluetooth/bt_slip.c
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

#include <assert.h>
#include <debug.h>
#include <fcntl.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/param.h>
#include <sys/types.h>

#include <nuttx/wqueue.h>
#include <nuttx/crc16.h>
#include <nuttx/kmalloc.h>
#include <nuttx/net/bluetooth.h>
#include <nuttx/semaphore.h>

#include <nuttx/wireless/bluetooth/bt_slip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HCI_3WIRE_ACK_PKT 0x00
#define HCI_COMMAND_PKT 0x01
#define HCI_ACLDATA_PKT 0x02
#define HCI_SCODATA_PKT 0x03
#define HCI_EVENT_PKT 0x04
#define HCI_ISODATA_PKT 0x05
#define HCI_3WIRE_LINK_PKT 0x0f
#define HCI_VENDOR_PKT 0xff

#define SLIP_DELIMITER 0xc0
#define SLIP_ESC 0xdb
#define SLIP_ESC_DELIM 0xdc
#define SLIP_ESC_ESC 0xdd

#define BT_SLIP_HEADER_LEN 4
#define BT_SLIP_CHECKSUM_LEN 2
#define BT_SLIP_RTX_TIMEOUT MSEC2TICK (150) /* 150ms */

#define BT_SLIP_GET_SEQ(header) ((header)[0] & 0x07)
#define BT_SLIP_GET_ACK(header) (((header)[0] >> 3) & 0x07)
#define BT_SLIP_GET_CRC(header) (((header)[0] >> 6) & 0x01)
#define BT_SLIP_GET_PKT_TYPE(header) ((header)[1] & 0x0f)
#define BT_SLIP_GET_LEN(header) ((((header)[1] >> 4) & 0x0f) + ((header)[2] << 4))

#define BT_SLIP_SET_SEQ(header, seq) ((header)[0] |= (seq))
#define BT_SLIP_SET_ACK(header, ack) ((header)[0] |= (ack) << 3)
#define BT_SLIP_SET_DIC(header) ((header)[0] |= 0x40)
#define BT_SLIP_SET_RELIABLE(header) ((header)[0] |= (1 << 7))
#define BT_SLIP_SET_TYPE(header, type) ((header)[1] |= (type))
#define BT_SLIP_SET_LEN(header, len)                                               \
  (((header)[1] |= ((len) & 0x0f) << 4), ((header)[2] |= (len) >> 4))
#define BT_SLIP_SET_HDRCHECKSUM(header)                                            \
  ((header)[3] = ~((header[0] + header[1] + header[2]) & 0xff))
#define BT_SLIP_SEQ_INCR(seq) ((seq) = (((seq) + 1) & 0x07))

#define BT_SLIP_IS_RELIABLE(header) (((header)[0] >> 7) & 0x01)
#define BT_SLIP_IS_HDRVALIDED(header)                                              \
  (((header[0] + header[1] + header[2] + header[3]) & 0xff) == 0xff)

#define BT_SLIP_GET_TXWIN(payload) (payload[2] & 0x07)
#define BT_SLIP_IS_DIC(payload) ((payload[2] & 0x10) == 0x10)

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct unack_pool_s
{
  size_t start;
  size_t end;
  size_t size;

  uint8_t buf[CONFIG_BLUETOOTH_SLIP_TXWIN][CONFIG_BLUETOOTH_SLIP_TXBUFSIZE];
};

struct unack_frame_s
{
  enum bt_buf_type_e type;
  size_t pktlen;
  uint8_t data[1];
};

struct sliphci_s
{
  FAR struct bt_driver_s dev;
  FAR struct bt_driver_s *drv;

  mutex_t sliplock;
  mutex_t unacklock;
  sem_t sem;

  struct work_s retxworker;
  struct unack_pool_s unackpool;

  enum
  {
    BT_SLIP_UNINITIALIZED,
    BT_SLIP_INITIALIZED,
    BT_SLIP_ACTIVE,
  } linkstate;

  bool dipresent; /* Data integrity check */

  uint8_t rxack; /* Last ack number received */
  uint8_t txseq; /* Next seq number to send */
  uint8_t txack; /* Next ack number to send */
  uint8_t txwin; /* Sliding window size */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int bt_slip_open(FAR struct bt_driver_s *dev);

static int bt_slip_send(FAR struct bt_driver_s *dev, enum bt_buf_type_e type,
                        FAR void *data, size_t len);

static int bt_slip_ioctl(FAR struct bt_driver_s *dev, int cmd,
                         unsigned long arg);

static void bt_slip_close(FAR struct bt_driver_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t g_sync_req[] =
  {
    0x01, 0x7e
  };

static const uint8_t g_sync_rsp[] =
  {
    0x02, 0x7d
  };

static const uint8_t g_conf_req[] =
  {
    0x03, 0xfc, 0x10 | CONFIG_BLUETOOTH_SLIP_TXWIN
  };

static const uint8_t g_conf_rsp[] =
  {
    0x04, 0x7b
  };

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint8_t bt_slip_bit8_reverse(uint8_t byte)
{
  static const uint8_t table[256] =
    {
      0x00, 0x80, 0x40, 0xc0, 0x20, 0xa0, 0x60, 0xe0, 0x10, 0x90, 0x50, 0xd0,
      0x30, 0xb0, 0x70, 0xf0, 0x08, 0x88, 0x48, 0xc8, 0x28, 0xa8, 0x68, 0xe8,
      0x18, 0x98, 0x58, 0xd8, 0x38, 0xb8, 0x78, 0xf8, 0x04, 0x84, 0x44, 0xc4,
      0x24, 0xa4, 0x64, 0xe4, 0x14, 0x94, 0x54, 0xd4, 0x34, 0xb4, 0x74, 0xf4,
      0x0c, 0x8c, 0x4c, 0xcc, 0x2c, 0xac, 0x6c, 0xec, 0x1c, 0x9c, 0x5c, 0xdc,
      0x3c, 0xbc, 0x7c, 0xfc, 0x02, 0x82, 0x42, 0xc2, 0x22, 0xa2, 0x62, 0xe2,
      0x12, 0x92, 0x52, 0xd2, 0x32, 0xb2, 0x72, 0xf2, 0x0a, 0x8a, 0x4a, 0xca,
      0x2a, 0xaa, 0x6a, 0xea, 0x1a, 0x9a, 0x5a, 0xda, 0x3a, 0xba, 0x7a, 0xfa,
      0x06, 0x86, 0x46, 0xc6, 0x26, 0xa6, 0x66, 0xe6, 0x16, 0x96, 0x56, 0xd6,
      0x36, 0xb6, 0x76, 0xf6, 0x0e, 0x8e, 0x4e, 0xce, 0x2e, 0xae, 0x6e, 0xee,
      0x1e, 0x9e, 0x5e, 0xde, 0x3e, 0xbe, 0x7e, 0xfe, 0x01, 0x81, 0x41, 0xc1,
      0x21, 0xa1, 0x61, 0xe1, 0x11, 0x91, 0x51, 0xd1, 0x31, 0xb1, 0x71, 0xf1,
      0x09, 0x89, 0x49, 0xc9, 0x29, 0xa9, 0x69, 0xe9, 0x19, 0x99, 0x59, 0xd9,
      0x39, 0xb9, 0x79, 0xf9, 0x05, 0x85, 0x45, 0xc5, 0x25, 0xa5, 0x65, 0xe5,
      0x15, 0x95, 0x55, 0xd5, 0x35, 0xb5, 0x75, 0xf5, 0x0d, 0x8d, 0x4d, 0xcd,
      0x2d, 0xad, 0x6d, 0xed, 0x1d, 0x9d, 0x5d, 0xdd, 0x3d, 0xbd, 0x7d, 0xfd,
      0x03, 0x83, 0x43, 0xc3, 0x23, 0xa3, 0x63, 0xe3, 0x13, 0x93, 0x53, 0xd3,
      0x33, 0xb3, 0x73, 0xf3, 0x0b, 0x8b, 0x4b, 0xcb, 0x2b, 0xab, 0x6b, 0xeb,
      0x1b, 0x9b, 0x5b, 0xdb, 0x3b, 0xbb, 0x7b, 0xfb, 0x07, 0x87, 0x47, 0xc7,
      0x27, 0xa7, 0x67, 0xe7, 0x17, 0x97, 0x57, 0xd7, 0x37, 0xb7, 0x77, 0xf7,
      0x0f, 0x8f, 0x4f, 0xcf, 0x2f, 0xaf, 0x6f, 0xef, 0x1f, 0x9f, 0x5f, 0xdf,
      0x3f, 0xbf, 0x7f, 0xff
    };

  return table[byte];
}

static uint16_t bt_slip_bit16_reverse(uint16_t x)
{
  return (bt_slip_bit8_reverse(x & 0xff) << 8)
         | bt_slip_bit8_reverse(x >> 8);
}

static uint16_t bt_slip_get_crc(FAR uint8_t *packet)
{
  return packet[1] + (packet[0] << 8);
}

static void bt_slip_unack_init(FAR struct sliphci_s *priv)
{
  FAR struct unack_pool_s *pool = &priv->unackpool;

  nxmutex_lock(&priv->unacklock);
  pool->start = 0;
  pool->end = 0;
  pool->size = 0;
  nxmutex_unlock(&priv->unacklock);
}

static FAR void *bt_slip_unack_ctor(FAR struct sliphci_s *priv)
{
  FAR struct unack_pool_s *pool = &priv->unackpool;
  FAR void *buf;

  nxmutex_lock(&priv->unacklock);
  buf = pool->buf[pool->start++];
  pool->start %= CONFIG_BLUETOOTH_SLIP_TXWIN;
  pool->size++;
  nxmutex_unlock(&priv->unacklock);

  return buf;
}

static FAR void *bt_slip_unack_dtor(FAR struct sliphci_s *priv)
{
  FAR struct unack_pool_s *pool = &priv->unackpool;
  FAR void *buf;

  nxmutex_lock(&priv->unacklock);
  buf = pool->buf[pool->end++];
  pool->end %= CONFIG_BLUETOOTH_SLIP_TXWIN;
  pool->size--;
  nxmutex_unlock(&priv->unacklock);

  return buf;
}

static size_t bt_slip_unack_size(FAR struct sliphci_s *priv)
{
  FAR struct unack_pool_s *pool = &priv->unackpool;
  size_t size;

  nxmutex_lock(&priv->unacklock);
  size = pool->size;
  nxmutex_unlock(&priv->unacklock);

  return size;
}

static void bt_slip_unack_cleanup(FAR struct sliphci_s *priv)
{
  bt_slip_unack_init(priv);
}

static FAR uint8_t *bt_slip_unslip_byte(FAR uint8_t *packet,
                                        FAR uint8_t *byte)
{
  uint8_t offset = 0;

  if (packet[0] != SLIP_ESC)
    {
      *byte = packet[0];
      return packet;
    }

  switch (packet[1])
    {
    case SLIP_ESC_DELIM:
      *byte = SLIP_DELIMITER;
      offset = 1;
      break;
    case SLIP_ESC_ESC:
      *byte = SLIP_ESC;
      offset = 1;
      break;
    default:
      wlerr("err: invalid escape byte %x\n", *byte);
      return NULL;
    }

  packet += offset;
  return packet;
}

static uint8_t *bt_slip_slip_byte(FAR uint8_t *packet, FAR uint8_t byte)
{
  uint8_t offset = 1;

  switch (byte)
    {
    case SLIP_DELIMITER:
      packet[0] = SLIP_ESC;
      packet[1] = SLIP_ESC_DELIM;
      offset = 2;
      break;
    case SLIP_ESC:
      packet[0] = SLIP_ESC;
      packet[1] = SLIP_ESC_ESC;
      offset = 2;
      break;
    default:
      packet[0] = byte;
      break;
    }

  packet += offset;
  return packet;
}

static void bt_slip_retx_work(FAR void *arg)
{
  FAR struct sliphci_s *priv = arg;
  FAR struct unack_frame_s *frame;
  size_t size;

  if (priv->linkstate != BT_SLIP_ACTIVE)
    {
      wlerr("err: linkstate:%d not active", priv->linkstate);
      return;
    }

  size = bt_slip_unack_size(priv);
  while (size > 0)
    {
      frame = (FAR struct unack_frame_s *)bt_slip_unack_dtor(priv);
      bt_slip_send(&priv->dev, frame->type, frame->data, frame->pktlen);
      size--;
    }
}

static bool bt_slip_reliable_packet(uint8_t type)
{
  switch (type)
    {
    case HCI_COMMAND_PKT:
    case HCI_ACLDATA_PKT:
    case HCI_EVENT_PKT:
    case HCI_ISODATA_PKT:
      return true;
    default:
      return false;
    }
}

static int bt_slip_send_packet(FAR struct sliphci_s *priv, uint8_t type,
                               FAR const uint8_t *payload, size_t len)
{
  uint8_t packet[CONFIG_BLUETOOTH_SLIP_TXBUFSIZE];
  uint8_t header[BT_SLIP_HEADER_LEN];
  FAR uint8_t *pend;
  uint16_t checksum = 0;
  int index;

  memset(header, 0, sizeof(header));
  if (bt_slip_reliable_packet(type))
    {
      BT_SLIP_SET_RELIABLE(header);
      BT_SLIP_SET_SEQ(header, priv->txseq);
      BT_SLIP_SEQ_INCR(priv->txseq);
    }

  BT_SLIP_SET_ACK(header, priv->txack);
  BT_SLIP_SET_TYPE(header, type);
  BT_SLIP_SET_LEN(header, len);

  if (priv->dipresent)
    {
      BT_SLIP_SET_DIC(header);
    }

  /* Set head checksum */

  BT_SLIP_SET_HDRCHECKSUM(header);
  pend = packet;

  /* Put SLIP start */

  *pend++ = SLIP_DELIMITER;

  /* Put h5 header */

  if (priv->dipresent)
    {
      checksum = crc16ccittpart(header, BT_SLIP_HEADER_LEN, 0xffff);
    }

  for (index = 0; index < BT_SLIP_HEADER_LEN; index++)
    {
      pend = bt_slip_slip_byte(pend, header[index]);
    }

  /* Put h5 payload */

  if (priv->dipresent)
    {
      checksum = crc16ccittpart(payload, len, checksum);
    }

  for (index = 0; index < len; index++)
    {
      pend = bt_slip_slip_byte(pend, payload[index]);
    }

  /* Put crc */

  if (priv->dipresent)
    {
      checksum = bt_slip_bit16_reverse(checksum);
      pend = bt_slip_slip_byte(pend, checksum >> 8);
      pend = bt_slip_slip_byte(pend, checksum & 0xff);
    }

  /* Put SLIP end */

  *pend++ = SLIP_DELIMITER;

  wlinfo("tx t:%d l:%d s:%d a:%d checksum:0x%04x \n",
         BT_SLIP_GET_PKT_TYPE(header), BT_SLIP_GET_LEN(header),
         BT_SLIP_GET_SEQ(header), BT_SLIP_GET_ACK(header), checksum);

  return priv->drv->send(priv->drv, type, packet, pend - packet);
}

static void bt_slip_send_link_control(FAR struct sliphci_s *priv,
                                      FAR const void *data, size_t len)
{
  bt_slip_send_packet(priv, HCI_3WIRE_LINK_PKT, data, len);
}

static void bt_slip_send_ack(FAR void *arg)
{
  FAR struct sliphci_s *priv = arg;

  if (priv->linkstate != BT_SLIP_ACTIVE)
    {
      wlerr("err: linkstate:%d not active", priv->linkstate);
      return;
    }

  bt_slip_send_packet(priv, HCI_3WIRE_ACK_PKT, NULL, 0);
}

static int bt_slip_link_packet_hanlde(FAR struct sliphci_s *priv,
                                      FAR const uint8_t *header,
                                      size_t len)
{
  FAR const uint8_t *payload = header + BT_SLIP_HEADER_LEN;
  int ret = len;

  if (memcmp(payload, g_sync_req, ARRAY_SIZE(g_sync_req)) == 0)
    {
      bt_slip_send_link_control(priv, g_sync_rsp, ARRAY_SIZE(g_sync_rsp));
    }
  else if (memcmp(payload, g_sync_rsp, ARRAY_SIZE(g_sync_rsp)) == 0)
    {
      wlinfo("h5: initialized");
      priv->linkstate = BT_SLIP_INITIALIZED;
      bt_slip_send_link_control(priv, g_conf_req, ARRAY_SIZE(g_conf_req));
    }
  else if (memcmp(payload, g_conf_req, ARRAY_SIZE(g_conf_req) - 1) == 0)
    {
      bt_slip_send_link_control(priv, g_conf_rsp, ARRAY_SIZE(g_conf_rsp));
      bt_slip_send_link_control(priv, g_conf_req, ARRAY_SIZE(g_conf_req));
    }
  else if (memcmp(payload, g_conf_rsp, ARRAY_SIZE(g_conf_rsp)) == 0)
    {
      if (BT_SLIP_GET_LEN(header) > 2)
        {
          priv->txwin = BT_SLIP_GET_TXWIN(payload);
          priv->dipresent = BT_SLIP_IS_DIC(payload);
          wlinfo("h5 txwin:%d, dipresent:%d", priv->txwin, priv->dipresent);
        }

      if (priv->txwin > CONFIG_BLUETOOTH_SLIP_TXWIN)
        {
          wlerr("err: txwin(%d) overflow(%d)", priv->txwin,
                CONFIG_BLUETOOTH_SLIP_TXWIN);
          return ret;
        }

      if (priv->linkstate == BT_SLIP_ACTIVE)
        {
          return ret;
        }

      wlinfo("h5: active");
      priv->linkstate = BT_SLIP_ACTIVE;
      nxsem_post(&priv->sem);
    }
  else
    {
      ret = -EINVAL;
    }

  return ret;
}

static void bt_slip_unack_handle(FAR struct sliphci_s *priv)
{
  size_t to_remove;
  uint8_t seq;

  to_remove = bt_slip_unack_size(priv);
  if (to_remove == 0)
    {
      return;
    }

  seq = priv->txseq;

  do
    {
      if (priv->rxack == seq)
        {
          break;
        }

      seq = (seq - 1) & 0x07;
    }
  while (--to_remove > 0);

  if (seq != priv->rxack)
    {
      wlerr("err: seq:%d != rxack:%d", seq, priv->rxack);
      return;
    }

  while (to_remove > 0)
    {
      bt_slip_unack_dtor(priv);

      /* When it was blocked by full tx window, we needs to notifiy
       * bt_slip_send.
       */

      if (bt_slip_unack_size(priv) == priv->txwin - 1)
        {
          int semcount;

          while (nxsem_get_value(&priv->sem, &semcount) >= 0 &&
                 semcount <= 0)
            {
              nxsem_post(&priv->sem);
            }
        }

      to_remove--;
    }

  to_remove = bt_slip_unack_size(priv);
  if (!to_remove)
    {
      work_cancel(HPWORK, &priv->retxworker);
    }
}

static int bt_slip_hci_packet_handle(FAR struct bt_driver_s *dev,
                                     uint8_t type, FAR void *data,
                                     size_t len)
{
  enum bt_buf_type_e var;

  if (len < 1)
    {
      return -EINVAL;
    }

  switch (type)
    {
    case HCI_EVENT_PKT:
      var = BT_EVT;
      break;
    case HCI_ACLDATA_PKT:
      var = BT_ACL_IN;
      break;
    case HCI_ISODATA_PKT:
      var = BT_ISO_IN;
      break;
    default:
      return -EINVAL;
    }

  return bt_netdev_receive(dev, var, data, len);
}

static int bt_slip_packet_receive(FAR struct sliphci_s *priv,
                             FAR uint8_t *header, size_t len)
{
  FAR uint8_t *payload = header + BT_SLIP_HEADER_LEN;
  int ret;

  if (BT_SLIP_IS_RELIABLE(header))
    {
      BT_SLIP_SEQ_INCR(priv->txack);
      bt_slip_send_ack(priv);
    }

  priv->rxack = BT_SLIP_GET_ACK(header);
  bt_slip_unack_handle(priv);

  switch (BT_SLIP_GET_PKT_TYPE(header))
    {
    case HCI_EVENT_PKT:
    case HCI_ACLDATA_PKT:
    case HCI_ISODATA_PKT:
      {
        ret = bt_slip_hci_packet_handle(&priv->dev,
                                        BT_SLIP_GET_PKT_TYPE (header),
                                        payload, BT_SLIP_GET_LEN (header));
      }
      break;
    case HCI_3WIRE_LINK_PKT:
      {
        ret = bt_slip_link_packet_hanlde(priv, header, len);
      }
      break;
    default:
      {
        ret = -ENAVAIL;
      }
      break;
    }

  return ret;
}

static int bt_slip_open(FAR struct bt_driver_s *dev)
{
  FAR struct sliphci_s *priv;
  FAR struct bt_driver_s *drv;
  int ret;

  DEBUGASSERT(dev != NULL);

  priv = (FAR struct sliphci_s *)dev;
  drv = priv->drv;

  ret = drv->open(drv);
  if (ret < 0)
    {
      return ret;
    }

  bt_slip_unack_init(priv);
  bt_slip_send_link_control(priv, g_sync_req, sizeof(g_sync_req));

  ret = nxsem_tickwait_uninterruptible(&priv->sem, SEC2TICK(3));
  if (ret == -ETIMEDOUT)
    {
      wlerr("err: bluetooth driver open timeout");
      goto err;
    }

  return ret;

err:
  bt_slip_close(dev);
  return ret;
}

static int bt_slip_send(FAR struct bt_driver_s *dev,
                        enum bt_buf_type_e type,
                        FAR void *data, size_t len)
{
  FAR struct sliphci_s *priv;
  FAR struct unack_frame_s *frame;
  int ret;
  uint8_t var;

  DEBUGASSERT(dev != NULL);

  if (data == NULL)
    {
      return -EINVAL;
    }

  priv = (FAR struct sliphci_s *)dev;

  ret = nxmutex_lock(&priv->sliplock);
  if (ret < 0)
    {
      return ret;
    }

  if (bt_slip_unack_size(priv) >= priv->txwin)
    {
      bt_slip_send_ack(priv);

      nxmutex_unlock(&priv->sliplock);
      nxsem_wait_uninterruptible(&priv->sem);
      ret = nxmutex_lock(&priv->sliplock);
      if (ret < 0)
        {
          return ret;
        }
    }

  switch (type)
    {
    case BT_CMD:
      var = HCI_COMMAND_PKT;
      break;
    case BT_ACL_OUT:
      var = HCI_ACLDATA_PKT;
      break;
    case BT_ISO_OUT:
      var = HCI_ISODATA_PKT;
      break;
    default:
      ret = -EINVAL;
      goto end;
    }

  ret = bt_slip_send_packet(priv, var, data, len);
  if (ret < 0)
    {
      goto end;
    }

  frame = (FAR struct unack_frame_s *)bt_slip_unack_ctor(priv);
  frame->type = type;
  frame->pktlen = len;
  memcpy(frame->data, data, len);

  if (work_available(&priv->retxworker))
    {
      work_queue(HPWORK, &priv->retxworker, bt_slip_retx_work,
                 priv, BT_SLIP_RTX_TIMEOUT);
    }

end:
  nxmutex_unlock(&priv->sliplock);
  return ret < 0 ? ret : len;
}

/* Note: bt_slip_receive could only handle completed slip packets with
 * len bytes data. Avoid adding a memory copy of the data, it won't
 * handle this case here. you need to combine multiple incomplete slip
 * packets to completed slip packets from bt vendor driver.
 */

static int bt_slip_receive(FAR struct bt_driver_s *drv,
                           enum bt_buf_type_e type,
                           FAR void *data, size_t len)
{
  FAR struct sliphci_s *priv = drv->priv;
  FAR uint8_t *packet;
  FAR uint8_t *cursor;
  FAR uint8_t *header;
  FAR uint8_t *pointer;
  uint8_t byte = 0;
  uint16_t checksum;
  size_t remaining;
  uint8_t state;
  int ret;
  enum
    {
      PACKET_START,
      PACKET_HEADER,
      PACKET_PAYLOAD,
      PACKET_DICHECK,
      PACKET_END,
    };

  ret = nxmutex_lock(&priv->sliplock);
  if (ret < 0)
    {
      return ret;
    }

  for (packet = data, cursor = data, header = data, state = PACKET_START;
       packet < (FAR uint8_t *)data + len; packet++)
    {
      switch (state)
        {
        case PACKET_START:
          {
            if (*packet == SLIP_DELIMITER)
              {
                state = PACKET_HEADER;
                remaining = BT_SLIP_HEADER_LEN;
              }
          }
          break;
        case PACKET_HEADER:
          {
            if (*packet == SLIP_DELIMITER)
              {
                break;
              }

            pointer = bt_slip_unslip_byte(packet, &byte);
            if (!pointer)
              {
                state = PACKET_START;
                break;
              }

            packet = pointer;
            *cursor++ = byte;
            remaining--;

            if (remaining)
              {
                break;
              }

            /* A packet from bt maybe consists of two H5 hci frame.
             */

            header = cursor - BT_SLIP_HEADER_LEN;

            wlinfo("rx t:%d l:%d s:%d a:%d", BT_SLIP_GET_PKT_TYPE(header),
                    BT_SLIP_GET_LEN(header), BT_SLIP_GET_SEQ(header),
                    BT_SLIP_GET_ACK(header));

            if (!BT_SLIP_IS_HDRVALIDED(header))
              {
                wlerr("err: invalid header checksum");
                state = PACKET_START;
                break;
              }

            if (BT_SLIP_IS_RELIABLE(header)
                && BT_SLIP_GET_SEQ(header) != priv->txack)
              {
                wlerr("err: out of order packet arrived(%u != %u)",
                       BT_SLIP_GET_SEQ(header), priv->txack);
                state = PACKET_START;
                break;
              }

            switch (BT_SLIP_GET_PKT_TYPE(header))
              {
              case HCI_EVENT_PKT:
              case HCI_ACLDATA_PKT:
              case HCI_ISODATA_PKT:
              case HCI_3WIRE_LINK_PKT:
              case HCI_3WIRE_ACK_PKT:
                {
                  /* In DIC mode, Ack packet which has 0 byte paylod
                   * and has 2 byte data integrity check
                   */

                  if (BT_SLIP_GET_LEN(header))
                    {
                      state = PACKET_PAYLOAD;
                      remaining = BT_SLIP_GET_LEN(header);
                    }
                  else if (BT_SLIP_GET_CRC(header))
                    {
                      state = PACKET_DICHECK;
                      remaining = BT_SLIP_CHECKSUM_LEN;
                    }
                  else
                    {
                      state = PACKET_END;
                      remaining = 0;
                    }
                }
                break;
              default:
                {
                  wlerr("err: packet type:%u", BT_SLIP_GET_PKT_TYPE(header));
                  state = PACKET_START;
                }
                break;
              }
          }
          break;
        case PACKET_PAYLOAD:
          {
            pointer = bt_slip_unslip_byte(packet, &byte);
            if (!pointer)
              {
                state = PACKET_START;
                break;
              }

            packet = pointer;
            *cursor++ = byte;
            remaining--;

            if (remaining)
              {
                break;
              }

            if (BT_SLIP_GET_CRC(header))
              {
                state = PACKET_DICHECK;
                remaining = BT_SLIP_CHECKSUM_LEN;
              }
            else
              {
                state = PACKET_END;
              }
          }
          break;
        case PACKET_DICHECK:
          {
            pointer = bt_slip_unslip_byte(packet, &byte);
            if (!pointer)
              {
                state = PACKET_START;
                break;
              }

            packet = pointer;
            *cursor++ = byte;
            remaining--;

            if (remaining)
              {
                break;
              }

            /* Remove 2 bytes crc payload, then caculate packect
             * checksum with packet header and body.
             */

            cursor -= BT_SLIP_CHECKSUM_LEN;
            checksum = crc16ccittpart(header, cursor - header, 0xffff);

            if (bt_slip_bit16_reverse(checksum) != bt_slip_get_crc(cursor))
              {
                wlerr("err: checksum(0x%04x) not match(0x%04x)", checksum,
                      bt_slip_get_crc(cursor));
                state = PACKET_START;
                break;
              }

            state = PACKET_END;
          }
          break;
        case PACKET_END:
          {
            if (*packet != SLIP_DELIMITER)
              {
                wlerr("err: miss slip end");
                state = PACKET_START;
                break;
              }

            bt_slip_packet_receive(priv, header, cursor - header);
            state = PACKET_START;
          }
          break;
        }
    }

  nxmutex_unlock(&priv->sliplock);
  return packet - (FAR uint8_t *)data;
}

static int bt_slip_ioctl(FAR struct bt_driver_s *dev, int cmd,
                         unsigned long arg)
{
  FAR struct sliphci_s *priv;
  FAR struct bt_driver_s *drv;

  DEBUGASSERT(dev != NULL);

  priv = (FAR struct sliphci_s *)dev;
  drv = priv->drv;

  if (!drv->ioctl)
    {
      return -ENOTTY;
    }

  return drv->ioctl(drv, cmd, arg);
}

static void bt_slip_close(FAR struct bt_driver_s *dev)
{
  FAR struct sliphci_s *priv;
  FAR struct bt_driver_s *drv;

  DEBUGASSERT(dev != NULL);

  priv = (FAR struct sliphci_s *)dev;
  drv = priv->drv;

  drv->close(drv);
  priv->linkstate = BT_SLIP_UNINITIALIZED;
  bt_slip_unack_cleanup(priv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct bt_driver_s *bt_slip_register(FAR struct bt_driver_s *drv)
{
  FAR struct sliphci_s *priv;

  DEBUGASSERT(drv != NULL);

  priv = kmm_zalloc(sizeof(struct sliphci_s));
  if (!priv)
    {
      return NULL;
    }

  priv->dev.open = bt_slip_open;
  priv->dev.send = bt_slip_send;
  priv->dev.ioctl = bt_slip_ioctl;
  priv->dev.close = bt_slip_close;

  priv->drv = drv;
  drv->priv = priv;
  drv->receive = bt_slip_receive;

  nxmutex_init(&priv->sliplock);
  nxmutex_init(&priv->unacklock);
  nxsem_init(&priv->sem, 0, 0);

  return &priv->dev;
}
