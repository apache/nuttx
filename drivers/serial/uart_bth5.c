/****************************************************************************
 * drivers/serial/uart_bth5.c
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

#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mm/circbuf.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>

#include <debug.h>
#include <fcntl.h>
#include <poll.h>
#include <string.h>

#include <nuttx/wireless/bluetooth/bt_driver.h>
#include <nuttx/wireless/bluetooth/bt_hci.h>
#include <nuttx/wireless/bluetooth/bt_uart.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

#define MAX_OPENCNT          (255) /* Limit of uint8_t */

#define HCI_3WIRE_ACK_PKT    0x00
#define HCI_COMMAND_PKT      0x01
#define HCI_ACLDATA_PKT      0x02
#define HCI_SCODATA_PKT      0x03
#define HCI_EVENT_PKT        0x04
#define HCI_ISODATA_PKT      0x05
#define HCI_3WIRE_LINK_PKT   0x0f
#define HCI_VENDOR_PKT       0xff

#define SLIP_DELIMITER       0xc0
#define SLIP_ESC             0xdb
#define SLIP_ESC_DELIM       0xdc
#define SLIP_ESC_ESC         0xdd

#define H5_BIT_RX_ESC        (0x00000001 << 0)
#define H5_BIT_RX_CRC        (0x00000001 << 1)

#define H5_HDR_SEQ(hdr)      ((hdr)[0] & 0x07)
#define H5_HDR_ACK(hdr)      (((hdr)[0] >> 3) & 0x07)
#define H5_HDR_CRC(hdr)      (((hdr)[0] >> 6) & 0x01)
#define H5_HDR_RELIABLE(hdr) (((hdr)[0] >> 7) & 0x01)
#define H5_HDR_PKT_TYPE(hdr) ((hdr)[1] & 0x0f)
#define H5_HDR_LEN(hdr)      ((((hdr)[1] >> 4) & 0x0f) + ((hdr)[2] << 4))

#define H5_SET_SEQ(hdr, seq)    ((hdr)[0] |= (seq))
#define H5_SET_ACK(hdr, ack)    ((hdr)[0] |= (ack) << 3)
#define H5_SET_RELIABLE(hdr)    ((hdr)[0] |= 1 << 7)
#define H5_SET_TYPE(hdr, type)  ((hdr)[1] |= (type))
#define H5_SET_LEN(hdr, len)    (((hdr)[1] |= ((len)&0x0f) << 4), ((hdr)[2] |= (len) >> 4))

#define H5_ACK_TIMEOUT MSEC2TICK(250)  /* 250ms */
#define H5_RTX_TIMEOUT MSEC2TICK(150)  /* 150ms */

union bt_hdr_u
{
  struct bt_hci_cmd_hdr_s cmd;
  struct bt_hci_acl_hdr_s acl;
  struct bt_hci_evt_hdr_s evt;
  struct bt_hci_iso_hdr_s iso;
};

enum
{
  H5_MSG_INVALID,
  H5_MSG_SYNC_REQ,
  H5_MSG_SYNC_RSP,
  H5_MSG_CONF_REQ,
  H5_MSG_CONF_RSP,
};

struct unack_pool_s
{
  size_t start;
  size_t end;
  size_t size;

  uint8_t buf[CONFIG_UART_BTH5_TXWIN][CONFIG_UART_BTH5_TXBUFSIZE];
};

struct uart_bth5_s
{
  FAR struct bt_driver_s *drv;

  struct circbuf_s circbuf;
  uint8_t sendbuf[CONFIG_UART_BTH5_TXBUFSIZE];
  uint8_t recvbuf[CONFIG_UART_BTH5_TXBUFSIZE * 2];

  bool crcvalid;
  uint8_t openrefs;
  uint16_t crcvalue;
  unsigned long flags;
  size_t sendlen; /* sendbuffer hci data len */
  size_t recvlen;

  size_t rxpending; /* Expecting more bytes */
  uint8_t rxack;    /* Last ack number received */
  uint8_t txseq;    /* Next seq number to send */
  uint8_t txack;    /* Next ack number to send */
  uint8_t txwin;    /* Sliding window size */

  mutex_t openlock;
  mutex_t sendlock;
  mutex_t recvlock;

  sem_t opensem;
  sem_t recvsem;
  sem_t acksem;

  struct work_s retxworker;
  struct work_s ackworker;

  struct unack_pool_s unackpool;

  CODE int (*rxfunc)(FAR struct uart_bth5_s *dev, uint8_t c);

  enum
  {
    H5_UNINITIALIZED,
    H5_INITIALIZED,
    H5_ACTIVE,
  } state;

  FAR struct pollfd *fds[CONFIG_UART_BTH5_NPOLLWAITERS];
};

struct unack_frame_s
{
  enum bt_buf_type_e type;
  size_t pktlen;
  uint8_t data[1];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int uart_bth5_open(FAR struct file *filep);
static int uart_bth5_close(FAR struct file *filep);
static ssize_t uart_bth5_read(FAR struct file *filep, FAR char *buffer,
                              size_t buflen);
static ssize_t uart_bth5_write(FAR struct file *filep,
                               FAR const char *buffer, size_t buflen);
static int uart_bth5_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg);
static void uart_bth5_post(FAR sem_t *sem);
static int uart_bth5_poll(FAR struct file *filep, FAR struct pollfd *fds,
                          bool setup);
static void uart_bth5_pollnotify(FAR struct uart_bth5_s *dev,
                                 pollevent_t eventset);

static void h5_rx_reset(FAR struct uart_bth5_s *dev);
static int uart_h5_send(FAR struct uart_bth5_s *dev, uint8_t type,
                        FAR const uint8_t *payload, size_t len);

static void h5_ack_work(FAR void *arg);
static void h5_retx_work(FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_uart_bth5_ops =
{
  uart_bth5_open,  /* open */
  uart_bth5_close, /* close */
  uart_bth5_read,  /* read */
  uart_bth5_write, /* write */
  NULL,            /* seek */
  uart_bth5_ioctl, /* ioctl */
  NULL,            /* mmap */
  NULL,            /* truncate */
  uart_bth5_poll   /* poll */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void
h5_peer_reset(FAR struct uart_bth5_s *dev)
{
  dev->state = H5_UNINITIALIZED;
  dev->txseq = 0;
  dev->txack = 0;

  work_cancel(HPWORK, &dev->retxworker);
  work_cancel(HPWORK, &dev->ackworker);
}

static uint8_t
h5_crc_rev8(uint8_t byte)
{
  static uint8_t rev8table[256] =
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

  return rev8table[byte];
}

static uint16_t
h5_crc_rev16(uint16_t x)
{
  return (h5_crc_rev8(x & 0xff) << 8) | h5_crc_rev8(x >> 8);
}

static void
h5_crc_update(FAR uint16_t *crc, uint8_t d)
{
  uint16_t reg;
  static const uint16_t crctable[16] =
  {
    0x0000, 0x1081, 0x2102, 0x3183, 0x4204, 0x5285, 0x6306, 0x7387,
    0x8408, 0x9489, 0xa50a, 0xb58b, 0xc60c, 0xd68d, 0xe70e, 0xf78f
  };

  reg = *crc;
  reg = (reg >> 4) ^ crctable[(reg ^ d) & 0x000f];
  reg = (reg >> 4) ^ crctable[(reg ^ (d >> 4)) & 0x000f];

  *crc = reg;
}

static uint16_t
h5_get_crc(FAR struct uart_bth5_s *dev)
{
  uint8_t *hdr = dev->recvbuf;
  uint8_t *data = hdr + 4 + H5_HDR_LEN(hdr);

  return data[1] + (data[0] << 8);
}

static void
h5_unack_init(FAR struct unack_pool_s *pool)
{
  pool->start = 0;
  pool->end = 0;
  pool->size = 0;
}

static FAR void *
h5_unack_ctor(FAR struct unack_pool_s *pool)
{
  FAR void *p;

  p = pool->buf[pool->start++];
  pool->start %= CONFIG_UART_BTH5_TXWIN;
  pool->size++;

  return p;
}

static FAR void *
h5_unack_dtor(FAR struct unack_pool_s *pool)
{
  FAR void *p;

  p = pool->buf[pool->end++];
  pool->end %= CONFIG_UART_BTH5_TXWIN;
  pool->size--;

  return p;
}

static size_t
h5_unack_size(FAR struct unack_pool_s *pool)
{
  return pool->size;
}

static void
h5_unack_cleanup(FAR struct unack_pool_s *pool)
{
  h5_unack_init(pool);
}

static void
h5_link_control(FAR struct uart_bth5_s *dev, FAR const void *data,
                size_t len)
{
  uart_h5_send(dev, HCI_3WIRE_LINK_PKT, data, len);
}

static void
h5_message_handle(FAR struct uart_bth5_s *dev)
{
  FAR const uint8_t *hdr = dev->recvbuf;
  FAR const uint8_t *data = &dev->recvbuf[4];
  const uint8_t sync_req[] =
  {
    0x01, 0x7e
  };

  const uint8_t sync_rsp[] =
  {
    0x02, 0x7d
  };

  uint8_t conf_req[] =
  {
    0x03, 0xfc, 0x10 | CONFIG_UART_BTH5_TXWIN
  };

  const uint8_t conf_rsp[] =
  {
    0x04, 0x7b
  };

  if (H5_HDR_LEN(hdr) < 2 || (H5_HDR_PKT_TYPE(hdr) != HCI_3WIRE_LINK_PKT))
    {
      return;
    }

  if (memcmp(data, sync_req, 2) == 0)
    {
      h5_link_control(dev, sync_rsp, 2);
    }
  else if (memcmp(data, sync_rsp, 2) == 0)
    {
      wlinfo("h5: initialized");
      dev->state = H5_INITIALIZED;
      h5_link_control(dev, conf_req, 3);
    }
  else if (memcmp(data, conf_req, 2) == 0)
    {
      h5_link_control(dev, conf_rsp, 2);
      h5_link_control(dev, conf_req, 3);
    }
  else if (memcmp(data, conf_rsp, 2) == 0)
    {
      if (dev->state == H5_ACTIVE)
        {
          return;
        }

      if (H5_HDR_LEN(hdr) > 2)
        {
          dev->txwin = (data[2] & 0x07);
          dev->crcvalid = ((data[2] & 0x10) == 0x10);
          wlinfo("h5 txwin:%d, crcvalid:%d", dev->txwin, dev->crcvalid);
        }

      if (dev->txwin < CONFIG_UART_BTH5_TXWIN)
        {
          wlerr("h5, txwin(%d) overflow(%d)", dev->txwin,
                CONFIG_UART_BTH5_TXWIN);
          return;
        }

        wlinfo("h5: active");
        dev->state = H5_ACTIVE;
        nxsem_post(&dev->opensem);
    }
  else
    {
      wlerr("ERROR Link Control: 0x%02hhx 0x%02hhx", data[0], data[1]);
    }
}

static void
h5_unack_handle(FAR struct uart_bth5_s *dev)
{
  size_t to_remove;
  uint8_t seq;

  nxmutex_lock(&dev->sendlock);

  to_remove = h5_unack_size(&dev->unackpool);
  if (to_remove == 0)
    {
      goto end;
    }

  seq = dev->txseq;

  do
    {
      if (dev->rxack == seq)
        {
          break;
        }

      seq = (seq - 1) & 0x07;
    }
  while (--to_remove > 0);

  if (seq != dev->rxack)
    {
      wlerr("error, %s seq:%d != rxack:%d", __func__, seq, dev->rxack);
      goto end;
    }

  while (to_remove > 0)
    {
      h5_unack_dtor(&dev->unackpool);
      if (to_remove == dev->txwin)
        {
          uart_bth5_post(&dev->acksem);
        }

      to_remove--;
    }

  if (!h5_unack_size(&dev->unackpool))
    {
      work_cancel(HPWORK, &dev->retxworker);
    }

end:
  nxmutex_unlock(&dev->sendlock);
}

static void
h5_recv_handle(FAR struct uart_bth5_s *dev)
{
  int reserve = dev->drv->head_reserve;
  FAR uint8_t *hdr = dev->recvbuf;
  uint8_t type;

  if (H5_HDR_RELIABLE(hdr))
    {
      dev->txack = (dev->txack + 1) & 0x07;
      if (work_available(&dev->ackworker))
        {
          work_queue(HPWORK, &dev->ackworker, h5_ack_work, dev,
                     H5_ACK_TIMEOUT);
        }
    }

  dev->rxack = H5_HDR_ACK(hdr);
  type = H5_HDR_PKT_TYPE(hdr);
  h5_unack_handle(dev);

  switch (H5_HDR_PKT_TYPE(hdr))
    {
      case HCI_EVENT_PKT:
      case HCI_ACLDATA_PKT:
      case HCI_SCODATA_PKT:
      case HCI_ISODATA_PKT:
        {
          nxmutex_lock(&dev->recvlock);
          if (circbuf_space(&dev->circbuf) >= dev->recvlen + reserve)
            {
              circbuf_write(&dev->circbuf, &type, reserve);
              circbuf_write(&dev->circbuf, dev->recvbuf + 4,
                            dev->recvlen - 4);
            }

          uart_bth5_pollnotify(dev, POLLIN);
          nxmutex_unlock(&dev->recvlock);
          break;
        }

      case HCI_3WIRE_LINK_PKT:
        {
          h5_message_handle(dev);
          break;
        }

      default:
        break;
    }

  h5_rx_reset(dev);
}

static int
h5_rx_crc(FAR struct uart_bth5_s *dev, uint8_t c)
{
  if (h5_crc_rev16(dev->crcvalue) != h5_get_crc(dev))
    {
      wlerr("error, crcvalue(%04x) recv(%04x)", h5_crc_rev16(dev->crcvalue),
            h5_get_crc(dev));
      h5_rx_reset(dev);
      return -EINVAL;
    }

  dev->recvlen -= 2;
  h5_recv_handle(dev);

  return 0;
}

static int
h5_rx_payload(FAR struct uart_bth5_s *dev, uint8_t c)
{
  FAR uint8_t *hdr = dev->recvbuf;

  if (H5_HDR_CRC(hdr))
    {
      dev->rxfunc = h5_rx_crc;
      dev->rxpending = 2;
      dev->flags |= H5_BIT_RX_CRC;
    }
  else
    {
      h5_recv_handle(dev);
    }

  return 0;
}

static int
h5_rx_header(FAR struct uart_bth5_s *dev, uint8_t c)
{
  FAR uint8_t *hdr = dev->recvbuf;

  wlinfo("rx t:%d l:%d s:%d a:%d", H5_HDR_PKT_TYPE(hdr), H5_HDR_LEN(hdr),
         H5_HDR_SEQ(hdr), H5_HDR_ACK(hdr));

  if (((hdr[0] + hdr[1] + hdr[2] + hdr[3]) & 0xff) != 0xff)
    {
      wlerr("error: invalid header checksum");
      h5_rx_reset(dev);
      return -EINVAL;
    }

  if (H5_HDR_RELIABLE(hdr) && H5_HDR_SEQ(hdr) != dev->txack)
    {
      work_queue(HPWORK, &dev->ackworker, h5_ack_work, dev, 0);
      h5_rx_reset(dev);
      return -EINVAL;
    }

  if (dev->state != H5_ACTIVE && H5_HDR_PKT_TYPE(hdr) != HCI_3WIRE_LINK_PKT)
    {
      wlerr("error: non-link packet received in non-active state");
      h5_rx_reset(dev);
      return -EINVAL;
    }

  dev->rxfunc = h5_rx_payload;
  dev->rxpending = H5_HDR_LEN(hdr);

  return 0;
}

static int
h5_rx_start(FAR struct uart_bth5_s *dev, uint8_t c)
{
  if (c == SLIP_DELIMITER)
    {
      return 1;
    }

  dev->rxfunc = h5_rx_header;
  dev->rxpending = 4;
  dev->crcvalue = 0xffff;
  return 0;
}

static int
h5_rx_delimiter(FAR struct uart_bth5_s *dev, uint8_t c)
{
  if (c == SLIP_DELIMITER)
    {
      dev->rxfunc = h5_rx_start;
    }

  return 1;
}

static void
h5_rx_reset(FAR struct uart_bth5_s *dev)
{
  dev->recvlen = 0;
  dev->rxfunc = h5_rx_delimiter;
  dev->rxpending = 0;
  dev->flags = 0;
}

static int
h5_unslip_one_byte(FAR struct uart_bth5_s *dev, uint8_t c)
{
  uint8_t byte = c;

  if (!(dev->flags & H5_BIT_RX_ESC) && c == SLIP_ESC)
    {
      dev->flags |= H5_BIT_RX_ESC;
      return 0;
    }

  if (dev->flags & H5_BIT_RX_ESC)
    {
      dev->flags &= ~H5_BIT_RX_ESC;
      switch (c)
        {
          case SLIP_ESC_DELIM:
            byte = SLIP_DELIMITER;
            break;

          case SLIP_ESC_ESC:
            byte = SLIP_ESC;
            break;

          default:
            wlerr("error: invalid esc byte 0x%02hhx", c);
            h5_rx_reset(dev);
            return -EINVAL;
        }
    }

  dev->recvbuf[dev->recvlen++] = byte;
  dev->rxpending--;

  if (H5_HDR_CRC(dev->recvbuf) && !(dev->flags & H5_BIT_RX_CRC))
    {
      h5_crc_update(&dev->crcvalue, byte);
    }

  return 0;
}

static bool
h5_reliable_packet(uint8_t type)
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

static int
h5_slip_delim(FAR uint8_t *frame, int index)
{
  frame[index] = SLIP_DELIMITER;
  return 1;
}

static int
h5_slip_one_byte(FAR uint8_t *frame, int index, uint8_t c)
{
  int ret;

  switch (c)
    {
      case SLIP_DELIMITER:
        {
          frame[index++] = SLIP_ESC;
          frame[index] = SLIP_ESC_DELIM;
          ret = 2;
          break;
        }

      case SLIP_ESC:
        {
          frame[index++] = SLIP_ESC;
          frame[index] = SLIP_ESC_ESC;
          ret = 2;
          break;
        }

      default:
        frame[index] = c;
        ret = 1;
        break;
    }

  return ret;
}

static int
h5_uart_header(FAR uint8_t *data, enum bt_buf_type_e *type,
               size_t *pktlen, size_t *hdrlen, size_t reserved)
{
  int ret = OK;
  FAR union bt_hdr_u *hdr = (FAR union bt_hdr_u *)data;

  switch (*(data - reserved))
    {
      case H4_CMD:
        {
          *hdrlen = sizeof(struct bt_hci_cmd_hdr_s);
          *pktlen = hdr->cmd.param_len;
          *type = HCI_COMMAND_PKT;
          break;
        }

      case H4_ACL:
        {
          *hdrlen = sizeof(struct bt_hci_acl_hdr_s);
          *pktlen = hdr->acl.len;
          *type = HCI_ACLDATA_PKT;
          break;
        }

      case H4_ISO:
        {
          *hdrlen = sizeof(struct bt_hci_iso_hdr_s);
          *pktlen = hdr->iso.len;
          *type = HCI_ISODATA_PKT;
          break;
        }

      default:
        {
          ret = -EINVAL;
          break;
        }
    }

  return ret;
}

static void
h5_ack_work(FAR void *arg)
{
  FAR struct uart_bth5_s *dev = arg;

  if (dev->state != H5_ACTIVE)
    {
      wlerr("%s state:%d not active", __func__, dev->state);
      return;
    }

  uart_h5_send(dev, HCI_3WIRE_ACK_PKT, NULL, 0);
}

static void
h5_retx_work(FAR void *arg)
{
  FAR struct uart_bth5_s *dev = arg;
  FAR struct unack_frame_s *frame;
  size_t size;

  if (dev->state != H5_ACTIVE)
    {
      wlerr("%s state:%d not active", __func__, dev->state);
      return;
    }

  nxmutex_lock(&dev->sendlock);
  size = h5_unack_size(&dev->unackpool);
  while (size > 0)
    {
      frame = (FAR struct unack_frame_s *)h5_unack_dtor(&dev->unackpool);
      uart_h5_send(dev, frame->type, frame->data, frame->pktlen);
      size--;
    }

  nxmutex_unlock(&dev->sendlock);
}

static void
uart_bth5_post(FAR sem_t *sem)
{
  int semcount;

  nxsem_get_value(sem, &semcount);
  if (semcount < 1)
    {
      nxsem_post(sem);
    }
}

static void
uart_bth5_pollnotify(FAR struct uart_bth5_s *dev, pollevent_t eventset)
{
  poll_notify(dev->fds, CONFIG_UART_BTH5_NPOLLWAITERS, eventset);

  if ((eventset & POLLIN) != 0)
    {
      uart_bth5_post(&dev->recvsem);
    }
}

static int
uart_bth5_receive(FAR struct bt_driver_s *drv, enum bt_buf_type_e type,
                  FAR void *buffer, size_t buflen)
{
  FAR struct uart_bth5_s *dev = drv->priv;
  FAR const uint8_t *ptr = buffer;
  int processed;

  while (buflen > 0)
    {
      if (dev->rxpending > 0)
        {
          if (*ptr == SLIP_DELIMITER)
            {
              wlerr("error, too short H5 packet");
              h5_rx_reset(dev);
              continue;
            }

          h5_unslip_one_byte(dev, *ptr);

          ptr++;
          buflen--;
          continue;
        }

      processed = dev->rxfunc(dev, *ptr);
      if (processed < 0)
        {
          return processed;
        }

      ptr += processed;
      buflen -= processed;
    }

  return 0;
}

static int
uart_bth5_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct uart_bth5_s *dev = inode->i_private;
  int ret;
  const uint8_t sync_req[] =
  {
    0x01, 0x7e
  };

  ret = nxmutex_lock(&dev->openlock);
  if (ret < 0)
    {
      return ret;
    }

  if (dev->openrefs == MAX_OPENCNT)
    {
      ret = -EMFILE;
      goto end;
    }
  else
    {
      dev->openrefs++;
    }

  if (dev->openrefs > 1)
    {
      goto end;
    }

  ret = dev->drv->open(dev->drv);
  if (ret < 0)
    {
      goto end;
    }

  dev->sendlen = 0;
  dev->state = H5_UNINITIALIZED;

  h5_link_control(dev, sync_req, sizeof(sync_req));

  ret = nxsem_tickwait_uninterruptible(&dev->opensem, SEC2TICK(3));
  if (ret == -ETIMEDOUT)
    {
      wlerr("error, bluetooth driver open timeout");
      nxmutex_unlock(&dev->openlock);
      return ret;
    }

  h5_unack_init(&dev->unackpool);

end:
  nxmutex_unlock(&dev->openlock);
  return OK;
}

static int
uart_bth5_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct uart_bth5_s *dev = inode->i_private;
  int ret = OK;

  ret = nxmutex_lock(&dev->openlock);
  if (ret < 0)
    {
      goto end;
    }

  if (dev->openrefs == 0)
    {
      ret = -EIO;
      goto end;
    }
  else
    {
      dev->openrefs--;
    }

  if (dev->openrefs > 0)
    {
      goto end;
    }

  dev->drv->close(dev->drv);
  dev->state = H5_UNINITIALIZED;

  h5_peer_reset(dev);
  h5_rx_reset(dev);
  uart_bth5_pollnotify(dev, POLLIN | POLLOUT);
  nxsem_post(&dev->opensem);
  h5_unack_cleanup(&dev->unackpool);

end:
  nxmutex_unlock(&dev->openlock);
  return ret;
}

static ssize_t
uart_bth5_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct uart_bth5_s *dev = inode->i_private;
  ssize_t nread;

  nxmutex_lock(&dev->recvlock);
  while (1)
    {
      nread = circbuf_read(&dev->circbuf, buffer, buflen);
      if (nread != 0 || (filep->f_oflags & O_NONBLOCK))
        {
          break;
        }

      while (circbuf_is_empty(&dev->circbuf))
        {
          nxmutex_unlock(&dev->recvlock);
          nxsem_wait_uninterruptible(&dev->recvsem);
          nxmutex_lock(&dev->recvlock);
        }
    }

  nxmutex_unlock(&dev->recvlock);
  return nread;
}

static int
uart_h5_send(FAR struct uart_bth5_s *dev, uint8_t type,
             FAR const uint8_t *payload, size_t len)
{
  uint8_t frame[CONFIG_UART_BTH5_TXBUFSIZE];
  uint8_t hdr[4];
  int idx;
  int length = 0;
  uint16_t h5_txmsg_crc = 0xffff;

  memset(hdr, 0, sizeof(hdr));
  if (h5_reliable_packet(type))
    {
      H5_SET_RELIABLE(hdr);
      H5_SET_SEQ(hdr, dev->txseq);
      dev->txseq = (dev->txseq + 1) & 0x07;
      work_queue(HPWORK, &dev->retxworker, h5_retx_work, dev,
                 H5_RTX_TIMEOUT);
    }

  H5_SET_ACK(hdr, dev->txack);
  H5_SET_TYPE(hdr, type);
  H5_SET_LEN(hdr, len);

  if (dev->crcvalid)
    {
      hdr[0] |= 0x40;
    }

  /* Set head checksum */

  hdr[3] = ~((hdr[0] + hdr[1] + hdr[2]) & 0xff);
  length += h5_slip_delim(frame, length);

  /* Put h5 header */

  for (idx = 0; idx < 4; idx++)
    {
      length += h5_slip_one_byte(frame, length, hdr[idx]);
      if (dev->crcvalid)
        {
          h5_crc_update(&h5_txmsg_crc, hdr[idx]);
        }
    }

  /* Put h5 payload */

  for (idx = 0; idx < len; idx++)
    {
      length += h5_slip_one_byte(frame, length, payload[idx]);
      if (dev->crcvalid)
        {
          h5_crc_update(&h5_txmsg_crc, payload[idx]);
        }
    }

  if (dev->crcvalid)
    {
      h5_txmsg_crc = h5_crc_rev16(h5_txmsg_crc);
      length += h5_slip_one_byte(frame, length,
                                 (uint8_t)((h5_txmsg_crc >> 8) & 0x00ff));
      length += h5_slip_one_byte(frame, length,
                                 (uint8_t)(h5_txmsg_crc & 0x00ff));
    }

  length += h5_slip_delim(frame, length);

  work_cancel(HPWORK, &dev->ackworker);

  wlinfo("tx t:%d l:%d s:%d a:%d\n", H5_HDR_PKT_TYPE(hdr), H5_HDR_LEN(hdr),
         H5_HDR_SEQ(hdr), H5_HDR_ACK(hdr));
  return dev->drv->send(dev->drv, type, frame, length);
}

static ssize_t
uart_bth5_write(FAR struct file *filep, FAR const char *buffer,
                size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct uart_bth5_s *dev = inode->i_private;
  enum bt_buf_type_e type;
  size_t reserved  = dev->drv->head_reserve;
  FAR uint8_t *data;
  size_t pktlen;
  size_t hdrlen;
  int ret;
  FAR struct unack_frame_s *frame;

  ret = nxmutex_lock(&dev->sendlock);
  if (ret < 0)
    {
      wlerr("%s error, nxmutex_lock", __func__);
      return ret;
    }

  data = dev->sendbuf + reserved + dev->sendlen;
  if (dev->sendlen + buflen > CONFIG_UART_BTH5_TXBUFSIZE - reserved)
    {
      ret = -E2BIG;
      goto err;
    }

  memcpy(data - reserved + dev->sendlen, buffer, buflen);
  dev->sendlen += buflen;

  for (; ; )
    {
      ret = h5_uart_header(data, &type, &pktlen, &hdrlen, reserved);
      if (ret < 0)
        {
          goto err;
        }

      /* Reassembly is incomplete ? */

      if (dev->sendlen < hdrlen)
        {
          goto out;
        }

      pktlen += hdrlen;
      if (dev->sendlen < pktlen)
        {
          goto out;
        }

      /* Got the full packet, send out */

      if (h5_unack_size(&dev->unackpool) > dev->txwin)
        {
          work_queue(HPWORK, &dev->ackworker, h5_ack_work, dev, 0);
          if (filep->f_oflags & O_NONBLOCK)
            {
              ret = -EAGAIN;
              goto out;
            }
          else
            {
              nxmutex_unlock(&dev->sendlock);
              nxsem_wait_uninterruptible(&dev->acksem);
              nxmutex_lock(&dev->sendlock);
            }
        }

      ret = uart_h5_send(dev, type, data, pktlen);
      if (ret < 0)
        {
          goto err;
        }

      frame = (FAR struct unack_frame_s *)h5_unack_ctor(&dev->unackpool);
      frame->type = type;
      frame->pktlen = pktlen;
      memcpy(frame->data, data, pktlen);

      dev->sendlen -= pktlen + reserved;

      if (dev->sendlen > 0)
        {
          memmove(data - reserved, dev->sendbuf + pktlen, dev->sendlen);
        }
    }

err:
  dev->sendlen = 0;

out:
  nxmutex_unlock(&dev->sendlock);
  return ret < 0 ? ret : buflen;
}

static int
uart_bth5_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct uart_bth5_s *dev = inode->i_private;

  if (!dev->drv->ioctl)
    {
      return -ENOTTY;
    }

  return dev->drv->ioctl(dev->drv, cmd, arg);
}

static int
uart_bth5_poll(FAR struct file *filep, FAR struct pollfd *fds, bool setup)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct uart_bth5_s *dev = inode->i_private;
  pollevent_t eventset = 0;
  int ret = 0;
  int i;

  if (setup)
    {
      for (i = 0; i < CONFIG_UART_BTH5_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!dev->fds[i])
            {
              /* Bind the poll structure and this slot */

              dev->fds[i] = fds;
              fds->priv = &dev->fds[i];
              break;
            }
        }

      if (i >= CONFIG_UART_BTH5_NPOLLWAITERS)
        {
          fds->priv = NULL;
          ret = -EBUSY;
        }

      nxmutex_lock(&dev->recvlock);
      if (!circbuf_is_empty(&dev->circbuf))
        {
          eventset |= POLLIN;
        }

      nxmutex_unlock(&dev->recvlock);
      eventset |= POLLOUT;

      poll_notify(&fds, 1, eventset);
    }
  else if (fds->priv != NULL)
    {
      for (i = 0; i < CONFIG_UART_BTH5_NPOLLWAITERS; i++)
        {
          if (fds == dev->fds[i])
            {
              dev->fds[i] = NULL;
              fds->priv = NULL;
              break;
            }
        }
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int
uart_bth5_register(FAR const char *path, FAR struct bt_driver_s *drv)
{
  FAR struct uart_bth5_s *dev;
  int ret;

  dev = kmm_zalloc(sizeof(struct uart_bth5_s));
  if (dev == NULL)
    {
      return -ENOMEM;
    }

  ret = circbuf_init(&dev->circbuf, NULL, CONFIG_UART_BTH5_RXBUFSIZE);
  if (ret < 0)
    {
      kmm_free(dev);
      return -ENOMEM;
    }

  dev->drv = drv;
  drv->receive = uart_bth5_receive;
  drv->priv = dev;

  nxsem_init(&dev->opensem, 0, 0);
  nxsem_init(&dev->recvsem, 0, 0);
  nxsem_init(&dev->acksem, 0, 0);
  nxmutex_init(&dev->sendlock);
  nxmutex_init(&dev->recvlock);
  nxmutex_init(&dev->openlock);

  h5_rx_reset(dev);

  ret = register_driver(path, &g_uart_bth5_ops, 0666, dev);
  if (ret < 0)
    {
      nxsem_destroy(&dev->recvsem);
      nxsem_destroy(&dev->opensem);
      nxsem_destroy(&dev->acksem);
      nxmutex_destroy(&dev->sendlock);
      nxmutex_destroy(&dev->openlock);
      nxmutex_destroy(&dev->recvlock);
      circbuf_uninit(&dev->circbuf);
      kmm_free(dev);
    }

  return ret;
}
