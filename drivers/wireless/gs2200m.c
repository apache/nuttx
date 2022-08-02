/****************************************************************************
 * drivers/wireless/gs2200m.c
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
 * gs2200m driver.
 *
 * See "GS2200MS2W Adapter Command Reference Guide" for the explanation
 * of AT commands. You can find the document at:
 * https://telit.com/m2m-iot-products/wifi-bluetooth-modules/wi-fi-gs2200m/
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <poll.h>

#include <nuttx/ascii.h>
#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/semaphore.h>
#include <nuttx/signal.h>
#include <nuttx/wireless/wireless.h>
#include <nuttx/wireless/gs2200m.h>
#include <nuttx/net/netdev.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(CONFIG_SCHED_WORKQUEUE)
#  error "Worker thread support is required (CONFIG_SCHED_WORKQUEUE)"
#endif

#ifndef MIN
#  define MIN(a,b)  (((a) < (b)) ? (a) : (b))
#endif

#define GS2200MWORK LPWORK

#define SPI_MAXFREQ  CONFIG_WL_GS2200M_SPI_FREQUENCY
#define NRESPMSG     (16 + 2)

#define BULK_CMD_HDR_SIZE_WITH_GUARD 36

#define MAX_PKT_LEN  1500
#define MAX_PAYLOAD  (MAX_PKT_LEN - BULK_CMD_HDR_SIZE_WITH_GUARD)
#define MAX_NOTIF_Q  18 /* for 16 sockets and disasso event + dummy */

#define WR_REQ       0x01
#define RD_REQ       0x02
#define DT_FROM_MCU  0x03

#define WR_RESP_OK   0x11
#define RD_RESP_OK   0x12
#define WR_RESP_NOK  0x13
#define RD_RESP_NOK  0x14

#define LED_GPIO     30
#define HAL_TIMEOUT  5000000  /* in us */
#define WR_MAX_RETRY 100

#define PORT_START   50000
#define PORT_END     59999

#define SEC_MODE_WEP     2
#define SEC_MODE_WPA2PSK 8

#define BULK_THRESHOLD (1024 * 8)

/****************************************************************************
 * Private Data Types
 ****************************************************************************/

enum pkt_state_e
{
  PKT_START = 0,
  PKT_EVENT,
  PKT_ESC_START,
  PKT_BULK_DATA_TCP,
  PKT_BULK_DATA_UDP,
};

enum spi_status_e
{
  SPI_OK = 0,
  SPI_ERROR,
  SPI_TIMEOUT
};

enum pkt_type_e
{
  TYPE_OK = 0,
  TYPE_ERROR = 1,
  TYPE_DISCONNECT = 2,
  TYPE_CONNECT = 3,
  TYPE_BOOT_MSG = 4,
  TYPE_BULK_DATA_TCP = 5,
  TYPE_BULK_DATA_UDP = 6,
  TYPE_FAIL = 7,
  TYPE_TIMEOUT = 8,
  TYPE_SPI_ERROR = 9,
  TYPE_DISASSOCIATE = 10,
  TYPE_UNMATCH = 11,
};

struct evt_code_s
{
  FAR const char *str;
  enum pkt_type_e code;
};

struct pkt_dat_s
{
  struct dq_entry_s  dq;
  enum pkt_type_e    type;
  struct sockaddr_in addr;
  char               cid;
  uint8_t            n;
  FAR char           *msg[NRESPMSG];
  uint16_t           remain; /* bulk data length to be read */
  uint16_t           len;    /* bulk data length */
  FAR uint8_t        *data;  /* bulk data */
};

struct pkt_ctx_s
{
  enum pkt_type_e  type;
  enum pkt_state_e state;
  FAR uint8_t      *ptr;
  FAR uint8_t      *head;
  char             cid;
  uint16_t         dlen;
};

struct notif_q_s
{
  uint8_t  rpos;
  uint8_t  wpos;
  uint8_t  count;
  uint32_t inuse;
  char     cids[MAX_NOTIF_Q];
};

struct gs2200m_dev_s
{
  FAR char             *path;
  FAR struct pollfd    *pfd;
  struct notif_q_s     notif_q;
  FAR struct spi_dev_s *spi;
  struct work_s        irq_work;
  sem_t                dev_sem;
  bool                 int_enabled;
  dq_queue_t           pkt_q[16];
  uint16_t             pkt_q_cnt[16];
  uint32_t             valid_cid_bits;
  uint32_t             aip_cid_bits;
  uint32_t             total_bulk;
  uint8_t              tx_buff[MAX_PKT_LEN];
  struct net_driver_s  net_dev;
  uint8_t              op_mode;
  FAR const struct gs2200m_lower_s *lower;
  bool                 disassociate_flag;
  struct gs2200m_assoc_msg reconnect_msg;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods */

static ssize_t gs2200m_read(FAR struct file *filep, FAR char *buff,
                            size_t len);
static ssize_t gs2200m_write(FAR struct file *filep, FAR const char *buff,
                             size_t len);
static int     gs2200m_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg);
static int     gs2200m_poll(FAR struct file *filep, FAR struct pollfd *fds,
                            bool setup);

/* Interrupt handler and work queue handler */

static int     gs2200m_irq(int irq, FAR void *context, FAR void *arg);
static void    gs2200m_irq_worker(FAR void *arg);

static void _remove_all_pkt(FAR struct gs2200m_dev_s *dev, uint8_t c);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This the vtable that supports the character driver interface. */

static const struct file_operations g_gs2200m_fops =
{
  NULL,          /* open */
  NULL,          /* close */
  gs2200m_read,  /* read */
  gs2200m_write, /* write */
  NULL,          /* seek */
  gs2200m_ioctl, /* ioctl */
  gs2200m_poll   /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL         /* unlink */
#endif
};

static struct evt_code_s _evt_table[] =
{
  {"OK", TYPE_OK},
  {"Disassociation Event", TYPE_DISASSOCIATE},
  {"ERROR", TYPE_ERROR},
  {"DISCONNECT", TYPE_DISCONNECT},
  {"CONNECT ", TYPE_CONNECT},
  {"Serial2WiFi APP", TYPE_BOOT_MSG}
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _spi_err_to_pkt_type
 ****************************************************************************/

static enum pkt_type_e _spi_err_to_pkt_type(enum spi_status_e s)
{
  enum pkt_type_e r;

  switch (s)
    {
      case SPI_OK:
        r = TYPE_OK;
        break;

      case SPI_ERROR:
        r = TYPE_SPI_ERROR;
        break;

      case SPI_TIMEOUT:
        r = TYPE_TIMEOUT;
        break;

      default:
        r = TYPE_UNMATCH;
        ASSERT(false);
    }

  return r;
}

/****************************************************************************
 * Name: _cid_to_uint8
 ****************************************************************************/

static uint8_t _cid_to_uint8(char c)
{
  uint8_t ret;

  if ('0' <= c && c <= '9')
    {
      ret = c - '0';
    }
  else if ('a' <= c && c <= 'f')
    {
      ret = (c - 'a') + 10;
    }
  else if (c == DISASSOCIATION_CID)
    {
      ret = 16;
    }
  else
    {
      ret = 0xff;
      ASSERT(false);
    }

  return ret;
}

/****************************************************************************
 * Name: _to_ascii_char
 ****************************************************************************/

static void _to_ascii_char(uint16_t num, char *str)
{
  DEBUGASSERT(num <= 2032); /* See Table 20 */
  snprintf(str, 6, "%04d", num);
}

/****************************************************************************
 * Name: _to_uint16
 ****************************************************************************/

static uint16_t _to_uint16(char *str)
{
  uint16_t ret = 0;
  int n;

  n = sscanf(str, "%04hu", &ret);
  ASSERT(1 == n);
  return ret;
}

/****************************************************************************
 * Name: _enable_cid
 ****************************************************************************/

static bool _enable_cid(uint32_t *cid_bits, char cid, bool on)
{
  uint32_t mask = 1 << _cid_to_uint8(cid);
  bool     ret  = true;

  if (on)
    {
      if (*cid_bits & mask)
        {
          ret = false; /* already set */
        }
      else
        {
          *cid_bits |= mask;
        }
    }
  else
    {
      if (*cid_bits & mask)
        {
          *cid_bits &= ~mask;
        }
      else
        {
          ret = false; /* not set yet */
        }
    }

  return ret;
}

/****************************************************************************
 * Name: _cid_is_set
 ****************************************************************************/

static bool _cid_is_set(uint32_t *cid_bits, char cid)
{
  uint32_t mask = 1 << _cid_to_uint8(cid);

  if (*cid_bits & mask)
    {
      return true;
    }
  else
    {
      return false;
    }
}

/****************************************************************************
 * Name: _notif_q_count()
 ****************************************************************************/

static uint8_t _notif_q_count(FAR struct gs2200m_dev_s *dev)
{
  return dev->notif_q.count;
}

/****************************************************************************
 * Name: _notif_q_push()
 ****************************************************************************/

static void _notif_q_push(FAR struct gs2200m_dev_s *dev, char cid)
{
  ASSERT(MAX_NOTIF_Q > dev->notif_q.count);

  /* Set cid in _notif_q.inuse */

  bool ret = _enable_cid(&dev->notif_q.inuse, cid, true);

  if (false == ret)
    {
      /* already registered */

      return;
    }

  dev->notif_q.cids[dev->notif_q.wpos % MAX_NOTIF_Q] = cid;
  dev->notif_q.wpos++;
  dev->notif_q.count++;

  if (dev->pfd)
    {
      /* If poll() waits and cid has been pushed to the queue, notify  */

      dev->pfd->revents |= POLLIN;
      nxsem_post(dev->pfd->sem);
    }

  wlinfo("+++ pushed %c count=%d\n", cid, dev->notif_q.count);
}

/****************************************************************************
 * Name: _notif_q_pop()
 ****************************************************************************/

static char _notif_q_pop(FAR struct gs2200m_dev_s *dev)
{
  char cid;

  ASSERT(0 < dev->notif_q.count);

  cid = dev->notif_q.cids[dev->notif_q.rpos % MAX_NOTIF_Q];
  dev->notif_q.rpos++;
  dev->notif_q.count--;

  /* Clear cid in _notif_q.inuse */

  _enable_cid(&dev->notif_q.inuse, cid, false);

  return cid;
}

/****************************************************************************
 * Name: _push_data_to_pkt
 ****************************************************************************/

static void _push_data_to_pkt(struct pkt_dat_s *pkt, uint8_t data)
{
  ASSERT(pkt->len < MAX_PKT_LEN);

  pkt->data[pkt->len++] = data;
  pkt->remain = pkt->len;
}

/****************************************************************************
 * Name: _release_pkt_dat
 ****************************************************************************/

static void _release_pkt_dat(FAR struct gs2200m_dev_s *dev,
                             FAR struct pkt_dat_s *pkt_dat)
{
  int i;

  for (i = 0; i < pkt_dat->n; i++)
    {
      kmm_free(pkt_dat->msg[i]);
    }

  if (pkt_dat->len)
    {
      kmm_free(pkt_dat->data);

      if (pkt_dat->type == TYPE_BULK_DATA_TCP ||
          pkt_dat->type == TYPE_BULK_DATA_UDP)
        {
          /* Update total bulk data size */

          ASSERT(dev->total_bulk >= pkt_dat->len);
          dev->total_bulk -= pkt_dat->len;
        }
    }

  pkt_dat->n   = 0;
  pkt_dat->len = 0;
}

/****************************************************************************
 * Name: _check_pkt_q_cnt
 ****************************************************************************/

static void _check_pkt_q_cnt(FAR struct gs2200m_dev_s *dev, char cid)
{
  uint8_t cnt;

  cnt = dev->pkt_q_cnt[_cid_to_uint8(cid)];

  if (0 != cnt)
    {
      wlinfo("--- _pkt_p_cnt[%c]=%d\n", cid, cnt);
    }
}

/****************************************************************************
 * Name: _check_pkt_q_empty
 ****************************************************************************/

static void _check_pkt_q_empty(FAR struct gs2200m_dev_s *dev, char cid)
{
  uint8_t c = _cid_to_uint8(cid);
  FAR struct pkt_dat_s *pkt_dat;

  if (0 != dev->pkt_q_cnt[c])
    {
      pkt_dat = (FAR struct pkt_dat_s *)dq_peek(&dev->pkt_q[c]);

      while (pkt_dat)
        {
          wlerr("=== error: found (cid=%c type=%d msg[0]=%s|)\n",
                cid, pkt_dat->type, pkt_dat->msg[0]);
          pkt_dat = (FAR struct pkt_dat_s *)pkt_dat->dq.flink;

          if (_cid_is_set(&dev->valid_cid_bits, cid))
            {
              wlerr("+++ error: cid=%c is still active !!!\n", cid);
            }

          /* NOTE: force to disable the cid to remove */

          _enable_cid(&dev->valid_cid_bits, cid, false);
        }

      _remove_all_pkt(dev, c);
    }
}

/****************************************************************************
 * Name: _control_pkt_q
 ****************************************************************************/

static bool _control_pkt_q(FAR struct gs2200m_dev_s *dev)
{
  bool over = BULK_THRESHOLD < dev->total_bulk ? true : false;

  /* TODO: should enable again if disabled for long time */

  if (dev->int_enabled && over)
    {
      wlinfo("--- disable irq\n");
      dev->int_enabled = false;
      dev->lower->disable();
    }

  if (!dev->int_enabled && !over)
    {
      wlinfo("--- enable irq again\n");
      dev->lower->enable();
      dev->int_enabled = true;
    }

  return over;
}

/****************************************************************************
 * Name: _remove_and_free_pkt
 ****************************************************************************/

static void _remove_and_free_pkt(FAR struct gs2200m_dev_s *dev, uint8_t c)
{
  FAR struct pkt_dat_s *pkt_dat;

  /* Decrement _pkt_q_cnt before remove */

  ASSERT(0 < dev->pkt_q_cnt[c]);
  dev->pkt_q_cnt[c]--;

  /* Remove a packet from the queue */

  pkt_dat = (FAR struct pkt_dat_s *)dq_remfirst(&dev->pkt_q[c]);
  ASSERT(pkt_dat);

  /* Release the packet */

  _release_pkt_dat(dev, pkt_dat);
  kmm_free(pkt_dat);
}

/****************************************************************************
 * Name: _remove_all_pkt
 ****************************************************************************/

static void _remove_all_pkt(FAR struct gs2200m_dev_s *dev, uint8_t c)
{
  FAR struct pkt_dat_s *pkt_dat;
  uint32_t mask;

  mask = 1 << c;
  ASSERT(0 == (dev->valid_cid_bits & mask));

  ASSERT(dev->pkt_q_cnt[c] == dq_count(&dev->pkt_q[c]));

  /* Remove all packets for this cid */

  pkt_dat = (FAR struct pkt_dat_s *)dq_peek(&dev->pkt_q[c]);

  while (pkt_dat)
    {
      _remove_and_free_pkt(dev, c);

      /* Check the next */

      pkt_dat = (FAR struct pkt_dat_s *)dq_peek(&dev->pkt_q[c]);
    }
}

/****************************************************************************
 * Name: _copy_data_from_pkt
 ****************************************************************************/

static bool _copy_data_from_pkt(FAR struct gs2200m_dev_s *dev,
                                struct gs2200m_recv_msg *msg)
{
  FAR struct pkt_dat_s *pkt_dat;
  uint8_t  c = _cid_to_uint8(msg->cid);
  uint16_t len;
  uint16_t off;
  bool ret = true;

  /* Peek a packet from the queue and check the remaining size */

  pkt_dat = (FAR struct pkt_dat_s *)dq_peek(&dev->pkt_q[c]);
  ASSERT(pkt_dat);

  wlinfo("+++ msg(req=%d:len=%d) pkt_data(t=%d:remain=%d)\n",
         msg->reqlen, msg->len, pkt_dat->type, pkt_dat->remain);

  if (msg->len && TYPE_DISCONNECT == pkt_dat->type)
    {
      /* Treat the packet separately */

      ret = false;
      goto errout;
    }

  /* Copy the pkt data to msg buffer up to MIN(request - len, remain) */

  len = MIN(msg->reqlen - msg->len, pkt_dat->remain);
  off = pkt_dat->len - pkt_dat->remain;
  memcpy(msg->buf + msg->len, pkt_dat->data + off, len);
  msg->len += len;
  msg->type = pkt_dat->type;

  /* Update the remaining size. If the remaining size is 0.
   * Remove the packet from the queue and free it.
   */

  pkt_dat->remain -= len;

  if (0 == pkt_dat->remain || TYPE_BULK_DATA_UDP == pkt_dat->type)
    {
      _remove_and_free_pkt(dev, c);
    }

errout:

  if (!msg->is_tcp)
    {
      /* Copy the source address and port */

      memcpy(&msg->addr, &pkt_dat->addr, sizeof(pkt_dat->addr));

      /* Set the address family
       * NOTE: gs2200m only supports IPv4
       */

      msg->addr.sin_family = AF_INET;

      /* In udp case, treat the packet separately */

      ret = false;
    }

  return ret;
}

/****************************************************************************
 * Name: gs2200m_lock
 ****************************************************************************/

static int gs2200m_lock(FAR struct gs2200m_dev_s *dev)
{
  return nxsem_wait_uninterruptible(&dev->dev_sem);
}

/****************************************************************************
 * Name: gs2200m_unlock
 ****************************************************************************/

static void gs2200m_unlock(FAR struct gs2200m_dev_s *dev)
{
  nxsem_post(&dev->dev_sem);
}

/****************************************************************************
 * Name: gs2200m_read
 ****************************************************************************/

static ssize_t gs2200m_read(FAR struct file *filep, FAR char *buffer,
                            size_t len)
{
  FAR struct inode *inode;
  FAR struct gs2200m_dev_s *dev;
  int ret;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct gs2200m_dev_s *)inode->i_private;

  ASSERT(1 == len);

  ret = nxsem_wait(&dev->dev_sem);
  if (ret < 0)
    {
      /* Return if a signal is received or if the the task was canceled
       * while we were waiting.
       */

      return ret;
    }

  ASSERT(0 < _notif_q_count(dev));
  char cid = _notif_q_pop(dev);

  wlinfo("---- cid=%c (notif_q_cnt=%d)\n", cid, _notif_q_count(dev));

  /* Copy the cid to the buffer */

  memcpy(buffer, &cid, sizeof(cid));

  gs2200m_unlock(dev);
  return 1;
}

/****************************************************************************
 * Name: gs2200m_write
 ****************************************************************************/

static ssize_t gs2200m_write(FAR struct file *filep, FAR const char *buffer,
                             size_t len)
{
  return 0;  /* REVISIT:  Zero is not a legal return value from write() */
}

/****************************************************************************
 * Name: gs2200m_spi_init
 ****************************************************************************/

static int gs2200m_spi_init(FAR struct gs2200m_dev_s *dev)
{
  SPI_LOCK(dev->spi, true);

  /* SPI settings (mode1/8bits/max freq) */

  SPI_SETMODE(dev->spi, SPIDEV_MODE1);
  SPI_SETBITS(dev->spi, 8);
  SPI_SETFREQUENCY(dev->spi, SPI_MAXFREQ);

  SPI_LOCK(dev->spi, false);
  return 0;
}

/****************************************************************************
 * Name: _checksum
 * NOTE: See 3.2.2.3 Annexure - HI Frame Format (From Host)
 ****************************************************************************/

static uint8_t _checksum(uint8_t *p, uint8_t len)
{
  uint8_t  i;
  uint32_t chksum = 0x0;

  for (i = 0; i < len; i++, p++)
    {
      chksum += *p;
    }

  chksum ^= ~0x0;
  return chksum;
}

/****************************************************************************
 * Name: _prepare_header
 * NOTE: See 3.2.2.3 Annexure - HI Frame Format (From Host)
 ****************************************************************************/

static void _prepare_header(uint8_t *p, uint16_t len, uint8_t class)
{
  *(p + 0) = 0xa5; /* SOF: start of frame */
  *(p + 1) = class;
  *(p + 2) = 0x0; /* reserved */
  *(p + 3) = 0x0; /* additional info */
  *(p + 4) = 0x0; /* additional info */
  *(p + 5) = (uint8_t)len;
  *(p + 6) = (uint8_t)(len >> 8);
  *(p + 7) = _checksum(p + 1, 6); /* exclude SOF */
}

/****************************************************************************
 * Name: _write_data
 ****************************************************************************/

static void _write_data(FAR struct gs2200m_dev_s *dev,
                        FAR uint8_t *buf,
                        FAR uint16_t len)
{
  SPI_SELECT(dev->spi, SPIDEV_WIRELESS(0), true);
  SPI_SNDBLOCK(dev->spi, buf, len);
  SPI_SELECT(dev->spi, SPIDEV_WIRELESS(0), false);
}

/****************************************************************************
 * Name: _read_data
 * NOTE: See 3.2.2.2 SPI Command Response (SPI-DMA)
 ****************************************************************************/

static void _read_data(FAR struct gs2200m_dev_s *dev,
                       FAR uint8_t  *buff,
                       FAR uint16_t len)
{
  memset(buff, 0, len);

  SPI_SELECT(dev->spi, SPIDEV_WIRELESS(0), true);
  SPI_RECVBLOCK(dev->spi, buff, len);
  SPI_SELECT(dev->spi, SPIDEV_WIRELESS(0), false);
}

/****************************************************************************
 * Name: _read_data_len
 ****************************************************************************/

static uint16_t _read_data_len(FAR struct gs2200m_dev_s *dev)
{
  uint8_t  hdr[8];
  uint8_t  res[8];
  uint16_t len = 0;
  int n = 0;

  /* Prepare header */

  _prepare_header(hdr, MAX_PKT_LEN, RD_REQ);

retry:

  /* Send the header read request */

  _write_data(dev, hdr, sizeof(hdr));

  /* Wait for data ready */

  while (!dev->lower->dready(NULL))
    {
      /* TODO: timeout */
    }

  /* NOTE: busy wait 50us
   * workaround to avoid an invalid frame response
   */

  up_udelay(50);

  /* Read frame response */

  _read_data(dev, res, sizeof(res));

  /* In case of NOK, retry */

  if (RD_RESP_NOK == res[1])
    {
      wlwarn("*** warning: RD_RESP_NOK received.. retrying. (n=%d)\n", n);
      nxsig_usleep(100 * 1000);
      n++;
      goto retry;
    }

  ASSERT(RD_RESP_OK == res[1]);

  /* Retrieve the length */

  len = ((uint16_t)res[6] << 8) + (uint16_t)res[5];

  return len;
}

/****************************************************************************
 * Name: gs2200m_hal_write
 * NOTE: See Figure 13,14 Transferring data from MCU to GS node
 ****************************************************************************/

enum spi_status_e gs2200m_hal_write(FAR struct gs2200m_dev_s *dev,
                                    const void *data,
                                    uint16_t txlen)
{
  uint8_t *tx = (uint8_t *)data;
  uint8_t  hdr[8];
  uint8_t  res[8];
  int n = 0;

  /* Prepare header */

  _prepare_header(hdr, txlen, WR_REQ);

retry:

  /* 1. Send the first 4bytes of WRITE_REQUEST */

  _write_data(dev, hdr, sizeof(hdr) / 2);

  /* 2. Delay 3.2usec (NOTE: here we specify 4us) */

  up_udelay(4);

  /* Check if a pending interrupt exists */

  if (dev->lower->dready(NULL))
    {
      wlwarn("*** warning: gs2200m is busy.. retrying. (n=%d)\n", n);

      if (!work_available(&dev->irq_work))
        {
          wlwarn("*** warning: there is still pending work ****\n");
        }
      else
        {
          /* NOTE: Disable gs2200m irq before calling work_queue()
           * This is the same sequence in the irq handler
           */

          dev->lower->disable();

          work_queue(GS2200MWORK, &dev->irq_work, gs2200m_irq_worker,
                     (FAR void *)dev, 0);
        }

      nxsig_usleep(100 * 1000);
      n++;
      goto retry;
    }

  /* 3. Send remaining 4bytes of the WRITE_REQUEST */

  _write_data(dev, hdr + (sizeof(hdr) / 2), sizeof(hdr) / 2);

  /* 4. Wait for dready status (GPIO37 goes high) */

  while (!dev->lower->dready(NULL))
    {
      /* TODO: timeout */
    }

  /* 5 Read 8bytes of WRITE_RESPONSE */

  _read_data(dev, res, sizeof(res));

  /* In case of NOK or 0x0, retry */

  if (WR_RESP_NOK == res[1] || 0x0 == res[1])
    {
      wlwarn("*** warning: 0x%x received.. retrying. (n=%d)\n",
             res[1], n);
      nxsig_usleep(10 * 1000);

      if (WR_MAX_RETRY < n)
        {
          return SPI_TIMEOUT;
        }

      n++;
      goto retry;
    }

  ASSERT(WR_RESP_OK == res[1]);

  /* Prepare header */

  _prepare_header(hdr, txlen, DT_FROM_MCU);

  /* 6. Send 8bytes of data header */

  _write_data(dev, hdr, sizeof(hdr));

  /* 7. Send actual data */

  _write_data(dev, tx, txlen);

  return SPI_OK;
}

/****************************************************************************
 * Name: gs2200m_hal_read
 ****************************************************************************/

enum spi_status_e gs2200m_hal_read(FAR struct gs2200m_dev_s *dev,
                                   FAR uint8_t *data,
                                   FAR uint16_t *len)
{
  enum spi_status_e r = SPI_OK;
  uint8_t hdr[8];
  int i;

  /* NOTE: need to wait for data ready even if we use irq */

  for (i = 0; i < HAL_TIMEOUT; i++)
    {
      if (dev->lower->dready(NULL))
        {
          break;
        }

      /* Busy wait 1us */

      up_udelay(1);
    }

  if (HAL_TIMEOUT == i)
    {
      wlerr("***** error: timeout!\n");
      r = SPI_TIMEOUT;
      goto errout;
    }

  /* Send READ_REQUEST then receive READ_RESPONSE
   * to get how many bytes we should read
   */

  *len = _read_data_len(dev);

  wlinfo("+++++ (len=%d)\n", *len);

  /* Check the length */

  ASSERT(0 < *len);

  /* Read data header */

  _read_data(dev, hdr, sizeof(hdr));

  /* Read the actual data */

  _read_data(dev, data, *len);

errout:
  return r;
}

/****************************************************************************
 * Name: _check_evt
 ****************************************************************************/

enum pkt_type_e _check_evt(FAR const char *buff)
{
  int i = 0;

  for (i = 0; i < sizeof(_evt_table) / sizeof(struct evt_code_s); i++)
    {
      if (strstr(buff, _evt_table[i].str))
        {
          return _evt_table[i].code;
        }
    }

  wlinfo("+++++ %s +++++\n", buff);
  return TYPE_UNMATCH;
}

/****************************************************************************
 * Name: _parse_pkt_in_s0
 ****************************************************************************/

static void _parse_pkt_in_s0(FAR struct pkt_ctx_s *pkt_ctx,
                             FAR struct pkt_dat_s *pkt_dat)
{
  switch (*(pkt_ctx->ptr))
    {
    case ASCII_CR:
    case ASCII_LF:
      break;

    case ASCII_ESC:
      pkt_ctx->state = PKT_ESC_START;
      break;

    default:
      pkt_ctx->head = pkt_ctx->ptr;
      pkt_ctx->state = PKT_EVENT;
      break;
    }
}

/****************************************************************************
 * Name: _parse_pkt_in_s1
 ****************************************************************************/

static void _parse_pkt_in_s1(FAR struct pkt_ctx_s *pkt_ctx,
                             FAR struct pkt_dat_s *pkt_dat)
{
  int msize;
  int n;

  if (ASCII_LF != *(pkt_ctx->ptr))
    {
      return;
    }

  ASSERT(pkt_ctx->ptr > pkt_ctx->head);
  msize = pkt_ctx->ptr - pkt_ctx->head;

  char *msg = (char *)kmm_calloc(msize + 1, 1);
  ASSERT(msg);

  memcpy(msg, pkt_ctx->head, msize);

  pkt_ctx->type = _check_evt(msg);

  if (pkt_ctx->type == TYPE_DISCONNECT)
    {
      ASSERT(pkt_dat);

      n = sscanf(msg, "DISCONNECT %c", &(pkt_dat->cid));
      ASSERT(1 == n);

      wlinfo("+++++ msg=%s| cid=%c\n", msg, pkt_dat->cid);
    }
  else if (pkt_ctx->type == TYPE_CONNECT)
    {
      ASSERT(pkt_dat);

      /* NOTE: CONNECT <server cid> <new cid> <ip> <address> */

      n = sscanf(msg, "CONNECT %c", &(pkt_dat->cid));
      DEBUGASSERT(1 == n);

      wlinfo("+++++ msg=%s|\n", msg);
    }

  if (pkt_dat)
    {
      /* If specified, store the msg pointer to pkt_dat */

      wlinfo("+++++ %d:(msize=%d, msg=%s|)\n", pkt_dat->n, msize, msg);
      ASSERT(pkt_dat->n < NRESPMSG);
      pkt_dat->msg[pkt_dat->n++] = msg;
    }
  else
    {
      wlinfo("+++++ (msize=%d, msg=%s|)\n", msize, msg);
      kmm_free(msg);
    }

  pkt_ctx->head = pkt_ctx->ptr + 1;

  if (TYPE_UNMATCH != pkt_ctx->type)
    {
      pkt_ctx->state = PKT_START;
    }
}

/****************************************************************************
 * Name: _parse_pkt_in_s2 (ESC detected)
 ****************************************************************************/

static void _parse_pkt_in_s2(FAR struct pkt_ctx_s *pkt_ctx,
                             FAR struct pkt_dat_s *pkt_dat)
{
  ASSERT(pkt_ctx && pkt_ctx->ptr);

  char c = (char)*(pkt_ctx->ptr);

  if ('Z' == c)
    {
      wlinfo("** <ESC>Z\n");

      /* NOTE: See 7.5.3.2 Bulk Data Handling */

      pkt_ctx->state = PKT_BULK_DATA_TCP;
    }
  else if ('y' == c)
    {
      wlinfo("** <ESC>y\n");

      /* NOTE: See 7.5.3.2 Bulk Data Handling */

      pkt_ctx->state = PKT_BULK_DATA_UDP;
    }
  else if ('F' == c)
    {
      wlwarn("** <ESC>F\n");

      /* NOTE: See Table 6 Data Handling Responses at Completion */

      pkt_ctx->state = PKT_START;
      pkt_ctx->type  = TYPE_FAIL;
    }
  else
    {
      wlerr("** <ESC>%c not supported\n", c);
      ASSERT(false);
    }
}

/****************************************************************************
 * Name: _parse_pkt_in_s3 (BULK data for TCP)
 ****************************************************************************/

static void _parse_pkt_in_s3(FAR struct pkt_ctx_s *pkt_ctx,
                             FAR struct pkt_dat_s *pkt_dat)
{
  ASSERT(pkt_dat);

  if ('z' == pkt_ctx->cid)
    {
      /* Proceed ptr to obtain data length
       * NOTE: <CID><Data Length xxxx 4 ascii char>
       */

      /* Read CID */

      pkt_ctx->cid = (char)*(pkt_ctx->ptr);
      pkt_ctx->ptr++;

      pkt_dat->cid = pkt_ctx->cid;

      /* Read data length */

      pkt_ctx->dlen = _to_uint16((char *)pkt_ctx->ptr);
      pkt_ctx->ptr += 3;

      wlinfo("dlen=%d\n", pkt_ctx->dlen);

      /* Allocate memory for the packet */

      pkt_dat->data = kmm_calloc(pkt_ctx->dlen, 1);
      ASSERT(pkt_dat->data);
    }
  else
    {
      _push_data_to_pkt(pkt_dat, *(pkt_ctx->ptr));

      pkt_ctx->dlen--;

      if (0 == pkt_ctx->dlen)
        {
          pkt_ctx->state = PKT_START;
          pkt_ctx->type  = TYPE_BULK_DATA_TCP;
        }
    }
}

/****************************************************************************
 * Name: _parse_pkt_in_s4 (BULK data for UDP: see Table 217)
 ****************************************************************************/

static void _parse_pkt_in_s4(FAR struct pkt_ctx_s *pkt_ctx,
                             FAR struct pkt_dat_s *pkt_dat)
{
  char addr[17];
  char port[6];
  int n;

  ASSERT(pkt_dat);

  if ('z' == pkt_ctx->cid)
    {
      /* <CID><address><sp><port><tab><Data Length xxxx 4 ascii char> */

      /* Read CID */

      pkt_ctx->cid = (char)*(pkt_ctx->ptr);
      pkt_ctx->ptr++;

      pkt_dat->cid = pkt_ctx->cid;

      /* Read address and port */

      memset(addr, 0, sizeof(addr));
      memset(port, 0, sizeof(port));
      n = sscanf((char *)pkt_ctx->ptr, "%s %s\t", addr, port);
      ASSERT(2 == n);

      wlinfo("from (%s:%s)\n", addr, port);

      inet_aton(addr, &pkt_dat->addr.sin_addr);
      pkt_dat->addr.sin_port = HTONS((uint16_t)atoi(port));

      /* Skip until data length */

      for (; '\t' != (char)*(pkt_ctx->ptr); pkt_ctx->ptr++);
      pkt_ctx->ptr++;

      /* Read data length */

      pkt_ctx->dlen = _to_uint16((char *)pkt_ctx->ptr);
      pkt_ctx->ptr += 3;

      wlinfo("dlen=%d\n", pkt_ctx->dlen);

      /* Allocate memory for the packet */

      pkt_dat->data = kmm_calloc(pkt_ctx->dlen, 1);
      ASSERT(pkt_dat->data);
    }
  else
    {
      _push_data_to_pkt(pkt_dat, *(pkt_ctx->ptr));

      pkt_ctx->dlen--;

      if (0 == pkt_ctx->dlen)
        {
          pkt_ctx->state = PKT_START;
          pkt_ctx->type  = TYPE_BULK_DATA_UDP;
        }
    }
}

/****************************************************************************
 * Name: _parse_pkt
 ****************************************************************************/

static enum pkt_type_e _parse_pkt(FAR uint8_t *p, uint16_t len,
                                  FAR struct pkt_dat_s *pkt_dat)
{
  struct pkt_ctx_s pkt_ctx;

  /* Initialize pkt_ctx */

  pkt_ctx.type  = TYPE_UNMATCH;
  pkt_ctx.state = PKT_START;
  pkt_ctx.head  = NULL;
  pkt_ctx.cid   = 'z';
  pkt_ctx.dlen  = 0;

  for (pkt_ctx.ptr = p; pkt_ctx.ptr < (p + len); pkt_ctx.ptr++)
    {
      switch (pkt_ctx.state)
        {
        case PKT_START:
          _parse_pkt_in_s0(&pkt_ctx, pkt_dat);
          break;

        case PKT_EVENT:
          _parse_pkt_in_s1(&pkt_ctx, pkt_dat);
          break;

        case PKT_ESC_START:
          _parse_pkt_in_s2(&pkt_ctx, pkt_dat);
          break;

        case PKT_BULK_DATA_TCP:
          _parse_pkt_in_s3(&pkt_ctx, pkt_dat);
          break;

        case PKT_BULK_DATA_UDP:
          _parse_pkt_in_s4(&pkt_ctx, pkt_dat);
          break;

        default:
          ASSERT(false);
          break;
        }
    }

  return pkt_ctx.type;
}

/****************************************************************************
 * Name: _dup_pkt_dat_and_notify
 ****************************************************************************/

static void _dup_pkt_dat_and_notify(FAR struct gs2200m_dev_s *dev,
                                    FAR struct pkt_dat_s *pkt_dat0)
{
  struct pkt_dat_s *pkt_dat;
  uint8_t c;

  /* Only bulk data */

  ASSERT(pkt_dat0->data && (0 == pkt_dat0->n));

  /* Allocate a new pkt_dat */

  pkt_dat = (FAR struct pkt_dat_s *)kmm_malloc(sizeof(struct pkt_dat_s));
  ASSERT(pkt_dat);

  /* Copy pkt_dat0 to pkt_dat */

  memcpy(pkt_dat, pkt_dat0, sizeof(struct pkt_dat_s));

  /* Allocate bulk data and copy */

  pkt_dat->data = (FAR uint8_t *)kmm_malloc(pkt_dat0->len);
  ASSERT(pkt_dat->data);
  memcpy(pkt_dat->data, pkt_dat0->data, pkt_dat0->len);

  /* Convert cid to c */

  c = _cid_to_uint8(pkt_dat->cid);

  /* Add the pkt_dat to the pkt_q */

  dq_addlast((FAR dq_entry_t *)pkt_dat, &dev->pkt_q[c]);
  dev->pkt_q_cnt[c]++;

  /* NOTE: total_bulk must be updated
   * Usually, total_bulk is updated in gs2200m_recv_pkt()
   * However, the pkt_dat was duplicated from pkt_dat0
   * So it needs to be updated, otherwise it will cause ASSERT
   */

  dev->total_bulk += pkt_dat->len;

  _notif_q_push(dev, pkt_dat->cid);
}

/****************************************************************************
 * Name: gs2200m_recv_pkt
 ****************************************************************************/

static enum pkt_type_e gs2200m_recv_pkt(FAR struct gs2200m_dev_s *dev,
                                        FAR struct pkt_dat_s *pkt_dat)
{
  enum pkt_type_e   t = TYPE_ERROR;
  enum spi_status_e s;
  uint16_t len;
  uint8_t *p;

  p = (uint8_t *)kmm_calloc(MAX_PKT_LEN, 1);
  ASSERT(p);

  s = gs2200m_hal_read(dev, p, &len);
  t = _spi_err_to_pkt_type(s);

  if (TYPE_OK != t)
    {
      goto errout;
    }

  wlinfo("+++ len=%d pkt_dat=%p\n", len, pkt_dat);

  /* Parse the received packet */

  t = _parse_pkt(p, len, pkt_dat);

  if (t == TYPE_DISCONNECT)
    {
      _check_pkt_q_cnt(dev, pkt_dat->cid);
    }

  if (t == TYPE_DISASSOCIATE)
    {
      dev->disassociate_flag = true;
    }

  if (pkt_dat)
    {
      pkt_dat->type = t;

      if (t == TYPE_BULK_DATA_TCP ||
          t == TYPE_BULK_DATA_UDP)
        {
          /* Update total bulk data size */

          dev->total_bulk += pkt_dat->len;
        }
    }

errout:
  kmm_free(p);
  return t;
}

/****************************************************************************
 * Name: gs2200m_send_cmd
 ****************************************************************************/

static enum pkt_type_e gs2200m_send_cmd(FAR struct gs2200m_dev_s *dev,
                                        FAR char *cmd,
                                        FAR struct pkt_dat_s *pkt_dat)
{
  enum spi_status_e s;
  enum pkt_type_e r = TYPE_SPI_ERROR;
  bool bulk = false;
  int n = 1;

  /* Disable gs2200m irq to poll dready */

  dev->lower->disable();

  wlinfo("+++ cmd=%s", cmd);

retry:

  s = gs2200m_hal_write(dev, cmd, strlen(cmd));
  r = _spi_err_to_pkt_type(s);

  if (TYPE_OK != r)
    {
      goto errout;
    }

retry_recv:

  r = gs2200m_recv_pkt(dev, pkt_dat);

  if ((TYPE_BULK_DATA_TCP == r || TYPE_BULK_DATA_UDP == r) && pkt_dat)
    {
      wlwarn("*** Found bulk data\n");

      /* Bulk data found in the response,
       * duplicate the packet and notify
       */

      _dup_pkt_dat_and_notify(dev, pkt_dat);

      /* release & initialize pkt_dat before retry */

      _release_pkt_dat(dev, pkt_dat);
      memset(pkt_dat, 0, sizeof(*pkt_dat));

      bulk = true;
      goto retry_recv;
    }

  /* NOTE: retry in case of errors */

  if ((TYPE_OK != r) && (0 <= --n))
    {
      if (pkt_dat)
        {
          /* release & initialize pkt_dat before retry */

          _release_pkt_dat(dev, pkt_dat);
          memset(pkt_dat, 0, sizeof(*pkt_dat));
        }

      wlwarn("*** retry cmd=%s (n=%d)\n", cmd, n);
      goto retry;
    }

errout:

  if (bulk)
    {
      wlwarn("*** Normal response r=%d\n", r);
    }

  /* Enable gs2200m irq again */

  dev->lower->enable();

  return r;
}

/****************************************************************************
 * Name: gs2200m_send_cmd2
 ****************************************************************************/

static enum pkt_type_e gs2200m_send_cmd2(FAR struct gs2200m_dev_s *dev,
                                         FAR char *cmd)
{
  struct pkt_dat_s pkt_dat;
  enum pkt_type_e  r;

  /* Initialize pkt_dat and send */

  memset(&pkt_dat, 0, sizeof(pkt_dat));
  r = gs2200m_send_cmd(dev, cmd, &pkt_dat);

  /* Release the pkt_dat */

  _release_pkt_dat(dev, &pkt_dat);
  return r;
}

/****************************************************************************
 * Name: gs2200m_set_opmode
 * NOTE: See 5.1.2 Operation Mode
 ****************************************************************************/

static enum pkt_type_e gs2200m_set_opmode(FAR struct gs2200m_dev_s *dev,
                                          uint8_t mode)
{
  enum pkt_type_e t;
  char cmd[20];

  snprintf(cmd, sizeof(cmd), "AT+WM=%d\r\n", mode);
  t = gs2200m_send_cmd2(dev, cmd);

  if (TYPE_OK == t)
    {
      dev->op_mode = mode;
    }

  return t;
}

/****************************************************************************
 * Name: gs2200m_get_mac
 * NOTE: See 4.5.2 Get MAC Address
 ****************************************************************************/

static enum pkt_type_e gs2200m_get_mac(FAR struct gs2200m_dev_s *dev)
{
  struct pkt_dat_s pkt_dat;
  enum pkt_type_e  r;
  uint32_t mac[6];
  char cmd[16];
  int n;

  /* Initialize pkt_dat and send command */

  memset(&pkt_dat, 0, sizeof(pkt_dat));
  snprintf(cmd, sizeof(cmd), "AT+NMAC=?\r\n");
  r = gs2200m_send_cmd(dev, cmd, &pkt_dat);

  if (r != TYPE_OK)
    {
      goto errout;
    }

  n = sscanf(pkt_dat.msg[0], "%2" PRIx32 ":%2" PRIx32 ":%2" PRIx32
                             ":%2" PRIx32 ":%2" PRIx32 ":%2" PRIx32,
             &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);
  DEBUGASSERT(n == 6);

  for (n = 0; n < 6; n++)
    {
      dev->net_dev.d_mac.ether.ether_addr_octet[n] = (uint8_t)mac[n];
    }

errout:
  _release_pkt_dat(dev, &pkt_dat);
  return r;
}

/****************************************************************************
 * Name: gs2200m_disassociate
 * NOTE: See 5.3.6 Disassociation
 ****************************************************************************/

static enum pkt_type_e gs2200m_disassociate(FAR struct gs2200m_dev_s *dev)
{
  return gs2200m_send_cmd2(dev, (char *)"AT+WD\r\n");
}

/****************************************************************************
 * Name: gs2200m_enable_dhcpc
 * NOTE: See 6.3 DHCP Client
 ****************************************************************************/

static enum pkt_type_e gs2200m_enable_dhcpc(FAR struct gs2200m_dev_s *dev,
                                            uint8_t on)
{
  char cmd[16];

  snprintf(cmd, sizeof(cmd), "AT+NDHCP=%d\r\n", on);
  return gs2200m_send_cmd2(dev, cmd);
}

/****************************************************************************
 * Name: gs2200m_calc_key
 * NOTE: See 5.3.3.5 WPA-PSK and WPA2-PSK Key Calculation
 ****************************************************************************/

static enum pkt_type_e gs2200m_calc_key(FAR struct gs2200m_dev_s *dev,
                                        FAR char *ssid, FAR char *psk)
{
  char cmd[80];

  snprintf(cmd, sizeof(cmd), "AT+WPAPSK=%s,%s\r\n", ssid, psk);
  return gs2200m_send_cmd2(dev, cmd);
}

/****************************************************************************
 * Name: gs2200m_set_security
 * NOTE: See 5.3.3.1 Security Setting
 ****************************************************************************/

static enum pkt_type_e gs2200m_set_security(FAR struct gs2200m_dev_s *dev,
                                            uint8_t mode)
{
  char cmd[16];

  snprintf(cmd, sizeof(cmd), "AT+WSEC=%d\r\n", mode);
  return gs2200m_send_cmd2(dev, cmd);
}

/****************************************************************************
 * Name: gs2200m_join_network
 * NOTE: See 5.3.5 Association
 ****************************************************************************/

static enum pkt_type_e gs2200m_join_network(FAR struct gs2200m_dev_s *dev,
                                            FAR char *ssid, uint8_t ch)
{
  struct pkt_dat_s pkt_dat;
  enum pkt_type_e  r;
  char cmd[64];
  char addr[3][17];
  int n;

  /* Initialize pkt_dat and send command */

  memset(&pkt_dat, 0, sizeof(pkt_dat));

  if (0 == dev->op_mode)
    {
      snprintf(cmd, sizeof(cmd), "AT+WA=%s\r\n", ssid);
    }
  else
    {
      /* In AP mode, we can specify channel to use */

      snprintf(cmd, sizeof(cmd), "AT+WA=%s,,%d\r\n", ssid, ch);
    }

  r = gs2200m_send_cmd(dev, cmd, &pkt_dat);

  if (r != TYPE_OK)
    {
      goto errout;
    }

  ASSERT(3 == pkt_dat.n);

  n = sscanf(pkt_dat.msg[1] + 1,
             " %[^:]:%[^:]:%[^ ]",
             addr[0], addr[1], addr[2]);
  ASSERT(3 == n);

  /* Set addresses to be shown with ifconfig */

  inet_aton(addr[0], (struct in_addr *)&dev->net_dev.d_ipaddr);
  inet_aton(addr[1], (struct in_addr *)&dev->net_dev.d_netmask);
  inet_aton(addr[2], (struct in_addr *)&dev->net_dev.d_draddr);

errout:
  _release_pkt_dat(dev, &pkt_dat);
  return r;
}

/****************************************************************************
 * Name: gs2200m_set_addresses
 * NOTE: See 6.4 IP Address
 ****************************************************************************/

static enum pkt_type_e gs2200m_set_addresses(FAR struct gs2200m_dev_s *dev,
                                             FAR const char *address,
                                             FAR const char *netmask,
                                             FAR const char *gateway)
{
  char cmd[100];

  snprintf(cmd, sizeof(cmd), "AT+NSET=%s,%s,%s\r\n",
           address, netmask, gateway);

  return gs2200m_send_cmd2(dev, cmd);
}

/****************************************************************************
 * Name: gs2200m_enable_dhcps
 * NOTE: See 6.5 DHCP Server
 ****************************************************************************/

static enum pkt_type_e gs2200m_enable_dhcps(FAR struct gs2200m_dev_s *dev,
                                            uint8_t on)
{
  char cmd[20];

  snprintf(cmd, sizeof(cmd), "AT+DHCPSRVR=%d\r\n", on);
  return gs2200m_send_cmd2(dev, cmd);
}

/****************************************************************************
 * Name: gs2200m_set_auth
 * NOTE: See 5.3.2 Authentication Mode
 ****************************************************************************/

static enum pkt_type_e gs2200m_set_auth(FAR struct gs2200m_dev_s *dev,
                                        int mode)
{
  char cmd[16];

  snprintf(cmd, sizeof(cmd), "AT+WAUTH=%d\r\n", mode);
  return gs2200m_send_cmd2(dev, cmd);
}

#ifdef CONFIG_WL_GS2200M_ENABLE_WEP

/****************************************************************************
 * Name: gs2200m_set_wepkey
 * NOTE: See 5.3.3.2
 ****************************************************************************/

static enum pkt_type_e gs2200m_set_wepkey(FAR struct gs2200m_dev_s *dev,
                                          FAR char *key)
{
  char cmd[32];

  snprintf(cmd, sizeof(cmd), "AT+WWEP1=%s\r\n", key);
  return gs2200m_send_cmd2(dev, cmd);
}

#else

/****************************************************************************
 * Name: gs2200m_set_wpa2pf
 * NOTE: See 5.3.3.4
 ****************************************************************************/

static enum pkt_type_e gs2200m_set_wpa2pf(FAR struct gs2200m_dev_s *dev,
                                          FAR char *key)
{
  char cmd[64];

  snprintf(cmd, sizeof(cmd), "AT+WWPA=%s\r\n", key);
  return gs2200m_send_cmd2(dev, cmd);
}

#endif /* CONFIG_WL_GS2200M_ENABLE_WEP */

/****************************************************************************
 * Name: gs2200m_get_wstatus
 * NOTE: See 11.3.5 WLAN Status
 ****************************************************************************/

enum pkt_type_e gs2200m_get_wstatus(FAR struct gs2200m_dev_s *dev)
{
  struct pkt_dat_s pkt_dat;
  enum pkt_type_e  r;
  int i;

  /* Initialize pkt_dat and send command */

  memset(&pkt_dat, 0, sizeof(pkt_dat));
  r = gs2200m_send_cmd(dev, (char *)"AT+WSTATUS\r\n", &pkt_dat);

  if (r != TYPE_OK)
    {
      goto errout;
    }

  for (i = 0; i < pkt_dat.n; i++)
    {
      wlinfo("%s\n", pkt_dat.msg[i]);
    }

errout:
  _release_pkt_dat(dev, &pkt_dat);
  return r;
}

/****************************************************************************
 * Name: gs2200m_create_tcpc
 * NOTE: See 7.5.1.1 Create TCP Clients and 7.5.1.2 Create UDP Client
 ****************************************************************************/

static enum pkt_type_e
gs2200m_create_clnt(FAR struct gs2200m_dev_s *dev,
                    FAR struct gs2200m_connect_msg *msg,
                    FAR char *cid)
{
  struct pkt_dat_s pkt_dat;
  enum pkt_type_e  r;
  char cmd[40];
  char *p;
  int   n;

  *cid = 'z'; /* Invalidate cid */

  if (SOCK_STREAM == msg->type)
    {
      snprintf(cmd, sizeof(cmd), "AT+NCTCP=%s,%s\r\n",
               msg->addr, msg->port);
    }
  else if (SOCK_DGRAM == msg->type)
    {
      if (0 == msg->lport)
        {
          snprintf(cmd, sizeof(cmd), "AT+NCUDP=%s,%s\r\n",
                   msg->addr, msg->port);
        }

      else
        {
          snprintf(cmd, sizeof(cmd), "AT+NCUDP=%s,%s,%d\r\n",
                   msg->addr, msg->port, msg->lport);
        }
    }
  else
    {
      ASSERT(false);
    }

  /* Initialize pkt_dat and send  */

  memset(&pkt_dat, 0, sizeof(pkt_dat));
  r = gs2200m_send_cmd(dev, cmd, &pkt_dat);

  if (r != TYPE_OK || pkt_dat.n == 0)
    {
      wlerr("+++ error: r=%d pkt_dat.msg[0]=%s\n",
            r, pkt_dat.msg[0]);
      goto errout;
    }

  if (NULL != (p = strstr(pkt_dat.msg[0], "CONNECT ")))
    {
      n = sscanf(p, "CONNECT %c", cid);
      ASSERT(1 == n);
      wlinfo("+++ OK: p=%s| (n=%d)\n", p, n);
    }

errout:
  _release_pkt_dat(dev, &pkt_dat);
  return r;
}

/****************************************************************************
 * Name: gs2200m_start_server
 * NOTE: See 7.5.1.3 Start TCP Server, 7.5.1.4 Start UDP Server
 ****************************************************************************/

static enum pkt_type_e gs2200m_start_server(FAR struct gs2200m_dev_s *dev,
                                            FAR char *port, bool is_tcp,
                                            FAR char *cid)
{
  struct pkt_dat_s pkt_dat;
  enum pkt_type_e  r;
  char cmd[40];
  char *p;
  int   n;

  /* Prepare cmd */

  if (is_tcp)
    {
      snprintf(cmd, sizeof(cmd), "AT+NSTCP=%s\r\n", port);
    }
  else
    {
      snprintf(cmd, sizeof(cmd), "AT+NSUDP=%s\r\n", port);
    }

  /* Initialize pkt_dat and send  */

  memset(&pkt_dat, 0, sizeof(pkt_dat));
  r = gs2200m_send_cmd(dev, cmd, &pkt_dat);

  if (r != TYPE_OK || pkt_dat.n == 0)
    {
      goto errout;
    }

  if (NULL != (p = strstr(pkt_dat.msg[0], "CONNECT ")))
    {
      n = sscanf(p, "CONNECT %c", cid);
      ASSERT(1 == n);
      wlinfo("+++ OK: p=%s| (n=%d)\n", p, n);
    }

errout:
  _release_pkt_dat(dev, &pkt_dat);
  return r;
}

/****************************************************************************
 * Name: gs2200m_send_bulk
 * NOTE: See 7.5.3.2 Bulk Data Handling
 ****************************************************************************/

static enum pkt_type_e gs2200m_send_bulk(FAR struct gs2200m_dev_s *dev,
                                         FAR struct gs2200m_send_msg *msg)
{
  enum spi_status_e s;
  enum pkt_type_e   r;
  int  bulk_hdr_size;
  char digits[6];
  char cmd[32];

  memset(cmd, 0, sizeof(cmd));

  if (MAX_PAYLOAD <= msg->len)
    {
      msg->len = MAX_PAYLOAD;
    }

  /* Convert the data length to 4 ascii char */

  _to_ascii_char(msg->len, digits);

  wlinfo("** cid=%c len=%d digits=%s\n", msg->cid, msg->len, digits);

  if (msg->is_tcp)
    {
      /* NOTE: See 7.5.3.2 Bulk Data Handling for TCP
       * <ESC>Z<CID><Data Length xxxx 4 ascii char><data>
       */

      snprintf(cmd, sizeof(cmd), "%cZ%c%s", ASCII_ESC, msg->cid, digits);
    }
  else
    {
      wlinfo("** addr=%s port=%d\n", inet_ntoa(msg->addr.sin_addr),
             NTOHS(msg->addr.sin_port));

      /* NOTE: See 7.5.3.2 Bulk Data Handling for UDP
       * <ESC>Y<CID><IP address>:<port>:<Data Length xxxx 4 ascii char><data>
       */

      snprintf(cmd, sizeof(cmd), "%cY%c%s:%d:%s",
               ASCII_ESC, msg->cid,
               inet_ntoa(msg->addr.sin_addr), NTOHS(msg->addr.sin_port),
               digits);
    }

  bulk_hdr_size = strlen(cmd);
  memcpy(dev->tx_buff, cmd, bulk_hdr_size);
  memcpy(dev->tx_buff + bulk_hdr_size, msg->buf, msg->len);

  /* Send the bulk data */

  s = gs2200m_hal_write(dev, (char *)dev->tx_buff, msg->len + bulk_hdr_size);

  if (s == SPI_TIMEOUT)
    {
      /* In case of SPI_TIMEOUT, return OK with 0 bytes sent */

      s = SPI_OK;
      msg->len = 0;
    }

  r = _spi_err_to_pkt_type(s);

  return r;
}

/****************************************************************************
 * Name: gs2200m_close_conn
 * NOTE: See 7.1.4 Closing a Connection
 ****************************************************************************/

static enum pkt_type_e gs2200m_close_conn(FAR struct gs2200m_dev_s *dev,
                                          char cid)
{
  char cmd[15];

  snprintf(cmd, sizeof(cmd), "AT+NCLOSE=%c\r\n", cid);
  return gs2200m_send_cmd2(dev, cmd);
}

/****************************************************************************
 * Name: gs2200m_enable_bulk
 * NOTE: See 7.1.1 Data Transfer in Bulk Mode
 ****************************************************************************/

static enum pkt_type_e gs2200m_enable_bulk(FAR struct gs2200m_dev_s *dev,
                                           uint8_t on)
{
  char cmd[20];

  snprintf(cmd, sizeof(cmd), "AT+BDATA=%d\r\n", on);
  return gs2200m_send_cmd2(dev, cmd);
}

/****************************************************************************
 * Name: gs2200m_enable_echo
 * NOTE: See 11.3.2 Echo
 ****************************************************************************/

static enum pkt_type_e gs2200m_enable_echo(FAR struct gs2200m_dev_s *dev,
                                           uint8_t on)
{
  char cmd[8];

  snprintf(cmd, sizeof(cmd), "ATE%d\r\n", on);
  return gs2200m_send_cmd2(dev, cmd);
}

/****************************************************************************
 * Name: gs2200m_activate_wrx
 * NOTE: See 9.1.1 Active Radio Receive
 ****************************************************************************/

static enum pkt_type_e gs2200m_activate_wrx(FAR struct gs2200m_dev_s *dev,
                                            uint8_t on)
{
  char cmd[30];

  snprintf(cmd, sizeof(cmd), "AT+WRXACTIVE=%d\r\n", on);
  return gs2200m_send_cmd2(dev, cmd);
}

/****************************************************************************
 * Name: gs2200m_powersave_wrx
 * NOTE: See 9.1.1 Active Radio Receive
 ****************************************************************************/

static enum pkt_type_e gs2200m_powersave_wrx(FAR struct gs2200m_dev_s *dev,
                                             uint8_t on)
{
  char cmd[30];

  snprintf(cmd, sizeof(cmd), "AT+WRXPS=%d\r\n", on);
  return gs2200m_send_cmd2(dev, cmd);
}

/****************************************************************************
 * Name: gs2200m_syncloss
 * NOTE: See 5.1.4 Sync loss interval
 ****************************************************************************/

static enum pkt_type_e gs2200m_syncloss(FAR struct gs2200m_dev_s *dev,
                                            int val)
{
  char cmd[30];

  snprintf(cmd, sizeof(cmd), "AT+WSYNCINTRL=%d\r\n", val);
  return gs2200m_send_cmd2(dev, cmd);
}

/****************************************************************************
 * Name: gs2200m_set_gpio
 * NOTE: See 10.3 GPIO Commands
 ****************************************************************************/

#ifdef USE_LED
static enum pkt_type_e gs2200m_set_gpio(FAR struct gs2200m_dev_s *dev,
                                        int n, int val)
{
  char cmd[24];

  snprintf(cmd, sizeof(cmd), "AT+DGPIO=%d,%d\r\n", n, val);
  return gs2200m_send_cmd2(dev, cmd);
}
#endif

/****************************************************************************
 * Name: gs2200m_set_loglevel
 * NOTE: See 11.3.1 Log Level
 ****************************************************************************/

#if CONFIG_WL_GS2200M_LOGLEVEL > 0
static enum pkt_type_e gs2200m_set_loglevel(FAR struct gs2200m_dev_s *dev,
                                            int level)
{
  char cmd[16];

  snprintf(cmd, sizeof(cmd), "AT+LOGLVL=%d\r\n", level);
  return gs2200m_send_cmd2(dev, cmd);
}
#endif

/****************************************************************************
 * Name: gs2200m_closeallsock
 * NOTE: See 7.1.5 Closing All Connections
 ****************************************************************************/

static void gs2200m_closeallsock(FAR struct gs2200m_dev_s *dev)
{
  gs2200m_send_cmd2(dev, "AT+NCLOSEALL\r\n");
}

/****************************************************************************
 * Name: gs2200m_get_version
 ****************************************************************************/

#ifdef CONFIG_WL_GS2200M_CHECK_VERSION
static enum pkt_type_e gs2200m_get_version(FAR struct gs2200m_dev_s *dev)
{
  char cmd[16];

  snprintf(cmd, sizeof(cmd), "AT+VER=??\r\n");
  return gs2200m_send_cmd2(dev, cmd);
}
#endif

/****************************************************************************
 * Name: gs2200m_get_cstatus
 ****************************************************************************/

static enum pkt_type_e gs2200m_get_cstatus(FAR struct gs2200m_dev_s *dev,
                                           FAR struct gs2200m_name_msg *msg)
{
  struct pkt_dat_s pkt_dat;
  enum pkt_type_e  r;
  char cmd[16];
  int i;

  snprintf(cmd, sizeof(cmd), "AT+CID=?\r\n");

  /* Initialize pkt_dat and send */

  memset(&pkt_dat, 0, sizeof(pkt_dat));
  r = gs2200m_send_cmd(dev, cmd, &pkt_dat);

  if (r != TYPE_OK || pkt_dat.n <= 2)
    {
      wlerr("+++ error: r=%d pkt_dat.msg[0]=%s\n",
            r, pkt_dat.msg[0]);

      goto errout;
    }

  /* Find cid in the connection status */

  for (i = 1; i < pkt_dat.n - 2; i++)
    {
      int  n;
      char c;
      int  a[4];
      int  p[2];
      char type[8];
      char mode[8];
      memset(type, 0, sizeof(type));
      memset(mode, 0, sizeof(mode));
      n = sscanf(pkt_dat.msg[i], "%c %7s %6s %d %d %d.%d.%d.%d",
                 &c, type, mode, &p[0], &p[1],
                 &a[0], &a[1], &a[2], &a[3]);
      ASSERT(9 == n);

      wlinfo("[%d]: %c %s %s %d %d %d.%d.%d.%d\n",
             i, c, type, mode, p[0], p[1],
             a[0], a[1], a[2], a[3]);

      if (c == msg->cid)
        {
          /* Set family,  port and address (remote only) */

          msg->addr.sin_family = AF_INET;

          if (msg->local)
            {
              msg->addr.sin_port = HTONS(p[0]);
            }
          else
            {
              char addr[20];
              msg->addr.sin_port = HTONS(p[1]);
              snprintf(addr, sizeof(addr),
                       "%d.%d.%d.%d", a[0], a[1], a[2], a[3]);
              inet_aton(addr, &msg->addr.sin_addr);
            }

          goto errout;
        }
    }

  /* Not found */

  r = TYPE_UNMATCH;

errout:
  _release_pkt_dat(dev, &pkt_dat);
  return r;
}

/****************************************************************************
 * Name: gs2200m_ioctl_bind
 ****************************************************************************/

static int gs2200m_ioctl_bind(FAR struct gs2200m_dev_s *dev,
                              FAR struct gs2200m_bind_msg *msg)
{
  enum pkt_type_e type = TYPE_OK;
  bool auto_assign = false;
  char port_str[6];
  uint16_t port;
  char cid = 'z';
  int ret = OK;

  wlinfo("+++ start: (cid=%c, port=%s)\n", msg->cid, msg->port);

  port = (uint16_t)strtol(msg->port, NULL, 10);

  if (0 == port)
    {
      auto_assign = true;
      port = PORT_START;
    }

retry:

  snprintf(port_str, sizeof(port_str), "%d", port);

  /* Start TCP/UDP server and retrieve cid */

  type = gs2200m_start_server(dev, port_str, msg->is_tcp, &cid);

  if (type != TYPE_OK)
    {
      if (auto_assign && (port < PORT_END))
        {
          port++;
          goto retry;
        }

      ret = -EINVAL;
      goto errout;
    }

  /* Enable the cid for server socket and if the pkt_q is empty */

  _enable_cid(&dev->valid_cid_bits, cid, true);
  _check_pkt_q_empty(dev, cid);

errout:

  msg->type = type;
  msg->cid  = cid;

  wlinfo("+++ end: type=%d (cid=%c)\n", type, cid);

  return ret;
}

/****************************************************************************
 * Name: gs2200m_ioctl_connect
 ****************************************************************************/

static int gs2200m_ioctl_connect(FAR struct gs2200m_dev_s *dev,
                                 FAR struct gs2200m_connect_msg *msg)
{
  enum pkt_type_e type;
  char cid = 'z';
  int ret = OK;

  wlinfo("++ start: addr=%s port=%s\n", msg->addr, msg->port);

  /* Create TCP or UDP connection */

  type = gs2200m_create_clnt(dev, msg, &cid);

  msg->type = type;

  switch (type)
    {
      case TYPE_OK:
        msg->cid = cid;

        /* Enable the cid and checi if the pkt_q is empty */

        _enable_cid(&dev->valid_cid_bits, cid, true);
        _check_pkt_q_empty(dev, cid);
        break;

      case TYPE_ERROR:

        /* We assume the connection has been refused */

        ret = -ECONNREFUSED;
        break;

      case TYPE_TIMEOUT:
        ret = -ETIMEDOUT;
        break;

      default:
        wlerr("+++ error: type=%d\n", type);
        ASSERT(false);
        ret = -EINVAL;
    }

  wlinfo("++ end: cid=%c (type=%d,ret=%d)\n", cid, type, ret);

  return ret;
}

/****************************************************************************
 * Name: gs2200m_ioctl_send
 ****************************************************************************/

static int gs2200m_ioctl_send(FAR struct gs2200m_dev_s *dev,
                              FAR struct gs2200m_send_msg *msg)
{
  FAR struct gs2200m_bind_msg bmsg;
  enum pkt_type_e type;
  int ret = OK;

  wlinfo("+++ start: (cid=%c)\n", msg->cid);

#ifdef USE_LED
  gs2200m_set_gpio(dev, LED_GPIO, 1);
#endif

  /* If the msg is udp having unassgined cid */

  if (!msg->is_tcp && 'z' == msg->cid)
    {
      /* NOTE: need to assign port automatically */

      memset(&bmsg, 0, sizeof(bmsg));
      ret = gs2200m_ioctl_bind(dev, &bmsg);
      ASSERT(0 == ret);

      wlinfo("+++ cid is assigned for udp (cid=%c)\n", bmsg.cid);
      msg->cid = bmsg.cid;
    }

  if (!_cid_is_set(&dev->valid_cid_bits, msg->cid))
    {
      wlinfo("+++ already closed\n");
      type = TYPE_DISCONNECT;
      ret = -ENOTCONN;
      goto errout;
    }

  type = gs2200m_send_bulk(dev, msg);

  msg->type = type;

errout:

  if (type != TYPE_OK && type != TYPE_DISCONNECT)
    {
      ret = -EINVAL;
    }

#ifdef USE_LED
  gs2200m_set_gpio(dev, LED_GPIO, 0);
#endif

  wlinfo("+++ end: cid=%c len=%d type=%d\n",
         msg->cid, msg->len, type);

  return ret;
}

/****************************************************************************
 * Name: gs2200m_ioctl_recv
 ****************************************************************************/

static int gs2200m_ioctl_recv(FAR struct gs2200m_dev_s *dev,
                              FAR struct gs2200m_recv_msg *msg)
{
  bool     cont = true;
  int      ret  = OK;
  uint8_t  c    = _cid_to_uint8(msg->cid);

  wlinfo("+++ start: cid=%c\n", msg->cid);

#ifdef USE_LED
  gs2200m_set_gpio(dev, LED_GPIO, 1);
#endif

  if (0 == dev->pkt_q_cnt[c])
    {
      /* REVISIT */

      wlwarn("**** no packet for cid=%c\n", msg->cid);
      ret = -EAGAIN;
      goto errout;
    }

  while (1)
    {
      /* Finished copying or no packet */

      if (msg->reqlen == msg->len || 0 == dev->pkt_q_cnt[c] || !cont)
        {
          break;
        }

      /* Copy data from the front-most packet */

      cont = _copy_data_from_pkt(dev, msg);
    }

  wlinfo("+++ pkt_q_cnt[%c]=%d\n", msg->cid, dev->pkt_q_cnt[c]);

  if (dev->pkt_q_cnt[c])
    {
      _notif_q_push(dev, msg->cid);
    }

  /* Do packet flow control */

  _control_pkt_q(dev);

errout:

#ifdef USE_LED
  gs2200m_set_gpio(dev, LED_GPIO, 0);
#endif

  wlinfo("+++ end: cid=%c len=%d type=%d ret=%d\n",
         msg->cid, msg->len, msg->type, ret);

  return ret;
}

/****************************************************************************
 * Name: gs2200m_ioctl_close
 ****************************************************************************/

static int gs2200m_ioctl_close(FAR struct gs2200m_dev_s *dev,
                               FAR struct gs2200m_close_msg *msg)
{
  enum pkt_type_e type = TYPE_OK;
  int ret = OK;

  wlinfo("++ start: (cid=%c)\n", msg->cid);

  if (!_cid_is_set(&dev->valid_cid_bits, msg->cid))
    {
      wlinfo("+++ already closed\n");
      goto errout;
    }

  /* Disable the cid */

  _enable_cid(&dev->valid_cid_bits, msg->cid, false);

  type = gs2200m_close_conn(dev, msg->cid);

  if (type != TYPE_OK)
    {
      ret = -EINVAL;
    }

errout:

  /* Remove all pkt associated with this cid */

  _remove_all_pkt(dev, _cid_to_uint8(msg->cid));

  /* Do packet flow control */

  _control_pkt_q(dev);

  wlinfo("++ end: cid=%c type=%d\n", msg->cid, type);

  return ret;
}

/****************************************************************************
 * Name: gs2200m_ioctl_accept
 ****************************************************************************/

static int gs2200m_ioctl_accept(FAR struct gs2200m_dev_s *dev,
                                FAR struct gs2200m_accept_msg *msg)
{
  FAR struct pkt_dat_s *pkt_dat;
  struct gs2200m_name_msg nmsg;
  enum pkt_type_e r;
  uint8_t c;
  char s_cid;
  char c_cid;
  int ret = OK;
  int n;

  wlinfo("+++ start: cid=%c\n", msg->cid);

  c = _cid_to_uint8(msg->cid);
  pkt_dat = (FAR struct pkt_dat_s *)dq_peek(&dev->pkt_q[c]);

  if (NULL == pkt_dat)
    {
      wlerr("*** error: cid=%c not found\n", msg->cid);
      ret = -EINVAL;
      goto errout;
    }

  n = sscanf(pkt_dat->msg[0], "CONNECT %c %c", &s_cid, &c_cid);
  ASSERT(2 == n);

  wlinfo("+++ s_cid=%c c_cid=%c\n", s_cid, c_cid);

  /* Remove the accept packet (actually CONNECT) from the queue */

  _remove_and_free_pkt(dev, _cid_to_uint8(msg->cid));

  /* Copy a client cid which was obtained in CONNECT event */

  msg->type = TYPE_OK;
  msg->cid  = c_cid; /* NOTE: override new client cid */

  /* Disable accept in progress */

  _enable_cid(&dev->aip_cid_bits, c_cid, false);

  /* If a packet still exists, notify it */

  if (dev->pkt_q_cnt[_cid_to_uint8(c_cid)])
    {
      _notif_q_push(dev, c_cid);
    }

  /* Obtain remote address info */

  nmsg.local = 0;
  nmsg.cid = msg->cid;
  r = gs2200m_get_cstatus(dev, &nmsg);

  if (TYPE_OK != r)
    {
      wlerr("*** error: cid=%c not found\n", msg->cid);
      ret = -EINVAL;
      goto errout;
    }

  msg->addr = nmsg.addr;

errout:
  wlinfo("+++ end: type=%d (msg->cid=%c)\n", msg->type, msg->cid);

  return ret;
}

/****************************************************************************
 * Name: gs2200m_ioctl_assoc_sta
 ****************************************************************************/

static int gs2200m_ioctl_assoc_sta(FAR struct gs2200m_dev_s *dev,
                                   FAR struct gs2200m_assoc_msg *msg)
{
  enum pkt_type_e t;

  /* Remember assoc request msg for reconnection */

  memcpy(&dev->reconnect_msg, msg, sizeof(struct gs2200m_assoc_msg));

  /* Disassociate */

  t = gs2200m_disassociate(dev);
  ASSERT(TYPE_OK == t);

  /* Set to STA mode */

  t = gs2200m_set_opmode(dev, 0);
  ASSERT(TYPE_OK == t);

#ifdef CONFIG_WL_GS2200M_DISABLE_DHCPC
  /* Disable DHCP Client */

  t = gs2200m_enable_dhcpc(dev, 0);
  ASSERT(TYPE_OK == t);

  /* Set static address */

  t = gs2200m_set_addresses(dev,
                            "10.0.0.2",
                            "255.255.255.0",
                            "10.0.0.1"
                            );
  ASSERT(TYPE_OK == t);
#else
  /* Enable DHCP Client */

  t = gs2200m_enable_dhcpc(dev, 1);
  ASSERT(TYPE_OK == t);
#endif

  /* Get mac address info */

  t = gs2200m_get_mac(dev);
  ASSERT(TYPE_OK == t);

  /* Set WPA2 Passphrase */

  if (TYPE_OK != gs2200m_calc_key(dev, msg->ssid, msg->key))
    {
      wlerr("*** error: invalid wpa2 key (key:%s)\n", msg->key);
      return -1;
    }

  /* Associate with AP */

  if (TYPE_OK != gs2200m_join_network(dev, msg->ssid, 0))
    {
      wlerr("*** error: failed to join (ssid:%s)\n", msg->ssid);
      return -1;
    }

  dev->disassociate_flag = false;

  /* Sync lost time interval */

  t = gs2200m_syncloss(dev, CONFIG_WL_GS2200M_SYNC_INTERVAL);
  ASSERT(TYPE_OK == t);

  return OK;
}

/****************************************************************************
 * Name: gs2200m_ioctl_assoc_ap
 ****************************************************************************/

static int gs2200m_ioctl_assoc_ap(FAR struct gs2200m_dev_s *dev,
                                  FAR struct gs2200m_assoc_msg *msg)
{
  enum pkt_type_e t;

  /* Set to AP mode */

  t = gs2200m_set_opmode(dev, 2);
  ASSERT(TYPE_OK == t);

  /* Get mac address info */

  t = gs2200m_get_mac(dev);
  ASSERT(TYPE_OK == t);

  /* Disassociate */

  t = gs2200m_disassociate(dev);
  ASSERT(TYPE_OK == t);

  /* Set address info */

  t = gs2200m_set_addresses(dev,
                           "192.168.11.1",
                           "255.255.255.0",
                           "192.168.11.1"
                           );
  ASSERT(TYPE_OK == t);

  /* Set auth mode */

  t = gs2200m_set_auth(dev, 2);
  ASSERT(TYPE_OK == t);

#ifdef CONFIG_WL_GS2200M_ENABLE_WEP
  /* Set security mode (WEP) */

  t = gs2200m_set_security(dev, SEC_MODE_WEP);
  ASSERT(TYPE_OK == t);

  /* Set WEP key */

  if (TYPE_OK != gs2200m_set_wepkey(dev, msg->key))
    {
      wlerr("*** error: invalid wepkey: %s\n", msg->key);
      return -1;
    }
#else
  /* Set security mode (WPA2-PSK) */

  t = gs2200m_set_security(dev, SEC_MODE_WPA2PSK);
  ASSERT(TYPE_OK == t);

  /* Set WPA-PSK and WPA2-PSK Passphrase */

  if (TYPE_OK != gs2200m_set_wpa2pf(dev, msg->key))
    {
      wlerr("*** error: invalid passphrase: %s\n", msg->key);
      return -1;
    }
#endif

  /* Start DHCP server */

  t = gs2200m_enable_dhcps(dev, 1);
  ASSERT(TYPE_OK == t);

  /* Enable the AP */

  if (TYPE_OK != gs2200m_join_network(dev, msg->ssid, msg->ch))
    {
      wlerr("*** error: failed to join (ssid:%s, ch:%d)\n",
            msg->ssid, msg->ch);
      return -1;
    }

  return OK;
}

/****************************************************************************
 * Name: gs2200m_gs2200m_ioctl_iwreq
 ****************************************************************************/

static int gs2200m_ioctl_iwreq(FAR struct gs2200m_dev_s *dev,
                               FAR struct gs2200m_ifreq_msg *msg)
{
  struct iwreq *res = (struct iwreq *)&msg->ifr;
  struct pkt_dat_s pkt_dat;
  enum pkt_type_e  r;
  char cmd[64];
  char cmd2[64];
  int  n = 0;

  snprintf(cmd, sizeof(cmd), "AT+NSTAT=?\r\n");

  /* Initialize pkt_dat and send */

  memset(&pkt_dat, 0, sizeof(pkt_dat));
  r = gs2200m_send_cmd(dev, cmd, &pkt_dat);

  if (r != TYPE_OK || pkt_dat.n <= 7)
    {
      wlerr("+++ error: r=%d pkt_dat.msg[0]=%s\n",
            r, pkt_dat.msg[0]);

      goto errout;
    }

  /* Find cid in the connection status */

  if (msg->cmd == SIOCGIWNWID)
    {
      if (strstr(pkt_dat.msg[2], "BSSID=") == NULL)
        {
          wlerr("+++ error: pkt_dat.msg[2]=%s\n", pkt_dat.msg[2]);
          goto errout;
        }

      n = sscanf(pkt_dat.msg[2], "BSSID=%c:%c:%c:%c:%c:%c %s",
                 &res->u.ap_addr.sa_data[0], &res->u.ap_addr.sa_data[1],
                 &res->u.ap_addr.sa_data[2], &res->u.ap_addr.sa_data[3],
                 &res->u.ap_addr.sa_data[4], &res->u.ap_addr.sa_data[5],
                 cmd);
      ASSERT(7 == n);
      wlinfo("BSSID:%02X:%02X:%02X:%02X:%02X:%02X\n",
             res->u.ap_addr.sa_data[0], res->u.ap_addr.sa_data[1],
             res->u.ap_addr.sa_data[2], res->u.ap_addr.sa_data[3],
             res->u.ap_addr.sa_data[4], res->u.ap_addr.sa_data[5]);
    }
  else if (msg->cmd == SIOCGIWFREQ)
    {
      if (strstr(pkt_dat.msg[2], "CHANNEL=") == NULL)
        {
          wlerr("+++ error: pkt_dat.msg[2]=%s\n", pkt_dat.msg[2]);
          goto errout;
        }

      n = sscanf(pkt_dat.msg[2], "%s CHANNEL=%" SCNd32 " %s",
                 cmd, &res->u.freq.m, cmd2);
      ASSERT(3 == n);
      wlinfo("CHANNEL:%" PRId32 "\n", res->u.freq.m);
    }
  else if (msg->cmd == SIOCGIWSENS)
    {
      if (strstr(pkt_dat.msg[3], "RSSI=") == NULL)
        {
          wlerr("+++ error: pkt_dat.msg[3]=%s\n", pkt_dat.msg[3]);
          goto errout;
        }

      n = sscanf(pkt_dat.msg[3], "RSSI=%" SCNd8, &res->u.qual.level);
      ASSERT(1 == n);
      wlinfo("RSSI:%d\n", res->u.qual.level);
    }

errout:
  _release_pkt_dat(dev, &pkt_dat);

  if (n == 0)
    {
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: gs2200m_ifreq_ifreq
 ****************************************************************************/

static int gs2200m_ioctl_ifreq(FAR struct gs2200m_dev_s *dev,
                               FAR struct gs2200m_ifreq_msg *msg)
{
  FAR struct sockaddr_in *inaddr;
  struct in_addr in[3];
  char addr[3][17];
  bool getreq = false;
  int ret = OK;

  wlinfo("+++ start: cmd=%" PRIx32 "\n", msg->cmd);

  inaddr = (FAR struct sockaddr_in *)&msg->ifr.ifr_addr;

  switch (msg->cmd)
    {
      case SIOCGIFHWADDR:
        getreq = true;
        memcpy(&msg->ifr.ifr_hwaddr.sa_data,
               dev->net_dev.d_mac.ether.ether_addr_octet, 6);
        break;

      case SIOCGIFADDR:
        getreq = true;
        memcpy(&inaddr->sin_addr,
               &dev->net_dev.d_ipaddr,
               sizeof(dev->net_dev.d_ipaddr)
               );
        break;

    case SIOCSIFADDR:
        memcpy(&dev->net_dev.d_ipaddr,
               &inaddr->sin_addr, sizeof(inaddr->sin_addr)
               );
        break;

      case SIOCSIFDSTADDR:
        memcpy(&dev->net_dev.d_draddr,
               &inaddr->sin_addr, sizeof(inaddr->sin_addr)
               );
        break;

      case SIOCSIFNETMASK:
        memcpy(&dev->net_dev.d_netmask,
               &inaddr->sin_addr, sizeof(inaddr->sin_addr)
               );
        break;

      case SIOCGIWNWID:
      case SIOCGIWFREQ:
      case SIOCGIWSENS:
        ret = gs2200m_ioctl_iwreq(dev, msg);
        break;

      default:
        ret = -EINVAL;
        break;
    }

  if (false == getreq && OK == ret)
    {
      memcpy(&in[0], &dev->net_dev.d_ipaddr, sizeof(in[0]));
      memcpy(&in[1], &dev->net_dev.d_netmask, sizeof(in[1]));
      memcpy(&in[2], &dev->net_dev.d_draddr, sizeof(in[2]));
      strncpy(addr[0], inet_ntoa(in[0]), sizeof(addr[0]));
      strncpy(addr[1], inet_ntoa(in[1]), sizeof(addr[1]));
      strncpy(addr[2], inet_ntoa(in[2]), sizeof(addr[2]));

      gs2200m_set_addresses(dev, addr[0], addr[1], addr[2]);
    }

  wlinfo("+++ end:\n");
  return ret;
}

/****************************************************************************
 * Name: gs2200m_ioctl_name
 ****************************************************************************/

static int gs2200m_ioctl_name(FAR struct gs2200m_dev_s *dev,
                              FAR struct gs2200m_name_msg *msg)
{
  enum pkt_type_e r;
  int ret = 0;

  /* Obtain connection status */

  r = gs2200m_get_cstatus(dev, msg);

  if (r != TYPE_OK)
    {
      ret = -EINVAL;
      goto errout;
    }

  if (msg->local)
    {
      /* Copy local address from net_dev */

      memcpy(&msg->addr.sin_addr,
             &dev->net_dev.d_ipaddr,
             sizeof(msg->addr.sin_addr)
             );
    }

errout:

  return ret;
}

/****************************************************************************
 * Name: gs2200m_ioctl
 ****************************************************************************/

static int gs2200m_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct gs2200m_dev_s *dev;
  int ret = -EINVAL;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct gs2200m_dev_s *)inode->i_private;

  /* Lock the device */

  ret = gs2200m_lock(dev);
  if (ret < 0)
    {
      /* Return only if the task was canceled */

      return ret;
    }

  /* Disable gs2200m irq to poll dready */

  DEBUGASSERT(dev);
  dev->lower->disable();

  switch (cmd)
    {
      case GS2200M_IOC_CONNECT:
        {
          struct gs2200m_connect_msg *msg =
            (struct gs2200m_connect_msg *)arg;

          ret = gs2200m_ioctl_connect(dev, msg);
        }
        break;

      case GS2200M_IOC_SEND:
        {
          struct gs2200m_send_msg *msg =
            (struct gs2200m_send_msg *)arg;

          ret = gs2200m_ioctl_send(dev, msg);
        }
        break;

      case GS2200M_IOC_RECV:
        {
          struct gs2200m_recv_msg *msg =
            (struct gs2200m_recv_msg *)arg;

          ret = gs2200m_ioctl_recv(dev, msg);
          break;
        }

      case GS2200M_IOC_CLOSE:
        {
          struct gs2200m_close_msg *msg =
            (struct gs2200m_close_msg *)arg;

          ret = gs2200m_ioctl_close(dev, msg);
          break;
        }

      case GS2200M_IOC_BIND:
        {
          struct gs2200m_bind_msg *msg =
            (struct gs2200m_bind_msg *)arg;

          ret = gs2200m_ioctl_bind(dev, msg);
          break;
        }

      case GS2200M_IOC_ACCEPT:
        {
          struct gs2200m_accept_msg *msg =
            (struct gs2200m_accept_msg *)arg;

          ret = gs2200m_ioctl_accept(dev, msg);
          break;
        }

      case GS2200M_IOC_ASSOC:
        {
          struct gs2200m_assoc_msg *msg =
            (struct gs2200m_assoc_msg *)arg;

          if (0 == msg->mode)
            {
              ret = gs2200m_ioctl_assoc_sta(dev, msg);
            }
          else
            {
              ret = gs2200m_ioctl_assoc_ap(dev, msg);
            }
          break;
        }

      case GS2200M_IOC_IFREQ:
        {
          struct gs2200m_ifreq_msg *msg =
            (struct gs2200m_ifreq_msg *)arg;

          ret = gs2200m_ioctl_ifreq(dev, msg);
          break;
        }

      case GS2200M_IOC_NAME:
        {
          struct gs2200m_name_msg *msg =
            (struct gs2200m_name_msg *)arg;

          ret = gs2200m_ioctl_name(dev, msg);
          break;
        }

      default:
        DEBUGPANIC();
        break;
    }

  /* Enable gs2200m irq again */

  dev->lower->enable();

  /* Unlock the device */

  gs2200m_unlock(dev);
  return ret;
}

/****************************************************************************
 * Name: gs2200m_poll
 ****************************************************************************/

static int gs2200m_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup)
{
  FAR struct inode *inode;
  FAR struct gs2200m_dev_s *dev;
  int ret = OK;

  wlinfo("== setup:%d\n", (int)setup);
  DEBUGASSERT(filep && fds);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  dev = (FAR struct gs2200m_dev_s *)inode->i_private;

  ret = gs2200m_lock(dev);
  if (ret < 0)
    {
      /* Return if the task was canceled */

      return ret;
    }

  /* Are we setting up the poll?  Or tearing it down? */

  if (setup)
    {
      /* Ignore waits that do not include POLLIN */

      if ((fds->events & POLLIN) == 0)
        {
          ret = -EDEADLK;
          goto errout;
        }

      /* NOTE: only one thread can poll the device at any time */

      if (dev->pfd)
        {
          ret = -EBUSY;
          goto errout;
        }

      dev->pfd = fds;

      uint8_t n = _notif_q_count(dev);

      if (0 < n)
        {
          dev->pfd->revents |= POLLIN;
          nxsem_post(dev->pfd->sem);
          wlinfo("==== _notif_q_count=%d\n", n);
        }
    }
  else
    {
      dev->pfd = NULL;
    }

errout:
  gs2200m_unlock(dev);
  return ret;
}

/****************************************************************************
 * Name: gs2200m_irq_worker
 ****************************************************************************/

static void gs2200m_irq_worker(FAR void *arg)
{
  FAR struct gs2200m_dev_s *dev;
  enum pkt_type_e t = TYPE_ERROR;
  struct pkt_dat_s *pkt_dat;
  bool ignored = false;
  bool over;
  uint8_t c;
  char s_cid;
  char c_cid;
  int n;
  int ec;
  int ret;

  DEBUGASSERT(arg != NULL);
  dev = (FAR struct gs2200m_dev_s *)arg;

  do
    {
      ret = gs2200m_lock(dev);

      /* The only failure would be if the worker thread were canceled.  That
       * is very unlikely, however.
       */

      DEBUGASSERT(ret == OK || ret == -ECANCELED);
    }
  while (ret < 0);

repeat:

  n = dev->lower->dready(&ec);
  wlinfo("== start (dready=%d, ec=%d)\n", n, ec);

  /* Allocate a new pkt_dat and initialize it */

  pkt_dat = (FAR struct pkt_dat_s *)kmm_malloc(sizeof(struct pkt_dat_s));
  ASSERT(NULL != pkt_dat);

  memset(pkt_dat, 0, sizeof(struct pkt_dat_s));
  pkt_dat->cid = 'z';

  /* Receive a packet */

  t = gs2200m_recv_pkt(dev, pkt_dat);

  if (true == dev->disassociate_flag)
    {
      /* Disassociate recovery */

      wlwarn("=== receive DISASSOCIATE\n");
      dev->valid_cid_bits = 0;

      do
        {
          /* Discard incoming packets until timeout happens */

          while (gs2200m_recv_pkt(dev, NULL) != TYPE_TIMEOUT)
            {
              nxsig_usleep(100 * 1000);
            }
        }
      while (gs2200m_ioctl_assoc_sta(dev, &dev->reconnect_msg) != OK);

      wlwarn("=== recover DISASSOCIATE\n");
      dev->disassociate_flag = false;

      gs2200m_closeallsock(dev);
      _notif_q_push(dev, DISASSOCIATION_CID);

      goto errout;
    }

  if (TYPE_ERROR == t || 'z' == pkt_dat->cid)
    {
      /* An error event? */

      wlerr("=== ignore (type=%d msg[0]=%s|)\n",
            pkt_dat->type, pkt_dat->msg[0]);

      ignored = true;
      goto errout;
    }

  /* Check if the cid has been invalid */

  if (!_cid_is_set(&dev->valid_cid_bits, pkt_dat->cid))
    {
      wlinfo("=== already closed (type=%d msg[0]=%s|)\n",
             pkt_dat->type, pkt_dat->msg[0]);

      ignored = true;
      goto errout;
    }

  c = _cid_to_uint8(pkt_dat->cid);

  /* Add the pkt_dat to the pkt_q */

  dq_addlast((FAR dq_entry_t *)pkt_dat, &dev->pkt_q[c]);
  dev->pkt_q_cnt[c]++;

  wlinfo("=== added to qkt_q[%d] t=%d\n", c, t);

  /* When a DISCONNECT packet received, disable the cid */

  if (TYPE_DISCONNECT == t)
    {
      wlinfo("=== received DISCONNECT for cid=%c\n", pkt_dat->cid);
      _enable_cid(&dev->valid_cid_bits, pkt_dat->cid, false);
    }

  /* If accept() is not in progress for the cid, add the packet to notif_q
   *
   * NOTE: we need this condition to process the packet in correct order,
   * when accept() sequcence is in progress.
   */

  if (!_cid_is_set(&dev->aip_cid_bits, pkt_dat->cid))
    {
      _notif_q_push(dev, pkt_dat->cid);
    }

  /* Check if the packet is CONNECT event from client */

  if (TYPE_CONNECT == t)
    {
      n = sscanf(pkt_dat->msg[0], "CONNECT %c %c", &s_cid, &c_cid);
      ASSERT(2 == n);

      wlinfo("==== CONNECT requested (%c:%c) pfd=%p\n",
             s_cid, c_cid, dev->pfd);

      /* Check pkt_q for the new client is empty */

      _check_pkt_q_empty(dev, c_cid);

      /* Enable the cid */

      _enable_cid(&dev->valid_cid_bits, c_cid, true);

      /* Enable accept in progress */

      _enable_cid(&dev->aip_cid_bits, c_cid, true);
    }

  /* Do packet flow control */

  over = _control_pkt_q(dev);

errout:

  if (ignored)
    {
      _release_pkt_dat(dev, pkt_dat);
      kmm_free(pkt_dat);
      ignored = false;
    }

  n = dev->lower->dready(&ec);

  wlinfo("== end: cid=%c (dready=%d, ec=%d) type=%d\n",
         pkt_dat->cid, n, ec, t);

  if (1 == n && !over)
    {
      goto repeat;
    }

  /* NOTE: Enable gs2200m irq which was disabled in gs2200m_irq() */

  dev->lower->enable();

  gs2200m_unlock(dev);
}

/****************************************************************************
 * Name: gs2200m_interrupt
 ****************************************************************************/

static int gs2200m_irq(int irq, FAR void *context, FAR void *arg)
{
  FAR struct gs2200m_dev_s *dev;

  DEBUGASSERT(arg != NULL);
  dev = (FAR struct gs2200m_dev_s *)arg;

  wlinfo(">>>>\n");

  if (!work_available(&dev->irq_work))
    {
      wlwarn("*** warning: there is still pending work ****\n");
      return 0;
    }

  /* NOTE: Disable gs2200m irq during processing */

  dev->lower->disable();

  return work_queue(GS2200MWORK, &dev->irq_work, gs2200m_irq_worker,
                    (FAR void *)dev, 0);
}

/****************************************************************************
 * Name: gs2200m_start
 ****************************************************************************/

static int gs2200m_start(FAR struct gs2200m_dev_s *dev)
{
  enum pkt_type_e t;

  /* NOTE: irq is still disabled here */

  /* Check boot msg */

  wlinfo("*** wait for boot msg\n");

  while (dev->lower->dready(NULL))
    {
      gs2200m_recv_pkt(dev, NULL);
      break;
    }

  /* TODO: Need to check Regulatory Domain stored in the internal flash.
   * If we need to change the damin, set here.
   */

  /* Disable echo */

  t = gs2200m_enable_echo(dev, 0);
  ASSERT(TYPE_OK == t);

#if CONFIG_WL_GS2200M_LOGLEVEL > 0
  /* Set log level */

  t = gs2200m_set_loglevel(dev, CONFIG_WL_GS2200M_LOGLEVEL);
  ASSERT(TYPE_OK == t);
#endif

#ifdef CONFIG_WL_GS2200M_CHECK_VERSION
  /* Version */

  t = gs2200m_get_version(dev);
  ASSERT(TYPE_OK == t);
#endif

  /* Activate RX */

  t = gs2200m_activate_wrx(dev, 1);
  ASSERT(TYPE_OK == t);

  /* Power save disable */

  t = gs2200m_powersave_wrx(dev, 0);
  ASSERT(TYPE_OK == t);

  /* Set Bulk Data mode */

  t = gs2200m_enable_bulk(dev, 1);
  ASSERT(TYPE_OK == t);

  /* Interface is up */

  dev->net_dev.d_flags |= IFF_UP;

  /* NOTE: Enable interrupt here */

  dev->lower->enable();

  return 0;
}

/****************************************************************************
 * Name: gs2200m_initialize
 ****************************************************************************/

static int gs2200m_initialize(FAR struct gs2200m_dev_s *dev,
                              FAR const struct gs2200m_lower_s *lower)
{
  int ret;
  int i;

  /* For each cid (0-f) */

  for (i = 0; i < 16; i++)
    {
      /* Initialize packet queue */

      dq_init(&dev->pkt_q[i]);
    }

  /* Initialize SPI driver. */

  ret = gs2200m_spi_init(dev);

  /* Reset and Unreset GS2200M */

  lower->reset(true);
  up_mdelay(1);
  lower->reset(false);
  up_mdelay(180);

  /* Attach interrupt handler */

  lower->attach(gs2200m_irq, dev);
  dev->int_enabled = true;

  /* Start gs2200m by sending commands */

  gs2200m_start(dev);

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gs2200m_register
 ****************************************************************************/

FAR void *gs2200m_register(FAR const char *devpath,
                           FAR struct spi_dev_s *spi,
                           FAR const struct gs2200m_lower_s *lower)
{
  FAR struct gs2200m_dev_s *dev;
  int ret;
  int size;

  size = sizeof(struct gs2200m_dev_s);
  dev = (FAR struct gs2200m_dev_s *)kmm_malloc(size);

  if (!dev)
    {
      wlerr("Failed to allocate instance.\n");
      return NULL;
    }

  memset(dev, 0, size);

  dev->spi   = spi;
  dev->path  = strdup(devpath);
  dev->lower = lower;

  nxsem_init(&dev->dev_sem, 0, 1);

  dev->pfd   = NULL;

  ret = gs2200m_initialize(dev, lower);

  if (ret < 0)
    {
      wlerr("Failed to initialize driver: %d\n", ret);
      goto errout;
    }

  ret = register_driver(devpath, &g_gs2200m_fops, 0666, dev);

  if (ret < 0)
    {
      wlerr("Failed to register driver: %d\n", ret);
      goto errout;
    }

  ret = netdev_register(&dev->net_dev, NET_LL_IEEE80211);

  return (FAR void *)dev;

errout:
  kmm_free(dev);
  return NULL;
}
