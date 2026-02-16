/****************************************************************************
 * drivers/rpmsg/rpmsg_port_uart.c
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
#include <execinfo.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>

#include <nuttx/crc16.h>
#include <nuttx/event.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/mutex.h>
#include <nuttx/nuttx.h>
#include <nuttx/power/pm.h>
#include <nuttx/reboot_notifier.h>
#include <nuttx/semaphore.h>

#include "rpmsg_port.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RPMSG_PORT_UART_START              0x7f
#define RPMSG_PORT_UART_CONNREQ            0x7e
#define RPMSG_PORT_UART_CONNACK            0x7d
#define RPMSG_PORT_UART_ESCAPE             0x7c
#define RPMSG_PORT_UART_SUSPEND            0x7b
#define RPMSG_PORT_UART_RESUME             0x7a
#define RPMSG_PORT_UART_STAYWAKE1          0x79
#define RPMSG_PORT_UART_STAYWAKEACK1       0x78
#define RPMSG_PORT_UART_RELAXWAKE          0x77
#define RPMSG_PORT_UART_POWEROFF           0x76
#define RPMSG_PORT_UART_STAYWAKE2          0x75
#define RPMSG_PORT_UART_STAYWAKEACK2       0x74
#define RPMSG_PORT_UART_END                0x70
#define RPMSG_PORT_UART_ESCAPE_MASK        0x20

#define RPMSG_PORT_UART_BUFLEN             256
#define RPMSG_PORT_UART_DBG_BUFLEN         4096

#define RPMSG_PORT_UART_RX_WAIT_START      1
#define RPMSG_PORT_UART_RX_RECV_NORMAL     2
#define RPMSG_PORT_UART_RX_RECV_ESCAPE     3

#define RPMSG_PORT_UART_TIMEOUT            100000 /* us */

#define RPMSG_PORT_UART_BACKTRACE          12

#define RPMSG_PORT_UART_EVT_WAKED          (1 << 0) /* Peer is waked state or not */
#define RPMSG_PORT_UART_EVT_WAKING         (1 << 1) /* Local is waking peer or not */
#define RPMSG_PORT_UART_EVT_CONNED         (1 << 2) /* Connected with peer or not */
#define RPMSG_PORT_UART_EVT_TX             (1 << 3) /* TX is ready or not */

#ifdef CONFIG_RPMSG_PORT_UART_CRC
#  define rpmsg_port_uart_crc16(hdr)       crc16ibm((FAR uint8_t *)&(hdr)->cmd, \
                                                     (hdr)->len - sizeof((hdr)->crc))
#else
#  define rpmsg_port_uart_crc16(hdr)       0
#endif

#ifdef CONFIG_RPMSG_PORT_UART_DEBUG
#  define rpmsgdump(m,b,s)                 lib_dumpbuffer(m, (FAR const uint8_t *)b, s)
#  define rpmsgdbg                         rpmsgerr
#else
#  define rpmsgdump(m,b,s)
#  define rpmsgdbg(f,...)
#endif

#define rpmsgerrdump(m,b,s)                lib_dumpbuffer(m, (FAR const uint8_t *)b, s)
#define rpmsgvbs                           rpmsgerr

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rpmsg_port_uart_s
{
  struct rpmsg_port_s             port;     /* Rpmsg port device */
  struct file                     file;     /* Indicate uart device */
  char                            localcpu[RPMSG_NAME_SIZE];
  FAR struct rpmsg_port_header_s *rx_hdr;
  uint16_t                        rx_next;
  uint8_t                         rx_state;
  rpmsg_port_rx_cb_t              rx_cb;
  pid_t                           rx_tid;
  uint8_t                         rx_dbg_buf[RPMSG_PORT_UART_DBG_BUFLEN];
  int                             rx_dbg_idx;
  nxevent_t                       event;
  struct notifier_block           nb;       /* Reboot notifier block */
  pid_t                           tx_tid;
  uint8_t                         tx_staywake;
#ifdef CONFIG_PM
  struct pm_wakelock_s            tx_wakelock;
  struct pm_wakelock_s            rx_wakelock;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void rpmsg_port_uart_send_data(FAR struct rpmsg_port_uart_s *rpuart,
                                      FAR struct rpmsg_port_header_s *hdr);

static void rpmsg_port_uart_tx_ready(FAR struct rpmsg_port_s *port);
static int
rpmsg_port_uart_queue_noavail(FAR struct rpmsg_port_s *port,
                              FAR struct rpmsg_port_queue_s *queue);
static void rpmsg_port_uart_register_callback(FAR struct rpmsg_port_s *port,
                                              rpmsg_port_rx_cb_t callback);
static void rpmsg_port_uart_dump(FAR struct rpmsg_port_s *port);

static void rpmsg_port_uart_rx_worker(FAR struct rpmsg_port_uart_s *rpuart,
                                      bool process_rx);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rpmsg_port_ops_s g_rpmsg_port_uart_ops =
{
  .notify_tx_ready      = rpmsg_port_uart_tx_ready,
  .notify_rx_free       = NULL,
  .notify_queue_noavail = rpmsg_port_uart_queue_noavail,
  .register_callback    = rpmsg_port_uart_register_callback,
  .dump                 = rpmsg_port_uart_dump,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpmsg_port_uart_check
 ****************************************************************************/

static inline bool
rpmsg_port_uart_check(FAR struct rpmsg_port_uart_s *rpuart,
                      nxevent_mask_t event)
{
  return nxevent_trywait(&rpuart->event, event, NXEVENT_WAIT_NOCLEAR) != 0;
}

/****************************************************************************
 * Name: rpmsg_port_uart_set
 ****************************************************************************/

static inline bool
rpmsg_port_uart_set(FAR struct rpmsg_port_uart_s *rpuart,
                    nxevent_mask_t event)
{
  bool ret = rpmsg_port_uart_check(rpuart, event);
  nxevent_post(&rpuart->event, event, 0);
  return ret;
}

/****************************************************************************
 * Name: rpmsg_port_uart_wait
 ****************************************************************************/

static inline bool
rpmsg_port_uart_wait(FAR struct rpmsg_port_uart_s *rpuart,
                     nxevent_mask_t event, bool clear, bool timeout)
{
  nxevent_flags_t eflags = clear ? 0 : NXEVENT_WAIT_NOCLEAR;
  uint32_t delay = timeout ? USEC2TICK(RPMSG_PORT_UART_TIMEOUT) : UINT32_MAX;

  return nxevent_tickwait(&rpuart->event, event, eflags, delay) != 0;
}

/****************************************************************************
 * Name: rpmsg_port_uart_clear
 ****************************************************************************/

static inline bool
rpmsg_port_uart_clear(FAR struct rpmsg_port_uart_s *rpuart,
                      nxevent_mask_t event)
{
  return nxevent_trywait(&rpuart->event, event, 0) != 0;
}

/****************************************************************************
 * Name: rpmsg_port_uart_send_one
 ****************************************************************************/

static inline void
rpmsg_port_uart_send_one(FAR struct rpmsg_port_uart_s *rpuart, uint8_t ch)
{
  ssize_t ret = file_write(&rpuart->file, &ch, 1);
  if (ret != 1)
    {
      rpmsgerr("Send ch %u failed, ret=%zd\n", ch, ret);
      PANIC();
    }
}

/****************************************************************************
 * Name: rpmsg_port_uart_staywake
 *
 * Description:
 *   Send wakeup data to peer if peer is suspended.
 *
 ****************************************************************************/

static void rpmsg_port_uart_staywake(FAR struct rpmsg_port_uart_s *rpuart)
{
  pm_wakelock_stay(&rpuart->tx_wakelock);

  if (rpmsg_port_uart_check(rpuart, RPMSG_PORT_UART_EVT_WAKED))
    {
      rpmsgdbg("Peer is running, no need to wakeup 0x%x 0x%x\n",
               rpuart->tx_staywake, rpuart->event.events);
      return;
    }

  rpmsg_port_uart_set(rpuart, RPMSG_PORT_UART_EVT_WAKING);
  for (; ; )
    {
      rpmsgdbg("Try to wakeup peer 0x%x 0x%x\n",
               rpuart->tx_staywake, rpuart->event.events);
      rpmsg_port_uart_send_one(rpuart, rpuart->tx_staywake);
      if (rpmsg_port_uart_wait(rpuart, RPMSG_PORT_UART_EVT_WAKED,
                               false, true))
        {
          break;
        }
    }

  rpmsg_port_uart_clear(rpuart, RPMSG_PORT_UART_EVT_WAKING);
  rpmsgdbg("Wakeup peer success 0x%x 0x%x\n",
           rpuart->tx_staywake, rpuart->event.events);
}

/****************************************************************************
 * Name: rpmsg_port_uart_relaxwake
 ****************************************************************************/

static void rpmsg_port_uart_relaxwake(FAR struct rpmsg_port_uart_s *rpuart)
{
  if (rpmsg_port_uart_check(rpuart, RPMSG_PORT_UART_EVT_WAKED))
    {
      rpmsgdbg("Try allow peer sleep 0x%x 0x%x\n",
               rpuart->tx_staywake, rpuart->event.events);
      rpmsg_port_uart_send_one(rpuart, RPMSG_PORT_UART_RELAXWAKE);
      rpmsg_port_uart_clear(rpuart, RPMSG_PORT_UART_EVT_WAKED);
    }

  pm_wakelock_relax(&rpuart->tx_wakelock);
  if (rpuart->tx_staywake == RPMSG_PORT_UART_STAYWAKE1)
    {
      rpuart->tx_staywake = RPMSG_PORT_UART_STAYWAKE2;
    }
  else
    {
      rpuart->tx_staywake = RPMSG_PORT_UART_STAYWAKE1;
    }
}

/****************************************************************************
 * Name: rpmsg_port_uart_send_packet
 *
 * Description:
 *   Send a data packet.
 *
 ****************************************************************************/

static void rpmsg_port_uart_send_packet(FAR struct rpmsg_port_uart_s *rpuart,
                                        FAR const void *data, size_t datalen)
{
  rpmsgdump("TX Packed Data", data, datalen);
  rpmsgdbg("Sent %zu Data\n", datalen);

  while (datalen > 0)
    {
      ssize_t ret = file_write(&rpuart->file, data, datalen);
      DEBUGASSERT(ret >= 0);

      data = (FAR uint8_t *)data + ret;
      datalen -= ret;
    }
}

/****************************************************************************
 * Name: rpmsg_port_uart_send_frame
 *
 * Description:
 *   Send a frame.
 *
 ****************************************************************************/

static void rpmsg_port_uart_send_frame(FAR struct rpmsg_port_uart_s *rpuart,
                                       FAR const void *data, size_t datalen)
{
  uint8_t buf[RPMSG_PORT_UART_BUFLEN];
  uint8_t ch;
  size_t next = 0;

  rpmsgdump("Send Data", data, datalen);

  /* Pack start frame char first */

  buf[next++] = RPMSG_PORT_UART_START;

  /* Pack the data */

  for (; datalen-- > 0; data = (FAR uint8_t *)data + 1)
    {
      ch = *(FAR uint8_t *)data;
      if (ch >= RPMSG_PORT_UART_END && ch <= RPMSG_PORT_UART_START)
        {
          buf[next++] = RPMSG_PORT_UART_ESCAPE;
          buf[next++] = ch ^ RPMSG_PORT_UART_ESCAPE_MASK;
        }
      else
        {
          buf[next++] = ch;
        }

      if (next > RPMSG_PORT_UART_BUFLEN - 2)
        {
          rpmsg_port_uart_send_packet(rpuart, buf, next);
          next = 0;
        }
    }

  /* Pack end frame char */

  buf[next++] = RPMSG_PORT_UART_END;

  /* Send this frame */

  rpmsg_port_uart_send_packet(rpuart, buf, next);
}

/****************************************************************************
 * Name: rpmsg_port_uart_send_data
 *
 * Description:
 *   Send a data frame.
 *
 ****************************************************************************/

static void rpmsg_port_uart_send_data(FAR struct rpmsg_port_uart_s *rpuart,
                                      FAR struct rpmsg_port_header_s *hdr)
{
  rpmsgdbg("Send data len: %" PRIu16 "\n", hdr->len);

  hdr->cmd = 0;
  hdr->avail = 0;
  hdr->crc = rpmsg_port_uart_crc16(hdr);

  rpmsg_port_uart_send_frame(rpuart, hdr, hdr->len);
}

/****************************************************************************
 * Name: rpmsg_port_uart_tx_ready
 ****************************************************************************/

static void rpmsg_port_uart_tx_ready(FAR struct rpmsg_port_s *port)
{
  FAR struct rpmsg_port_uart_s *rpuart =
    (FAR struct rpmsg_port_uart_s *)port;

  if (rpmsg_port_uart_check(rpuart, RPMSG_PORT_UART_EVT_CONNED))
    {
      rpmsg_port_uart_set(rpuart, RPMSG_PORT_UART_EVT_TX);
    }
  else
    {
      rpmsg_port_drop_packets(&rpuart->port, RPMSG_PORT_DROP_TXQ);
    }
}

/****************************************************************************
 * Name: rpmsg_port_uart_queue_noavail
 ****************************************************************************/

static int
rpmsg_port_uart_queue_noavail(FAR struct rpmsg_port_s *port,
                              FAR struct rpmsg_port_queue_s *queue)
{
  FAR struct rpmsg_port_uart_s *rpuart =
    (FAR struct rpmsg_port_uart_s *)port;

  if (nxsched_gettid() != rpuart->rx_tid || queue != &rpuart->port.txq)
    {
      return -ENOTSUP;
    }

  rpmsg_port_uart_rx_worker(rpuart, false);
  return 0;
}

/****************************************************************************
 * Name: rpmsg_port_uart_register_callback
 ****************************************************************************/

static void rpmsg_port_uart_register_callback(FAR struct rpmsg_port_s *port,
                                              rpmsg_port_rx_cb_t callback)
{
  FAR struct rpmsg_port_uart_s *rpuart =
    (FAR struct rpmsg_port_uart_s *)port;

  rpuart->rx_cb = callback;
}

/****************************************************************************
 * Name: rpmsg_port_uart_dump
 ****************************************************************************/

static void rpmsg_port_uart_dump(FAR struct rpmsg_port_s *port)
{
  FAR struct rpmsg_port_uart_s *rpuart =
    (FAR struct rpmsg_port_uart_s *)port;

  rpmsgvbs("Dump rpmsg port uart:\n");
  rpmsgvbs("Event: 0x%lx\n", rpuart->event.events);
  if (rpuart->rx_tid != 0)
    {
      rpmsgvbs("Dump rx thread: %d\n", rpuart->rx_tid);
      sched_dumpstack(rpuart->rx_tid);
    }
  else
    {
      rpmsgvbs("Rx thread not running\n");
    }

  if (rpuart->tx_tid != 0)
    {
      rpmsgvbs("Dump tx thread: %d\n", rpuart->tx_tid);
      sched_dumpstack(rpuart->tx_tid);
    }
  else
    {
      rpmsgvbs("Tx thread not running\n");
    }
}

/****************************************************************************
 * Name: rpmsg_port_uart_process_rx_conn
 ****************************************************************************/

static void
rpmsg_port_uart_process_rx_conn(FAR struct rpmsg_port_uart_s *rpuart,
                                uint8_t ch)
{
  if (ch == RPMSG_PORT_UART_CONNREQ)
    {
      rpmsgvbs("Connect Request Command 0x%lx\n", rpuart->event.events);
      if (rpmsg_port_uart_set(rpuart, RPMSG_PORT_UART_EVT_CONNED))
        {
          rpmsg_port_drop_packets(&rpuart->port, RPMSG_PORT_DROP_TXQ);
          rpmsg_port_unregister(&rpuart->port);
        }

      rpmsg_port_drop_packets(&rpuart->port, RPMSG_PORT_DROP_ALL);
      rpmsg_port_uart_send_one(rpuart, RPMSG_PORT_UART_CONNACK);
      rpmsg_port_register(&rpuart->port, rpuart->localcpu);
    }
  else if (ch == RPMSG_PORT_UART_CONNACK)
    {
      rpmsgvbs("Connect Ack Command 0x%lx\n", rpuart->event.events);
      if (!rpmsg_port_uart_set(rpuart, RPMSG_PORT_UART_EVT_CONNED))
        {
          rpmsg_port_drop_packets(&rpuart->port, RPMSG_PORT_DROP_ALL);
          rpmsg_port_register(&rpuart->port, rpuart->localcpu);
        }
    }
}

/****************************************************************************
 * Name: rpmsg_port_uart_process_rx_cmd
 ****************************************************************************/

static bool
rpmsg_port_uart_process_rx_cmd(FAR struct rpmsg_port_uart_s *rpuart,
                               FAR struct rpmsg_port_header_s **hdr,
                               FAR uint8_t *state, uint8_t ch)
{
  bool iscmd = true;
  int count;

  switch (ch)
    {
      case RPMSG_PORT_UART_CONNREQ:
      case RPMSG_PORT_UART_CONNACK:
        if (*hdr != NULL)
          {
            rpmsg_port_queue_return_buffer(&rpuart->port.rxq, *hdr);
            *hdr = NULL;
          }

        rpmsg_port_uart_process_rx_conn(rpuart, ch);
        *state = RPMSG_PORT_UART_RX_WAIT_START;
        break;
      case RPMSG_PORT_UART_SUSPEND:
        rpmsgdbg("Received suspend command\n");
        rpmsg_modify_signals(&rpuart->port.rpmsg, 0, RPMSG_SIGNAL_RUNNING);
        break;
      case RPMSG_PORT_UART_RESUME:
        rpmsgdbg("Received resume command\n");
        rpmsg_modify_signals(&rpuart->port.rpmsg, RPMSG_SIGNAL_RUNNING, 0);
        break;
      case RPMSG_PORT_UART_POWEROFF:
        count = pm_wakelock_staycount(&rpuart->rx_wakelock);
        rpmsgvbs("Received poweroff command %d 0x%lx\n",
                 count, rpuart->event.events);
        if (rpmsg_port_uart_check(rpuart, RPMSG_PORT_UART_EVT_WAKING))
          {
            rpmsg_port_uart_set(rpuart, RPMSG_PORT_UART_EVT_WAKED);
          }

        if (rpmsg_port_uart_clear(rpuart, RPMSG_PORT_UART_EVT_CONNED))
          {
            rpmsg_port_drop_packets(&rpuart->port, RPMSG_PORT_DROP_TXQ);
            rpmsg_port_unregister(&rpuart->port);
          }

        DEBUGVERIFY(file_ioctl(&rpuart->file, TIOCVHANGUP, 0) >= 0);
        if (count != 0)
          {
            pm_wakelock_relax(&rpuart->rx_wakelock);
          }
        break;
      case RPMSG_PORT_UART_STAYWAKE1:
        count = pm_wakelock_staycount(&rpuart->rx_wakelock);
        rpmsgdbg("Received staywake1 command %d 0x%x\n",
                 count, rpuart->event.events);
        if (count == 0)
          {
            pm_wakelock_stay(&rpuart->rx_wakelock);
          }

        rpmsg_port_uart_send_one(rpuart, RPMSG_PORT_UART_STAYWAKEACK1);
        break;
      case RPMSG_PORT_UART_STAYWAKE2:
        count = pm_wakelock_staycount(&rpuart->rx_wakelock);
        rpmsgdbg("Received staywake2 command %d 0x%x\n",
                 count, rpuart->event.events);
        if (count == 0)
          {
            pm_wakelock_stay(&rpuart->rx_wakelock);
          }

        rpmsg_port_uart_send_one(rpuart, RPMSG_PORT_UART_STAYWAKEACK2);
        break;
      case RPMSG_PORT_UART_RELAXWAKE:
        count = pm_wakelock_staycount(&rpuart->rx_wakelock);
        rpmsgdbg("Received relaxwake command %d 0x%x\n",
                 count, rpuart->event.events);
        if (count != 0)
          {
            pm_wakelock_relax(&rpuart->rx_wakelock);
          }
        break;
      case RPMSG_PORT_UART_STAYWAKEACK1:
      case RPMSG_PORT_UART_STAYWAKEACK2:
        rpmsgdbg("Received staywake ack command 0x%x 0x%x 0x%x\n",
                 ch, rpuart->tx_staywake, rpuart->event.events);
        if (ch == rpuart->tx_staywake - 1 &&
            rpmsg_port_uart_check(rpuart, RPMSG_PORT_UART_EVT_WAKING))
          {
            rpmsg_port_uart_set(rpuart, RPMSG_PORT_UART_EVT_WAKED);
          }
        break;
      default:
        if (ch < RPMSG_PORT_UART_START && ch > RPMSG_PORT_UART_END &&
            ch != RPMSG_PORT_UART_ESCAPE)
          {
            rpmsgerr("Receive Command %x\n", ch);
          }
        else
          {
            iscmd = false;
          }
        break;
    }

  return iscmd;
}

/****************************************************************************
 * Name: rpmsg_port_uart_process_rx_data
 ****************************************************************************/

static void
rpmsg_port_uart_process_rx_data(FAR struct rpmsg_port_uart_s *rpuart)
{
  FAR struct rpmsg_port_queue_s *rxq = &rpuart->port.rxq;
  FAR struct rpmsg_port_header_s *hdr;

  DEBUGASSERT(rpuart->rx_cb != NULL);

  while ((hdr = rpmsg_port_queue_get_buffer(rxq, false)) != NULL)
    {
      rpmsgdbg("Received data hdr=%p len=%u\n", hdr, hdr->len);
      rpuart->rx_cb(&rpuart->port, hdr);
    }
}

/****************************************************************************
 * Name: rpmsg_port_uart_rx_worker
 ****************************************************************************/

static void rpmsg_port_uart_rx_worker(FAR struct rpmsg_port_uart_s *rpuart,
                                      bool process_rx)
{
  FAR struct rpmsg_port_queue_s *rxq = &rpuart->port.rxq;
  FAR struct rpmsg_port_header_s *hdr = rpuart->rx_hdr;
  uint16_t next = rpuart->rx_next;
  uint8_t state = rpuart->rx_state;
  uint8_t buf[RPMSG_PORT_UART_BUFLEN];
  ssize_t ret;
  ssize_t i;

  ret = file_read(&rpuart->file, buf, sizeof(buf));
  if (ret < 0)
    {
      rpmsgerr("file_read failed, ret=%zd\n", ret);
      PANIC();
    }

  rpmsgdump("Received data:", buf, ret);

  for (i = 0; i < ret; i++)
    {
      rpuart->rx_dbg_buf[rpuart->rx_dbg_idx++] = buf[i];
      if (rpuart->rx_dbg_idx >= RPMSG_PORT_UART_DBG_BUFLEN)
        {
          rpuart->rx_dbg_idx = 0;
        }

      if (rpmsg_port_uart_process_rx_cmd(rpuart, &hdr, &state, buf[i]))
        {
          continue;
        }

      if (!rpmsg_port_uart_check(rpuart, RPMSG_PORT_UART_EVT_CONNED))
        {
          continue;
        }

      switch (state)
        {
          case RPMSG_PORT_UART_RX_WAIT_START:
            if (buf[i] == RPMSG_PORT_UART_START)
              {
                if (hdr == NULL)
                  {
                    hdr = rpmsg_port_queue_get_available_buffer(rxq,
                                                                true);
                    next = 0;
                  }

                state = RPMSG_PORT_UART_RX_RECV_NORMAL;
              }
            else if (buf[i] == RPMSG_PORT_UART_END)
              {
                rpmsgerr("Recv dup end char buf[%zd]=%x dbg_i=%d\n",
                         i, buf[i], rpuart->rx_dbg_idx);
                rpmsgerrdump("Recv dbg:", rpuart->rx_dbg_buf,
                             sizeof(rpuart->rx_dbg_buf));
                rpmsgerrdump("Recv error:", buf, ret);
              }
            else
              {
                rpmsgerr("Recv non start char buf[%zd]=%x dbg_i=%d\n",
                          i, buf[i], rpuart->rx_dbg_idx);
              }
            break;

          case RPMSG_PORT_UART_RX_RECV_NORMAL:
            if (buf[i] == RPMSG_PORT_UART_START)
              {
                rpmsgerr("Recv dup start char, len=%u next=%u i=%zd "
                         "dbg_i=%d\n", hdr->len, next, i,
                         rpuart->rx_dbg_idx);
                rpmsgerrdump("Recv dbg:", rpuart->rx_dbg_buf,
                              sizeof(rpuart->rx_dbg_buf));
                rpmsgerrdump("Recv error:", buf, ret);
                rpmsgerrdump("Recv hdr:", hdr, hdr->len);
                state = RPMSG_PORT_UART_RX_WAIT_START;
                next = 0;
              }
            else if (buf[i] == RPMSG_PORT_UART_END)
              {
                if (hdr->len != next || (hdr->crc != 0 &&
                    hdr->crc != rpmsg_port_uart_crc16(hdr)))
                  {
                    rpmsgerr("Recv error crc=%u len=%u next=%u i=%zd "
                             "dbg_i=%d\n", hdr->crc, hdr->len, next, i,
                             rpuart->rx_dbg_idx);
                    rpmsgerrdump("Recv dbg:", rpuart->rx_dbg_buf,
                                 sizeof(rpuart->rx_dbg_buf));
                    rpmsgerrdump("Recv error:", buf, ret);
                    rpmsgerrdump("Recv hdr:", hdr, hdr->len);
                  }

                rpmsg_port_queue_add_buffer(rxq, hdr);

                state = RPMSG_PORT_UART_RX_WAIT_START;
                hdr = NULL;
              }
            else if (buf[i] == RPMSG_PORT_UART_ESCAPE)
              {
                state = RPMSG_PORT_UART_RX_RECV_ESCAPE;
              }
            else if (next < rxq->len)
              {
                *((FAR char *)hdr + next++) = buf[i];
              }
            else
              {
                rpmsgerr("Recv len %u exceed buffer len %u\n",
                         next, rxq->len);
              }
            break;

          case RPMSG_PORT_UART_RX_RECV_ESCAPE:
            if (next < rxq->len)
              {
                *((FAR char *)hdr + next++) =
                  buf[i] ^ RPMSG_PORT_UART_ESCAPE_MASK;
              }
            else
              {
                rpmsgerr("Recv escape len %u exceed buffer len %u\n",
                         next, rxq->len);
              }

            state = RPMSG_PORT_UART_RX_RECV_NORMAL;
            break;

          default:
            PANIC();
        }
    }

  rpuart->rx_hdr = hdr;
  rpuart->rx_next = next;
  rpuart->rx_state = state;

  if (process_rx)
    {
      rpmsg_port_uart_process_rx_data(rpuart);
    }
}

/****************************************************************************
 * Name: rpmsg_port_uart_rx_thread
 ****************************************************************************/

static int rpmsg_port_uart_rx_thread(int argc, FAR char *argv[])
{
  FAR struct rpmsg_port_uart_s *rpuart =
    (FAR struct rpmsg_port_uart_s *)(uintptr_t)strtoul(argv[2], NULL, 16);

  rpuart->rx_tid = nxsched_gettid();

  for (; ; )
    {
      rpmsg_port_uart_rx_worker(rpuart, true);
    }

  return 0;
}

/****************************************************************************
 * Name: rpmsg_port_uart_tx_thread
 ****************************************************************************/

static int rpmsg_port_uart_tx_thread(int argc, FAR char *argv[])
{
  FAR struct rpmsg_port_uart_s *rpuart =
    (FAR struct rpmsg_port_uart_s *)(uintptr_t)strtoul(argv[2], NULL, 16);
  FAR struct rpmsg_port_queue_s *txq = &rpuart->port.txq;
  FAR struct rpmsg_port_header_s *hdr;

  rpuart->tx_tid = nxsched_gettid();

  rpmsg_port_uart_staywake(rpuart);
  if (!rpmsg_port_uart_check(rpuart, RPMSG_PORT_UART_EVT_CONNED))
    {
      rpmsgvbs("Not Connected, Send Conn Req\n");
      rpmsg_port_uart_send_one(rpuart, RPMSG_PORT_UART_CONNREQ);
    }
  else
    {
      rpmsgvbs("Connected, no need Send Conn Req\n");
    }

  for (; ; )
    {
      while ((hdr = rpmsg_port_queue_get_buffer(txq, false)) != NULL)
        {
          rpmsg_port_uart_send_data(rpuart, hdr);
          rpmsg_port_queue_return_buffer(txq, hdr);
        }

      rpmsg_port_uart_relaxwake(rpuart);
      rpmsg_port_uart_wait(rpuart, RPMSG_PORT_UART_EVT_TX, true, false);
      rpmsg_port_uart_staywake(rpuart);
    }

  rpmsg_port_uart_relaxwake(rpuart);
  return 0;
}

/****************************************************************************
 * Name: rpmsg_port_uart_reboot_notifier
 ****************************************************************************/

static int rpmsg_port_uart_reboot_notifier(FAR struct notifier_block *nb,
                                           unsigned long action,
                                           FAR void *data)
{
  FAR struct rpmsg_port_uart_s *rpuart =
    container_of(nb, struct rpmsg_port_uart_s, nb);

  if ((action == SYS_POWER_OFF || action == SYS_RESTART) &&
      rpmsg_port_uart_check(rpuart, RPMSG_PORT_UART_EVT_CONNED))
    {
      rpmsg_port_uart_staywake(rpuart);
      rpmsg_port_uart_send_one(rpuart, RPMSG_PORT_UART_POWEROFF);
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpmsg_port_uart_initialize
 *
 * Description:
 *   Initialize a rpmsg_port_uart device to communicate between two chips.
 *
 * Input Parameters:
 *   cfg      - Configuration of buffers needed for communication.
 *   uartpath - Uart device path.
 *   localcpu - Local cpu name
 *
 ****************************************************************************/

int rpmsg_port_uart_initialize(FAR const struct rpmsg_port_config_s *cfg,
                               FAR const char *uartpath,
                               FAR const char *localcpu)
{
  FAR struct rpmsg_port_uart_s *rpuart;
  FAR char *argv[3];
  char arg1[32];
  int ret;
  int rx;

  rpuart = kmm_zalloc(sizeof(*rpuart));
  if (rpuart == NULL)
    {
      rpmsgerr("Malloc rptun uart failed\n");
      return -ENOMEM;
    }

  rpuart->rx_state = RPMSG_PORT_UART_RX_WAIT_START;
  rpuart->tx_staywake = RPMSG_PORT_UART_STAYWAKE1;
  nxevent_init(&rpuart->event, 0);

  /* Hardware initialize */

  ret = file_open(&rpuart->file, uartpath, O_RDWR);
  if (ret < 0)
    {
      rpmsgerr("Open uart device failed, ret=%d\n", ret);
      goto err_file;
    }

  if (localcpu != NULL)
    {
      strlcpy(rpuart->localcpu, localcpu, sizeof(rpuart->localcpu));
    }

  ret = rpmsg_port_initialize(&rpuart->port, cfg, &g_rpmsg_port_uart_ops);
  if (ret < 0)
    {
      rpmsgerr("Port initialize failed, ret=%d\n", ret);
      goto err_rpmsg_port;
    }

#ifdef CONFIG_PM
  snprintf(arg1, sizeof(arg1), "rpmsg-uart-rx-%s", cfg->remotecpu);
  pm_wakelock_init(&rpuart->rx_wakelock, arg1, PM_IDLE_DOMAIN, PM_IDLE);
  snprintf(arg1, sizeof(arg1), "rpmsg-uart-tx-%s", cfg->remotecpu);
  pm_wakelock_init(&rpuart->tx_wakelock, arg1, PM_IDLE_DOMAIN, PM_IDLE);
#endif

  snprintf(arg1, sizeof(arg1), "%p", rpuart);

  argv[0] = (FAR char *)cfg->remotecpu;
  argv[1] = arg1;
  argv[2] = NULL;
  ret = kthread_create("rpmsg-uart-rx",
                       CONFIG_RPMSG_PORT_UART_RX_THREAD_PRIORITY,
                       CONFIG_RPMSG_PORT_UART_RX_THREAD_STACKSIZE,
                       rpmsg_port_uart_rx_thread, argv);
  if (ret < 0)
    {
      rpmsgerr("Rx thread create failed, ret=%d\n", ret);
      goto err_rx_thread;
    }

  rx = ret;
  argv[0] = (FAR char *)cfg->remotecpu;
  argv[1] = arg1;
  argv[2] = NULL;
  ret = kthread_create("rpmsg-uart-tx",
                       CONFIG_RPMSG_PORT_UART_TX_THREAD_PRIORITY,
                       CONFIG_RPMSG_PORT_UART_TX_THREAD_STACKSIZE,
                       rpmsg_port_uart_tx_thread, argv);
  if (ret < 0)
    {
      rpmsgerr("Tx thread create failed, ret=%d\n", ret);
      goto err_tx_thread;
    }

  rpuart->nb.notifier_call = rpmsg_port_uart_reboot_notifier;
  register_reboot_notifier(&rpuart->nb);
  return ret;

err_tx_thread:
  kthread_delete(rx);
err_rx_thread:
#ifdef CONFIG_PM
  pm_wakelock_uninit(&rpuart->tx_wakelock);
  pm_wakelock_uninit(&rpuart->rx_wakelock);
#endif
  rpmsg_port_uninitialize(&rpuart->port);
err_rpmsg_port:
  file_close(&rpuart->file);
err_file:
  nxevent_destroy(&rpuart->event);
  kmm_free(rpuart);
  return ret;
}
