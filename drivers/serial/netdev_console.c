/****************************************************************************
 * drivers/serial/netdev_console.c
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

#include <errno.h>
#include <debug.h>
#include <string.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <nuttx/poll.h>
#include <netinet/in.h>

#include <nuttx/clock.h>
#include <nuttx/init.h>
#include <nuttx/mutex.h>
#include <nuttx/signal.h>
#include <nuttx/net/net.h>
#include <nuttx/serial/serial.h>
#include <nuttx/wqueue.h>

#ifdef CONFIG_NET_CAN
#  include <nuttx/can.h>
#  ifdef CONFIG_NET_CAN_ISOTP
#    include <nuttx/can/isotp.h>
#  endif
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct netdev_consol_s
{
  struct uart_dev_s dev;
  struct socket     psock;
  struct work_s     txwork;
  struct work_s     rxwork;
  struct work_s     ctrlwork;
#ifdef CONFIG_CONSOLE_NETDEV_WORKQUEUE
  FAR struct kwork_wqueue_s *wqueue;
#endif
  mutex_t           lock;
  int               sockerr;
  bool              wantopen;
  bool              opened;
  uint16_t          rxdgram_off;
  uint16_t          rxdgram_len;
  uint8_t           rxdgram[CONFIG_CONSOLE_NETDEV_RXBUFSIZE];
  bool              rxint;
  struct pollfd_s   fds;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_NET_UDP
static int  netdev_console_udp_init(FAR struct uart_dev_s *dev);
#endif
#ifdef CONFIG_NET_CAN
static int  netdev_console_can_init(FAR struct uart_dev_s *dev);
#endif

static int  netdev_console_setup(FAR struct uart_dev_s *dev);
static void netdev_console_shutdown(FAR struct uart_dev_s *dev);
static int  netdev_console_attach(FAR struct uart_dev_s *dev);
static void netdev_console_detach(FAR struct uart_dev_s *dev);
static void netdev_console_rxint(FAR struct uart_dev_s *dev, bool enable);
#ifdef CONFIG_SERIAL_IFLOWCONTROL
static bool netdev_console_rxflowcontrol(FAR struct uart_dev_s *dev,
                                         unsigned int nbuffered,
                                         bool upper);
#endif
static void netdev_console_dmasend(FAR struct uart_dev_s *dev);
static void netdev_console_dmareceive(FAR struct uart_dev_s *dev);
static void netdev_console_dmarxfree(FAR struct uart_dev_s *dev);
static void netdev_console_dmatxavail(FAR struct uart_dev_s *dev);
static void netdev_console_send(FAR struct uart_dev_s *dev, int ch);
static void netdev_console_txint(FAR struct uart_dev_s *dev, bool enable);
static bool netdev_console_txready(FAR struct uart_dev_s *dev);
static bool netdev_console_txempty(FAR struct uart_dev_s *dev);
static void netdev_console_poll(FAR struct pollfd_s *fds);
static void netdev_console_txwork(FAR void *arg);
static void netdev_console_rxwork(FAR void *arg);
static void netdev_console_ctrlwork(FAR void *arg);
static void netdev_console_queue(FAR struct netdev_consol_s *priv,
                                 FAR struct work_s *work, worker_t worker,
                                 clock_t delay);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char g_netdev_console_rxbuf[CONFIG_CONSOLE_NETDEV_RXBUFSIZE];
static char g_netdev_console_txbuf[CONFIG_CONSOLE_NETDEV_TXBUFSIZE];

static const struct uart_ops_s g_netdev_console_ops =
{
  .setup         = netdev_console_setup,
  .shutdown      = netdev_console_shutdown,
  .attach        = netdev_console_attach,
  .detach        = netdev_console_detach,
  .rxint         = netdev_console_rxint,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol = netdev_console_rxflowcontrol,
#endif
  .dmasend       = netdev_console_dmasend,
  .dmareceive    = netdev_console_dmareceive,
  .dmarxfree     = netdev_console_dmarxfree,
  .dmatxavail    = netdev_console_dmatxavail,
  .send          = netdev_console_send,
  .txint         = netdev_console_txint,
  .txready       = netdev_console_txready,
  .txempty       = netdev_console_txempty,
};

static struct netdev_consol_s g_netdev_console =
{
  .dev =
  {
    .ops = &g_netdev_console_ops,
    .xmit =
    {
      .size   = CONFIG_CONSOLE_NETDEV_TXBUFSIZE,
      .buffer = g_netdev_console_txbuf,
    },
    .recv =
    {
      .size   = CONFIG_CONSOLE_NETDEV_RXBUFSIZE,
      .buffer = g_netdev_console_rxbuf,
    },
    .isconsole = true
  },
  .lock = NXMUTEX_INITIALIZER,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_CONSOLE_NETDEV_UDP
/****************************************************************************
 * Name: netdev_console_udp_init
 ****************************************************************************/

static int netdev_console_udp_init(FAR struct uart_dev_s *dev)
{
  FAR struct netdev_consol_s *priv = (FAR struct netdev_consol_s *)dev;
  struct sockaddr_in          addr;
  int                         ret;

  /* Initialize UDP socket for CONSOLE TX/RX */

  ret = psock_socket(PF_INET, SOCK_DGRAM, 0, &priv->psock);
  if (ret < 0)
    {
      return ret;
    }

  /* Bind with source socket */

  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(CONFIG_CONSOLE_NETDEV_IPADDR);
  addr.sin_port = htons(CONFIG_CONSOLE_NETDEV_PORT);

  ret = psock_connect(&priv->psock, (FAR struct sockaddr *)&addr,
                      sizeof(addr));
  if (ret < 0)
    {
      psock_close(&priv->psock);
      return ret;
    }

  return OK;
}
#endif

#ifdef CONFIG_CONSOLE_NETDEV_CAN
/****************************************************************************
 * Name: netdev_console_can_init
 ****************************************************************************/

static int netdev_console_can_init(FAR struct uart_dev_s *dev)
{
  FAR struct netdev_consol_s  *priv = (FAR struct netdev_consol_s *)dev;
#ifdef CONFIG_NET_CAN_CANFD
  struct can_isotp_ll_options  ll;
#endif
  struct can_isotp_options     opts;
  struct sockaddr_can          addr;
  int                          ret;

  ret = psock_socket(PF_CAN, SOCK_DGRAM, CAN_ISOTP, &priv->psock);
  if (ret < 0)
    {
      return ret;
    }

  memset(&addr, 0, sizeof(addr));
  addr.can_family        = AF_CAN;
#ifdef CONFIG_NETDEV_IFINDEX
  addr.can_ifindex       = if_nametoindex(CONFIG_CONSOLE_NETDEV_INTF);
#else
  addr.can_ifindex       = 0;
#endif
  addr.can_addr.tp.tx_id = CONFIG_CONSOLE_NETDEV_PORT;
  addr.can_addr.tp.rx_id = CONFIG_CONSOLE_NETDEV_PORT + 1;

  ret = psock_bind(&priv->psock, (FAR struct sockaddr *)&addr,
                   sizeof(addr));
  if (ret < 0)
    {
      psock_close(&priv->psock);
      return ret;
    }

  /* Console write must be blocking */

  memset(&opts, 0, sizeof(opts));
  opts.flags = CAN_ISOTP_WAIT_TX_DONE;

  ret = psock_setsockopt(&priv->psock, SOL_CAN_ISOTP, CAN_ISOTP_OPTS,
                         &opts, sizeof(opts));
  if (ret < 0)
    {
      psock_close(&priv->psock);
      return ret;
    }

#ifdef CONFIG_NET_CAN_CANFD
  /* Always use CAN FD if enabled */

  ll.mtu      = CANFD_MTU;
  ll.tx_dl    = CANFD_MAX_DLEN;
  ll.tx_flags = 0;

  psock_setsockopt(&priv->psock, SOL_CAN_ISOTP, CAN_ISOTP_LL_OPTS, &ll,
                   sizeof(ll));
#endif

  return OK;
}
#endif

/****************************************************************************
 * Name: netdev_console_queue
 ****************************************************************************/

static void netdev_console_queue(FAR struct netdev_consol_s *priv,
                                 FAR struct work_s *work, worker_t worker,
                                 clock_t delay)
{
#ifdef CONFIG_CONSOLE_NETDEV_WORKQUEUE
  work_queue_wq(priv->wqueue, work, worker, priv, delay);
#else
  work_queue(LPWORK, work, worker, priv, delay);
#endif
}

/****************************************************************************
 * Name: netdev_console_setup
 ****************************************************************************/

static int netdev_console_setup(FAR struct uart_dev_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: netdev_console_shutdown
 ****************************************************************************/

static void netdev_console_shutdown(FAR struct uart_dev_s *dev)
{
}

/****************************************************************************
 * Name: netdev_console_sockopen
 ****************************************************************************/

static int netdev_console_sockopen(FAR struct uart_dev_s *dev)
{
  FAR struct netdev_consol_s *priv = (FAR struct netdev_consol_s *)dev;
  int ret = -EINVAL;

#ifdef CONFIG_CONSOLE_NETDEV_CAN
  ret = netdev_console_can_init(dev);
  if (ret < 0)
#endif
    {
#ifdef CONFIG_CONSOLE_NETDEV_UDP
      ret = netdev_console_udp_init(dev);
#endif
    }

  if (ret < 0)
    {
      return ret;
    }

  /* Register the poll callback so that received data wakes us up on demand
   * instead of polling the socket periodically from a work queue.
   */

  priv->fds.arg    = priv;
  priv->fds.events = POLLIN;
  priv->fds.cb     = netdev_console_poll;
  ret = psock_poll(&priv->psock, &priv->fds, true);
  if (ret < 0)
    {
      psock_close(&priv->psock);
    }

  return ret;
}

/****************************************************************************
 * Name: netdev_console_attach
 ****************************************************************************/

static int netdev_console_attach(FAR struct uart_dev_s *dev)
{
  FAR struct netdev_consol_s *priv = (FAR struct netdev_consol_s *)dev;

  /* Called with the uart spinlock held, so the socket, which may block, is
   * opened by netdev_console_ctrlwork().
   */

  priv->wantopen = true;
  netdev_console_queue(priv, &priv->ctrlwork, netdev_console_ctrlwork, 0);
  return OK;
}

/****************************************************************************
 * Name: netdev_console_detach
 ****************************************************************************/

static void netdev_console_detach(FAR struct uart_dev_s *dev)
{
  FAR struct netdev_consol_s *priv = (FAR struct netdev_consol_s *)dev;

  /* Called with the uart spinlock held: let netdev_console_ctrlwork() close
   * the socket.
   */

  priv->wantopen = false;
  netdev_console_queue(priv, &priv->ctrlwork, netdev_console_ctrlwork, 0);
}

/****************************************************************************
 * Name: netdev_console_rxint
 ****************************************************************************/

static void netdev_console_rxint(FAR struct uart_dev_s *dev, bool enable)
{
  FAR struct netdev_consol_s *priv = (FAR struct netdev_consol_s *)dev;

  priv->rxint = enable;
}

#ifdef CONFIG_SERIAL_IFLOWCONTROL
/****************************************************************************
 * Name: netdev_console_rxflowcontrol
 ****************************************************************************/

static bool netdev_console_rxflowcontrol(FAR struct uart_dev_s *dev,
                                         unsigned int nbuffered, bool upper)
{
  return false;
}
#endif

/****************************************************************************
 * Name: netdev_console_dmasend
 ****************************************************************************/

static void netdev_console_dmasend(FAR struct uart_dev_s *dev)
{
  FAR struct netdev_consol_s *priv = (FAR struct netdev_consol_s *)dev;

  /* Called with the uart spinlock held, so the send, which may block, is
   * done by netdev_console_txwork().
   */

  netdev_console_queue(priv, &priv->txwork, netdev_console_txwork, 0);
}

/****************************************************************************
 * Name: netdev_console_dmareceive
 ****************************************************************************/

static void netdev_console_dmareceive(FAR struct uart_dev_s *dev)
{
  FAR struct netdev_consol_s *priv = (FAR struct netdev_consol_s *)dev;
  uint16_t                    space;
  uint16_t                    copylen;
  uint16_t                    first;
  uint16_t                    second;

  space = dev->dmarx.length + dev->dmarx.nlength;
  if (space == 0)
    {
      dev->dmarx.nbytes = 0;
      uart_recvchars_done(dev);
      return;
    }

  /* Console expects a continuous byte stream. UDP/ISO-TP recv returns one
   * datagram, and any bytes that do not fit in this DMA window are lost.
   * Keep one datagram here and drain it over multiple DMA windows.
   */

  if (priv->rxdgram_off >= priv->rxdgram_len)
    {
      /* Nothing buffered.  psock_recv() may block and we hold the uart
       * spinlock here, so let netdev_console_rxwork() read the socket.
       */

      netdev_console_queue(priv, &priv->rxwork, netdev_console_rxwork, 0);
      dev->dmarx.nbytes = 0;
      uart_recvchars_done(dev);
      return;
    }

  copylen = space;
  if (copylen > priv->rxdgram_len - priv->rxdgram_off)
    {
      copylen = priv->rxdgram_len - priv->rxdgram_off;
    }

  /* First DMA segment */

  first = copylen > dev->dmarx.length ? dev->dmarx.length : copylen;
  memcpy(dev->dmarx.buffer, &priv->rxdgram[priv->rxdgram_off], first);

  /* Second DMA segment */

  second = copylen - first;
  if (second > 0 && dev->dmarx.nbuffer != NULL)
    {
      memcpy(dev->dmarx.nbuffer, &priv->rxdgram[priv->rxdgram_off + first],
             second);
    }

  priv->rxdgram_off += copylen;
  if (priv->rxdgram_off >= priv->rxdgram_len)
    {
      priv->rxdgram_off = 0;
      priv->rxdgram_len = 0;
    }

  dev->dmarx.nbytes = copylen;
  uart_recvchars_done(dev);
}

/****************************************************************************
 * Name: netdev_console_dmarxfree
 ****************************************************************************/

static void netdev_console_dmarxfree(FAR struct uart_dev_s *dev)
{
  uart_recvchars_dma(dev);
}

/****************************************************************************
 * Name: netdev_console_dmatxavail
 ****************************************************************************/

static void netdev_console_dmatxavail(FAR struct uart_dev_s *dev)
{
  /* Start a new transfer only when the previous one is done. */

  if (dev->dmatx.length == 0)
    {
      uart_xmitchars_dma(dev);
    }
}

/****************************************************************************
 * Name: netdev_console_send
 ****************************************************************************/

static void netdev_console_send(FAR struct uart_dev_s *dev, int ch)
{
  int nexthead;

  /* Called from interrupt or idle context with the uart spinlock held, so
   * only put the character in the xmit buffer for netdev_console_txwork().
   * Drop it if the buffer is full, or in a panic, where the works no longer
   * run and uart_writev() sends the xmit buffer through this function.
   */

  if (OSINIT_IS_PANIC())
    {
      return;
    }

  nexthead = dev->xmit.head + 1;
  if (nexthead >= dev->xmit.size)
    {
      nexthead = 0;
    }

  if (nexthead != dev->xmit.tail)
    {
      dev->xmit.buffer[dev->xmit.head] = ch;
      dev->xmit.head = nexthead;
    }
}

/****************************************************************************
 * Name: netdev_console_txint
 ****************************************************************************/

static void netdev_console_txint(FAR struct uart_dev_s *dev, bool enable)
{
}

/****************************************************************************
 * Name: netdev_console_txready
 ****************************************************************************/

static bool netdev_console_txready(FAR struct uart_dev_s *dev)
{
  return true;
}

/****************************************************************************
 * Name: netdev_console_txempty
 ****************************************************************************/

static bool netdev_console_txempty(FAR struct uart_dev_s *dev)
{
  return true;
}

/****************************************************************************
 * Name: netdev_console_poll
 *
 * Description:
 *   Socket poll callback.  Invoked by the network stack when the console
 *   socket becomes readable.  It runs with the connection locked and before
 *   the datagram reaches the read-ahead buffer, so the receive is deferred
 *   to netdev_console_rxwork(), which waits for that lock in psock_recv().
 *
 ****************************************************************************/

static void netdev_console_poll(FAR struct pollfd_s *fds)
{
  FAR struct netdev_consol_s *priv = fds->arg;

  if ((fds->revents & POLLIN) != 0)
    {
      fds->revents &= ~POLLIN;
      if (priv->rxint)
        {
          netdev_console_queue(priv, &priv->rxwork, netdev_console_rxwork,
                               0);
        }
    }
}

/****************************************************************************
 * Name: netdev_console_txwork
 *
 * Description:
 *   Send the buffer prepared by uart_xmitchars_dma().  This runs without the
 *   uart spinlock, so psock_send() may block as it did in dmasend().
 *
 ****************************************************************************/

static void netdev_console_txwork(FAR void *arg)
{
  FAR struct netdev_consol_s *priv = arg;
  FAR struct uart_dev_s      *dev  = &priv->dev;
  int                         retry = 3;
  irqstate_t                  flags;
  ssize_t                     ret;

  nxmutex_lock(&priv->lock);

  /* Nothing to send when the buffer was flushed while the work was queued. */

  flags = uart_spinlock(dev, true);
  if (dev->dmatx.length == 0 && dev->dmatx.nlength == 0)
    {
      uart_spinunlock(dev, true, flags);
      nxmutex_unlock(&priv->lock);
      return;
    }

  uart_spinunlock(dev, true, flags);

  if (!priv->opened)
    {
      /* Keep the data until netdev_console_ctrlwork() opens the socket. */

      ret = 0;
    }
  else
    {
      do
        {
          ret = psock_send(&priv->psock, dev->dmatx.buffer,
                           dev->dmatx.length, 0);

          /* Wait for ARP */

          if (ret < 0)
            {
              nxsig_sleep(1);
            }
        }
      while (ret < 0 && retry--);
    }

  flags = uart_spinlock(dev, true);

  dev->dmatx.nbytes = ret > 0 ? ret : 0;
  uart_xmitchars_done(dev);

  /* Continue with whatever is still buffered in the xmit ring. */

  if (ret > 0 && dev->xmit.head != dev->xmit.tail)
    {
      uart_xmitchars_dma(dev);
    }

  uart_spinunlock(dev, true, flags);
  nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Name: netdev_console_rxwork
 *
 * Description:
 *   Read one datagram from the console socket and hand it to the serial
 *   upper half.  This runs without the uart spinlock, so psock_recv() may
 *   block on the connection lock.
 *
 ****************************************************************************/

static void netdev_console_rxwork(FAR void *arg)
{
  FAR struct netdev_consol_s *priv = arg;
  FAR struct uart_dev_s      *dev  = &priv->dev;
  irqstate_t                  flags;
  ssize_t                     ret;

  nxmutex_lock(&priv->lock);

  /* The socket is not open, or the console was closed. */

  if (!priv->opened || !priv->rxint)
    {
      nxmutex_unlock(&priv->lock);
      return;
    }

  flags = uart_spinlock(dev, true);

  /* A previous datagram is still waiting to be drained. */

  if (priv->rxdgram_off < priv->rxdgram_len)
    {
      uart_spinunlock(dev, true, flags);
      nxmutex_unlock(&priv->lock);
      return;
    }

  priv->rxdgram_off = 0;
  priv->rxdgram_len = 0;
  uart_spinunlock(dev, true, flags);

  ret = psock_recv(&priv->psock, priv->rxdgram, sizeof(priv->rxdgram),
                   MSG_DONTWAIT);
  if (ret > 0)
    {
      flags = uart_spinlock(dev, true);
      priv->rxdgram_len = ret;
      uart_recvchars_dma(dev);
      uart_spinunlock(dev, true, flags);
    }

  nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Name: netdev_console_ctrlwork
 *
 * Description:
 *   Open or close the console socket as requested by attach() and detach().
 *   Closing waits for txwork and rxwork through priv->lock.  Opening is
 *   retried every second while the console is open.
 *
 ****************************************************************************/

static void netdev_console_ctrlwork(FAR void *arg)
{
  FAR struct netdev_consol_s *priv = arg;
  FAR struct uart_dev_s      *dev  = &priv->dev;
  irqstate_t                  flags;
  bool                        wantopen;
  bool                        opened;
  int                         ret = OK;

  nxmutex_lock(&priv->lock);

  flags = uart_spinlock(dev, true);
  wantopen = priv->wantopen;
  if (wantopen != priv->opened)
    {
      priv->rxdgram_off = 0;
      priv->rxdgram_len = 0;
    }

  uart_spinunlock(dev, true, flags);

  if (wantopen && !priv->opened)
    {
      ret = netdev_console_sockopen(dev);
      priv->opened = ret >= 0;
    }
  else if (!wantopen && priv->opened)
    {
      psock_poll(&priv->psock, &priv->fds, false);
      psock_close(&priv->psock);
      priv->opened = false;
    }

  /* Do not log here: the log may go to this console, and the writer would
   * then wait on the work that has to send it.
   */

  priv->sockerr = ret;
  opened = priv->opened;
  nxmutex_unlock(&priv->lock);

  if (ret < 0)
    {
      netdev_console_queue(priv, &priv->ctrlwork, netdev_console_ctrlwork,
                           SEC2TICK(1));
    }

  /* Send what was written while the socket was not open. */

  flags = uart_spinlock(dev, true);
  if (opened && dev->dmatx.length == 0 && dev->xmit.head != dev->xmit.tail)
    {
      uart_xmitchars_dma(dev);
    }

  uart_spinunlock(dev, true, flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netdev_console_init
 *
 * Description:
 *   Register a NETDEV console channel
 *
 * Input Parameters:
 *   dev - The lower half device driver structure to be registered.
 *
 * Returned Value:
 *   Zero on success. A negated errno value is returned on a failure.
 *
 ****************************************************************************/

int netdev_console_init(void)
{
#ifdef CONFIG_CONSOLE_NETDEV_WORKQUEUE
  g_netdev_console.wqueue =
    work_queue_create("netdev_console", CONFIG_CONSOLE_NETDEV_WORKPRIORITY,
                      NULL, CONFIG_CONSOLE_NETDEV_WORKSTACKSIZE, 1);
  if (g_netdev_console.wqueue == NULL)
    {
      return -ENOMEM;
    }
#endif

  /* Register as console */

  return uart_register("/dev/console", &g_netdev_console.dev);
}
