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
#include <netinet/in.h>

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
 * Pre-processor definitions
 ****************************************************************************/

#define CONSOLE_WORK_DELAY USEC2TICK(CONFIG_CONSOLE_NETDEV_DELAY_USEC)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct netdev_consol_s
{
  struct uart_dev_s dev;
  struct socket     psock;
  uint16_t          rxdgram_off;
  uint16_t          rxdgram_len;
  uint8_t           rxdgram[CONFIG_CONSOLE_NETDEV_RXBUFSIZE];
  bool              rxint;
  struct work_s     work;
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
static void netdev_console_work(FAR void *arg);

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
      nerr("ERROR: Failed to create a socket: %d\n", ret);
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
      nerr("ERROR: Failed to connect socket: %d\n", ret);
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
      nerr("ERROR: Failed to create ISO-TP socket: %d\n", ret);
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
      nerr("ERROR: Failed to bind ISO-TP socket: %d\n", ret);
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
      nerr("ERROR: Failed to configure ISO-TP socket: %d\n", ret);
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
 * Name: netdev_console_attach
 ****************************************************************************/

static int netdev_console_attach(FAR struct uart_dev_s *dev)
{
  FAR struct netdev_consol_s *priv = (FAR struct netdev_consol_s *)dev;

  priv->rxdgram_off = 0;
  priv->rxdgram_len = 0;

#ifdef CONFIG_CONSOLE_NETDEV_UDP
  return netdev_console_udp_init(dev);
#endif

#ifdef CONFIG_CONSOLE_NETDEV_CAN
  return netdev_console_can_init(dev);
#endif
}

/****************************************************************************
 * Name: netdev_console_detach
 ****************************************************************************/

static void netdev_console_detach(FAR struct uart_dev_s *dev)
{
  FAR struct netdev_consol_s *priv = (FAR struct netdev_consol_s *)dev;

  priv->rxdgram_off = 0;
  priv->rxdgram_len = 0;
  psock_close(&priv->psock);
}

/****************************************************************************
 * Name: netdev_console_rxint
 ****************************************************************************/

static void netdev_console_rxint(FAR struct uart_dev_s *dev, bool enable)
{
  FAR struct netdev_consol_s *priv = (FAR struct netdev_consol_s *)dev;

  priv->rxint = enable;
  if (enable)
    {
      work_queue(LPWORK, &priv->work, netdev_console_work, dev, 0);
    }
  else
    {
      work_cancel(LPWORK, &priv->work);
    }
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
  FAR struct netdev_consol_s *priv  = (FAR struct netdev_consol_s *)dev;
  int                         retry = 3;
  ssize_t                     ret;

  do
    {
      ret = psock_send(&priv->psock, dev->dmatx.buffer, dev->dmatx.length,
                       0);

      /* Wait for ARP */

      if (ret < 0)
        {
          nxsig_sleep(1);
        }
    }
  while (ret < 0 && retry--);

  dev->dmatx.nbytes = ret > 0 ? ret : 0;
  uart_xmitchars_done(dev);
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
  ssize_t                     ret;

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
      ret = psock_recv(&priv->psock, priv->rxdgram, sizeof(priv->rxdgram),
                       MSG_DONTWAIT);
      if (ret <= 0)
        {
          dev->dmarx.nbytes = 0;
          uart_recvchars_done(dev);
          return;
        }

      priv->rxdgram_off = 0;
      priv->rxdgram_len = ret;
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
  uart_xmitchars_dma(dev);
}

/****************************************************************************
 * Name: netdev_console_send
 ****************************************************************************/

static void netdev_console_send(FAR struct uart_dev_s *dev, int ch)
{
  FAR struct netdev_consol_s *priv = (FAR struct netdev_consol_s *)dev;
  char                        tmp  = ch;

  psock_send(&priv->psock, &tmp, 1, 0);
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
 * Name: netdev_console_work
 ****************************************************************************/

static void netdev_console_work(FAR void *arg)
{
  FAR struct netdev_consol_s *priv = arg;

  if (priv->rxint)
    {
      uart_dmarxfree(&priv->dev);
    }

  work_queue_next(LPWORK, &priv->work, netdev_console_work, arg,
                  CONSOLE_WORK_DELAY);
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
  /* Register as console */

  return uart_register("/dev/console", &g_netdev_console.dev);
}
