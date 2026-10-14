/****************************************************************************
 * arch/arm/src/rm57/rm57_dcan_sock.c
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

/* SocketCAN (net/can) frontend for the RM57 DCAN core.  One instance of
 * this frontend serves every DCAN configured with RM57_DCANn_SOCKET;
 * instances configured as character devices are served by
 * rm57_dcan_chardev.c instead.
 *
 * Receive and error frames are dispatched to can_input() directly from
 * interrupt context (same as the FlexCAN SocketCAN drivers); transmit
 * uses the standard txavail -> devif_poll -> txpoll flow, with TX-done
 * completions scheduling a devif_poll on the work queue.
 *
 * The DCAN is a classic CAN 2.0A/B controller: CAN FD is not available,
 * so frames are always struct can_frame.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/can.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/can.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "hardware/rm57_dcan.h"
#include "rm57_dcan.h"

#if defined(CONFIG_RM57_DCAN) && defined(CONFIG_NET_CAN)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CANWORK LPWORK

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rm57can_sock_s
{
  struct net_driver_s dev;      /* Network interface (must be first) */
  struct rm57can_s   *priv;     /* Bound DCAN core instance */
  struct work_s       pollwork; /* devif_poll deferral */
  bool                bifup;    /* Interface is up */
  struct can_frame    txframe;  /* devif_poll TX staging buffer */
  struct can_frame    rxframe;  /* RX dispatch buffer */
#ifdef CONFIG_NET_CAN_ERRORS
  struct can_frame    errframe; /* Error frame dispatch buffer */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  rm57can_sk_ifup(struct net_driver_s *dev);
static int  rm57can_sk_ifdown(struct net_driver_s *dev);
static int  rm57can_sk_txavail(struct net_driver_s *dev);
#ifdef CONFIG_NETDEV_IOCTL
static int  rm57can_sk_ioctl(struct net_driver_s *dev, int cmd,
                             unsigned long arg);
#endif
static int  rm57can_sk_txpoll(struct net_driver_s *dev);
static void rm57can_sk_pollwork(void *arg);

static void rm57can_sk_rxframe(void *ctx, uint32_t arb, uint32_t mctl,
                               uint32_t da, uint32_t db);
static void rm57can_sk_txdone(void *ctx);
static void rm57can_sk_errevent(void *ctx, uint32_t es, uint32_t errc);
static void rm57can_sk_rxoverflow(void *ctx);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rm57can_cbs_s g_rm57can_sk_cbs =
{
  .rxframe  = rm57can_sk_rxframe,
  .txdone   = rm57can_sk_txdone,
  .txempty    = NULL,
  .errevent   = rm57can_sk_errevent,
  .rxoverflow = rm57can_sk_rxoverflow,
};

static struct rm57can_sock_s g_rm57can_sock[RM57_NCAN];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rm57can_sk_rxframe
 *
 * Description:
 *   Core RX callback (interrupt context): convert the raw message
 *   object registers to a struct can_frame and feed the stack.
 *
 ****************************************************************************/

static void rm57can_sk_rxframe(void *ctx, uint32_t arb, uint32_t mctl,
                               uint32_t da, uint32_t db)
{
  struct rm57can_sock_s *sock = (struct rm57can_sock_s *)ctx;
  struct can_frame *frame = &sock->rxframe;
  uint8_t dlc;
  int i;

  if (!sock->bifup)
    {
      return;
    }

  if ((arb & DCAN_IFARB_XTD) != 0)
    {
      frame->can_id = (arb & DCAN_IFARB_ID_MASK) | CAN_EFF_FLAG;
    }
  else
    {
      frame->can_id = (arb >> DCAN_IFARB_ID_STD_SHIFT) & CAN_SFF_MASK;
    }

  dlc = mctl & DCAN_IFMCTL_DLC_MASK;
  if (dlc > 8)
    {
      dlc = 8;
    }

  frame->can_dlc = dlc;

  for (i = 0; i < 4; i++)
    {
      frame->data[i]     = (uint8_t)(da >> (8 * i));
      frame->data[i + 4] = (uint8_t)(db >> (8 * i));
    }

  sock->dev.d_len = sizeof(struct can_frame);
  sock->dev.d_buf = (uint8_t *)frame;

  NETDEV_RXPACKETS(&sock->dev);
  can_input(&sock->dev);

  /* Point the packet buffer back at the TX staging frame */

  sock->dev.d_buf = (uint8_t *)&sock->txframe;
}

/****************************************************************************
 * Name: rm57can_sk_txdone
 *
 * Description:
 *   Core TX-completion callback (interrupt context).
 *
 ****************************************************************************/

static void rm57can_sk_txdone(void *ctx)
{
  struct rm57can_sock_s *sock = (struct rm57can_sock_s *)ctx;

  NETDEV_TXDONE(&sock->dev);

  /* There is a free TX mailbox again: poll the stack for more frames */

  if (work_available(&sock->pollwork))
    {
      work_queue(CANWORK, &sock->pollwork, rm57can_sk_pollwork, sock, 0);
    }
}

/****************************************************************************
 * Name: rm57can_sk_errevent
 *
 * Description:
 *   Core status/error callback (interrupt context): translate the ES
 *   snapshot into a SocketCAN error frame.
 *
 ****************************************************************************/

static void rm57can_sk_errevent(void *ctx, uint32_t es, uint32_t errc)
{
#ifdef CONFIG_NET_CAN_ERRORS
  struct rm57can_sock_s *sock = (struct rm57can_sock_s *)ctx;
  struct can_frame *frame = &sock->errframe;
  canid_t errbits = 0;
  uint8_t rec = (errc & DCAN_ERRC_REC_MASK) >> DCAN_ERRC_REC_SHIFT;
  uint8_t tec = (errc & DCAN_ERRC_TEC_MASK) >> DCAN_ERRC_TEC_SHIFT;

  if (!sock->bifup)
    {
      return;
    }

  memset(frame, 0, sizeof(*frame));

  /* Controller state */

  if ((es & DCAN_ES_BOFF) != 0)
    {
      errbits |= CAN_ERR_BUSOFF;
    }
  else
    {
      if ((es & DCAN_ES_EWARN) != 0)
        {
          errbits |= CAN_ERR_CRTL;
          frame->data[1] |= (rec >= 96) ? CAN_ERR_CRTL_RX_WARNING : 0;
          frame->data[1] |= (tec >= 96) ? CAN_ERR_CRTL_TX_WARNING : 0;
        }

      if ((es & DCAN_ES_EPASS) != 0)
        {
          errbits |= CAN_ERR_CRTL;
          frame->data[1] |= ((errc & DCAN_ERRC_RP) != 0) ?
                            CAN_ERR_CRTL_RX_PASSIVE :
                            CAN_ERR_CRTL_TX_PASSIVE;
        }
    }

  /* Last error code */

  switch (es & DCAN_ES_LEC_MASK)
    {
      case DCAN_ES_LEC_STUFF:
        errbits |= CAN_ERR_PROT;
        frame->data[2] |= CAN_ERR_PROT_STUFF;
        break;

      case DCAN_ES_LEC_FORM:
        errbits |= CAN_ERR_PROT;
        frame->data[2] |= CAN_ERR_PROT_FORM;
        break;

      case DCAN_ES_LEC_ACK:
        errbits |= CAN_ERR_ACK;
        frame->data[3]  = CAN_ERR_PROT_LOC_ACK;
        break;

      case DCAN_ES_LEC_BIT1:
        errbits |= CAN_ERR_PROT;
        frame->data[2] |= CAN_ERR_PROT_BIT1 | CAN_ERR_PROT_TX;
        break;

      case DCAN_ES_LEC_BIT0:
        errbits |= CAN_ERR_PROT;
        frame->data[2] |= CAN_ERR_PROT_BIT0 | CAN_ERR_PROT_TX;
        break;

      case DCAN_ES_LEC_CRC:
        errbits |= CAN_ERR_PROT;
        frame->data[3]  = CAN_ERR_PROT_LOC_CRC_SEQ;
        break;

      default:
        break;
    }

  if (errbits == 0)
    {
      return;
    }

  frame->can_id  = errbits | CAN_ERR_FLAG;
  frame->can_dlc = CAN_ERR_DLC;

  sock->dev.d_len = sizeof(struct can_frame);
  sock->dev.d_buf = (uint8_t *)frame;

  NETDEV_RXERRORS(&sock->dev);
  can_input(&sock->dev);

  sock->dev.d_buf = (uint8_t *)&sock->txframe;
#endif /* CONFIG_NET_CAN_ERRORS */
}

/****************************************************************************
 * Name: rm57can_sk_rxoverflow
 *
 * Description:
 *   Core callback (interrupt context): the controller overwrote a frame
 *   in its message object before the driver read it.  Reported as a
 *   controller receive overflow so it stays distinguishable from a
 *   frame the network stack itself could not take.
 *
 ****************************************************************************/

static void rm57can_sk_rxoverflow(void *ctx)
{
#ifdef CONFIG_NET_CAN_ERRORS
  struct rm57can_sock_s *sock = (struct rm57can_sock_s *)ctx;
  struct can_frame *frame = &sock->errframe;

  if (!sock->bifup)
    {
      return;
    }

  memset(frame, 0, sizeof(*frame));

  frame->can_id  = CAN_ERR_CRTL | CAN_ERR_FLAG;
  frame->can_dlc = CAN_ERR_DLC;
  frame->data[1] = CAN_ERR_CRTL_RX_OVERFLOW;

  sock->dev.d_len = sizeof(struct can_frame);
  sock->dev.d_buf = (uint8_t *)frame;

  NETDEV_RXERRORS(&sock->dev);
  can_input(&sock->dev);

  sock->dev.d_buf = (uint8_t *)&sock->txframe;
#else
  UNUSED(ctx);
#endif
}

/****************************************************************************
 * Name: rm57can_sk_txpoll
 *
 * Description:
 *   devif_poll() callback: transmit the frame the stack placed in
 *   d_buf/d_len.  Returning non-zero terminates the poll.
 *
 ****************************************************************************/

static int rm57can_sk_txpoll(struct net_driver_s *dev)
{
  struct rm57can_sock_s *sock = (struct rm57can_sock_s *)dev->d_private;

  if (dev->d_len > 0)
    {
      struct can_frame *frame = (struct can_frame *)dev->d_buf;
      bool extid = (frame->can_id & CAN_EFF_FLAG) != 0;
      bool rtr   = (frame->can_id & CAN_RTR_FLAG) != 0;
      uint32_t id = frame->can_id &
                    (extid ? CAN_EFF_MASK : CAN_SFF_MASK);
      int ret;

      ret = rm57can_send(sock->priv, extid, rtr, id, frame->can_dlc,
                         frame->data);
      if (ret < 0)
        {
          NETDEV_TXERRORS(dev);
          return -EBUSY;
        }

      NETDEV_TXPACKETS(dev);
      dev->d_len = 0;

      if (!rm57can_txready(sock->priv))
        {
          return -EBUSY;
        }
    }

  return 0;
}

/****************************************************************************
 * Name: rm57can_sk_pollwork
 *
 * Description:
 *   Run devif_poll() on the work queue after a TX completion.
 *
 ****************************************************************************/

static void rm57can_sk_pollwork(void *arg)
{
  struct rm57can_sock_s *sock = (struct rm57can_sock_s *)arg;

  net_lock();

  if (sock->bifup && rm57can_txready(sock->priv))
    {
      devif_poll(&sock->dev, rm57can_sk_txpoll);
    }

  net_unlock();
}

/****************************************************************************
 * Name: rm57can_sk_ifup / rm57can_sk_ifdown
 ****************************************************************************/

static int rm57can_sk_ifup(struct net_driver_s *dev)
{
  struct rm57can_sock_s *sock = (struct rm57can_sock_s *)dev->d_private;
  int ret;

  ret = rm57can_setup(sock->priv);
  if (ret < 0)
    {
      nerr("ERROR: DCAN%d setup failed: %d\n", sock->priv->port, ret);
      return ret;
    }

  sock->bifup = true;
  dev->d_buf  = (uint8_t *)&sock->txframe;

  netdev_carrier_on(dev);
  return OK;
}

static int rm57can_sk_ifdown(struct net_driver_s *dev)
{
  struct rm57can_sock_s *sock = (struct rm57can_sock_s *)dev->d_private;

  netdev_carrier_off(dev);
  rm57can_shutdown(sock->priv);
  sock->bifup = false;

  return OK;
}

/****************************************************************************
 * Name: rm57can_sk_txavail
 *
 * Description:
 *   The stack has new frames to send.  Runs on the caller's thread.
 *
 ****************************************************************************/

static int rm57can_sk_txavail(struct net_driver_s *dev)
{
  struct rm57can_sock_s *sock = (struct rm57can_sock_s *)dev->d_private;

  if (sock->bifup)
    {
      rm57can_sk_pollwork(sock);
    }

  return OK;
}

/****************************************************************************
 * Name: rm57can_sk_ioctl
 ****************************************************************************/

#ifdef CONFIG_NETDEV_IOCTL
static int rm57can_sk_ioctl(struct net_driver_s *dev, int cmd,
                            unsigned long arg)
{
  struct rm57can_sock_s *sock = (struct rm57can_sock_s *)dev->d_private;
  struct rm57can_s *priv = sock->priv;

  switch (cmd)
    {
#ifdef CONFIG_NETDEV_CAN_BITRATE_IOCTL
      case SIOCGCANBITRATE:
        {
          struct can_ioctl_data_s *req =
            (struct can_ioctl_data_s *)((uintptr_t)arg);
          int ntq = 1 + priv->timing.tseg1 + priv->timing.tseg2;

          req->arbi_bitrate = priv->timing.baud;
          req->arbi_samplep = ((1 + priv->timing.tseg1) * 100) / ntq;
          req->data_bitrate = 0;
          req->data_samplep = 0;
          return OK;
        }

      case SIOCSCANBITRATE:
        {
          const struct can_ioctl_data_s *req =
            (const struct can_ioctl_data_s *)((uintptr_t)arg);
          struct rm57can_timing_s timing;
          int ret;

          ret = rm57can_timing_from_baud(req->arbi_bitrate, &timing);
          if (ret < 0)
            {
              return ret;
            }

          ret = rm57can_set_timing(priv, &timing);
          if (ret < 0)
            {
              return ret;
            }

          return OK;
        }
#endif /* CONFIG_NETDEV_CAN_BITRATE_IOCTL */

#ifdef CONFIG_NETDEV_CAN_FILTER_IOCTL
      case SIOCACANSTDFILTER:
      case SIOCACANEXTFILTER:
        {
          const struct can_ioctl_filter_s *req =
            (const struct can_ioctl_filter_s *)((uintptr_t)arg);

          if (req->ftype != CAN_FILTER_MASK)
            {
              return -EINVAL;
            }

          return rm57can_add_filter(priv, cmd == SIOCACANEXTFILTER,
                                    req->fid1, req->fid2);
        }

      case SIOCDCANSTDFILTER:
      case SIOCDCANEXTFILTER:
        {
          const struct can_ioctl_filter_s *req =
            (const struct can_ioctl_filter_s *)((uintptr_t)arg);

          return rm57can_del_filter(priv, (int)req->fid1);
        }
#endif /* CONFIG_NETDEV_CAN_FILTER_IOCTL */

      default:
        return -ENOTTY;
    }
}
#endif /* CONFIG_NETDEV_IOCTL */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_dcan_sock_initialize
 *
 * Description:
 *   Bind DCAN instance n (0..3) to the SocketCAN stack and register it
 *   as network interface can<n>.
 *
 ****************************************************************************/

int rm57_dcan_sock_initialize(int n)
{
  struct rm57can_s *priv = rm57can_instance(n);
  struct rm57can_sock_s *sock;
  int ret;

  if (priv == NULL || n < 0 || n >= RM57_NCAN)
    {
      return -ENODEV;
    }

  sock = &g_rm57can_sock[n];
  memset(sock, 0, sizeof(*sock));

  sock->priv          = priv;
  sock->dev.d_ifup    = rm57can_sk_ifup;
  sock->dev.d_ifdown  = rm57can_sk_ifdown;
  sock->dev.d_txavail = rm57can_sk_txavail;
#ifdef CONFIG_NETDEV_IOCTL
  sock->dev.d_ioctl   = rm57can_sk_ioctl;
#endif
  sock->dev.d_private = sock;
  sock->dev.d_buf     = (uint8_t *)&sock->txframe;

  ret = rm57can_init(priv, &g_rm57can_sk_cbs, sock);
  if (ret < 0)
    {
      return ret;
    }

  /* Register the device, interface down */

  rm57can_sk_ifdown(&sock->dev);

  ret = netdev_register(&sock->dev, NET_LL_CAN);
  if (ret < 0)
    {
      nerr("ERROR: DCAN%d netdev_register failed: %d\n", n + 1, ret);
      return ret;
    }

  ninfo("DCAN%d registered as SocketCAN interface\n", n + 1);
  return OK;
}

/* arm_netinitialize() (which self-registers this frontend's instances
 * when !CONFIG_NETDEV_LATEINIT) now lives in rm57_netinitialize.c, which
 * is also the EMAC driver's registration point - a single file must own
 * this symbol.  See that file for the registration list.
 */

#endif /* CONFIG_RM57_DCAN && CONFIG_NET_CAN */
