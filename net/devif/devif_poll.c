/****************************************************************************
 * net/devif/devif_poll.c
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
#ifdef CONFIG_NET

#include <debug.h>

#include <nuttx/clock.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/net.h>
#include <nuttx/net/arp.h>

#include "devif/devif.h"
#include "arp/arp.h"
#include "can/can.h"
#include "tcp/tcp.h"
#include "udp/udp.h"
#include "pkt/pkt.h"
#include "bluetooth/bluetooth.h"
#include "ieee802154/ieee802154.h"
#include "icmp/icmp.h"
#include "igmp/igmp.h"
#include "icmpv6/icmpv6.h"
#include "mld/mld.h"
#include "ipforward/ipforward.h"
#include "sixlowpan/sixlowpan.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum devif_packet_type
{
  DEVIF_PKT = 0,
  DEVIF_ICMP,
  DEVIF_IGMP,
  DEVIF_TCP,
  DEVIF_UDP,
  DEVIF_ICMP6
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: devif_packet_conversion
 *
 * Description:
 *   Generic output conversion hook.  Only needed for IEEE802.15.4 (and
 *   other non-standard packet radios) for now but this is a point where
 *   support for other conversions may be provided.
 *
 *   TCP output comes through three different mechansims.  Either from:
 *
 *   1. TCP socket output.  For the case of TCP output to a radio,
 *      the TCP output is caught in the socket send()/sendto() logic and
 *      redirected to 6LoWPAN logic.
 *   2. TCP output from the TCP state machine.  That will occur
 *      during TCP packet processing by the TCP state meachine.
 *   3. TCP output resulting from TX or timer polling
 *
 *   Cases 2 is handled here.  Logic here detected if (1) an attempt
 *   to return with d_len > 0 and (2) that the device is a radio
 *   network driver. Under those conditions, 6LoWPAN logic will be called
 *   to create the radio frames.
 *
 *   All outgoing ICMPv6 messages come through one of two mechanisms:
 *
 *   1. The output from internal ICMPv6 message passing.  These outgoing
 *      messages will use device polling and will be handled here.
 *   2. ICMPv6 output resulting from TX or timer polling.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_6LOWPAN
static void devif_packet_conversion(FAR struct net_driver_s *dev,
                                    enum devif_packet_type pkttype)
{
  if (dev->d_len > 0)
    {
      /* Check if this is a device served by 6LoWPAN */

      if (dev->d_lltype == NET_LL_IEEE802154 ||
          dev->d_lltype == NET_LL_PKTRADIO)
        {
          FAR struct ipv6_hdr_s *ipv6 = (FAR struct ipv6_hdr_s *)dev->d_buf;

#ifdef CONFIG_NET_IPv4
          if ((ipv6->vtc & IP_VERSION_MASK) != IPv6_VERSION)
            {
               nerr("ERROR: IPv6 version error: %02x...  Packet dropped\n",
                    ipv6->vtc);
            }
          else
#endif
#ifdef CONFIG_NET_TCP
          if (pkttype == DEVIF_TCP)
            {
              /* This packet came from a response to TCP polling and is
               * directed to an radio driver using 6LoWPAN.  Verify that the
               * outgoing packet is IPv6 with TCP protocol.
               */

              if (ipv6->proto == IP_PROTO_TCP)
                {
                  /* Let 6LoWPAN convert IPv6 TCP output into radio frames. */

                  sixlowpan_tcp_send(dev, dev, ipv6);
                }
              else
                {
                  nerr("ERROR: TCP protocol error: %u...  Packet dropped\n",
                       ipv6->proto);
                }
            }
          else
#endif
#ifdef CONFIG_NET_ICMPv6
          if (pkttype == DEVIF_ICMP6)
            {
              /* This packet came from a response to ICMPv6 polling and is
               * directed to a radio using 6LoWPAN.  Verify that the outgoing
               * packet is IPv6 with TCP protocol.
               */

              if (ipv6->proto == IP_PROTO_ICMP6)
                {
                  /* Let 6LoWPAN convert IPv6 ICMPv6 output into radio
                   * frames.
                   */

                  sixlowpan_icmpv6_send(dev, dev, ipv6);
                }
              else
                {
                  nerr("ERROR: ICMPv6 protocol error: %u..."
                        "  Packet dropped\n",
                       ipv6->proto);
                }
            }
          else
#endif
            {
              nerr("ERROR: Unhandled packet dropped."
                    "  pkttype=%u protocol=%u\n",
                    pkttype, ipv6->proto);
            }

          UNUSED(ipv6);
          dev->d_len = 0;
        }
    }
}
#else
#  define devif_packet_conversion(dev,pkttype)
#endif /* CONFIG_NET_6LOWPAN */

/****************************************************************************
 * Name: devif_poll_pkt_connections
 *
 * Description:
 *   Poll all packet connections for available packets to send.
 *
 * Assumptions:
 *   This function is called from the MAC device driver with the network
 *   locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_PKT
static int devif_poll_pkt_connections(FAR struct net_driver_s *dev,
                                      devif_poll_callback_t callback)
{
  FAR struct pkt_conn_s *pkt_conn = NULL;
  int bstop = 0;

  /* Traverse all of the allocated packet connections and perform the poll
   * action.
   */

  while (!bstop && (pkt_conn = pkt_nextconn(pkt_conn)))
    {
      /* Perform the packet TX poll */

      pkt_poll(dev, pkt_conn);

      /* Perform any necessary conversions on outgoing packets */

      devif_packet_conversion(dev, DEVIF_PKT);

      /* Call back into the driver */

      bstop = callback(dev);
    }

  return bstop;
}
#endif /* CONFIG_NET_PKT */

/****************************************************************************
 * Name: devif_poll_can_connections
 *
 * Description:
 *   Poll all packet connections for available packets to send.
 *
 * Assumptions:
 *   This function is called from the MAC device driver with the network
 *   locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_CAN
static int devif_poll_can_connections(FAR struct net_driver_s *dev,
                                      devif_poll_callback_t callback)
{
  FAR struct can_conn_s *can_conn = NULL;
  int bstop = 0;

  /* Traverse all of the allocated packet connections and
   * perform the poll action
   */

  while (!bstop && (can_conn = can_nextconn(can_conn)))
    {
      /* Skip connections that are bound to other polling devices */

      if (dev == can_conn->dev)
        {
          /* Perform the packet TX poll */

          can_poll(dev, can_conn);

          /* Call back into the driver */

          bstop = callback(dev);
        }
    }

  return bstop;
}
#endif /* CONFIG_NET_PKT */

/****************************************************************************
 * Name: devif_poll_bluetooth_connections
 *
 * Description:
 *   Poll all packet connections for available packets to send.
 *
 * Assumptions:
 *   This function is called from the MAC device driver with the network
 *   locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_BLUETOOTH
static int devif_poll_bluetooth_connections(FAR struct net_driver_s *dev,
                                            devif_poll_callback_t callback)
{
  FAR struct bluetooth_conn_s *bluetooth_conn = NULL;
  int bstop = 0;

  /* Traverse all of the allocated packet connections and perform the poll
   * action.
   */

  while (!bstop && (bluetooth_conn = bluetooth_conn_next(bluetooth_conn)))
    {
      /* Perform the packet TX poll */

      bluetooth_poll(dev, bluetooth_conn);

      /* Call back into the driver */

      bstop = callback(dev);
    }

  return bstop;
}
#endif /* CONFIG_NET_BLUETOOTH */

/****************************************************************************
 * Name: devif_poll_ieee802154_connections
 *
 * Description:
 *   Poll all packet connections for available packets to send.
 *
 * Assumptions:
 *   This function is called from the MAC device driver with the network
 *   locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IEEE802154
static int devif_poll_ieee802154_connections(FAR struct net_driver_s *dev,
                                             devif_poll_callback_t callback)
{
  FAR struct ieee802154_conn_s *ieee802154_conn = NULL;
  int bstop = 0;

  /* Traverse all of the allocated packet connections and perform the poll
   * action.
   */

  while (!bstop && (ieee802154_conn = ieee802154_conn_next(ieee802154_conn)))
    {
      /* Perform the packet TX poll */

      ieee802154_poll(dev, ieee802154_conn);

      /* Call back into the driver */

      bstop = callback(dev);
    }

  return bstop;
}
#endif /* CONFIG_NET_IEEE802154 */

/****************************************************************************
 * Name: devif_poll_icmp
 *
 * Description:
 *   Poll all of the connections waiting to send an ICMP ECHO request
 *
 ****************************************************************************/

#if defined(CONFIG_NET_ICMP) && defined(CONFIG_NET_ICMP_SOCKET)
static inline int devif_poll_icmp(FAR struct net_driver_s *dev,
                                  devif_poll_callback_t callback)
{
  FAR struct icmp_conn_s *conn = NULL;
  int bstop = 0;

  /* Traverse all of the allocated ICMP connections and perform the poll
   * action.
   */

  while (!bstop && (conn = icmp_nextconn(conn)) != NULL)
    {
      /* Skip ICMP connections that are bound to other polling devices */

      if (dev == conn->dev)
        {
          /* Perform the ICMP poll */

          icmp_poll(dev, conn);

          /* Perform any necessary conversions on outgoing packets */

          devif_packet_conversion(dev, DEVIF_ICMP);

          /* Call back into the driver */

          bstop = devif_poll_out(dev, callback);
        }
    }

  return bstop;
}
#endif /* CONFIG_NET_ICMP && CONFIG_NET_ICMP_SOCKET */

/****************************************************************************
 * Name: devif_poll_icmpv6
 *
 * Description:
 *   Poll all of the connections waiting to send an ICMPv6 ECHO request
 *
 ****************************************************************************/

#if defined(CONFIG_NET_ICMPv6_SOCKET) || defined(CONFIG_NET_ICMPv6_NEIGHBOR)
static inline int devif_poll_icmpv6(FAR struct net_driver_s *dev,
                                    devif_poll_callback_t callback)
{
  FAR struct icmpv6_conn_s *conn = NULL;
  int bstop = 0;

  /* Traverse all of the allocated ICMPv6 connections and perform the poll
   * action.
   */

  do
    {
      /* Skip ICMPv6 connections that are bound to other polling devices */

      if (conn == NULL || dev == conn->dev)
        {
          /* Perform the ICMPV6 poll
           * Note: conn equal NULL in the first iteration means poll dev's
           * callback list since icmpv6_autoconfig and icmpv6_neighbor still
           * append it's callback into this list.
           */

          icmpv6_poll(dev, conn);

          /* Perform any necessary conversions on outgoing packets */

          devif_packet_conversion(dev, DEVIF_ICMP6);

          /* Call back into the driver */

          bstop = devif_poll_out(dev, callback);
        }
    }
  while (!bstop && (conn = icmpv6_nextconn(conn)) != NULL);

  return bstop;
}
#endif /* CONFIG_NET_ICMPv6_SOCKET || CONFIG_NET_ICMPv6_NEIGHBOR*/

/****************************************************************************
 * Name: devif_poll_forward
 *
 * Description:
 *   Poll the device event to see if any task is waiting to forward a packet.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPFORWARD
static inline int devif_poll_forward(FAR struct net_driver_s *dev,
                                     devif_poll_callback_t callback)
{
  /* Perform the forwarding poll */

  ipfwd_poll(dev);

  /* NOTE: that 6LoWPAN packet conversions are handled differently for
   * forwarded packets.  That is because we don't know what the packet
   * type is at this point; not within peeking into the device's d_buf.
   */

  /* Call back into the driver */

  return devif_poll_out(dev, callback);
}
#endif /* CONFIG_NET_ICMPv6_SOCKET || CONFIG_NET_ICMPv6_NEIGHBOR*/

/****************************************************************************
 * Name: devif_poll_igmp
 *
 * Description:
 *   Poll all IGMP connections for available packets to send.
 *
 * Assumptions:
 *   This function is called from the MAC device driver with the network
 *   locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IGMP
static inline int devif_poll_igmp(FAR struct net_driver_s *dev,
                                  devif_poll_callback_t callback)
{
  /* Perform the IGMP TX poll */

  igmp_poll(dev);

  /* Perform any necessary conversions on outgoing packets */

  devif_packet_conversion(dev, DEVIF_IGMP);

  /* Call back into the driver */

  return devif_poll_out(dev, callback);
}
#endif /* CONFIG_NET_IGMP */

/****************************************************************************
 * Name: devif_poll_mld
 *
 * Description:
 *   Poll all MLD connections for available packets to send.
 *
 * Assumptions:
 *   This function is called from the MAC device driver with the network
 *   locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_MLD
static inline int devif_poll_mld(FAR struct net_driver_s *dev,
                                 devif_poll_callback_t callback)
{
  /* Perform the MLD TX poll */

  mld_poll(dev);

  /* Perform any necessary conversions on outgoing ICMPv6 packets */

  devif_packet_conversion(dev, DEVIF_ICMP6);

  /* Call back into the driver */

  return devif_poll_out(dev, callback);
}
#endif /* CONFIG_NET_MLD */

/****************************************************************************
 * Name: devif_poll_udp_connections
 *
 * Description:
 *   Poll all UDP connections for available packets to send.
 *
 * Assumptions:
 *   This function is called from the MAC device driver with the network
 *   locked.
 *
 ****************************************************************************/

#ifdef NET_UDP_HAVE_STACK
static int devif_poll_udp_connections(FAR struct net_driver_s *dev,
                                      devif_poll_callback_t callback)
{
  FAR struct udp_conn_s *conn = NULL;
  int bstop = 0;

  /* Traverse all of the allocated UDP connections and perform the poll
   * action.
   */

  while (!bstop && (conn = udp_nextconn(conn)))
    {
#ifdef CONFIG_NET_UDP_WRITE_BUFFERS
      /* Skip UDP connections that are bound to other polling devices */

      if (dev == conn->dev)
#endif
        {
          /* Perform the UDP TX poll */

          udp_poll(dev, conn);

          /* Perform any necessary conversions on outgoing packets */

          devif_packet_conversion(dev, DEVIF_UDP);

          /* Call back into the driver */

          bstop = devif_poll_out(dev, callback);
        }
    }

  return bstop;
}
#endif /* NET_UDP_HAVE_STACK */

/****************************************************************************
 * Name: devif_poll_tcp_connections
 *
 * Description:
 *   Poll all TCP connections for available packets to send.
 *
 * Assumptions:
 *   This function is called from the MAC device driver with the network
 *   locked.
 *
 ****************************************************************************/

#ifdef NET_TCP_HAVE_STACK
static inline int devif_poll_tcp_connections(FAR struct net_driver_s *dev,
                                             devif_poll_callback_t callback)
{
  FAR struct tcp_conn_s *conn  = NULL;
  int bstop = 0;

  /* Traverse all of the active TCP connections and perform the poll action */

  while (!bstop && (conn = tcp_nextconn(conn)))
    {
      /* Skip TCP connections that are bound to other polling devices */

      if (dev == conn->dev)
        {
          /* Perform the TCP TX poll */

          tcp_poll(dev, conn);

          /* Perform any necessary conversions on outgoing packets */

          devif_packet_conversion(dev, DEVIF_TCP);

          /* Call back into the driver */

          bstop = devif_poll_out(dev, callback);
        }
    }

  return bstop;
}
#else
# define devif_poll_tcp_connections(dev, callback) (0)
#endif

/****************************************************************************
 * Name: devif_poll_connections
 *
 * Description:
 *   This function will traverse each active network connection structure and
 *   will perform network polling operations. devif_poll() may be called
 *   asynchronously with the network driver can accept another outgoing
 *   packet.
 *
 *   This function will call the provided callback function for every active
 *   connection. Polling will continue until all connections have been polled
 *   or until the user-supplied function returns a non-zero value (which it
 *   should do only if it cannot accept further write data).
 *
 *   When the callback function is called, there may be an outbound packet
 *   waiting for service in the device packet buffer, and if so the d_len
 *   field is set to a value larger than zero. The device driver should then
 *   send out the packet.
 *
 * Assumptions:
 *   This function is called from the MAC device driver with the network
 *   locked.
 *
 ****************************************************************************/

static int devif_poll_connections(FAR struct net_driver_s *dev,
                                  devif_poll_callback_t callback)
{
  int bstop = false;

  /* Traverse all of the active packet connections and perform the poll
   * action.
   */

#ifdef CONFIG_NET_ARP_SEND
  /* Check for pending ARP requests */

  bstop = arp_poll(dev, callback);
  if (!bstop)
#endif
#ifdef CONFIG_NET_PKT
    {
      /* Check for pending packet socket transfer */

      bstop = devif_poll_pkt_connections(dev, callback);
    }

  if (!bstop)
#endif
#ifdef CONFIG_NET_CAN
    {
      /* Check for pending CAN socket transfer */

      bstop = devif_poll_can_connections(dev, callback);
    }

  if (!bstop)
#endif
#ifdef CONFIG_NET_BLUETOOTH
    {
      /* Check for pending PF_BLUETOOTH socket transfer */

      bstop = devif_poll_bluetooth_connections(dev, callback);
    }

  if (!bstop)
#endif
#ifdef CONFIG_NET_IEEE802154
    {
      /* Check for pending PF_IEEE802154 socket transfer */

      bstop = devif_poll_ieee802154_connections(dev, callback);
    }

  if (!bstop)
#endif
#ifdef CONFIG_NET_IGMP
    {
      /* Check for pending IGMP messages */

      bstop = devif_poll_igmp(dev, callback);
    }

  if (!bstop)
#endif
#ifdef CONFIG_NET_MLD
    {
      /* Check for pending MLD messages */

      bstop = devif_poll_mld(dev, callback);
    }

  if (!bstop)
#endif
#ifdef NET_TCP_HAVE_STACK
    {
      /* Traverse all of the active TCP connections and perform the poll
       * action.
       */

      bstop = devif_poll_tcp_connections(dev, callback);
    }

  if (!bstop)
#endif
#ifdef NET_UDP_HAVE_STACK
    {
      /* Traverse all of the allocated UDP connections and perform
       * the poll action
       */

      bstop = devif_poll_udp_connections(dev, callback);
    }

  if (!bstop)
#endif
#if defined(CONFIG_NET_ICMP) && defined(CONFIG_NET_ICMP_SOCKET)
    {
      /* Traverse all of the tasks waiting to send an ICMP ECHO request. */

      bstop = devif_poll_icmp(dev, callback);
    }

  if (!bstop)
#endif
#if defined(CONFIG_NET_ICMPv6_SOCKET) || defined(CONFIG_NET_ICMPv6_NEIGHBOR)
    {
      /* Traverse all of the tasks waiting to send an ICMPv6 ECHO request. */

      bstop = devif_poll_icmpv6(dev, callback);
    }

  if (!bstop)
#endif
#ifdef CONFIG_NET_IPFORWARD
    {
      /* Traverse all of the tasks waiting to forward a packet to this
       * device.
       */

      bstop = devif_poll_forward(dev, callback);
    }

  if (!bstop)
#endif
    {
      /* Nothing more to do */
    }

  return bstop;
}

/****************************************************************************
 * Name: devif_iob_poll
 *
 * Description:
 *   This function will traverse each active network connection structure and
 *   will perform network polling operations. devif_poll() may be called
 *   asynchronously with the network driver can accept another outgoing
 *   packet.
 *
 *   This function will call the provided callback function for every active
 *   connection. Polling will continue until all connections have been polled
 *   or until the user-supplied function returns a non-zero value (which it
 *   should do only if it cannot accept further write data).
 *
 *   When the callback function is called, there may be an outbound packet
 *   waiting for service in the device packet buffer, and if so the d_len
 *   field is set to a value larger than zero. The device driver should then
 *   send out the packet.
 *
 *   This is the iob buffer version of devif_input(),
 *   this function will support send/receive iob vectors directly between
 *   the driver and l3/l4 stack to avoid unnecessary memory copies,
 *   especially on hardware that supports Scatter/gather, which can
 *   greatly improve performance
 *   this function will uses d_iob as packets input which used by some
 *   NICs such as celluler net driver.
 *
 *   If NIC hardware support Scatter/gather transfer
 *
 *                  tcp_poll()/udp_poll()/pkt_poll()/...(l3/l4)
 *                             /           \
 *                            /             \
 *  devif_poll_[l3|l4]_connections()  devif_iob_send() (nocopy:udp/icmp/...)
 *             /                                \      (copy:tcp)
 *            /                                  \
 *    devif_iob_poll("NIC"_txpoll)             callback() // "NIC"_txpoll
 *
 *
 * Assumptions:
 *   This function is called from the MAC device driver with the network
 *   locked.
 *
 ****************************************************************************/

static int devif_iob_poll(FAR struct net_driver_s *dev,
                          devif_poll_callback_t callback)
{
  int bstop;

  /* Device polling, prepare iob buffer */

  if (netdev_iob_prepare(dev, false, 0) != OK)
    {
      return true;
    }

  /* Perform all connections poll */

  bstop = devif_poll_connections(dev, callback);

  /* Device polling completed, release iob */

  netdev_iob_release(dev);

  return bstop;
}

/****************************************************************************
 * Name: devif_poll_callback
 *
 * Description:
 *   This function will help us to gather multiple iob memory slices into a
 *   linear device buffer. if devices with small memory, this function will
 *   trigger a memory copy if net device start transmit the iob slices to
 *   flat buffer
 *
 ****************************************************************************/

static int devif_poll_callback(FAR struct net_driver_s *dev)
{
  if (dev->d_len > 0)
    {
      return true;
    }

  return false;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: devif_poll
 *
 * Description:
 *   This function will traverse each active network connection structure and
 *   will perform network polling operations. devif_poll() may be called
 *   asynchronously with the network driver can accept another outgoing
 *   packet.
 *
 *   This function will call the provided callback function for every active
 *   connection. Polling will continue until all connections have been polled
 *   or until the user-supplied function returns a non-zero value (which it
 *   should do only if it cannot accept further write data).
 *
 *   When the callback function is called, there may be an outbound packet
 *   waiting for service in the device packet buffer, and if so the d_len
 *   field is set to a value larger than zero. The device driver should then
 *   send out the packet.
 *
 *   Compatible with all old flat buffer NICs
 *
 *                 tcp_poll()/udp_poll()/pkt_poll()/...(l3|l4)
 *                            /              \
 *                           /                \
 * devif_poll_[l3|l4]_connections()     devif_iob_send() (nocopy:udp/icmp/..)
 *            /                                   \      (copy:tcp)
 *           /                                     \
 *   devif_iob_poll(devif_poll_callback())  devif_poll_callback()
 *        /                                           \
 *       /                                             \
 *  devif_poll("NIC"_txpoll)                     "NIC"_send()(dev->d_buf)
 *
 *
 * Assumptions:
 *   This function is called from the MAC device driver with the network
 *   locked.
 *
 ****************************************************************************/

int devif_poll(FAR struct net_driver_s *dev, devif_poll_callback_t callback)
{
  uint16_t llhdrlen;
  FAR uint8_t *buf;
  int bstop;

  if (dev->d_buf == NULL)
    {
      return devif_iob_poll(dev, callback);
    }

  buf = dev->d_buf;

  llhdrlen = NET_LL_HDRLEN(dev);

  do
    {
      /* Device polling, prepare iob buffer */

      if (netdev_iob_prepare(dev, false, 0) != OK)
        {
          bstop = true;
          break;
        }

      /* Perform all connections poll */

      bstop = devif_poll_connections(dev, devif_poll_callback);
      if (dev->d_len > 0)
        {
          /* Copy iob to flat buffer */

          iob_copyout(buf + llhdrlen,
                      dev->d_iob, dev->d_len, 0);

          /* Copy l2 header (arp out) */

          memcpy(buf, dev->d_iob->io_data +
                 (CONFIG_NET_LL_GUARDSIZE - llhdrlen), llhdrlen);

          /* Restore flat buffer pointer */

          dev->d_buf = buf;

          /* Call the real device callback */

          bstop = callback(dev);

          /* Flat buffer changed by NIC ? */

          if (dev->d_buf != buf)
            {
              buf = dev->d_buf;

              if (buf == NULL)
                {
                  break;
                }
            }
        }
    }
  while (bstop);

  /* Device polling completed, release iob */

  netdev_iob_release(dev);

  /* Restore the flat buffer */

  dev->d_buf = buf;

  return bstop;
}

/****************************************************************************
 * Name: devif_out
 *
 * Description:
 *   Generic interface to build L2 headers
 *
 * Assumptions:
 *   This function is called from the MAC device driver with the network
 *   locked.
 *
 ****************************************************************************/

void devif_out(FAR struct net_driver_s *dev)
{
  if (dev->d_len == 0)
    {
      return;
    }

  switch (dev->d_lltype)
    {
#ifdef CONFIG_NET_ETHERNET
      case NET_LL_ETHERNET:
      case NET_LL_IEEE80211:
#  ifdef CONFIG_NET_IPv4
#    ifdef CONFIG_NET_IPv6
        if (IFF_IS_IPv4(dev->d_flags))
#    endif /* CONFIG_NET_IPv6 */
          {
            arp_out(dev);
          }
#  endif /* CONFIG_NET_IPv4 */
#  ifdef CONFIG_NET_IPv6
#    ifdef CONFIG_NET_IPv4
        else
#    endif /* CONFIG_NET_IPv4 */
          {
            neighbor_out(dev);
          }
#  endif /* CONFIG_NET_IPv6 */

        break;
#endif /* CONFIG_NET_ETHERNET */
      default:
        break;
    }
}

/****************************************************************************
 * Name: devif_poll_out
 *
 * Description:
 *   Generic callback before device output to build L2 headers before sending
 *
 * Assumptions:
 *   This function is called from the MAC device driver with the network
 *   locked.
 *
 ****************************************************************************/

int devif_poll_out(FAR struct net_driver_s *dev,
                   devif_poll_callback_t callback)
{
  int bstop;

  if (dev->d_len == 0)
    {
      return 0;
    }

  devif_out(dev);

  bstop = devif_loopback(dev);
  if (bstop)
    {
      return bstop;
    }

  if (callback)
    {
      return callback(dev);
    }

  return 0;
}

#endif /* CONFIG_NET */
