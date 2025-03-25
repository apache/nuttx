/****************************************************************************
 * drivers/net/rpmsgdrv.c
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
#include <debug.h>
#include <errno.h>
#include <stdio.h>

#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/semaphore.h>
#include <nuttx/wqueue.h>

#include <nuttx/net/dns.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/netdev_lowerhalf.h>
#include <nuttx/net/pkt.h>
#include <nuttx/net/rpmsg.h>
#include <nuttx/net/rpmsgdrv.h>
#include <nuttx/rpmsg/rpmsg.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NET_RPMSG_DRV_BUFSIZE (CONFIG_NET_ETH_PKTSIZE + CONFIG_NET_GUARDSIZE)
#define NET_RPMSG_DRV_MAX_PKT_SIZE \
    ((CONFIG_NET_LL_GUARDSIZE - ETH_HDRLEN) + NET_RPMSG_DRV_BUFSIZE)
#define NET_RPMSG_DRV_MAX_NIOB \
    ((NET_RPMSG_DRV_MAX_PKT_SIZE + CONFIG_IOB_BUFSIZE - 1) / \
     CONFIG_IOB_BUFSIZE)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct net_rpmsg_drv_cookie_s
{
  FAR struct net_rpmsg_header_s *header;
  sem_t                         sem;
};

/* net_rpmsg_drv_s encapsulates all state information for a single hardware
 * interface
 */

struct net_rpmsg_drv_s
{
  FAR const char        *cpuname;
  FAR const char        *devname;
  netpkt_queue_t        rxqueue; /* RX packet queue */
  struct rpmsg_endpoint ept;

  /* This holds the information visible to the NuttX network */

  struct netdev_lowerhalf_s dev; /* Interface understood by the network */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* RPMSG related functions */

static int net_rpmsg_drv_default_handler(FAR struct rpmsg_endpoint *ept,
                                         FAR void *data, size_t len,
                                         uint32_t src, FAR void *priv);
static int net_rpmsg_drv_sockioctl_handler(FAR struct rpmsg_endpoint *ept,
                                           FAR void *data, size_t len,
                                           uint32_t src, FAR void *priv);
static int net_rpmsg_drv_transfer_handler(FAR struct rpmsg_endpoint *ept,
                                          FAR void *data, size_t len,
                                          uint32_t src, FAR void *priv);

static void net_rpmsg_drv_device_created(FAR struct rpmsg_device *rdev,
                                         FAR void *priv_);
static void net_rpmsg_drv_device_destroy(FAR struct rpmsg_device *rdev,
                                         FAR void *priv_);
static int  net_rpmsg_drv_ept_cb(FAR struct rpmsg_endpoint *ept, void *data,
                                 size_t len, uint32_t src, FAR void *priv);

static int  net_rpmsg_drv_send_recv(struct netdev_lowerhalf_s *dev,
                                    void *header_, uint32_t command,
                                    int len);

/* NuttX callback functions */

static int  net_rpmsg_drv_ifup(FAR struct netdev_lowerhalf_s *dev);
static int  net_rpmsg_drv_ifdown(FAR struct netdev_lowerhalf_s *dev);

static int  net_rpmsg_drv_transmit(FAR struct netdev_lowerhalf_s *dev,
                                   FAR netpkt_t *pkt);
static FAR netpkt_t *
net_rpmsg_drv_receive(FAR struct netdev_lowerhalf_s *dev);

#ifdef CONFIG_NET_MCASTGROUP
static int  net_rpmsg_drv_addmac(FAR struct netdev_lowerhalf_s *dev,
                                 FAR const uint8_t *mac);
#ifdef CONFIG_NET_IGMP
static int  net_rpmsg_drv_rmmac(FAR struct netdev_lowerhalf_s *dev,
                                FAR const uint8_t *mac);
#endif
#endif
#ifdef CONFIG_NETDEV_IOCTL
static int  net_rpmsg_drv_ioctl(FAR struct netdev_lowerhalf_s *dev, int cmd,
                                unsigned long arg);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const rpmsg_ept_cb g_net_rpmsg_drv_handler[] =
{
  [NET_RPMSG_IFUP]      = net_rpmsg_drv_default_handler,
  [NET_RPMSG_IFDOWN]    = net_rpmsg_drv_default_handler,
  [NET_RPMSG_ADDMCAST]  = net_rpmsg_drv_default_handler,
  [NET_RPMSG_RMMCAST]   = net_rpmsg_drv_default_handler,
  [NET_RPMSG_DEVIOCTL]  = net_rpmsg_drv_default_handler,
  [NET_RPMSG_SOCKIOCTL] = net_rpmsg_drv_sockioctl_handler,
  [NET_RPMSG_TRANSFER]  = net_rpmsg_drv_transfer_handler,
};

static const struct netdev_ops_s g_net_rpmsg_drv_ops =
{
  .ifup     = net_rpmsg_drv_ifup,
  .ifdown   = net_rpmsg_drv_ifdown,
  .transmit = net_rpmsg_drv_transmit,
  .receive  = net_rpmsg_drv_receive,
#ifdef CONFIG_NET_MCASTGROUP
  .addmac   = net_rpmsg_drv_addmac,
  .rmmac    = net_rpmsg_drv_rmmac,
#endif
#ifdef CONFIG_NETDEV_IOCTL
  .ioctl    = net_rpmsg_drv_ioctl,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_rpmsg_drv_transmit
 *
 * Description:
 *   Start hardware transmission.
 *
 * Parameters:
 *   dev - Reference to the NuttX driver state structure
 *   pkt - The packet to be sent
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static int net_rpmsg_drv_transmit(FAR struct netdev_lowerhalf_s *dev,
                                  FAR netpkt_t *pkt)
{
  FAR struct net_rpmsg_drv_s *priv =
                              container_of(dev, struct net_rpmsg_drv_s, dev);
  FAR struct net_rpmsg_transfer_s *transfer;
  unsigned int datalen = netpkt_getdatalen(dev, pkt);
  uint32_t len;
  int ret;

  transfer = rpmsg_get_tx_payload_buffer(&priv->ept, &len, true);
  if (transfer == NULL)
    {
      nwarn("WARNING: Failed to get buffer for xmit\n");
      return -ENOMEM;
    }

  if (len < sizeof(*transfer) + datalen)
    {
      nerr("ERROR: Buffer is too small for xmit\n");
      rpmsg_release_tx_buffer(&priv->ept, transfer);
      return -ENOMEM;
    }

  transfer->header.command = NET_RPMSG_TRANSFER;
  transfer->header.result  = 0;
  transfer->header.cookie  = 0;
  transfer->length         = datalen;
  netpkt_copyout(dev, (FAR uint8_t *)(transfer + 1), pkt, datalen, 0);

  len = sizeof(*transfer) + datalen;
  ret = rpmsg_send_nocopy(&priv->ept, transfer, len);
  if (ret < 0)
    {
      nerr("ERROR: Failed to send packet\n");
      rpmsg_release_tx_buffer(&priv->ept, transfer);
      return ret;
    }

  netpkt_free(dev, pkt, NETPKT_TX);
  netdev_lower_txdone(dev);
  return OK;
}

static FAR netpkt_t *
net_rpmsg_drv_receive(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct net_rpmsg_drv_s *priv =
                              container_of(dev, struct net_rpmsg_drv_s, dev);
  return netpkt_remove_queue(&priv->rxqueue);
}

/* RPMSG related functions */

static int net_rpmsg_drv_default_handler(FAR struct rpmsg_endpoint *ept,
                                         FAR void *data, size_t len,
                                         uint32_t src, FAR void *priv)
{
  FAR struct net_rpmsg_header_s *header = data;
  FAR struct net_rpmsg_drv_cookie_s *cookie =
    (struct net_rpmsg_drv_cookie_s *)(uintptr_t)header->cookie;

  memcpy(cookie->header, header, len);
  nxsem_post(&cookie->sem);
  return 0;
}

static int net_rpmsg_drv_sockioctl_task(int argc, FAR char *argv[])
{
  FAR struct net_rpmsg_ioctl_s *msg;
  FAR struct rpmsg_endpoint *ept;
  struct socket sock;

  int domain   = NET_SOCK_FAMILY;
  int type     = NET_SOCK_TYPE;
  int protocol = NET_SOCK_PROTOCOL;

  /* Restore pointers from argv */

  ept = (FAR struct rpmsg_endpoint *)strtoul(argv[1], NULL, 16);
  msg = (FAR struct net_rpmsg_ioctl_s *)strtoul(argv[2], NULL, 16);

  /* We need a temporary sock for ioctl here */

  if (msg->code == SIOCIFAUTOCONF)
    {
      domain   = PF_INET6;
      type     = SOCK_DGRAM;
      protocol = IPPROTO_ICMP6;
    }

  msg->header.result = psock_socket(domain, type, protocol, &sock);
  if (msg->header.result >= 0)
    {
      msg->header.result = psock_ioctl(&sock, msg->code,
                                       (unsigned long)msg->arg);
      psock_close(&sock); /* Close the temporary sock */
    }

  /* Send the response only when cookie doesn't equal NULL */

  if (msg->header.cookie)
    {
      rpmsg_send(ept, msg, sizeof(*msg) + msg->length);
    }

  rpmsg_release_rx_buffer(ept, msg);
  return 0;
}

static int net_rpmsg_drv_sockioctl_handler(FAR struct rpmsg_endpoint *ept,
                                           FAR void *data, size_t len,
                                           uint32_t src, FAR void *priv)
{
  FAR char *argv[3];
  char arg1[16];
  char arg2[16];

  /* Save pointers into argv */

  snprintf(arg1, sizeof(arg1), "%p", ept);
  snprintf(arg2, sizeof(arg2), "%p", data);

  argv[0] = arg1;
  argv[1] = arg2;
  argv[2] = NULL;

  /* Move the action into a temp thread to avoid the deadlock */

  rpmsg_hold_rx_buffer(ept, data);
  kthread_create("rpmsg-net", CONFIG_NET_RPMSG_PRIORITY,
          CONFIG_NET_RPMSG_STACKSIZE, net_rpmsg_drv_sockioctl_task, argv);

  return 0;
}

/****************************************************************************
 * Name: net_rpmsg_drv_transfer_handler
 *
 * Description:
 *   An message was received indicating the availability of a new RX packet
 *
 * Parameters:
 *   ept - Reference to the endpoint which receive the message
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static int net_rpmsg_drv_transfer_handler(FAR struct rpmsg_endpoint *ept,
                                          FAR void *data, size_t len,
                                          uint32_t src, FAR void *priv_)
{
  FAR struct net_rpmsg_transfer_s *transfer = data;
  FAR struct net_rpmsg_drv_s *priv = priv_;
  FAR struct netdev_lowerhalf_s *dev = &priv->dev;
  FAR netpkt_t *pkt;

  if (transfer->length > len - sizeof(*transfer))
    {
      nerr("ERROR: net_rpmsg got invalid transfer length!");
      goto drop;
    }

  /* TODO: No-Copy, hold rx buffer */

  pkt = netpkt_alloc(dev, NETPKT_RX);
  if (pkt == NULL)
    {
      nerr("ERROR: Failed to allocate buffer!\n");
      goto drop;
    }

  if (netpkt_copyin(dev, pkt, (FAR uint8_t *)(transfer + 1),
                    transfer->length, 0) < 0)
    {
      nerr("ERROR: Failed to copy in data!\n");
      goto free;
    }

  if (netpkt_tryadd_queue(pkt, &priv->rxqueue) < 0)
    {
      nerr("ERROR: Failed to add pkt to queue!\n");
      goto free;
    }

  netdev_lower_rxready(dev);
  return 0;

free:
  netpkt_free(dev, pkt, NETPKT_RX);

drop:
  NETDEV_RXDROPPED(&dev->netdev);
  return 0;
}

static void net_rpmsg_drv_device_created(FAR struct rpmsg_device *rdev,
                                         FAR void *priv_)
{
  FAR struct net_rpmsg_drv_s *priv = priv_;
  char eptname[RPMSG_NAME_SIZE];

  if (!strcmp(priv->cpuname, rpmsg_get_cpuname(rdev)))
    {
      priv->ept.priv = priv;
      snprintf(eptname, sizeof(eptname),
               NET_RPMSG_EPT_NAME, priv->devname);

      rpmsg_create_ept(&priv->ept, rdev, eptname,
                       RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                       net_rpmsg_drv_ept_cb, NULL);
    }
}

static void net_rpmsg_drv_device_destroy(FAR struct rpmsg_device *rdev,
                                         FAR void *priv_)
{
  FAR struct net_rpmsg_drv_s *priv = priv_;

  if (!strcmp(priv->cpuname, rpmsg_get_cpuname(rdev)))
    {
      rpmsg_destroy_ept(&priv->ept);
    }
}

static int net_rpmsg_drv_ept_cb(FAR struct rpmsg_endpoint *ept, void *data,
                                size_t len, uint32_t src, FAR void *priv)
{
  FAR struct net_rpmsg_header_s *header = data;
  uint32_t command = header->command;

  if (command < nitems(g_net_rpmsg_drv_handler))
    {
      return g_net_rpmsg_drv_handler[command](ept, data, len, src, priv);
    }

  return -EINVAL;
}

static int net_rpmsg_drv_send_recv(FAR struct netdev_lowerhalf_s *dev,
                                   FAR void *header_, uint32_t command,
                                   int len)
{
  FAR struct net_rpmsg_drv_s *priv =
                              container_of(dev, struct net_rpmsg_drv_s, dev);
  FAR struct net_rpmsg_header_s *header = header_;
  FAR struct net_rpmsg_drv_cookie_s cookie;
  int ret;

  nxsem_init(&cookie.sem, 0, 0);

  cookie.header   = header;
  header->command = command;
  header->result  = -ENXIO;
  header->cookie  = (uintptr_t)&cookie;

  ret = rpmsg_send(&priv->ept, header, len);
  if (ret < 0)
    {
      goto out;
    }

  net_sem_timedwait2(&cookie.sem, false, UINT_MAX, &dev->netdev.d_lock,
                     NULL);
  ret = cookie.header->result;

out:
  nxsem_destroy(&cookie.sem);
  return ret;
}

/****************************************************************************
 * Name: net_rpmsg_drv_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the link interface when an IP address is
 *   provided
 *
 * Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static int net_rpmsg_drv_ifup(FAR struct netdev_lowerhalf_s *dev)
{
  struct net_rpmsg_ifup_s msg =
  {
  };

  int ret;

#ifdef CONFIG_NET_IPv4
  ninfo("Bringing up: %u.%u.%u.%u\n",
        ip4_addr1(dev->netdev.d_ipaddr), ip4_addr2(dev->netdev.d_ipaddr),
        ip4_addr3(dev->netdev.d_ipaddr), ip4_addr4(dev->netdev.d_ipaddr));
#endif
#ifdef CONFIG_NET_IPv6
  ninfo("Bringing up: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        dev->netdev.d_ipv6addr[0], dev->netdev.d_ipv6addr[1],
        dev->netdev.d_ipv6addr[2], dev->netdev.d_ipv6addr[3],
        dev->netdev.d_ipv6addr[4], dev->netdev.d_ipv6addr[5],
        dev->netdev.d_ipv6addr[6], dev->netdev.d_ipv6addr[7]);
#endif

  netdev_lock(&dev->netdev);

  /* Prepare the message */

  msg.lnkaddr.length = netdev_lladdrsize(&dev->netdev);
  memcpy(msg.lnkaddr.addr, &dev->netdev.d_mac, msg.lnkaddr.length);

#ifdef CONFIG_NET_IPv4
  net_ipv4addr_copy(msg.ipaddr, dev->netdev.d_ipaddr);
  net_ipv4addr_copy(msg.draddr, dev->netdev.d_draddr);
  net_ipv4addr_copy(msg.netmask, dev->netdev.d_netmask);
#endif

#ifdef CONFIG_NET_IPv6
  net_ipv6addr_copy(msg.ipv6addr, dev->netdev.d_ipv6addr);
  net_ipv6addr_copy(msg.ipv6draddr, dev->netdev.d_ipv6draddr);
  net_ipv6addr_copy(msg.ipv6netmask, dev->netdev.d_ipv6netmask);
#endif

  /* Send the message */

  ret = net_rpmsg_drv_send_recv(dev, &msg, NET_RPMSG_IFUP, sizeof(msg));
  if (ret < 0)
    {
      netdev_unlock(&dev->netdev);
      return ret;
    }

  /* Update net_driver_t field */

  memcpy(&dev->netdev.d_mac, msg.lnkaddr.addr, msg.lnkaddr.length);

#ifdef CONFIG_NET_IPv4
  net_ipv4addr_copy(dev->netdev.d_ipaddr, msg.ipaddr);
  net_ipv4addr_copy(dev->netdev.d_draddr, msg.draddr);
  net_ipv4addr_copy(dev->netdev.d_netmask, msg.netmask);
#endif

#ifdef CONFIG_NET_IPv6
  net_ipv6addr_copy(dev->netdev.d_ipv6addr, msg.ipv6addr);
  net_ipv6addr_copy(dev->netdev.d_ipv6draddr, msg.ipv6draddr);
  net_ipv6addr_copy(dev->netdev.d_ipv6netmask, msg.ipv6netmask);
#endif

  netdev_unlock(&dev->netdev);

#ifdef CONFIG_NETDB_DNSCLIENT
#  ifdef CONFIG_NET_IPv4
  if (!net_ipv4addr_cmp(msg.dnsaddr, INADDR_ANY))
    {
      struct sockaddr_in dnsaddr =
      {
      };

      dnsaddr.sin_family = AF_INET;
      dnsaddr.sin_port   = HTONS(DNS_DEFAULT_PORT);
      memcpy(&dnsaddr.sin_addr, &msg.dnsaddr, sizeof(msg.dnsaddr));

      dns_add_nameserver((FAR const struct sockaddr *)&dnsaddr,
                         sizeof(dnsaddr));
    }
#  endif

#  ifdef CONFIG_NET_IPv6
  if (!net_ipv6addr_cmp(msg.ipv6dnsaddr, &in6addr_any))
    {
      struct sockaddr_in6 dnsaddr =
      {
      };

      dnsaddr.sin6_family = AF_INET6;
      dnsaddr.sin6_port   = HTONS(DNS_DEFAULT_PORT);
      memcpy(&dnsaddr.sin6_addr, msg.ipv6dnsaddr, sizeof(msg.ipv6dnsaddr));

      dns_add_nameserver((FAR const struct sockaddr *)&dnsaddr,
                         sizeof(dnsaddr));
    }
#  endif
#endif

  return OK;
}

/****************************************************************************
 * Name: net_rpmsg_drv_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static int net_rpmsg_drv_ifdown(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct net_rpmsg_ifdown_s msg;

  /* Put the EMAC in its reset, non-operational state.  This should be
   * a known configuration that will guarantee the net_rpmsg_drv_ifup()
   * always successfully brings the interface back up.
   */

  return net_rpmsg_drv_send_recv(dev, &msg, NET_RPMSG_IFDOWN, sizeof(msg));
}

/****************************************************************************
 * Name: net_rpmsg_drv_addmac
 *
 * Description:
 *   NuttX Callback: Add the specified MAC address to the hardware multicast
 *   address filtering
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be added
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_MCASTGROUP
static int net_rpmsg_drv_addmac(FAR struct netdev_lowerhalf_s *dev,
                                FAR const uint8_t *mac)
{
  struct net_rpmsg_mcast_s msg;

  /* Add the MAC address to the hardware multicast routing table */

  msg.lnkaddr.length = netdev_lladdrsize(&dev->netdev);
  memcpy(msg.lnkaddr.addr, mac, msg.lnkaddr.length);
  return net_rpmsg_drv_send_recv(dev, &msg, NET_RPMSG_ADDMCAST, sizeof(msg));
}

/****************************************************************************
 * Name: net_rpmsg_drv_rmmac
 *
 * Description:
 *   NuttX Callback: Remove the specified MAC address from the hardware
 *   multicast address filtering
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be removed
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int net_rpmsg_drv_rmmac(FAR struct netdev_lowerhalf_s *dev,
                               FAR const uint8_t *mac)
{
  struct net_rpmsg_mcast_s msg;

  /* Remove the MAC address from the hardware multicast routing table */

  msg.lnkaddr.length = netdev_lladdrsize(&dev->netdev);
  memcpy(msg.lnkaddr.addr, mac, msg.lnkaddr.length);
  return net_rpmsg_drv_send_recv(dev, &msg, NET_RPMSG_RMMCAST, sizeof(msg));
}
#endif

/****************************************************************************
 * Name: net_rpmsg_drv_ioctl
 *
 * Description:
 *   Handle network IOCTL commands directed to this device.
 *
 * Parameters:
 *   dev - Reference to the NuttX driver state structure
 *   cmd - The IOCTL command
 *   arg - The argument for the IOCTL command
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_IOCTL
static int net_rpmsg_drv_ioctl(FAR struct netdev_lowerhalf_s *dev, int cmd,
                               unsigned long arg)
{
  ssize_t len;
  int ret;

  len = net_ioctl_arglen(PF_RPMSG, cmd);
  if (len >= 0)
    {
      FAR struct net_rpmsg_ioctl_s *msg;
      char buf[sizeof(*msg) + len];

      msg = (FAR struct net_rpmsg_ioctl_s *)buf;

      msg->code   = cmd;
      msg->length = len;
      memcpy(msg->arg, (FAR void *)arg, len);

      ret = net_rpmsg_drv_send_recv(dev, msg,
              NET_RPMSG_DEVIOCTL, sizeof(*msg) + len);
      if (ret >= 0)
        {
          memcpy((FAR void *)arg, msg->arg, len);
        }
    }
  else
    {
      ret = len;
    }

  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_rpmsg_drv_init
 *
 * Description:
 *   Initialize the net rpmsg driver
 *
 * Parameters:
 *   name - Specify the netdev name
 *   lltype - Identify the link type
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *   Called early in initialization before multi-tasking is initiated.
 *
 ****************************************************************************/

int net_rpmsg_drv_init(FAR const char *cpuname,
                       FAR const char *devname,
                       enum net_lltype_e lltype)
{
  FAR struct net_rpmsg_drv_s *priv;
  FAR struct netdev_lowerhalf_s *dev;
  int ret;

  /* Allocate the interface structure */

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  dev = &priv->dev;

  priv->cpuname = cpuname;
  priv->devname = devname;

  /* Initialize the driver structure */

  strlcpy(dev->netdev.d_ifname, devname, IFNAMSIZ);

  dev->quota[NETPKT_RX] = CONFIG_IOB_NBUFFERS / NET_RPMSG_DRV_MAX_NIOB / 4;
  dev->quota[NETPKT_TX] = 1;
  dev->ops = &g_net_rpmsg_drv_ops;

  /* Register the device with the openamp */

  ret = rpmsg_register_callback(priv,
                                net_rpmsg_drv_device_created,
                                net_rpmsg_drv_device_destroy,
                                NULL,
                                NULL);

  if (ret < 0)
    {
      kmm_free(priv);
      return ret;
    }

  /* Register the device with the OS so that socket IOCTLs can be performed */

  ret = netdev_lower_register(dev, lltype);
  if (ret < 0)
    {
      rpmsg_unregister_callback(dev,
                                net_rpmsg_drv_device_created,
                                net_rpmsg_drv_device_destroy,
                                NULL,
                                NULL);
      kmm_free(priv);
    }

  return ret;
}
