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
#include <nuttx/spinlock.h>
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
  char                  cpuname[RPMSG_NAME_SIZE];
  netpkt_queue_t        rxqueue; /* RX packet queue */
  spinlock_t            lock;    /* Spinlock for protecting rxqueue */
  FAR void             *priv;    /* Private data for upper layer */
  net_rpmsg_drv_cb_t    cb;      /* IFUP/DOWN Callback function */
  sem_t                 wait;    /* Wait sem, used for preventing any
                                  * operation until the connection
                                  * between two cpu established.
                                  */
  struct rpmsg_endpoint ept;

  /* This holds the information visible to the NuttX network */

  struct netdev_lowerhalf_s dev; /* Interface understood by the network */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* RPMSG related functions */

/* Request handler functions */

static int net_rpmsg_drv_default_handler(FAR struct rpmsg_endpoint *ept,
                                         FAR void *data, size_t len,
                                         uint32_t src, FAR void *priv);
static int net_rpmsg_drv_ifup_handler(FAR struct rpmsg_endpoint *ept,
                                      FAR void *data, size_t len,
                                      uint32_t src, FAR void *priv);
static int net_rpmsg_drv_ifdown_handler(FAR struct rpmsg_endpoint *ept,
                                        FAR void *data, size_t len,
                                        uint32_t src, FAR void *priv);
static int net_rpmsg_drv_sockioctl_handler(FAR struct rpmsg_endpoint *ept,
                                           FAR void *data, size_t len,
                                           uint32_t src, FAR void *priv);
static int net_rpmsg_drv_transfer_handler(FAR struct rpmsg_endpoint *ept,
                                          FAR void *data, size_t len,
                                          uint32_t src, FAR void *priv);

/* Response handler functions */

static int net_rpmsg_drv_default_response(FAR struct rpmsg_endpoint *ept,
                                          FAR void *data, size_t len,
                                          uint32_t src, FAR void *priv);

/* RPMSG device related functions */

static void net_rpmsg_drv_device_created(FAR struct rpmsg_device *rdev,
                                         FAR void *priv);
static void net_rpmsg_drv_device_destroy(FAR struct rpmsg_device *rdev,
                                         FAR void *priv);
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
  [NET_RPMSG_IFUP]      = net_rpmsg_drv_ifup_handler,
  [NET_RPMSG_IFDOWN]    = net_rpmsg_drv_ifdown_handler,
  [NET_RPMSG_ADDMCAST]  = net_rpmsg_drv_default_handler,
  [NET_RPMSG_RMMCAST]   = net_rpmsg_drv_default_handler,
  [NET_RPMSG_DEVIOCTL]  = net_rpmsg_drv_default_handler,
  [NET_RPMSG_SOCKIOCTL] = net_rpmsg_drv_sockioctl_handler,
  [NET_RPMSG_TRANSFER]  = net_rpmsg_drv_transfer_handler,
};

static const rpmsg_ept_cb g_net_rpmsg_drv_response[] =
{
  [NET_RPMSG_IFUP]      = net_rpmsg_drv_default_response,
  [NET_RPMSG_IFDOWN]    = net_rpmsg_drv_default_response,
  [NET_RPMSG_ADDMCAST]  = net_rpmsg_drv_default_response,
  [NET_RPMSG_RMMCAST]   = net_rpmsg_drv_default_response,
  [NET_RPMSG_DEVIOCTL]  = net_rpmsg_drv_default_response,
  [NET_RPMSG_SOCKIOCTL] = net_rpmsg_drv_default_response,
  [NET_RPMSG_TRANSFER]  = net_rpmsg_drv_default_response,
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
  FAR struct net_rpmsg_drv_s *drv =
                              container_of(dev, struct net_rpmsg_drv_s, dev);
  FAR struct net_rpmsg_transfer_s *transfer;
  unsigned int datalen = netpkt_getdatalen(dev, pkt);
  uint32_t len;
  int ret;

  transfer = rpmsg_get_tx_payload_buffer(&drv->ept, &len, true);
  if (transfer == NULL)
    {
      nwarn("WARNING: Failed to get buffer for xmit\n");
      return -ENOMEM;
    }

  if (len < sizeof(*transfer) + datalen)
    {
      nerr("ERROR: Buffer is too small for xmit\n");
      rpmsg_release_tx_buffer(&drv->ept, transfer);
      return -ENOMEM;
    }

  transfer->header.command = NET_RPMSG_TRANSFER;
  transfer->header.result  = 0;
  transfer->header.cookie  = 0;
  transfer->length         = datalen;
  netpkt_copyout(dev, (FAR uint8_t *)(transfer + 1), pkt, datalen, 0);

  len = sizeof(*transfer) + datalen;
  ret = rpmsg_send_nocopy(&drv->ept, transfer, len);
  if (ret < 0)
    {
      nerr("ERROR: Failed to send packet\n");
      rpmsg_release_tx_buffer(&drv->ept, transfer);
      return ret;
    }

  netpkt_free(dev, pkt, NETPKT_TX);
  netdev_lower_txdone(dev);
  return OK;
}

static FAR netpkt_t *
net_rpmsg_drv_receive(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct net_rpmsg_drv_s *drv =
                              container_of(dev, struct net_rpmsg_drv_s, dev);
  FAR netpkt_t *pkt;
  irqstate_t flags;

  flags = spin_lock_irqsave(&drv->lock);
  pkt = netpkt_remove_queue(&drv->rxqueue);
  spin_unlock_irqrestore(&drv->lock, flags);

  return pkt;
}

/* RPMSG related functions */

static void rpmsg_send_response(FAR struct rpmsg_endpoint *ept,
                                FAR struct net_rpmsg_header_s *header,
                                size_t len, int result)
{
  header->command |= NET_RPMSG_RESPONSE;
  header->result   = result;
  rpmsg_send(ept, header, len);
}

static int net_rpmsg_drv_default_handler(FAR struct rpmsg_endpoint *ept,
                                         FAR void *data, size_t len,
                                         uint32_t src, FAR void *priv)
{
  FAR struct net_rpmsg_header_s *header = data;

  if (header->cookie)
    {
      rpmsg_send_response(ept, header, sizeof(*header), -EOPNOTSUPP);
    }

  return 0;
}

static int net_rpmsg_drv_ifup_handler(FAR struct rpmsg_endpoint *ept,
                                        FAR void *data, size_t len,
                                        uint32_t src, FAR void *priv)
{
  FAR struct net_rpmsg_drv_s *drv = priv;
  FAR struct net_rpmsg_header_s *header = data;

  netdev_lower_carrier_on(&drv->dev);
  if (drv->cb != NULL)
    {
      drv->cb(&drv->dev, NET_RPMSG_EVENT_CARRIER_ON);
    }

  rpmsg_send_response(ept, header, sizeof(*header), 0);

  return 0;
}

static int net_rpmsg_drv_ifdown_handler(FAR struct rpmsg_endpoint *ept,
                                        FAR void *data, size_t len,
                                        uint32_t src, FAR void *priv)
{
  FAR struct net_rpmsg_drv_s *drv = priv;
  FAR struct net_rpmsg_header_s *header = data;

  netdev_lower_carrier_off(&drv->dev);
  if (drv->cb != NULL)
    {
      drv->cb(&drv->dev, NET_RPMSG_EVENT_CARRIER_OFF);
    }

  rpmsg_send_response(ept, header, sizeof(*header), 0);

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
      rpmsg_send_response(ept, &msg->header, sizeof(*msg) + msg->length,
                          msg->header.result);
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
                                          uint32_t src, FAR void *priv)
{
  FAR struct net_rpmsg_transfer_s *transfer = data;
  FAR struct net_rpmsg_drv_s *drv = priv;
  FAR struct netdev_lowerhalf_s *dev = &drv->dev;
  FAR netpkt_t *pkt;
  irqstate_t flags;
  int ret;

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

  flags = spin_lock_irqsave(&drv->lock);
  ret = netpkt_tryadd_queue(pkt, &drv->rxqueue);
  spin_unlock_irqrestore(&drv->lock, flags);
  if (ret < 0)
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

/****************************************************************************
 * Name: net_rpmsg_drv_default_response
 *
 * Description:
 *   This function is used to handle the response from the RPMSG device.
 *   It is used to copy the response to the cookie and post the semaphore.
 *
 ****************************************************************************/

static int net_rpmsg_drv_default_response(FAR struct rpmsg_endpoint *ept,
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

/****************************************************************************
 * Name: net_rpmsg_drv_ept_release
 ****************************************************************************/

static void net_rpmsg_drv_ept_release(FAR struct rpmsg_endpoint *ept)
{
  FAR struct net_rpmsg_drv_s *drv = ept->priv;

  netdev_lower_carrier_off(&drv->dev);
  rpmsg_wait(&drv->ept, &drv->wait);
}

/****************************************************************************
 * Name: net_rpmsg_drv_ns_bound
 *
 * Description:
 *   Rpmsg device end point service bound callback function , called when
 *   remote end point address is received.
 *
 * Parameters:
 *   ept  - The rpmsg-device end point
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void net_rpmsg_drv_ns_bound(FAR struct rpmsg_endpoint *ept)
{
  FAR struct net_rpmsg_drv_s *drv = ept->priv;

  rpmsg_post(&drv->ept, &drv->wait);
}

/****************************************************************************
 * Name: net_rpmsg_drv_device_created
 ****************************************************************************/

static void net_rpmsg_drv_device_created(FAR struct rpmsg_device *rdev,
                                         FAR void *priv)
{
  FAR struct net_rpmsg_drv_s *drv = priv;
  char eptname[RPMSG_NAME_SIZE];

  if (!strcmp(drv->cpuname, rpmsg_get_cpuname(rdev)))
    {
      drv->ept.priv = drv;
      snprintf(eptname, sizeof(eptname),
               NET_RPMSG_EPT_PREFIX "%s", drv->dev.netdev.d_ifname);

      rpmsg_create_ept(&drv->ept, rdev, eptname,
                       RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                       net_rpmsg_drv_ept_cb, NULL);
    }
}

/****************************************************************************
 * Name: net_rpmsg_drv_device_destroy
 ****************************************************************************/

static void net_rpmsg_drv_device_destroy(FAR struct rpmsg_device *rdev,
                                         FAR void *priv)
{
  FAR struct net_rpmsg_drv_s *drv = priv;

  if (!strcmp(drv->cpuname, rpmsg_get_cpuname(rdev)))
    {
      rpmsg_destroy_ept(&drv->ept);
    }
}

static int net_rpmsg_drv_ept_cb(FAR struct rpmsg_endpoint *ept, void *data,
                                size_t len, uint32_t src, FAR void *priv)
{
  FAR struct net_rpmsg_header_s *header = data;
  uint32_t cmd = NET_RPMSG_GET_COMMAND(header->command);

  if (cmd < nitems(g_net_rpmsg_drv_handler))
    {
      if (NET_RPMSG_IS_RESPONSE(header->command))
        {
          return g_net_rpmsg_drv_response[cmd](ept, data, len, src, priv);
        }
      else
        {
          return g_net_rpmsg_drv_handler[cmd](ept, data, len, src, priv);
        }
    }

  return -EINVAL;
}

static int net_rpmsg_drv_send_recv(FAR struct netdev_lowerhalf_s *dev,
                                   FAR void *header_, uint32_t command,
                                   int len)
{
  FAR struct net_rpmsg_drv_s *drv =
                              container_of(dev, struct net_rpmsg_drv_s, dev);
  FAR struct net_rpmsg_header_s *header = header_;
  FAR struct net_rpmsg_drv_cookie_s cookie;
  int sval = 0;
  int ret;

  nxsem_get_value(&drv->wait, &sval);
  if (sval <= 0)
    {
      rpmsg_wait(&drv->ept, &drv->wait);
      rpmsg_post(&drv->ept, &drv->wait);
    }

  nxsem_init(&cookie.sem, 0, 0);

  cookie.header   = header;
  header->command = command;
  header->result  = -ENXIO;
  header->cookie  = (uintptr_t)&cookie;

  ret = rpmsg_send(&drv->ept, header, len);
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
  FAR struct net_rpmsg_drv_s *drv =
                              container_of(dev, struct net_rpmsg_drv_s, dev);
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

  if (drv->cb != NULL)
    {
      drv->cb(dev, NET_RPMSG_EVENT_IF_UP);
    }

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
  FAR struct net_rpmsg_drv_s *drv =
                              container_of(dev, struct net_rpmsg_drv_s, dev);
  struct net_rpmsg_ifdown_s msg =
  {
  };

  int ret;

  ret = net_rpmsg_drv_send_recv(dev, &msg, NET_RPMSG_IFDOWN, sizeof(msg));
  if (ret < 0)
    {
      return ret;
    }

  if (drv->cb != NULL)
    {
      drv->cb(dev, NET_RPMSG_EVENT_IF_DOWN);
    }

  return ret;
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
 * Name: net_rpmsg_drv_alloc
 ****************************************************************************/

static FAR struct net_rpmsg_drv_s *
net_rpmsg_drv_alloc(FAR const char *devname, enum net_lltype_e lltype)
{
  FAR struct net_rpmsg_drv_s *drv = kmm_zalloc(sizeof(*drv));
  FAR struct netdev_lowerhalf_s *netdev;

  if (!drv)
    {
      return NULL;
    }

  netdev = &drv->dev;
  netdev->quota[NETPKT_RX] = CONFIG_IOB_NBUFFERS /
                             NET_RPMSG_DRV_MAX_NIOB / 4;
  netdev->quota[NETPKT_TX] = 1;
  netdev->ops = &g_net_rpmsg_drv_ops;

  drv->ept.priv = drv;
  drv->ept.release_cb = net_rpmsg_drv_ept_release;
  drv->ept.ns_bound_cb = net_rpmsg_drv_ns_bound;

  nxsem_init(&drv->wait, 0, 0);
  spin_lock_init(&drv->lock);

  /* Init a random MAC address, the caller can override it. */

  arc4random_buf(&netdev->netdev.d_mac.ether.ether_addr_octet,
                 sizeof(netdev->netdev.d_mac.ether.ether_addr_octet));

  strlcpy(netdev->netdev.d_ifname, devname, IFNAMSIZ);

  netdev_lower_register(netdev, lltype);

  return drv;
}

#ifdef CONFIG_NET_RPMSG_DRV_SERVER
/****************************************************************************
 * Name: net_rpmsg_drv_ns_match
 ****************************************************************************/

static bool net_rpmsg_drv_ns_match(FAR struct rpmsg_device *rdev,
                                   FAR void *priv, FAR const char *name,
                                   uint32_t dest)
{
  return !strncmp(name, NET_RPMSG_EPT_PREFIX, strlen(NET_RPMSG_EPT_PREFIX));
}

/****************************************************************************
 * Name: net_rpmsg_drv_ns_bind
 ****************************************************************************/

static void net_rpmsg_drv_ns_bind(FAR struct rpmsg_device *rdev,
                                  FAR void *priv_, FAR const char *name,
                                  uint32_t dest)
{
  FAR struct net_rpmsg_drv_s *drv;
  FAR struct net_driver_s *dev;
  const char *devname = name + strlen(NET_RPMSG_EPT_PREFIX);

  dev = netdev_findbyname(devname);
  if (dev)
    {
      drv = container_of(dev, struct net_rpmsg_drv_s, dev.netdev);
      drv->ept.priv = drv;
      drv->ept.release_cb = net_rpmsg_drv_ept_release;
      drv->ept.ns_bound_cb = net_rpmsg_drv_ns_bound;
    }
  else
    {
      drv = net_rpmsg_drv_alloc(devname, NET_LL_ETHERNET);
      if (!drv)
        {
          return;
        }
    }

  rpmsg_create_ept(&drv->ept, rdev, name, RPMSG_ADDR_ANY, dest,
                   net_rpmsg_drv_ept_cb, rpmsg_destroy_ept);
  rpmsg_post(&drv->ept, &drv->wait);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_rpmsg_drv_init
 *
 * Description:
 *   Allocate a new network device instance for the RPMSG network and
 *   register it with the network device manager.  This is the client side of
 *   the RPMSG driver.  The RPMSG driver is the server side of the driver.
 *
 * Parameters:
 *   cpuname    - Remote CPU name
 *   devname    - Local and remote network device name
 *   lltype     - Link layer type
 *
 * Returned Value:
 *   A pointer to the allocated network device instance.  NULL is returned on
 *   failure.
 *
 ****************************************************************************/

FAR struct netdev_lowerhalf_s *
net_rpmsg_drv_init(FAR const char *cpuname, FAR const char *devname,
                   enum net_lltype_e lltype)
{
  FAR struct net_rpmsg_drv_s *drv;
  FAR struct netdev_lowerhalf_s *dev;
  int ret;

  /* Allocate the interface structure */

  if (!devname || !cpuname ||
      !(drv = net_rpmsg_drv_alloc(devname, lltype)))
    {
      return NULL;
    }

  strlcpy(drv->cpuname, cpuname, RPMSG_NAME_SIZE);

  dev = &drv->dev;

  /* Register the device with the openamp */

  ret = rpmsg_register_callback(drv,
                                net_rpmsg_drv_device_created,
                                net_rpmsg_drv_device_destroy,
                                NULL,
                                NULL);

  if (ret < 0)
    {
      netdev_lower_unregister(dev);
      nxsem_destroy(&drv->wait);
      kmm_free(drv);
      return NULL;
    }

  return dev;
}

/****************************************************************************
 * Name: net_rpmsg_drv_priv
 ****************************************************************************/

FAR void *net_rpmsg_drv_priv(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct net_rpmsg_drv_s *drv =
                              container_of(dev, struct net_rpmsg_drv_s, dev);

  return drv->priv;
}

/****************************************************************************
 * Name: net_rpmsg_drv_set_callback
 ****************************************************************************/

void net_rpmsg_drv_set_callback(FAR struct netdev_lowerhalf_s *dev,
                                net_rpmsg_drv_cb_t cb, FAR void *priv)
{
  FAR struct net_rpmsg_drv_s *drv =
                              container_of(dev, struct net_rpmsg_drv_s, dev);

  drv->cb = cb;
  drv->priv = priv;
}

#ifdef CONFIG_NET_RPMSG_DRV_SERVER
/****************************************************************************
 * Name: net_rpmsg_drv_server_init
 ****************************************************************************/

int net_rpmsg_drv_server_init(void)
{
  return rpmsg_register_callback(NULL,
                                 NULL,
                                 NULL,
                                 net_rpmsg_drv_ns_match,
                                 net_rpmsg_drv_ns_bind);
}
#endif
