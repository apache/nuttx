/****************************************************************************
 * drivers/net/netdev_upperhalf.c
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

#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/mm/iob.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev_lowerhalf.h>
#include <nuttx/net/pkt.h>
#include <nuttx/semaphore.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NETDEV_TX_CONTINUE 1 /* Return value for devif_poll */

#define NETDEV_THREAD_NAME_FMT "netdev-%s"

#ifdef CONFIG_NETDEV_HPWORK_THREAD
#  define NETDEV_WORK HPWORK
#else
#  define NETDEV_WORK LPWORK
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the state of the upper half driver */

struct netdev_upperhalf_s
{
  FAR struct netdev_lowerhalf_s *lower;

  /* Deferring poll work to work queue or thread */

#ifdef CONFIG_NETDEV_WORK_THREAD
  pid_t tid;
  sem_t sem;
  sem_t sem_exit;
#else
  struct work_s work;
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netpkt_get
 *
 * Description:
 *   Wraps the d_iob of dev to a netpkt.
 *
 * Assumptions:
 *   Called with the network locked.
 *
 ****************************************************************************/

static FAR netpkt_t *netpkt_get(FAR struct net_driver_s *dev,
                                enum netpkt_type_e type)
{
  FAR struct netdev_upperhalf_s *upper;
  FAR netpkt_t *pkt;

  DEBUGASSERT(dev && dev->d_iob);

  upper = (FAR struct netdev_upperhalf_s *)dev->d_private;
  pkt = dev->d_iob;

  netdev_iob_clear(dev);

  /* Do not limit quota here (simply relay iob instead of dropping), most
   * cases will be limited by netdev_upper_can_tx and seldom reaches here.
   */

  if (upper->lower->quota[type]-- <= 0)
    {
      nwarn("WARNING: Allowing temperarily exceeding quota of %s.\n",
            dev->d_ifname);
    }

  return pkt;
}

/****************************************************************************
 * Name: netpkt_put
 *
 * Description:
 *   Relay IOB to dev.
 *
 * Assumptions:
 *   Called with the network locked.
 *
 ****************************************************************************/

static void netpkt_put(FAR struct net_driver_s *dev, FAR netpkt_t *pkt,
                       enum netpkt_type_e type)
{
  FAR struct netdev_upperhalf_s *upper = dev->d_private;

  DEBUGASSERT(dev && pkt);

  /* TODO: Using netdev_iob_release instead of netdev_iob_replace now,
   *       because netdev_iob_replace sets d_len = L3_LEN and d_buf,
   *       but we don't want these changes.
   */

  upper->lower->quota[type]++;
  netdev_iob_release(dev);
  dev->d_iob = pkt;
  dev->d_len = netpkt_getdatalen(upper->lower, pkt);
}

/****************************************************************************
 * Name: netdev_upper_alloc
 *
 * Description:
 *   Get the upper half of lower half structure, create one if not present.
 *
 ****************************************************************************/

static FAR struct netdev_upperhalf_s *
netdev_upper_alloc(FAR struct netdev_lowerhalf_s *dev)
{
  /* Allocate the upper-half data structure */

  FAR struct netdev_upperhalf_s *upper;

  DEBUGASSERT(dev != NULL && dev->netdev.d_private == NULL);

  upper = kmm_zalloc(sizeof(struct netdev_upperhalf_s));
  if (upper == NULL)
    {
      nerr("ERROR: Allocation failed\n");
      return NULL;
    }

  upper->lower = dev;
  dev->netdev.d_private = upper;

  return upper;
}

/****************************************************************************
 * Name: netdev_upper_can_tx
 *
 * Description:
 *   Check if we allow tx on this device.
 *
 * Assumptions:
 *   Called with the network locked.
 *
 ****************************************************************************/

static inline bool netdev_upper_can_tx(FAR struct netdev_upperhalf_s *upper)
{
  return upper->lower->quota[NETPKT_TX] > 0;
}

/****************************************************************************
 * Name: netdev_upper_txpoll
 *
 * Description:
 *   The transmitter is available, check if the network has any outgoing
 *   packets ready to send.  This is a callback from devif_poll().
 *   devif_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete
 *   2. When the preceding TX packet send times out and the interface is
 *      reset
 *   3. During normal TX polling
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   Negated errno value - Error number that occurs.
 *   OK                  - Driver can send more, continue the poll.
 *
 * Assumptions:
 *   Called with the network locked.
 *
 ****************************************************************************/

static int netdev_upper_txpoll(FAR struct net_driver_s *dev)
{
  FAR struct netdev_upperhalf_s *upper = dev->d_private;
  FAR struct netdev_lowerhalf_s *lower = upper->lower;
  FAR netpkt_t                  *pkt;
  int                            ret;

  DEBUGASSERT(dev->d_len > 0);

  NETDEV_TXPACKETS(dev);

#ifdef CONFIG_NET_PKT
  /* When packet sockets are enabled, feed the tx frame into it */

  pkt_input(dev);
#endif

  pkt = netpkt_get(dev, NETPKT_TX);
  ret = lower->ops->transmit(lower, pkt);

  if (ret != OK)
    {
      /* Stop polling on any error
       * REVISIT: maybe store the pkt in upper half and retry later?
       */

      NETDEV_TXERRORS(dev);
      netpkt_put(dev, pkt, NETPKT_TX);
      return ret;
    }

  return NETDEV_TX_CONTINUE;
}

/****************************************************************************
 * Name: netdev_upper_txavail_work
 *
 * Description:
 *   Perform an out-of-cycle tx poll on the worker thread.
 *
 * Input Parameters:
 *   upper - Reference to the upper half driver structure
 *
 * Assumptions:
 *   Called with the network locked.
 *
 ****************************************************************************/

static void netdev_upper_txavail_work(FAR struct netdev_upperhalf_s *upper)
{
  FAR struct net_driver_s *dev = &upper->lower->netdev;

  /* Ignore the notification if the interface is not yet up */

  if (IFF_IS_UP(dev->d_flags))
    {
      DEBUGASSERT(dev->d_buf == NULL); /* Make sure: IOB only. */
      while (netdev_upper_can_tx(upper) &&
             devif_poll(dev, netdev_upper_txpoll) == NETDEV_TX_CONTINUE);
    }
}

/****************************************************************************
 * Function: netdev_upper_rxpoll_work
 *
 * Description:
 *   Try to receive packets from device and pass packets into IP
 *   stack and send packets which is from IP stack if necessary.
 *
 * Input Parameters:
 *   upper - Reference to the upper half driver structure
 *
 * Assumptions:
 *   Called with the network locked.
 *
 ****************************************************************************/

static void netdev_upper_rxpoll_work(FAR struct netdev_upperhalf_s *upper)
{
  FAR struct netdev_lowerhalf_s *lower = upper->lower;
  FAR struct net_driver_s       *dev   = &lower->netdev;
  FAR struct eth_hdr_s          *eth_hdr;
  FAR netpkt_t                  *pkt;

  /* Loop while receive() successfully retrieves valid Ethernet frames. */

  while ((pkt = lower->ops->receive(lower)) != NULL)
    {
      NETDEV_RXPACKETS(dev);

      if (!IFF_IS_UP(dev->d_flags))
        {
          /* Interface down, drop frame */

          NETDEV_RXDROPPED(dev);
          netpkt_free(lower, pkt, NETPKT_RX);
          continue;
        }

      netpkt_put(dev, pkt, NETPKT_RX);

#ifdef CONFIG_NET_PKT
      /* When packet sockets are enabled, feed the frame into the tap */

      pkt_input(dev);
#endif

      /* TODO: Support other ll types. */

      DEBUGASSERT(dev->d_lltype == NET_LL_ETHERNET ||
                  dev->d_lltype == NET_LL_IEEE80211);

      eth_hdr = (FAR struct eth_hdr_s *)NETLLBUF;

      /* Check if this is an 802.1Q VLAN tagged packet */

      if (eth_hdr->type == HTONS(TPID_8021QVLAN))
        {
          /* Need to remove the 4 octet VLAN Tag, by moving src and dest
           * addresses 4 octets to the right, and then read the actual
           * ethertype. The VLAN ID and priority fields are currently
           * ignored.
           */

          memmove((FAR uint8_t *)eth_hdr + 4, eth_hdr,
                  offsetof(struct eth_hdr_s, type));
          dev->d_iob  = iob_trimhead(dev->d_iob, 4);
          dev->d_len -= 4;

          eth_hdr = (FAR struct eth_hdr_s *)NETLLBUF;
        }

      /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv4
      if (eth_hdr->type == HTONS(ETHTYPE_IP))
        {
          ninfo("IPv4 frame\n");
          NETDEV_RXIPV4(dev);

          /* Receive an IPv4 packet from the network device */

          ipv4_input(dev);
        }
      else
#endif
#ifdef CONFIG_NET_IPv6
      if (eth_hdr->type == HTONS(ETHTYPE_IP6))
        {
          ninfo("IPv6 frame\n");
          NETDEV_RXIPV6(dev);

          /* Give the IPv6 packet to the network layer */

          ipv6_input(dev);
        }
      else
#endif
#ifdef CONFIG_NET_ARP
      if (eth_hdr->type == HTONS(ETHTYPE_ARP))
        {
          ninfo("ARP frame\n");
          NETDEV_RXARP(dev);

          /* Handle ARP packet */

          arp_input(dev);
        }
      else
#endif
        {
          ninfo("INFO: Dropped, Unknown type: %04x\n", eth_hdr->type);
          NETDEV_RXDROPPED(dev);
          dev->d_len = 0;
        }

      /* If the above function invocation resulted in data
       * that should be sent out on the network,
       * the field d_len will set to a value > 0.
       */

      if (dev->d_len > 0)
        {
          /* And send the packet */

          netdev_upper_txpoll(dev);
        }
    }
}

/****************************************************************************
 * Name: netdev_upper_txavail_work
 *
 * Description:
 *   Perform an out-of-cycle poll on a dedicated thread or the worker thread.
 *
 * Input Parameters:
 *   arg - Reference to the upper half driver structure (cast to void *)
 *
 ****************************************************************************/

static void netdev_upper_work(FAR void *arg)
{
  FAR struct netdev_upperhalf_s *upper = arg;

  /* RX may release quota and driver buffer, so do RX first. */

  net_lock();
  netdev_upper_rxpoll_work(upper);
  netdev_upper_txavail_work(upper);
  net_unlock();
}

/****************************************************************************
 * Name: netdev_upper_loop
 *
 * Description:
 *   The loop for dedicated thread.
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_WORK_THREAD
static int netdev_upper_loop(int argc, FAR char *argv[])
{
  FAR struct netdev_upperhalf_s *upper =
    (FAR struct netdev_upperhalf_s *)((uintptr_t)strtoul(argv[1], NULL, 16));

  while (nxsem_wait(&upper->sem) == OK && upper->tid != INVALID_PROCESS_ID)
    {
      netdev_upper_work(upper);
    }

  nwarn("WARNING: Netdev work thread quitting.");
  nxsem_post(&upper->sem_exit);
  return 0;
}
#endif

/****************************************************************************
 * Name: netdev_upper_queue_work
 *
 * Description:
 *   Called when there is any work to do.
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 ****************************************************************************/

static inline void netdev_upper_queue_work(FAR struct net_driver_s *dev)
{
  FAR struct netdev_upperhalf_s *upper = dev->d_private;

#ifdef CONFIG_NETDEV_WORK_THREAD
  int semcount;
  if (nxsem_get_value(&upper->sem, &semcount) == OK && semcount <= 0)
    {
      nxsem_post(&upper->sem);
    }
#else
  if (work_available(&upper->work))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(NETDEV_WORK, &upper->work, netdev_upper_work, upper, 0);
    }
#endif
}

/****************************************************************************
 * Name: netdev_upper_txavail
 *
 * Description:
 *   Called when new TX data is available.
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 ****************************************************************************/

static int netdev_upper_txavail(FAR struct net_driver_s *dev)
{
  netdev_upper_queue_work(dev);
  return OK;
}

/****************************************************************************
 * Name: netdev_upper_ifup/ifdown/addmac/rmmac/ioctl
 *
 * Description:
 *   Called by net stack and relayed to lower half driver.
 *
 ****************************************************************************/

static int netdev_upper_ifup(FAR struct net_driver_s *dev)
{
  FAR struct netdev_upperhalf_s *upper = dev->d_private;

#ifdef CONFIG_NETDEV_WORK_THREAD
  /* Try to bring up a dedicated thread for work. */

  if (upper->tid <= 0)
    {
      FAR char *argv[2];
      char      arg1[32];
      char      name[32];

      snprintf(arg1, sizeof(arg1), "%p", upper);
      snprintf(name, sizeof(name), NETDEV_THREAD_NAME_FMT, dev->d_ifname);
      argv[0] = arg1;
      argv[1] = NULL;

      upper->tid = kthread_create(name, CONFIG_NETDEV_WORK_THREAD_PRIORITY,
                                  CONFIG_DEFAULT_TASK_STACKSIZE,
                                  netdev_upper_loop, argv);
      if (upper->tid < 0)
        {
          return upper->tid;
        }
    }
#endif

  if (upper->lower->ops->ifup)
    {
      return upper->lower->ops->ifup(upper->lower);
    }

  return -ENOSYS;
}

static int netdev_upper_ifdown(FAR struct net_driver_s *dev)
{
  FAR struct netdev_upperhalf_s *upper = dev->d_private;

#ifndef CONFIG_NETDEV_WORK_THREAD
  work_cancel(NETDEV_WORK, &upper->work);
#endif

  if (upper->lower->ops->ifdown)
    {
      return upper->lower->ops->ifdown(upper->lower);
    }

  return -ENOSYS;
}

#ifdef CONFIG_NET_MCASTGROUP
static int netdev_upper_addmac(FAR struct net_driver_s *dev,
                               FAR const uint8_t *mac)
{
  FAR struct netdev_upperhalf_s *upper = dev->d_private;

  if (upper->lower->ops->addmac)
    {
      return upper->lower->ops->addmac(upper->lower, mac);
    }

  return -ENOSYS;
}
#endif

#ifdef CONFIG_NET_MCASTGROUP
static int netdev_upper_rmmac(FAR struct net_driver_s *dev,
                              FAR const uint8_t *mac)
{
  FAR struct netdev_upperhalf_s *upper = dev->d_private;

  if (upper->lower->ops->rmmac)
    {
      return upper->lower->ops->rmmac(upper->lower, mac);
    }

  return -ENOSYS;
}
#endif

#ifdef CONFIG_NETDEV_IOCTL
static int netdev_upper_ioctl(FAR struct net_driver_s *dev, int cmd,
                              unsigned long arg)
{
  FAR struct netdev_upperhalf_s *upper = dev->d_private;

  if (upper->lower->ops->ioctl)
    {
      return upper->lower->ops->ioctl(upper->lower, cmd, arg);
    }

  return -ENOTTY;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netdev_lower_register
 *
 * Description:
 *   Register a network device driver and assign a name to it so that it can
 *   be found in subsequent network ioctl operations on the device.
 *
 * Input Parameters:
 *   dev    - The lower half device driver structure to be registered.
 *   lltype - Link level protocol used by the driver (Ethernet, SLIP, TUN,
 *            ...
 *
 * Returned Value:
 *   0:Success; negated errno on failure.
 *
 ****************************************************************************/

int netdev_lower_register(FAR struct netdev_lowerhalf_s *dev,
                          enum net_lltype_e lltype)
{
  FAR struct netdev_upperhalf_s *upper;
  int ret;

  if (dev == NULL || dev->ops == NULL ||
      dev->ops->transmit == NULL || dev->ops->receive == NULL)
    {
      return -EINVAL;
    }

  if ((upper = netdev_upper_alloc(dev)) == NULL)
    {
      return -ENOMEM;
    }

  dev->netdev.d_ifup    = netdev_upper_ifup;
  dev->netdev.d_ifdown  = netdev_upper_ifdown;
  dev->netdev.d_txavail = netdev_upper_txavail;
#ifdef CONFIG_NET_MCASTGROUP
  dev->netdev.d_addmac  = netdev_upper_addmac;
  dev->netdev.d_rmmac   = netdev_upper_rmmac;
#endif
#ifdef CONFIG_NETDEV_IOCTL
  dev->netdev.d_ioctl   = netdev_upper_ioctl;
#endif
  dev->netdev.d_private = upper;

  ret = netdev_register(&dev->netdev, lltype);
  if (ret < 0)
    {
      kmm_free(upper);
      dev->netdev.d_private = NULL;
    }

#ifdef CONFIG_NETDEV_WORK_THREAD
  nxsem_init(&upper->sem, 0, 0);
  nxsem_init(&upper->sem_exit, 0, 0);
#endif

  return ret;
}

/****************************************************************************
 * Name: netdev_lower_unregister
 *
 * Description:
 *   Unregister a network device driver.
 *
 * Input Parameters:
 *   dev - The lower half device driver structure to un-register
 *
 * Returned Value:
 *   0:Success; negated errno on failure
 *
 ****************************************************************************/

int netdev_lower_unregister(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct netdev_upperhalf_s *upper;
  int ret;

  if (dev == NULL || dev->netdev.d_private == NULL)
    {
      return -EINVAL;
    }

  upper = (FAR struct netdev_upperhalf_s *)dev->netdev.d_private;
  ret = netdev_unregister(&dev->netdev);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_NETDEV_WORK_THREAD
  if (upper->tid > 0)
    {
      /* Try to tear down the dedicated thread for work. */

      upper->tid = INVALID_PROCESS_ID;
      nxsem_post(&upper->sem);
      nxsem_wait(&upper->sem_exit);
    }

  nxsem_destroy(&upper->sem);
  nxsem_destroy(&upper->sem_exit);
#endif

  kmm_free(upper);
  dev->netdev.d_private = NULL;

  return OK;
}

/****************************************************************************
 * Name: netdev_lower_carrier_on
 *
 * Description:
 *   Notifies the networking layer about an available carrier.
 *   (e.g. a cable was plugged in)
 *
 * Input Parameters:
 *   dev - The lower half device driver structure
 *
 * Returned Value:
 *   0:Success; negated errno on failure
 *
 ****************************************************************************/

int netdev_lower_carrier_on(FAR struct netdev_lowerhalf_s *dev)
{
  return netdev_carrier_on(&dev->netdev);
}

/****************************************************************************
 * Name: netdev_lower_carrier_off
 *
 * Description:
 *   Notifies the networking layer about an disappeared carrier.
 *   (e.g. a cable was unplugged)
 *
 * Input Parameters:
 *   dev - The lower half device driver structure
 *
 * Returned Value:
 *   0:Success; negated errno on failure
 *
 ****************************************************************************/

int netdev_lower_carrier_off(FAR struct netdev_lowerhalf_s *dev)
{
  return netdev_carrier_off(&dev->netdev);
}

/****************************************************************************
 * Name: netdev_lower_rxready
 *
 * Description:
 *   Notifies the networking layer about an RX packet is ready to read.
 *
 * Input Parameters:
 *   dev - The lower half device driver structure
 *
 ****************************************************************************/

void netdev_lower_rxready(FAR struct netdev_lowerhalf_s *dev)
{
  netdev_upper_queue_work(&dev->netdev);
}

/****************************************************************************
 * Name: netdev_lower_txdone
 *
 * Description:
 *   Notifies the networking layer about a TX packet is sent.
 *
 * Input Parameters:
 *   dev - The lower half device driver structure
 *
 ****************************************************************************/

void netdev_lower_txdone(FAR struct netdev_lowerhalf_s *dev)
{
  NETDEV_TXDONE(&dev->netdev);
  netdev_upper_queue_work(&dev->netdev);
}

/****************************************************************************
 * Name: netpkt_alloc
 *
 * Description:
 *   Allocate a netpkt structure.
 *
 * Input Parameters:
 *   dev  - The lower half device driver structure
 *   type - Whether used for TX or RX
 *
 * Returned Value:
 *   Pointer to the packet, NULL on failure
 *
 ****************************************************************************/

FAR netpkt_t *netpkt_alloc(FAR struct netdev_lowerhalf_s *dev,
                           enum netpkt_type_e type)
{
  FAR netpkt_t *pkt;

  if (dev->quota[type] <= 0)
    {
      return NULL;
    }

  pkt = iob_tryalloc(false);
  if (pkt == NULL)
    {
      return NULL;
    }

  net_lock(); /* REVISIT: Do we have better solution? */
  dev->quota[type]--;
  net_unlock();

  iob_reserve(pkt, CONFIG_NET_LL_GUARDSIZE);
  return pkt;
}

/****************************************************************************
 * Name: netpkt_free
 *
 * Description:
 *   Release a netpkt structure.
 *
 * Input Parameters:
 *   dev  - The lower half device driver structure
 *   pkt  - The packet to release
 *   type - Whether used for TX or RX
 *
 ****************************************************************************/

void netpkt_free(FAR struct netdev_lowerhalf_s *dev, FAR netpkt_t *pkt,
                 enum netpkt_type_e type)
{
  net_lock(); /* REVISIT: Do we have better solution? */
  dev->quota[type]++;
  net_unlock();

  iob_free_chain(pkt);
}

/****************************************************************************
 * Name: netpkt_copyin
 *
 * Description:
 *   Copy 'len' bytes of data from a buffer into the netpkt, starting at
 *   'offset' of netpkt.
 *
 * Input Parameters:
 *   dev    - The lower half device driver structure
 *   pkt    - The net packet
 *   src    - The source buffer
 *   len    - How many bytes to copy
 *   offset - The offset of netpkt to put the data
 *
 * Returned Value:
 *   0:Success; negated errno on failure
 *
 ****************************************************************************/

int netpkt_copyin(FAR struct netdev_lowerhalf_s *dev, FAR netpkt_t *pkt,
                  FAR const uint8_t *src, unsigned int len, int offset)
{
  return iob_trycopyin(pkt, src, len,
                       offset - NET_LL_HDRLEN(&dev->netdev), false);
}

/****************************************************************************
 * Name: netpkt_copyout
 *
 * Description:
 *   Copy 'len' bytes of data from netpkt into a buffer, starting at
 *   'offset' of netpkt.
 *
 * Input Parameters:
 *   dev    - The lower half device driver structure
 *   dest   - The destination buffer
 *   pkt    - The net packet
 *   len    - How many bytes to copy
 *   offset - The offset of netpkt to get the data
 *
 * Returned Value:
 *   0:Success; negated errno on failure
 *
 ****************************************************************************/

int netpkt_copyout(FAR struct netdev_lowerhalf_s *dev, FAR uint8_t *dest,
                   FAR const netpkt_t *pkt, unsigned int len, int offset)
{
  return iob_copyout(dest, pkt, len, offset - NET_LL_HDRLEN(&dev->netdev));
}

/****************************************************************************
 * Name: netpkt_getdata/getbase
 *
 * Description:
 *   Get the pointer of data/base in a netpkt, used when NETPKT_BUFLEN is
 *   big enough to fit a full packet in.
 *
 * Input Parameters:
 *   dev    - The lower half device driver structure
 *   pkt    - The net packet
 *
 * Returned Value:
 *   Pointer data/base, NULL on failure.
 *
 ****************************************************************************/

FAR uint8_t *netpkt_getdata(FAR struct netdev_lowerhalf_s *dev,
                             FAR netpkt_t *pkt)
{
  return IOB_DATA(pkt) - NET_LL_HDRLEN(&dev->netdev);
}

FAR uint8_t *netpkt_getbase(FAR netpkt_t *pkt)
{
  return pkt->io_data;
}

/****************************************************************************
 * Name: netpkt_setdatalen
 *
 * Description:
 *   Set the length of data in netpkt, used when data is written into
 *   netpkt by data/base pointer, no need to set this length after
 *   copyin.
 *
 * Input Parameters:
 *   dev    - The lower half device driver structure
 *   pkt    - The net packet
 *   len    - The length of data in netpkt
 *
 ****************************************************************************/

void netpkt_setdatalen(FAR struct netdev_lowerhalf_s *dev,
                       FAR netpkt_t *pkt, unsigned int len)
{
  iob_update_pktlen(pkt, len - NET_LL_HDRLEN(&dev->netdev));
}

/****************************************************************************
 * Name: netpkt_getdatalen
 *
 * Description:
 *   Get the length of data in netpkt.
 *
 * Input Parameters:
 *   dev    - The lower half device driver structure
 *   pkt    - The net packet
 *
 * Returned Value:
 *   The length of data in netpkt.
 *
 ****************************************************************************/

unsigned int netpkt_getdatalen(FAR struct netdev_lowerhalf_s *dev,
                               FAR netpkt_t *pkt)
{
  return pkt->io_pktlen + NET_LL_HDRLEN(&dev->netdev);
}

/****************************************************************************
 * Name: netpkt_reset_reserved
 *
 * Description:
 *   Reset the reserved length (the starting point of data) of netpkt
 *
 * Input Parameters:
 *   dev    - The lower half device driver structure
 *   pkt    - The net packet
 *   len    - The reserved length
 *
 ****************************************************************************/

void netpkt_reset_reserved(FAR struct netdev_lowerhalf_s *dev,
                           FAR netpkt_t *pkt, unsigned int len)
{
  iob_reserve(pkt, len + NET_LL_HDRLEN(&dev->netdev));
}

/****************************************************************************
 * Name: netpkt_is_fragmented
 *
 * Description:
 *   Returns whether the netpkt is fragmented into different blocks.
 *     In other words, NETPKT_BUFLEN < reserved + total data
 *
 * Input Parameters:
 *   pkt - The net packet
 *
 ****************************************************************************/

bool netpkt_is_fragmented(FAR netpkt_t *pkt)
{
  return pkt->io_flink != NULL;
}
