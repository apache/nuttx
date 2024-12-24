/****************************************************************************
 * drivers/net/netdev_upperhalf.c
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

#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/mm/iob.h>
#include <nuttx/net/can.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev_lowerhalf.h>
#include <nuttx/net/pkt.h>
#include <nuttx/semaphore.h>
#include <nuttx/spinlock.h>

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

#ifdef CONFIG_NETDEV_RSS
#  define NETDEV_THREAD_COUNT CONFIG_SMP_NCPUS
#else
#  define NETDEV_THREAD_COUNT 1
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
  pid_t tid[NETDEV_THREAD_COUNT];
  sem_t sem[NETDEV_THREAD_COUNT];
  sem_t sem_exit[NETDEV_THREAD_COUNT];
#else
  struct work_s work;
#endif

  /* TX queue for re-queueing replies */

#if CONFIG_IOB_NCHAINS > 0
  struct iob_queue_s txq;
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: quota_is_valid
 *
 * Description:
 *   Check if the quota of the lower half is not too big.
 *
 ****************************************************************************/

static bool quota_is_valid(FAR struct netdev_lowerhalf_s *lower)
{
  int total = 0;
  enum netpkt_type_e type;

  for (type = NETPKT_TX; type < NETPKT_TYPENUM; type++)
    {
      total += netdev_lower_quota_load(lower, type);
    }

  if (total > NETPKT_BUFNUM)
    {
      nerr("ERROR: Too big quota when registering device: %d\n", total);
      return false;
    }

  if (total > NETPKT_BUFNUM / 2)
    {
      nwarn("WARNING: The quota of the registering device may consume more "
            "than half of the network buffers, which may hurt performance. "
            "Please consider decreasing driver quota or increasing nIOB.\n");
    }

  return true;
}

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

  if (atomic_fetch_sub(&upper->lower->quota[type], 1) <= 0)
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

  atomic_fetch_add(&upper->lower->quota[type], 1);
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
  FAR struct netdev_lowerhalf_s *lower = upper->lower;
  int quota = netdev_lower_quota_load(lower, NETPKT_TX);

  if (quota <= 0 && lower->ops->reclaim)
    {
      lower->ops->reclaim(lower);
      quota = netdev_lower_quota_load(lower, NETPKT_TX);
    }

  return quota > 0;
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
 *   NETDEV_TX_CONTINUE  - Driver can send more, continue the poll.
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

  if (netpkt_getdatalen(lower, pkt) > NETDEV_PKTSIZE(dev))
    {
      nerr("ERROR: Packet too long to send!\n");
      ret = -EMSGSIZE;
    }
  else
    {
      ret = lower->ops->transmit(lower, pkt);
    }

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
 * Name: netdev_upper_tx
 *
 * Description:
 *   Do the actual transmission of packets, including pre-queued packets and
 *   packets from the network stack.
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   Negated errno value - Error number that occurs.
 *   NETDEV_TX_CONTINUE  - Driver can send more, continue the poll.
 *
 * Assumptions:
 *   Called with the network locked.
 *
 ****************************************************************************/

static int netdev_upper_tx(FAR struct net_driver_s *dev)
{
#if CONFIG_IOB_NCHAINS > 0
  FAR struct netdev_upperhalf_s *upper = dev->d_private;

  if (!IOB_QEMPTY(&upper->txq))
    {
      /* Put the packet back to the device */

      netdev_iob_replace(dev, iob_remove_queue(&upper->txq));
      return netdev_upper_txpoll(dev);
    }
#endif

  /* No more TX packets in queue, poll the net stack to get more packets */

  return devif_poll(dev, netdev_upper_txpoll);
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
             netdev_upper_tx(dev) == NETDEV_TX_CONTINUE);
    }
}

/****************************************************************************
 * Name: netdev_upper_queue_tx
 *
 * Description:
 *   Queue a TX packet to the upper half for sending later.
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Assumptions:
 *   Called with the network locked.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_LOOPBACK) || defined(CONFIG_NET_ETHERNET) || \
    defined(CONFIG_DRIVERS_IEEE80211) || defined(CONFIG_NET_MBIM)
static void netdev_upper_queue_tx(FAR struct net_driver_s *dev)
{
#if CONFIG_IOB_NCHAINS > 0
  FAR struct netdev_upperhalf_s *upper = dev->d_private;
  int ret;

  if ((ret = iob_tryadd_queue(dev->d_iob, &upper->txq)) >= 0)
    {
      netdev_iob_clear(dev);
    }
  else
    {
      nwarn("WARNING: Failed to queue TX packet, dropping: %d\n", ret);
    }
#else
  /* Fall back to send the packet directly if we don't have IOB queue. */

  netdev_upper_txpoll(dev);
#endif
}
#endif

/****************************************************************************
 * Name: eth_input
 *
 * Description:
 *   Handle L2 packet input.
 *
 * Input Parameters:
 *   dev - Reference to the NuttX network driver state structure
 *
 * Assumptions:
 *   Called with the network locked.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_LOOPBACK) || defined(CONFIG_NET_ETHERNET) || \
    defined(CONFIG_DRIVERS_IEEE80211)
static void eth_input(FAR struct net_driver_s *dev)
{
  FAR struct eth_hdr_s *eth_hdr = (FAR struct eth_hdr_s *)NETLLBUF;

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
      /* And queue the packet for sending later.
       * Note: RX is tried before TX, so we don't need to call txavail here.
       */

      netdev_upper_queue_tx(dev);
    }
}
#endif

/****************************************************************************
 * Name: ip_input
 *
 * Description:
 *   Handle L3 packet input.
 *
 * Input Parameters:
 *   dev - Reference to the NuttX network driver state structure
 *
 * Assumptions:
 *   Called with the network locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_MBIM
static void ip_input(FAR struct net_driver_s *dev)
{
  /* We only accept IP packets of the configured type */

#ifdef CONFIG_NET_IPv4
  if ((IPv4BUF->vhl & IP_VERSION_MASK) == IPv4_VERSION)
    {
      ninfo("IPv4 frame\n");
      NETDEV_RXIPV4(dev);

      /* Receive an IPv4 packet from the network device */

      ipv4_input(dev);
    }
  else
#endif
#ifdef CONFIG_NET_IPv6
  if ((IPv6BUF->vtc & IP_VERSION_MASK) == IPv6_VERSION)
    {
      ninfo("IPv6 frame\n");
      NETDEV_RXIPV6(dev);

      /* Give the IPv6 packet to the network layer */

      ipv6_input(dev);
    }
  else
#endif
    {
      ninfo("INFO: Dropped, Unknown type\n");
      NETDEV_RXDROPPED(dev);
      dev->d_len = 0;
    }

  /* If the above function invocation resulted in data
   * that should be sent out on the network,
   * the field d_len will set to a value > 0.
   */

  if (dev->d_len > 0)
    {
      /* And queue the packet for sending later.
       * Note: RX is tried before TX, so we don't need to call txavail here.
       */

      netdev_upper_queue_tx(dev);
    }
}
#endif

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
  FAR netpkt_t                  *pkt;

  /* Loop while receive() successfully retrieves valid Ethernet frames. */

  while ((pkt = lower->ops->receive(lower)) != NULL)
    {
      if (!IFF_IS_UP(dev->d_flags))
        {
          /* Interface down, drop frame */

          NETDEV_RXDROPPED(dev);
          netpkt_free(lower, pkt, NETPKT_RX);
          continue;
        }

      netpkt_put(dev, pkt, NETPKT_RX);
      NETDEV_RXPACKETS(dev);

#ifdef CONFIG_NET_PKT
      /* When packet sockets are enabled, feed the frame into the tap */

      pkt_input(dev);
#endif

      switch (dev->d_lltype)
        {
#ifdef CONFIG_NET_LOOPBACK
        case NET_LL_LOOPBACK:
#endif
#ifdef CONFIG_NET_ETHERNET
        case NET_LL_ETHERNET:
#endif
#ifdef CONFIG_DRIVERS_IEEE80211
        case NET_LL_IEEE80211:
#endif
#if defined(CONFIG_NET_LOOPBACK) || defined(CONFIG_NET_ETHERNET) || \
    defined(CONFIG_DRIVERS_IEEE80211)
          eth_input(dev);
          break;
#endif
#ifdef CONFIG_NET_MBIM
        case NET_LL_MBIM:
          ip_input(dev);
          break;
#endif
#ifdef CONFIG_NET_CAN
        case NET_LL_CAN:
          ninfo("CAN frame");
          can_input(dev);
          break;
#endif
        default:
          nerr("Unknown link type %d\n", dev->d_lltype);
          break;
        }
    }
}

/****************************************************************************
 * Name: netdev_upper_work
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
 * Name: netdev_upper_wait
 *
 * Description:
 *   Wait for timeout or signal.
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_WORK_THREAD
static int netdev_upper_wait(FAR sem_t *sem)
{
#if CONFIG_NETDEV_WORK_THREAD_POLLING_PERIOD > 0
  int ret =
    nxsem_tickwait(sem, USEC2TICK(CONFIG_NETDEV_WORK_THREAD_POLLING_PERIOD));

  return ret == -ETIMEDOUT ? OK : ret;
#else
  return nxsem_wait(sem);
#endif
}

/****************************************************************************
 * Name: netdev_upper_loop
 *
 * Description:
 *   The loop for dedicated thread.
 *
 ****************************************************************************/

static int netdev_upper_loop(int argc, FAR char *argv[])
{
  FAR struct netdev_upperhalf_s *upper =
    (FAR struct netdev_upperhalf_s *)((uintptr_t)strtoul(argv[1], NULL, 16));
  int cpu = atoi(argv[2]);

#ifdef CONFIG_NETDEV_RSS
  cpu_set_t cpuset;

  CPU_ZERO(&cpuset);
  CPU_SET(cpu, &cpuset);
  sched_setaffinity(upper->tid[cpu], sizeof(cpu_set_t), &cpuset);
#endif

  while (netdev_upper_wait(&upper->sem[cpu]) == OK &&
         upper->tid[cpu] != INVALID_PROCESS_ID)
    {
      netdev_upper_work(upper);
    }

  nwarn("WARNING: Netdev work thread quitting.");
  nxsem_post(&upper->sem_exit[cpu]);
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
#  ifdef CONFIG_NETDEV_RSS
  int cpu = this_cpu();
#  else
  const int cpu = 0;
#  endif
  int semcount;

  if (nxsem_get_value(&upper->sem[cpu], &semcount) == OK &&
      semcount <= 0)
    {
      nxsem_post(&upper->sem[cpu]);
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
 * Name: netdev_upper_wireless_ioctl
 *
 * Description:
 *   Support for wireless handlers in ioctl.
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_WIRELESS_HANDLER
int netdev_upper_wireless_ioctl(FAR struct netdev_lowerhalf_s *lower,
                                int cmd, unsigned long arg)
{
  int ret = -ENOTTY; /* Default to ENOTTY to indicate not serving. */
  FAR struct iwreq *iwr = (FAR struct iwreq *)arg;
  FAR const struct wireless_ops_s *ops = lower->iw_ops;
  struct ether_addr zero;

  /* Decode and dispatch the driver-specific IOCTL command */

  switch (cmd)
    {
      case SIOCSIWENCODEEXT: /* Set encoding token & mode */
        if (ops->passwd)
          {
            ret = ops->passwd(lower, iwr, true);
          }
        break;

      case SIOCGIWENCODEEXT: /* Get encoding token & mode */
        if (ops->passwd)
          {
            ret = ops->passwd(lower, iwr, false);
          }
        break;

      case SIOCSIWESSID: /* Set ESSID */
        if (ops->essid)
          {
            if ((iwr->u.essid.flags == IW_ESSID_ON) ||
                (iwr->u.essid.flags == IW_ESSID_DELAY_ON))
              {
                ret = ops->essid(lower, iwr, true);
                if (ret < 0)
                  {
                    break;
                  }

                if (iwr->u.essid.flags == IW_ESSID_ON)
                  {
                    ret = ops->connect(lower);
                    if (ret < 0)
                      {
                        nerr("ERROR: Failed to connect\n");
                        break;
                      }
                  }
              }
            else
              {
                ret = ops->disconnect(lower);
                if (ret < 0)
                  {
                    nerr("ERROR: Failed to disconnect\n");
                    break;
                  }
              }
          }
        break;

      case SIOCGIWESSID: /* Get ESSID */
        if (ops->essid)
          {
            ret = ops->essid(lower, iwr, false);
          }
        break;

      case SIOCSIWAP: /* Set access point MAC addresses */
        if (ops->bssid)
          {
            memset(&zero, 0, sizeof(zero));
            if (memcmp(iwr->u.ap_addr.sa_data, &zero, sizeof(zero)) != 0)
              {
                ret = ops->bssid(lower, iwr, true);
                if (ret < 0)
                  {
                    nerr("ERROR: Failed to set BSSID\n");
                    break;
                  }

                ret = ops->connect(lower);
                if (ret < 0)
                  {
                    nerr("ERROR: Failed to connect\n");
                    break;
                  }
              }
            else
              {
                ret = ops->disconnect(lower);
                if (ret < 0)
                  {
                    nerr("ERROR: Failed to disconnect\n");
                    break;
                  }
              }
          }
        break;

      case SIOCGIWAP: /* Get access point MAC addresses */
        if (ops->bssid)
          {
            ret = ops->bssid(lower, iwr, false);
          }
        break;

      case SIOCSIWSCAN: /* Trigger scanning */
        if (ops->scan)
          {
            ret = ops->scan(lower, iwr, true);
          }
        break;

      case SIOCGIWSCAN: /* Get scanning results */
        if (ops->scan)
          {
            ret = ops->scan(lower, iwr, false);
          }
        break;

      case SIOCSIWCOUNTRY: /* Set country code */
        if (ops->country)
          {
            ret = ops->country(lower, iwr, true);
          }
        break;

      case SIOCGIWCOUNTRY: /* Get country code */
        if (ops->country)
          {
            ret = ops->country(lower, iwr, false);
          }
        break;

      case SIOCSIWSENS: /* Set sensitivity (dBm) */
        if (ops->sensitivity)
          {
            ret = ops->sensitivity(lower, iwr, true);
          }
        break;

      case SIOCGIWSENS: /* Get sensitivity (dBm) */
        if (ops->sensitivity)
          {
            ret = ops->sensitivity(lower, iwr, false);
          }
        break;

      case SIOCSIWMODE: /* Set operation mode */
        if (ops->mode)
          {
            ret = ops->mode(lower, iwr, true);
          }
        break;

      case SIOCGIWMODE: /* Get operation mode */
        if (ops->mode)
          {
            ret = ops->mode(lower, iwr, false);
          }
        break;

      case SIOCSIWAUTH: /* Set authentication mode params */
        if (ops->auth)
          {
            ret = ops->auth(lower, iwr, true);
          }
        break;

      case SIOCGIWAUTH: /* Get authentication mode params */
        if (ops->auth)
          {
            ret = ops->auth(lower, iwr, false);
          }
        break;

      case SIOCSIWFREQ: /* Set channel/frequency (MHz) */
        if (ops->freq)
          {
            ret = ops->freq(lower, iwr, true);
          }
        break;

      case SIOCGIWFREQ: /* Get channel/frequency (MHz) */
        if (ops->freq)
          {
            ret = ops->freq(lower, iwr, false);
          }
        break;

      case SIOCSIWRATE: /* Set default bit rate (Mbps) */
        if (ops->bitrate)
          {
            ret = ops->bitrate(lower, iwr, true);
          }
        break;

      case SIOCGIWRATE: /* Get default bit rate (Mbps) */
        if (ops->bitrate)
          {
            ret = ops->bitrate(lower, iwr, false);
          }
        break;

      case SIOCSIWTXPOW: /* Set transmit power (dBm) */
        if (ops->txpower)
          {
            ret = ops->txpower(lower, iwr, true);
          }
        break;

      case SIOCGIWTXPOW: /* Get transmit power (dBm) */
        if (ops->txpower)
          {
            ret = ops->txpower(lower, iwr, false);
          }
        break;

      case SIOCGIWRANGE: /* Get range of parameters */
        if (ops->range)
          {
            ret = ops->range(lower, iwr);
          }
        break;

      default:
        nerr("ERROR: Unrecognized IOCTL command: %d\n", cmd);
        break;
    }

  return ret;
}
#endif  /* CONFIG_NETDEV_WIRELESS_HANDLER */

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
  int i;

  /* Try to bring up a dedicated thread for work. */

  for (i = 0; i < NETDEV_THREAD_COUNT; i++)
    {
      if (upper->tid[i] <= 0)
        {
          FAR char *argv[3];
          char      arg1[32];
          char      arg2[32];
          char      name[32];

          snprintf(arg1, sizeof(arg1), "%p", upper);
          argv[0] = arg1;

          snprintf(arg2, sizeof(arg2), "%d", i);
          argv[1] = arg2;
          argv[2] = NULL;

          snprintf(name, sizeof(name), NETDEV_THREAD_NAME_FMT,
                   dev->d_ifname);

          upper->tid[i] = kthread_create(name,
                                         CONFIG_NETDEV_WORK_THREAD_PRIORITY,
                                         CONFIG_DEFAULT_TASK_STACKSIZE,
                                         netdev_upper_loop, argv);
          if (upper->tid[i] < 0)
            {
              return upper->tid[i];
            }
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
  FAR struct netdev_lowerhalf_s *lower = upper->lower;

#ifdef CONFIG_NETDEV_WIRELESS_HANDLER
  if (lower->iw_ops)
    {
      int ret = netdev_upper_wireless_ioctl(lower, cmd, arg);
      if (ret != -ENOTTY)
        {
          return ret;
        }
    }
#endif

  if (lower->ops->ioctl)
    {
      return lower->ops->ioctl(lower, cmd, arg);
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
#ifdef CONFIG_NETDEV_WORK_THREAD
  int i;
#endif

  if (dev == NULL || quota_is_valid(dev) == false || dev->ops == NULL ||
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
  for (i = 0; i < NETDEV_THREAD_COUNT; i++)
    {
      upper->tid[i] = INVALID_PROCESS_ID;
      nxsem_init(&upper->sem[i], 0, 0);
      nxsem_init(&upper->sem_exit[i], 0, 0);
    }
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
#ifdef CONFIG_NETDEV_WORK_THREAD
  int i;
#endif

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
  for (i = 0; i < NETDEV_THREAD_COUNT; i++)
    {
      if (upper->tid[i] > 0)
        {
          /* Try to tear down the dedicated thread for work. */

          upper->tid[i] = INVALID_PROCESS_ID;
          nxsem_post(&upper->sem[i]);
          nxsem_wait(&upper->sem_exit[i]);
        }

      nxsem_destroy(&upper->sem[i]);
      nxsem_destroy(&upper->sem_exit[i]);
    }
#endif

#if CONFIG_IOB_NCHAINS > 0
  iob_free_queue(&upper->txq);
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
 ****************************************************************************/

void netdev_lower_carrier_on(FAR struct netdev_lowerhalf_s *dev)
{
  netdev_carrier_on(&dev->netdev);
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
 ****************************************************************************/

void netdev_lower_carrier_off(FAR struct netdev_lowerhalf_s *dev)
{
  netdev_carrier_off(&dev->netdev);
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
#if CONFIG_NETDEV_WORK_THREAD_POLLING_PERIOD == 0
  netdev_upper_queue_work(&dev->netdev);
#endif
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
#if CONFIG_NETDEV_WORK_THREAD_POLLING_PERIOD == 0
  netdev_upper_queue_work(&dev->netdev);
#endif
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

  if (atomic_fetch_sub(&dev->quota[type], 1) <= 0)
    {
      atomic_fetch_add(&dev->quota[type], 1);
      return NULL;
    }

  pkt = iob_tryalloc(false);
  if (pkt == NULL)
    {
      atomic_fetch_add(&dev->quota[type], 1);
      return NULL;
    }

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
  atomic_fetch_add(&dev->quota[type], 1);
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
 * Returned Value:
 *   The new effective data length, or a negated errno value on error.
 *
 ****************************************************************************/

int netpkt_setdatalen(FAR struct netdev_lowerhalf_s *dev,
                      FAR netpkt_t *pkt, unsigned int len)
{
  uint8_t llhdrlen = NET_LL_HDRLEN(&dev->netdev);
  int ret = iob_update_pktlen(pkt, len - llhdrlen, false);
  return ret >= 0 ? ret + llhdrlen : ret;
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

/****************************************************************************
 * Name: netpkt_to_iov
 *
 * Description:
 *   Write each piece of data/len into iov array.
 *
 * Input Parameters:
 *   dev    - The lower half device driver structure
 *   pkt    - The net packet
 *   iov    - The iov array to write
 *   iovcnt - The number of elements in the iov array
 *
 * Returned Value:
 *   The actual written count of iov entries.
 *
 ****************************************************************************/

int netpkt_to_iov(FAR struct netdev_lowerhalf_s *dev, FAR netpkt_t *pkt,
                  FAR struct iovec *iov, int iovcnt)
{
  int i;

  for (i = 0; pkt != NULL && i < iovcnt; pkt = pkt->io_flink, i++)
    {
      if (i == 0)
        {
          iov[i].iov_base = IOB_DATA(pkt) - NET_LL_HDRLEN(&dev->netdev);
          iov[i].iov_len  = pkt->io_len   + NET_LL_HDRLEN(&dev->netdev);
        }
      else
        {
          iov[i].iov_base = IOB_DATA(pkt);
          iov[i].iov_len  = pkt->io_len;
        }
    }

  return i;
}
