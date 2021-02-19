/****************************************************************************
 * wireless/pktradio/pktradio_loopback.c
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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arpa/inet.h>
#include <net/if.h>

#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/mm/iob.h>
#include <nuttx/net/net.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/radiodev.h>
#include <nuttx/net/sixlowpan.h>
#include <nuttx/wireless/pktradio.h>

#ifdef CONFIG_PKTRADIO_LOOPBACK

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* We need to have the work queue to handle SPI interrupts */

#if !defined(CONFIG_SCHED_WORKQUEUE)
#  error Worker thread support is required (CONFIG_SCHED_WORKQUEUE)
#else
#  if defined(CONFIG_SCHED_LPWORK)
#    define LPBKWORK LPWORK
#  elif defined(CONFIG_SCHED_HPWORK)
#    define LPBKWORK HPWORK
#  else
#    error Neither CONFIG_SCHED_LPWORK nor CONFIG_SCHED_HPWORK defined
#  endif
#endif

#ifndef CONFIG_WIRELESS_PKTRADIO
#  error CONFIG_WIRELESS_PKTRADIO=y is required.
#endif

#ifndef CONFIG_NET_6LOWPAN
#  error CONFIG_NET_6LOWPAN=y is required.
#endif

#if (CONFIG_PKTRADIO_ADDRLEN != 1) && (CONFIG_PKTRADIO_ADDRLEN != 2) && \
    (CONFIG_PKTRADIO_ADDRLEN != 8)
#  error No support for CONFIG_PKTRADIO_ADDRLEN other than {1,2,8}
#endif

/* TX poll delay = 1 seconds.
 * CLK_TCK is the number of clock ticks per second
 */

#define LO_WDDELAY   (1*CLK_TCK)

/* Fake value for MAC header length */

#if CONFIG_IOB_BUFSIZE > 40
#  define MAC_HDRLEN   4
#else
#  define MAC_HDRLEN   0
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The lo_driver_s encapsulates all state information for a single hardware
 * interface
 */

struct lo_driver_s
{
  bool lo_bifup;               /* true:ifup false:ifdown */
  bool lo_pending;             /* True: TX poll pending */
  uint8_t lo_panid[2];         /* Fake PAN ID for testing */
  struct wdog_s lo_polldog;    /* TX poll timer */
  struct work_s lo_work;       /* For deferring poll work to the work queue */
  FAR struct iob_s *lo_head;   /* Head of IOBs queued for loopback */
  FAR struct iob_s *lo_tail;   /* Tail of IOBs queued for loopback */

  /* This holds the information visible to the NuttX network */

  struct radio_driver_s lo_radio;  /* Interface understood by the network */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct lo_driver_s g_loopback;
#ifdef CONFIG_NET_6LOWPAN
static struct sixlowpan_reassbuf_s g_iobuffer;
#endif

static uint8_t g_mac_addr[CONFIG_PKTRADIO_ADDRLEN] =
{
#if CONFIG_PKTRADIO_ADDRLEN == 1
  0xab
#elif CONFIG_PKTRADIO_ADDRLEN == 2
  0xab, 0xcd
#elif CONFIG_PKTRADIO_ADDRLEN == 8
  0x0c, 0xfa, 0xde, 0x00, 0xde, 0xad, 0xbe, 0xef
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Utility functions */

static void lo_addr2ip(FAR struct net_driver_s *dev);
static inline void lo_netmask(FAR struct net_driver_s *dev);

/* Polling logic */

static int  lo_loopback(FAR struct net_driver_s *dev);
static void lo_loopback_work(FAR void *arg);
static void lo_poll_work(FAR void *arg);
static void lo_poll_expiry(wdparm_t arg);

/* NuttX callback functions */

static int  lo_ifup(FAR struct net_driver_s *dev);
static int  lo_ifdown(FAR struct net_driver_s *dev);
static void lo_txavail_work(FAR void *arg);
static int  lo_txavail(FAR struct net_driver_s *dev);
#ifdef CONFIG_NET_MCASTGROUP
static int  lo_addmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac);
static int  lo_rmmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac);
#endif
#ifdef CONFIG_NETDEV_IOCTL
static int  lo_ioctl(FAR struct net_driver_s *dev, int cmd,
              unsigned long arg);
#endif
static int lo_get_mhrlen(FAR struct radio_driver_s *netdev,
              FAR const void *meta);
static int lo_req_data(FAR struct radio_driver_s *netdev,
              FAR const void *meta, FAR struct iob_s *framelist);
static int lo_properties(FAR struct radio_driver_s *netdev,
              FAR struct radiodev_properties_s *properties);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lo_addr2ip
 *
 * Description:
 *   Create a MAC-based IP address from the IEEE 802.15.14 short or extended
 *   address assigned to the node.
 *
 *  128  112  96   80    64   48   32   16
 * ---- ---- ---- ---- ---- ---- ---- ----
 * fe80 0000 0000 0000 0000 00ff fe00 xxxx 2-byte
 *                                            short address IEEE 48-bit MAC
 * fe80 0000 0000 0000 xxxx xxxx xxxx xxxx 8-byte
 *                                            extended address IEEE EUI-64
 *
 ****************************************************************************/

static void lo_addr2ip(FAR struct net_driver_s *dev)
{
  /* Set the MAC address as the saddr */

  dev->d_mac.radio.nv_addrlen = CONFIG_PKTRADIO_ADDRLEN;
  memcpy(dev->d_mac.radio.nv_addr, g_mac_addr, CONFIG_PKTRADIO_ADDRLEN);

  /* Set the IP address */

  dev->d_ipv6addr[0]  = HTONS(0xfe80);
  dev->d_ipv6addr[1]  = 0;
  dev->d_ipv6addr[2]  = 0;
  dev->d_ipv6addr[3]  = 0;

#if CONFIG_PKTRADIO_ADDRLEN == 1
  /* Set the IP address based on the 1 byte address */

  dev->d_ipv6addr[4]  = 0;
  dev->d_ipv6addr[5]  = HTONS(0x00ff);
  dev->d_ipv6addr[6]  = HTONS(0xfe00);
  dev->d_ipv6addr[7]  = (uint16_t)g_mac_addr[0] << 8;

#elif CONFIG_PKTRADIO_ADDRLEN == 2
  /* Set the IP address based on the 2 byte address */

  dev->d_ipv6addr[4]  = 0;
  dev->d_ipv6addr[5]  = HTONS(0x00ff);
  dev->d_ipv6addr[6]  = HTONS(0xfe00);
  dev->d_ipv6addr[7]  = (uint16_t)g_mac_addr[0] << 8 |
                        (uint16_t)g_mac_addr[1];

#elif CONFIG_PKTRADIO_ADDRLEN == 8
  /* Set the IP address based on the 8-byte address */

  dev->d_ipv6addr[4]  = (uint16_t)g_mac_addr[0] << 8 |
                        (uint16_t)g_mac_addr[1];
  dev->d_ipv6addr[5]  = (uint16_t)g_mac_addr[2] << 8 |
                        (uint16_t)g_mac_addr[3];
  dev->d_ipv6addr[6]  = (uint16_t)g_mac_addr[4] << 8 |
                        (uint16_t)g_mac_addr[5];
  dev->d_ipv6addr[7]  = (uint16_t)g_mac_addr[6] << 8 |
                        (uint16_t)g_mac_addr[7];
#endif
}

/****************************************************************************
 * Name: lo_netmask
 *
 * Description:
 *   Create a netmask of a MAC-based IP address which may be based on either
 *   the IEEE 802.15.14 short or extended address of the MAC.
 *
 *  128  112   96   80   64   48   32   16
 * ---- ---- ---- ---- ---- ---- ---- ----
 * fe80 0000 0000 0000 0000 00ff fe00 xxxx 2-byte
 *                                           short address IEEE 48-bit MAC
 * fe80 0000 0000 0000 xxxx xxxx xxxx xxxx 8-byte
 *                                           extended address IEEE EUI-64
 *
 ****************************************************************************/

static inline void lo_netmask(FAR struct net_driver_s *dev)
{
  dev->d_ipv6netmask[0]  = 0xffff;
  dev->d_ipv6netmask[1]  = 0xffff;
  dev->d_ipv6netmask[2]  = 0xffff;
  dev->d_ipv6netmask[3]  = 0xffff;

#if CONFIG_PKTRADIO_ADDRLEN == 1
  dev->d_ipv6netmask[4]  = 0xffff;
  dev->d_ipv6netmask[5]  = 0xffff;
  dev->d_ipv6netmask[6]  = 0xffff;
  dev->d_ipv6netmask[7]  = HTONS(0xff00);

#elif CONFIG_PKTRADIO_ADDRLEN == 2
  dev->d_ipv6netmask[4]  = 0xffff;
  dev->d_ipv6netmask[5]  = 0xffff;
  dev->d_ipv6netmask[6]  = 0xffff;
  dev->d_ipv6netmask[7]  = 0;

#elif CONFIG_PKTRADIO_ADDRLEN == 8
  dev->d_ipv6netmask[4]  = 0;
  dev->d_ipv6netmask[5]  = 0;
  dev->d_ipv6netmask[6]  = 0;
  dev->d_ipv6netmask[7]  = 0;
#endif
}

/****************************************************************************
 * Name: lo_loopback
 *
 * Description:
 *   Check if the network has any outgoing packets ready to send.  This is
 *   a callback from devif_poll() or devif_timer().  devif_poll() will be
 *   called only during normal TX polling.
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   the network is locked.
 *
 ****************************************************************************/

static int lo_loopback(FAR struct net_driver_s *dev)
{
  FAR struct lo_driver_s *priv = (FAR struct lo_driver_s *)dev->d_private;
  struct pktradio_metadata_s pktmeta;
  FAR struct iob_s *iob;
  int ret;

  /* Create some fake metadata */

  memset(&pktmeta, 0, sizeof(struct pktradio_metadata_s));

  pktmeta.pm_src.pa_addrlen  = CONFIG_PKTRADIO_ADDRLEN;
  pktmeta.pm_dest.pa_addrlen = CONFIG_PKTRADIO_ADDRLEN;

  /* On loopback the local address is both the source and destination. */

  memcpy(pktmeta.pm_src.pa_addr, g_mac_addr, CONFIG_PKTRADIO_ADDRLEN);
  memcpy(pktmeta.pm_dest.pa_addr, g_mac_addr, CONFIG_PKTRADIO_ADDRLEN);

  /* Loop while there framelist to be sent, i.e., while the freme list is not
   * empty.  Sending, of course, just means relaying back through the network
   * for this driver.
   */

  while (priv->lo_head != NULL)
    {
      ninfo("Looping frame IOB %p\n", iob);

      /* Increment statistics */

      NETDEV_RXPACKETS(&priv->lo_radio.r_dev);

      /* Remove the IOB from the queue */

      iob           = priv->lo_head;
      priv->lo_head = iob->io_flink;
      iob->io_flink = NULL;

      /* Did the framelist become empty? */

      if (priv->lo_head == NULL)
        {
          priv->lo_tail = NULL;
        }

      /* Make sure the our single packet buffer is attached */

      priv->lo_radio.r_dev.d_buf = g_iobuffer.rb_buf;

      /* Return the next frame to the network */

      ninfo("Send frame %p to the network:  Offset=%u Length=%u\n",
            iob, iob->io_offset, iob->io_len);

      ret = sixlowpan_input(&priv->lo_radio, iob, (FAR void *)&pktmeta);

      /* Increment statistics */

      NETDEV_TXPACKETS(&priv->lo_radio.r_dev);

      if (ret < 0)
        {
          nerr("ERROR: sixlowpan_input returned %d\n", ret);
          NETDEV_TXERRORS(&priv->lo_radio.r_dev);
          NETDEV_ERRORS(&priv->lo_radio.r_dev);
        }
    }

  return 0;
}

/****************************************************************************
 * Name: lo_loopback_work
 *
 * Description:
 *   Perform loopback of received framelist.
 *
 * Input Parameters:
 *   arg - The argument passed when work_queue() as called.
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *   The network is locked
 *
 ****************************************************************************/

static void lo_loopback_work(FAR void *arg)
{
  FAR struct lo_driver_s *priv = (FAR struct lo_driver_s *)arg;

  /* Perform the loopback */

  net_lock();
  lo_loopback(&priv->lo_radio.r_dev);
  net_unlock();
}

/****************************************************************************
 * Name: lo_poll_work
 *
 * Description:
 *   Perform periodic polling from the worker thread
 *
 * Input Parameters:
 *   arg - The argument passed when work_queue() as called.
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *   The network is locked
 *
 ****************************************************************************/

static void lo_poll_work(FAR void *arg)
{
  FAR struct lo_driver_s *priv = (FAR struct lo_driver_s *)arg;

  /* Perform the poll */

  net_lock();

#ifdef CONFIG_NET_6LOWPAN
  /* Make sure the our single packet buffer is attached */

  priv->lo_radio.r_dev.d_buf = g_iobuffer.rb_buf;
#endif

  /* And perform the poll */

  devif_timer(&priv->lo_radio.r_dev, LO_WDDELAY, lo_loopback);

  /* Setup the watchdog poll timer again */

  wd_start(&priv->lo_polldog, LO_WDDELAY, lo_poll_expiry, (wdparm_t)priv);
  net_unlock();
}

/****************************************************************************
 * Name: lo_poll_expiry
 *
 * Description:
 *   Periodic timer handler.  Called from the timer interrupt handler.
 *
 * Input Parameters:
 *   arg  - The argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void lo_poll_expiry(wdparm_t arg)
{
  FAR struct lo_driver_s *priv = (FAR struct lo_driver_s *)arg;

  if (!work_available(&priv->lo_work) || priv->lo_head != NULL)
    {
      nwarn("WARNING: lo_work NOT available\n");
      priv->lo_pending = true;
    }
  else
    {
      /* Schedule to perform the interrupt processing on the worker thread. */

      priv->lo_pending = false;
      work_queue(LPBKWORK, &priv->lo_work, lo_poll_work, priv, 0);
    }
}

/****************************************************************************
 * Name: lo_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Ethernet interface when an IP address is
 *   provided
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int lo_ifup(FAR struct net_driver_s *dev)
{
  FAR struct lo_driver_s *priv = (FAR struct lo_driver_s *)dev->d_private;

  ninfo("Bringing up: IPv6 %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        dev->d_ipv6addr[0], dev->d_ipv6addr[1], dev->d_ipv6addr[2],
        dev->d_ipv6addr[3], dev->d_ipv6addr[4], dev->d_ipv6addr[5],
        dev->d_ipv6addr[6], dev->d_ipv6addr[7]);

#if CONFIG_PKTRADIO_ADDRLEN == 1
  ninfo("Node: %02x\n",
         dev->d_mac.radio.nv_addr[0]);

#elif CONFIG_PKTRADIO_ADDRLEN == 2
  ninfo("Node: %02x:%02x\n",
         dev->d_mac.radio.nv_addr[0], dev->d_mac.radio.nv_addr[1]);

#elif CONFIG_PKTRADIO_ADDRLEN == 8
  ninfo("Node: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x PANID=%02x:%02x\n",
         dev->d_mac.radio.nv_addr[0], dev->d_mac.radio.nv_addr[1],
         dev->d_mac.radio.nv_addr[2], dev->d_mac.radio.nv_addr[3],
         dev->d_mac.radio.nv_addr[4], dev->d_mac.radio.nv_addr[5],
         dev->d_mac.radio.nv_addr[6], dev->d_mac.radio.nv_addr[7]);
#endif

  /* Set and activate a timer process */

  wd_start(&priv->lo_polldog, LO_WDDELAY,
           lo_poll_expiry, (wdparm_t)priv);

  priv->lo_bifup = true;
  return OK;
}

/****************************************************************************
 * Name: lo_ifdown
 *
 * Description:
 *   NuttX Callback: Stop the interface.
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int lo_ifdown(FAR struct net_driver_s *dev)
{
  FAR struct lo_driver_s *priv = (FAR struct lo_driver_s *)dev->d_private;

  ninfo("IP up: %u\n", priv->lo_bifup);

  /* Cancel the TX poll timer and TX timeout timers */

  wd_cancel(&priv->lo_polldog);

  /* Mark the device "down" */

  priv->lo_bifup = false;
  return OK;
}

/****************************************************************************
 * Name: lo_txavail_work
 *
 * Description:
 *   Perform an out-of-cycle poll on the worker thread.
 *
 * Input Parameters:
 *   arg - Reference to the NuttX driver state structure (cast to void*)
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called on the higher priority worker thread.
 *
 ****************************************************************************/

static void lo_txavail_work(FAR void *arg)
{
  FAR struct lo_driver_s *priv = (FAR struct lo_driver_s *)arg;

  ninfo("TX available work. IP up: %u\n", priv->lo_bifup);

  /* Ignore the notification if the interface is not yet up */

  net_lock();
  if (priv->lo_bifup)
    {
      /* If so, then poll the network for new XMIT data */

#ifdef CONFIG_NET_6LOWPAN
      /* Make sure the our single packet buffer is attached */

      priv->lo_radio.r_dev.d_buf = g_iobuffer.rb_buf;
#endif

      /* Then perform the poll */

      devif_poll(&priv->lo_radio.r_dev, lo_loopback);
    }

  net_unlock();
}

/****************************************************************************
 * Name: lo_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called in normal user mode
 *
 ****************************************************************************/

static int lo_txavail(FAR struct net_driver_s *dev)
{
  FAR struct lo_driver_s *priv = (FAR struct lo_driver_s *)dev->d_private;

  ninfo("Available: %u\n", work_available(&priv->lo_work));

  /* Is our single work structure available?  It may not be if there are
   * pending actions and we will have to ignore the Tx availability
   * action.
   */

  if (!work_available(&priv->lo_work) || priv->lo_head != NULL)
    {
      nwarn("WARNING: lo_work NOT available\n");
      priv->lo_pending = true;
    }
  else
    {
      /* Schedule to perform the interrupt processing on the worker thread. */

      priv->lo_pending = false;
      work_queue(LPBKWORK, &priv->lo_work, lo_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: lo_addmac
 *
 * Description:
 *   NuttX Callback: Add the specified MAC address to the hardware multicast
 *   address filtering
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be added
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_MCASTGROUP
static int lo_addmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac)
{
#if CONFIG_PKTRADIO_ADDRLEN == 1
  ninfo("MAC: %02x\n", mac[0]);

#elif CONFIG_PKTRADIO_ADDRLEN == 2
  ninfo("MAC: %02x:%02x\n", mac[0], mac[1]);

#elif CONFIG_PKTRADIO_ADDRLEN == 8
  ninfo("MAC: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
         mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], mac[6], mac[7]);
#endif

  /* There is no multicast support in the loopback driver */

  return OK;
}
#endif

/****************************************************************************
 * Name: lo_rmmac
 *
 * Description:
 *   NuttX Callback:
 *   Remove the specified MAC address from the hardware multicast
 *   address filtering
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be removed
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_MCASTGROUP
static int lo_rmmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac)
{
#if CONFIG_PKTRADIO_ADDRLEN == 1
  ninfo("MAC: %02x\n", mac[0]);

#elif CONFIG_PKTRADIO_ADDRLEN == 2
  ninfo("MAC: %02x:%02x\n", mac[0], mac[1]);

#elif CONFIG_PKTRADIO_ADDRLEN == 8
  ninfo("MAC: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x\n",
         mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], mac[6], mac[7]);
#endif

  /* There is no multicast support in the loopback driver */

  return OK;
}
#endif

/****************************************************************************
 * Name: macnet_ioctl
 *
 * Description:
 *   Handle network IOCTL commands directed to this device.
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *   cmd - The IOCTL command
 *   arg - The argument for the IOCTL command
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_IOCTL
static int lo_ioctl(FAR struct net_driver_s *dev, int cmd,
                    unsigned long arg)
{
  FAR struct pktradio_ifreq_s *cmddata;
  FAR struct lo_driver_s *priv;
  int ret = -ENOTTY;

  DEBUGASSERT(dev != NULL && dev->d_private != NULL && arg != 0ul);
  priv    = (FAR struct lo_driver_s *)dev->d_private;
  cmddata = (FAR struct pktradio_ifreq_s *)((uintptr_t)arg);

  switch (cmd)
    {
      /* SIOCPKTRADIOGGPROPS
       *   Description:   Get the radio properties
       *   Input:         Pointer to read-write instance of struct
       *                  pktradio_ifreq_s
       *   Output:        Properties returned in struct pktradio_ifreq_s
       *                  instance
       */

      case SIOCPKTRADIOGGPROPS:
        {
          FAR struct radio_driver_s *radio =
            (FAR struct radio_driver_s *)dev;
          FAR struct radiodev_properties_s *props =
            (FAR struct radiodev_properties_s *)&cmddata->pifr_props;

          ret = lo_properties(radio, props);
        }
        break;

      /* SIOCPKTRADIOSNODE
       *   Description:   Set the radio node address
       *   Input:         Pointer to read-only instance of struct
       *                  pktradio_ifreq_s
       *   Output:        None
       */

      case SIOCPKTRADIOSNODE:
        {
          FAR const struct pktradio_addr_s *newaddr =
            (FAR const struct pktradio_addr_s *)&cmddata->pifr_hwaddr;

          if (newaddr->pa_addrlen != 1)
            {
              ret = -EINVAL;
            }
          else
           {
              FAR struct netdev_varaddr_s *devaddr = &dev->d_mac.radio;

              devaddr->nv_addrlen = 1;
              devaddr->nv_addr[0] = newaddr->pa_addr[0];
#if CONFIG_PKTRADIO_ADDRLEN > 1
              memset(&devaddr->pa_addr[1], 0, CONFIG_PKTRADIO_ADDRLEN - 1);
#endif
              ret = OK;
            }
        }
        break;

      /* SIOCPKTRADIOGNODE
       *   Description:   Get the radio node address
       *   Input:         Pointer to read-write instance of
       *                  struct pktradio_ifreq_s
       *   Output:        Node address return in struct pktradio_ifreq_s
       *                  instance
       */

      case SIOCPKTRADIOGNODE:
        {
          FAR struct pktradio_addr_s *retaddr =
            (FAR struct pktradio_addr_s *)&cmddata->pifr_hwaddr;
          FAR const struct netdev_varaddr_s *devaddr = &dev->d_mac.radio;

          retaddr->pa_addrlen = devaddr->nv_addrlen;
          retaddr->pa_addr[0] = devaddr->nv_addr[0];
#if CONFIG_PKTRADIO_ADDRLEN > 1
          memset(&addr->pa_addr[1], 0, CONFIG_PKTRADIO_ADDRLEN - 1);
#endif
          ret = OK;
        }
        break;

      default:
        wlwarn("WARNING: Unrecognized IOCTL command: %02x\n", cmd);
        break;
    }

  UNUSED(priv);
  return ret;
}
#endif

/****************************************************************************
 * Name: lo_get_mhrlen
 *
 * Description:
 *   Calculate the MAC header length given the frame meta-data.
 *
 * Input Parameters:
 *   netdev    - The networkd device that will mediate the MAC interface
 *   meta      - Obfuscated metadata structure needed to create the radio
 *               MAC header
 *
 * Returned Value:
 *   A non-negative MAC headeer length is returned on success; a negated
 *   errno value is returned on any failure.
 *
 ****************************************************************************/

static int lo_get_mhrlen(FAR struct radio_driver_s *netdev,
                         FAR const void *meta)
{
  return MAC_HDRLEN;
}

/****************************************************************************
 * Name: lo_req_data
 *
 * Description:
 *   Requests the transfer of a list of frames to the MAC.
 *
 * Input Parameters:
 *   netdev    - The networkd device that will mediate the MAC interface
 *   meta      - Obfuscated metadata structure needed to create the radio
 *               MAC header
 *   framelist - Head of a list of frames to be transferred.
 *
 * Returned Value:
 *   Zero (OK) returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int lo_req_data(FAR struct radio_driver_s *netdev,
                       FAR const void *meta, FAR struct iob_s *framelist)
{
  FAR struct lo_driver_s *priv;
  FAR struct iob_s *iob;

  DEBUGASSERT(netdev != NULL && netdev->r_dev.d_private != NULL);
  priv = (FAR struct lo_driver_s *)netdev->r_dev.d_private;

  DEBUGASSERT(meta != NULL && framelist != NULL);

  /* Add the incoming list of framelist to queue of framelist to loopback */

  for (iob = framelist; iob != NULL; iob = framelist)
    {
      /* Increment statistics */

      NETDEV_RXPACKETS(&priv->lo_radio.r_dev);

      /* Remove the IOB from the queue */

      framelist     = iob->io_flink;
      iob->io_flink = NULL;

      ninfo("Queuing frame IOB %p\n", iob);

      /* Just zero the MAC header for test purposes */

      DEBUGASSERT(iob->io_offset == MAC_HDRLEN);
      memset(iob->io_data, 0, MAC_HDRLEN);

      /* Add the IOB to the tail of the queue of framelist to be looped
       * back
       */

      if (priv->lo_tail == NULL)
        {
          priv->lo_head = iob;
        }
      else
        {
          priv->lo_tail->io_flink = iob;
        }

      priv->lo_tail = iob;
    }

  /* Schedule to serialize the poll on the worker thread. */

  work_queue(LPBKWORK, &priv->lo_work, lo_loopback_work, priv, 0);
  return OK;
}

/****************************************************************************
 * Name: lo_properties
 *
 * Description:
 *   Different packet radios may have different properties.  If there are
 *   multiple packet radios, then those properties have to be queried at
 *   run time.  This information is provided to the 6LoWPAN network via the
 *   following structure.
 *
 * Input Parameters:
 *   netdev     - The network device to be queried
 *   properties - Location where radio properties will be returned.
 *
 * Returned Value:
 *   Zero (OK) returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int lo_properties(FAR struct radio_driver_s *netdev,
                         FAR struct radiodev_properties_s *properties)
{
  DEBUGASSERT(netdev != NULL && properties != NULL);
  memset(properties, 0, sizeof(struct radiodev_properties_s));

  /* General */

  properties->sp_addrlen  = CONFIG_PKTRADIO_ADDRLEN; /* Length of an address */
  properties->sp_framelen = CONFIG_IOB_BUFSIZE;      /* Fixed frame length */

  /* Multicast address */

  properties->sp_mcast.nv_addrlen = CONFIG_PKTRADIO_ADDRLEN;
  memset(properties->sp_mcast.nv_addr, 0xee, RADIO_MAX_ADDRLEN);

  /* Broadcast address */

  properties->sp_bcast.nv_addrlen = CONFIG_PKTRADIO_ADDRLEN;
  memset(properties->sp_mcast.nv_addr, 0xff, RADIO_MAX_ADDRLEN);

#ifdef CONFIG_NET_STARPOINT
  /* Star hub node address */

  properties->sp_hubnode.nv_addrlen = 1;
  properties->sp_hubnode.nv_addr[0] = CONFIG_SPIRIT_HUBNODE;
#endif

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pktradio_loopback
 *
 * Description:
 *   Initialize and register the Ieee802.15.4 MAC loopback network driver.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int pktradio_loopback(void)
{
  FAR struct lo_driver_s *priv;
  FAR struct radio_driver_s *radio;
  FAR struct net_driver_s *dev;

  ninfo("Initializing\n");

  /* Get the interface structure associated with this interface number. */

  priv = &g_loopback;

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct lo_driver_s));

  radio               = &priv->lo_radio;
  dev                 = &radio->r_dev;
  dev->d_ifup         = lo_ifup;          /* I/F up (new IP address) callback */
  dev->d_ifdown       = lo_ifdown;        /* I/F down callback */
  dev->d_txavail      = lo_txavail;       /* New TX data callback */
#ifdef CONFIG_NET_MCASTGROUP
  dev->d_addmac       = lo_addmac;        /* Add multicast MAC address */
  dev->d_rmmac        = lo_rmmac;         /* Remove multicast MAC address */
#endif
#ifdef CONFIG_NETDEV_IOCTL
  dev->d_ioctl        = lo_ioctl;         /* Handle network IOCTL commands */
#endif
  dev->d_private      = priv;             /* Used to recover private state from dev */

  /* Set the network mask and advertise our MAC-based IP address */

  lo_netmask(dev);
  lo_addr2ip(dev);

  /* Initialize the Network frame-related callbacks */

  radio->r_get_mhrlen = lo_get_mhrlen;    /* Get MAC header length */
  radio->r_req_data   = lo_req_data;      /* Enqueue frame for transmission */
  radio->r_properties = lo_properties;    /* Returns radio properties */

#ifdef CONFIG_NET_6LOWPAN
  /* Make sure the our single packet buffer is attached.
   * We must do this before registering the device since,
   *  once the device is registered, a packet may
   * be attempted to be forwarded and require the buffer.
   */

  priv->lo_radio.r_dev.d_buf = g_iobuffer.rb_buf;
#endif

  /* Register the loopabck device with the OS so that socket IOCTLs can
   * be performed.
   */

  netdev_register(&priv->lo_radio.r_dev, NET_LL_PKTRADIO);

  /* Put the network in the UP state */

  dev->d_flags = IFF_UP;
  return lo_ifup(&priv->lo_radio.r_dev);
}

#endif /* CONFIG_PKTRADIO_LOOPBACK */
