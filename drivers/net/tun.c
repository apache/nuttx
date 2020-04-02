/****************************************************************************
 * drivers/net/tun.c
 *
 *   Copyright (C) 2015-2016 Max Nekludov. All rights reserved.
 *   Author: Max Nekludov <macscomp@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <fcntl.h>
#include <debug.h>
#include <poll.h>

#include <arpa/inet.h>

#include <net/if.h>

#ifdef CONFIG_NET_PKT
#  include <nuttx/net/pkt.h>
#endif

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/ethernet.h>
#include <nuttx/net/tun.h>

#if defined(CONFIG_NET) && defined(CONFIG_NET_TUN)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If processing is not done at the interrupt level, then high priority
 * work queue support is required.
 */

#if !defined(CONFIG_SCHED_WORKQUEUE)
#  error Work queue support is required in this configuration (CONFIG_SCHED_WORKQUEUE)
#endif

/* The low priority work queue is preferred.  If it is not enabled, LPWORK
 * will be the same as HPWORK.
 *
 * NOTE:  However, the network should NEVER run on the high priority work
 * queue!  That queue is intended only to service short back end interrupt
 * processing that never suspends.  Suspending the high priority work queue
 * may bring the system to its knees!
 */

#define TUNWORK LPWORK

/* CONFIG_TUN_NINTERFACES determines the number of physical interfaces
 * that will be supported.
 */

#ifndef CONFIG_TUN_NINTERFACES
#  define CONFIG_TUN_NINTERFACES 1
#endif

/* Make sure that packet buffers include in configured guard size and are an
 * even multiple of 16-bits in length.
 */

#define NET_TUN_PKTSIZE ((CONFIG_NET_TUN_PKTSIZE + CONFIG_NET_GUARDSIZE + 1) & ~1)

/* TX poll delay = 1 seconds. CLK_TCK is the number of clock ticks per
 * second
 */

#define TUN_WDDELAY  (1 * CLK_TCK)

/* This is a helper pointer for accessing the contents of the Ethernet header */

#ifdef CONFIG_NET_ETHERNET
#  define BUF ((FAR struct eth_hdr_s *)priv->dev.d_buf)
#endif

/* This is a helper pointer for accessing the contents of the ip header */

#define IPv4BUF ((FAR struct ipv4_hdr_s *)(priv->dev.d_buf + priv->dev.d_llhdrlen))
#define IPv6BUF ((FAR struct ipv6_hdr_s *)(priv->dev.d_buf + priv->dev.d_llhdrlen))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The tun_device_s encapsulates all state information for a single hardware
 * interface
 */

struct tun_device_s
{
  bool              bifup;     /* true:ifup false:ifdown */
  bool              read_wait;
  bool              write_wait;
  WDOG_ID           txpoll;    /* TX poll timer */
  struct work_s     work;      /* For deferring poll work to the work queue */
  FAR struct pollfd *poll_fds;
  sem_t             waitsem;
  sem_t             read_wait_sem;
  sem_t             write_wait_sem;
  size_t            read_d_len;
  size_t            write_d_len;

  /* These packet buffer arrays required 16-bit alignment.  That alignment
   * is assured only by the preceding wide data types.
   */

  uint8_t           read_buf[NET_TUN_PKTSIZE];
  uint8_t           write_buf[NET_TUN_PKTSIZE];

  /* This holds the information visible to the NuttX network */

  struct net_driver_s dev;     /* Interface understood by the network */
};

struct tun_driver_s
{
  uint8_t free_tuns;
  sem_t waitsem;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  tun_lock(FAR struct tun_device_s *priv);
static void tun_unlock(FAR struct tun_device_s *priv);

/* Common TX logic */

static void tun_fd_transmit(FAR struct tun_device_s *priv);
static int  tun_txpoll(FAR struct net_driver_s *dev);
#ifdef CONFIG_NET_ETHERNET
static int  tun_txpoll_tap(FAR struct net_driver_s *dev);
#endif
static int  tun_txpoll_tun(FAR struct net_driver_s *dev);

/* Interrupt handling */

static void tun_net_receive(FAR struct tun_device_s *priv);
#ifdef CONFIG_NET_ETHERNET
static void tun_net_receive_tap(FAR struct tun_device_s *priv);
#endif
static void tun_net_receive_tun(FAR struct tun_device_s *priv);

static void tun_txdone(FAR struct tun_device_s *priv);

/* Watchdog timer expirations */

static void tun_poll_work(FAR void *arg);
static void tun_poll_expiry(int argc, wdparm_t arg, ...);

/* NuttX callback functions */

static int tun_ifup(FAR struct net_driver_s *dev);
static int tun_ifdown(FAR struct net_driver_s *dev);
static int tun_txavail(FAR struct net_driver_s *dev);
#ifdef CONFIG_NET_MCASTGROUP
static int tun_addmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac);
static int tun_rmmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac);
#endif

static int tun_dev_init(FAR struct tun_device_s *priv,
                        FAR struct file *filep,
                        FAR const char *devfmt, bool tun);
static void tun_dev_uninit(FAR struct tun_device_s *priv);

/* File interface */

static int tun_open(FAR struct file *filep);
static int tun_close(FAR struct file *filep);
static ssize_t tun_read(FAR struct file *filep, FAR char *buffer,
                        size_t buflen);
static ssize_t tun_write(FAR struct file *filep, FAR const char *buffer,
                         size_t buflen);
static int tun_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int tun_poll(FAR struct file *filep, FAR struct pollfd *fds,
                    bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct tun_driver_s g_tun;
static struct tun_device_s g_tun_devices[CONFIG_TUN_NINTERFACES];

static const struct file_operations g_tun_file_ops =
{
  tun_open,     /* open */
  tun_close,    /* close */
  tun_read,     /* read */
  tun_write,    /* write */
  NULL,         /* seek */
  tun_ioctl,    /* ioctl */
  tun_poll,     /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  NULL,         /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tundev_lock
 ****************************************************************************/

static int tundev_lock(FAR struct tun_driver_s *tun)
{
  return nxsem_wait_uninterruptible(&tun->waitsem);
}

/****************************************************************************
 * Name: tundev_unlock
 ****************************************************************************/

static void tundev_unlock(FAR struct tun_driver_s *tun)
{
  nxsem_post(&tun->waitsem);
}

/****************************************************************************
 * Name: tun_lock
 ****************************************************************************/

static int tun_lock(FAR struct tun_device_s *priv)
{
  return nxsem_wait_uninterruptible(&priv->waitsem);
}

/****************************************************************************
 * Name: tun_unlock
 ****************************************************************************/

static void tun_unlock(FAR struct tun_device_s *priv)
{
  nxsem_post(&priv->waitsem);
}

/****************************************************************************
 * Name: tun_pollnotify
 ****************************************************************************/

static void tun_pollnotify(FAR struct tun_device_s *priv,
                           pollevent_t eventset)
{
  FAR struct pollfd *fds = priv->poll_fds;

  if (priv->read_wait && (eventset & POLLIN))
    {
      priv->read_wait = false;
      nxsem_post(&priv->read_wait_sem);
    }

  if (priv->write_wait && (eventset & POLLOUT))
    {
      priv->write_wait = false;
      nxsem_post(&priv->write_wait_sem);
    }

  if (fds == NULL)
    {
      return;
    }

  eventset &= fds->events;

  if (eventset != 0)
    {
      fds->revents |= eventset;
      nxsem_post(fds->sem);
    }
}

/****************************************************************************
 * Name: tun_fd_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static void tun_fd_transmit(FAR struct tun_device_s *priv)
{
  NETDEV_TXPACKETS(&priv->dev);
  tun_pollnotify(priv, POLLIN);
}

/****************************************************************************
 * Name: tun_txpoll
 *
 * Description:
 *   The transmitter is available, check if the network has any outgoing
 *   packets ready to send.  This is a callback from devif_poll().
 *   devif_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete,
 *   2. When the preceding TX packet send timesout and the interface is reset
 *   3. During normal TX polling
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static int tun_txpoll(FAR struct net_driver_s *dev)
{
  int ret;

#ifdef CONFIG_NET_ETHERNET
  if (dev->d_lltype == NET_LL_ETHERNET)
    {
      ret = tun_txpoll_tap(dev);
    }
  else
#endif
    {
      ret = tun_txpoll_tun(dev);
    }

  return ret;
}

/****************************************************************************
 * Name: tun_txpoll_tap : for tap (ethernet bridge) mode
 *
 * Description:
 *   The transmitter is available, check if the network has any outgoing
 *   packets ready to send.  This is a callback from devif_poll().
 *   devif_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete,
 *   2. When the preceding TX packet send timesout and the interface is reset
 *   3. During normal TX polling
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ETHERNET
static int tun_txpoll_tap(FAR struct net_driver_s *dev)
{
  FAR struct tun_device_s *priv = (FAR struct tun_device_s *)dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (priv->dev.d_len > 0)
    {
      /* Look up the destination MAC address and add it to the Ethernet
       * header.
       */

#ifdef CONFIG_NET_IPv4
      if (IFF_IS_IPv4(priv->dev.d_flags))
        {
          arp_out(&priv->dev);
        }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
      if (IFF_IS_IPv6(priv->dev.d_flags))
        {
          neighbor_out(&priv->dev);
        }
#endif /* CONFIG_NET_IPv6 */

      if (!devif_loopback(dev))
        {
          /* Send the packet */

          priv->read_d_len = priv->dev.d_len;
          tun_fd_transmit(priv);

          return 1;
        }
    }

  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return 0;
}
#endif

/****************************************************************************
 * Name: tun_txpoll_tun : for tun (IP tunneling) mode
 *
 * Description:
 *   The transmitter is available, check if the network has any outgoing
 *   packets ready to send.  This is a callback from devif_poll().
 *   devif_poll() may be called:
 *
 *   1. When the preceding TX packet send is complete,
 *   2. When the preceding TX packet send timesout and the interface is reset
 *   3. During normal TX polling
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   May or may not be called from an interrupt handler.  In either case,
 *   global interrupts are disabled, either explicitly or indirectly through
 *   interrupt handling logic.
 *
 ****************************************************************************/

static int tun_txpoll_tun(FAR struct net_driver_s *dev)
{
  FAR struct tun_device_s *priv = (FAR struct tun_device_s *)dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (priv->dev.d_len > 0)
    {
      if (!devif_loopback(dev))
        {
          /* Send the packet */

          priv->read_d_len = priv->dev.d_len;
          tun_fd_transmit(priv);

          return 1;
        }
    }

  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return 0;
}

/****************************************************************************
 * Name: tun_net_receive
 *
 * Description:
 *   An interrupt was received indicating the availability of a new RX
 *   packet
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void tun_net_receive(FAR struct tun_device_s *priv)
{
#ifdef CONFIG_NET_ETHERNET
  if (priv->dev.d_lltype == NET_LL_ETHERNET)
    {
      tun_net_receive_tap(priv);
    }
  else
#endif
    {
      tun_net_receive_tun(priv);
    }
}

/****************************************************************************
 * Name: tun_net_receive_tap : for tap (ethernet bridge) mode
 *
 * Description:
 *   An interrupt was received indicating the availability of a new RX packet
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ETHERNET
static void tun_net_receive_tap(FAR struct tun_device_s *priv)
{
  /* Copy the data data from the hardware to priv->dev.d_buf.  Set amount of
   * data in priv->dev.d_len
   */

  NETDEV_RXPACKETS(&priv->dev);

#ifdef CONFIG_NET_PKT
  /* When packet sockets are enabled, feed the frame into the packet tap */

  pkt_input(&priv->dev);
#endif

  /* We only accept IP packets of the configured type and ARP packets */

#if defined(CONFIG_NET_IPv4)
  if (BUF->type == HTONS(ETHTYPE_IP))
    {
      ninfo("IPv4 frame\n");
      NETDEV_RXIPV4(&priv->dev);

      /* Give the IPv4 packet to the network layer. */

      arp_ipin(&priv->dev);
      ipv4_input(&priv->dev);
    }
  else
#endif
#ifdef CONFIG_NET_IPv6
  if (BUF->type == HTONS(ETHTYPE_IP6))
    {
      ninfo("IPv6 frame\n");
      NETDEV_RXIPV6(&priv->dev);

      /* Give the IPv6 packet to the network layer. */

      ipv6_input(&priv->dev);
    }
  else
#endif
#ifdef CONFIG_NET_ARP
  if (BUF->type == htons(ETHTYPE_ARP))
    {
      arp_arpin(&priv->dev);
      NETDEV_RXARP(&priv->dev);

      /* If the above function invocation resulted in data that should be
       * sent out on the network, the field d_len will set to a value > 0.
       */

      if (priv->dev.d_len > 0)
        {
          priv->write_d_len = priv->dev.d_len;
          tun_fd_transmit(priv);
          priv->dev.d_len = 0;
        }
    }
  else
#endif
    {
      NETDEV_RXDROPPED(&priv->dev);
      priv->dev.d_len = 0;
    }

  /* If the above function invocation resulted in data that should be
   * sent out on the network, the field d_len will set to a value > 0.
   */

  if (priv->dev.d_len > 0)
    {
      /* Update the Ethernet header with the correct MAC address */

#ifdef CONFIG_NET_IPv6
      if (IFF_IS_IPv4(priv->dev.d_flags))
#endif
        {
          arp_out(&priv->dev);
        }
#ifdef CONFIG_NET_IPv6
      else
        {
          neighbor_out(&priv->dev);
        }
#endif

      /* And send the packet */

      priv->write_d_len = priv->dev.d_len;
      tun_fd_transmit(priv);
    }
}
#endif

/****************************************************************************
 * Name: tun_net_receive_tun : for tun (IP tunneling) mode
 *
 * Description:
 *   An interrupt was received indicating the availability of a new RX
 *   packet
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void tun_net_receive_tun(FAR struct tun_device_s *priv)
{
  /* Copy the data data from the hardware to priv->dev.d_buf.  Set amount of
   * data in priv->dev.d_len
   */

  NETDEV_RXPACKETS(&priv->dev);

#ifdef CONFIG_NET_PKT
  /* When packet sockets are enabled, feed the frame into the packet tap */

  pkt_input(&priv->dev);
#endif

  /* We only accept IP packets of the configured type */

#if defined(CONFIG_NET_IPv4)
  if ((IPv4BUF->vhl & IP_VERSION_MASK) == IPv4_VERSION)
    {
      ninfo("IPv4 frame\n");
      NETDEV_RXIPV4(&priv->dev);

      /* Give the IPv4 packet to the network layer. */

      ipv4_input(&priv->dev);
    }
  else
#endif
#if defined(CONFIG_NET_IPv6)
  if ((IPv6BUF->vtc & IP_VERSION_MASK) == IPv6_VERSION)
    {
      ninfo("IPv6 frame\n");
      NETDEV_RXIPV6(&priv->dev);

      /* Give the IPv6 packet to the network layer. */

      ipv6_input(&priv->dev);
    }
  else
#endif
    {
      NETDEV_RXDROPPED(&priv->dev);
      priv->dev.d_len = 0;
    }

  /* If the above function invocation resulted in data that should be
   * sent out on the network, the field  d_len will set to a value > 0.
   */

  if (priv->dev.d_len > 0)
    {
      priv->write_d_len = priv->dev.d_len;
      tun_fd_transmit(priv);
    }
}

/****************************************************************************
 * Name: tun_txdone
 *
 * Description:
 *   An interrupt was received indicating that the last TX packet(s) is done
 *
 * Input Parameters:
 *   dev - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void tun_txdone(FAR struct tun_device_s *priv)
{
  /* Check for errors and update statistics */

  NETDEV_TXDONE(&priv->dev);

  /* Then poll the network for new XMIT data */

  priv->dev.d_buf = priv->read_buf;
  devif_poll(&priv->dev, tun_txpoll);
}

/****************************************************************************
 * Name: tun_poll_work
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
 *   Ethernet interrupts are disabled
 *
 ****************************************************************************/

static void tun_poll_work(FAR void *arg)
{
  FAR struct tun_device_s *priv = (FAR struct tun_device_s *)arg;
  int ret;

  /* Perform the poll */

  ret = tun_lock(priv);
  if (ret < 0)
    {
      /* This would indicate that the worker thread was canceled.. not a
       * likely event.
       */

      DEBUGASSERT(ret == -ECANCELED);
      return;
    }

  net_lock();

  /* Check if there is room in the send another TX packet.  We cannot perform
   * the TX poll if he are unable to accept another packet for transmission.
   */

  if (priv->read_d_len == 0)
    {
      /* If so, poll the network for new XMIT data. */

      priv->dev.d_buf = priv->read_buf;
      devif_timer(&priv->dev, TUN_WDDELAY, tun_txpoll);
    }

  /* Setup the watchdog poll timer again */

  wd_start(priv->txpoll, TUN_WDDELAY, tun_poll_expiry, 1, priv);

  net_unlock();
  tun_unlock(priv);
}

/****************************************************************************
 * Name: tun_poll_expiry
 *
 * Description:
 *   Periodic timer handler.  Called from the timer interrupt handler.
 *
 * Input Parameters:
 *   argc - The number of available arguments
 *   arg  - The first argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by the watchdog logic.
 *
 ****************************************************************************/

static void tun_poll_expiry(int argc, wdparm_t arg, ...)
{
  FAR struct tun_device_s *priv = (FAR struct tun_device_s *)arg;

  /* Schedule to perform the timer expiration on the worker thread. */

  work_queue(TUNWORK, &priv->work, tun_poll_work, priv, 0);
}

/****************************************************************************
 * Name: tun_ifup
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

static int tun_ifup(FAR struct net_driver_s *dev)
{
  FAR struct tun_device_s *priv = (FAR struct tun_device_s *)dev->d_private;

#ifdef CONFIG_NET_IPv4
  ninfo("Bringing up: %d.%d.%d.%d\n",
        dev->d_ipaddr & 0xff, (dev->d_ipaddr >> 8) & 0xff,
        (dev->d_ipaddr >> 16) & 0xff, dev->d_ipaddr >> 24);
#endif
#ifdef CONFIG_NET_IPv6
  ninfo("Bringing up: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        dev->d_ipv6addr[0], dev->d_ipv6addr[1], dev->d_ipv6addr[2],
        dev->d_ipv6addr[3], dev->d_ipv6addr[4], dev->d_ipv6addr[5],
        dev->d_ipv6addr[6], dev->d_ipv6addr[7]);
#endif

  /* Set and activate a timer process */

  wd_start(priv->txpoll, TUN_WDDELAY, tun_poll_expiry,
           1, (wdparm_t)priv);

  priv->bifup = true;
  return OK;
}

/****************************************************************************
 * Name: tun_ifdown
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

static int tun_ifdown(FAR struct net_driver_s *dev)
{
  FAR struct tun_device_s *priv = (FAR struct tun_device_s *)dev->d_private;
  irqstate_t flags;

  flags = enter_critical_section();

  /* Cancel the TX poll timer */

  wd_cancel(priv->txpoll);

  /* Mark the device "down" */

  priv->bifup = false;

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: tun_txavail_work
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

static void tun_txavail_work(FAR void *arg)
{
  FAR struct tun_device_s *priv = (FAR struct tun_device_s *)arg;
  int ret;

  ret = tun_lock(priv);
  if (ret < 0)
    {
      /* Thread has been canceled, skip poll-related work */

      return;
    }

  /* Check if there is room to hold another network packet. */

  if (priv->read_d_len != 0)
    {
      tun_unlock(priv);
      return;
    }

  net_lock();
  if (priv->bifup)
    {
      /* Poll the network for new XMIT data */

      priv->dev.d_buf = priv->read_buf;
      devif_poll(&priv->dev, tun_txpoll);
    }

  net_unlock();
  tun_unlock(priv);
}

/****************************************************************************
 * Name: tun_txavail
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
 *   Called from the network stack with the network locked.
 *
 ****************************************************************************/

static int tun_txavail(FAR struct net_driver_s *dev)
{
  FAR struct tun_device_s *priv = (FAR struct tun_device_s *)dev->d_private;

  /* Schedule to perform the TX poll on the worker thread. */

  if (work_available(&priv->work))
    {
      work_queue(TUNWORK, &priv->work, tun_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: tun_addmac
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
static int tun_addmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac)
{
  /* Add the MAC address to the hardware multicast routing table */

  return OK;
}
#endif

/****************************************************************************
 * Name: tun_rmmac
 *
 * Description:
 *   NuttX Callback: Remove the specified MAC address from the hardware
 *   multicast address filtering
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
static int tun_rmmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac)
{
  /* Add the MAC address to the hardware multicast routing table */

  return OK;
}
#endif

/****************************************************************************
 * Name: tun_dev_init
 *
 * Description:
 *   Initialize the TUN device
 *
 * Input Parameters:
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int tun_dev_init(FAR struct tun_device_s *priv,
                        FAR struct file *filep,
                        FAR const char *devfmt, bool tun)
{
  int ret;

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct tun_device_s));
  priv->dev.d_ifup    = tun_ifup;     /* I/F up (new IP address) callback */
  priv->dev.d_ifdown  = tun_ifdown;   /* I/F down callback */
  priv->dev.d_txavail = tun_txavail;  /* New TX data callback */
#ifdef CONFIG_NET_MCASTGROUP
  priv->dev.d_addmac  = tun_addmac;   /* Add multicast MAC address */
  priv->dev.d_rmmac   = tun_rmmac;    /* Remove multicast MAC address */
#endif
  priv->dev.d_private = (FAR void *)priv; /* Used to recover private state from dev */

  /* Initialize the mutual exlcusion and wait semaphore */

  nxsem_init(&priv->waitsem, 0, 1);
  nxsem_init(&priv->read_wait_sem, 0, 0);
  nxsem_init(&priv->write_wait_sem, 0, 0);

  /* The wait semaphore is used for signaling and, hence, should not have
   * priority inheritance enabled.
   */

  nxsem_setprotocol(&priv->read_wait_sem, SEM_PRIO_NONE);
  nxsem_setprotocol(&priv->write_wait_sem, SEM_PRIO_NONE);

  /* Create a watchdog for timing polling for and timing of transmissions */

  priv->txpoll = wd_create(); /* Create periodic poll timer */

  /* Assign d_ifname if specified. */

  if (devfmt)
    {
      strncpy(priv->dev.d_ifname, devfmt, IFNAMSIZ);
    }

  /* Register the device with the OS so that socket IOCTLs can be performed */

  ret = netdev_register(&priv->dev, tun ? NET_LL_TUN : NET_LL_ETHERNET);
  if (ret != OK)
    {
      nxsem_destroy(&priv->waitsem);
      nxsem_destroy(&priv->read_wait_sem);
      nxsem_destroy(&priv->write_wait_sem);
      return ret;
    }

  filep->f_priv = priv; /* Set link to TUN device */
  return ret;
}

/****************************************************************************
 * Name: tun_dev_uninit
 ****************************************************************************/

static void tun_dev_uninit(FAR struct tun_device_s *priv)
{
  /* Put the interface in the down state */

  tun_ifdown(&priv->dev);

  /* Remove the device from the OS */

  netdev_unregister(&priv->dev);

  nxsem_destroy(&priv->waitsem);
  nxsem_destroy(&priv->read_wait_sem);
  nxsem_destroy(&priv->write_wait_sem);
}

/****************************************************************************
 * Name: tun_open
 ****************************************************************************/

static int tun_open(FAR struct file *filep)
{
  filep->f_priv = 0;
  return OK;
}

/****************************************************************************
 * Name: tun_close
 ****************************************************************************/

static int tun_close(FAR struct file *filep)
{
  FAR struct inode *inode       = filep->f_inode;
  FAR struct tun_driver_s *tun  = inode->i_private;
  FAR struct tun_device_s *priv = filep->f_priv;
  int intf;
  int ret;

  if (priv == NULL)
    {
      return OK;
    }

  intf = priv - g_tun_devices;
  ret  = tundev_lock(tun);
  if (ret >= 0)
    {
      tun->free_tuns |= (1 << intf);
      tun_dev_uninit(priv);

      tundev_unlock(tun);
    }

  return ret;
}

/****************************************************************************
 * Name: tun_write
 ****************************************************************************/

static ssize_t tun_write(FAR struct file *filep, FAR const char *buffer,
                         size_t buflen)
{
  FAR struct tun_device_s *priv = filep->f_priv;
  ssize_t nwritten = 0;
  int ret;

  if (priv == NULL || buflen > CONFIG_NET_TUN_PKTSIZE)
    {
      return -EINVAL;
    }

  for (; ; )
    {
      /* Write must return immediately if interrupted by a signal (or if the
       * thread is canceled) and no data has yet been written.
       */

      ret = nxsem_wait(&priv->waitsem);
      if (ret < 0)
        {
          return nwritten == 0 ? (ssize_t)ret : nwritten;
        }

      /* Check if there are free space to write */

      if (priv->write_d_len == 0)
        {
          memcpy(priv->write_buf, buffer, buflen);

          net_lock();
          priv->dev.d_buf = priv->write_buf;
          priv->dev.d_len = buflen;

          tun_net_receive(priv);
          net_unlock();

          nwritten = buflen;
          break;
        }

      /* Wait if there are no free space to write */

      if ((filep->f_oflags & O_NONBLOCK) != 0)
        {
          nwritten = -EAGAIN;
          break;
        }

      priv->write_wait = true;
      tun_unlock(priv);
      nxsem_wait(&priv->write_wait_sem);
    }

  tun_unlock(priv);
  return nwritten;
}

/****************************************************************************
 * Name: tun_read
 ****************************************************************************/

static ssize_t tun_read(FAR struct file *filep, FAR char *buffer,
                        size_t buflen)
{
  FAR struct tun_device_s *priv = filep->f_priv;
  ssize_t nread = 0;
  int ret;

  if (priv == NULL)
    {
      return -EINVAL;
    }

  for (; ; )
    {
      /* Read must return immediately if interrupted by a signal (or if the
       * thread is canceled) and no data has yet been read.
       */

      ret = nxsem_wait(&priv->waitsem);
      if (ret < 0)
        {
          return nread == 0 ? (ssize_t)ret : nread;
        }

      /* Check if there are data to read in write buffer */

      if (priv->write_d_len > 0)
        {
          if (buflen < priv->write_d_len)
            {
              nread = -EINVAL;
              break;
            }

          memcpy(buffer, priv->write_buf, priv->write_d_len);
          nread = priv->write_d_len;
          priv->write_d_len = 0;

          NETDEV_TXDONE(&priv->dev);
          tun_pollnotify(priv, POLLOUT);
          break;
        }

      /* Check if there are data to read in read buffer */

      if (priv->read_d_len > 0)
        {
          if (buflen < priv->read_d_len)
            {
              nread = -EINVAL;
              break;
            }

          memcpy(buffer, priv->read_buf, priv->read_d_len);
          nread = priv->read_d_len;
          priv->read_d_len = 0;

          net_lock();
          tun_txdone(priv);
          net_unlock();
          break;
        }

      /* Wait if there are no data to read */

      if ((filep->f_oflags & O_NONBLOCK) != 0)
        {
          nread = -EAGAIN;
          break;
        }

      priv->read_wait = true;
      tun_unlock(priv);
      nxsem_wait(&priv->read_wait_sem);
    }

  tun_unlock(priv);
  return nread;
}

/****************************************************************************
 * Name: tun_poll
 ****************************************************************************/

int tun_poll(FAR struct file *filep, FAR struct pollfd *fds, bool setup)
{
  FAR struct tun_device_s *priv = filep->f_priv;
  pollevent_t eventset;
  int ret;

  /* Some sanity checking */

  if (priv == NULL || fds == NULL)
    {
      return -EINVAL;
    }

  ret = tun_lock(priv);
  if (ret < 0)
    {
      return ret;
    }

  if (setup)
    {
      if (priv->poll_fds)
        {
          ret = -EBUSY;
          goto errout;
        }

      priv->poll_fds = fds;

      eventset = 0;

      /* If write buffer is empty notify App.  */

      if (priv->write_d_len == 0)
        {
          eventset |= (fds->events & POLLOUT);
        }

      /* The write buffer sometimes could be used for TX.
       * So check it too.
       */

      if (priv->read_d_len != 0 || priv->write_d_len != 0)
        {
          eventset |= (fds->events & POLLIN);
        }

      if (eventset)
        {
          tun_pollnotify(priv, eventset);
        }
    }
  else
    {
      priv->poll_fds = 0;
    }

errout:
  tun_unlock(priv);
  return ret;
}

/****************************************************************************
 * Name: tun_ioctl
 ****************************************************************************/

static int tun_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode       = filep->f_inode;
  FAR struct tun_driver_s *tun  = inode->i_private;
  FAR struct tun_device_s *priv = filep->f_priv;
  int ret = OK;

  if (cmd == TUNSETIFF && priv == NULL)
    {
      uint8_t free_tuns;
      int intf;
      FAR struct ifreq *ifr = (FAR struct ifreq *)arg;

      if (ifr  == NULL ||
         ((ifr->ifr_flags & IFF_MASK) != IFF_TUN &&
          (ifr->ifr_flags & IFF_MASK) != IFF_TAP))
        {
          return -EINVAL;
        }

      ret = tundev_lock(tun);
      if (ret < 0)
        {
          return ret;
        }

      free_tuns = tun->free_tuns;

      if (free_tuns == 0)
        {
          tundev_unlock(tun);
          return -ENOMEM;
        }

      for (intf = 0;
           intf < CONFIG_TUN_NINTERFACES && !(free_tuns & 1);
           intf++, free_tuns >>= 1);

      ret = tun_dev_init(&g_tun_devices[intf], filep,
                         *ifr->ifr_name ? ifr->ifr_name : 0,
                         (ifr->ifr_flags & IFF_MASK) == IFF_TUN);
      if (ret != OK)
        {
          tundev_unlock(tun);
          return ret;
        }

      tun->free_tuns &= ~(1 << intf);

      priv = filep->f_priv;
      strncpy(ifr->ifr_name, priv->dev.d_ifname, IFNAMSIZ);
      tundev_unlock(tun);

      return OK;
    }

  return -EBADFD;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tun_initialize
 *
 * Description:
 *   Instantiate a TUN network interface.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int tun_initialize(void)
{
  nxsem_init(&g_tun.waitsem, 0, 1);

  g_tun.free_tuns = (1 << CONFIG_TUN_NINTERFACES) - 1;

  register_driver("/dev/tun", &g_tun_file_ops, 0644, &g_tun);
  return OK;
}

#endif /* CONFIG_NET && CONFIG_NET_TUN */
