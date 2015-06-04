/****************************************************************************
 * drivers/net/tun.c
 *
 *   Copyright (C) 2015 Max Nekludov. All rights reserved.
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_TUN)

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <fcntl.h>
#include <debug.h>

#ifndef CONFIG_DISABLE_POLL
#  include <poll.h>
#endif

#include <arpa/inet.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/wdog.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/netdev.h>

#include <net/if.h>
#include <nuttx/net/tun.h>

#ifdef CONFIG_NET_NOINTS
#  include <nuttx/wqueue.h>
#endif

#ifdef CONFIG_NET_PKT
#  include <nuttx/net/pkt.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* If processing is not done at the interrupt level, then high priority
 * work queue support is required.
 */

#if defined(CONFIG_NET_NOINTS) && !defined(CONFIG_SCHED_HPWORK)
#  error High priority work queue support is required
#endif

/* CONFIG_TUN_NINTERFACES determines the number of physical interfaces
 * that will be supported.
 */

#ifndef CONFIG_TUN_NINTERFACES
# define CONFIG_TUN_NINTERFACES 1
#endif

/* TX poll delay = 1 seconds. CLK_TCK is the number of clock ticks per
 * second
 */

#define TUN_WDDELAY   (1*CLK_TCK)
#define TUN_POLLHSEC  (1*2)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The tun_device_s encapsulates all state information for a single hardware
 * interface
 */

struct tun_device_s
{
  bool              bifup;   /* true:ifup false:ifdown */
  WDOG_ID           txpoll;  /* TX poll timer */
#ifdef CONFIG_NET_NOINTS
  struct work_s     work;    /* For deferring work to the work queue */
#endif

  FAR struct file  *filep;

#ifndef CONFIG_DISABLE_POLL
  FAR struct pollfd *poll_fds;
#endif

  bool              read_wait;

  uint8_t           read_buf[CONFIG_NET_TUN_MTU];
  size_t            read_d_len;
  uint8_t           write_buf[CONFIG_NET_TUN_MTU];
  size_t            write_d_len;

  sem_t             waitsem;
  sem_t             read_wait_sem;

  /* This holds the information visible to uIP/NuttX */

  struct net_driver_s dev;     /* Interface understood by uIP */
};

struct tun_driver_s
{
  uint8_t free_tuns;
  sem_t waitsem;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void tun_lock(FAR struct tun_device_s *priv);
static void tun_unlock(FAR struct tun_device_s *priv);

/* Common TX logic */

static int  tun_transmit(FAR struct tun_device_s *priv);
static int  tun_txpoll(struct net_driver_s *dev);

/* Interrupt handling */

static void tun_receive(FAR struct tun_device_s *priv);
static void tun_txdone(FAR struct tun_device_s *priv);

/* Watchdog timer expirations */

static inline void tun_poll_process(FAR struct tun_device_s *priv);
#ifdef CONFIG_NET_NOINTS
static void tun_poll_work(FAR void *arg);
#endif
static void tun_poll_expiry(int argc, wdparm_t arg, ...);

/* NuttX callback functions */

static int tun_ifup(FAR struct net_driver_s *dev);
static int tun_ifdown(FAR struct net_driver_s *dev);
static int tun_txavail(FAR struct net_driver_s *dev);
#if defined(CONFIG_NET_IGMP) || defined(CONFIG_NET_ICMPv6)
static int tun_addmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac);
#endif
#ifdef CONFIG_NET_IGMP
static int tun_rmmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac);
#endif
#ifdef CONFIG_NET_ICMPv6
static void tun_ipv6multicast(FAR struct tun_device_s *priv);
#endif

static int tun_dev_init(FAR struct tun_device_s *priv,
                        FAR struct file *filep, FAR const char* devfmt);
static int tun_dev_uninit(FAR struct tun_device_s *priv);

/* File interface */

static int tun_open(FAR struct file *filep);
static int tun_close(FAR struct file *filep);
static ssize_t tun_read(FAR struct file *filep, FAR char *buffer,
                        size_t buflen);
static ssize_t tun_write(FAR struct file *filep, FAR const char *buffer,
                         size_t buflen);
static int tun_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
#ifndef CONFIG_DISABLE_POLL
static int tun_poll(FAR struct file *filep, FAR struct pollfd *fds,
                    bool setup);
#endif

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
  0,                 /* seek */
  tun_ioctl,    /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  tun_poll,     /* poll */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tundev_lock
 ****************************************************************************/

static void tundev_lock(FAR struct tun_driver_s *tun)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(&tun->waitsem) != 0)
    {
      /* The only case that an error should occur here is if
       * the wait was awakened by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

/****************************************************************************
 * Name: tundev_unlock
 ****************************************************************************/

static void tundev_unlock(FAR struct tun_driver_s *tun)
{
  sem_post(&tun->waitsem);
}

/****************************************************************************
 * Name: tun_lock
 ****************************************************************************/

static void tun_lock(FAR struct tun_device_s *priv)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(&priv->waitsem) != 0)
    {
      /* The only case that an error should occur here is if
       * the wait was awakened by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

/****************************************************************************
 * Name: tun_unlock
 ****************************************************************************/

static void tun_unlock(FAR struct tun_device_s *priv)
{
  sem_post(&priv->waitsem);
}

/****************************************************************************
 * Name: tun_pollnotify
 ****************************************************************************/

#ifndef CONFIG_DISABLE_POLL
static void tun_pollnotify(FAR struct tun_device_s *priv, pollevent_t eventset)
{
  FAR struct pollfd *fds = priv->poll_fds;

  if (fds == NULL)
    {
      return;
    }

  eventset &= fds->events;

  if (eventset != 0)
    {
      fds->revents |= eventset;
      //fvdbg("Report events: %02x\n", fds->revents);
      sem_post(fds->sem);
    }
}
#else
#  define tun_pollnotify(dev, event)
#endif

/****************************************************************************
 * Function: tun_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 * Parameters:
 *   priv - Reference to the driver state structure
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

static int tun_transmit(FAR struct tun_device_s *priv)
{
  /* Verify that the hardware is ready to send another packet.  If we get
   * here, then we are committed to sending a packet; Higher level logic
   * must have assured that there is no transmission in progress.
   */

  if (priv->read_wait)
    {
      priv->read_wait = false;
      sem_post(&priv->read_wait_sem);
    }

  tun_pollnotify(priv, POLLIN);
  return OK;
}

/****************************************************************************
 * Function: tun_txpoll
 *
 * Description:
 *   The transmitter is available, check if uIP has any outgoing packets
 *   ready to send.  This is a callback from devif_poll().  devif_poll() may
 *   be called:
 *
 *   1. When the preceding TX packet send is complete,
 *   2. When the preceding TX packet send timesout and the interface is reset
 *   3. During normal TX polling
 *
 * Parameters:
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

static int tun_txpoll(struct net_driver_s *dev)
{
  FAR struct tun_device_s *priv = (FAR struct tun_device_s *)dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (priv->dev.d_len > 0)
    {
      /* Send the packet */

      tun_transmit(priv);

      return 1;
    }

  /* If zero is returned, the polling will continue until all connections have
   * been examined.
   */

  return 0;
}

/****************************************************************************
 * Function: tun_receive
 *
 * Description:
 *   An interrupt was received indicating the availability of a new RX packet
 *
 * Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void tun_receive(FAR struct tun_device_s *priv)
{
  /* Copy the data data from the hardware to priv->dev.d_buf.  Set amount of
   * data in priv->dev.d_len
   */

#ifdef CONFIG_NET_PKT
  /* When packet sockets are enabled, feed the frame into the packet tap */

  pkt_input(&priv->dev);
#endif

  /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv4
    {
      nllvdbg("IPv4 frame\n");

      /* Give the IPv4 packet to the network layer */

      ipv4_input(&priv->dev);

      /* If the above function invocation resulted in data that should be
       * sent out on the network, the field  d_len will set to a value > 0.
       */

      if (priv->dev.d_len > 0)
        {
          priv->write_d_len = priv->dev.d_len;
          tun_transmit(priv);
        }
      else
        {
          priv->write_d_len = 0;
          tun_pollnotify(priv, POLLOUT);
        }
    }
#endif

#if 0
#ifdef CONFIG_NET_IPv6
  if (BUF->type == HTONS(ETHTYPE_IP6))
    {
      nllvdbg("Iv6 frame\n");

      /* Give the IPv6 packet to the network layer */

      ipv6_input(&priv->dev);

      /* If the above function invocation resulted in data that should be
        * sent out on the network, the field  d_len will set to a value > 0.
        */

      if (priv->dev.d_len > 0)
        {
          /* Update the Ethernet header with the correct MAC address */

#ifdef CONFIG_NET_IPv4
          if (IFF_IS_IPv4(priv->dev.d_flags))
            {
              arp_out(&priv->dev);
            }
          else
#endif
#ifdef CONFIG_NET_IPv6
            {
              neighbor_out(&priv->dev);
            }
#endif

          /* And send the packet */

          tun_transmit(priv);
        }
    }
#endif
#endif
}

/****************************************************************************
 * Function: tun_txdone
 *
 * Description:
 *   An interrupt was received indicating that the last TX packet(s) is done
 *
 * Parameters:
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

  /* Then poll uIP for new XMIT data */

  priv->dev.d_buf = priv->read_buf;
  (void)devif_poll(&priv->dev, tun_txpoll);
  priv->read_d_len = priv->dev.d_len;
}

/****************************************************************************
 * Function: tun_poll_process
 *
 * Description:
 *   Perform the periodic poll.  This may be called either from watchdog
 *   timer logic or from the worker thread, depending upon the configuration.
 *
 * Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static void tun_poll_process(FAR struct tun_device_s *priv)
{
  /* Check if there is room in the send another TX packet.  We cannot perform
   * the TX poll if he are unable to accept another packet for transmission.
   */

  /* If so, update TCP timing states and poll uIP for new XMIT data. Hmmm..
   * might be bug here.  Does this mean if there is a transmit in progress,
   * we will missing TCP time state updates?
   */

  priv->dev.d_buf = priv->read_buf;
  (void)devif_timer(&priv->dev, tun_txpoll, TUN_POLLHSEC);
  priv->read_d_len = priv->dev.d_len;

  /* Setup the watchdog poll timer again */

  (void)wd_start(priv->txpoll, TUN_WDDELAY, tun_poll_expiry, 1, priv);
}

/****************************************************************************
 * Function: tun_poll_work
 *
 * Description:
 *   Perform periodic polling from the worker thread
 *
 * Parameters:
 *   arg - The argument passed when work_queue() as called.
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *   Ethernet interrupts are disabled
 *
 ****************************************************************************/

#ifdef CONFIG_NET_NOINTS
static void tun_poll_work(FAR void *arg)
{
  FAR struct tun_device_s *priv = (FAR struct tun_device_s *)arg;
  net_lock_t state;

  /* Perform the poll */

  state = net_lock();
  tun_poll_process(priv);
  net_unlock(state);
}
#endif

/****************************************************************************
 * Function: tun_poll_expiry
 *
 * Description:
 *   Periodic timer handler.  Called from the timer interrupt handler.
 *
 * Parameters:
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

#ifdef CONFIG_NET_NOINTS
  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions.
   */

  if (work_available(&priv->work))
    {
      /* Schedule to perform the interrupt processing on the worker thread. */

      work_queue(HPWORK, &priv->work, tun_poll_work, priv, 0);
    }
  else
    {
      /* No.. Just re-start the watchdog poll timer, missing one polling
       * cycle.
       */

      (void)wd_start(priv->txpoll, TUN_WDDELAY, tun_poll_expiry, 1, arg);
    }

#else
  /* Process the interrupt now */

  tun_poll_process(priv);
#endif
}

/****************************************************************************
 * Function: tun_ifup
 *
 * Description:
 *   NuttX Callback: Bring up the Ethernet interface when an IP address is
 *   provided
 *
 * Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

static int tun_ifup(struct net_driver_s *dev)
{
  FAR struct tun_device_s *priv = (FAR struct tun_device_s *)dev->d_private;

#ifdef CONFIG_NET_IPv4
  ndbg("Bringing up: %d.%d.%d.%d\n",
       dev->d_ipaddr & 0xff, (dev->d_ipaddr >> 8) & 0xff,
       (dev->d_ipaddr >> 16) & 0xff, dev->d_ipaddr >> 24);
#endif
#ifdef CONFIG_NET_IPv6
  ndbg("Bringing up: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
       dev->d_ipv6addr[0], dev->d_ipv6addr[1], dev->d_ipv6addr[2],
       dev->d_ipv6addr[3], dev->d_ipv6addr[4], dev->d_ipv6addr[5],
       dev->d_ipv6addr[6], dev->d_ipv6addr[7]);
#endif

  /* Initialize PHYs, the Ethernet interface, and setup up Ethernet interrupts */

  /* Instantiate the MAC address from priv->dev.d_mac.ether_addr_octet */

#ifdef CONFIG_NET_ICMPv6
  /* Set up IPv6 multicast address filtering */

  tun_ipv6multicast(priv);
#endif

  /* Set and activate a timer process */

  (void)wd_start(priv->txpoll, TUN_WDDELAY, tun_poll_expiry, 1, (wdparm_t)priv);

  priv->bifup = true;
  return OK;
}

/****************************************************************************
 * Function: tun_ifdown
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
 *
 ****************************************************************************/

static int tun_ifdown(struct net_driver_s *dev)
{
  FAR struct tun_device_s *priv = (FAR struct tun_device_s *)dev->d_private;

  tun_lock(priv);

  /* Cancel the TX poll timer */

  wd_cancel(priv->txpoll);

  /* Mark the device "down" */

  priv->bifup = false;
  tun_unlock(priv);
  return OK;
}

/****************************************************************************
 * Function: tun_txavail
 *
 * Description:
 *   Driver callback invoked when new TX data is available.  This is a
 *   stimulus perform an out-of-cycle poll and, thereby, reduce the TX
 *   latency.
 *
 * Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called in normal user mode
 *
 ****************************************************************************/

static int tun_txavail(struct net_driver_s *dev)
{
  FAR struct tun_device_s *priv = (FAR struct tun_device_s *)dev->d_private;
  net_lock_t state;

  tun_lock(priv);

  /* Check if there is room to hold another network packet. */

  if (priv->read_d_len)
    {
      tun_unlock(priv);
      return OK;
    }

  state = net_lock();

  if (priv->bifup)
    {
      /* Poll uIP for new XMIT data */

      priv->dev.d_buf = priv->read_buf;
      (void)devif_poll(&priv->dev, tun_txpoll);
      priv->read_d_len = priv->dev.d_len;
    }

  net_unlock(state);
  tun_unlock(priv);

  return OK;
}

/****************************************************************************
 * Function: tun_addmac
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
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#if defined(CONFIG_NET_IGMP) || defined(CONFIG_NET_ICMPv6)
static int tun_addmac(struct net_driver_s *dev, FAR const uint8_t *mac)
{
  //FAR struct tun_device_s *priv = (FAR struct tun_device_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  return OK;
}
#endif

/****************************************************************************
 * Function: tun_rmmac
 *
 * Description:
 *   NuttX Callback: Remove the specified MAC address from the hardware multicast
 *   address filtering
 *
 * Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be removed
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IGMP
static int tun_rmmac(struct net_driver_s *dev, FAR const uint8_t *mac)
{
  //FAR struct tun_device_s *priv = (FAR struct tun_device_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  return OK;
}
#endif

/****************************************************************************
 * Function: tun_ipv6multicast
 *
 * Description:
 *   Configure the IPv6 multicast MAC address.
 *
 * Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6
static void tun_ipv6multicast(FAR struct tun_device_s *priv)
{
}
#endif /* CONFIG_NET_ICMPv6 */

/****************************************************************************
 * Function: tun_dev_init
 *
 * Description:
 *   Initialize the TUN device
 *
 * Parameters:
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

static int tun_dev_init(FAR struct tun_device_s *priv, FAR struct file *filep,
                        FAR const char* devfmt)
{
  int ret;

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct tun_device_s));
  priv->dev.d_ifup    = tun_ifup;     /* I/F up (new IP address) callback */
  priv->dev.d_ifdown  = tun_ifdown;   /* I/F down callback */
  priv->dev.d_txavail = tun_txavail;  /* New TX data callback */
#ifdef CONFIG_NET_IGMP
  priv->dev.d_addmac  = tun_addmac;   /* Add multicast MAC address */
  priv->dev.d_rmmac   = tun_rmmac;    /* Remove multicast MAC address */
#endif
  priv->dev.d_private = (void*)priv;   /* Used to recover private state from dev */

  /* Initialize the wait semaphore */

  sem_init(&priv->waitsem, 0, 1);
  sem_init(&priv->read_wait_sem, 0, 0);

  /* Create a watchdog for timing polling for and timing of transmisstions */

  priv->txpoll        = wd_create();  /* Create periodic poll timer */

  /* Initialize other variables */

  priv->write_d_len   = 0;
  priv->read_wait     = false;

  /* Put the interface in the down state */

  tun_ifdown(&priv->dev);

  if (devfmt)
    {
      strncpy(priv->dev.d_ifname, devfmt, IFNAMSIZ);
    }

  /* Register the device with the OS so that socket IOCTLs can be performed */

  ret = netdev_register(&priv->dev, NET_LL_TUN);
  if (ret != OK)
    {
      sem_destroy(&priv->waitsem);
      sem_destroy(&priv->read_wait_sem);
      return ret;
    }

  priv->filep         = filep;        /* Set link to file */
  filep->f_priv       = priv;         /* Set link to TUN device */

  return ret;
}

/****************************************************************************
 * Name: tun_dev_uninit
 ****************************************************************************/

static int tun_dev_uninit(FAR struct tun_device_s *priv)
{
  /* Put the interface in the down state */

  tun_ifdown(&priv->dev);

  /* Remove the device from the OS */

  (void)netdev_unregister(&priv->dev);

  sem_destroy(&priv->waitsem);
  sem_destroy(&priv->read_wait_sem);

  return OK;
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

  if (!priv)
    {
      return OK;
    }

  intf = priv - g_tun_devices;
  tundev_lock(tun);

  tun->free_tuns |= (1 << intf);
  (void)tun_dev_uninit(priv);

  tundev_unlock(tun);

  return OK;
}

/****************************************************************************
 * Name: tun_write
 ****************************************************************************/

static ssize_t tun_write(FAR struct file *filep, FAR const char *buffer,
                         size_t buflen)
{
  FAR struct tun_device_s *priv = filep->f_priv;
  net_lock_t state;
  ssize_t ret;

  if (!priv)
    {
      return -EINVAL;
    }

  tun_lock(priv);

  if (priv->write_d_len > 0)
    {
      tun_unlock(priv);
      return -EBUSY;
    }

  state = net_lock();

  if (buflen > CONFIG_NET_TUN_MTU)
    {
      ret = -EINVAL;
    }
  else
    {
      memcpy(priv->write_buf, buffer, buflen);

      priv->dev.d_buf = priv->write_buf;
      priv->dev.d_len = buflen;

      tun_receive(priv);

      ret = (ssize_t)buflen;
    }

  net_unlock(state);
  tun_unlock(priv);

  return ret;
}

/****************************************************************************
 * Name: tun_read
 ****************************************************************************/

static ssize_t tun_read(FAR struct file *filep, FAR char *buffer,
                        size_t buflen)
{
  FAR struct tun_device_s *priv = filep->f_priv;
  net_lock_t state;
  ssize_t ret;
  size_t write_d_len;
  size_t read_d_len;

  if (!priv)
    {
      return -EINVAL;
    }

  tun_lock(priv);

  /* Check if there are data to read in write buffer */

  write_d_len = priv->write_d_len;
  if (write_d_len > 0)
    {
      if (buflen < write_d_len)
        {
          ret = -EINVAL;
          goto out;
        }

      memcpy(buffer, priv->write_buf, write_d_len);
      ret = (ssize_t)write_d_len;

      priv->write_d_len = 0;
      tun_pollnotify(priv, POLLOUT);
      goto out;
    }

  if (priv->read_d_len == 0)
    {
      if ((filep->f_oflags & O_NONBLOCK) != 0)
        {
          ret = -EAGAIN;
          goto out;
        }

      priv->read_wait = true;
      tun_unlock(priv);
      sem_wait(&priv->read_wait_sem);
      tun_lock(priv);
    }

  state = net_lock();

  read_d_len = priv->read_d_len;
  if (buflen < read_d_len)
    {
      ret = -EINVAL;
    }
  else
    {
      memcpy(buffer, priv->read_buf, read_d_len);
      ret = (ssize_t)read_d_len;
    }

  priv->read_d_len = 0;
  tun_txdone(priv);

  net_unlock(state);

out:
  tun_unlock(priv);

  return ret;
}

/****************************************************************************
 * Name: tun_poll
 ****************************************************************************/

#ifndef CONFIG_DISABLE_POLL
int tun_poll(FAR struct file *filep, FAR struct pollfd *fds, bool setup)
{
  FAR struct tun_device_s *priv = filep->f_priv;
  pollevent_t eventset;
  int ret = OK;
  
  if (!priv)
    {
      return -EINVAL;
    }

  /* Some sanity checking */

  if (!priv || !fds)
    {
      return -ENODEV;
    }

  tun_lock(priv);

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
#endif

/****************************************************************************
 * Name: tun_ioctl
 ****************************************************************************/

static int tun_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode      = filep->f_inode;
  FAR struct tun_driver_s *tun = inode->i_private;
  FAR struct tun_device_s *priv = filep->f_priv;
  int ret = OK;

  if (cmd == TUNSETIFF && !priv)
    {
      uint8_t free_tuns;
      int intf;
      FAR struct ifreq *ifr = (FAR struct ifreq*)arg;

      if (!ifr || ifr->ifr_flags != IFF_TUN)
        {
          return -EINVAL;
        }

      tundev_lock(tun);

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
                         *ifr->ifr_name ? ifr->ifr_name : 0);
      if (ret != OK)
        {
          tundev_unlock(tun);
          return ret;
        }

      tun->free_tuns &= ~(1 << intf);

      priv = filep->f_priv;
      strncpy(ifr->ifr_name, priv->dev.d_ifname, IFNAMSIZ);
      lldbg("--- %s\n", priv->dev.d_ifname);

      tundev_unlock(tun);

      return OK;
    }

  return -EBADFD;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: tun_initialize
 *
 * Description:
 *   Instantiate a SLIP network interface.
 *
 * Parameters:
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int tun_initialize(void)
{
  sem_init(&g_tun.waitsem, 0, 1);

  g_tun.free_tuns = (1 << CONFIG_TUN_NINTERFACES) - 1;

  (void)register_driver("/dev/tun", &g_tun_file_ops, 0644, &g_tun);
  return OK;
}

#endif /* CONFIG_NET && CONFIG_NET_TUN */
