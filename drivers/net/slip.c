/****************************************************************************
 * drivers/net/slip.c
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

/* Reference: RFC 1055 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <fcntl.h>
#include <poll.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arpa/inet.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/signal.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/netdev.h>

#ifdef CONFIG_NET_SLIP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* NOTE:  Slip requires UART hardware handshake.  If hardware handshake is
 * not available with your UART, then you might try the 'slattach' option
 * -L which enable "3-wire operation."  That allows operation without the
 * hardware handshake (but with the possibility of data overrun).
 */

/* Configuration ************************************************************/

/* The Linux slip module hard-codes its MTU size to 296 (40 bytes for the
 * IP+TCP headers plus 256 bytes of data).  So you might as well set
 * CONFIG_NET_SLIP_PKTSIZE to 296 as well.
 *
 * There may be an issue with this setting, however.  I see that Linux uses
 * a MTU of 296 and window of 256, but actually only sends 168 bytes of data:
 * 40 + 128.  I believe that is to allow for the 2x worst cast packet
 * expansion.  Ideally we would like to advertise the 256 MSS, but restrict
 * transfers to 128 bytes (possibly by modifying the tcp_mss() macro).
 */

#if CONFIG_NET_SLIP_PKTSIZE < 296
#  error "CONFIG_NET_SLIP_PKTSIZE >= 296 is required"
#endif

/* Work queue support is required. */

#if !defined(CONFIG_SCHED_WORKQUEUE)
#  error Work queue support is required in this configuration (CONFIG_SCHED_WORKQUEUE)
#else

/* The low priority work queue is preferred.  If it is not enabled, LPWORK
 * will be the same as HPWORK.
 *
 * NOTE:  However, the network should NEVER run on the high priority work
 * queue!  That queue is intended only to service short back end interrupt
 * processing that never suspends.  Suspending the high priority work queue
 * may bring the system to its knees!
 */

#define SLIPWORK LPWORK

/* CONFIG_NET_SLIP_NINTERFACES determines the number of
 * physical interfaces that will be supported.
 */

#ifndef CONFIG_NET_SLIP_NINTERFACES
#  define CONFIG_NET_SLIP_NINTERFACES 1
#endif

/*  SLIP special character codes ********************************************/

#define SLIP_END      0300    /* Indicates end of packet */
#define SLIP_ESC      0333    /* Indicates byte stuffing */
#define SLIP_ESC_END  0334    /* ESC ESC_END means SLIP_END data byte */
#define SLIP_ESC_ESC  0335    /* ESC ESC_ESC means ESC data byte */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The slip_driver_s encapsulates all state information for a single hardware
 * interface
 */

struct slip_driver_s
{
  bool             bifup;     /* true:ifup false:ifdown */
  struct work_s    irqwork;   /* For deferring interrupt work */
  struct work_s    pollwork;  /* For deferring poll work to the work queue */
  struct file      tty;       /* TTY file */
  struct pollfd    pollfd;    /* Polling TTY for read- or writeable */

  uint8_t          rxbuf[2 * CONFIG_NET_SLIP_PKTSIZE + 2];
  size_t           rxlen;

  uint8_t          txbuf[2 * CONFIG_NET_SLIP_PKTSIZE + 2];
  size_t           txlen;
  size_t           txsent;

  /* This holds the information visible to the NuttX network */

  struct net_driver_s dev;  /* Interface understood by the network */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Driver state structure */

static struct slip_driver_s g_slip[CONFIG_NET_SLIP_NINTERFACES];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Common TX logic */

static void slip_transmit(FAR struct slip_driver_s *self);
static int  slip_txpoll(FAR struct net_driver_s *dev);

/* Interrupt handling */

static void slip_reply(FAR struct slip_driver_s *self);
static void slip_receive(FAR struct slip_driver_s *self);
static void slip_txdone(FAR struct slip_driver_s *self);

static void slip_interrupt_work(FAR void *arg);
static void slip_pollfd_cb(FAR struct pollfd *pollfd);
static void slip_set_pollfd_events(FAR struct slip_driver_s *self,
                                   short events);

/* NuttX callback functions */

static int  slip_ifup(FAR struct net_driver_s *dev);
static int  slip_ifdown(FAR struct net_driver_s *dev);

static void slip_txavail_work(FAR void *arg);
static int  slip_txavail(FAR struct net_driver_s *dev);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: slip_pollfd_cb
 *
 * Description:
 *   TTY reports to be read- or writable
 *
 * Input Parameters:
 *   pollfd  - Information about the event that happened.
 *
 * Returned Value:
 *   OK on success
 *
 ****************************************************************************/

static void slip_pollfd_cb(FAR struct pollfd *pollfd)
{
  FAR struct slip_driver_s *self = (FAR struct slip_driver_s *)pollfd->arg;

  DEBUGASSERT(self != NULL);

  /* Schedule to perform the processing on the worker thread. */

  if (work_available(&self->irqwork))
    {
      work_queue(SLIPWORK, &self->irqwork, slip_interrupt_work, self, 0);
    }
}

/****************************************************************************
 * Name: slip_set_pollfd_events
 *
 * Description:
 *   Setup TTY to report poll events (such as POLLIN and POLLOUT)
 *
 * Input Parameters:
 *   self   - The SLIP interface to register for poll events
 *   events - The poll events to request reporting for
 *
 ****************************************************************************/

static void slip_set_pollfd_events(FAR struct slip_driver_s *self,
                                   short events)
{
  int ret;

  /* Teardown any potentially pending poll, if applicable */

  if (self->pollfd.events != 0)
    {
      ret = file_poll(&self->tty, &self->pollfd, false);

      if (ret != OK)
        {
          nerr("file_poll(false) failed: %d\n", ret);
        }
    }

  memset(&self->pollfd, 0, sizeof(self->pollfd));

  /* Setup requested poll, if applicable */

  if (events != 0)
    {
      self->pollfd.arg     = self;
      self->pollfd.cb      = slip_pollfd_cb;
      self->pollfd.events  = events;
      self->pollfd.revents = 0;
      self->pollfd.priv    = NULL;

      ret = file_poll(&self->tty, &self->pollfd, true);

      if (ret != OK)
        {
          nerr("file_poll(true) failed: %d\n", ret);
        }
    }
}

/****************************************************************************
 * Name: slip_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 * Input Parameters:
 *   self - Reference to the driver state structure
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void slip_transmit(FAR struct slip_driver_s *self)
{
  ssize_t ssz;
  uint8_t *p;

  DEBUGASSERT(self->dev.d_len > 0);

  /* Verify that the hardware is ready to send another packet.  If we get
   * here, then we are committed to sending a packet; Higher level logic
   * must have assured that there is no transmission in progress.
   */

  if (self->txlen > 0)
    {
      /* Transmission of previous packet is still pending.  This might happen
       * in the 'slip_receive' -> 'slip_reply' -> 'slip_transmit' case.  Try
       * to forward pending packet into UART's transmit buffer.  Timeout on
       * packet if not forwarded within a second.
       */

      for (int i = 0; (i < 10) && (self->txsent != self->txlen); )
        {
          ssz = file_write(&self->tty,
                           &self->txbuf[self->txsent],
                           self->txlen - self->txsent);
          if (ssz <= 0)
            {
              nxsig_usleep(10000);
              i++;
              continue;
            }

          self->txsent += ssz;
        }

      if (self->txsent == self->txlen)
        {
          slip_txdone(self);
        }
      else
        {
          NETDEV_TXTIMEOUTS(&self->dev);
        }
    }

  self->txlen = 0;
  self->txsent = 0;

  /* Send an initial END character to flush out any data that may have
   * accumulated in the receiver due to line noise
   */

  self->txbuf[self->txlen++] = SLIP_END;

  /* Now copy the I/O buffer into self->txbuf */

  for (unsigned int bytesread = 0; bytesread < self->dev.d_len; )
    {
      unsigned int chunk_sz = sizeof(self->txbuf) - self->txlen;
      int copied;

      if (self->dev.d_len - bytesread < chunk_sz)
        {
          chunk_sz = self->dev.d_len - bytesread;
        }

      copied = iob_copyout(&self->txbuf[self->txlen],
                           self->dev.d_iob,
                           chunk_sz,
                           bytesread);
      if (copied <= 0)
        {
          goto error;
        }

      bytesread += (unsigned int)copied;
      self->txlen += (size_t)copied;
    }

  /* SLIP encode self->txbuf.  First escape the ESC bytes. */

  for (p = memchr(&self->txbuf[1], SLIP_ESC, self->txlen - 1);
       p != NULL;
       p = memchr(p, SLIP_ESC, self->txlen - 1))
    {
      ssize_t postfix_len = self->txlen - (p - self->txbuf) - 1;

      if (self->txlen >= sizeof(self->txbuf))
        {
          goto error;
        }

      if (postfix_len > 0)
        {
          memmove(p + 2, p + 1, postfix_len);
        }

      p++;
      *p = SLIP_ESC_ESC;
      self->txlen++;
    }

  /* SLIP encode self->txbuf.  Then escape the END bytes. */

  for (p = memchr(&self->txbuf[1], SLIP_END, self->txlen - 1);
       p != NULL;
       p = memchr(p, SLIP_END, self->txlen - 1))
    {
      ssize_t postfix_len = self->txlen - (p - self->txbuf) - 1;

      if (self->txlen >= sizeof(self->txbuf))
        {
          goto error;
        }

      if (postfix_len > 0)
        {
          memmove(p + 2, p + 1, postfix_len);
        }

      *p = SLIP_ESC;
      p++;
      *p = SLIP_ESC_END;
      self->txlen++;
    }

  /* Append the END token */

  if (self->txlen >= sizeof(self->txbuf))
    {
      goto error;
    }

  self->txbuf[self->txlen++] = SLIP_END;

  /* Increment statistics */

  NETDEV_TXPACKETS(&self->dev);

  /* Try to send packet */

  ssz = file_write(&self->tty, self->txbuf, self->txlen);

  if (ssz > 0)
    {
      self->txsent = (size_t)ssz;
    }
  else
    {
      self->txsent = 0;
    }

  if (self->txsent == self->txlen)
    {
      /* Complete packet went out at first try. */

      slip_txdone(self);
    }

  return;

error:

  /* Drop the packet and reset the receiver logic. */

  self->txlen  = 0;
  self->txsent = 0;

  NETDEV_TXERRORS(&self->dev);
}

/****************************************************************************
 * Name: slip_txpoll
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
 *   The network is locked.
 *
 ****************************************************************************/

static int slip_txpoll(FAR struct net_driver_s *dev)
{
  FAR struct slip_driver_s *self =
      (FAR struct slip_driver_s *)dev->d_private;

  /* Send the packet */

  slip_transmit(self);

  /* If zero is returned, the polling will continue until all connections
   * have been examined.  We return -EBUSY if there is still transmission
   * data pending in the TTY's buffer.
   */

  return (self->txlen > 0) ? -EBUSY : OK;
}

/****************************************************************************
 * Name: slip_reply
 *
 * Description:
 *   After a packet has been received and dispatched to the network, it
 *   may return return with an outgoing packet.  This function checks for
 *   that case and performs the transmission if necessary.
 *
 * Input Parameters:
 *   self - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void slip_reply(struct slip_driver_s *self)
{
  /* If the packet dispatch resulted in data that should be sent out on the
   * network, the field d_len will set to a value > 0.
   */

  if (self->dev.d_len > 0)
    {
      /* And send the packet */

      slip_transmit(self);
    }
}

/****************************************************************************
 * Name: slip_receive
 *
 * Description:
 *   An interrupt was received indicating the availability of a new RX packet
 *
 * Input Parameters:
 *   self - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void slip_receive(FAR struct slip_driver_s *self)
{
  FAR struct net_driver_s *dev = &self->dev;
  FAR struct iob_s *iob;
  FAR uint8_t *p;
  FAR uint8_t *pend;
  size_t remaining;
  size_t copied;
  int ret;

  /* Drop potential prefix SLIP_ENDs */

  while ((self->rxbuf[0] == SLIP_END) && (self->rxlen > 0))
    {
      self->rxlen--;
      memmove(&self->rxbuf[0], &self->rxbuf[1], self->rxlen);
    }

  /* Find end of packet */

  pend = memchr(self->rxbuf, SLIP_END, self->rxlen);

  if (pend == NULL)
    {
      /* No complete packet present.  Let's wait for more bytes to arrive. */

      if (self->rxlen == sizeof(self->rxbuf))
        {
          /* Purge receive buffer overflow due to overflow. */

          NETDEV_RXERRORS(&self->dev);
          self->rxlen = 0;
        }

      return;
    }

  p = self->rxbuf;
  remaining = pend - p;
  copied = 0;
  iob = iob_alloc(false);
  iob_reserve(iob, CONFIG_NET_LL_GUARDSIZE);

  while (remaining)
    {
      uint8_t *pesc = memchr(p, SLIP_ESC, remaining);

      if (pesc != NULL)
        {
          unsigned int prefix_len = (unsigned int)(pesc - p);

          if (prefix_len > 0)
            {
              ret = iob_copyin(iob, p, prefix_len, copied, false);

              DEBUGASSERT(ret >= 0);
              DEBUGASSERT((unsigned int)ret == prefix_len);

              copied += prefix_len;
              remaining -= prefix_len;
            }

          p = pesc + 1;
          remaining--;

          switch (*p)
            {
              case SLIP_ESC_END:
                *p = SLIP_END;
                break;

              case SLIP_ESC_ESC:
                *p = SLIP_ESC;
                break;

              default:

                /* SLIP protocol error */

                goto error;
            }

          ret = iob_copyin(iob, p, 1, copied, false);
          DEBUGASSERT(ret == 1);
          p++;
          copied++;
          remaining--;
        }
      else
        {
          ret = iob_copyin(iob, p, remaining, copied, false);
          DEBUGASSERT(ret >= 0);
          DEBUGASSERT((unsigned int)ret == remaining);
          p += remaining;
          copied += remaining;
          remaining = 0;
        }
    }

  /* Move remaining bytes in rxbuf to the front */

  DEBUGASSERT((pend - self->rxbuf) <= self->rxlen);
  self->rxlen -= (pend - self->rxbuf);
  memmove(self->rxbuf, pend, self->rxlen);

  /* Handle the IP input. */

  netdev_iob_replace(&self->dev, iob);
  iob = NULL;

  NETDEV_RXPACKETS(&self->dev);

  /* All packets are assumed to be IP packets (we don't have a choice..
   * there is no Ethernet header containing the EtherType).  So pass the
   * received packet on for IP processing -- but only if it is big
   * enough to hold an IP header.
   */

  if ((IPv4BUF->vhl & IP_VERSION_MASK) == IPv4_VERSION)
    {
      NETDEV_RXIPV4(&self->dev);

      ipv4_input(&self->dev);

      slip_reply(self);
    }
  else
    {
      NETDEV_RXDROPPED(&self->dev);
    }

  return;

error:

  NETDEV_RXERRORS(&self->dev);

  if (iob)
    {
      iob_free_chain(iob);
      iob = NULL;
    }

  /* Move remaining bytes in rxbuf to the front */

  DEBUGASSERT((pend - self->rxbuf) <= self->rxlen);
  self->rxlen -= (pend - self->rxbuf);
  memmove(self->rxbuf, pend, self->rxlen);
}

/****************************************************************************
 * Name: slip_txdone
 *
 * Description:
 *   An interrupt was received indicating that the last TX packet(s) is done
 *
 * Input Parameters:
 *   self - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void slip_txdone(FAR struct slip_driver_s *self)
{
  /* Update statistics */

  self->txlen  = 0;
  self->txsent = 0;

  NETDEV_TXDONE(&self->dev);

  /* Poll the network for new TX data */

  if (work_available(&self->pollwork))
    {
      work_queue(SLIPWORK, &self->pollwork, slip_txavail_work, self, 0);
    }
}

/****************************************************************************
 * Name: slip_interrupt_work
 *
 * Description:
 *   Perform interrupt related work from the worker thread
 *
 * Input Parameters:
 *   arg - The argument passed when work_queue() was called.
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *   Runs on a worker thread.
 *
 ****************************************************************************/

static void slip_interrupt_work(FAR void *arg)
{
  FAR struct slip_driver_s *self = (FAR struct slip_driver_s *)arg;
  ssize_t ssz;

  if (!self->bifup)
    {
      return;
    }

  /* Lock the network and serialize driver operations if necessary.
   * NOTE: Serialization is only required in the case where the driver work
   * is performed on an LP worker thread and where more than one LP worker
   * thread has been configured.
   */

  net_lock();

  /* Process pending Ethernet interrupts */

  /* Get and clear interrupt status bits */

  /* Handle interrupts according to status bit settings */

  if (self->rxlen < sizeof(self->rxbuf))
    {
      ssz = file_read(&self->tty,
                      &self->rxbuf[self->rxlen],
                      sizeof(self->rxbuf) - self->rxlen);
      if (ssz > 0)
        {
          self->rxlen += (size_t)ssz;
        }
    }

  if (self->txsent < self->txlen)
    {
      ssz = file_write(&self->tty,
                       &self->txbuf[self->txsent],
                       self->txlen - self->txsent);
      if (ssz > 0)
        {
          self->txsent += (size_t)ssz;
        }
    }

  /* Check if we received an incoming packet, if so, call slip_receive() */

  if (self->rxlen == sizeof(self->rxbuf) ||
      memchr(self->rxbuf, SLIP_END, self->rxlen))
    {
      slip_receive(self);
    }

  /* Check if a packet transmission just completed.  If so, call skel_txdone.
   * This may disable further Tx interrupts if there are no pending
   * transmissions.
   */

  if (self->txlen > 0 && self->txsent == self->txlen)
    {
      slip_txdone(self);
    }

  net_unlock();
}

/****************************************************************************
 * Name: slip_ifup
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
 *   The network is locked.
 *
 ****************************************************************************/

static int slip_ifup(FAR struct net_driver_s *dev)
{
  FAR struct slip_driver_s *self =
      (FAR struct slip_driver_s *)dev->d_private;

  ninfo("Bringing up: %d.%d.%d.%d\n",
        (int)dev->d_ipaddr & 0xff,
        (int)(dev->d_ipaddr >> 8) & 0xff,
        (int)(dev->d_ipaddr >> 16) & 0xff,
        (int)dev->d_ipaddr >> 24);

  /* Enable POLLIN and POLLOUT events on the TTY */

  slip_set_pollfd_events(self, POLLIN | POLLOUT);

  /* Mark the device "up" */

  self->bifup = true;

  return OK;
}

/****************************************************************************
 * Name: slip_ifdown
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
 *   The network is locked.
 *
 ****************************************************************************/

static int slip_ifdown(FAR struct net_driver_s *dev)
{
  FAR struct slip_driver_s *self =
    (FAR struct slip_driver_s *)dev->d_private;

  /* Disable the Ethernet interrupt */

  slip_set_pollfd_events(self, 0);

  /* Mark the device "down" */

  self->bifup = false;

  return OK;
}

/****************************************************************************
 * Name: slip_txavail_work
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
 *   Runs on a work queue thread.
 *
 ****************************************************************************/

static void slip_txavail_work(FAR void *arg)
{
  FAR struct slip_driver_s *self = (FAR struct slip_driver_s *)arg;

  /* Lock the network and serialize driver operations if necessary.
   * NOTE: Serialization is only required in the case where the driver work
   * is performed on an LP worker thread and where more than one LP worker
   * thread has been configured.
   */

  net_lock();

  /* Ignore the notification if the interface is not yet up */

  if (self->bifup)
    {
      /* Check if there is room in the hardware to hold another packet. */

      if (self->txlen == 0)
        {
          /* If so, then poll the network for new XMIT data */

          self->dev.d_buf = NULL;

          devif_poll(&self->dev, slip_txpoll);
        }
    }

  net_unlock();
}

/****************************************************************************
 * Name: slip_txavail
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
 *   The network is locked.
 *
 ****************************************************************************/

static int slip_txavail(FAR struct net_driver_s *dev)
{
  FAR struct slip_driver_s *self =
      (FAR struct slip_driver_s *)dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&self->pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(SLIPWORK, &self->pollwork, slip_txavail_work, self, 0);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: slip_initialize
 *
 * Description:
 *   Instantiate a SLIP network interface.
 *
 * Input Parameters:
 *   intf    - In the case where there are multiple SLIP interfaces, this
 *             value identifies which is to be initialized. The number of
 *             possible SLIP interfaces is determined by
 *   devname - This is the path to the serial device that will support SLIP.
 *             For example, this might be "/dev/ttyS1"
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int slip_initialize(int intf, FAR const char *devname)
{
  FAR struct slip_driver_s *self;
  int ret;

  /* Get the interface structure associated with this interface number. */

  DEBUGASSERT(intf < CONFIG_NET_SLIP_NINTERFACES);
  self = &g_slip[intf];

  /* Initialize the driver structure */

  memset(self, 0, sizeof(struct slip_driver_s));
  self->dev.d_ifup    = slip_ifup;    /* I/F up (new IP address) callback */
  self->dev.d_ifdown  = slip_ifdown;  /* I/F down callback */
  self->dev.d_txavail = slip_txavail; /* New TX data callback */
  self->dev.d_private = self;         /* Used to recover SLIP I/F instance */

  ret = file_open(&self->tty, devname, O_RDWR | O_NONBLOCK);

  if (ret < 0)
    {
      nerr("ERROR: Failed to open %s: %d\n", devname, ret);
      return ret;
    }

  /* Put the interface in the down state.  This usually amounts to resetting
   * the device and/or calling slip_ifdown().
   */

  slip_set_pollfd_events(self, 0);
  self->bifup = false;

  /* Register the device with the OS so that socket IOCTLs can be performed */

  netdev_register(&self->dev, NET_LL_SLIP);

  return OK;
}

#endif /* !defined(CONFIG_SCHED_WORKQUEUE) */

#endif /* CONFIG_NET_SLIP */
