/****************************************************************************
 * drivers/virtio/virtio-mmio-net.c
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
#include <time.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arpa/inet.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/irq.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/netdev.h>
#include <nuttx/virtio/virtio-mmio.h>

#include "virtio-mmio-net.h"

#ifdef CONFIG_NET_PKT
#  include <nuttx/net/pkt.h>
#endif

#ifdef CONFIG_DRIVERS_VIRTIO_NET

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DEBUG

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

#define ETHWORK LPWORK

/* VIRTIO_NET_NINTERFACES determines the number of
 * physical interfaces that will be supported.
 */

#ifndef VIRTIO_NET_NINTERFACES
# define VIRTIO_NET_NINTERFACES 1
#endif

/* TX timeout = 1 minute */

#define VIRTNET_TXTIMEOUT (60*CLK_TCK)

/* Packet buffer size */

#define PKTBUF_SIZE (MAX_NETDEV_PKTSIZE + CONFIG_NET_GUARDSIZE)

/* This is a helper pointer for accessing the contents of Ethernet header */

#define BUF ((struct eth_hdr_s *)priv->vnet_dev.d_buf)

/* virtio net related definition */

#define VIRTIO_NET_HDRLEN 10

#define VIRTIO_NET_Q_RX 0
#define VIRTIO_NET_Q_TX 1

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct virtnet_hdr_s
{
  uint8_t  flags;
  uint8_t  gso_type;
  uint16_t hdr_len;
  uint16_t gso_size;
  uint16_t csum_start;
  uint16_t csum_offset;
};

/* The virtnet_driver_s encapsulates all state information for
 * a single hardware interface
 */

struct virtnet_driver_s
{
  bool vnet_bifup;               /* true:ifup false:ifdown */
  int  irq;

  FAR struct virtio_mmio_regs *regs; /* virtio_mmio registers */
  FAR struct virtqueue   *txq;       /* TX queue */
  FAR struct virtqueue   *rxq;       /* RX queue */

  struct wdog_s vnet_txtimeout;  /* TX timeout timer */
  struct work_s vnet_irqwork;    /* For deferring interrupt work to the work queue */
  struct work_s vnet_pollwork;   /* For deferring poll work to the work queue */

  /* This holds the information visible to the NuttX network */

  struct net_driver_s vnet_dev;  /* Interface understood by the network */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* These statically allocated structures would mean that only a single
 * instance of the device could be supported.  In order to support multiple
 * devices instances, this data would have to be allocated dynamically.
 */

/* A single packet buffer per device is used in this example.  There might
 * be multiple packet buffers in a more complex, pipelined design.  Many
 * contemporary Ethernet interfaces, for example,  use multiple, linked DMA
 * descriptors in rings to implement such a pipeline.  This example assumes
 * much simpler hardware that simply handles one packet at a time.
 *
 * NOTE that if VIRTIO_NET_NINTERFACES were greater than 1,
 * you would need a minimum on one packet buffer per instance.
 * Much better to be allocated dynamically in cases where more than
 * one are needed.
 */

static uint16_t
g_pktbuf[VIRTIO_NET_NINTERFACES][(PKTBUF_SIZE + 1) / 2];

/* Driver state structure */

static struct virtnet_driver_s g_virtnet[VIRTIO_NET_NINTERFACES];

static uint32_t g_ninterfaces = 0;

/* Common virtnet header */

static struct virtnet_hdr_s g_hdr;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Common TX logic */

static int  virtnet_transmit(FAR struct virtnet_driver_s *priv);
static int  virtnet_txpoll(FAR struct net_driver_s *dev);

/* Interrupt handling */

static void virtnet_reply(FAR struct virtnet_driver_s *priv);
static void virtnet_receive(FAR struct virtnet_driver_s *priv);

#ifdef TODO
static void virtnet_txdone(FAR struct virtnet_driver_s *priv);
#endif

static void virtnet_interrupt_work(FAR void *arg);
static int  virtnet_interrupt(int irq, FAR void *context, FAR void *arg);

/* Watchdog timer expirations */

static void virtnet_txtimeout_work(FAR void *arg);
static void virtnet_txtimeout_expiry(FAR wdparm_t arg);

/* NuttX callback functions */

static int  virtnet_ifup(FAR struct net_driver_s *dev);
static int  virtnet_ifdown(FAR struct net_driver_s *dev);

static void virtnet_txavail_work(FAR void *arg);
static int  virtnet_txavail(FAR struct net_driver_s *dev);

#if defined(CONFIG_NET_MCASTGROUP) || defined(CONFIG_NET_ICMPv6)
static int  virtnet_addmac(FAR struct net_driver_s *dev,
                           FAR const uint8_t *mac);
#ifdef CONFIG_NET_MCASTGROUP
static int  virtnet_rmmac(FAR struct net_driver_s *dev,
                          FAR const uint8_t *mac);
#endif
#ifdef CONFIG_NET_ICMPv6
static void virtnet_ipv6multicast(FAR struct virtnet_driver_s *priv);
#endif
#endif
#ifdef CONFIG_NETDEV_IOCTL
static int  virtnet_ioctl(FAR struct net_driver_s *dev, int cmd,
                          unsigned long arg);
#endif

static void virtnet_rxdispatch(FAR struct virtnet_driver_s *priv);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: virtnet_cmd_status
 ****************************************************************************/

#ifdef DEBUG
static void virtnet_cmd_status(FAR struct virtnet_driver_s *priv)
{
  FAR struct virtio_mmio_regs *regs = priv->regs;

  vrtinfo("virtnet at %p\n", regs);
  vrtinfo("  status=0x%" PRIx32 "\n",
          virtio_getreg32(&regs->status));
  vrtinfo("  interrupt_status=0x%" PRIx32 "\n",
          virtio_getreg32(&regs->interrupt_status));

  virtio_putreg32(VIRTIO_NET_Q_TX, &regs->queue_sel);
  virtio_mb();

  vrtinfo("  txq: avail->idx=%d, used->idx=%d, ready=0x%" PRIx32 "\n",
          priv->txq->avail->idx, priv->txq->used->idx,
          virtio_getreg32(&regs->queue_ready));

  virtio_putreg32(VIRTIO_NET_Q_RX, &regs->queue_sel);
  virtio_mb();

  vrtinfo("  rxq: avail->idx=%d used->idx=%d ready=0x%" PRIx32 "\n",
          priv->rxq->avail->idx, priv->rxq->used->idx,
          virtio_getreg32(&regs->queue_ready));
}
#endif

/****************************************************************************
 * Name: virtnet_add_packets_to_rxq
 ****************************************************************************/

static void virtnet_add_packets_to_rxq(FAR struct virtnet_driver_s *priv)
{
  FAR struct virtqueue *rxq = priv->rxq;
  FAR uint8_t  *pkt;
  uint16_t d1;
  uint16_t d2;
  uint32_t i;

  vrtinfo("+++ rxq->len=%" PRId32 "  rxq->avail-idx=%d \n",
          rxq->len, rxq->avail->idx);
  DEBUGASSERT(rxq->avail->idx == 0);

  for (i = 0; i < rxq->len / 2; i++)
    {
      pkt = kmm_memalign(16, PKTBUF_SIZE);
      ASSERT(pkt);

      /* Allocate new descriptors for header and packet */

      d1 = virtq_alloc_desc(rxq, (i * 2),  (FAR void *)&g_hdr);
      d2 = virtq_alloc_desc(rxq, (i * 2) + 1, pkt);

      /* Set up the descriptor for header */

      rxq->desc[d1].len   = VIRTIO_NET_HDRLEN;
      rxq->desc[d1].flags = VIRTQ_DESC_F_WRITE | VIRTQ_DESC_F_NEXT;
      rxq->desc[d1].next  = d2;

      /* Set up the descriptor for packet */

      rxq->desc[d2].len   = PKTBUF_SIZE;
      rxq->desc[d2].flags = VIRTQ_DESC_F_WRITE;

      /* Set the first descriptor to the avail->ring */

      rxq->avail->ring[i] = d1;

      /* Increment the avail->idx for each two descriptors */

      virtio_mb();
      rxq->avail->idx += 1;
    }

  vrtinfo("+++ virtq->avail-idx=%d \n", rxq->avail->idx);
  vrtinfo("+++ virtq->used->idx=%d \n", rxq->used->idx);
}

/****************************************************************************
 * Name: virtnet_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from the txdone interrupt
 *   handling or from watchdog based polling.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static int virtnet_transmit(FAR struct virtnet_driver_s *priv)
{
  FAR void *pkt = priv->vnet_dev.d_buf;
  uint32_t len  = priv->vnet_dev.d_len;
  uint16_t idx  = priv->txq->avail->idx;
  uint16_t d1;
  uint16_t d2;

  /* Send the packet:
   * address=priv->vnet_dev.d_buf, length=priv->vnet_dev.d_len
   */

  /* Verify that the hardware is ready to send another packet.  If we get
   * here, then we are committed to sending a packet; Higher level logic
   * must have assured that there is no transmission in progress.
   */

  /* Increment statistics */

  NETDEV_TXPACKETS(priv->vnet_dev);

  vrtinfo("=== Sending packet, length: %d\n", priv->vnet_dev.d_len);

#ifdef DEBUG
  virtnet_cmd_status(priv);
#endif

  /* Allocate new descriptors for header and packet */

  d1 = virtq_alloc_desc(priv->txq, idx, (FAR void *)&g_hdr);
  d2 = virtq_alloc_desc(priv->txq, idx + 1, pkt);
  DEBUGASSERT(d1 != d2);

  /* Set up the descriptor for header */

  priv->txq->desc[d2].len   = len;
  priv->txq->desc[d2].flags = 0;

  /* Set up the descriptor for packet */

  priv->txq->desc[d1].len   = VIRTIO_NET_HDRLEN;
  priv->txq->desc[d1].flags = VIRTQ_DESC_F_NEXT;
  priv->txq->desc[d1].next  = d2;

  /* Set the first descriptor to the avail->ring */

  priv->txq->avail->ring[d1] = d1;

  /* Increment the avail->idx for each two descriptors */

  virtio_mb();
  priv->txq->avail->idx += 1;

  vrtinfo("*** d1=%d, d2=%d, txq->avail->idx=%d\n",
          d1, d2, priv->txq->avail->idx);

  virtio_putreg32(VIRTIO_NET_Q_TX, &priv->regs->queue_notify);

  vrtinfo("*** finish updating queue_notify\n");

#ifdef TODO
  /* Enable Tx interrupts */
#endif

  /* Setup the TX timeout watchdog (perhaps restarting the timer) */

  wd_start(&priv->vnet_txtimeout, VIRTNET_TXTIMEOUT,
           virtnet_txtimeout_expiry, (wdparm_t)priv);
  return OK;
}

/****************************************************************************
 * Name: virtnet_txpoll
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

static int virtnet_txpoll(FAR struct net_driver_s *dev)
{
  FAR struct virtnet_driver_s *priv =
    (FAR struct virtnet_driver_s *)dev->d_private;

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  vrtinfo("Poll result: d_len=%d\n", priv->vnet_dev.d_len);

  /* Send the packet */

  virtnet_reply(priv);

  /* If zero is returned, the polling will continue until all connections
   * have been examined.
   */

  return 0;
}

/****************************************************************************
 * Name: virtnet_reply
 *
 * Description:
 *   After a packet has been received and dispatched to the network, it
 *   may return return with an outgoing packet.  This function checks for
 *   that case and performs the transmission if necessary.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void virtnet_reply(FAR struct virtnet_driver_s *priv)
{
  /* If the packet dispatch resulted in data that should be sent out on the
   * network, the field d_len will set to a value > 0.
   */

  if (priv->vnet_dev.d_len > 0)
    {
      /* And send the packet */

      virtnet_transmit(priv);
    }
}

/****************************************************************************
 * Function: virtnet_rxdispatch
 *
 * Description:
 *   A new Rx packet was received; dispatch that packet to the network layer
 *   as necessary.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Global interrupts are disabled by interrupt handling logic.
 *
 ****************************************************************************/

static void virtnet_rxdispatch(FAR struct virtnet_driver_s *priv)
{
  /* Update statistics */

  NETDEV_RXPACKETS(&priv->dev);

#ifdef CONFIG_NET_PKT
  /* When packet sockets are enabled, feed the frame into the tap */

  pkt_input(&priv->vnet_dev);
#endif

#ifdef CONFIG_NET_IPv4
  /* Check for an IPv4 packet */

  if (BUF->type == HTONS(ETHTYPE_IP))
    {
      vrtinfo("IPv4 frame\n");
      NETDEV_RXIPV4(&priv->vnet_dev);

      /* Receive an IPv4 packet from the network device */

      vrtinfo("+++ call ipv4_input() d_len=%d\n", priv->vnet_dev.d_len);
      ipv4_input(&priv->vnet_dev);

      /* Check for a reply to the IPv4 packet */

      virtnet_reply(priv);
    }
  else
#endif
#ifdef CONFIG_NET_IPv6
  /* Check for an IPv6 packet */

  if (BUF->type == HTONS(ETHTYPE_IP6))
    {
      vrtinfo("IPv6 frame\n");
      NETDEV_RXIPV6(&priv->vnet_dev);

      /* Dispatch IPv6 packet to the network layer */

      ipv6_input(&priv->vnet_dev);

      /* Check for a reply to the IPv6 packet */

      virtnet_reply(priv);
    }
  else
#endif
#ifdef CONFIG_NET_ARP
  /* Check for an ARP packet */

  if (BUF->type == HTONS(ETHTYPE_ARP))
    {
      /* Dispatch ARP packet to the network layer */

      arp_input(&priv->vnet_dev);
      NETDEV_RXARP(&priv->vnet_dev);

      /* If the above function invocation resulted in data that should be
       * sent out on the network, the field  d_len will set to a value
       * > 0.
       */

      if (priv->vnet_dev.d_len > 0)
        {
          virtnet_transmit(priv);
        }
    }
  else
#endif
    {
      NETDEV_RXDROPPED(&priv->vnet_dev);

      vrtwarn("+++ dropped BUF->type=0x%x \n", BUF->type);
      DEBUGASSERT(0 != BUF->type);
    }
}

/****************************************************************************
 * Name: virtnet_receive
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
 *   The network is locked.
 *
 ****************************************************************************/

static void virtnet_receive(FAR struct virtnet_driver_s *priv)
{
  uint16_t used = priv->rxq->used->idx;
  uint16_t idx;

  vrtinfo("+++ rxq->last_used_idx=%d: rxq->used->idx=%d\n",
          priv->rxq->last_used_idx, priv->rxq->used->idx);

  for (idx = priv->rxq->last_used_idx; idx != used; idx++)
    {
      uint16_t id  = idx % priv->rxq->len;
      uint16_t d1  = priv->rxq->used->ring[id].id; /* index for header */
      uint16_t d2  = priv->rxq->desc[d1].next;     /* index for packet */
      uint32_t len = priv->rxq->used->ring[id].len;
      DEBUGASSERT(d2 == (d1 + 1));

      vrtinfo("+++ idx=%d id=%d used->idx=%d: d1=%d d2=%d len=%" PRId32 "\n",
              idx, id, priv->rxq->used->idx, d1, d2, len);

      /* Set the packet info to d_buf and set d_len */

      priv->vnet_dev.d_buf = priv->rxq->desc_virt[d2];
      priv->vnet_dev.d_len = len - VIRTIO_NET_HDRLEN;

      vrtinfo("Receiving packet, pktlen: %d\n", priv->vnet_dev.d_len);

      /* Dispatch the incoming packet */

      virtnet_rxdispatch(priv);

      /* Set the descriptor back into the avail queue */

      id = priv->rxq->avail->idx % priv->rxq->len;
      priv->rxq->avail->ring[id] = d1;

      virtio_mb();
      priv->rxq->avail->idx += 1;

      vrtinfo("+++ rxq->avail->idx=%d\n", priv->rxq->avail->idx);
    }

  priv->rxq->last_used_idx = used;
  vrtinfo("+++ rxq->last_used_idx=%d\n",  priv->rxq->last_used_idx);
}

/****************************************************************************
 * Name: virtnet_txdone
 *
 * Description:
 *   An interrupt was received indicating that the last TX packet(s) is done
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

#ifdef TODO
static void virtnet_txdone(FAR struct virtnet_driver_s *priv)
{
  /* Check for errors and update statistics */

  NETDEV_TXDONE(priv->vnet_dev);

  /* Check if there are pending transmissions */

  /* If no further transmissions are pending, then cancel the TX timeout and
   * disable further Tx interrupts.
   */

  wd_cancel(&priv->vnet_txtimeout);

  /* And disable further TX interrupts. */

  /* In any event, poll the network for new TX data */

  devif_poll(&priv->vnet_dev, virtnet_txpoll);
}
#endif

/****************************************************************************
 * Name: virtnet_interrupt_work
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

static void virtnet_interrupt_work(FAR void *arg)
{
  FAR struct virtnet_driver_s *priv = (FAR struct virtnet_driver_s *)arg;

  /* Lock the network and serialize driver operations if necessary.
   * NOTE: Serialization is only required in the case where the driver work
   * is performed on an LP worker thread and where more than one LP worker
   * thread has been configured.
   */

  net_lock();

  /* Process pending Ethernet interrupts */

  /* Handle interrupts according to status bit settings */

  /* Check if we received an incoming packet, if so, call virtnet_receive() */

  virtnet_receive(priv);

  /* Check if a packet transmission just completed.
   * If so, call virtnet_txdone.
   * This may disable further Tx interrupts if there are no pending
   * transmissions.
   */

#ifdef TODO
  virtnet_txdone(priv);
#endif

  net_unlock();

  if (priv->rxq->last_used_idx == priv->rxq->used->idx)
    {
      /* Re-enable Ethernet interrupts */

      priv->rxq->avail->flags = 0;
      priv->txq->avail->flags = 0;
      virtio_mb();

      virtio_putreg32(VIRTIO_NET_Q_RX, &priv->regs->queue_notify);
    }
  else if (work_available(&priv->vnet_irqwork))
    {
      work_queue(ETHWORK, &priv->vnet_irqwork,
                 virtnet_interrupt_work, priv, 0);
    }
}

/****************************************************************************
 * Name: virtnet_interrupt
 *
 * Description:
 *   Hardware interrupt handler
 *
 * Input Parameters:
 *   irq     - Number of the IRQ that generated the interrupt
 *   context - Interrupt register state save info (architecture-specific)
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *   Runs in the context of a the Ethernet interrupt handler.  Local
 *   interrupts are disabled by the interrupt logic.
 *
 ****************************************************************************/

static int virtnet_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct virtnet_driver_s *priv = (FAR struct virtnet_driver_s *)arg;
  uint32_t stat;

  DEBUGASSERT(priv != NULL);

  /* Get and clear interrupt status bits */

  stat = virtio_getreg32(&priv->regs->interrupt_status);
  virtio_putreg32(stat, &priv->regs->interrupt_ack);
  vrtinfo("+++ called (stat=0x%" PRIx32 ")\n", stat);

  /* Disable further Ethernet interrupts.  Because Ethernet interrupts are
   * also disabled if the TX timeout event occurs, there can be no race
   * condition here.
   */

  priv->rxq->avail->flags = 1;
  priv->txq->avail->flags = 1;
  virtio_mb();

  /* Schedule to perform the interrupt processing on the worker thread. */

  if (work_available(&priv->vnet_irqwork))
    {
      work_queue(ETHWORK, &priv->vnet_irqwork,
                 virtnet_interrupt_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: virtnet_txtimeout_work
 *
 * Description:
 *   Perform TX timeout related work from the worker thread
 *
 * Input Parameters:
 *   arg - The argument passed when work_queue() as called.
 *
 * Returned Value:
 *   OK on success
 *
 ****************************************************************************/

static void virtnet_txtimeout_work(FAR void *arg)
{
  FAR struct virtnet_driver_s *priv = (FAR struct virtnet_driver_s *)arg;

  /* Lock the network and serialize driver operations if necessary.
   * NOTE: Serialization is only required in the case where the driver work
   * is performed on an LP worker thread and where more than one LP worker
   * thread has been configured.
   */

  net_lock();

  /* Increment statistics and dump debug info */

  NETDEV_TXTIMEOUTS(priv->vnet_dev);

  /* Then reset the hardware */

  /* Then poll the network for new XMIT data */

  devif_poll(&priv->vnet_dev, virtnet_txpoll);
  net_unlock();
}

/****************************************************************************
 * Name: virtnet_txtimeout_expiry
 *
 * Description:
 *   Our TX watchdog timed out.  Called from the timer interrupt handler.
 *   The last TX never completed.  Reset the hardware and start again.
 *
 * Input Parameters:
 *   arg  - The argument
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Runs in the context of a the timer interrupt handler.  Local
 *   interrupts are disabled by the interrupt logic.
 *
 ****************************************************************************/

static void virtnet_txtimeout_expiry(wdparm_t arg)
{
  FAR struct virtnet_driver_s *priv = (FAR struct virtnet_driver_s *)arg;

#ifdef TODO
  /* Disable further Ethernet interrupts.  This will prevent some race
   * conditions with interrupt work.  There is still a potential race
   * condition with interrupt work that is already queued and in progress.
   */
#endif

  /* Schedule to perform the TX timeout processing on the worker thread. */

  if (work_available(&priv->vnet_irqwork))
    {
      work_queue(ETHWORK, &priv->vnet_irqwork,
                 virtnet_txtimeout_work, priv, 0);
    }
}

/****************************************************************************
 * Name: virtnet_ifup
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

static int virtnet_ifup(FAR struct net_driver_s *dev)
{
  FAR struct virtnet_driver_s *priv =
    (FAR struct virtnet_driver_s *)dev->d_private;

#ifdef CONFIG_NET_IPv4
  vrtinfo("Bringing up: %d.%d.%d.%d\n",
          (int)dev->d_ipaddr & 0xff,
          (int)(dev->d_ipaddr >> 8) & 0xff,
          (int)(dev->d_ipaddr >> 16) & 0xff,
          (int)dev->d_ipaddr >> 24);
#endif
#ifdef CONFIG_NET_IPv6
  vrtinfo("Bringing up: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
          dev->d_ipv6addr[0], dev->d_ipv6addr[1], dev->d_ipv6addr[2],
          dev->d_ipv6addr[3], dev->d_ipv6addr[4], dev->d_ipv6addr[5],
          dev->d_ipv6addr[6], dev->d_ipv6addr[7]);
#endif

  /* Initialize PHYs, Ethernet interface, and setup up Ethernet interrupts */

  /* Instantiate MAC address from
   * priv->vnet_dev.d_mac.ether.ether_addr_octet
   */

#ifdef CONFIG_NET_ICMPv6
  /* Set up IPv6 multicast address filtering */

  virtnet_ipv6multicast(priv);
#endif

  /* Enable the Ethernet interrupt */

  priv->vnet_bifup = true;
  up_enable_irq(priv->irq);
  return OK;
}

/****************************************************************************
 * Name: virtnet_ifdown
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

static int virtnet_ifdown(FAR struct net_driver_s *dev)
{
  FAR struct virtnet_driver_s *priv =
    (FAR struct virtnet_driver_s *)dev->d_private;
  irqstate_t flags;

  /* Disable the Ethernet interrupt */

  flags = enter_critical_section();
  up_disable_irq(priv->irq);

  /* Cancel the TX timeout timers */

  wd_cancel(&priv->vnet_txtimeout);

  /* Put the EMAC in its reset, non-operational state.  This should be
   * a known configuration that will guarantee the virtnet_ifup() always
   * successfully brings the interface back up.
   */

  /* Mark the device "down" */

  priv->vnet_bifup = false;
  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: virtnet_txavail_work
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

static void virtnet_txavail_work(FAR void *arg)
{
  FAR struct virtnet_driver_s *priv = (FAR struct virtnet_driver_s *)arg;

  /* Lock the network and serialize driver operations if necessary.
   * NOTE: Serialization is only required in the case where the driver work
   * is performed on an LP worker thread and where more than one LP worker
   * thread has been configured.
   */

  net_lock();

  /* Ignore the notification if the interface is not yet up */

  if (priv->vnet_bifup)
    {
      /* Check if there is room in the hardware to hold another packet. */

      /* If so, then poll the network for new XMIT data */

      devif_poll(&priv->vnet_dev, virtnet_txpoll);
    }

  net_unlock();
}

/****************************************************************************
 * Name: virtnet_txavail
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

static int virtnet_txavail(FAR struct net_driver_s *dev)
{
  FAR struct virtnet_driver_s *priv =
    (FAR struct virtnet_driver_s *)dev->d_private;

  /* Is our single work structure available?  It may not be if there are
   * pending interrupt actions and we will have to ignore the Tx
   * availability action.
   */

  if (work_available(&priv->vnet_pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(ETHWORK, &priv->vnet_pollwork,
                 virtnet_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: virtnet_addmac
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
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_MCASTGROUP) || defined(CONFIG_NET_ICMPv6)
static int virtnet_addmac(FAR struct net_driver_s *dev,
                          FAR const uint8_t *mac)
{
  FAR struct virtnet_driver_s *priv =
    (FAR struct virtnet_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  UNUSED(priv);
  return OK;
}
#endif

/****************************************************************************
 * Name: virtnet_rmmac
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
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_MCASTGROUP
static int virtnet_rmmac(FAR struct net_driver_s *dev,
                         FAR const uint8_t *mac)
{
  FAR struct virtnet_driver_s *priv =
    (FAR struct virtnet_driver_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  UNUSED(priv);
  return OK;
}
#endif

/****************************************************************************
 * Name: virtnet_ipv6multicast
 *
 * Description:
 *   Configure the IPv6 multicast MAC address.
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6
static void virtnet_ipv6multicast(FAR struct virtnet_driver_s *priv)
{
  FAR struct net_driver_s *dev;
  uint16_t tmp16;
  uint8_t mac[6];

  /* For ICMPv6, we need to add the IPv6 multicast address
   *
   * For IPv6 multicast addresses, the Ethernet MAC is derived by
   * the four low-order octets OR'ed with the MAC 33:33:00:00:00:00,
   * so for example the IPv6 address FF02:DEAD:BEEF::1:3 would map
   * to the Ethernet MAC address 33:33:00:01:00:03.
   *
   * NOTES:  This appears correct for the ICMPv6 Router Solicitation
   * Message, but the ICMPv6 Neighbor Solicitation message seems to
   * use 33:33:ff:01:00:03.
   */

  mac[0] = 0x33;
  mac[1] = 0x33;

  dev    = &priv->dev;
  tmp16  = dev->d_ipv6addr[6];
  mac[2] = 0xff;
  mac[3] = tmp16 >> 8;

  tmp16  = dev->d_ipv6addr[7];
  mac[4] = tmp16 & 0xff;
  mac[5] = tmp16 >> 8;

  vrtinfo("IPv6 Multicast: %02x:%02x:%02x:%02x:%02x:%02x\n",
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  virtnet_addmac(dev, mac);

#ifdef CONFIG_NET_ICMPv6_AUTOCONF
  /* Add the IPv6 all link-local nodes Ethernet address.  This is the
   * address that we expect to receive ICMPv6 Router Advertisement
   * packets.
   */

  virtnet_addmac(dev, g_ipv6_ethallnodes.ether_addr_octet);

#endif /* CONFIG_NET_ICMPv6_AUTOCONF */

#ifdef CONFIG_NET_ICMPv6_ROUTER
  /* Add the IPv6 all link-local routers Ethernet address.  This is the
   * address that we expect to receive ICMPv6 Router Solicitation
   * packets.
   */

  virtnet_addmac(dev, g_ipv6_ethallrouters.ether_addr_octet);

#endif /* CONFIG_NET_ICMPv6_ROUTER */
}
#endif /* CONFIG_NET_ICMPv6 */

/****************************************************************************
 * Name: virtnet_ioctl
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
 *   The network is locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_IOCTL
static int virtnet_ioctl(FAR struct net_driver_s *dev, int cmd,
                         unsigned long arg)
{
  FAR struct virtnet_driver_s *priv =
    (FAR struct virtnet_driver_s *)dev->d_private;
  int ret;

  /* Decode and dispatch the driver-specific IOCTL command */

  switch (cmd)
    {
      /* Add cases here to support the IOCTL commands */

      default:
        vrterr("ERROR: Unrecognized IOCTL command: %d\n", command);
        return -ENOTTY;  /* Special return value for this case */
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: virtnet_initialize
 *
 * Description:
 *   Initialize the virt-net driver
 *
 * Input Parameters:
 *
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *   Called early in initialization before multi-tasking is initiated.
 *
 ****************************************************************************/

static int virtnet_initialize(FAR struct virtio_mmio_regs *regs, int irq)
{
  FAR struct virtnet_driver_s *priv;

  /* Get the interface structure associated with this interface number. */

  DEBUGASSERT(g_ninterfaces < VIRTIO_NET_NINTERFACES);
  priv = &g_virtnet[g_ninterfaces];

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct virtnet_driver_s));

  /* Check if a Ethernet chip is recognized at its I/O base */

  /* Attach the IRQ to the driver */

  priv->irq = irq;

  if (irq_attach(priv->irq, virtnet_interrupt, priv))
    {
      /* We could not attach the ISR to the interrupt */

      return -EAGAIN;
    }

  /* Setup virtio related */

  priv->regs = regs;

  priv->txq  = virtq_create(CONFIG_DRIVERS_VIRTIO_NET_QUEUE_LEN);
  priv->rxq  = virtq_create(CONFIG_DRIVERS_VIRTIO_NET_QUEUE_LEN);

  virtq_add_to_mmio_device(regs, priv->rxq, VIRTIO_NET_Q_RX);
  virtq_add_to_mmio_device(regs, priv->txq, VIRTIO_NET_Q_TX);

  /* Prepare packets for rxq */

  virtnet_add_packets_to_rxq(priv);

  priv->vnet_dev.d_buf     = (FAR uint8_t *)g_pktbuf[g_ninterfaces];  /* Single packet buffer */

  priv->vnet_dev.d_ifup    = virtnet_ifup;               /* I/F up (new IP address) callback */
  priv->vnet_dev.d_ifdown  = virtnet_ifdown;             /* I/F down callback */
  priv->vnet_dev.d_txavail = virtnet_txavail;            /* New TX data callback */
#ifdef CONFIG_NET_MCASTGROUP
  priv->vnet_dev.d_addmac  = virtnet_addmac;             /* Add multicast MAC address */
  priv->vnet_dev.d_rmmac   = virtnet_rmmac;              /* Remove multicast MAC address */
#endif
#ifdef CONFIG_NETDEV_IOCTL
  priv->vnet_dev.d_ioctl   = virtnet_ioctl;              /* Handle network IOCTL commands */
#endif
  priv->vnet_dev.d_private = g_virtnet;                  /* Used to recover private state from dev */

#ifdef TODO
  /* Put the interface in the down state.  This usually amounts to resetting
   * the device and/or calling virtnet_ifdown().
   */

  /* Read the MAC address from the hardware into
   * priv->vnet_dev.d_mac.ether.ether_addr_octet
   * Applies only if the Ethernet MAC has its own internal address.
   */
#endif

  /* Register the device with the OS so that socket IOCTLs can be performed */

  netdev_register(&priv->vnet_dev, NET_LL_ETHERNET);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: virtio_mmio_net_init
 *
 * Description:
 *   Called from virtio-mmio.c to initialize virtnet
 *
 ****************************************************************************/

int virtio_mmio_net_init(FAR struct virtio_mmio_regs *regs, uint32_t irq)
{
  int ret = OK;

  /* TODO: feature negotiation */

  /* Set STATUS_FEATURE_OK */

  virtio_putreg32(virtio_getreg32(&regs->status) | VIRTIO_STATUS_FEATURES_OK,
                  &regs->status);
  virtio_mb();

  ret = virtnet_initialize(regs, irq);

  if (OK != ret)
    {
      vrterr("error: virtnet_initialize() returned %d \n", ret);
      return ret;
    }

  /* Set STATUS_FRIVER_OK */

  virtio_putreg32(virtio_getreg32(&regs->status) | VIRTIO_STATUS_DRIVER_OK,
                  &regs->status);
  virtio_mb();

  g_ninterfaces++;

  return ret;
}

#endif /* !defined(CONFIG_SCHED_WORKQUEUE) */
#endif /* CONFIG_DRIVERS_VIRTIO_NET */
