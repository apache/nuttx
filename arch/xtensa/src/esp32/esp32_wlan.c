/****************************************************************************
 * arch/xtensa/src/esp32/esp32_wlan.c
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

#ifdef CONFIG_ESP32_WIRELESS

#include <queue.h>
#include <errno.h>
#include <debug.h>
#include <crc64.h>
#include <arpa/inet.h>

#include <nuttx/nuttx.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/netdev.h>
#if defined(CONFIG_NET_PKT)
#  include <nuttx/net/pkt.h>
#endif

#include "esp32_wifi_adapter.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* WLAN station device ID */

#define WLAN_STA_DEVNO            (0)

/**
 * TX poll delay = 1 seconds.
 * CLK_TCK is the number of clock ticks per second
 */

#define WLAN_WDDELAY              (1 * CLK_TCK)

/* TX timeout = 1 minute */

#define WLAN_TXTOUT               (60 * CLK_TCK)

/* Low-priority workqueue processes RX/TX */

#define WLAN_WORK                 LPWORK

/**
 * Ethernet frame:
 *     Resource address   :   6 bytes
 *     Destination address:   6 bytes
 *     Type               :   2 bytes
 *     Payload            :   MAX 1500
 *     Checksum           :   Ignore
 *
 *     Total size         :   1514
 */

#define WLAN_BUF_SIZE             (CONFIG_NET_ETH_PKTSIZE)

/* WiFi receive buffer number */

#define WLAN_RXBUF_NUM            (CONFIG_ESP32_WLAN_RXBUF_NUM)

/**
 * Receive threshold which let the receive function to trigger a sheduler
 * to active application if possible.
 */

#ifdef CONFIG_MM_IOB
#  define IOBBUF_SIZE             (CONFIG_IOB_NBUFFERS * CONFIG_IOB_BUFSIZE)
#  if (IOBBUF_SIZE) > (WLAN_BUF_SIZE + 1)
#    define WLAN_RX_THRESHOLD     (IOBBUF_SIZE - WLAN_BUF_SIZE + 1)
#  endif
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Receive buffer */

struct wlan_rxbuf
{
  sq_entry_t    entry;          /* Queue entry */

  /* Packet data buffer */

  uint8_t       buffer[WLAN_BUF_SIZE];
  uint16_t      len;            /* Packet data length */
};

/* The wlan_priv_s encapsulates all state information for a single
 * hardware interface
 */

struct wlan_priv_s
{
  bool   ifup;                  /* true:ifup false:ifdown */

  struct wdog_s txpoll;         /* TX poll timer */
  struct wdog_s txtimeout;      /* TX timeout timer */

  struct work_s rxwork;         /* Send packet work */
  struct work_s txwork;         /* Receive packet work */
  struct work_s pollwork;       /* Poll work */
  struct work_s toutwork;       /* Send packet timeout work */

  /* This holds the information visible to the NuttX network */

  struct net_driver_s dev;

  /* TX buffer */

  uint8_t       txbuf[WLAN_BUF_SIZE];

  /* Rest data in TX buffer which needs being sent */

  uint8_t       txrst;

  /* RX buffer cache */

  struct wlan_rxbuf  rxbuf[WLAN_RXBUF_NUM];

  /* RX buffer queue */

  sq_queue_t    rxb;

  /* Free buffer queue */

  sq_queue_t    freeb;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct wlan_priv_s g_wlan_priv;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Common TX logic */

static int  wlan_transmit(FAR struct wlan_priv_s *priv);
static void wlan_rxpoll(FAR void *arg);
static int  wlan_txpoll(FAR struct net_driver_s *dev);
static void wlan_dopoll(FAR struct wlan_priv_s *priv);

/* Watchdog timer expirations */

static void wlan_txtimeout_work(FAR void *arg);
static void wlan_txtimeout_expiry(wdparm_t arg);

static void wlan_poll_work(FAR void *arg);
static void wlan_poll_expiry(wdparm_t arg);

/* NuttX callback functions */

static int wlan_ifup(struct net_driver_s *dev);
static int wlan_ifdown(struct net_driver_s *dev);

static void wlan_txavail_work(FAR void *arg);
static int wlan_txavail(struct net_driver_s *dev);

#if defined(CONFIG_NET_MCASTGROUP) || defined(CONFIG_NET_ICMPv6)
static int wlan_addmac(struct net_driver_s *dev, FAR const uint8_t *mac);
#endif

#ifdef CONFIG_NET_MCASTGROUP
static int wlan_rmmac(struct net_driver_s *dev, FAR const uint8_t *mac);
#endif

#ifdef CONFIG_NETDEV_IOCTL
static int wlan_ioctl(struct net_driver_s *dev, int cmd,
                      unsigned long arg);
#endif

static void wlan_tx_done(uint8_t ifidx, uint8_t *data,
                         uint16_t *len, bool txstatus);
static int wlan_rx_done(void *buffer, uint16_t len, void *eb);
static int esp32_net_initialize(unsigned int devno);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/**
 * Note:
 *     All TX done/RX done/Error trigger functions are not called from
 *     interrupts, this is much different from ethernet driver, including:
 *       * wlan_rx_done
 *       * wlan_tx_done
 *
 *     These functions are called in a WiFi private thread. So we just use
 *     mutex/semaphore instead of disable interrupt, if necessary.
 */

/****************************************************************************
 * Function: wlan_init_buffer
 *
 * Description:
 *   Initialize the free buffer list
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void wlan_init_buffer(struct wlan_priv_s *priv)
{
  int i;
  irqstate_t flags;

  flags = enter_critical_section();

  priv->txrst = 0;

  priv->dev.d_buf = NULL;
  priv->dev.d_len = 0;

  sq_init(&priv->freeb);
  sq_init(&priv->rxb);

  for (i = 0; i < WLAN_RXBUF_NUM; i++)
    {
      sq_addlast(&priv->rxbuf[i].entry, &priv->freeb);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Function: wlan_alloc_buffer
 *
 * Description:
 *   Allocate one buffer from the free buffer queue
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   Pointer to the allocated buffer on success; NULL on failure
 *
 ****************************************************************************/

static inline struct wlan_rxbuf *wlan_alloc_buffer(struct wlan_priv_s *priv)
{
  sq_entry_t *entry;
  irqstate_t flags;
  struct wlan_rxbuf *rxbuf = NULL;

  flags = enter_critical_section();

  entry = sq_remfirst(&priv->freeb);
  if (entry)
    {
      rxbuf = container_of(entry, struct wlan_rxbuf, entry);
    }

  leave_critical_section(flags);

  return rxbuf;
}

/****************************************************************************
 * Function: wlan_free_buffer
 *
 * Description:
 *   Insert a free Rx buffer into free queue
 *
 * Input Parameters:
 *   priv   - Reference to the driver state structure
 *   buffer - A pointer to the buffer to be freed
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void wlan_free_buffer(struct wlan_priv_s *priv,
                                    uint8_t *buffer)
{
  struct wlan_rxbuf *rxbuf;
  irqstate_t flags;

  flags = enter_critical_section();

  rxbuf = container_of(buffer, struct wlan_rxbuf, buffer);
  sq_addlast(&rxbuf->entry, &priv->freeb);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: wifi_tx_available
 *
 * Description:
 *   Check if WiFi can send data. This function will re-send rest data
 *   which was sent failed.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   true if available or false if unavailable
 *
 ****************************************************************************/

static bool wifi_tx_available(FAR struct wlan_priv_s *priv)
{
  int ret;

  if (priv->txrst)
    {
      ret = esp_wifi_sta_send_data(priv->txbuf, priv->txrst);
      if (ret)
        {
          ninfo("ERROR: Failed to transmit rest frame\n");
          return false;
        }
      else
        {
          priv->txrst = 0;
        }
    }

  return true;
}

/****************************************************************************
 * Name: wlan_transmit
 *
 * Description:
 *   Send the data to WiFi driver. If this sending fails, cache the data
 *   and re-send it when TX done callback or timer poll function triggers.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

static int wlan_transmit(FAR struct wlan_priv_s *priv)
{
  int ret;
  struct net_driver_s *dev = &priv->dev;
  void *buffer = dev->d_buf;
  uint32_t len = dev->d_len;

  if (!wifi_tx_available(priv))
    {
      return -ENOBUFS;
    }

  ret = esp_wifi_sta_send_data(buffer, len);
  if (ret)
    {
      priv->txrst = len;
      if (buffer != priv->txbuf)
        {
          memcpy(priv->txbuf, buffer, len);
        }

      wd_start(&priv->txtimeout, WLAN_TXTOUT,
               wlan_txtimeout_expiry, (uint32_t)priv);

      return -EIO;
    }
  else
    {
      priv->txrst = 0;
    }

  return OK;
}

/****************************************************************************
 * Function: wlan_recvframe
 *
 * Description:
 *   Try to receive RX buffer from RX done buffer queue.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   RX buffer if success or NULl if no buffer in queue.
 *
 ****************************************************************************/

static struct wlan_rxbuf *wlan_recvframe(FAR struct wlan_priv_s *priv)
{
  irqstate_t flags;
  sq_entry_t *entry;
  struct wlan_rxbuf *rxbuf = NULL;

  flags = enter_critical_section();

  entry = sq_remfirst(&priv->rxb);
  if (entry)
    {
      rxbuf = container_of(entry, struct wlan_rxbuf, entry);
    }

  leave_critical_section(flags);

  return rxbuf;
}

/****************************************************************************
 * Name: wlan_tx_done
 *
 * Description:
 *   WiFi TX done callback function. If this is called, it means sending
 *   next packet.
 *
 * Input Parameters:
 *   ifidx  - The interface id that the tx callback has been triggered from.
 *   data   - Pointer to the data transmitted.
 *   len    - Length of the data transmitted.
 *   status - True if data was transmitted sucessfully or false if failed.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void wlan_tx_done(uint8_t ifidx, uint8_t *data,
                         uint16_t *len, bool status)
{
  FAR struct wlan_priv_s *priv = &g_wlan_priv;

  wd_cancel(&priv->txtimeout);

  wlan_txavail(&priv->dev);
}

/****************************************************************************
 * Function: wlan_rx_done
 *
 * Description:
 *   WiFi RX done callback function. If this is called, it means receiveing
 *   packet.
 *
 * Input Parameters:
 *   buffer - WiFi received packet buffer
 *   len    - Length of received packet
 *   eb     - WiFi receive callback input eb pointer
 *
 * Returned Value:
 *   0 on success or a negated errno on failure
 *
 ****************************************************************************/

static int wlan_rx_done(void *buffer, uint16_t len, void *eb)
{
  struct wlan_rxbuf *rxbuf;
  irqstate_t flags;
  FAR struct wlan_priv_s *priv = &g_wlan_priv;

  if (!priv->ifup)
    {
      return 0;
    }

  if (len > WLAN_BUF_SIZE)
    {
      nwarn("ERROR: Wlan receive %d larger than %d\n",
             len, WLAN_BUF_SIZE);
      return -EINVAL;
    }

  rxbuf = wlan_alloc_buffer(priv);
  if (!rxbuf)
    {
      if (eb)
        {
          esp_wifi_free_eb(eb);
        }

      return -ENOBUFS;
    }

  memcpy(rxbuf->buffer, buffer, len);
  rxbuf->len = len;

  if (eb)
    {
      esp_wifi_free_eb(eb);
    }

  flags = enter_critical_section();
  sq_addlast(&rxbuf->entry, &priv->rxb);
  leave_critical_section(flags);

  if (work_available(&priv->rxwork))
    {
      work_queue(WLAN_WORK, &priv->rxwork, wlan_rxpoll, priv, 0);
    }

  return 0;
}

/****************************************************************************
 * Function: wlan_rxpoll
 *
 * Description:
 *   Try to receive packets from RX done queue and pass packets into IP
 *   stack and send packets which is from IP stack if necessary.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void wlan_rxpoll(FAR void *arg)
{
  struct wlan_rxbuf *rxbuf;
  struct eth_hdr_s *eth_hdr;
  FAR struct wlan_priv_s *priv = (FAR struct wlan_priv_s *)arg;
  FAR struct net_driver_s *dev = &priv->dev;
#ifdef WLAN_RX_THRESHOLD
  uint32_t rbytes = 0;
#endif

  /* Loop while while wlan_recvframe() successfully retrieves valid
   * Ethernet frames.
   */

  net_lock();

  while ((rxbuf = wlan_recvframe(priv)) != NULL)
    {
      dev->d_buf = rxbuf->buffer;
      dev->d_len = rxbuf->len;

#ifdef WLAN_RX_THRESHOLD
      rbytes += rxbuf->len;
#endif

#ifdef CONFIG_NET_PKT

      /* When packet sockets are enabled,
       * feed the frame into the packet tap.
       */

      pkt_input(&priv->dev);
#endif

      /* Check if the packet is a valid size for the network
       * buffer configuration (this should not happen)
       */

      if (dev->d_len > WLAN_BUF_SIZE)
        {
          nwarn("WARNING: DROPPED Too big: %d\n", dev->d_len);

          /* Free dropped packet buffer */

          if (dev->d_buf)
            {
              wlan_free_buffer(priv, dev->d_buf);
              dev->d_buf = NULL;
              dev->d_len = 0;
            }

          continue;
        }

      eth_hdr = (struct eth_hdr_s *)dev->d_buf;

      /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv4
      if (eth_hdr->type == HTONS(ETHTYPE_IP))
        {
          ninfo("IPv4 frame\n");

          /* Handle ARP on input then give the IPv4 packet to the network
           * layer
           */

          arp_ipin(&priv->dev);
          ipv4_input(&priv->dev);

          /* If the above function invocation resulted in data
           * that should be sent out on the network,
           * the field  d_len will set to a value > 0.
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

              wlan_transmit(priv);
            }
        }
      else
#endif
#ifdef CONFIG_NET_IPv6
      if (eth_hdr->type == HTONS(ETHTYPE_IP6))
        {
          ninfo("IPv6 frame\n");

          /* Give the IPv6 packet to the network layer */

          ipv6_input(&priv->dev);

          /* If the above function invocation resulted in data
           * that should be sent out on the network, the field
           * d_len will set to a value > 0.
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

              wlan_transmit(priv);
            }
        }
      else
#endif
#ifdef CONFIG_NET_ARP
      if (eth_hdr->type == htons(ETHTYPE_ARP))
        {
          ninfo("ARP frame\n");

          /* Handle ARP packet */

          arp_arpin(&priv->dev);

          /* If the above function invocation resulted in data
           * that should be sent out on the network, the field
           * d_len will set to a value > 0.
           */

          if (priv->dev.d_len > 0)
            {
              wlan_transmit(priv);
            }
        }
      else
#endif
        {
          ninfo("INFO: Dropped, Unknown type: %04x\n", eth_hdr->type);
        }

      /* We are finished with the RX buffer.  NOTE:  If the buffer is
       * re-used for transmission, the dev->d_buf field will have been
       * nullified.
       */

      if (dev->d_buf)
        {
          /* Free the receive packet buffer */

          wlan_free_buffer(priv, dev->d_buf);
          dev->d_buf = NULL;
          dev->d_len = 0;
        }

#ifdef WLAN_RX_THRESHOLD
      /**
       * If received total bytes is larger than receive threshold,
       * then do "unlock" to try to active applicantion to receive
       * data from low-level buffer of IP stack.
       */

      if (rbytes >= WLAN_RX_THRESHOLD)
        {
          net_unlock();
          rbytes = 0;
          net_lock();
        }
#endif
    }

    net_unlock();
}

/****************************************************************************
 * Name: wlan_txpoll
 *
 * Description:
 *   The transmitter is available, check if the network has any outgoing
 *   packets ready to send.  This is a callback from devif_poll().
 *   devif_poll() may be called:
 *
 *   1. When the preceding TX packet send times out and the interface is
 *      reset
 *   2. During normal TX polling
 *
 * Input Parameters:
 *   dev - Reference to the NuttX driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

static int wlan_txpoll(FAR struct net_driver_s *dev)
{
  FAR struct wlan_priv_s *priv = (FAR struct wlan_priv_s *)dev->d_private;

  DEBUGASSERT(dev->d_buf != NULL);

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (dev->d_len > 0)
    {
      /* Look up the destination MAC address and add it to the Ethernet
       * header.
       */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
      if (IFF_IS_IPv4(dev->d_flags))
#endif
        {
          arp_out(dev);
        }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
      else
#endif
        {
          neighbor_out(dev);
        }
#endif /* CONFIG_NET_IPv6 */

      int ret = wlan_transmit(priv);
      if (ret)
        {
          return -EBUSY;
        }
    }

  /* If zero is returned, the polling will continue until
   * all connections have been examined.
   */

  return OK;
}

/****************************************************************************
 * Function: wlan_dopoll
 *
 * Description:
 *   The function is called in order to perform an out-of-sequence TX poll.
 *   This is done:
 *
 *   1. When new TX data is available (wlan_txavail)
 *   2. After a TX timeout to restart the sending process
 *      (wlan_txtimeout_expiry).
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void wlan_dopoll(FAR struct wlan_priv_s *priv)
{
  FAR struct net_driver_s *dev = &priv->dev;

  if (!wifi_tx_available(priv))
    {
      return ;
    }

  dev->d_buf = priv->txbuf;

  /* If so, then poll the network for new XMIT data */

  devif_poll(dev, wlan_txpoll);

  dev->d_buf = NULL;
}

/****************************************************************************
 * Function: wlan_txtimeout_work
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

static void wlan_txtimeout_work(void *arg)
{
  struct wlan_priv_s *priv = (struct wlan_priv_s *)arg;

  net_lock();

  wlan_ifdown(&priv->dev);
  wlan_ifup(&priv->dev);

  /* Then poll for new XMIT data */

  wlan_dopoll(priv);

  net_unlock();
}

/****************************************************************************
 * Function: wlan_txtimeout_expiry
 *
 * Description:
 *   Our TX watchdog timed out.  Called from the timer callback handler.
 *   The last TX never completed.  Reset the hardware and start again.
 *
 * Input Parameters:
 *   argc - The number of available arguments
 *   arg  - The first argument
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void wlan_txtimeout_expiry(wdparm_t arg)
{
  struct wlan_priv_s *priv = (struct wlan_priv_s *)arg;

  /* Schedule to perform the TX timeout processing on the worker thread. */

  if (work_available(&priv->toutwork))
    {
      work_queue(WLAN_WORK, &priv->toutwork, wlan_txtimeout_work, priv, 0);
    }
}

/****************************************************************************
 * Name: wlan_poll_work
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
 *   The network is locked.
 *
 ****************************************************************************/

static void wlan_poll_work(FAR void *arg)
{
  int32_t delay = WLAN_WDDELAY;
  FAR struct wlan_priv_s *priv = (FAR struct wlan_priv_s *)arg;
  struct net_driver_s *dev = &priv->dev;

  /* Lock the network and serialize driver operations if necessary.
   * NOTE: Serialization is only required in the case where the driver work
   * is performed on an LP worker thread and where more than one LP worker
   * thread has been configured.
   */

  net_lock();

  /* Check if there is room in the send another TX packet.  We cannot perform
   * the TX poll if he are unable to accept another packet for transmission.
   *
   * If there is no room, we should reset the timeout value to be 1 to
   * trigger the timer as soon as possible.
   */

  if (!wifi_tx_available(priv))
    {
      delay = 1;
      goto exit;
    }

  dev->d_buf = priv->txbuf;

  /* Update TCP timing states and poll the network for new XMIT data. */

  devif_timer(&priv->dev, delay, wlan_txpoll);

  dev->d_buf = NULL;

exit:
  wd_start(&priv->txpoll, delay, wlan_poll_expiry, (wdparm_t)priv);

  net_unlock();
}

/****************************************************************************
 * Name: wlan_poll_expiry
 *
 * Description:
 *   Periodic timer handler.  Called from the timer callback handler.
 *
 * Input Parameters:
 *   argc - The number of available arguments
 *   arg  - The first argument
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void wlan_poll_expiry(wdparm_t arg)
{
  FAR struct wlan_priv_s *priv = (FAR struct wlan_priv_s *)arg;

  if (priv->ifup)
    {
      work_queue(WLAN_WORK, &priv->pollwork, wlan_poll_work, priv, 0);
    }
}

/****************************************************************************
 * Name: wlan_txavail_work
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

static void wlan_txavail_work(FAR void *arg)
{
  FAR struct wlan_priv_s *priv = (FAR struct wlan_priv_s *)arg;

  /* Lock the network and serialize driver operations if necessary.
   * NOTE: Serialization is only required in the case where the driver work
   * is performed on an LP worker thread and where more than one LP worker
   * thread has been configured.
   */

  net_lock();

  /* Ignore the notification if the interface is not yet up */

  if (priv->ifup)
    {
      /* Poll the network for new XMIT data */

      wlan_dopoll(priv);
    }

  net_unlock();
}

/****************************************************************************
 * Name: wlan_ifup
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
 ****************************************************************************/

static int wlan_ifup(FAR struct net_driver_s *dev)
{
  FAR struct wlan_priv_s *priv = (FAR struct wlan_priv_s *)dev->d_private;

#ifdef CONFIG_NET_IPv4
  ninfo("Bringing up: %d.%d.%d.%d\n",
        dev->d_ipaddr & 0xff, (dev->d_ipaddr >> 8) & 0xff,
        (dev->d_ipaddr >> 16) & 0xff, dev->d_ipaddr >> 24);
#endif
#ifdef CONFIG_NET_IPv6
  winfo("Bringing up: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        dev->d_ipv6addr[0], dev->d_ipv6addr[1], dev->d_ipv6addr[2],
        dev->d_ipv6addr[3], dev->d_ipv6addr[4], dev->d_ipv6addr[5],
        dev->d_ipv6addr[6], dev->d_ipv6addr[7]);
#endif

  net_lock();

  if (priv->ifup)
    {
      net_unlock();
      return OK;
    }

#ifdef CONFIG_NET_ICMPv6

  /* Set up IPv6 multicast address filtering */

  wlan_ipv6multicast(priv);
#endif

  wlan_init_buffer(priv);

  /* Set and activate a timer process */

  wd_start(&priv->txpoll, WLAN_WDDELAY, wlan_poll_expiry, (wdparm_t)priv);

  priv->ifup = true;

  net_unlock();

  return OK;
}

/****************************************************************************
 * Name: wlan_ifdown
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
 ****************************************************************************/

static int wlan_ifdown(FAR struct net_driver_s *dev)
{
  FAR struct wlan_priv_s *priv = (FAR struct wlan_priv_s *)dev->d_private;

  net_lock();

  if (!priv->ifup)
    {
      net_unlock();
      return OK;
    }

  /* Cancel the TX poll timer and TX timeout timers */

  wd_cancel(&priv->txpoll);
  wd_cancel(&priv->txtimeout);

  /* Mark the device "down" */

  priv->ifup = false;

  net_unlock();

  return OK;
}

/****************************************************************************
 * Name: wlan_txavail
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

static int wlan_txavail(FAR struct net_driver_s *dev)
{
  FAR struct wlan_priv_s *priv = (FAR struct wlan_priv_s *)dev->d_private;

  if (work_available(&priv->txwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(WLAN_WORK, &priv->txwork, wlan_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: wlan_addmac
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
 ****************************************************************************/

#if defined(CONFIG_NET_MCASTGROUP) || defined(CONFIG_NET_ICMPv6)
static int wlan_addmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct wlan_priv_s *priv = (FAR struct wlan_priv_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  return OK;
}
#endif

/****************************************************************************
 * Name: wlan_rmmac
 *
 * Description:
 *   NuttX Callback: Remove the specified MAC address from the
 *   hardware multicast address filtering
 *
 * Input Parameters:
 *   dev  - Reference to the NuttX driver state structure
 *   mac  - The MAC address to be removed
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NET_MCASTGROUP
static int wlan_rmmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct wlan_priv_s *priv = (FAR struct wlan_priv_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  return OK;
}
#endif

/****************************************************************************
 * Name: wlan_ipv6multicast
 *
 * Description:
 *   Configure the IPv6 multicast MAC address.
 *
 * Input Parameters:
 *   priv - A reference to the private driver state structure
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6
static void wlan_ipv6multicast(FAR struct wlan_priv_s *priv)
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

  ninfo("IPv6 Multicast: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  wlan_addmac(dev, mac);

#ifdef CONFIG_NET_ICMPv6_AUTOCONF
  /* Add the IPv6 all link-local nodes Ethernet address.  This is the
   * address that we expect to receive ICMPv6 Router Advertisement
   * packets.
   */

  wlan_addmac(dev, g_ipv6_ethallnodes.ether_addr_octet);
#endif /* CONFIG_NET_ICMPv6_AUTOCONF */

#ifdef CONFIG_NET_ICMPv6_ROUTER
  /* Add the IPv6 all link-local routers Ethernet address.  This is the
   * address that we expect to receive ICMPv6 Router Solicitation
   * packets.
   */

  wlan_addmac(dev, g_ipv6_ethallrouters.ether_addr_octet);
#endif /* CONFIG_NET_ICMPv6_ROUTER */
}
#endif /* CONFIG_NET_ICMPv6 */

/****************************************************************************
 * Name: wlan_ioctl
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
 ****************************************************************************/

#ifdef CONFIG_NETDEV_IOCTL
static int wlan_ioctl(FAR struct net_driver_s *dev,
                      int cmd,
                      unsigned long arg)
{
  int ret;
  struct iw_point *essid;
  struct iw_encode_ext *ext;
  struct iwreq *iwr = (struct iwreq *)arg;

  /* Decode and dispatch the driver-specific IOCTL command */

  switch (cmd)
    {
#ifdef CONFIG_NETDEV_PHY_IOCTL
#ifdef CONFIG_ARCH_PHY_INTERRUPT
      case SIOCMIINOTIFY: /* Set up for PHY event notifications */
        {
          struct mii_ioctl_notify_s *req = (struct mii_ioctl_notify_s *)arg;
          ret = esp_wifi_notify_subscribe(req->pid, &req->event);
          if (ret)
            {
              nerr("ERROR: Failed to subscribe event\n");
            }
        }
        break;
#endif
#endif

      case SIOCSIWENCODEEXT:
        {
          ext = iwr->u.encoding.pointer;
          ret = esp_wifi_set_password(ext->key, ext->key_len);
          if (ret)
            {
              nerr("ERROR: Failed to set password\n");
            }
        }
        break;
      case SIOCSIWESSID:
        {
          iwr = (struct iwreq *)arg;
          essid = &iwr->u.essid;
          ret = esp_wifi_set_ssid(essid->pointer, essid->length);
          if (ret)
            {
              nerr("ERROR: Failed to set SSID\n");
              break;
            }

          ret = esp_wifi_connect_internal();
          if (ret)
            {
              nerr("ERROR: Failed to start connecting\n");
              break;
            }
        }
        break;
      case SIOCSIWMODE:
        ret = OK;
        break;
      case SIOCSIWAUTH:
        ret = OK;
        break;
      case SIOCSIWFREQ:
        ret = OK;
        break;
      default:
        nerr("ERROR: Unrecognized IOCTL command: %d\n", cmd);
        ret = -ENOTTY;  /* Special return value for this case */
        break;
    }

  return ret;
}
#endif  /* CONFIG_NETDEV_IOCTL */

/****************************************************************************
 * Name: esp32_net_initialize
 *
 * Description:
 *   Initialize the esp32 driver
 *
 * Input Parameters:
 *   devno - The device number.
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int esp32_net_initialize(unsigned int devno)
{
  int ret;
  uint8_t eth_mac[6];
  FAR struct wlan_priv_s *priv;

  /* Get the interface structure associated with this interface number. */

  priv = &g_wlan_priv;

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct wlan_priv_s));

  priv->dev.d_ifup    = wlan_ifup;     /* I/F down callback */
  priv->dev.d_ifdown  = wlan_ifdown;   /* I/F up (new IP address) callback */
  priv->dev.d_txavail = wlan_txavail;  /* New TX data callback */
#ifdef CONFIG_NET_MCASTGROUP
  priv->dev.d_addmac  = wlan_addmac;   /* Add multicast MAC address */
  priv->dev.d_rmmac   = wlan_rmmac;    /* Remove multicast MAC address */
#endif
#ifdef CONFIG_NETDEV_IOCTL
  priv->dev.d_ioctl   = wlan_ioctl;    /* Handle network IOCTL commands */
#endif

  /* Used to recover private state from dev */

  priv->dev.d_private = (void *)&g_wlan_priv;

  /* Create a watchdog for timing polling for and timing of transmissions */

  esp_wifi_sta_read_mac(eth_mac);

  memcpy(priv->dev.d_mac.ether.ether_addr_octet, eth_mac, sizeof(eth_mac));

  ninfo("%02X:%02X:%02X:%02X:%02X:%02X \r\n",
        eth_mac[0], eth_mac[1], eth_mac[2],
        eth_mac[3], eth_mac[4], eth_mac[5]);

  /* Put the interface in the down state. */

  wlan_ifdown(&priv->dev);

  ret = netdev_register(&priv->dev, NET_LL_IEEE80211);
  if (ret)
    {
      nerr("ERROR: Initialization of Ethernet block failed: %d\n", ret);
      return ret;
    }

  ret = esp_wifi_adapter_init();
  if (ret)
    {
      nerr("ERROR: Initialize WiFi adapter error: %d\n", ret);
      netdev_unregister(&priv->dev);
      return ret;
    }

  ret = esp_wifi_sta_register_recv_cb(wlan_rx_done);
  if (ret)
    {
      DEBUGASSERT(0);
    }

  ret = esp_wifi_sta_register_txdone_cb(wlan_tx_done);
  if (ret)
    {
      DEBUGASSERT(0);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_wlan_sta_initialize
 *
 * Description:
 *   Initialize the esp32 WLAN station netcard driver
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32_wlan_sta_initialize(void)
{
  return esp32_net_initialize(WLAN_STA_DEVNO);
}

#endif  /* CONFIG_ESP32_WIRELESS */
