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
#include <debug.h>
#include <queue.h>
#include <errno.h>
#include <arpa/inet.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/mii.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/netdev.h>
#include <crc64.h>

#if defined(CONFIG_NET_PKT)
#  include <nuttx/net/pkt.h>
#endif

#include <nuttx/net/net.h>
#include <nuttx/kmalloc.h>
#include <debug.h>

#include "esp32_wifi_adapter.h"

#include <arch/board/board.h>

#ifdef CONFIG_ESP32_WIRELESS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STA_DEVNO                  0

/* TX poll delay = 1 seconds.
 * CLK_TCK is the number of clock ticks per second
 */

#define ESP_WDDELAY                (1*CLK_TCK)
#define ESPWORK                    LPWORK

/* TX timeout = 1 minute */

#define ESP_TXTIMEOUT              (60*CLK_TCK)
#define DEFAULT_SCAN_LIST_SIZE     2

/* Add 4 to the configured buffer size to account for the 2 byte checksum
 * memory needed at the end of the maximum size packet.  Buffer sizes must
 * be an even multiple of 4, 8, or 16 bytes (depending on buswidth).  We
 * will use the 16-byte alignment in all cases.
 */

#define OPTIMAL_ETH_BUFSIZE ((CONFIG_NET_ETH_PKTSIZE + 4 + 15) & ~15)

#ifndef CONFIG_ESP_ETH_BUFSIZE
#  define CONFIG_ESP_ETH_BUFSIZE   OPTIMAL_ETH_BUFSIZE
#endif

#ifndef CONFIG_ESP_ETH_NTXDESC
#  define CONFIG_ESP_ETH_NTXDESC   4
#endif

/* We need at least one more free buffer than transmit buffers */

#define ESP_ETH_NFREEBUFFERS (CONFIG_ESP_ETH_NTXDESC+1)
#define ETH_MAX_LEN                1518

/* This is a helper pointer for accessing the contents of wlan header */

#define BUF ((struct eth_hdr_s *)priv->esp_dev.d_buf)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The esp_dev_s encapsulates all state information for a single
 * hardware interface
 */

struct esp_dev_s
{
  bool   ifup;                  /* true:ifup false:ifdown */
  struct wdog_s esp_txpoll;     /* TX poll timer */
  struct wdog_s esp_txtimeout;  /* TX timeout timer */
  struct work_s esp_irqwork;    /* For deferring interrupt work to the work queue */
  struct work_s esp_pollwork;   /* For deferring poll work to the work queue */

  /* This holds the information visible to the NuttX network */

  struct net_driver_s esp_dev;
  sq_queue_t           freeb;    /* The free buffer list */

  /* Buffer allocations */

  uint8_t alloc[CONFIG_ESP_ETH_NTXDESC*CONFIG_ESP_ETH_BUFSIZE];
  uint8_t rxbuf[ETH_MAX_LEN];
  uint32_t rx_len;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct esp_dev_s s_esp32_dev;
static bool g_tx_ready = false;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Free buffer management */

static void esp_initbuffer(FAR struct esp_dev_s *priv);
static inline uint8_t *esp_allocbuffer(FAR struct esp_dev_s *priv);
static inline void esp_freebuffer(FAR struct esp_dev_s *priv,
                                  uint8_t *buffer);
static inline bool esp_isfreebuffer(FAR struct esp_dev_s *priv);

/* Common TX logic */

static int  esp_transmit(FAR struct esp_dev_s *priv);
static void esp_receive(FAR struct esp_dev_s *priv);
static int  esp_txpoll(FAR struct net_driver_s *dev);
static void esp_rxpoll(FAR void *arg);
static void esp_dopoll(FAR struct esp_dev_s *priv);
static void esp_netdev_notify_rx(FAR struct esp_dev_s *priv,
                                 void *buffer, uint16_t len);
static int esp_sta_input(void *buffer, uint16_t len, void *eb);

/* Watchdog timer expirations */

static void esp_txtimeout_work(FAR void *arg);
static void esp_txtimeout_expiry(wdparm_t arg);

static void esp_poll_work(FAR void *arg);
static void esp_poll_expiry(wdparm_t arg);

/* NuttX callback functions */

static int  esp_ifup(struct net_driver_s *dev);
static int  esp_ifdown(struct net_driver_s *dev);

static void esp_txavail_work(FAR void *arg);
static int  esp_txavail(struct net_driver_s *dev);

#if defined(CONFIG_NET_MCASTGROUP) || defined(CONFIG_NET_ICMPv6)
static int  esp_addmac(struct net_driver_s *dev, FAR const uint8_t *mac);
#endif
#ifdef CONFIG_NET_MCASTGROUP
static int  esp_rmmac(struct net_driver_s *dev, FAR const uint8_t *mac);
#endif
#ifdef CONFIG_NETDEV_IOCTL
static int  esp_ioctl(struct net_driver_s *dev, int cmd,
              unsigned long arg);
#endif
static int esp32_net_initialize(unsigned int devno);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/**
 * Note:
 *     All TX done/RX done/Error trigger functions are not called from
 *     interrupts, this is much different from ethernet driver, including:
 *       * esp_sta_input
 *
 *     These functions are called in a WiFi private thread. So we just use
 *     mutex/semaphore instead of disable interrupt, if necessary.
 */

/****************************************************************************
 * Function: esp_initbuffer
 *
 * Description:
 *   Initialize the free buffer list.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp_initbuffer(FAR struct esp_dev_s *priv)
{
  uint8_t *buffer;
  int i;

  /* Initialize the head of the free buffer list */

  sq_init(&priv->freeb);

  /* Add all of the pre-allocated buffers to the free buffer list */

  for (i = 0, buffer = priv->alloc; i < ESP_ETH_NFREEBUFFERS;
       i++, buffer += CONFIG_ESP_ETH_BUFSIZE)
    {
      sq_addlast((FAR sq_entry_t *)buffer, &priv->freeb);
    }
}

/****************************************************************************
 * Function: esp_allocbuffer
 *
 * Description:
 *   Allocate one buffer from the free buffer list.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   Pointer to the allocated buffer on success; NULL on failure
 *
 ****************************************************************************/

static inline uint8_t *esp_allocbuffer(FAR struct esp_dev_s *priv)
{
  /* Allocate a buffer by returning the head of the free buffer list */

  return (uint8_t *)sq_remfirst(&priv->freeb);
}

/****************************************************************************
 * Function: esp_freebuffer
 *
 * Description:
 *   Return a buffer to the free buffer list.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *   buffer - A pointer to the buffer to be freed
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void esp_freebuffer(FAR struct esp_dev_s *priv,
                                  uint8_t *buffer)
{
  /* Free the buffer by adding it to the end of the free buffer list */

  sq_addlast((FAR sq_entry_t *)buffer, &priv->freeb);
}

/****************************************************************************
 * Function: esp_isfreebuffer
 *
 * Description:
 *   Return TRUE if the free buffer list is not empty.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   True if there are one or more buffers in the free buffer list;
 *   false if the free buffer list is empty
 *
 ****************************************************************************/

static inline bool esp_isfreebuffer(FAR struct esp_dev_s *priv)
{
  /* Return TRUE if the free buffer list is not empty */

  return !sq_empty(&priv->freeb);
}

/****************************************************************************
 * Name: esp_transmit
 *
 * Description:
 *   Start hardware transmission.  Called either from TX process or
 *   from watchdog based polling.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

static int esp_transmit(FAR struct esp_dev_s *priv)
{
  int ret = 0;
  uint8_t *buffer;
  uint32_t buffer_len;

  /* Set up all but the last TX descriptor */

  buffer = priv->esp_dev.d_buf;
  buffer_len = priv->esp_dev.d_len;
  ret = esp_wifi_sta_send_data(buffer, buffer_len);

  if (ret != 0)
    {
      wlerr("ERROR: Failed to transmit frame\n");
      (void)wd_start(&priv->esp_txtimeout, ESP_TXTIMEOUT,
                     esp_txtimeout_expiry, (uint32_t)priv);
      return -EIO;
    }

  return OK;
}

/****************************************************************************
 * Function: esp_recvframe
 *
 * Description:
 *   It scans the RX descriptors of the received frame.
 *
 *   NOTE: This function will silently discard any packets containing errors.
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   OK if a packet was successfully returned; -EAGAIN if there are no
 *   further packets available
 *
 ****************************************************************************/

static int esp_recvframe(FAR struct esp_dev_s *priv)
{
  struct net_driver_s *dev = &priv->esp_dev;
  uint8_t *buffer;
  uint32_t buffer_len = 0;
  buffer = dev->d_buf;
  buffer_len = dev->d_len;

  /* Check if there are free buffers.  We cannot receive new frames in this
   * design unless there is at least one free buffer.
   */

  if (!esp_isfreebuffer(priv))
    {
      wlerr("ERROR: No free buffers\n");
      return -ENOMEM;
    }

  /* Check if any errors are reported in the frame */

  if (buffer == NULL || buffer_len == 0)
    {
      return -EAGAIN;
    }

  return OK;
}

/****************************************************************************
 * Function: esp_receive
 *
 * Description:
 *   An event was received indicating the availability of a new RX packet
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp_receive(FAR struct esp_dev_s *priv)
{
  struct net_driver_s *dev = &priv->esp_dev;

  /* Loop while while esp_recvframe() successfully retrieves valid
   * Ethernet frames.
   */

  while (esp_recvframe(priv) == OK)
    {
#ifdef CONFIG_NET_PKT

      /* When packet sockets are enabled,
       * feed the frame into the packet tap.
       */

      pkt_input(&priv->esp_dev);
#endif

      /* Check if the packet is a valid size for the network
       * buffer configuration (this should not happen)
       */

      if (dev->d_len > CONFIG_NET_ETH_PKTSIZE)
        {
          wlwarn("WARNING: DROPPED Too big: %d\n", dev->d_len);

          /* Free dropped packet buffer */

          if (dev->d_buf)
            {
              esp_freebuffer(priv, dev->d_buf);
              dev->d_buf = NULL;
              dev->d_len = 0;
            }

          continue;
        }

      /* We only accept IP packets of the configured type and ARP packets */

#ifdef CONFIG_NET_IPv4
      if (BUF->type == HTONS(ETHTYPE_IP))
        {
          wlinfo("IPv4 frame\n");

          /* Handle ARP on input then give the IPv4 packet to the network
           * layer
           */

          arp_ipin(&priv->esp_dev);
          ipv4_input(&priv->esp_dev);

          /* If the above function invocation resulted in data
           * that should be sent out on the network,
           * the field  d_len will set to a value > 0.
           */

          if (priv->esp_dev.d_len > 0)
            {
              /* Update the Ethernet header with the correct MAC address */

#ifdef CONFIG_NET_IPv6
              if (IFF_IS_IPv4(priv->esp_dev.d_flags))
#endif
                {
                  arp_out(&priv->esp_dev);
                }
#ifdef CONFIG_NET_IPv6
              else
                {
                  neighbor_out(&priv->esp_dev);
                }
#endif

              /* And send the packet */

              esp_transmit(priv);
            }
        }
      else
#endif
#ifdef CONFIG_NET_IPv6
      if (BUF->type == HTONS(ETHTYPE_IP6))
        {
          wlinfo("Iv6 frame\n");

          /* Give the IPv6 packet to the network layer */

          ipv6_input(&priv->esp_dev);

          /* If the above function invocation resulted in data
           * that should be sent out on the network, the field
           * d_len will set to a value > 0.
           */

          if (priv->esp_dev.d_len > 0)
            {
              /* Update the Ethernet header with the correct MAC address */

#ifdef CONFIG_NET_IPv4
              if (IFF_IS_IPv4(priv->esp_dev.d_flags))
                {
                  arp_out(&priv->esp_dev);
                }
              else
#endif
#ifdef CONFIG_NET_IPv6
                {
                  neighbor_out(&priv->esp_dev);
                }
#endif

              /* And send the packet */

              esp_transmit(priv);
            }
        }
      else
#endif
#ifdef CONFIG_NET_ARP
      if (BUF->type == htons(ETHTYPE_ARP))
        {
          wlinfo("ARP frame\n");

          /* Handle ARP packet */

          arp_arpin(&priv->esp_dev);

          /* If the above function invocation resulted in data
           * that should be sent out on the network, the field
           * d_len will set to a value > 0.
           */

          if (priv->esp_dev.d_len > 0)
            {
              esp_transmit(priv);
            }
        }
      else
#endif
        {
          wlinfo("INFO: Dropped, Unknown type: %04x\n", BUF->type);
        }

      /* We are finished with the RX buffer.  NOTE:  If the buffer is
       * re-used for transmission, the dev->d_buf field will have been
       * nullified.
       */

      if (dev->d_buf)
        {
          /* Free the receive packet buffer */

          esp_freebuffer(priv, dev->d_buf);
          dev->d_buf = NULL;
          dev->d_len = 0;
        }
    }
}

/****************************************************************************
 * Name: esp_txpoll
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

static int esp_txpoll(FAR struct net_driver_s *dev)
{
  FAR struct esp_dev_s *priv = (FAR struct esp_dev_s *)dev->d_private;

  DEBUGASSERT(priv->esp_dev.d_buf != NULL);

  /* If the polling resulted in data that should be sent out on the network,
   * the field d_len is set to a value > 0.
   */

  if (priv->esp_dev.d_len > 0)
    {
      /* Look up the destination MAC address and add it to the Ethernet
       * header.
       */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
      if (IFF_IS_IPv4(priv->esp_dev.d_flags))
#endif
        {
          arp_out(&priv->esp_dev);
        }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
      else
#endif
        {
          neighbor_out(&priv->esp_dev);
        }
#endif /* CONFIG_NET_IPv6 */

      if (!devif_loopback(&priv->esp_dev))
        {
          /* Send the packet */

          int ret = esp_transmit(priv);
          if (ret != OK)
            {
              wlerr("TX failed\r\n");
              return -EBUSY;
            }
        }
    }

  /* If zero is returned, the polling will continue until
   * all connections have been examined.
   */

  return OK;
}

/****************************************************************************
 * Name: esp_rxpoll
 *
 * Description:
 *   Process RX frames
 *
 * Input Parameters:
 *   arg - context of device to use
 *
 * Returned Value:
 *   OK on success
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static void esp_rxpoll(FAR void *arg)
{
  FAR struct esp_dev_s *priv = (FAR struct esp_dev_s *)arg;

  if (priv->esp_dev.d_buf == NULL)
    {
      priv->esp_dev.d_buf = priv->rxbuf;
      priv->esp_dev.d_len = priv->rx_len;
    }
  else
    {
      wlinfo("priv->esp_dev.d_buf != NULL");
      return;
    }

  /* Lock the network and serialize driver operations if necessary.
   * NOTE: Serialization is only required in the case where the driver work
   * is performed on an LP worker thread and where more than one LP worker
   * thread has been configured.
   */

  net_lock();

  esp_receive(priv);

  if (priv->esp_dev.d_buf)
    {
      priv->esp_dev.d_buf = NULL;
      memset(priv->rxbuf, 0x0, sizeof(priv->rxbuf));
      priv->rx_len = 0;
    }

  net_unlock();
}

/****************************************************************************
 * Function: esp_dopoll
 *
 * Description:
 *   The function is called in order to perform an out-of-sequence TX poll.
 *   This is done:
 *
 *   1. When new TX data is available (esp_txavail), and
 *   2. After a TX timeout to restart the sending process
 *      (esp_txtimeout_expiry).
 *
 * Input Parameters:
 *   priv  - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp_dopoll(FAR struct esp_dev_s *priv)
{
  FAR struct net_driver_s *dev = &priv->esp_dev;

  if (g_tx_ready == true)
    {
      /* Check if there is room in the hardware to
       * hold another outgoing packet.
       */

      dev->d_buf = esp_allocbuffer(priv);

      /* We can't poll if we have no buffers */

      if (dev->d_buf)
        {
          /* If so, then poll the network for new XMIT data */

          (void)devif_poll(dev, esp_txpoll);

          /* We will, most likely end up with a buffer to be freed.
           * But it might not be the same one that we allocated above.
           */

          if (dev->d_buf)
            {
              esp_freebuffer(priv, dev->d_buf);
              dev->d_buf = NULL;
            }
        }
      else
        {
          wlerr("Alloc buffer error");
        }
    }
  else
    {
      wlwarn("Tx is not ready");
    }
}

/****************************************************************************
 * Name: esp_netdev_notify_rx
 *
 * Description:
 *   Notify callback called when RX frame is available
 *
 * Input Parameters:
 *   priv   - Reference to the driver state structure
 *   buffer - Receive buffer
 *   len    - Length of receive buffer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp_netdev_notify_rx(FAR struct esp_dev_s *priv,
                                 void *buffer, uint16_t len)
{
  struct esp_dev_s *priv_dev = priv;

  memcpy(priv_dev->rxbuf, buffer, len);
  priv_dev->rx_len = len;
  work_queue(ESPWORK, &priv_dev->esp_irqwork, esp_rxpoll, priv_dev, 0);
}

/****************************************************************************
 * Function: esp_sta_input
 *
 * Description:
 *   This function should be called when a packet is ready to be read
 *   from the interface. It uses the function low_level_input() that
 *   should handle the actual reception of bytes from the network
 *   interface. Then the type of the received packet is determined and
 *   the appropriate input function is called.
 *
 * Input Parameters:
 *   buffer - WiFi receive buffer
 *   len    - Length of receive buffer
 *   eb     - WiFi receive callback input eb pointer
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

static int esp_sta_input(void *buffer, uint16_t len, void *eb)
{
  FAR struct esp_dev_s *priv = &s_esp32_dev;

  if (!buffer || (priv->ifup == false))
    {
      if (eb)
        {
          esp_wifi_free_eb(eb);
        }

      return -1;
    }
  else
    {
      esp_netdev_notify_rx(priv, buffer, len);
    }

  esp_wifi_free_eb(eb);
  return 0;
}

/****************************************************************************
 * Function: esp_txtimeout_work
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

static void esp_txtimeout_work(void *arg)
{
  struct esp_dev_s *priv = (struct esp_dev_s *)arg;

  /* Reset the hardware.  Just take the interface down, then back up again. */

  net_lock();
  esp_ifdown(&priv->esp_dev);
  esp_ifup(&priv->esp_dev);

  /* Then poll for new XMIT data */

  esp_dopoll(priv);
  net_unlock();
}

/****************************************************************************
 * Function: esp_txtimeout_expiry
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

static void esp_txtimeout_expiry(wdparm_t arg)
{
  struct esp_dev_s *priv = (struct esp_dev_s *)arg;
  wlinfo("Timeout!\n");

  /* Schedule to perform the TX timeout processing on the worker thread. */

  DEBUGASSERT(work_available(&priv->esp_irqwork));
  work_queue(ESPWORK, &priv->esp_irqwork, esp_txtimeout_work, priv, 0);
}

/****************************************************************************
 * Name: esp_poll_work
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

static void esp_poll_work(FAR void *arg)
{
  FAR struct esp_dev_s *priv = (FAR struct esp_dev_s *)arg;
  struct net_driver_s *dev  = &priv->esp_dev;

  /* Lock the network and serialize driver operations if necessary.
   * NOTE: Serialization is only required in the case where the driver work
   * is performed on an LP worker thread and where more than one LP worker
   * thread has been configured.
   */

  net_lock();

  /* Check if there is room in the send another TX packet.  We cannot perform
   * the TX poll if he are unable to accept another packet for transmission.
   */

  if (g_tx_ready == true)
    {
      dev->d_buf = esp_allocbuffer(priv);

      /* We can't poll if we have no buffers */

      if (dev->d_buf)
        {
          /* Update TCP timing states and poll
           * the network for new XMIT data.
           */

          (void)devif_timer(&priv->esp_dev, ESP_WDDELAY, esp_txpoll);

          /* We will, most likely end up with a buffer to be freed.
           * But it might not be the same one that we allocated above.
           */

          if (dev->d_buf)
            {
              esp_freebuffer(priv, dev->d_buf);
              dev->d_buf = NULL;
            }
        }
      else
        {
          wlerr("ERROR: Failed to TX pkt");
        }
    }

  wd_start(&priv->esp_txpoll, ESP_WDDELAY, esp_poll_expiry,
          (wdparm_t)priv);
  net_unlock();
}

/****************************************************************************
 * Name: esp_poll_expiry
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

static void esp_poll_expiry(wdparm_t arg)
{
  FAR struct esp_dev_s *priv = (FAR struct esp_dev_s *)arg;

  work_queue(ESPWORK, &priv->esp_pollwork, esp_poll_work, priv, 0);
}

/****************************************************************************
 * Name: esp_ifup
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

static int esp_ifup(FAR struct net_driver_s *dev)
{
  FAR struct esp_dev_s *priv = (FAR struct esp_dev_s *)dev->d_private;

#ifdef CONFIG_NET_IPv4
  wlinfo("Bringing up: %d.%d.%d.%d\n",
        dev->d_ipaddr & 0xff, (dev->d_ipaddr >> 8) & 0xff,
        (dev->d_ipaddr >> 16) & 0xff, dev->d_ipaddr >> 24);
#endif
#ifdef CONFIG_NET_IPv6
  winfo("Bringing up: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        dev->d_ipv6addr[0], dev->d_ipv6addr[1], dev->d_ipv6addr[2],
        dev->d_ipv6addr[3], dev->d_ipv6addr[4], dev->d_ipv6addr[5],
        dev->d_ipv6addr[6], dev->d_ipv6addr[7]);
#endif

#ifdef CONFIG_NET_ICMPv6

  /* Set up IPv6 multicast address filtering */

  esp_ipv6multicast(priv);
#endif

  /* Initialize the free buffer list */

  esp_initbuffer(priv);

  /* Set and activate a timer process */

  (void)wd_start(&priv->esp_txpoll, ESP_WDDELAY, esp_poll_expiry,
                (wdparm_t)priv);

  priv->ifup = true;

  return OK;
}

/****************************************************************************
 * Name: esp_ifdown
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

static int esp_ifdown(FAR struct net_driver_s *dev)
{
  FAR struct esp_dev_s *priv = (FAR struct esp_dev_s *)dev->d_private;
  irqstate_t flags;

  flags = enter_critical_section();

  /* Cancel the TX poll timer and TX timeout timers */

  wd_cancel(&priv->esp_txpoll);
  wd_cancel(&priv->esp_txtimeout);

  /* Mark the device "down" */

  priv->ifup = false;
  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: esp_txavail_work
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

static void esp_txavail_work(FAR void *arg)
{
  FAR struct esp_dev_s *priv = (FAR struct esp_dev_s *)arg;

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

      esp_dopoll(priv);
    }

  net_unlock();
}

/****************************************************************************
 * Name: esp_txavail
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

static int esp_txavail(FAR struct net_driver_s *dev)
{
  FAR struct esp_dev_s *priv = (FAR struct esp_dev_s *)dev->d_private;

  if (work_available(&priv->esp_pollwork))
    {
      /* Schedule to serialize the poll on the worker thread. */

      work_queue(ESPWORK, &priv->esp_pollwork, esp_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: esp_addmac
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
static int esp_addmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct esp_dev_s *priv = (FAR struct esp_dev_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  return OK;
}
#endif

/****************************************************************************
 * Name: esp_rmmac
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
static int esp_rmmac(FAR struct net_driver_s *dev, FAR const uint8_t *mac)
{
  FAR struct esp_dev_s *priv = (FAR struct esp_dev_s *)dev->d_private;

  /* Add the MAC address to the hardware multicast routing table */

  return OK;
}
#endif

/****************************************************************************
 * Name: esp_ipv6multicast
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
static void esp_ipv6multicast(FAR struct esp_dev_s *priv)
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

  dev    = &priv->esp_dev;
  tmp16  = dev->d_ipv6addr[6];
  mac[2] = 0xff;
  mac[3] = tmp16 >> 8;

  tmp16  = dev->d_ipv6addr[7];
  mac[4] = tmp16 & 0xff;
  mac[5] = tmp16 >> 8;

  wlinfo("IPv6 Multicast: %02x:%02x:%02x:%02x:%02x:%02x\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  esp_addmac(dev, mac);

#ifdef CONFIG_NET_ICMPv6_AUTOCONF
  /* Add the IPv6 all link-local nodes Ethernet address.  This is the
   * address that we expect to receive ICMPv6 Router Advertisement
   * packets.
   */

  esp_addmac(dev, g_ipv6_ethallnodes.ether_addr_octet);
#endif /* CONFIG_NET_ICMPv6_AUTOCONF */

#ifdef CONFIG_NET_ICMPv6_ROUTER
  /* Add the IPv6 all link-local routers Ethernet address.  This is the
   * address that we expect to receive ICMPv6 Router Solicitation
   * packets.
   */

  esp_addmac(dev, g_ipv6_ethallrouters.ether_addr_octet);
#endif /* CONFIG_NET_ICMPv6_ROUTER */
}
#endif /* CONFIG_NET_ICMPv6 */

/****************************************************************************
 * Name: esp_ioctl
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
static int esp_ioctl(FAR struct net_driver_s *dev,
                     int cmd, unsigned long arg)
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
              wlerr("ERROR: Failed to subscribe event\n");
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
              wlerr("ERROR: Failed to set password\n");
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
              wlerr("ERROR: Failed to set SSID\n");
              break;
            }

          ret = esp_wifi_connect_internal();
          if (ret)
            {
              wlerr("ERROR: Failed to start connecting\n");
              break;
            }
          else
            {
              g_tx_ready = true;
            }
        }
        break;
      case SIOCSIWMODE:
        ret = OK;
        break;
      case SIOCSIWAUTH:
        ret = OK;
        break;
      default:
        wlerr("ERROR: Unrecognized IOCTL command: %d\n", cmd);
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
  FAR struct esp_dev_s *priv;

  /* Get the interface structure associated with this interface number. */

  priv = &s_esp32_dev;

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct esp_dev_s));

  priv->esp_dev.d_ifup    = esp_ifup;     /* I/F down callback */
  priv->esp_dev.d_ifdown  = esp_ifdown;   /* I/F up (new IP address) callback */
  priv->esp_dev.d_txavail = esp_txavail;  /* New TX data callback */
#ifdef CONFIG_NET_MCASTGROUP
  priv->esp_dev.d_addmac  = esp_addmac;   /* Add multicast MAC address */
  priv->esp_dev.d_rmmac   = esp_rmmac;    /* Remove multicast MAC address */
#endif
#ifdef CONFIG_NETDEV_IOCTL
  priv->esp_dev.d_ioctl   = esp_ioctl;    /* Handle network IOCTL commands */
#endif

  /* Used to recover private state from dev */

  priv->esp_dev.d_private = (void *)&s_esp32_dev;

  /* Create a watchdog for timing polling for and timing of transmissions */

  /* Initialize network stack interface buffer */

  priv->esp_dev.d_buf     = NULL;
  g_tx_ready = false;

  assert(esp_wifi_adapter_init() == 0);
  esp_wifi_sta_register_recv_cb(esp_sta_input);

  /* Register the device with the OS so that socket IOCTLs can be performed */

  esp_wifi_sta_read_mac(priv->esp_dev.d_mac.ether.ether_addr_octet);
  memcpy(eth_mac, priv->esp_dev.d_mac.ether.ether_addr_octet,
         sizeof(eth_mac));
  wlinfo("%02X:%02X:%02X:%02X:%02X:%02X \r\n", eth_mac[0], eth_mac[1],
                       eth_mac[2], eth_mac[3], eth_mac[4], eth_mac[5]);

  /* Put the interface in the down state. */

  ret = esp_ifdown(&priv->esp_dev);
  if (ret < 0)
    {
      wlerr("ERROR: Initialization of Ethernet block failed: %d\n", ret);
      return ret;
    }

  (void)netdev_register(&s_esp32_dev.esp_dev, NET_LL_IEEE80211);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_wlan_initialize
 *
 * Description:
 *   Initialize the esp32 wlan driver
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int esp32_wlan_initialize(void)
{
  return esp32_net_initialize(STA_DEVNO);
}

#endif  /* CONFIG_ESP32_WIRELESS */
