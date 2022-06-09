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

#ifdef CONFIG_ESP32_WIFI

#include <queue.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <crc64.h>
#include <arpa/inet.h>

#include <nuttx/nuttx.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/spinlock.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/netdev.h>
#if defined(CONFIG_NET_PKT)
#  include <nuttx/net/pkt.h>
#endif

#include "esp32_wlan.h"
#include "esp32_wifi_utils.h"
#include "esp32_wifi_adapter.h"
#include "esp32_systemreset.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* TX timeout = 1 minute */

#define WLAN_TXTOUT               (60 * CLK_TCK)

/* Low-priority work queue processes RX/TX */

#define WLAN_WORK                 LPWORK

/* Ethernet frame:
 *     Resource address   :   6 bytes
 *     Destination address:   6 bytes
 *     Type               :   2 bytes
 *     Payload            :   MAX 1500
 *     Checksum           :   Ignore
 *
 *     Total size         :   1514
 */

#define WLAN_BUF_SIZE             (CONFIG_NET_ETH_PKTSIZE)

/* WLAN packet buffer number */

#define WLAN_PKTBUF_NUM           (CONFIG_ESP32_WLAN_PKTBUF_NUM)

/* Receive threshold which allows the receive function to trigger a scheduler
 * to activate the application if possible.
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

/* WLAN packet buffer */

struct wlan_pktbuf
{
  sq_entry_t    entry;          /* Queue entry */

  /* Packet data buffer */

  uint8_t       buffer[WLAN_BUF_SIZE];
  uint16_t      len;            /* Packet data length */
};

/* WLAN operations */

struct wlan_ops
{
  int (*start)(void);
  int (*send)(void *pdata, size_t n);
  int (*essid)(struct iwreq *iwr, bool set);
  int (*bssid)(struct iwreq *iwr, bool set);
  int (*passwd)(struct iwreq *iwr, bool set);
  int (*mode)(struct iwreq *iwr, bool set);
  int (*auth)(struct iwreq *iwr, bool set);
  int (*freq)(struct iwreq *iwr, bool set);
  int (*bitrate)(struct iwreq *iwr, bool set);
  int (*txpower)(struct iwreq *iwr, bool set);
  int (*channel)(struct iwreq *iwr, bool set);
  int (*country)(struct iwreq *iwr, bool set);
  int (*rssi)(struct iwreq *iwr, bool set);
  int (*connect)(void);
  int (*disconnect)(void);
  int (*event)(pid_t pid, struct sigevent *event);
  int (*stop)(void);
};

/* The wlan_priv_s encapsulates all state information for a single
 * hardware interface
 */

struct wlan_priv_s
{
  int    ref;                   /* Referernce count */

  bool   ifup;                  /* true:ifup false:ifdown */

  struct wdog_s txtimeout;      /* TX timeout timer */

  struct work_s rxwork;         /* Send packet work */
  struct work_s txwork;         /* Receive packet work */
  struct work_s pollwork;       /* Poll work */
  struct work_s toutwork;       /* Send packet timeout work */

  const struct wlan_ops *ops;   /* WLAN operations */

  /* This holds the information visible to the NuttX network */

  struct net_driver_s dev;

  /* Packet buffer cache */

  struct wlan_pktbuf  pktbuf[WLAN_PKTBUF_NUM];

  /* RX packet queue */

  sq_queue_t    rxb;

  /* TX ready packet queue */

  sq_queue_t    txb;

  /* Free packet buffer queue */

  sq_queue_t    freeb;

  /* Device specific lock */

  spinlock_t    lock;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Reference count of register Wi-Fi handler */

static uint8_t g_callback_register_ref = 0;

static struct wlan_priv_s g_wlan_priv[ESP32_WLAN_DEVS];

#ifdef ESP32_WLAN_HAS_STA
static const struct wlan_ops g_sta_ops =
{
  .start      = esp_wifi_sta_start,
  .send       = esp_wifi_sta_send_data,
  .essid      = esp_wifi_sta_essid,
  .bssid      = esp_wifi_sta_bssid,
  .passwd     = esp_wifi_sta_password,
  .mode       = esp_wifi_sta_mode,
  .auth       = esp_wifi_sta_auth,
  .freq       = esp_wifi_sta_freq,
  .bitrate    = esp_wifi_sta_bitrate,
  .txpower    = esp_wifi_sta_txpower,
  .channel    = esp_wifi_sta_channel,
  .country    = esp_wifi_sta_country,
  .rssi       = esp_wifi_sta_rssi,
  .connect    = esp_wifi_sta_connect,
  .disconnect = esp_wifi_sta_disconnect,
  .event      = esp_wifi_notify_subscribe,
  .stop       = esp_wifi_sta_stop
};
#endif

#ifdef ESP32_WLAN_HAS_SOFTAP
static const struct wlan_ops g_softap_ops =
{
  .start      = esp_wifi_softap_start,
  .send       = esp_wifi_softap_send_data,
  .essid      = esp_wifi_softap_essid,
  .bssid      = esp_wifi_softap_bssid,
  .passwd     = esp_wifi_softap_password,
  .mode       = esp_wifi_softap_mode,
  .auth       = esp_wifi_softap_auth,
  .freq       = esp_wifi_softap_freq,
  .bitrate    = esp_wifi_softap_bitrate,
  .txpower    = esp_wifi_softap_txpower,
  .channel    = esp_wifi_softap_channel,
  .country    = esp_wifi_softap_country,
  .rssi       = esp_wifi_softap_rssi,
  .connect    = esp_wifi_softap_connect,
  .disconnect = esp_wifi_softap_disconnect,
  .event      = esp_wifi_notify_subscribe,
  .stop       = esp_wifi_softap_stop
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Common TX logic */

static void wlan_transmit(struct wlan_priv_s *priv);
static void wlan_rxpoll(void *arg);
static int  wlan_txpoll(struct net_driver_s *dev);
static void wlan_dopoll(struct wlan_priv_s *priv);

/* Watchdog timer expirations */

static void wlan_txtimeout_work(void *arg);
static void wlan_txtimeout_expiry(wdparm_t arg);

/* NuttX callback functions */

static int wlan_ifup(struct net_driver_s *dev);
static int wlan_ifdown(struct net_driver_s *dev);

static void wlan_txavail_work(void *arg);
static int wlan_txavail(struct net_driver_s *dev);

#if defined(CONFIG_NET_MCASTGROUP) || defined(CONFIG_NET_ICMPv6)
static int wlan_addmac(struct net_driver_s *dev, const uint8_t *mac);
#endif

#ifdef CONFIG_NET_MCASTGROUP
static int wlan_rmmac(struct net_driver_s *dev, const uint8_t *mac);
#endif

#ifdef CONFIG_NETDEV_IOCTL
static int wlan_ioctl(struct net_driver_s *dev, int cmd,
                      unsigned long arg);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Note:
 *     All TX done/RX done/Error trigger functions are not called from
 *     interrupts, this is much different from ethernet driver, including:
 *       * wlan_rx_done
 *       * wlan_tx_done
 *
 *     These functions are called in a Wi-Fi private thread. So we just use
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

  flags = spin_lock_irqsave(&priv->lock);

  priv->dev.d_buf = NULL;
  priv->dev.d_len = 0;

  sq_init(&priv->freeb);
  sq_init(&priv->rxb);
  sq_init(&priv->txb);

  for (i = 0; i < WLAN_PKTBUF_NUM; i++)
    {
      sq_addlast(&priv->pktbuf[i].entry, &priv->freeb);
    }

  spin_unlock_irqrestore(&priv->lock, flags);
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

static inline struct wlan_pktbuf *wlan_alloc_buffer(struct wlan_priv_s *priv)
{
  sq_entry_t *entry;
  irqstate_t flags;
  struct wlan_pktbuf *pktbuf = NULL;

  flags = spin_lock_irqsave(&priv->lock);

  entry = sq_remfirst(&priv->freeb);
  if (entry)
    {
      pktbuf = container_of(entry, struct wlan_pktbuf, entry);
    }

  spin_unlock_irqrestore(&priv->lock, flags);

  return pktbuf;
}

/****************************************************************************
 * Function: wlan_free_buffer
 *
 * Description:
 *   Insert a free Rx buffer into the free queue
 *
 * Input Parameters:
 *   priv   - Reference to the driver state structure
 *   buffer - A pointer to the packet buffer to be freed
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void wlan_free_buffer(struct wlan_priv_s *priv,
                                    uint8_t *buffer)
{
  struct wlan_pktbuf *pktbuf;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);

  pktbuf = container_of(buffer, struct wlan_pktbuf, buffer);
  sq_addlast(&pktbuf->entry, &priv->freeb);

  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Function: wlan_cache_txpkt_tail
 *
 * Description:
 *   Cache packet from dev->d_buf into tail of TX ready queue.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void wlan_cache_txpkt_tail(struct wlan_priv_s *priv)
{
  struct wlan_pktbuf *pktbuf;
  irqstate_t flags;
  struct net_driver_s *dev = &priv->dev;

  pktbuf = container_of(dev->d_buf, struct wlan_pktbuf, buffer);
  pktbuf->len = dev->d_len;

  flags = spin_lock_irqsave(&priv->lock);
  sq_addlast(&pktbuf->entry, &priv->txb);
  spin_unlock_irqrestore(&priv->lock, flags);

  dev->d_buf = NULL;
  dev->d_len = 0;
}

/****************************************************************************
 * Function: wlan_add_txpkt_head
 *
 * Description:
 *   Add packet into head of TX ready queue.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void wlan_add_txpkt_head(struct wlan_priv_s *priv,
                                       struct wlan_pktbuf *pktbuf)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);
  sq_addfirst(&pktbuf->entry, &priv->txb);
  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Function: wlan_recvframe
 *
 * Description:
 *   Try to receive RX packet from RX done packet queue.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   RX packet if success or NULl if no packet in queue.
 *
 ****************************************************************************/

static struct wlan_pktbuf *wlan_recvframe(struct wlan_priv_s *priv)
{
  irqstate_t flags;
  sq_entry_t *entry;
  struct wlan_pktbuf *pktbuf = NULL;

  flags = spin_lock_irqsave(&priv->lock);

  entry = sq_remfirst(&priv->rxb);
  if (entry)
    {
      pktbuf = container_of(entry, struct wlan_pktbuf, entry);
    }

  spin_unlock_irqrestore(&priv->lock, flags);

  return pktbuf;
}

/****************************************************************************
 * Function: wlan_txframe
 *
 * Description:
 *   Try to receive TX buffer from TX ready buffer queue.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   TX packets buffer if success or NULL if no packet in queue.
 *
 ****************************************************************************/

static struct wlan_pktbuf *wlan_txframe(struct wlan_priv_s *priv)
{
  irqstate_t flags;
  sq_entry_t *entry;
  struct wlan_pktbuf *pktbuf = NULL;

  flags = spin_lock_irqsave(&priv->lock);

  entry = sq_remfirst(&priv->txb);
  if (entry)
    {
      pktbuf = container_of(entry, struct wlan_pktbuf, entry);
    }

  spin_unlock_irqrestore(&priv->lock, flags);

  return pktbuf;
}

/****************************************************************************
 * Name: wlan_transmit
 *
 * Description:
 *   Try to send all TX packets in TX ready queue to Wi-Fi driver. If this
 *    sending fails, then breaks loop and returns.
 *
 * Input Parameters:
 *   priv - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void wlan_transmit(struct wlan_priv_s *priv)
{
  struct wlan_pktbuf *pktbuf;
  int ret;

  while ((pktbuf = wlan_txframe(priv)))
    {
      ret = priv->ops->send(pktbuf->buffer, pktbuf->len);
      if (ret == -ENOMEM)
        {
          wlan_add_txpkt_head(priv, pktbuf);
          wd_start(&priv->txtimeout, WLAN_TXTOUT,
                   wlan_txtimeout_expiry, (uint32_t)priv);
          break;
        }
      else
        {
          if (ret < 0)
            {
              nwarn("WARN: Failed to send pkt, ret: %d\n", ret);
            }

          wlan_free_buffer(priv, pktbuf->buffer);
        }
    }
}

/****************************************************************************
 * Name: wlan_tx_done
 *
 * Description:
 *   Wi-Fi TX done callback function. If this is called, it means sending
 *   next packet.
 *
 * Input Parameters:
 *   priv   - Reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void wlan_tx_done(struct wlan_priv_s *priv)
{
  wd_cancel(&priv->txtimeout);

  wlan_txavail(&priv->dev);
}

/****************************************************************************
 * Function: wlan_rx_done
 *
 * Description:
 *   Wi-Fi RX done callback function. If this is called, it means receiving
 *   packet.
 *
 * Input Parameters:
 *   priv   - Reference to the driver state structure
 *   buffer - Wi-Fi received packet buffer
 *   len    - Length of received packet
 *   eb     - Wi-Fi receive callback input eb pointer
 *
 * Returned Value:
 *   0 on success or a negated errno on failure
 *
 ****************************************************************************/

static int wlan_rx_done(struct wlan_priv_s *priv, void *buffer,
                        uint16_t len, void *eb)
{
  struct wlan_pktbuf *pktbuf;
  irqstate_t flags;
  int ret = 0;

  if (!priv->ifup)
    {
      goto out;
    }

  if (len > WLAN_BUF_SIZE)
    {
      nwarn("ERROR: Wlan receive %d larger than %d\n",
             len, WLAN_BUF_SIZE);
      ret = -EINVAL;
      goto out;
    }

  pktbuf = wlan_alloc_buffer(priv);
  if (!pktbuf)
    {
      ret = -ENOBUFS;
      goto out;
    }

  memcpy(pktbuf->buffer, buffer, len);
  pktbuf->len = len;

  if (eb)
    {
      esp_wifi_free_eb(eb);
    }

  flags = spin_lock_irqsave(&priv->lock);
  sq_addlast(&pktbuf->entry, &priv->rxb);
  spin_unlock_irqrestore(&priv->lock, flags);

  if (work_available(&priv->rxwork))
    {
      work_queue(WLAN_WORK, &priv->rxwork, wlan_rxpoll, priv, 0);
    }

  return 0;

out:
  if (eb)
    {
      esp_wifi_free_eb(eb);
    }

  return ret;
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

static void wlan_rxpoll(void *arg)
{
  struct wlan_pktbuf *pktbuf;
  struct eth_hdr_s *eth_hdr;
  struct wlan_priv_s *priv = (struct wlan_priv_s *)arg;
  struct net_driver_s *dev = &priv->dev;
#ifdef WLAN_RX_THRESHOLD
  uint32_t rbytes = 0;
#endif

  /* Try to send all cached TX packets for TX ack and so on */

  wlan_transmit(priv);

  /* Loop while while wlan_recvframe() successfully retrieves valid
   * Ethernet frames.
   */

  net_lock();

  while ((pktbuf = wlan_recvframe(priv)) != NULL)
    {
      dev->d_buf = pktbuf->buffer;
      dev->d_len = pktbuf->len;

#ifdef WLAN_RX_THRESHOLD
      rbytes += pktbuf->len;
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

              wlan_cache_txpkt_tail(priv);
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

              wlan_cache_txpkt_tail(priv);
            }
        }
      else
#endif
#ifdef CONFIG_NET_ARP
      if (eth_hdr->type == HTONS(ETHTYPE_ARP))
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
              wlan_cache_txpkt_tail(priv);
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

  /* Try to send all cached TX packets */

  wlan_transmit(priv);

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
 *   1. When the preceding TX packets send times out and the interface is
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

static int wlan_txpoll(struct net_driver_s *dev)
{
  struct wlan_pktbuf *pktbuf;
  struct wlan_priv_s *priv = (struct wlan_priv_s *)dev->d_private;

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

      wlan_cache_txpkt_tail(priv);

      pktbuf = wlan_alloc_buffer(priv);
      if (!pktbuf)
        {
          return -ENOMEM;
        }

      dev->d_buf = pktbuf->buffer;
      dev->d_len = WLAN_BUF_SIZE;
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

static void wlan_dopoll(struct wlan_priv_s *priv)
{
  struct net_driver_s *dev = &priv->dev;
  struct wlan_pktbuf *pktbuf;
  uint8_t *txbuf;
  int ret;

  pktbuf = wlan_alloc_buffer(priv);
  if (!pktbuf)
    {
      return ;
    }

  dev->d_buf = pktbuf->buffer;
  dev->d_len = WLAN_BUF_SIZE;

  /* Try to let TCP/IP to send all packets to netcard driver */

  do
    {
      txbuf = dev->d_buf;
      ret = devif_poll(dev, wlan_txpoll);
    }
  while ((ret == 0) &&
         (dev->d_buf != txbuf));

  if (dev->d_buf)
    {
      wlan_free_buffer(priv, dev->d_buf);

      dev->d_buf = NULL;
      dev->d_len = 0;
    }

  /* Try to send all cached TX packets */

  wlan_transmit(priv);
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

  /* Try to send all cached TX packets */

  wlan_transmit(priv);

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

static void wlan_txavail_work(void *arg)
{
  struct wlan_priv_s *priv = (struct wlan_priv_s *)arg;

  /* Try to send all cached TX packets even if net is down */

  wlan_transmit(priv);

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

static int wlan_ifup(struct net_driver_s *dev)
{
  int ret;
  struct wlan_priv_s *priv = (struct wlan_priv_s *)dev->d_private;

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

  net_lock();

  if (priv->ifup)
    {
      net_unlock();
      return OK;
    }

  ret = priv->ops->start();
  if (ret < 0)
    {
      net_unlock();
      nerr("ERROR: Failed to start Wi-Fi ret=%d\n", ret);
      return ret;
    }

#ifdef CONFIG_NET_ICMPv6

  /* Set up IPv6 multicast address filtering */

  wlan_ipv6multicast(priv);
#endif

  wlan_init_buffer(priv);

  priv->ifup = true;
  if (g_callback_register_ref == 0)
    {
      ret = esp32_register_shutdown_handler(esp_wifi_stop_callback);
      if (ret < 0)
        {
          nwarn("WARN: Failed to register handler ret=%d\n", ret);
        }
    }

  ++g_callback_register_ref;
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

static int wlan_ifdown(struct net_driver_s *dev)
{
  int ret;
  struct wlan_priv_s *priv = (struct wlan_priv_s *)dev->d_private;

  net_lock();

  if (!priv->ifup)
    {
      net_unlock();
      return OK;
    }

  /* Cancel the TX poll timer and TX timeout timers */

  wd_cancel(&priv->txtimeout);

  /* Mark the device "down" */

  priv->ifup = false;

  ret = priv->ops->stop();
  if (ret < 0)
    {
      nerr("ERROR: Failed to stop Wi-Fi ret=%d\n", ret);
    }

  --g_callback_register_ref;
  if (g_callback_register_ref == 0)
    {
      ret = esp32_unregister_shutdown_handler(esp_wifi_stop_callback);
      if (ret < 0)
        {
          nwarn("WARN: Failed to unregister handler ret=%d\n", ret);
        }
    }

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

static int wlan_txavail(struct net_driver_s *dev)
{
  struct wlan_priv_s *priv = (struct wlan_priv_s *)dev->d_private;

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
static int wlan_addmac(struct net_driver_s *dev, const uint8_t *mac)
{
  struct wlan_priv_s *priv = (struct wlan_priv_s *)dev->d_private;

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
static int wlan_rmmac(struct net_driver_s *dev, const uint8_t *mac)
{
  struct wlan_priv_s *priv = (struct wlan_priv_s *)dev->d_private;

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
static void wlan_ipv6multicast(struct wlan_priv_s *priv)
{
  struct net_driver_s *dev;
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
static int wlan_ioctl(struct net_driver_s *dev,
                      int cmd,
                      unsigned long arg)
{
  int ret;
  struct iwreq *iwr = (struct iwreq *)arg;
  struct wlan_priv_s *priv = (struct wlan_priv_s *)dev->d_private;
  const struct wlan_ops *ops = priv->ops;

  /* Decode and dispatch the driver-specific IOCTL command */

  switch (cmd)
    {
#ifdef CONFIG_NETDEV_PHY_IOCTL
#ifdef CONFIG_ARCH_PHY_INTERRUPT
      case SIOCMIINOTIFY: /* Set up for PHY event notifications */
        {
          struct mii_ioctl_notify_s *req = (struct mii_ioctl_notify_s *)arg;
          ret = ops->event(req->pid, &req->event);
          if (ret < 0)
            {
              nerr("ERROR: Failed to subscribe event\n");
            }
        }
        break;
#endif
#endif

      case SIOCSIWENCODEEXT:
        ret = ops->passwd(iwr, true);

        break;

      case SIOCGIWENCODEEXT:
        ret = ops->passwd(iwr, false);
        break;

      case SIOCSIWESSID:
        if ((iwr->u.essid.flags == IW_ESSID_ON) ||
            (iwr->u.essid.flags == IW_ESSID_DELAY_ON))
          {
            ret = ops->essid(iwr, true);
            if (ret < 0)
              {
                break;
              }

            if (iwr->u.essid.flags == IW_ESSID_ON)
              {
                ret = ops->connect();
              }
          }
        else
          {
            ret = ops->disconnect();
          }

        break;

      case SIOCGIWESSID:    /* Get ESSID */
        ret = ops->essid(iwr, false);
        break;

      case SIOCSIWAP:       /* Set access point MAC addresses */
        if (iwr->u.ap_addr.sa_data[0] != 0 &&
            iwr->u.ap_addr.sa_data[1] != 0 &&
            iwr->u.ap_addr.sa_data[2] != 0)
          {
            ret = ops->bssid(iwr, true);
            if (ret < 0)
              {
                nerr("ERROR: Failed to set BSSID\n");
                break;
              }

            ret = ops->connect();
            if (ret < 0)
              {
                nerr("ERROR: Failed to connect\n");
                break;
              }
          }
        else
          {
            ret = ops->disconnect();
            if (ret < 0)
              {
                nerr("ERROR: Failed to connect\n");
                break;
              }
          }

        break;

      case SIOCGIWAP:       /* Get access point MAC addresses */
        ret = ops->bssid(iwr, false);
        break;

      case SIOCSIWSCAN:
        ret = esp_wifi_start_scan(iwr);
        break;

      case SIOCGIWSCAN:
        ret = esp_wifi_get_scan_results(iwr);
        break;

      case SIOCSIWCOUNTRY:  /* Set country code */
        ret = ops->country(iwr, true);
        break;

      case SIOCGIWSENS:    /* Get sensitivity (dBm) */
        ret = ops->rssi(iwr, false);
        break;

      case SIOCSIWMODE:     /* Set operation mode */
        ret = ops->mode(iwr, true);
        break;

      case SIOCGIWMODE:     /* Get operation mode */
        ret = ops->mode(iwr, false);
        break;

      case SIOCSIWAUTH:    /* Set authentication mode params */
        ret = ops->auth(iwr, true);
        break;

      case SIOCGIWAUTH:    /* Get authentication mode params */
        ret = ops->auth(iwr, false);
        break;

      case SIOCSIWFREQ:     /* Set channel/frequency (MHz) */
        ret = ops->freq(iwr, true);
        break;

      case SIOCGIWFREQ:     /* Get channel/frequency (MHz) */
        ret = ops->freq(iwr, false);
        break;

      case SIOCSIWRATE:     /* Set default bit rate (Mbps) */
        wlwarn("WARNING: SIOCSIWRATE not implemented\n");
        ret = -ENOSYS;
        break;

      case SIOCGIWRATE:     /* Get default bit rate (Mbps) */
        ret = ops->bitrate(iwr, false);
        break;

      case SIOCSIWTXPOW:    /* Set transmit power (dBm) */
        ret = ops->txpower(iwr, true);
        break;

      case SIOCGIWTXPOW:    /* Get transmit power (dBm) */
        ret = ops->txpower(iwr, false);
        break;

      case SIOCGIWRANGE:    /* Get range of parameters */
        ret = ops->channel(iwr, false);
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
 *   devno    - The device number
 *   mac_addr - MAC address
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static int esp32_net_initialize(int devno, uint8_t *mac_addr,
                                const struct wlan_ops *ops)
{
  int ret;
  struct wlan_priv_s *priv;
  struct net_driver_s *netdev;

  priv = &g_wlan_priv[devno];
  if (priv->ref)
    {
      priv->ref++;
      return OK;
    }

  netdev = &priv->dev;

  /* Initialize the driver structure */

  memset(priv, 0, sizeof(struct wlan_priv_s));

  netdev->d_ifup    = wlan_ifup;     /* I/F down callback */
  netdev->d_ifdown  = wlan_ifdown;   /* I/F up (new IP address) callback */
  netdev->d_txavail = wlan_txavail;  /* New TX data callback */
#ifdef CONFIG_NET_MCASTGROUP
  netdev->d_addmac  = wlan_addmac;   /* Add multicast MAC address */
  netdev->d_rmmac   = wlan_rmmac;    /* Remove multicast MAC address */
#endif
#ifdef CONFIG_NETDEV_IOCTL
  netdev->d_ioctl   = wlan_ioctl;    /* Handle network IOCTL commands */
#endif

  /* Used to recover private state from dev */

  netdev->d_private = (void *)priv;

  memcpy(netdev->d_mac.ether.ether_addr_octet, mac_addr, 6);

  ret = netdev_register(netdev, NET_LL_IEEE80211);
  if (ret < 0)
    {
      nerr("ERROR: Initialization of Ethernet block failed: %d\n", ret);
      return ret;
    }

  priv->ops = ops;

  priv->ref++;

  ninfo("INFO: Initialize Wi-Fi adapter No.%d success\n", devno);

  return OK;
}

/****************************************************************************
 * Function: wlan_sta_rx_done
 *
 * Description:
 *   Wi-Fi station RX done callback function. If this is called, it means
 *   station receiveing packet.
 *
 * Input Parameters:
 *   buffer - Wi-Fi received packet buffer
 *   len    - Length of received packet
 *   eb     - Wi-Fi receive callback input eb pointer
 *
 * Returned Value:
 *   0 on success or a negated errno on failure
 *
 ****************************************************************************/

#ifdef ESP32_WLAN_HAS_STA
static int wlan_sta_rx_done(void *buffer, uint16_t len, void *eb)
{
  struct wlan_priv_s *priv = &g_wlan_priv[ESP32_WLAN_STA_DEVNO];

  return wlan_rx_done(priv, buffer, len, eb);
}

/****************************************************************************
 * Name: wlan_sta_tx_done
 *
 * Description:
 *   Wi-Fi station TX done callback function. If this is called, it means
 *   station sending next packet.
 *
 * Input Parameters:
 *   ifidx  - The interface id that the tx callback has been triggered from.
 *   data   - Pointer to the data transmitted.
 *   len    - Length of the data transmitted.
 *   status - True if data was transmitted successfully or false if failed.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void wlan_sta_tx_done(uint8_t *data, uint16_t *len, bool status)
{
  struct wlan_priv_s *priv = &g_wlan_priv[ESP32_WLAN_STA_DEVNO];

  wlan_tx_done(priv);
}
#endif

/****************************************************************************
 * Function: wlan_softap_rx_done
 *
 * Description:
 *   Wi-Fi softAP RX done callback function. If this is called, it means
 *   softAP receiveing packet.
 *
 * Input Parameters:
 *   buffer - Wi-Fi received packet buffer
 *   len    - Length of received packet
 *   eb     - Wi-Fi receive callback input eb pointer
 *
 * Returned Value:
 *   0 on success or a negated errno on failure
 *
 ****************************************************************************/

#ifdef ESP32_WLAN_HAS_SOFTAP
static int wlan_softap_rx_done(void *buffer, uint16_t len, void *eb)
{
  struct wlan_priv_s *priv = &g_wlan_priv[ESP32_WLAN_SOFTAP_DEVNO];

  return wlan_rx_done(priv, buffer, len, eb);
}

/****************************************************************************
 * Name: wlan_softap_tx_done
 *
 * Description:
 *   Wi-Fi softAP TX done callback function. If this is called, it means
 *   softAP sending next packet.
 *
 * Input Parameters:
 *   ifidx  - The interface id that the tx callback has been triggered from.
 *   data   - Pointer to the data transmitted.
 *   len    - Length of the data transmitted.
 *   status - True if data was transmitted successfully or false if failed.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void wlan_softap_tx_done(uint8_t *data, uint16_t *len, bool status)
{
  struct wlan_priv_s *priv = &g_wlan_priv[ESP32_WLAN_SOFTAP_DEVNO];

  wlan_tx_done(priv);
}
#endif

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

#ifdef ESP32_WLAN_HAS_STA
int esp32_wlan_sta_initialize(void)
{
  int ret;
  uint8_t eth_mac[6];

  ret = esp_wifi_adapter_init();
  if (ret < 0)
    {
      nerr("ERROR: Initialize Wi-Fi adapter error: %d\n", ret);
      return ret;
    }

  ret = esp_wifi_sta_read_mac(eth_mac);
  if (ret < 0)
    {
      nerr("ERROR: Failed to read MAC address\n");
      return ret;
    }

  ninfo("Wi-Fi station MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
        eth_mac[0], eth_mac[1], eth_mac[2],
        eth_mac[3], eth_mac[4], eth_mac[5]);

  ret = esp_wifi_scan_init();
  if (ret < 0)
    {
      nerr("ERROR: Initialize Wi-Fi scan parameter error: %d\n", ret);
      return ret;
    }

  ret = esp32_net_initialize(ESP32_WLAN_STA_DEVNO, eth_mac, &g_sta_ops);
  if (ret < 0)
    {
      nerr("ERROR: Failed to initialize net\n");
      return ret;
    }

  ret = esp_wifi_sta_register_recv_cb(wlan_sta_rx_done);
  if (ret < 0)
    {
      nerr("ERROR: Failed to register RX callback\n");
      return ret;
    }

  esp_wifi_sta_register_txdone_cb(wlan_sta_tx_done);

  ninfo("INFO: Initialize Wi-Fi station success net\n");

  return OK;
}
#endif

/****************************************************************************
 * Name: esp32_wlan_softap_initialize
 *
 * Description:
 *   Initialize the esp32 WLAN softAP netcard driver
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

#ifdef ESP32_WLAN_HAS_SOFTAP
int esp32_wlan_softap_initialize(void)
{
  int ret;
  uint8_t eth_mac[6];

  ret = esp_wifi_adapter_init();
  if (ret < 0)
    {
      nerr("ERROR: Initialize Wi-Fi adapter error: %d\n", ret);
      return ret;
    }

  ret = esp_wifi_softap_read_mac(eth_mac);
  if (ret < 0)
    {
      nerr("ERROR: Failed to read MAC address\n");
      return ret;
    }

  ninfo("Wi-Fi softAP MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
        eth_mac[0], eth_mac[1], eth_mac[2],
        eth_mac[3], eth_mac[4], eth_mac[5]);

  ret = esp_wifi_scan_init();
  if (ret < 0)
    {
      nerr("ERROR: Initialize Wi-Fi scan parameter error: %d\n", ret);
      return ret;
    }

  ret = esp32_net_initialize(ESP32_WLAN_SOFTAP_DEVNO, eth_mac,
                             &g_softap_ops);
  if (ret < 0)
    {
      nerr("ERROR: Failed to initialize net\n");
      return ret;
    }

  ret = esp_wifi_softap_register_recv_cb(wlan_softap_rx_done);
  if (ret < 0)
    {
      nerr("ERROR: Failed to register RX callback\n");
      return ret;
    }

  esp_wifi_softap_register_txdone_cb(wlan_softap_tx_done);

  ninfo("INFO: Initialize Wi-Fi softAP net success\n");

  return OK;
}
#endif

#endif  /* CONFIG_ESP32_WIFI */
