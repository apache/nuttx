/****************************************************************************
 * arch/arm/src/common/ameba/ameba_wlan.c
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
 * NuttX WLAN (STA) network device for Ameba WHC SoCs.
 *
 * The NP runs the WiFi MAC/PHY and delivers/accepts 802.3 (Ethernet)
 * frames over the WHC IPC, so this is a plain Ethernet-style netdev
 * (NET_LL_IEEE80211).
 * The data path crosses to the SDK-header side through a thin byte-buffer
 * ABI (the SDK's lwIP/pbuf headers collide with NuttX's, so they cannot
 * share a
 * translation unit):
 *
 *   TX:  ameba_wlan_txpoll -> ameba_wifi_txframe()  [ameba_wifi_depend.c]
 *   RX:  netif_adapter_wifi_recv_whc() -> ameba_wlan_rxframe()  (this file)
 *
 * Follows the standard NuttX Ethernet-style netdev (devif) RX/TX idiom.
 * IW (wireless-extension) ioctls drive scan / connect / SoftAP via wapi.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <arpa/inet.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/wqueue.h>
#include <nuttx/kmalloc.h>
#include <nuttx/net/netdev.h>
#include <nuttx/mm/iob.h>
#ifdef CONFIG_NET_PKT
#  include <nuttx/net/pkt.h>
#endif
#ifdef CONFIG_NETDEV_IOCTL
#  include <nuttx/wireless/wireless.h>
#endif

#include "ameba_wlan.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AMEBA_WLAN_STA_IDX  0       /* STA interface index on the NP           */
#define AMEBA_WLAN_AP_IDX   1       /* SoftAP interface index on the NP        */
#define AMEBA_WLAN_DEVNUM   AMEBA_WLAN_STA_IDX

/* Per-frame work buffer size (Ethernet MTU + headers + guard). */

#define AMEBA_WLAN_BUFSIZE  (CONFIG_NET_ETH_PKTSIZE + CONFIG_NET_GUARDSIZE)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ameba_wlan_s
{
  struct net_driver_s dev;         /* Interface understood by the network   */
  struct work_s       pollwork;    /* For deferring TX poll work to LPWORK   */
  struct work_s       rxwork;      /* For deferring RX processing to LPWORK   */
  struct iob_queue_s  rxq;         /* Frames queued by the WHC RX callback    */
  bool                bifup;       /* true: ifup; false: ifdown               */
  uint8_t             devnum;      /* NP interface index                      */

  /* Association state collected across IW (wapi) ioctls. */

  uint8_t             mode;        /* IW_MODE_INFRA (STA) or IW_MODE_MASTER   */
  uint8_t             channel;     /* SoftAP channel hint (1..165, 0=default) */
  uint8_t             ssid[AMEBA_WLAN_SSID_MAXLEN + 1];
  uint8_t             ssid_len;
  uint8_t             psk[64];     /* WPA passphrase (8..63 chars)            */
  uint8_t             psk_len;

  /* Cached scan results (filled on SIOCSIWSCAN, read on SIOCGIWSCAN). */

  struct ameba_scan_ap scan[AMEBA_WLAN_MAX_SCAN_AP];
  int                 scan_count;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct ameba_wlan_s g_ameba_wlan;

/* TX work buffer (network stack fills it during devif_poll) and RX work
 * buffer (incoming frame copied in, with room for an in-place reply).
 */

static uint8_t g_txbuf[AMEBA_WLAN_BUFSIZE]
                 aligned_data(4);
static uint8_t g_rxbuf[AMEBA_WLAN_BUFSIZE]
                 aligned_data(4);

/****************************************************************************
 * External Function Prototypes (SDK side, libameba_wifi.a)
 ****************************************************************************/

extern int ameba_wifi_txframe(int idx, const void *buf, unsigned int len,
                              unsigned char is_special);
extern int ameba_wifi_get_mac(int idx, unsigned char *mac);
extern int ameba_wifi_scan_start(void);
extern int ameba_wifi_scan_results(struct ameba_scan_ap *out, int max);
extern int ameba_wifi_connect(const unsigned char *ssid, int ssid_len,
                              const unsigned char *pw, int pw_len);
extern int ameba_wifi_disconnect(void);
extern int ameba_wifi_start_ap(const unsigned char *ssid, int ssid_len,
                               const unsigned char *pw, int pw_len,
                               int channel);
extern int ameba_wifi_stop_ap(void);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ameba_wlan_is_dhcp
 *
 * Description:
 *   Detect a DHCP frame (IPv4 UDP ports 67/68) so the NP can prioritise it,
 *   mirroring the SDK lwIP adapter's is_special_pkt logic.
 *
 ****************************************************************************/

static unsigned char ameba_wlan_is_dhcp(const uint8_t *frame,
                                        unsigned int len)
{
  const struct eth_hdr_s *eth = (const struct eth_hdr_s *)frame;

  if (len >= ETH_HDRLEN + 24 && eth->type == HTONS(ETHTYPE_IP))
    {
      const uint8_t *ip = frame + ETH_HDRLEN;

      /* UDP src/dst ports at IP+20 (no-options header):
       * 67=server, 68=client.
       */

      if ((ip[21] == 68 && ip[23] == 67) || (ip[21] == 67 && ip[23] == 68))
        {
          return 1;
        }
    }

  return 0;
}

/****************************************************************************
 * Name: ameba_wlan_transmit
 *
 * Description:
 *   Hand the frame currently in dev->d_buf/d_len to the NP over WHC.
 *   Called with the network locked.
 *
 ****************************************************************************/

static int ameba_wlan_transmit(struct ameba_wlan_s *priv)
{
  struct net_driver_s *dev = &priv->dev;
  int ret;

  NETDEV_TXPACKETS(dev);

  ret = ameba_wifi_txframe(priv->devnum, dev->d_buf, dev->d_len,
                           ameba_wlan_is_dhcp(dev->d_buf, dev->d_len));
  ninfo("tx len=%u type=0x%04x ret=%d\n",
        (unsigned)dev->d_len,
        (unsigned)((dev->d_buf[12] << 8) | dev->d_buf[13]), ret);
  if (ret != 0)
    {
      NETDEV_TXERRORS(dev);
      return -EIO;
    }

  NETDEV_TXDONE(dev);
  return OK;
}

/****************************************************************************
 * Name: ameba_wlan_txpoll
 *
 * Description:
 *   devif_poll() callback: transmit one prepared packet.  whc_host_send()
 *   copies the frame, so the work buffer is immediately reusable -> return 0
 *   to keep draining the queued packets.
 *
 ****************************************************************************/

static int ameba_wlan_txpoll(struct net_driver_s *dev)
{
  struct ameba_wlan_s *priv = (struct ameba_wlan_s *)dev->d_private;

  if (dev->d_len > 0)
    {
      ameba_wlan_transmit(priv);
    }

  return 0;
}

/****************************************************************************
 * Name: ameba_wlan_reply
 *
 * Description:
 *   Send the response, if any, that the network stack produced while
 *   handling a received frame.
 *
 *   NuttX processes RX packets IN PLACE and SYNCHRONOUSLY: ipv4_input() /
 *   ipv6_input() / arp_input() consume the frame in dev->d_buf and, when
 *   the protocol requires an immediate answer, build that answer into the
 *   SAME buffer and set dev->d_len to its length.  So after *_input()
 *   returns:
 *     - d_len > 0  -> the stack left a reply to send (e.g. an ARP reply to
 *                     an ARP request, an ICMP echo reply to a ping, a TCP
 *                     ACK) -> transmit it.
 *     - d_len == 0 -> nothing to send (e.g. a UDP datagram delivered to a
 *                     socket) -> do nothing.
 *
 *   This is NOT an echo/loopback of the received frame; it is the stack's
 *   own response.  It is the standard NuttX devif RX-then-maybe-TX idiom
 *   used by every NuttX Ethernet driver.
 *
 ****************************************************************************/

static void ameba_wlan_reply(struct ameba_wlan_s *priv)
{
  if (priv->dev.d_len > 0)
    {
      ameba_wlan_transmit(priv);
    }
}

/****************************************************************************
 * Name: ameba_wlan_rxdispatch
 *
 * Description:
 *   Feed one staged frame (already in dev->d_buf/d_len) into the network
 *   stack by EtherType and transmit any in-place reply it produced (see
 *   ameba_wlan_reply): <proto>_input() may turn the RX buffer into a
 *   response (ARP/ICMP/TCP) with d_len set.  Called from ameba_wlan_rxwork
 *   with the network locked.
 *
 ****************************************************************************/

static void ameba_wlan_rxdispatch(struct ameba_wlan_s *priv)
{
  struct net_driver_s *dev = &priv->dev;
  struct eth_hdr_s *eth = (struct eth_hdr_s *)dev->d_buf;

  NETDEV_RXPACKETS(dev);

#ifdef CONFIG_NET_PKT
  pkt_input(dev);
#endif

  if (eth->type == HTONS(ETHTYPE_IP))
    {
#ifdef CONFIG_NET_IPv4
      NETDEV_RXIPV4(dev);
      ipv4_input(dev);
      ameba_wlan_reply(priv);
#else
      NETDEV_RXDROPPED(dev);
#endif
    }
  else if (eth->type == HTONS(ETHTYPE_IP6))
    {
#ifdef CONFIG_NET_IPv6
      NETDEV_RXIPV6(dev);
      ipv6_input(dev);
      ameba_wlan_reply(priv);
#else
      NETDEV_RXDROPPED(dev);
#endif
    }
  else if (eth->type == HTONS(ETHTYPE_ARP))
    {
#ifdef CONFIG_NET_ARP
      NETDEV_RXARP(dev);
      arp_input(dev);
      ameba_wlan_reply(priv);
#else
      NETDEV_RXDROPPED(dev);
#endif
    }
  else
    {
      NETDEV_RXDROPPED(dev);
    }
}

/****************************************************************************
 * Name: ameba_wlan_rxwork
 *
 * Description:
 *   Deferred RX processing (HPWORK).  Drains the frames queued by
 *   ameba_wlan_rxframe and runs each through the network stack.
 *
 *   Decoupling stack processing from the WHC RX callback is the whole point:
 *   the callback now returns immediately, so the WHC layer recycles its NP
 *   RX buffer (sends RECV_DONE) at once instead of waiting for ipv4_input()
 *   and the in-place reply TX.  Otherwise the NP RX ring drains and the NP
 *   drops the bulk of an incoming stream (iperf collapsed to <1 Mbps).  This
 *   is the standard NuttX WiFi netdev idiom and matches what the vendor lwIP
 *   adapter does via its tcpip mailbox.
 *
 ****************************************************************************/

static void ameba_wlan_rxwork(void *arg)
{
  struct ameba_wlan_s *priv = (struct ameba_wlan_s *)arg;
  struct net_driver_s *dev = &priv->dev;
  struct iob_s *iob;
  irqstate_t flags;

  net_lock();

  for (; ; )
    {
      flags = enter_critical_section();
      iob = iob_remove_queue(&priv->rxq);
      leave_critical_section(flags);

      if (iob == NULL)
        {
          break;
        }

      /* Copy the queued frame out into the flat work buffer (room for an
       * in-place reply) and run it through the stack.
       */

      dev->d_len = iob_copyout(g_rxbuf, iob, iob->io_pktlen, 0);
      dev->d_buf = g_rxbuf;
      iob_free_chain(iob);

      ameba_wlan_rxdispatch(priv);
    }

  net_unlock();
}

/****************************************************************************
 * Name: ameba_wlan_rxframe
 *
 * Description:
 *   RX entry from the WHC host RX path (SDK side).  Runs in the WHC RX task
 *   context: copy the frame into a pooled IOB, queue it, and schedule the
 *   deferred worker -- then return at once so the WHC layer can recycle the
 *   NP buffer (RECV_DONE) without waiting for stack processing.
 *
 ****************************************************************************/

void ameba_wlan_rxframe(int idx, const unsigned char *buf, unsigned int len)
{
  struct ameba_wlan_s *priv = &g_ameba_wlan;
  struct iob_s *iob;
  irqstate_t flags;
  int ret;

  UNUSED(idx);

  if (!priv->bifup || len == 0 || len > AMEBA_WLAN_BUFSIZE)
    {
      return;
    }

  /* Backpressure: drop if the IOB pool cannot hold the frame.  Dropping here
   * is far better than blocking the WHC RX task, which would stall the NP.
   */

  if (len > iob_navail(false) * CONFIG_IOB_BUFSIZE)
    {
      NETDEV_RXDROPPED(&priv->dev);
      return;
    }

  iob = iob_tryalloc(false);
  if (iob == NULL)
    {
      NETDEV_RXDROPPED(&priv->dev);
      return;
    }

  ret = iob_trycopyin(iob, buf, len, 0, false);
  if (ret != (int)len)
    {
      iob_free_chain(iob);
      NETDEV_RXDROPPED(&priv->dev);
      return;
    }

  flags = enter_critical_section();
  ret = iob_tryadd_queue(iob, &priv->rxq);
  leave_critical_section(flags);

  if (ret < 0)
    {
      iob_free_chain(iob);
      NETDEV_RXDROPPED(&priv->dev);
      return;
    }

  if (work_available(&priv->rxwork))
    {
      work_queue(HPWORK, &priv->rxwork, ameba_wlan_rxwork, priv, 0);
    }
}

/****************************************************************************
 * Name: ameba_wlan_txavail_work / ameba_wlan_txavail
 ****************************************************************************/

static void ameba_wlan_txavail_work(void *arg)
{
  struct ameba_wlan_s *priv = (struct ameba_wlan_s *)arg;

  net_lock();
  if (priv->bifup)
    {
      priv->dev.d_buf = g_txbuf;
      priv->dev.d_len = 0;
      devif_poll(&priv->dev, ameba_wlan_txpoll);
    }

  net_unlock();
}

static int ameba_wlan_txavail(struct net_driver_s *dev)
{
  struct ameba_wlan_s *priv = (struct ameba_wlan_s *)dev->d_private;

  if (work_available(&priv->pollwork))
    {
      work_queue(LPWORK, &priv->pollwork, ameba_wlan_txavail_work, priv, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: ameba_wlan_ifup / ameba_wlan_ifdown
 ****************************************************************************/

static int ameba_wlan_ifup(struct net_driver_s *dev)
{
  struct ameba_wlan_s *priv = (struct ameba_wlan_s *)dev->d_private;
  uint8_t mac[IFHWADDRLEN];

  if (ameba_wifi_get_mac(priv->devnum, mac) == 0)
    {
      memcpy(dev->d_mac.ether.ether_addr_octet, mac, IFHWADDRLEN);
    }

  ninfo("Bringing up wlan%d %02x:%02x:%02x:%02x:%02x:%02x\n", priv->devnum,
        dev->d_mac.ether.ether_addr_octet[0],
        dev->d_mac.ether.ether_addr_octet[1],
        dev->d_mac.ether.ether_addr_octet[2],
        dev->d_mac.ether.ether_addr_octet[3],
        dev->d_mac.ether.ether_addr_octet[4],
        dev->d_mac.ether.ether_addr_octet[5]);

  priv->bifup = true;

  /* Mark the link as running so the network stack will route TX out this
   * device (udp_sendto rejects a !IFF_RUNNING device with -EHOSTUNREACH,
   * which is what blocked DHCP DISCOVER from ever being transmitted).
   */

  netdev_carrier_on(dev);
  return OK;
}

static int ameba_wlan_ifdown(struct net_driver_s *dev)
{
  struct ameba_wlan_s *priv = (struct ameba_wlan_s *)dev->d_private;
  irqstate_t flags;

  flags = enter_critical_section();
  priv->bifup = false;
  leave_critical_section(flags);

  /* Stop the deferred RX worker and drop any frames it had not yet drained
   * so the IOBs are not leaked across an ifdown/ifup cycle.
   */

  work_cancel(HPWORK, &priv->rxwork);

  flags = enter_critical_section();
  iob_free_queue(&priv->rxq);
  leave_critical_section(flags);

  netdev_carrier_off(dev);
  return OK;
}

/****************************************************************************
 * Name: ameba_wlan_ioctl
 ****************************************************************************/

#ifdef CONFIG_NETDEV_IOCTL

/****************************************************************************
 * Name: ameba_wlan_format_scan
 *
 * Description:
 *   Encode the cached scan results into the wapi iw_event stream expected
 *   by SIOCGIWSCAN (one
 *   SIOCGIWAP/SIOCGIWFREQ/IWEVQUAL/SIOCGIWESSID/SIOCGIWENCODE set per AP),
 *   the iw_event stream layout the WAPI tool decodes.
 *   The SIOCGIWENCODE event carries only the enabled/disabled flag (the SDK
 *   scan record's security field), so wapi shows 0x8000 (open) vs a
 *   non-zero "encryption enabled" value rather than the 0xffff "unknown"
 *   placeholder.
 *
 ****************************************************************************/

static int ameba_wlan_format_scan(struct ameba_wlan_s *priv,
                                  struct iwreq *iwr)
{
  unsigned int need;
  uint8_t *pos;
  int i;

  if (priv->scan_count <= 0)
    {
      iwr->u.data.length = 0;
      return OK;
    }

  need = priv->scan_count *
         (offsetof(struct iw_event, u) * 5 + sizeof(struct sockaddr) +
          sizeof(struct iw_freq) + sizeof(struct iw_quality) +
          sizeof(struct iw_point) + IW_ESSID_MAX_SIZE +
          sizeof(struct iw_point));

  if (iwr->u.data.pointer == NULL || iwr->u.data.length < need)
    {
      iwr->u.data.length = need;
      return -E2BIG;
    }

  pos = iwr->u.data.pointer;

  for (i = 0; i < priv->scan_count; i++)
    {
      struct ameba_scan_ap *ap = &priv->scan[i];
      struct iw_event *iwe;

      iwe = (struct iw_event *)pos;
      iwe->len = offsetof(struct iw_event, u) + sizeof(struct sockaddr);
      iwe->cmd = SIOCGIWAP;
      memcpy(iwe->u.ap_addr.sa_data, ap->bssid, 6);

      iwe = (struct iw_event *)((uintptr_t)iwe + iwe->len);
      iwe->len      = offsetof(struct iw_event, u) + sizeof(struct iw_freq);
      iwe->cmd      = SIOCGIWFREQ;
      iwe->u.freq.e = 0;
      iwe->u.freq.m = ap->channel;

      iwe = (struct iw_event *)((uintptr_t)iwe + iwe->len);
      iwe->len = offsetof(struct iw_event, u) + sizeof(struct iw_quality);
      iwe->cmd = IWEVQUAL;
      iwe->u.qual.level   = (uint8_t)ap->rssi;
      iwe->u.qual.updated = IW_QUAL_DBM;

      iwe = (struct iw_event *)((uintptr_t)iwe + iwe->len);
      iwe->len = offsetof(struct iw_event, u) + sizeof(struct iw_point) +
                 IW_ESSID_MAX_SIZE;
      iwe->cmd             = SIOCGIWESSID;
      iwe->u.essid.length  = ap->ssid_len;
      iwe->u.essid.flags   = 1;
      iwe->u.essid.pointer = (FAR void *)(uintptr_t)sizeof(struct iw_point);
      memcpy((uint8_t *)iwe + offsetof(struct iw_event, u) +
             sizeof(struct iw_point), ap->ssid, IW_ESSID_MAX_SIZE);

      iwe = (struct iw_event *)((uintptr_t)iwe + iwe->len);
      iwe->len = offsetof(struct iw_event, u) + sizeof(struct iw_point);
      iwe->cmd = SIOCGIWENCODE;
      iwe->u.data.length  = 0;
      iwe->u.data.pointer = NULL;
      iwe->u.data.flags   = ap->security ?
                            (IW_ENCODE_ENABLED | IW_ENCODE_NOKEY) :
                            IW_ENCODE_DISABLED;

      pos = (uint8_t *)((uintptr_t)iwe + iwe->len);
    }

  iwr->u.data.length = need;
  return OK;
}

/****************************************************************************
 * Name: ameba_wlan_freq2channel
 *
 * Description:
 *   Map an SIOCSIWFREQ iw_freq into an 802.11 channel number.  wapi encodes
 *   "wapi freq <dev> <n> fixed" with wapi_float2freq(): a small <n> arrives
 *   as a bare channel number (e == 0), a larger value as a frequency
 *   (m * 10^e Hz, or MHz when e == 0).  Returns 0 when it cannot be mapped,
 *   leaving the caller's default in place.
 *
 ****************************************************************************/

static uint8_t ameba_wlan_freq2channel(const struct iw_freq *f)
{
  uint64_t hz = (uint64_t)f->m;
  uint32_t mhz;
  int      e  = f->e;

  if (e == 0 && f->m >= 1 && f->m <= 165)
    {
      return (uint8_t)f->m;            /* already a channel number */
    }

  while (e-- > 0)
    {
      hz *= 10;
    }

  mhz = (hz >= 1000000) ? (uint32_t)(hz / 1000000) : (uint32_t)hz;

  if (mhz >= 2412 && mhz <= 2484)
    {
      return (mhz == 2484) ? 14 : (uint8_t)((mhz - 2412) / 5 + 1);
    }

  if (mhz >= 5000 && mhz <= 5900)
    {
      return (uint8_t)((mhz - 5000) / 5);
    }

  return 0;
}

static int ameba_wlan_ioctl(struct net_driver_s *dev, int cmd,
                            unsigned long arg)
{
  struct ameba_wlan_s *priv = (struct ameba_wlan_s *)dev->d_private;
  struct iwreq *iwr = (struct iwreq *)((uintptr_t)arg);
  int ret = OK;

  switch (cmd)
    {
      case SIOCSIWSCAN:           /* Trigger a (blocking) scan               */
        {
          ret = ameba_wifi_scan_start();
          if (ret < 0)
            {
              priv->scan_count = 0;
              return -EIO;
            }

          ret = ameba_wifi_scan_results(priv->scan, AMEBA_WLAN_MAX_SCAN_AP);
          priv->scan_count = (ret < 0) ? 0 : ret;
          return OK;
        }

      case SIOCGIWSCAN:           /* Read back the cached scan results       */
        return ameba_wlan_format_scan(priv, iwr);

      case SIOCSIWENCODEEXT:      /* Store the WPA passphrase                */
        {
          struct iw_encode_ext *ext = iwr->u.encoding.pointer;
          uint8_t klen;

          if (ext == NULL)
            {
              return -EINVAL;
            }

          klen = ext->key_len > (int)sizeof(priv->psk) - 1 ?
                 sizeof(priv->psk) - 1 : ext->key_len;
          memcpy(priv->psk, ext->key, klen);
          priv->psk[klen] = '\0';
          priv->psk_len   = klen;
          return OK;
        }

      case SIOCSIWESSID:          /* Set SSID; flags!=0 => connect / start AP */
        {
          uint8_t slen = iwr->u.essid.length;

          if (iwr->u.essid.pointer == NULL)
            {
              return -EINVAL;
            }

          if (slen > AMEBA_WLAN_SSID_MAXLEN)
            {
              slen = AMEBA_WLAN_SSID_MAXLEN;
            }

          memcpy(priv->ssid, iwr->u.essid.pointer, slen);
          priv->ssid[slen] = '\0';
          priv->ssid_len   = slen;

          /* In master (SoftAP) mode the SSID commit starts/stops the AP;
           * otherwise it is the STA connect/disconnect trigger.
           */

          if (priv->mode == IW_MODE_MASTER)
            {
              if (iwr->u.essid.flags == 0)
                {
                  ret = ameba_wifi_stop_ap();
                  priv->devnum = AMEBA_WLAN_STA_IDX;
                  return ret < 0 ? -EIO : OK;
                }

              ret = ameba_wifi_start_ap(priv->ssid, priv->ssid_len,
                                        priv->psk, priv->psk_len,
                                        priv->channel);
              if (ret < 0)
                {
                  return -EHOSTUNREACH;
                }

              /* Traffic now flows on the SoftAP interface: bind the netdev
               * TX path to it and adopt the AP MAC as the host L2 address.
               */

              priv->devnum = AMEBA_WLAN_AP_IDX;
              ameba_wifi_get_mac(priv->devnum,
                                 dev->d_mac.ether.ether_addr_octet);
              return OK;
            }

          if (iwr->u.essid.flags == 0)
            {
              return ameba_wifi_disconnect();
            }

          return ameba_wifi_connect(priv->ssid, priv->ssid_len,
                                    priv->psk, priv->psk_len) < 0 ?
                 -EHOSTUNREACH : OK;
        }

      case SIOCSIWMODE:           /* STA (infra) vs SoftAP (master)          */
        priv->mode = (uint8_t)iwr->u.mode;
        return OK;

      case SIOCSIWFREQ:           /* SoftAP channel hint (STA uses full scan) */
        {
          uint8_t ch = ameba_wlan_freq2channel(&iwr->u.freq);
          if (ch != 0)
            {
              priv->channel = ch;
            }

          return OK;
        }

      case SIOCSIWAUTH:           /* Auth params derived from PSK presence   */
      case SIOCSIWAP:             /* BSSID pinning not used                  */
        return OK;

      case SIOCGIWESSID:
        if (iwr->u.essid.pointer != NULL)
          {
            memcpy(iwr->u.essid.pointer, priv->ssid, priv->ssid_len);
            iwr->u.essid.length = priv->ssid_len;
            iwr->u.essid.flags  = priv->bifup ? 1 : 0;
          }

        return OK;

      default:
        nwarn("Unsupported WLAN ioctl 0x%04x\n", cmd);
        return -ENOTTY;
    }

  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int ameba_wlan_initialize(void)
{
  struct ameba_wlan_s *priv = &g_ameba_wlan;
  struct net_driver_s *dev  = &priv->dev;

  memset(priv, 0, sizeof(*priv));
  priv->devnum   = AMEBA_WLAN_DEVNUM;
  priv->mode     = IW_MODE_INFRA;   /* STA by default; wapi can switch to AP */

  dev->d_ifup    = ameba_wlan_ifup;
  dev->d_ifdown  = ameba_wlan_ifdown;
  dev->d_txavail = ameba_wlan_txavail;
#ifdef CONFIG_NETDEV_IOCTL
  dev->d_ioctl   = ameba_wlan_ioctl;
#endif
  dev->d_private = priv;
  dev->d_buf     = g_txbuf;

  return netdev_register(dev, NET_LL_IEEE80211);
}
