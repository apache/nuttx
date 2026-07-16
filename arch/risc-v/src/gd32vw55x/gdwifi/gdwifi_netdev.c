/****************************************************************************
 * arch/risc-v/src/gd32vw55x/gdwifi/gdwifi_netdev.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * GD32VW55x network driver (wlan0) on top of the native NuttX network
 * stack.
 *
 * Replaces the SDK lwIP: it implements the binary seam that the MAC
 * firmware (libwifi.a) imports -- macsw/import/lwip_import.h -- on top of
 * the NuttX netdev lowerhalf.  The prebuilt library never dereferences
 * "struct netif": for it net_if is an opaque void*, so we pass our own
 * context.
 *
 * Validated vendor HAL: GD32VW55x Wi-Fi & BLE SDK V1.0.3g (2026-04-23,
 * commit 945c6e2).  The libraries are prebuilt -- see
 * gigadevice_port/SDK_VERSION.md before updating the SDK.
 *
 * Contracts imposed by the prebuilt library (do not change without
 * re-reading lwip_import.h):
 *   - "struct pbuf" has a FIXED 16-byte layout (the MAC writes into the
 *     fields);
 *   - every TX buffer needs NET_AL_TX_HEADROOM (348 B) free before the
 *     payload, and net_buf_tx_info() moves the payload back into that
 *     headroom, returning the 4-byte aligned pointer;
 *   - net_if_input() delivers the RX with a free_fn that returns the
 *     buffer to the hardware and must be called exactly once.
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

#include <sys/socket.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <stddef.h>
#include <net/if_arp.h>
#include <debug.h>
#include <syslog.h>
#include <netinet/in.h>

#include <nuttx/kmalloc.h>
#include <nuttx/spinlock.h>
#include <nuttx/net/netdev_lowerhalf.h>
#include <nuttx/wireless/wireless.h>

#include "mac_types.h"
#include "macif_api.h"

#include "gdwifi_netdev.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Headroom required by the MAC before the TX payload (lwip_import.h:56).
 * The SDK reserves it in every pbuf; here we allocate it together with the
 * buffer.
 */

#define NET_AL_TX_HEADROOM   348

/* pbuf flags used by the seam (lwip/pbuf.h) */

#define PBUF_FLAG_IS_CUSTOM  0x02
#define PBUF_TYPE_RAM        0x80   /* Payload contiguous in the heap */
#define PBUF_TYPE_REF        0x41   /* Struct only; payload is external */

#define ALIGN4_HI(x)         (((uintptr_t)(x) + 3) & ~3ul)

#define GDWIFI_VIF_STA       0

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* TX buffer: the struct pbuf that the prebuilt library sees comes first
 * (the pointer handed to it is the one of this struct), followed by our own
 * book-keeping.
 */

struct gdwifi_txbuf_s
{
  struct pbuf pbuf;        /* MUST be the first field (prebuilt lib ABI) */
  uint8_t    *alloc;       /* Start of the allocation (headroom + data) */

  /* Has net_buf_tx_info() moved the payload back already? */

  bool        payload_shifted;
};

struct gdwifi_dev_s
{
  struct netdev_lowerhalf_s dev;   /* MUST be the first field */
  spinlock_t                rx_lock;
  netpkt_queue_t            rx_queue;
  bool                      ifup;
  uint8_t                   flatbuf[CONFIG_NET_ETH_PKTSIZE];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct gdwifi_dev_s g_wlan;

/****************************************************************************
 * SDK Functions Used Here
 ****************************************************************************/

int  macif_tx_start(void *net_if, struct pbuf *buf,
                    void (*cfm_cb)(uint32_t, bool, void *), void *cfm_arg);
int  wifi_management_connect(char *ssid, char *password, uint8_t blocked);
int  wifi_management_ap_start(char *ssid, char *passwd, uint32_t channel,
                             int auth_mode, uint32_t hidden);
int  wifi_management_ap_stop(void);
int  wifi_management_disconnect(void);
int  wifi_management_scan(uint8_t blocked, const char *ssid);
int  wifi_vif_is_sta_connected(int vif_idx);
void *vif_idx_to_net_if(uint8_t vif_idx);
int  wifi_wpa_rx_eapol_event(void *wvif, uint16_t type, uint8_t *data,
                             uint32_t len);
void *vif_idx_to_wvif(uint8_t vif_idx);

/* Scan results: use the SDK headers (the struct layout belongs to the SDK;
 * redefining it by hand corrupts the heap when the firmware fills in the
 * fields).
 */

int   wifi_netlink_scan_results_get(int vif_idx,
                                    struct macif_scan_results *results);

void *sys_malloc(size_t size);
void  sys_mfree(void *ptr);

/****************************************************************************
 * Public Functions -- prebuilt library seam (macsw/import/lwip_import.h)
 *
 * Note: vif_idx_to_net_if() belongs to the SDK itself (wifi_vif.c) and
 * returns the placeholder embedded in the VIF.  The prebuilt library only
 * hands that pointer back to the functions below, so it is opaque and we do
 * not need to interpret it.
 ****************************************************************************/

/****************************************************************************
 * Name: net_buf_tx_alloc
 *
 * Description:
 *   Allocate a TX buffer with headroom.  Allocation layout:
 *
 *     [ headroom 348 B ][ payload length B ]
 *                       ^ pbuf->payload
 *
 *   net_buf_tx_info() later moves the payload back into the headroom.
 *
 ****************************************************************************/

net_buf_tx_t *net_buf_tx_alloc(uint32_t length)
{
  struct gdwifi_txbuf_s *tx;
  uint8_t *mem;

  tx = kmm_zalloc(sizeof(*tx));
  if (tx == NULL)
    {
      return NULL;
    }

  /* +4 so that the headroom pointer can be 4-byte aligned without
   * overflowing the allocation.
   */

  mem = kmm_malloc(NET_AL_TX_HEADROOM + length + 4);
  if (mem == NULL)
    {
      kmm_free(tx);
      return NULL;
    }

  tx->alloc            = mem;
  tx->pbuf.payload     = mem + NET_AL_TX_HEADROOM;
  tx->pbuf.len         = length;
  tx->pbuf.tot_len     = length;
  tx->pbuf.ref         = 1;
  tx->pbuf.type_internal = PBUF_TYPE_RAM;

  return &tx->pbuf;
}

/* Struct only: the prebuilt library itself fills in payload/len later
 * (softAP bridging, no copy).
 */

net_buf_tx_t *net_buf_tx_alloc_ref(uint32_t length)
{
  struct gdwifi_txbuf_s *tx = kmm_zalloc(sizeof(*tx));

  if (tx == NULL)
    {
      return NULL;
    }

  tx->pbuf.len           = length;
  tx->pbuf.tot_len       = length;
  tx->pbuf.ref           = 1;
  tx->pbuf.type_internal = PBUF_TYPE_REF;

  return &tx->pbuf;
}

/* Free the whole chain (the MAC may have chained it with net_buf_tx_cat) */

static void gdwifi_txbuf_release(struct pbuf *p)
{
  while (p != NULL)
    {
      struct gdwifi_txbuf_s *tx = (struct gdwifi_txbuf_s *)p;
      struct pbuf *next = p->next;

      if (--p->ref == 0)
        {
          if (tx->alloc != NULL)
            {
              kmm_free(tx->alloc);
            }

          kmm_free(tx);
        }

      p = next;
    }
}

void net_buf_tx_pbuf_free(net_buf_tx_t *buf)
{
  gdwifi_txbuf_release(buf);
}

/* Undo the payload shift done by net_buf_tx_info() and free */

void net_buf_tx_free(net_buf_tx_t *buf)
{
  struct gdwifi_txbuf_s *tx = (struct gdwifi_txbuf_s *)buf;

  if (buf != NULL && tx->payload_shifted)
    {
      buf->payload = (uint8_t *)buf->payload + NET_AL_TX_HEADROOM;
      buf->len    -= NET_AL_TX_HEADROOM;
      buf->tot_len -= NET_AL_TX_HEADROOM;
      tx->payload_shifted = false;
    }

  gdwifi_txbuf_release(buf);
}

void net_buf_tx_cat(net_buf_tx_t *buf1, net_buf_tx_t *buf2)
{
  struct pbuf *p;

  if (buf1 == NULL || buf2 == NULL)
    {
      return;
    }

  for (p = buf1; p->next != NULL; p = p->next)
    {
      p->tot_len += buf2->tot_len;
    }

  p->tot_len += buf2->tot_len;
  p->next     = buf2;
}

/****************************************************************************
 * Name: net_buf_tx_info
 *
 * Description:
 *   Hand the segment list to the MAC DMA and return the headroom pointer
 *   (4-byte aligned).  MUTATES the pbuf: moves the payload back 348 bytes.
 *
 ****************************************************************************/

void *net_buf_tx_info(net_buf_tx_t *buf, uint16_t *tot_len, int *seg_cnt,
                      uint32_t seg_addr[], uint16_t seg_len[])
{
  struct gdwifi_txbuf_s *tx = (struct gdwifi_txbuf_s *)buf;
  int seg_max = *seg_cnt;
  uint16_t length;
  void *headroom;
  int idx;

  if (buf == NULL)
    {
      return NULL;
    }

  length      = buf->tot_len;
  *tot_len    = length;
  seg_addr[0] = (uint32_t)(uintptr_t)buf->payload;
  seg_len[0]  = buf->len;
  length     -= buf->len;

  /* Grow the pbuf into the headroom */

  if ((uint8_t *)buf->payload - NET_AL_TX_HEADROOM < tx->alloc)
    {
      nerr("ERROR: not enough headroom in the TX buffer\n");
      return NULL;
    }

  buf->payload = (uint8_t *)buf->payload - NET_AL_TX_HEADROOM;
  buf->len    += NET_AL_TX_HEADROOM;
  buf->tot_len += NET_AL_TX_HEADROOM;
  tx->payload_shifted = true;

  headroom = (void *)ALIGN4_HI(buf->payload);

  /* Extra segments from the chain */

  buf = buf->next;
  idx = 1;

  while (length > 0 && buf != NULL && idx < seg_max)
    {
      seg_addr[idx] = (uint32_t)(uintptr_t)buf->payload;
      seg_len[idx]  = buf->len;
      length       -= buf->len;
      buf           = buf->next;
      idx++;
    }

  *seg_cnt = idx;

  if (length != 0)
    {
      nerr("ERROR: TX buffer not covered by the segments\n");
      return NULL;
    }

  return headroom;
}

/****************************************************************************
 * Name: net_if_input
 *
 * Description:
 *   RX: the MAC delivers an Ethernet frame.  Runs on the MACIF-RX task (not
 *   in an ISR).  We copy it into a netpkt and return the buffer to the
 *   hardware right away (the NuttX IOB does not accept an external
 *   payload).
 *
 ****************************************************************************/

int net_if_input(net_buf_rx_t *buf, void *net_if, void *addr, uint16_t len,
                 net_buf_free_fn free_fn)
{
  struct gdwifi_dev_s *priv = &g_wlan;
  uint8_t *frame = addr;
  uint16_t ethertype;
  netpkt_t *pkt;
  irqstate_t flags;

  if (len < 14 || len > CONFIG_NET_ETH_PKTSIZE)
    {
      free_fn(buf);
      return -1;
    }

  /* EAPOL does not go to the IP stack: it is the WPA 4-way handshake and
   * the consumer is the SDK supplicant (this used to be the lwIP
   * net_eth_receive hook).
   */

  ethertype = (uint16_t)((frame[12] << 8) | frame[13]);

  if (ethertype == 0x888e)
    {
          wifi_wpa_rx_eapol_event(vif_idx_to_wvif(GDWIFI_VIF_STA), ethertype,
                              frame, len);
      free_fn(buf);
      return 0;
    }

  if (!priv->ifup)
    {
      free_fn(buf);
      return -1;
    }

  pkt = netpkt_alloc(&priv->dev, NETPKT_RX);
  if (pkt == NULL)
    {
      free_fn(buf);
      return -1;
    }

  netpkt_copyin(&priv->dev, pkt, addr, len, 0);
  free_fn(buf);                 /* Return the DMA buffer to the MAC */

  flags = spin_lock_irqsave(&priv->rx_lock);
  netpkt_tryadd_queue(pkt, &priv->rx_queue);
  spin_unlock_irqrestore(&priv->rx_lock, flags);

  netdev_lower_rxready(&priv->dev);
  return 0;
}

void net_if_up(void *net_if)
{
  netdev_lower_carrier_on(&g_wlan.dev);
}

void net_if_down(void *net_if)
{
  netdev_lower_carrier_off(&g_wlan.dev);
}

/****************************************************************************
 * Prebuilt library control link: loopback UDP sockets (127.0.0.1)
 ****************************************************************************/

int net_lpbk_socket_create(int protocol)
{
  return socket(PF_INET, SOCK_DGRAM, protocol);
}

int net_lpbk_socket_bind(int sock_recv, uint32_t port)
{
  struct sockaddr_in addr;

  memset(&addr, 0, sizeof(addr));
  addr.sin_family      = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port        = htons(port);

  return bind(sock_recv, (struct sockaddr *)&addr,
              sizeof(addr)) < 0 ? -1 : 0;
}

int net_lpbk_socket_connect(int sock_send, uint32_t port)
{
  struct sockaddr_in addr;

  memset(&addr, 0, sizeof(addr));
  addr.sin_family      = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port        = htons(port);

  return connect(sock_send, (struct sockaddr *)&addr,
                 sizeof(addr)) < 0 ? -1 : 0;
}

/* The prebuilt library (macif_cntrl.o) calls these names; they map onto the
 * POSIX sockets.
 */

int lwip_socket(int domain, int type, int protocol)
{
  return socket(domain, type, protocol);
}

int lwip_send(int s, const void *data, size_t size, int flags)
{
  return send(s, data, size, flags);
}

int lwip_recv(int s, void *mem, size_t len, int flags)
{
  return recv(s, mem, len, flags);
}

int lwip_close(int s)
{
  return close(s);
}

/* Only meaningful with softAP + the SDK DHCP server, which we do not use */

void dhcpd_delete_ipaddr_by_macaddr(uint8_t *mac_addr)
{
}

/****************************************************************************
 * netdev lowerhalf
 ****************************************************************************/

static int gdwifi_ifup(struct netdev_lowerhalf_s *dev)
{
  struct gdwifi_dev_s *priv = (struct gdwifi_dev_s *)dev;

  priv->ifup = true;
  ninfo("wlan0 up\n");
  return OK;
}

static int gdwifi_ifdown(struct netdev_lowerhalf_s *dev)
{
  struct gdwifi_dev_s *priv = (struct gdwifi_dev_s *)dev;

  priv->ifup = false;
  netpkt_free_queue(&priv->rx_queue);
  return OK;
}

/* TX: build a pbuf with headroom and hand it to the MAC.  The MAC frees the
 * buffer later, via net_buf_tx_free().
 */

static int gdwifi_transmit(struct netdev_lowerhalf_s *dev, netpkt_t *pkt)
{
  struct gdwifi_dev_s *priv = (struct gdwifi_dev_s *)dev;
  unsigned int len = netpkt_getdatalen(dev, pkt);
  struct pbuf *p;

  if (len == 0 || len > CONFIG_NET_ETH_PKTSIZE)
    {
      return -EINVAL;
    }

  /* Only transmit once the link is associated: handing a frame to the MAC
   * while it is still scanning/associating trips the firmware HW_IDLE
   * assert.  (The NuttX network stack starts sending as soon as the
   * interface comes up.)
   */

  if (!IFF_IS_RUNNING(dev->netdev.d_flags))
    {
      netpkt_free(dev, pkt, NETPKT_TX);
      return OK;
    }

  p = net_buf_tx_alloc(len);
  if (p == NULL)
    {
      return -ENOMEM;
    }

  netpkt_copyout(dev, p->payload, pkt, len, 0);

  /* The net_if MUST be the one of the VIF: the MAC uses that pointer to
   * find out on which interface to transmit.  Passing another one (e.g.
   * the driver context) makes the frame be silently dropped.
   */

  if (macif_tx_start(vif_idx_to_net_if(GDWIFI_VIF_STA), p, NULL, NULL) != 0)
    {
      net_buf_tx_pbuf_free(p);
      return -EIO;
    }

  netpkt_free(dev, pkt, NETPKT_TX);
  return OK;
}

static netpkt_t *gdwifi_receive(struct netdev_lowerhalf_s *dev)
{
  struct gdwifi_dev_s *priv = (struct gdwifi_dev_s *)dev;
  irqstate_t flags;
  netpkt_t *pkt;

  flags = spin_lock_irqsave(&priv->rx_lock);
  pkt = netpkt_remove_queue(&priv->rx_queue);
  spin_unlock_irqrestore(&priv->rx_lock, flags);

  return pkt;
}

/****************************************************************************
 * Wireless ops (wapi commands)
 ****************************************************************************/

static char g_essid[33];
static char g_passwd[65];
static int  g_mode = IW_MODE_INFRA;   /* INFRA = station, MASTER = softAP */

/* wifi_management.h: AUTH_MODE_OPEN = 0, ..., AUTH_MODE_WPA2 = 3 */

#define GDWIFI_AUTH_OPEN  0
#define GDWIFI_AUTH_WPA2  3   /* AUTH_MODE_WPA2 (SAE/WPA3 no AP estoura o crypto) */
#define GDWIFI_AP_CHANNEL 11

/****************************************************************************
 * Name: gdwifi_target_supported
 *
 * Description:
 *   The port is WPA2-only (the SAE handshake faults inside the prebuilt
 *   supplicant), so refuse a network that offers no AKM we can complete --
 *   WPA3(SAE)-only being the common case -- with a clear error instead of
 *   letting the association fail obscurely.  A targeted blocking scan
 *   fetches the AKM bitmap of the AP; if the network cannot be found the
 *   decision is left to the vendor connect path (same behavior as today).
 *
 ****************************************************************************/

static int gdwifi_target_supported(void)
{
  struct macif_scan_results *results;
  size_t ssid_len = strlen(g_essid);
  uint32_t supported = CO_BIT(MAC_AKM_NONE) | CO_BIT(MAC_AKM_PRE_RSN) |
                       CO_BIT(MAC_AKM_PSK) | CO_BIT(MAC_AKM_PSK_SHA256);
  int ret = OK;
  uint32_t i;

  if (wifi_management_scan(1, g_essid) != 0)
    {
      return OK;
    }

  results = sys_malloc(sizeof(*results));
  if (results == NULL)
    {
      return -ENOMEM;
    }

  if (wifi_netlink_scan_results_get(GDWIFI_VIF_STA, results) != 0)
    {
      sys_mfree(results);
      return OK;
    }

  for (i = 0; i < results->result_cnt; i++)
    {
      struct mac_scan_result *ap = &results->result[i];

      if (!ap->valid_flag || ap->ssid.length != ssid_len ||
          memcmp(ap->ssid.array, g_essid, ssid_len) != 0)
        {
          continue;
        }

      if ((ap->akm & supported) == 0)
        {
          /* syslog, not nerr: the rejection must be visible in release
           * configs too, or the refused connect looks like a silent no-op.
           */

          syslog(LOG_ERR, "gdwifi: '%s' offers no supported AKM (bitmap "
                 "0x%lx): WPA3/SAE, OWE and 802.1X are not supported, "
                 "WPA2-PSK only\n", g_essid, (unsigned long)ap->akm);
          ret = -ENOTSUP;
        }

      break;
    }

  sys_mfree(results);
  return ret;
}

static int gdwifi_connect(struct netdev_lowerhalf_s *dev)
{
  int ret;

  if (g_essid[0] == '\0')
    {
      return -EINVAL;
    }

  /* In master mode "connect" means "start the softAP".  Both are driven
   * from the same WAPI ioctl (SIOCSIWESSID) -- see gdwifi_mode().
   */

  if (g_mode == IW_MODE_MASTER)
    {
      int auth = g_passwd[0] ? GDWIFI_AUTH_WPA2 : GDWIFI_AUTH_OPEN;

      ninfo("softAP start '%s'\n", g_essid);
      ret = wifi_management_ap_start(g_essid,
                                     g_passwd[0] ? g_passwd : NULL,
                                     GDWIFI_AP_CHANNEL, auth, 0);
      return ret == 0 ? OK : -EAGAIN;
    }

  ret = gdwifi_target_supported();
  if (ret < 0)
    {
      return ret;
    }

  ninfo("connect '%s'\n", g_essid);
  ret = wifi_management_connect(g_essid,
                                g_passwd[0] ? g_passwd : NULL, 1);
  return ret == 0 ? OK : -EAGAIN;
}

static int gdwifi_disconnect(struct netdev_lowerhalf_s *dev)
{
  if (g_mode == IW_MODE_MASTER)
    {
      wifi_management_ap_stop();
    }
  else
    {
      wifi_management_disconnect();
    }

  return OK;
}

static int gdwifi_essid(struct netdev_lowerhalf_s *dev, struct iwreq *iwr,
                        bool set)
{
  struct iw_point *essid = &iwr->u.essid;

  if (set)
    {
      size_t len = essid->length;

      if (len >= sizeof(g_essid))
        {
          len = sizeof(g_essid) - 1;
        }

      memcpy(g_essid, essid->pointer, len);
      g_essid[len] = '\0';
    }
  else
    {
      size_t len = strlen(g_essid);

      memcpy(essid->pointer, g_essid, len);
      essid->length = len;
      essid->flags  = wifi_vif_is_sta_connected(GDWIFI_VIF_STA) ?
                      IW_ESSID_ON : 0;
    }

  return OK;
}

static int gdwifi_passwd(struct netdev_lowerhalf_s *dev, struct iwreq *iwr,
                         bool set)
{
  struct iw_encode_ext *ext;
  size_t len;

  if (!set)
    {
      return -ENOTTY;
    }

  ext = iwr->u.encoding.pointer;
  if (ext == NULL)
    {
      return -EINVAL;
    }

  len = ext->key_len;
  if (len >= sizeof(g_passwd))
    {
      return -EINVAL;
    }

  memcpy(g_passwd, ext->key, len);
  g_passwd[len] = '\0';
  return OK;
}

/* The security mode (WPA/WPA2 + cipher) is deduced by the wifi_manager from
 * the AP beacon, so we just accept and ignore it.
 */

static int gdwifi_auth(struct netdev_lowerhalf_s *dev, struct iwreq *iwr,
                       bool set)
{
  return OK;
}

static int gdwifi_mode(struct netdev_lowerhalf_s *dev, struct iwreq *iwr,
                       bool set)
{
  if (set)
    {
      /* INFRA = station, MASTER = softAP.  The single-VIF firmware does
       * one or the other, not both at once.
       */

      if (iwr->u.mode != IW_MODE_INFRA && iwr->u.mode != IW_MODE_MASTER)
        {
          return -ENOSYS;
        }

      g_mode = iwr->u.mode;
      return OK;
    }

  iwr->u.mode = g_mode;
  return OK;
}

/****************************************************************************
 * Name: gdwifi_scan
 *
 * Description:
 *   SIOCSIWSCAN (set=true)  -> trigger the scan.
 *   SIOCGIWSCAN (set=false) -> return the APs as a packed array of
 *   struct iw_event, which is the format wapi expects (same as the ESP32).
 *   If the user buffer is too small, return -E2BIG with the required size
 *   in u.data.length -- wapi reallocates and calls again.
 *
 ****************************************************************************/

#define IW_EVT_SIZE(field) \
  (offsetof(struct iw_event, u) + sizeof(((union iwreq_data *)0)->field))

static int gdwifi_scan(struct netdev_lowerhalf_s *dev, struct iwreq *iwr,
                       bool set)
{
  struct macif_scan_results *results;
  struct iw_event *iwe;
  uint8_t *buf;
  size_t need = 0;
  int ret = OK;
  uint32_t i;

  if (set)
    {
      /* Blocking scan: when it returns, the results are already ready */

      return wifi_management_scan(1, NULL) == 0 ? OK : -EIO;
    }

  results = sys_malloc(sizeof(*results));
  if (results == NULL)
    {
      return -ENOMEM;
    }

  if (wifi_netlink_scan_results_get(GDWIFI_VIF_STA, results) != 0)
    {
      sys_mfree(results);
      return -EIO;
    }

  /* How much space the events take */

  for (i = 0; i < results->result_cnt; i++)
    {
      size_t ssid_len = results->result[i].ssid.length;

      need += IW_EVT_SIZE(ap_addr);                    /* BSSID */
      need += IW_EVT_SIZE(essid) + ((ssid_len + 3) & ~3);
      need += IW_EVT_SIZE(qual);                       /* RSSI */
      need += IW_EVT_SIZE(mode);
      need += IW_EVT_SIZE(data);                       /* Security */
      need += IW_EVT_SIZE(freq);                       /* Channel */
    }

  if (iwr->u.data.pointer == NULL || iwr->u.data.length < need)
    {
      iwr->u.data.length = need;
      sys_mfree(results);
      return -E2BIG;
    }

  buf = iwr->u.data.pointer;

  for (i = 0; i < results->result_cnt; i++)
    {
      struct mac_scan_result *ap = &results->result[i];
      size_t ssid_len = ap->ssid.length;

      /* BSSID */

      iwe = (struct iw_event *)buf;
      iwe->cmd = SIOCGIWAP;
      iwe->len = IW_EVT_SIZE(ap_addr);
      iwe->u.ap_addr.sa_family = ARPHRD_ETHER;
      memcpy(iwe->u.ap_addr.sa_data, ap->bssid.array, 6);
      buf += iwe->len;

      /* SSID (the name goes right after the struct; the "pointer" carries
       * the offset, not a pointer -- that is the wireless ext convention)
       */

      iwe = (struct iw_event *)buf;
      iwe->cmd = SIOCGIWESSID;
      iwe->len = IW_EVT_SIZE(essid) + ((ssid_len + 3) & ~3);
      iwe->u.essid.flags   = 1;
      iwe->u.essid.length  = ssid_len;
      iwe->u.essid.pointer = (void *)sizeof(iwe->u.essid);
      memcpy(&iwe->u.essid + 1, ap->ssid.array, ssid_len);
      buf += iwe->len;

      /* Signal quality */

      iwe = (struct iw_event *)buf;
      iwe->cmd = IWEVQUAL;
      iwe->len = IW_EVT_SIZE(qual);
      iwe->u.qual.qual    = 0;
      iwe->u.qual.level   = ap->rssi;
      iwe->u.qual.noise   = 0;
      iwe->u.qual.updated = IW_QUAL_DBM | IW_QUAL_ALL_UPDATED;
      buf += iwe->len;

      /* Mode (AP) */

      iwe = (struct iw_event *)buf;
      iwe->cmd = SIOCGIWMODE;
      iwe->len = IW_EVT_SIZE(mode);
      iwe->u.mode = IW_MODE_MASTER;
      buf += iwe->len;

      /* Security: wapi only wants to know whether there is crypto */

      iwe = (struct iw_event *)buf;
      iwe->cmd = SIOCGIWENCODE;
      iwe->len = IW_EVT_SIZE(data);
      iwe->u.data.flags   = (ap->akm != 0) ?
                            (IW_ENCODE_ENABLED | IW_ENCODE_NOKEY) :
                            IW_ENCODE_DISABLED;
      iwe->u.data.length  = 0;
      iwe->u.data.pointer = NULL;
      buf += iwe->len;

      /* Channel (wapi shows this value in the "frequency" column, same as
       * the ESP32 driver)
       */

      iwe = (struct iw_event *)buf;
      iwe->cmd = SIOCGIWFREQ;
      iwe->len = IW_EVT_SIZE(freq);
      iwe->u.freq.e     = 0;
      iwe->u.freq.flags = IW_FREQ_FIXED;
      iwe->u.freq.m     = (ap->chan != NULL && ap->chan->freq >= 2412) ?
                          ((ap->chan->freq - 2407) / 5) : 0;
      buf += iwe->len;
    }

  iwr->u.data.length = need;
  sys_mfree(results);
  return ret;
}

static const struct netdev_ops_s g_netdev_ops =
{
  .ifup     = gdwifi_ifup,
  .ifdown   = gdwifi_ifdown,
  .transmit = gdwifi_transmit,
  .receive  = gdwifi_receive,
};

static const struct wireless_ops_s g_wireless_ops =
{
  .connect    = gdwifi_connect,
  .disconnect = gdwifi_disconnect,
  .essid      = gdwifi_essid,
  .passwd     = gdwifi_passwd,
  .auth       = gdwifi_auth,
  .mode       = gdwifi_mode,
  .scan       = gdwifi_scan,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gdwifi_netdev_register
 *
 * Description:
 *   Register wlan0.  Must be called after the SDK wifi_init() (the MAC has
 *   to be ready to supply the MAC address).
 *
 ****************************************************************************/

struct net_driver_s *gdwifi_netdev_get(void)
{
  return &g_wlan.dev.netdev;
}

int gdwifi_netdev_register(const uint8_t *mac)
{
  struct gdwifi_dev_s *priv = &g_wlan;

  memcpy(priv->dev.netdev.d_mac.ether.ether_addr_octet, mac, 6);

  priv->dev.ops      = &g_netdev_ops;
  priv->dev.iw_ops   = &g_wireless_ops;
  priv->dev.quota[NETPKT_RX] = 8;
  priv->dev.quota[NETPKT_TX] = 4;
  priv->dev.rxtype   = NETDEV_RX_WORK;   /* Process RX in the work queue */

  /* This is NOT a task priority: netdev_lowerhalf uses it as the work
   * queue id in work_queue(lower->priority, ...).  Only HPWORK and LPWORK
   * are valid.  Anything else makes work_queue() fail silently, receive()
   * is never called, the RX queue fills up and every packet is dropped.
   */

  priv->dev.priority = LPWORK;

  spin_lock_init(&priv->rx_lock);
  IOB_QINIT(&priv->rx_queue);

  return netdev_lower_register(&priv->dev, NET_LL_IEEE80211);
}
