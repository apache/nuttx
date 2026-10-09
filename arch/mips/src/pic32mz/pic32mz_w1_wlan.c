/****************************************************************************
 * arch/mips/src/pic32mz/pic32mz_w1_wlan.c
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

/* Wi-Fi driver for the PIC32MZ-W1 (station and Soft-AP), built on
 * Microchip's closed WLAN library (pic32mzw1.a, MPLAB Harmony
 * wireless_wifi).  The library runs on the CPU itself; it is driven by a
 * kernel thread, three RF interrupts and a byte-coded configuration
 * protocol ("WIDs").  The network device is a netdev_upperhalf lower half
 * with the wireless operations used by wapi (scan, essid, psk, mode, freq).
 * WPA3-Personal needs the BA414E engine (pic32mz_ba414e.c).
 *
 * Every hardware or library fact below is tagged with where it comes from:
 *
 *   [DS]  PIC32MZ W1 and WFI32E01 Family Data Sheet, DS70005425P.
 *   [EX]  Not in [DS].  Observed in Microchip's MPLAB Harmony wireless_wifi
 *         v3.13.0 driver sources (driver/pic32mzw1/wdrv_pic32mzw*.c,
 *         drv_pic32mzw1_crypto.c, include/drv_pic32mzw1.h), which define
 *         the library interface.  Only the interface facts (function
 *         names, message formats, identifiers, values) are used here; no
 *         code was copied.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <inttypes.h>
#include <syslog.h>
#include <errno.h>
#include <debug.h>

#include <net/if_arp.h>

#include <nuttx/arch.h>
#include <nuttx/cache.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/queue.h>
#include <nuttx/semaphore.h>
#include <nuttx/mutex.h>
#include <nuttx/spinlock.h>
#include <nuttx/net/netdev_lowerhalf.h>
#include <nuttx/wireless/wireless.h>

#include <arch/irq.h>

#include "mips_internal.h"
#include "hardware/pic32mzw1_pmuclk.h"
#include "pic32mz_w1_wlan.h"
#ifdef CONFIG_PIC32MZ_W1_BA414E
#  include "pic32mz_ba414e.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define WLAN_CACHE_LINE      16

/* [DS] Table 4-6: 64 KB "Data Buffer Memory" at physical 0x00040000 on
 * the 1 MB-flash parts, separate from the 256 KB SRAM.  [EX] Harmony
 * places its reserved packet pool there (region "wlan_mem"), uncached.
 */

#define WLAN_PKTMEM_BASE     0xa0040000
#define WLAN_PKTMEM_SIZE     0x10000

/* [EX] Size of one reserved packet buffer */

#define WLAN_PKT_BUFSIZE     1596

#define WLAN_PKT_NODESIZE    (sizeof(struct wlan_hdr_s) + WLAN_PKT_BUFSIZE)
#define WLAN_PKT_NUM         (WLAN_PKTMEM_SIZE / WLAN_PKT_NODESIZE)

/* [EX] Room the library needs in front of a transmitted Ethernet frame */

#define WLAN_TX_HDROFFSET    34

/* [EX] Packet memory priority for data transmission (MEM_PRI_TX) */

#define WLAN_MEM_PRI_TX      4

/* [DS] PMUCLKCTRL.WLDOOFF: WLAN LDO off */

#define PMUCLKCTRL_WLDOOFF   (1 << 30)

/* [EX] WID identifiers.  The top nibble is the value type: 0 char,
 * 1 short, 2 int, 3 string, 4 binary.
 */

#define WID_CURRENT_TX_RATE     0x0001
#define WID_PREAMBLE            0x0003
#define WID_11G_OPERATING_MODE  0x0004
#define WID_SCAN_TYPE           0x0007
#define WID_QOS_ENABLE          0x000a
#define WID_POWER_MANAGEMENT    0x000b
#define WID_ACK_POLICY          0x0011
#define WID_BCAST_SSID          0x0015
#define WID_DISCONNECT          0x0016
#define WID_START_SCAN_REQ      0x001e
#define WID_RSSI                0x001f
#define WID_ASSOC_STAT          0x0022
#define WID_SCAN_FILTER         0x0036
#define WID_SWITCH_MODE         0x004a
#define WID_RF_MAC_CONFIG_STATUS 0x005a
#define WID_11N_ENABLE          0x0082
#define WID_COEX_ENABLE         0x00e6
#define WID_PS_CORRELATION      0x0212
#define WID_ACTIVE_SCAN_TIME    0x100c
#define WID_USER_PREF_CHANNEL   0x1020
#define WID_CURR_OPER_CHANNEL   0x1021
#define WID_USER_SCAN_CHANNEL   0x1022
#define WID_11I_SETTINGS        0x2036
#define WID_SCAN_CH_BITMAP_2GHZ 0x208a
#define WID_BSSID               0x3003
#define WID_11I_PSK             0x3008
#define WID_MAC_ADDR            0x300c
#define WID_GET_SCAN_RESULTS    0x3034
#define WID_REG_DOMAIN          0x4010
#define WID_STA_JOIN_INFO       0x4008
#define WID_RSNA_PASSWORD       0x4012
#define WID_SSID                0x4020

#define WID_TYPE(w)             ((w) >> 12)
#define WID_TYPE_STR            3
#define WID_TYPE_BIN            4

/* [EX] RF_MAC_CONFIG_STATUS bits needed before the radio can be used:
 * power-on calibration, factory calibration, gain table, MAC address.
 */

#define RFMAC_MIN_CONFIG        0x0f

/* [EX] 11i settings (DRV_PIC32MZW_11I_MASK) */

#define DOT11I_PRIVACY          0x0001
#define DOT11I_WPAIE            0x0010
#define DOT11I_RSNE             0x0020
#define DOT11I_CCMP128          0x0040
#define DOT11I_TKIP             0x0080
#define DOT11I_BIPCMAC128       0x0100
#define DOT11I_MFP_REQUIRED     0x0200
#define DOT11I_PSK              0x0800
#define DOT11I_SAE              0x1000
#define DOT11I_AP               0x8000

/* [EX] Harmony's WPA2-Personal and WPA/WPA2-Personal settings */

#define DOT11I_WPA2_PERSONAL \
  (DOT11I_PRIVACY | DOT11I_RSNE | DOT11I_CCMP128 | DOT11I_BIPCMAC128 | \
   DOT11I_PSK)
#define DOT11I_WPAWPA2_PERSONAL \
  (DOT11I_WPA2_PERSONAL | DOT11I_WPAIE | DOT11I_TKIP)

/* [EX] Harmony's WPA3-Personal settings (SAE, management frame protection
 * required)
 */

#define DOT11I_WPA3_PERSONAL \
  (DOT11I_PRIVACY | DOT11I_RSNE | DOT11I_CCMP128 | DOT11I_BIPCMAC128 | \
   DOT11I_MFP_REQUIRED | DOT11I_SAE)

/* Soft-AP channel when none was requested */

#define WLAN_AP_CHANNEL         1

/* [EX] Channels 1-11, Harmony's default 2.4 GHz scan mask */

#define WLAN_CHANNEL_MASK       0x07ff

#define WID_MSG_MAX             256

#define WLAN_SCAN_BUFSIZE       2048
#define WLAN_SCAN_TIMEOUT       SEC2TICK(10)

#define WLAN_RX_QUOTA           8
#define WLAN_TX_QUOTA           4

#define WLAN_CRYPTO_DEFER_MAX   8

#define IW_EVENT_SIZE(field) \
  (offsetof(struct iw_event, u) + sizeof(((union iwreq_data *)0)->field))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Header in front of every buffer handed to the library.  [EX] Harmony
 * uses a 16-byte header with this layout; keep it, the library has not
 * been shown not to depend on it.
 */

struct wlan_hdr_s
{
  FAR struct wlan_hdr_s *next;
  FAR void *alloc;              /* kmm block, or NULL for the pool */
  uint16_t size;
  uint8_t users;
  int8_t prio;
  FAR void *spare;
};

struct wid_msg_s
{
  FAR uint8_t *buf;
  FAR uint8_t *ptr;
  char op;                      /* 'W' write or 'Q' query */
};

struct wlan_crypto_defer_s
{
  wlan_crypto_cb_t cb;
  int result;
  uintptr_t context;
};

enum wlan_scan_e
{
  WLAN_SCAN_IDLE = 0,
  WLAN_SCAN_RUNNING,
  WLAN_SCAN_DONE
};

struct pic32mz_wlan_s
{
  struct netdev_lowerhalf_s dev; /* Must be first */

  /* Library interface */

  sem_t evsem;                  /* Wakes the WLAN thread */
  sem_t readysem;               /* RF configured and MAC known */
  mutex_t lock;                 /* Exclusive library access */
  sq_queue_t txq;               /* WID messages to the library */
  sq_queue_t rxq;               /* WID responses from the library */
  sq_queue_t pool;              /* Free reserved packet buffers */
  uint8_t rfmac_status;
  bool macvalid;
  bool ready;
  uint8_t mac[6];

  /* Deferred crypto completion callbacks (ring) */

  struct wlan_crypto_defer_s defer[WLAN_CRYPTO_DEFER_MAX];
  uint8_t defer_head;
  uint8_t defer_tail;

  /* Network device */

  spinlock_t rxlock;
  netpkt_queue_t rxqueue;       /* Received frames for the upper half */
  bool registered;

  /* Connection parameters and state */

  uint8_t ssid[IW_ESSID_MAX_SIZE];
  uint8_t ssidlen;
  uint8_t bssid[6];
  bool bssidvalid;
  uint8_t psk[64];
  uint8_t psklen;
  uint32_t wpaver;              /* IW_AUTH_WPA_VERSION_* */
  uint32_t cipher;              /* IW_AUTH_CIPHER_* */
  uint8_t reqchannel;           /* 0: any */
  bool ap;                      /* IW_MODE_MASTER: connect starts a Soft-AP */
  bool connected;               /* Associated (STA) or started (AP) */
  uint8_t channel;
  int8_t rssi;
  uint8_t curbssid[6];

  /* Scan results as a wireless event stream (SIOCGIWSCAN) */

  uint8_t scanstate;
  uint8_t scancount;
  clock_t scanstart;
  FAR uint8_t *scanbuf;
  size_t scanlen;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int wlan_ifup(FAR struct netdev_lowerhalf_s *dev);
static int wlan_ifdown(FAR struct netdev_lowerhalf_s *dev);
static int wlan_transmit(FAR struct netdev_lowerhalf_s *dev,
                         FAR netpkt_t *pkt);
static FAR netpkt_t *wlan_receive(FAR struct netdev_lowerhalf_s *dev);

static int wlan_connect(FAR struct netdev_lowerhalf_s *dev);
static int wlan_disconnect(FAR struct netdev_lowerhalf_s *dev);
static int wlan_essid(FAR struct netdev_lowerhalf_s *dev,
                      FAR struct iwreq *iwr, bool set);
static int wlan_bssid(FAR struct netdev_lowerhalf_s *dev,
                      FAR struct iwreq *iwr, bool set);
static int wlan_passwd(FAR struct netdev_lowerhalf_s *dev,
                       FAR struct iwreq *iwr, bool set);
static int wlan_mode(FAR struct netdev_lowerhalf_s *dev,
                     FAR struct iwreq *iwr, bool set);
static int wlan_auth(FAR struct netdev_lowerhalf_s *dev,
                     FAR struct iwreq *iwr, bool set);
static int wlan_freq(FAR struct netdev_lowerhalf_s *dev,
                     FAR struct iwreq *iwr, bool set);
static int wlan_scan(FAR struct netdev_lowerhalf_s *dev,
                     FAR struct iwreq *iwr, bool set);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct netdev_ops_s g_wlan_ops =
{
  .ifup     = wlan_ifup,
  .ifdown   = wlan_ifdown,
  .transmit = wlan_transmit,
  .receive  = wlan_receive,
};

static const struct wireless_ops_s g_wlan_iw_ops =
{
  .connect    = wlan_connect,
  .disconnect = wlan_disconnect,
  .essid      = wlan_essid,
  .bssid      = wlan_bssid,
  .passwd     = wlan_passwd,
  .mode       = wlan_mode,
  .auth       = wlan_auth,
  .freq       = wlan_freq,
  .scan       = wlan_scan,
};

static struct pic32mz_wlan_s g_wlan;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* [EX] Read by the library: number of buffers in the reserved pool */

const uint8_t pic32mzw_rsr_pkt_num = WLAN_PKT_NUM;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline FAR struct wlan_hdr_s *wlan_hdr(FAR void *buf)
{
  return (FAR struct wlan_hdr_s *)buf - 1;
}

static inline FAR uint8_t *wlan_data(FAR sq_entry_t *entry)
{
  return (FAR uint8_t *)((FAR struct wlan_hdr_s *)entry + 1);
}

static inline bool wlan_inpool(FAR struct wlan_hdr_s *hdr)
{
  return (uintptr_t)hdr >= WLAN_PKTMEM_BASE &&
         (uintptr_t)hdr < WLAN_PKTMEM_BASE + WLAN_PKTMEM_SIZE;
}

static void wlan_pool_init(void)
{
  uintptr_t addr = WLAN_PKTMEM_BASE;
  int i;

  sq_init(&g_wlan.pool);
  for (i = 0; i < WLAN_PKT_NUM; i++)
    {
      sq_addlast((FAR sq_entry_t *)addr, &g_wlan.pool);
      addr += (WLAN_PKT_NODESIZE + 3) & ~3;
    }
}

/* WID message builder ([EX] format: 'W'/'Q', 0, total length (16-bit LE)
 * including this 4-byte header, then the WIDs).
 */

static int wid_init(FAR struct wid_msg_s *msg, char op)
{
  msg->buf = DRV_PIC32MZW_MemAlloc(WID_MSG_MAX);
  if (msg->buf == NULL)
    {
      return -ENOMEM;
    }

  msg->ptr = msg->buf + 4;
  msg->op  = op;
  return OK;
}

static void wid_value(FAR struct wid_msg_s *msg, uint16_t wid,
                      uint32_t val)
{
  int len = 1 << WID_TYPE(wid);
  int i;

  *msg->ptr++ = wid & 0xff;
  *msg->ptr++ = wid >> 8;
  *msg->ptr++ = len;
  for (i = 0; i < len; i++)
    {
      *msg->ptr++ = val & 0xff;
      val >>= 8;
    }
}

static void wid_data(FAR struct wid_msg_s *msg, uint16_t wid,
                     FAR const uint8_t *data, uint16_t len)
{
  uint8_t sum = 0;
  int i;

  *msg->ptr++ = wid & 0xff;
  *msg->ptr++ = wid >> 8;
  *msg->ptr++ = len & 0xff;
  if (WID_TYPE(wid) == WID_TYPE_BIN)
    {
      *msg->ptr++ = len >> 8;
    }

  memcpy(msg->ptr, data, len);
  msg->ptr += len;

  /* [EX] Binary WIDs end with an 8-bit sum of the data */

  if (WID_TYPE(wid) == WID_TYPE_BIN)
    {
      for (i = 0; i < len; i++)
        {
          sum += data[i];
        }

      *msg->ptr++ = sum;
    }
}

static void wid_query(FAR struct wid_msg_s *msg, uint16_t wid)
{
  *msg->ptr++ = wid & 0xff;
  *msg->ptr++ = wid >> 8;
}

static void wid_send(FAR struct wid_msg_s *msg)
{
  uint16_t len = msg->ptr - msg->buf;
  irqstate_t flags;

  DEBUGASSERT(len <= WID_MSG_MAX);

  msg->buf[0] = msg->op;
  msg->buf[1] = 0;
  msg->buf[2] = len & 0xff;
  msg->buf[3] = len >> 8;

  flags = enter_critical_section();
  sq_addlast((FAR sq_entry_t *)wlan_hdr(msg->buf), &g_wlan.txq);
  leave_critical_section(flags);

  nxsem_post(&g_wlan.evsem);
}

/* Append one wireless event to the scan buffer; the caller has checked
 * that it fits.
 */

static FAR struct iw_event *wlan_scan_event(int cmd, size_t len)
{
  FAR struct iw_event *iwe;

  iwe = (FAR struct iw_event *)&g_wlan.scanbuf[g_wlan.scanlen];
  memset(iwe, 0, len);
  iwe->len = len;
  iwe->cmd = cmd;
  g_wlan.scanlen += len;
  return iwe;
}

static void wlan_scan_finish(void)
{
  g_wlan.scanstate = WLAN_SCAN_DONE;
  wlinfo("scan done, %u BSS\n", g_wlan.scancount);
}

static void wlan_scanresult(FAR const uint8_t *p, uint16_t len)
{
  /* [EX] Layout: index, ofTotal, bssid[6], ssid[32], ssid length, rssi,
   * bss type, channel, 11i info (16-bit, at offset 44).
   */

  FAR struct iw_event *iwe;
  size_t ssidlen;
  size_t ssidpad;
  size_t need;
  uint16_t dot11i;

  if (g_wlan.scanstate != WLAN_SCAN_RUNNING || len < 44)
    {
      return;
    }

  if (p[1] == 0)
    {
      wlan_scan_finish();
      return;
    }

  ssidlen = p[40] > IW_ESSID_MAX_SIZE ? IW_ESSID_MAX_SIZE : p[40];
  ssidpad = (ssidlen + 3) & ~3;
  dot11i  = len >= 46 ? (p[44] | (p[45] << 8)) : 0;

  need = IW_EVENT_SIZE(ap_addr) + IW_EVENT_SIZE(essid) + ssidpad +
         IW_EVENT_SIZE(qual) + IW_EVENT_SIZE(mode) +
         IW_EVENT_SIZE(data) + IW_EVENT_SIZE(freq);

  if (g_wlan.scanlen + need <= WLAN_SCAN_BUFSIZE)
    {
      iwe = wlan_scan_event(SIOCGIWAP, IW_EVENT_SIZE(ap_addr));
      iwe->u.ap_addr.sa_family = ARPHRD_ETHER;
      memcpy(iwe->u.ap_addr.sa_data, &p[2], 6);

      iwe = wlan_scan_event(SIOCGIWESSID, IW_EVENT_SIZE(essid) + ssidpad);
      iwe->u.essid.length  = ssidlen;
      iwe->u.essid.flags   = 1;

      /* Special processing for iw_point: offset in the pointer field */

      iwe->u.essid.pointer = (FAR void *)sizeof(iwe->u.essid);
      memcpy(&iwe->u.essid + 1, &p[8], ssidlen);

      iwe = wlan_scan_event(IWEVQUAL, IW_EVENT_SIZE(qual));
      iwe->u.qual.level   = (int8_t)p[41];
      iwe->u.qual.updated = IW_QUAL_DBM | IW_QUAL_ALL_UPDATED;

      iwe = wlan_scan_event(SIOCGIWMODE, IW_EVENT_SIZE(mode));
      iwe->u.mode = IW_MODE_MASTER;

      iwe = wlan_scan_event(SIOCGIWENCODE, IW_EVENT_SIZE(data));
      iwe->u.data.flags = (dot11i & DOT11I_PRIVACY) ?
                          IW_ENCODE_ENABLED | IW_ENCODE_NOKEY :
                          IW_ENCODE_DISABLED;

      iwe = wlan_scan_event(SIOCGIWFREQ, IW_EVENT_SIZE(freq));
      iwe->u.freq.m = p[43];
    }
  else
    {
      wlwarn("scan buffer full\n");
    }

  if (++g_wlan.scancount >= p[1])
    {
      wlan_scan_finish();
    }
}

static void wlan_assoc(uint8_t status)
{
  struct wid_msg_s msg;

  /* [EX] ASSOC_STAT: 0 disconnected, 1 connected, 2 roamed,
   * 3 reconnected.
   */

  if (g_wlan.ap)
    {
      /* [EX] In AP mode 1 means the AP has started, 0 that it stopped */

      if (status == 1 && !g_wlan.connected)
        {
          g_wlan.connected = true;
          syslog(LOG_INFO, "wlan: AP started\n");

          if (g_wlan.registered)
            {
              netdev_lower_carrier_on(&g_wlan.dev);
            }
        }
      else if (status == 0 && g_wlan.connected)
        {
          g_wlan.connected = false;
          syslog(LOG_INFO, "wlan: AP stopped\n");

          if (g_wlan.registered)
            {
              netdev_lower_carrier_off(&g_wlan.dev);
            }
        }

      return;
    }

  if (status == 1 && !g_wlan.connected)
    {
      g_wlan.connected = true;
      syslog(LOG_INFO, "wlan: connected\n");

      if (wid_init(&msg, 'Q') == OK)
        {
          wid_query(&msg, WID_RSSI);
          wid_query(&msg, WID_BSSID);
          wid_query(&msg, WID_CURR_OPER_CHANNEL);
          wid_send(&msg);
        }

      if (g_wlan.registered)
        {
          netdev_lower_carrier_on(&g_wlan.dev);
        }
    }
  else if (status == 0 && g_wlan.connected)
    {
      g_wlan.connected = false;
      syslog(LOG_INFO, "wlan: disconnected\n");

      if (g_wlan.registered)
        {
          netdev_lower_carrier_off(&g_wlan.dev);
        }
    }
}

static void wlan_widprocess(uint16_t wid, uint16_t len,
                            FAR const uint8_t *data)
{
  if (len < 1)
    {
      return;
    }

  switch (wid)
    {
      case WID_MAC_ADDR:
        if (len >= 6)
          {
            memcpy(g_wlan.mac, data, 6);
            g_wlan.macvalid = true;
          }
        break;

      case WID_RF_MAC_CONFIG_STATUS:
        g_wlan.rfmac_status = data[0];
        wlinfo("RF/MAC config status 0x%02x\n", data[0]);
        break;

      case WID_GET_SCAN_RESULTS:
        wlan_scanresult(data, len);
        break;

      case WID_ASSOC_STAT:
        wlan_assoc(data[0]);
        break;

      case WID_STA_JOIN_INFO:

        /* [EX] Joined flag, station MAC address, association ID */

        if (g_wlan.ap && len >= 8)
          {
            syslog(LOG_INFO,
                   "wlan: station %02x:%02x:%02x:%02x:%02x:%02x %s "
                   "(aid %u)\n", data[1], data[2], data[3], data[4],
                   data[5], data[6], data[0] ? "joined" : "left", data[7]);
          }
        break;

      case WID_RSSI:
        g_wlan.rssi = (int8_t)data[0];
        break;

      case WID_BSSID:
        if (len >= 6)
          {
            memcpy(g_wlan.curbssid, data, 6);
          }
        break;

      case WID_CURR_OPER_CHANNEL:
        g_wlan.channel = data[0];
        break;

      default:
        wlinfo("WID 0x%04x len %u\n", wid, len);
        break;
    }

  if (!g_wlan.ready && g_wlan.macvalid &&
      (g_wlan.rfmac_status & RFMAC_MIN_CONFIG) == RFMAC_MIN_CONFIG)
    {
      g_wlan.ready = true;
      nxsem_post(&g_wlan.readysem);
    }
}

/* Parse a response from the library ([EX] 'R' or 'I', 0, total length,
 * then WIDs; binary WIDs have a 16-bit length and a trailing checksum).
 */

static void wlan_response(FAR uint8_t *rsp)
{
  FAR uint8_t *p = rsp;
  uint16_t remain;
  uint16_t wid;
  uint16_t len;

  if (p[0] != 'R' && p[0] != 'I')
    {
      wlinfo("message '%c'\n", p[0]);
      goto out;
    }

  remain = p[2] | (p[3] << 8);
  if (remain < 4)
    {
      goto out;
    }

  remain -= 4;
  p += 4;

  while (remain >= 3)
    {
      wid = p[0] | (p[1] << 8);
      len = p[2];
      p += 3;
      remain -= 3;

      if (WID_TYPE(wid) == WID_TYPE_BIN)
        {
          if (remain < 1)
            {
              break;
            }

          len |= *p++ << 8;
          remain--;
        }

      if (remain < len)
        {
          break;
        }

      wlan_widprocess(wid, len, p);
      p += len;
      remain -= len;

      if (WID_TYPE(wid) == WID_TYPE_BIN && remain > 0)
        {
          p++;
          remain--;
        }
    }

out:
  DRV_PIC32MZW_PacketMemFree(rsp);
}

static int wlan_isr(int irq, FAR void *context, FAR void *arg)
{
  switch (irq)
    {
      case PIC32MZ_IRQ_RFMAC:
        wdrv_pic32mzw_mac_isr(1);
        break;

      case PIC32MZ_IRQ_RFTM0:
        wdrv_pic32mzw_timer_tick_isr(0);
        break;

      case PIC32MZ_IRQ_RFSMC:
        wdrv_pic32mzw_smc_isr(1);
        break;
    }

  mips_clrpend_irq(irq);
  nxsem_post(&g_wlan.evsem);
  return OK;
}

static void wlan_send_init(void)
{
  struct wid_msg_s msg;

  /* [EX] Same initial configuration as Harmony's defaults: generic
   * regulatory domain, QoS on, power save off, coexistence off.
   */

  static const uint8_t regdom[6] = "GEN";

  if (wid_init(&msg, 'W') == OK)
    {
      wid_data(&msg, WID_REG_DOMAIN, regdom, sizeof(regdom));
      wid_value(&msg, WID_QOS_ENABLE, 1);
      wid_value(&msg, WID_POWER_MANAGEMENT, 0);
      wid_value(&msg, WID_PS_CORRELATION, 0);
      wid_value(&msg, WID_COEX_ENABLE, 0);
      wid_send(&msg);
    }

  if (wid_init(&msg, 'Q') == OK)
    {
      wid_query(&msg, WID_MAC_ADDR);
      wid_send(&msg);
    }
}

/* Run the deferred crypto callbacks; called with the library lock held */

static void wlan_crypto_run(void)
{
  struct wlan_crypto_defer_s d;
  irqstate_t flags;

  for (; ; )
    {
      flags = enter_critical_section();
      if (g_wlan.defer_head == g_wlan.defer_tail)
        {
          leave_critical_section(flags);
          break;
        }

      d = g_wlan.defer[g_wlan.defer_tail];
      g_wlan.defer_tail = (g_wlan.defer_tail + 1) % WLAN_CRYPTO_DEFER_MAX;
      leave_critical_section(flags);

      d.cb(d.result, d.context);
    }
}

static int wlan_thread(int argc, FAR char *argv[])
{
  FAR sq_entry_t *entry;
  irqstate_t flags;

  nxmutex_lock(&g_wlan.lock);
  wdrv_pic32mzw_user_main();
  nxmutex_unlock(&g_wlan.lock);

  wlan_send_init();

  for (; ; )
    {
      nxsem_wait_uninterruptible(&g_wlan.evsem);

      nxmutex_lock(&g_wlan.lock);

      flags = enter_critical_section();
      entry = sq_remfirst(&g_wlan.txq);
      leave_critical_section(flags);

      if (entry != NULL)
        {
          wdrv_pic32mzw_process_cfg_message(wlan_data(entry));
        }

      wlan_crypto_run();
      wdrv_pic32mzw_mac_controller_task();
      nxmutex_unlock(&g_wlan.lock);

      for (; ; )
        {
          flags = enter_critical_section();
          entry = sq_remfirst(&g_wlan.rxq);
          leave_critical_section(flags);

          if (entry == NULL)
            {
              break;
            }

          wlan_response(wlan_data(entry));
        }

      if (g_wlan.scanstate == WLAN_SCAN_RUNNING &&
          clock_systime_ticks() - g_wlan.scanstart > WLAN_SCAN_TIMEOUT)
        {
          wlwarn("scan timeout\n");
          wlan_scan_finish();
        }
    }

  return OK;
}

/****************************************************************************
 * Network device operations
 ****************************************************************************/

static int wlan_ifup(FAR struct netdev_lowerhalf_s *dev)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(&g_wlan.rxlock);
  netpkt_free_queue(&g_wlan.rxqueue);
  spin_unlock_irqrestore(&g_wlan.rxlock, flags);

  if (g_wlan.connected)
    {
      netdev_lower_carrier_on(dev);
    }

  return OK;
}

static int wlan_ifdown(FAR struct netdev_lowerhalf_s *dev)
{
  return OK;
}

static int wlan_transmit(FAR struct netdev_lowerhalf_s *dev,
                         FAR netpkt_t *pkt)
{
  unsigned int len = netpkt_getdatalen(dev, pkt);
  FAR uint8_t *buf;

  if (!g_wlan.connected || len > WLAN_PKT_BUFSIZE - WLAN_TX_HDROFFSET)
    {
      netpkt_free(dev, pkt, NETPKT_TX);
      return OK;
    }

  buf = DRV_PIC32MZW_PacketMemAlloc(WLAN_TX_HDROFFSET + len,
                                    WLAN_MEM_PRI_TX);
  if (buf == NULL)
    {
      return -ENOMEM;
    }

  netpkt_copyout(dev, buf + WLAN_TX_HDROFFSET, pkt, len, 0);
  netpkt_free(dev, pkt, NETPKT_TX);

  /* [EX] The library frees the buffer once it has been sent */

  nxmutex_lock(&g_wlan.lock);
  wdrv_pic32mzw_wlan_send_packet(buf, len, 0, 0);
  nxmutex_unlock(&g_wlan.lock);

  nxsem_post(&g_wlan.evsem);
  return OK;
}

static FAR netpkt_t *wlan_receive(FAR struct netdev_lowerhalf_s *dev)
{
  FAR netpkt_t *pkt;
  irqstate_t flags;

  flags = spin_lock_irqsave(&g_wlan.rxlock);
  pkt = netpkt_remove_queue(&g_wlan.rxqueue);
  spin_unlock_irqrestore(&g_wlan.rxlock, flags);

  return pkt;
}

/****************************************************************************
 * Wireless operations
 ****************************************************************************/

/* Convert the requested WPA version to 11i settings */

static int wlan_dot11i(FAR uint32_t *dot11i)
{
  switch (g_wlan.wpaver)
    {
      case IW_AUTH_WPA_VERSION_DISABLED:
        *dot11i = 0;
        break;

      case IW_AUTH_WPA_VERSION_WPA:
        *dot11i = DOT11I_WPAWPA2_PERSONAL;
        break;

      case IW_AUTH_WPA_VERSION_WPA2:
        *dot11i = DOT11I_WPA2_PERSONAL;
        break;

#ifdef CONFIG_PIC32MZ_W1_BA414E
      case IW_AUTH_WPA_VERSION_WPA3:
        *dot11i = DOT11I_WPA3_PERSONAL;
        break;
#endif

      default:
        wlerr("WPA version %" PRIu32 " not supported\n", g_wlan.wpaver);
        return -ENOTSUP;
    }

  if (*dot11i != 0 && g_wlan.psklen < ((*dot11i & DOT11I_PSK) ? 8 : 1))
    {
      wlerr("no passphrase\n");
      return -EINVAL;
    }

  return OK;
}

static int wlan_apstart(uint32_t dot11i)
{
  struct wid_msg_s msg;
  uint8_t channel;
  int ret;

  ret = wid_init(&msg, 'W');
  if (ret < 0)
    {
      return ret;
    }

  channel = g_wlan.reqchannel ? g_wlan.reqchannel : WLAN_AP_CHANNEL;

  /* [EX] Soft-AP start, as Harmony's WDRV_PIC32MZW_APStart() */

  wid_value(&msg, WID_SWITCH_MODE, 1);
  wid_value(&msg, WID_BCAST_SSID, 0);
  wid_value(&msg, WID_CURRENT_TX_RATE, 0);
  wid_data(&msg, WID_SSID, g_wlan.ssid, g_wlan.ssidlen);
  wid_value(&msg, WID_USER_PREF_CHANNEL, channel);
  wid_value(&msg, WID_11I_SETTINGS, dot11i | DOT11I_AP);
  if (dot11i & DOT11I_PSK)
    {
      wid_data(&msg, WID_11I_PSK, g_wlan.psk, g_wlan.psklen);
    }

  if (dot11i & DOT11I_SAE)
    {
      wid_data(&msg, WID_RSNA_PASSWORD, g_wlan.psk, g_wlan.psklen);
    }

  wid_value(&msg, WID_11G_OPERATING_MODE, 2);
  wid_value(&msg, WID_ACK_POLICY, 0);
  wid_value(&msg, WID_11N_ENABLE, 1);
  wid_send(&msg);

  g_wlan.channel = channel;
  syslog(LOG_INFO, "wlan: starting AP %.*s on channel %u\n",
         g_wlan.ssidlen, g_wlan.ssid, channel);
  return OK;
}

static int wlan_connect(FAR struct netdev_lowerhalf_s *dev)
{
  struct wid_msg_s msg;
  uint32_t dot11i;
  int ret;

  if (!g_wlan.ready || g_wlan.ssidlen == 0)
    {
      return -EINVAL;
    }

  ret = wlan_dot11i(&dot11i);
  if (ret < 0)
    {
      return ret;
    }

  if (g_wlan.ap)
    {
      return g_wlan.connected ? -EBUSY : wlan_apstart(dot11i);
    }

  ret = wid_init(&msg, 'W');
  if (ret < 0)
    {
      return ret;
    }

  /* [EX] Station connect, as Harmony's WDRV_PIC32MZW_BSSConnect() */

  wid_value(&msg, WID_SWITCH_MODE, 0);
  wid_value(&msg, WID_CURRENT_TX_RATE, 0);
  if (g_wlan.bssidvalid)
    {
      wid_data(&msg, WID_BSSID, g_wlan.bssid, 6);
    }

  wid_data(&msg, WID_SSID, g_wlan.ssid, g_wlan.ssidlen);
  wid_value(&msg, WID_SCAN_FILTER, g_wlan.reqchannel ? 0x10 : 0);
  wid_value(&msg, WID_USER_SCAN_CHANNEL,
            g_wlan.reqchannel ? g_wlan.reqchannel : 0xff);
  wid_value(&msg, WID_11I_SETTINGS, dot11i);
  if (dot11i & DOT11I_PSK)
    {
      wid_data(&msg, WID_11I_PSK, g_wlan.psk, g_wlan.psklen);
    }

  if (dot11i & DOT11I_SAE)
    {
      wid_data(&msg, WID_RSNA_PASSWORD, g_wlan.psk, g_wlan.psklen);
    }

  wid_value(&msg, WID_11G_OPERATING_MODE, 2);
  wid_value(&msg, WID_ACK_POLICY, 0);
  wid_value(&msg, WID_11N_ENABLE, 1);
  wid_value(&msg, WID_PREAMBLE, 2);
  wid_send(&msg);

  syslog(LOG_INFO, "wlan: connecting to %.*s\n", g_wlan.ssidlen,
         g_wlan.ssid);
  return OK;
}

static int wlan_disconnect(FAR struct netdev_lowerhalf_s *dev)
{
  struct wid_msg_s msg;
  int ret;

  ret = wid_init(&msg, 'W');
  if (ret < 0)
    {
      return ret;
    }

  if (g_wlan.ap)
    {
      /* [EX] Back to STA mode with an empty SSID, as Harmony's
       * WDRV_PIC32MZW_APStop()
       */

      wid_value(&msg, WID_SWITCH_MODE, 0);
      wid_data(&msg, WID_SSID, NULL, 0);
      wid_send(&msg);

      if (g_wlan.connected)
        {
          g_wlan.connected = false;
          netdev_lower_carrier_off(dev);
          syslog(LOG_INFO, "wlan: AP stopped\n");
        }

      return OK;
    }

  /* [EX] Empty SSID followed by DISCONNECT, as Harmony does */

  wid_data(&msg, WID_SSID, NULL, 0);
  wid_value(&msg, WID_DISCONNECT, 1);
  wid_send(&msg);
  return OK;
}

static int wlan_essid(FAR struct netdev_lowerhalf_s *dev,
                      FAR struct iwreq *iwr, bool set)
{
  FAR struct iw_point *essid = &iwr->u.essid;

  if (set)
    {
      if (essid->pointer == NULL || essid->length > IW_ESSID_MAX_SIZE)
        {
          return -EINVAL;
        }

      memcpy(g_wlan.ssid, essid->pointer, essid->length);
      g_wlan.ssidlen = essid->length;
      return OK;
    }

  if (essid->pointer == NULL || essid->length < g_wlan.ssidlen)
    {
      return -EINVAL;
    }

  memcpy(essid->pointer, g_wlan.ssid, g_wlan.ssidlen);
  essid->length = g_wlan.ssidlen;
  essid->flags  = g_wlan.connected ? IW_ESSID_ON : IW_ESSID_OFF;
  return OK;
}

static int wlan_bssid(FAR struct netdev_lowerhalf_s *dev,
                      FAR struct iwreq *iwr, bool set)
{
  if (set)
    {
      memcpy(g_wlan.bssid, iwr->u.ap_addr.sa_data, 6);
      g_wlan.bssidvalid = true;
      return OK;
    }

  iwr->u.ap_addr.sa_family = ARPHRD_ETHER;
  if (g_wlan.ap)
    {
      memcpy(iwr->u.ap_addr.sa_data, g_wlan.mac, 6);
    }
  else if (g_wlan.connected)
    {
      memcpy(iwr->u.ap_addr.sa_data, g_wlan.curbssid, 6);
    }
  else
    {
      memset(iwr->u.ap_addr.sa_data, 0, 6);
    }

  return OK;
}

static int wlan_passwd(FAR struct netdev_lowerhalf_s *dev,
                       FAR struct iwreq *iwr, bool set)
{
  FAR struct iw_encode_ext *ext = iwr->u.encoding.pointer;

  if (ext == NULL)
    {
      return -EINVAL;
    }

  if (set)
    {
      if (ext->key_len > sizeof(g_wlan.psk))
        {
          return -EINVAL;
        }

      memcpy(g_wlan.psk, ext->key, ext->key_len);
      g_wlan.psklen = ext->key_len;
      return OK;
    }

  if (iwr->u.encoding.length < sizeof(*ext) + g_wlan.psklen)
    {
      return -E2BIG;
    }

  ext->alg     = g_wlan.psklen ? IW_ENCODE_ALG_CCMP : IW_ENCODE_ALG_NONE;
  ext->key_len = g_wlan.psklen;
  memcpy(ext->key, g_wlan.psk, g_wlan.psklen);
  return OK;
}

static int wlan_mode(FAR struct netdev_lowerhalf_s *dev,
                     FAR struct iwreq *iwr, bool set)
{
  if (set)
    {
      if (iwr->u.mode != IW_MODE_INFRA && iwr->u.mode != IW_MODE_MASTER)
        {
          return -ENOTSUP;
        }

      /* The library runs either a station or an AP, not both */

      if (g_wlan.connected &&
          g_wlan.ap != (iwr->u.mode == IW_MODE_MASTER))
        {
          return -EBUSY;
        }

      g_wlan.ap = iwr->u.mode == IW_MODE_MASTER;
      return OK;
    }

  iwr->u.mode = g_wlan.ap ? IW_MODE_MASTER : IW_MODE_INFRA;
  return OK;
}

static int wlan_auth(FAR struct netdev_lowerhalf_s *dev,
                     FAR struct iwreq *iwr, bool set)
{
  int index = iwr->u.param.flags & IW_AUTH_INDEX;

  switch (index)
    {
      case IW_AUTH_WPA_VERSION:
        if (set)
          {
            g_wlan.wpaver = iwr->u.param.value;
          }
        else
          {
            iwr->u.param.value = g_wlan.wpaver;
          }

        return OK;

      case IW_AUTH_CIPHER_PAIRWISE:
      case IW_AUTH_CIPHER_GROUP:
        if (set)
          {
            g_wlan.cipher = iwr->u.param.value;
          }
        else
          {
            iwr->u.param.value = g_wlan.cipher;
          }

        return OK;

      default:
        return -ENOTSUP;
    }
}

static int wlan_freq(FAR struct netdev_lowerhalf_s *dev,
                     FAR struct iwreq *iwr, bool set)
{
  if (set)
    {
      /* Channel number (e == 0) for the next connect; 0 means any */

      if (iwr->u.freq.e != 0 || iwr->u.freq.m < 0 || iwr->u.freq.m > 14)
        {
          return -EINVAL;
        }

      g_wlan.reqchannel = iwr->u.freq.m;
      return OK;
    }

  iwr->u.freq.m     = g_wlan.connected ? g_wlan.channel : 0;
  iwr->u.freq.e     = 0;
  iwr->u.freq.flags = IW_FREQ_FIXED;
  return OK;
}

static int wlan_scan(FAR struct netdev_lowerhalf_s *dev,
                     FAR struct iwreq *iwr, bool set)
{
  struct wid_msg_s msg;
  int ret;

  if (set)
    {
      if (!g_wlan.ready)
        {
          return -EAGAIN;
        }

      if (g_wlan.scanstate == WLAN_SCAN_RUNNING ||
          (g_wlan.ap && g_wlan.connected))
        {
          return -EBUSY;
        }

      if (g_wlan.scanbuf == NULL)
        {
          g_wlan.scanbuf = kmm_malloc(WLAN_SCAN_BUFSIZE);
          if (g_wlan.scanbuf == NULL)
            {
              return -ENOMEM;
            }
        }

      ret = wid_init(&msg, 'W');
      if (ret < 0)
        {
          return ret;
        }

      g_wlan.scanlen   = 0;
      g_wlan.scancount = 0;
      g_wlan.scanstart = clock_systime_ticks();
      g_wlan.scanstate = WLAN_SCAN_RUNNING;

      /* [EX] Active scan on all channels with Harmony's defaults: 20 ms
       * per channel, channels 1-11.
       */

      wid_value(&msg, WID_SCAN_FILTER, 0);
      wid_value(&msg, WID_USER_SCAN_CHANNEL, 0xff);
      wid_value(&msg, WID_ACTIVE_SCAN_TIME, 20);
      wid_value(&msg, WID_SCAN_TYPE, 1);
      wid_value(&msg, WID_SCAN_CH_BITMAP_2GHZ, WLAN_CHANNEL_MASK);
      wid_value(&msg, WID_BCAST_SSID, 0);
      wid_value(&msg, WID_START_SCAN_REQ, 1);
      wid_send(&msg);
      return OK;
    }

  switch (g_wlan.scanstate)
    {
      case WLAN_SCAN_RUNNING:
        return -EAGAIN;

      case WLAN_SCAN_IDLE:
        iwr->u.data.length = 0;
        return OK;

      default:
        break;
    }

  if (iwr->u.data.pointer == NULL || iwr->u.data.length < g_wlan.scanlen)
    {
      iwr->u.data.length = g_wlan.scanlen;
      return -E2BIG;
    }

  memcpy(iwr->u.data.pointer, g_wlan.scanbuf, g_wlan.scanlen);
  iwr->u.data.length = g_wlan.scanlen;
  return OK;
}

/****************************************************************************
 * Library callbacks
 ****************************************************************************/

/* General memory: cache-line aligned, used through KSEG1 (uncached) as
 * Harmony does, so the library needs no cache maintenance.
 */

FAR void *DRV_PIC32MZW_MemAlloc(uint16_t size)
{
  FAR struct wlan_hdr_s *hdr;
  FAR void *alloc;
  size_t total;

  total = (sizeof(struct wlan_hdr_s) + size + WLAN_CACHE_LINE - 1) &
          ~(WLAN_CACHE_LINE - 1);

  alloc = kmm_memalign(WLAN_CACHE_LINE, total);
  if (alloc == NULL)
    {
      wlerr("MemAlloc(%u) failed\n", size);
      return NULL;
    }

  up_flush_dcache((uintptr_t)alloc, (uintptr_t)alloc + total);

  hdr        = (FAR struct wlan_hdr_s *)((uintptr_t)alloc | KSEG1_BASE);
  hdr->next  = NULL;
  hdr->alloc = alloc;
  hdr->size  = size;
  hdr->users = 1;
  hdr->prio  = -1;
  hdr->spare = NULL;

  return hdr + 1;
}

int8_t DRV_PIC32MZW_MemAddUsers(FAR void *buf, int count)
{
  irqstate_t flags;

  if (buf == NULL)
    {
      return 0;
    }

  flags = enter_critical_section();
  wlan_hdr(buf)->users += count;
  leave_critical_section(flags);
  return 1;
}

int8_t DRV_PIC32MZW_MemFree(FAR void *buf)
{
  FAR struct wlan_hdr_s *hdr;
  irqstate_t flags;

  if (buf == NULL)
    {
      return 0;
    }

  hdr = wlan_hdr(buf);

  flags = enter_critical_section();
  if (--hdr->users > 0)
    {
      leave_critical_section(flags);
      return 0;
    }

  if (hdr->prio >= 0)
    {
      g_pktmem_pri[hdr->prio].num_allocd--;
    }

  if (wlan_inpool(hdr))
    {
      sq_addlast((FAR sq_entry_t *)hdr, &g_wlan.pool);
      leave_critical_section(flags);
      return 1;
    }

  leave_critical_section(flags);

  kmm_free(hdr->alloc);
  return 1;
}

FAR void *DRV_PIC32MZW_PacketMemAlloc(uint16_t size, int prio)
{
  FAR struct wlan_hdr_s *hdr = NULL;
  FAR void *buf;
  irqstate_t flags;

  if (prio < 0 || prio >= 5)
    {
      return NULL;
    }

  if (size <= WLAN_PKT_BUFSIZE)
    {
      flags = enter_critical_section();
      hdr = (FAR struct wlan_hdr_s *)sq_remfirst(&g_wlan.pool);
      leave_critical_section(flags);
    }

  if (hdr != NULL)
    {
      hdr->next  = NULL;
      hdr->alloc = NULL;
      hdr->size  = size;
      hdr->users = 1;
      hdr->spare = NULL;
      buf = hdr + 1;
    }
  else
    {
      buf = DRV_PIC32MZW_MemAlloc(size);
      if (buf == NULL)
        {
          return NULL;
        }

      hdr = wlan_hdr(buf);
    }

  flags = enter_critical_section();
  hdr->prio = prio;
  g_pktmem_pri[prio].num_allocd++;
  leave_critical_section(flags);

  return buf;
}

void DRV_PIC32MZW_PacketMemFree(FAR void *buf)
{
  DRV_PIC32MZW_MemFree(buf);
}

void DRV_PIC32MZW_WIDRxQueuePush(FAR void *buf)
{
  irqstate_t flags;

  if (buf == NULL)
    {
      return;
    }

  flags = enter_critical_section();
  sq_addlast((FAR sq_entry_t *)wlan_hdr(buf), &g_wlan.rxq);
  leave_critical_section(flags);
}

/* Received Ethernet frame, called from the WLAN thread.  [EX] The frame
 * lives in a library buffer that starts 'offset' bytes before it.
 */

void DRV_PIC32MZW_MACEthernetSendPacket(FAR const uint8_t *frame,
                                        uint16_t len, uint8_t offset)
{
  FAR netpkt_t *pkt = NULL;
  irqstate_t flags;
  int ret = -ENODEV;

  if (g_wlan.registered)
    {
      pkt = netpkt_alloc(&g_wlan.dev, NETPKT_RX);
      ret = -ENOMEM;
    }

  if (pkt != NULL)
    {
      ret = netpkt_copyin(&g_wlan.dev, pkt, frame, len, 0);
      if (ret >= 0)
        {
          flags = spin_lock_irqsave(&g_wlan.rxlock);
          ret = netpkt_tryadd_queue(pkt, &g_wlan.rxqueue);
          spin_unlock_irqrestore(&g_wlan.rxlock, flags);
        }

      if (ret < 0)
        {
          netpkt_free(&g_wlan.dev, pkt, NETPKT_RX);
        }
    }

  DRV_PIC32MZW_MemFree((FAR void *)(frame - offset));

  if (ret >= 0)
    {
      netdev_lower_rxready(&g_wlan.dev);
    }
}

void DRV_PIC32MZW_MACTimer0Enable(void)
{
  up_enable_irq(PIC32MZ_IRQ_RFTM0);
}

void DRV_PIC32MZW_MACTimer0Disable(void)
{
  up_disable_irq(PIC32MZ_IRQ_RFTM0);
}

void DRV_PIC32MZW_MACTimer1Enable(void)
{
  up_enable_irq(PIC32MZ_IRQ_RFTM1);
}

void DRV_PIC32MZW_MACTimer1Disable(void)
{
  up_disable_irq(PIC32MZ_IRQ_RFTM1);
}

bool DRV_PIC32MZW_BTPriorityRead(void)
{
  /* [EX] No Bluetooth coexistence: Harmony returns true */

  return true;
}

/* The library prints its own messages (RF calibration, MAC state...)
 * through printf(), puts() and putchar(), renamed in the copy that is
 * linked (see Make.defs).  Send them to the syslog with the wireless debug
 * output, or drop them.
 */

static void wlan_libprint(FAR const IPTR char *fmt, va_list ap)
{
#ifdef CONFIG_DEBUG_WIRELESS_INFO
  vsyslog(LOG_INFO, fmt, ap);
#endif
}

int pic32mzw1_printf(FAR const IPTR char *fmt, ...)
{
  va_list ap;

  va_start(ap, fmt);
  wlan_libprint(fmt, ap);
  va_end(ap);
  return 0;
}

int pic32mzw1_printf_npux(FAR const IPTR char *fmt, ...)
{
  va_list ap;

  va_start(ap, fmt);
  wlan_libprint(fmt, ap);
  va_end(ap);
  return 0;
}

int pic32mzw1_puts(FAR const char *str)
{
  wlinfo("%s\n", str);
  return 0;
}

int pic32mzw1_putchar(int c)
{
  wlinfo("%c", c);
  return c;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mz_wlan_crypto_defer
 *
 * Description:
 *   Queue a crypto completion callback; the WLAN thread runs it with the
 *   library lock held ([EX] as Harmony's DRV_PIC32MZW_CryptoCallbackPush()).
 *
 ****************************************************************************/

void pic32mz_wlan_crypto_defer(wlan_crypto_cb_t cb, int result,
                               uintptr_t context)
{
  irqstate_t flags;
  uint8_t next;

  flags = enter_critical_section();
  next = (g_wlan.defer_head + 1) % WLAN_CRYPTO_DEFER_MAX;
  if (next == g_wlan.defer_tail)
    {
      leave_critical_section(flags);
      wlerr("crypto callback queue full\n");
      return;
    }

  g_wlan.defer[g_wlan.defer_head].cb      = cb;
  g_wlan.defer[g_wlan.defer_head].result  = result;
  g_wlan.defer[g_wlan.defer_head].context = context;
  g_wlan.defer_head = next;
  leave_critical_section(flags);

  nxsem_post(&g_wlan.evsem);
}

/****************************************************************************
 * Name: pic32mz_wlan_initialize
 *
 * Description:
 *   Power the WLAN block, start Microchip's WLAN library and its thread,
 *   wait for the RF calibration and the MAC address, then register the
 *   wlan0 network device.
 *
 ****************************************************************************/

int pic32mz_wlan_initialize(void)
{
  struct pic32mz_wlan_init_s init;
  FAR struct netdev_lowerhalf_s *dev = &g_wlan.dev;
  int ret;

  memset(&g_wlan, 0, sizeof(g_wlan));
  nxsem_init(&g_wlan.evsem, 0, 0);
  nxsem_init(&g_wlan.readysem, 0, 0);
  nxmutex_init(&g_wlan.lock);
  spin_lock_init(&g_wlan.rxlock);
  IOB_QINIT(&g_wlan.rxqueue);
  sq_init(&g_wlan.txq);
  sq_init(&g_wlan.rxq);
  wlan_pool_init();

  g_wlan.wpaver = IW_AUTH_WPA_VERSION_DISABLED;

#ifdef CONFIG_PIC32MZ_W1_BA414E
  ret = pic32mz_ba414e_initialize();
  if (ret < 0)
    {
      return ret;
    }
#endif

  /* [EX] Turn the WLAN LDO on */

  modifyreg32(PIC32MZ_PMUCLKCTRL, PMUCLKCTRL_WLDOOFF, 0);

  irq_attach(PIC32MZ_IRQ_RFMAC, wlan_isr, NULL);
  irq_attach(PIC32MZ_IRQ_RFTM0, wlan_isr, NULL);
  irq_attach(PIC32MZ_IRQ_RFSMC, wlan_isr, NULL);
  up_enable_irq(PIC32MZ_IRQ_RFMAC);
  up_enable_irq(PIC32MZ_IRQ_RFTM0);
  up_enable_irq(PIC32MZ_IRQ_RFSMC);

  /* [EX] Harmony's default alarm periods are 0 */

  init.alarm_1ms = 0;
  init.alarm_max = 0;
  if (!wdrv_pic32mzw_init(&init))
    {
      wlerr("wdrv_pic32mzw_init failed\n");
      return -EIO;
    }

  ret = kthread_create("wlan", CONFIG_PIC32MZ_W1_WLAN_PRIORITY,
                       CONFIG_PIC32MZ_W1_WLAN_STACKSIZE, wlan_thread, NULL);
  if (ret < 0)
    {
      return ret;
    }

  /* The RF calibration is loaded from OTP when the library starts */

  ret = nxsem_tickwait_uninterruptible(&g_wlan.readysem, SEC2TICK(5));
  if (ret < 0)
    {
      wlerr("RF not ready (status 0x%02x)\n", g_wlan.rfmac_status);
      return ret;
    }

  memcpy(dev->netdev.d_mac.ether.ether_addr_octet, g_wlan.mac, 6);
  dev->ops                = &g_wlan_ops;
  dev->iw_ops             = &g_wlan_iw_ops;
  dev->quota[NETPKT_RX]   = WLAN_RX_QUOTA;
  dev->quota[NETPKT_TX]   = WLAN_TX_QUOTA;
  dev->rxtype             = NETDEV_RX_THREAD;
  dev->priority           = CONFIG_PIC32MZ_W1_WLAN_PRIORITY;

  ret = netdev_lower_register(dev, NET_LL_IEEE80211);
  if (ret < 0)
    {
      return ret;
    }

  g_wlan.registered = true;
  syslog(LOG_INFO, "wlan: %s %02x:%02x:%02x:%02x:%02x:%02x\n",
         dev->netdev.d_ifname, g_wlan.mac[0], g_wlan.mac[1], g_wlan.mac[2],
         g_wlan.mac[3], g_wlan.mac[4], g_wlan.mac[5]);
  return OK;
}
