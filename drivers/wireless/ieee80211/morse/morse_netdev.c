/****************************************************************************
 * drivers/wireless/ieee80211/morse/morse_netdev.c
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

/* Network interface for the Morse Micro MM6108/MM8108 Wi-Fi HaLow
 * (IEEE 802.11ah) transceivers.
 *
 * The 802.11ah UMAC, the supplicant (WPA3-SAE/OWE) and the chip transport
 * are provided by the Morse Micro host library (libmorse).  This file binds
 * the libmorse WLAN API to the NuttX lower half network driver interface,
 * including the wireless extensions used by tools like wapi.
 *
 * The libmorse datapath works with 802.3 frames, so the interface is
 * registered as an Ethernet-like IEEE 802.11 link.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <sys/param.h>

#include <net/if.h>
#include <net/if_arp.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/spinlock.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/netdev_lowerhalf.h>
#include <nuttx/wireless/wireless.h>
#include <nuttx/wireless/morse.h>

#include "mmwlan.h"
#include "mmpkt.h"
#include "mmregdb.h"
#include "morse.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SCHED_LPWORK
#  define LPWORK HPWORK
#endif

#define MORSE_IW_EVENT_SIZE(field) \
  (offsetof(struct iw_event, u) + sizeof(((union iwreq_data *)0)->field))

/* 802.11 capability information: privacy bit */

#define MORSE_CAPAB_PRIVACY   0x0010

/* Frequencies are reported as m * 10^e Hz.  An exponent of zero means a
 * channel number in the wireless extensions, so use kHz units (all the S1G
 * channel centres are multiple of 500 kHz).
 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum morse_scan_state_e
{
  MORSE_SCAN_IDLE = 0,
  MORSE_SCAN_RUNNING,
  MORSE_SCAN_DONE,
};

struct morse_dev_s
{
  struct netdev_lowerhalf_s dev;    /* Must be first */

  mutex_t lock;                     /* Serializes the wireless operations */
  netpkt_queue_t rxq;               /* Received packets */
  spinlock_t rxlock;
  struct work_s linkwork;           /* Carrier on/off notification */
  sem_t linksem;                    /* Posted when the link goes up */
  volatile bool linkup;
  bool sta_enabled;

  /* Station configuration */

  uint8_t ssid[MMWLAN_SSID_MAXLEN];
  uint16_t ssid_len;
  uint8_t bssid[MMWLAN_MAC_ADDR_LEN];
  char passphrase[MMWLAN_PASSPHRASE_MAXLEN + 1];
  uint16_t passphrase_len;
  enum mmwlan_security_type security;
  char country[MMWLAN_COUNTRY_CODE_LEN];
  FAR const struct mmwlan_s1g_channel_list *channels;
  int txpower;                      /* Max TX power (dBm), 0 = default */

  /* Scan results, stored as a stream of struct iw_event */

  volatile uint8_t scan_state;
  FAR uint8_t *scan_buf;
  size_t scan_len;
  bool scan_full;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Network device operations */

static int morse_ifup(FAR struct netdev_lowerhalf_s *dev);
static int morse_ifdown(FAR struct netdev_lowerhalf_s *dev);
static int morse_transmit(FAR struct netdev_lowerhalf_s *dev,
                          FAR netpkt_t *pkt);
static FAR netpkt_t *morse_receive(FAR struct netdev_lowerhalf_s *dev);

/* Wireless operations */

static int morse_connect(FAR struct netdev_lowerhalf_s *dev);
static int morse_disconnect(FAR struct netdev_lowerhalf_s *dev);
static int morse_essid(FAR struct netdev_lowerhalf_s *dev,
                       FAR struct iwreq *iwr, bool set);
static int morse_bssid(FAR struct netdev_lowerhalf_s *dev,
                       FAR struct iwreq *iwr, bool set);
static int morse_passwd(FAR struct netdev_lowerhalf_s *dev,
                        FAR struct iwreq *iwr, bool set);
static int morse_mode(FAR struct netdev_lowerhalf_s *dev,
                      FAR struct iwreq *iwr, bool set);
static int morse_auth(FAR struct netdev_lowerhalf_s *dev,
                      FAR struct iwreq *iwr, bool set);
static int morse_freq(FAR struct netdev_lowerhalf_s *dev,
                      FAR struct iwreq *iwr, bool set);
static int morse_txpower(FAR struct netdev_lowerhalf_s *dev,
                         FAR struct iwreq *iwr, bool set);
static int morse_country(FAR struct netdev_lowerhalf_s *dev,
                         FAR struct iwreq *iwr, bool set);
static int morse_sensitivity(FAR struct netdev_lowerhalf_s *dev,
                             FAR struct iwreq *iwr, bool set);
static int morse_scan(FAR struct netdev_lowerhalf_s *dev,
                      FAR struct iwreq *iwr, bool set);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct netdev_ops_s g_morse_netdev_ops =
{
  .ifup     = morse_ifup,
  .ifdown   = morse_ifdown,
  .transmit = morse_transmit,
  .receive  = morse_receive,
};

static const struct wireless_ops_s g_morse_wireless_ops =
{
  .connect     = morse_connect,
  .disconnect  = morse_disconnect,
  .essid       = morse_essid,
  .bssid       = morse_bssid,
  .passwd      = morse_passwd,
  .mode        = morse_mode,
  .auth        = morse_auth,
  .freq        = morse_freq,
  .txpower     = morse_txpower,
  .country     = morse_country,
  .sensitivity = morse_sensitivity,
  .scan        = morse_scan,
};

/* libmorse supports a single WLAN interface; the callbacks without an
 * argument use this reference.
 */

static FAR struct morse_dev_s *g_morse_dev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int morse_status2errno(enum mmwlan_status status)
{
  switch (status)
    {
      case MMWLAN_SUCCESS:
        return OK;

      case MMWLAN_INVALID_ARGUMENT:
      case MMWLAN_CHANNEL_INVALID:
        return -EINVAL;

      case MMWLAN_UNAVAILABLE:
      case MMWLAN_SHUTDOWN_BLOCKED:
        return -EBUSY;

      case MMWLAN_NO_MEM:
        return -ENOMEM;

      case MMWLAN_TIMED_OUT:
        return -ETIMEDOUT;

      case MMWLAN_NOT_FOUND:
        return -ENOENT;

      case MMWLAN_CHANNEL_LIST_NOT_SET:
      case MMWLAN_NOT_RUNNING:
      case MMWLAN_NOT_INITIALIZED:
        return -ENETDOWN;

      default:
        return -EIO;
    }
}

/* Look up the centre frequency of an S1G channel in the active
 * regulatory domain.  Returns 0 if not found.
 */

static uint32_t morse_chan2freq(FAR struct morse_dev_s *priv,
                                unsigned int chan)
{
  unsigned int i;

  if (priv->channels == NULL)
    {
      return 0;
    }

  for (i = 0; i < priv->channels->num_channels; i++)
    {
      if (priv->channels->channels[i].s1g_chan_num == chan)
        {
          return priv->channels->channels[i].centre_freq_hz;
        }
    }

  return 0;
}

static int morse_set_country(FAR struct morse_dev_s *priv,
                             FAR const char *code, bool boot)
{
  FAR const struct mmwlan_s1g_channel_list *list;
  enum mmwlan_status status;

  list = mmwlan_lookup_regulatory_domain(get_regulatory_db(), code);
  if (list == NULL)
    {
      wlerr("ERROR: unsupported country code %.2s\n", code);
      return -EINVAL;
    }

  /* The channel list cannot be changed while the chip is booted: shut it
   * down (no effect if not booted), apply the new list and boot it again.
   */

  mmwlan_shutdown();
  status = mmwlan_set_channel_list(list);
  if (status == MMWLAN_SUCCESS && boot)
    {
      status = mmwlan_boot(NULL);
    }

  if (status != MMWLAN_SUCCESS)
    {
      return morse_status2errno(status);
    }

  priv->channels   = list;
  priv->country[0] = code[0];
  priv->country[1] = code[1];
  priv->country[2] = '\0';
  return OK;
}

/* Datapath callbacks *******************************************************/

static void morse_rx_callback(FAR struct mmpkt *mmpkt, FAR void *arg)
{
  FAR struct morse_dev_s *priv = arg;
  FAR struct mmpktview *view;
  FAR netpkt_t *pkt;
  irqstate_t flags;
  uint32_t len;
  int ret;

  view = mmpkt_open(mmpkt);
  len  = mmpkt_get_data_length(view);

  pkt = netpkt_alloc(&priv->dev, NETPKT_RX);
  if (pkt == NULL)
    {
      wlwarn("WARNING: no RX buffer, packet dropped\n");
      goto out;
    }

  ret = netpkt_copyin(&priv->dev, pkt, mmpkt_get_data_start(view), len, 0);
  if (ret < 0)
    {
      netpkt_free(&priv->dev, pkt, NETPKT_RX);
      goto out;
    }

  flags = spin_lock_irqsave(&priv->rxlock);
  ret = netpkt_tryadd_queue(pkt, &priv->rxq);
  spin_unlock_irqrestore(&priv->rxlock, flags);
  if (ret < 0)
    {
      netpkt_free(&priv->dev, pkt, NETPKT_RX);
      goto out;
    }

  netdev_lower_rxready(&priv->dev);

out:
  mmpkt_close(&view);
  mmpkt_release(mmpkt);
}

static void morse_link_worker(FAR void *arg)
{
  FAR struct morse_dev_s *priv = arg;

  if (priv->linkup)
    {
      netdev_lower_carrier_on(&priv->dev);
    }
  else
    {
      netdev_lower_carrier_off(&priv->dev);
    }
}

static void morse_link_callback(enum mmwlan_link_state link_state,
                                FAR void *arg)
{
  FAR struct morse_dev_s *priv = arg;

  /* Must not block here: defer the carrier change to the work queue */

  priv->linkup = (link_state == MMWLAN_LINK_UP);
  wlinfo("link %s\n", priv->linkup ? "up" : "down");

  work_queue(LPWORK, &priv->linkwork, morse_link_worker, priv, 0);

  if (priv->linkup)
    {
      nxsem_post(&priv->linksem);
    }
}

static void morse_sta_status_callback(enum mmwlan_sta_state sta_state)
{
  wlinfo("STA state %d\n", sta_state);
  UNUSED(sta_state);
}

/* Scan callbacks ***********************************************************/

static FAR struct iw_event *morse_scan_event(FAR struct morse_dev_s *priv,
                                             size_t len)
{
  FAR struct iw_event *iwe;

  if (priv->scan_len + len > CONFIG_IEEE80211_MORSE_SCAN_BUFSIZE)
    {
      priv->scan_full = true;
      return NULL;
    }

  iwe = (FAR struct iw_event *)&priv->scan_buf[priv->scan_len];
  iwe->len = len;
  priv->scan_len += len;
  return iwe;
}

static void morse_scan_rx_callback(FAR const struct mmwlan_scan_result *res,
                                   FAR void *arg)
{
  FAR struct morse_dev_s *priv = arg;
  FAR struct iw_event *iwe;
  size_t ssid_len;
  size_t needed;

  /* Only add complete records */

  ssid_len = MIN(res->ssid_len, IW_ESSID_MAX_SIZE);
  needed   = MORSE_IW_EVENT_SIZE(ap_addr) +
             MORSE_IW_EVENT_SIZE(essid) + ((ssid_len + 3) & ~3) +
             MORSE_IW_EVENT_SIZE(qual) + MORSE_IW_EVENT_SIZE(mode) +
             MORSE_IW_EVENT_SIZE(data) + MORSE_IW_EVENT_SIZE(freq);

  if (priv->scan_buf == NULL ||
      priv->scan_len + needed > CONFIG_IEEE80211_MORSE_SCAN_BUFSIZE)
    {
      priv->scan_full = true;
      return;
    }

  iwe = morse_scan_event(priv, MORSE_IW_EVENT_SIZE(ap_addr));
  iwe->cmd = SIOCGIWAP;
  iwe->u.ap_addr.sa_family = ARPHRD_ETHER;
  memcpy(iwe->u.ap_addr.sa_data, res->bssid, MMWLAN_MAC_ADDR_LEN);

  iwe = morse_scan_event(priv, MORSE_IW_EVENT_SIZE(essid) +
                               ((ssid_len + 3) & ~3));
  iwe->cmd = SIOCGIWESSID;
  iwe->u.essid.flags  = 0;
  iwe->u.essid.length = ssid_len;

  /* The iw_point pointer field holds the offset to the data */

  iwe->u.essid.pointer = (FAR void *)sizeof(iwe->u.essid);
  memcpy(&iwe->u.essid + 1, res->ssid, ssid_len);

  iwe = morse_scan_event(priv, MORSE_IW_EVENT_SIZE(qual));
  iwe->cmd = IWEVQUAL;
  iwe->u.qual.qual    = 0;
  iwe->u.qual.level   = (uint8_t)res->rssi;
  iwe->u.qual.noise   = (uint8_t)res->noise_dbm;
  iwe->u.qual.updated = IW_QUAL_DBM | IW_QUAL_ALL_UPDATED;

  iwe = morse_scan_event(priv, MORSE_IW_EVENT_SIZE(mode));
  iwe->cmd    = SIOCGIWMODE;
  iwe->u.mode = IW_MODE_MASTER;

  iwe = morse_scan_event(priv, MORSE_IW_EVENT_SIZE(data));
  iwe->cmd = SIOCGIWENCODE;
  iwe->u.data.flags   = (res->capability_info & MORSE_CAPAB_PRIVACY) ?
                        IW_ENCODE_ENABLED | IW_ENCODE_NOKEY :
                        IW_ENCODE_DISABLED;
  iwe->u.data.length  = 0;
  iwe->u.data.pointer = NULL;

  iwe = morse_scan_event(priv, MORSE_IW_EVENT_SIZE(freq));
  iwe->cmd      = SIOCGIWFREQ;
  iwe->u.freq.m = res->channel_freq_hz / 1000;
  iwe->u.freq.e = 3;
  iwe->u.freq.i = 0;
  iwe->u.freq.flags = IW_FREQ_FIXED;

  wlinfo("scan: %.*s rssi %d freq %" PRIu32 "\n", (int)ssid_len,
         res->ssid, res->rssi, res->channel_freq_hz);
}

static void morse_scan_complete_callback(enum mmwlan_scan_state state,
                                         FAR void *arg)
{
  FAR struct morse_dev_s *priv = arg;

  wlinfo("scan complete (%d), %zu bytes%s\n", state, priv->scan_len,
         priv->scan_full ? ", results truncated" : "");
  priv->scan_state = MORSE_SCAN_DONE;
}

/* Network device operations ************************************************/

static int morse_ifup(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct morse_dev_s *priv = (FAR struct morse_dev_s *)dev;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->rxlock);
  netpkt_free_queue(&priv->rxq);
  spin_unlock_irqrestore(&priv->rxlock, flags);

  /* The carrier follows the WLAN link state */

  if (priv->linkup)
    {
      netdev_lower_carrier_on(dev);
    }
  else
    {
      netdev_lower_carrier_off(dev);
    }

  return OK;
}

static int morse_ifdown(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct morse_dev_s *priv = (FAR struct morse_dev_s *)dev;
  irqstate_t flags;

  morse_disconnect(dev);

  flags = spin_lock_irqsave(&priv->rxlock);
  netpkt_free_queue(&priv->rxq);
  spin_unlock_irqrestore(&priv->rxlock, flags);

  return OK;
}

static int morse_transmit(FAR struct netdev_lowerhalf_s *dev,
                          FAR netpkt_t *pkt)
{
  unsigned int len = netpkt_getdatalen(dev, pkt);
  struct mmwlan_tx_metadata metadata = MMWLAN_TX_METADATA_INIT;
  FAR struct mmpktview *view;
  FAR struct mmpkt *mmpkt;
  enum mmwlan_status status;
  int ret;

  /* The upper half drops the packet on error, so give the chip a short
   * time to drain its queues when TX is flow controlled.
   */

  status = mmwlan_tx_wait_until_ready(CONFIG_IEEE80211_MORSE_TX_TIMEOUT);
  if (status != MMWLAN_SUCCESS)
    {
      return morse_status2errno(status);
    }

  mmpkt = mmwlan_alloc_mmpkt_for_tx(len, metadata.tid);
  if (mmpkt == NULL)
    {
      return -ENOMEM;
    }

  view = mmpkt_open(mmpkt);
  ret  = netpkt_copyout(dev, mmpkt_append(view, len), pkt, len, 0);
  mmpkt_close(&view);
  if (ret < 0)
    {
      mmpkt_release(mmpkt);
      return ret;
    }

  /* The mmpkt is consumed by mmwlan_tx_pkt() in all cases */

  status = mmwlan_tx_pkt(mmpkt, &metadata);
  if (status != MMWLAN_SUCCESS)
    {
      return morse_status2errno(status);
    }

  netpkt_free(dev, pkt, NETPKT_TX);
  netdev_lower_txdone(dev);
  return OK;
}

static FAR netpkt_t *morse_receive(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct morse_dev_s *priv = (FAR struct morse_dev_s *)dev;
  FAR netpkt_t *pkt;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->rxlock);
  pkt = netpkt_remove_queue(&priv->rxq);
  spin_unlock_irqrestore(&priv->rxlock, flags);

  return pkt;
}

/* Wireless operations ******************************************************/

static int morse_connect(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct morse_dev_s *priv = (FAR struct morse_dev_s *)dev;
  struct mmwlan_sta_args args = MMWLAN_STA_ARGS_INIT;
  enum mmwlan_status status;
  int ret;

  nxmutex_lock(&priv->lock);

  if (priv->ssid_len == 0)
    {
      ret = -EINVAL;
      goto out;
    }

  /* Reconfigure: drop the current association first */

  if (priv->sta_enabled)
    {
      mmwlan_sta_disable();
      priv->sta_enabled = false;
    }

  memcpy(args.ssid, priv->ssid, priv->ssid_len);
  args.ssid_len = priv->ssid_len;
  memcpy(args.bssid, priv->bssid, sizeof(args.bssid));
  args.security_type = priv->security;
  if (priv->security == MMWLAN_SAE)
    {
      memcpy(args.passphrase, priv->passphrase, priv->passphrase_len);
      args.passphrase_len = priv->passphrase_len;
    }

  if (priv->security == MMWLAN_OPEN)
    {
      args.pmf_mode = MMWLAN_PMF_DISABLED;
    }

  while (nxsem_trywait(&priv->linksem) >= 0);

  /* Without the WAKE and BUSY lines the host cannot wake the chip from
   * power save: keep it always awake.
   */

  if (!morse_hal_has_powersave())
    {
      mmwlan_set_power_save_mode(MMWLAN_PS_DISABLED);
    }

  wlinfo("connecting to %.*s (security %d)\n", priv->ssid_len, priv->ssid,
         priv->security);

  status = mmwlan_sta_enable(&args, morse_sta_status_callback);
  if (status != MMWLAN_SUCCESS)
    {
      wlerr("ERROR: mmwlan_sta_enable failed: %d\n", status);
      ret = morse_status2errno(status);
      goto out;
    }

  priv->sta_enabled = true;

  /* libmorse keeps on trying in the background, so on timeout the link can
   * still come up later.
   */

  ret = nxsem_tickwait_uninterruptible(&priv->linksem,
                       SEC2TICK(CONFIG_IEEE80211_MORSE_CONNECT_TIMEOUT));
  if (ret < 0)
    {
      wlwarn("WARNING: link not up after %d s\n",
             CONFIG_IEEE80211_MORSE_CONNECT_TIMEOUT);
      ret = -ETIMEDOUT;
    }

out:
  nxmutex_unlock(&priv->lock);
  return ret;
}

static int morse_disconnect(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct morse_dev_s *priv = (FAR struct morse_dev_s *)dev;
  enum mmwlan_status status = MMWLAN_SUCCESS;

  nxmutex_lock(&priv->lock);

  if (priv->sta_enabled)
    {
      status = mmwlan_sta_disable();
      priv->sta_enabled = false;
      if (status == MMWLAN_SHUTDOWN_BLOCKED)
        {
          status = MMWLAN_SUCCESS;
        }
    }

  nxmutex_unlock(&priv->lock);
  return morse_status2errno(status);
}

static int morse_essid(FAR struct netdev_lowerhalf_s *dev,
                       FAR struct iwreq *iwr, bool set)
{
  FAR struct morse_dev_s *priv = (FAR struct morse_dev_s *)dev;
  int ret = OK;

  nxmutex_lock(&priv->lock);

  if (set)
    {
      if (iwr->u.essid.length > MMWLAN_SSID_MAXLEN ||
          iwr->u.essid.pointer == NULL)
        {
          ret = -EINVAL;
        }
      else
        {
          memcpy(priv->ssid, iwr->u.essid.pointer, iwr->u.essid.length);
          priv->ssid_len = iwr->u.essid.length;
        }
    }
  else
    {
      if (iwr->u.essid.length < priv->ssid_len)
        {
          ret = -E2BIG;
        }
      else
        {
          memcpy(iwr->u.essid.pointer, priv->ssid, priv->ssid_len);
          iwr->u.essid.length = priv->ssid_len;
          iwr->u.essid.flags  = priv->linkup ? IW_ESSID_ON : IW_ESSID_OFF;
        }
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

static int morse_bssid(FAR struct netdev_lowerhalf_s *dev,
                       FAR struct iwreq *iwr, bool set)
{
  FAR struct morse_dev_s *priv = (FAR struct morse_dev_s *)dev;

  if (set)
    {
      memcpy(priv->bssid, iwr->u.ap_addr.sa_data, MMWLAN_MAC_ADDR_LEN);
      return OK;
    }

  iwr->u.ap_addr.sa_family = ARPHRD_ETHER;
  if (!priv->linkup ||
      mmwlan_get_bssid((FAR uint8_t *)iwr->u.ap_addr.sa_data) !=
      MMWLAN_SUCCESS)
    {
      memset(iwr->u.ap_addr.sa_data, 0, MMWLAN_MAC_ADDR_LEN);
    }

  return OK;
}

static int morse_passwd(FAR struct netdev_lowerhalf_s *dev,
                        FAR struct iwreq *iwr, bool set)
{
  FAR struct morse_dev_s *priv = (FAR struct morse_dev_s *)dev;
  FAR struct iw_encode_ext *ext = iwr->u.encoding.pointer;
  int ret = OK;

  if (ext == NULL)
    {
      return -EINVAL;
    }

  nxmutex_lock(&priv->lock);

  if (set)
    {
      /* Wi-Fi HaLow only allows open, OWE and WPA3-SAE networks: a
       * passphrase selects SAE.
       */

      if (ext->alg == IW_ENCODE_ALG_NONE || ext->key_len == 0)
        {
          priv->passphrase_len = 0;
          if (priv->security == MMWLAN_SAE)
            {
              priv->security = MMWLAN_OPEN;
            }
        }
      else if (ext->key_len > MMWLAN_PASSPHRASE_MAXLEN)
        {
          ret = -EINVAL;
        }
      else
        {
          memcpy(priv->passphrase, ext->key, ext->key_len);
          priv->passphrase[ext->key_len] = '\0';
          priv->passphrase_len = ext->key_len;
          priv->security = MMWLAN_SAE;
        }
    }
  else
    {
      if (iwr->u.encoding.length < sizeof(*ext) + priv->passphrase_len)
        {
          ret = -E2BIG;
        }
      else
        {
          ext->alg     = priv->security == MMWLAN_SAE ?
                         IW_ENCODE_ALG_CCMP : IW_ENCODE_ALG_NONE;
          ext->key_len = priv->passphrase_len;
          memcpy(ext->key, priv->passphrase, priv->passphrase_len);
        }
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

static int morse_mode(FAR struct netdev_lowerhalf_s *dev,
                      FAR struct iwreq *iwr, bool set)
{
  if (set)
    {
      return iwr->u.mode == IW_MODE_INFRA ? OK : -ENOSYS;
    }

  iwr->u.mode = IW_MODE_INFRA;
  return OK;
}

static int morse_auth(FAR struct netdev_lowerhalf_s *dev,
                      FAR struct iwreq *iwr, bool set)
{
  FAR struct morse_dev_s *priv = (FAR struct morse_dev_s *)dev;
  int flag = iwr->u.param.flags & IW_AUTH_INDEX;
  int ret = OK;

  nxmutex_lock(&priv->lock);

  if (set)
    {
      if (flag == IW_AUTH_WPA_VERSION)
        {
          if (iwr->u.param.value & IW_AUTH_WPA_VERSION_DISABLED)
            {
              priv->security = MMWLAN_OPEN;
            }
          else if (iwr->u.param.value & IW_AUTH_WPA_VERSION_WPA3)
            {
              priv->security = priv->passphrase_len > 0 ? MMWLAN_SAE :
                                                          MMWLAN_OWE;
            }
          else
            {
              /* WPA/WPA2 are not allowed by the 802.11ah standard: use
               * WPA3-SAE instead (requires a passphrase).
               */

              wlwarn("WARNING: HaLow requires WPA3, using SAE\n");
              priv->security = MMWLAN_SAE;
            }
        }
    }
  else
    {
      if (flag == IW_AUTH_WPA_VERSION)
        {
          iwr->u.param.value = priv->security == MMWLAN_OPEN ?
                               IW_AUTH_WPA_VERSION_DISABLED :
                               IW_AUTH_WPA_VERSION_WPA3;
        }
      else if (flag == IW_AUTH_CIPHER_PAIRWISE ||
               flag == IW_AUTH_CIPHER_GROUP)
        {
          iwr->u.param.value = priv->security == MMWLAN_OPEN ?
                               IW_AUTH_CIPHER_NONE : IW_AUTH_CIPHER_CCMP;
        }
      else
        {
          ret = -ENOSYS;
        }
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

static int morse_freq(FAR struct netdev_lowerhalf_s *dev,
                      FAR struct iwreq *iwr, bool set)
{
  FAR struct morse_dev_s *priv = (FAR struct morse_dev_s *)dev;
  struct mmwlan_vif_channel_info info;

  /* The channel is selected by the access point */

  if (set)
    {
      return -ENOSYS;
    }

  if (!priv->linkup ||
      mmwlan_get_vif_channel_info(MMWLAN_VIF_STA, &info) != MMWLAN_SUCCESS)
    {
      return -ENOTCONN;
    }

  iwr->u.freq.m     = morse_chan2freq(priv, info.s1g_chan_num) / 1000;
  iwr->u.freq.e     = 3;
  iwr->u.freq.i     = 0;
  iwr->u.freq.flags = IW_FREQ_FIXED;
  return OK;
}

static int morse_txpower(FAR struct netdev_lowerhalf_s *dev,
                         FAR struct iwreq *iwr, bool set)
{
  FAR struct morse_dev_s *priv = (FAR struct morse_dev_s *)dev;
  enum mmwlan_status status;

  if (set)
    {
      if (iwr->u.txpower.flags != IW_TXPOW_DBM ||
          iwr->u.txpower.value < 0)
        {
          return -EINVAL;
        }

      status = mmwlan_override_max_tx_power(iwr->u.txpower.value);
      if (status == MMWLAN_SUCCESS)
        {
          priv->txpower = iwr->u.txpower.value;
        }

      return morse_status2errno(status);
    }

  iwr->u.txpower.value    = priv->txpower;
  iwr->u.txpower.fixed    = priv->txpower != 0;
  iwr->u.txpower.disabled = 0;
  iwr->u.txpower.flags    = IW_TXPOW_DBM;
  return OK;
}

static int morse_country(FAR struct netdev_lowerhalf_s *dev,
                         FAR struct iwreq *iwr, bool set)
{
  FAR struct morse_dev_s *priv = (FAR struct morse_dev_s *)dev;
  FAR char *code = iwr->u.data.pointer;
  int ret = OK;

  if (code == NULL)
    {
      return -EINVAL;
    }

  nxmutex_lock(&priv->lock);

  if (set)
    {
      /* The channel list can only be changed while inactive */

      if (priv->sta_enabled)
        {
          ret = -EBUSY;
        }
      else
        {
          ret = morse_set_country(priv, code, true);
        }
    }
  else if (iwr->u.data.length < 3)
    {
      ret = -E2BIG;
    }
  else
    {
      strlcpy(code, priv->country, iwr->u.data.length);
      iwr->u.data.length = strlen(priv->country);
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

static int morse_sensitivity(FAR struct netdev_lowerhalf_s *dev,
                             FAR struct iwreq *iwr, bool set)
{
  FAR struct morse_dev_s *priv = (FAR struct morse_dev_s *)dev;

  /* Report the RSSI of the connection, like the other NuttX drivers */

  if (set)
    {
      return -ENOSYS;
    }

  if (!priv->linkup)
    {
      return -ENOTCONN;
    }

  iwr->u.sens.value = -mmwlan_get_rssi();
  return OK;
}

static int morse_scan(FAR struct netdev_lowerhalf_s *dev,
                      FAR struct iwreq *iwr, bool set)
{
  FAR struct morse_dev_s *priv = (FAR struct morse_dev_s *)dev;
  struct mmwlan_scan_req req = MMWLAN_SCAN_REQ_INIT;
  FAR struct iw_scan_req *scanreq;
  enum mmwlan_status status;
  int ret = OK;

  nxmutex_lock(&priv->lock);

  if (set)
    {
      if (priv->scan_state == MORSE_SCAN_RUNNING)
        {
          ret = -EBUSY;
          goto out;
        }

      if (priv->scan_buf == NULL)
        {
          priv->scan_buf = kmm_malloc(CONFIG_IEEE80211_MORSE_SCAN_BUFSIZE);
          if (priv->scan_buf == NULL)
            {
              ret = -ENOMEM;
              goto out;
            }
        }

      priv->scan_len  = 0;
      priv->scan_full = false;

      /* Optional directed scan for a given ESSID */

      scanreq = (FAR struct iw_scan_req *)iwr->u.data.pointer;
      if (scanreq != NULL && iwr->u.data.length >= sizeof(*scanreq) &&
          (iwr->u.data.flags & IW_SCAN_THIS_ESSID) != 0 &&
          scanreq->essid_len <= MMWLAN_SSID_MAXLEN)
        {
          memcpy(req.args.ssid, scanreq->essid, scanreq->essid_len);
          req.args.ssid_len = scanreq->essid_len;
        }

      req.scan_rx_cb       = morse_scan_rx_callback;
      req.scan_complete_cb = morse_scan_complete_callback;
      req.scan_cb_arg      = priv;

      priv->scan_state = MORSE_SCAN_RUNNING;
      status = mmwlan_scan_request(&req);
      if (status != MMWLAN_SUCCESS)
        {
          wlerr("ERROR: mmwlan_scan_request failed: %d\n", status);
          priv->scan_state = MORSE_SCAN_IDLE;
          ret = morse_status2errno(status);
        }

      goto out;
    }

  /* Get the scan results */

  if (priv->scan_state == MORSE_SCAN_RUNNING)
    {
      ret = -EAGAIN;
    }
  else if (priv->scan_state == MORSE_SCAN_IDLE)
    {
      ret = -EINVAL;
    }
  else if (iwr->u.data.pointer == NULL ||
           iwr->u.data.length < priv->scan_len)
    {
      /* Tell the caller the size of the result buffer needed */

      iwr->u.data.length = priv->scan_len;
      ret = -E2BIG;
    }
  else
    {
      memcpy(iwr->u.data.pointer, priv->scan_buf, priv->scan_len);
      iwr->u.data.length = priv->scan_len;

      /* The results are consumed: further reads return an empty list
       * until the next scan.
       */

      kmm_free(priv->scan_buf);
      priv->scan_buf = NULL;
      priv->scan_len = 0;
    }

out:
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: morse_register
 *
 * Description:
 *   Initialize the Morse Micro HaLow driver and register the network
 *   interface.  See include/nuttx/wireless/morse.h.
 *
 ****************************************************************************/

int morse_register(FAR struct spi_dev_s *spi,
                   FAR const struct morse_lower_s *lower)
{
  FAR struct morse_dev_s *priv;
  struct mmwlan_version version;
  enum mmwlan_status status;
  uint8_t mac[MMWLAN_MAC_ADDR_LEN];
  int ret;

  DEBUGASSERT(spi != NULL && lower != NULL);

  if (g_morse_dev != NULL)
    {
      return -EBUSY;
    }

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  nxmutex_init(&priv->lock);
  nxsem_init(&priv->linksem, 0, 0);
  spin_lock_init(&priv->rxlock);
  priv->security = MMWLAN_OPEN;

  ret = morse_hal_initialize(spi, lower);
  if (ret < 0)
    {
      goto errout;
    }

  mmwlan_init();

  ret = morse_set_country(priv, CONFIG_IEEE80211_MORSE_COUNTRY, false);
  if (ret < 0)
    {
      goto errout_deinit;
    }

  mmwlan_register_rx_pkt_cb(morse_rx_callback, priv);
  mmwlan_register_link_state_cb(morse_link_callback, priv);

  /* Boot the chip to read its version and MAC address */

  status = mmwlan_boot(NULL);
  if (status != MMWLAN_SUCCESS)
    {
      wlerr("ERROR: failed to boot the HaLow chip: %d\n", status);
      ret = morse_status2errno(status);
      goto errout_deinit;
    }

  if (mmwlan_get_version(&version) == MMWLAN_SUCCESS)
    {
      syslog(LOG_INFO, "morse: chip 0x%04" PRIx32 " firmware %s "
             "morselib %s\n", version.morse_chip_id,
             version.morse_fw_version, version.morselib_version);
    }

  status = mmwlan_get_vif_mac_addr(MMWLAN_VIF_STA, mac);
  if (status != MMWLAN_SUCCESS)
    {
      ret = morse_status2errno(status);
      goto errout_shutdown;
    }

  memcpy(priv->dev.netdev.d_mac.ether.ether_addr_octet, mac, sizeof(mac));

  priv->dev.ops              = &g_morse_netdev_ops;
  priv->dev.iw_ops           = &g_morse_wireless_ops;
  priv->dev.quota[NETPKT_TX] = CONFIG_IEEE80211_MORSE_TX_QUOTA;
  priv->dev.quota[NETPKT_RX] = CONFIG_IEEE80211_MORSE_RX_QUOTA;
  priv->dev.rxtype           = NETDEV_RX_THREAD;
  priv->dev.priority         = CONFIG_IEEE80211_MORSE_TASK_PRIORITY;

  g_morse_dev = priv;

  ret = netdev_lower_register(&priv->dev, NET_LL_IEEE80211);
  if (ret < 0)
    {
      g_morse_dev = NULL;
      goto errout_shutdown;
    }

  return OK;

errout_shutdown:
  mmwlan_shutdown();
errout_deinit:
  mmwlan_deinit();
errout:
  nxsem_destroy(&priv->linksem);
  nxmutex_destroy(&priv->lock);
  kmm_free(priv);
  return ret;
}
