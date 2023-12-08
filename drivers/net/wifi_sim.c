/****************************************************************************
 * drivers/net/wifi_sim.c
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

#include <debug.h>
#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <netinet/if_ether.h>

#include <nuttx/compiler.h>
#include <nuttx/kmalloc.h>
#include <nuttx/net/ip.h>
#include <nuttx/wireless/ieee80211/ieee80211.h>
#include <nuttx/virtio/virtio.h>
#include <netpacket/netlink.h>
#include <nuttx/net/netlink.h>

#include <nuttx/net/wifi_sim.h>
#include <nuttx/fs/fs.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BSS_FILE_NAME "bss"
#define BSS_FILE_PATH CONFIG_WIFI_SIM_CONFDIR "/" BSS_FILE_NAME

#define SSID_MAX_LEN 32

#define IEEE80211_CAP_PRIVACY 0x0010

/* Maximum number of supported rates (from both Supported Rates and Extended
 * Supported Rates IEs).
 */

#define WLAN_SUPP_RATES_MAX 32

#define WLAN_DEFAULT_TXPOWER 20

#define LOWERDEV2WIFIDEV(dev) \
  ((FAR struct wifi_sim_s *)((FAR struct wifi_sim_lowerhalf_s *)dev)->wifi)

#define CHAN2G(freq)              \
  {                               \
    .band = WLAN_80211_BAND_2GHZ, \
    .center_freq = (freq),        \
  }

#define CHAN5G(freq)              \
  {                               \
    .band = WLAN_80211_BAND_5GHZ, \
    .center_freq = (freq),        \
  }

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum WLAN_ALG_E
{
  WLAN_ALG_NONE,
  WLAN_ALG_WEP,
  WLAN_ALG_TKIP,
  WLAN_ALG_CCMP,
  WLAN_ALG_BIP_CMAC_128,
  WLAN_ALG_GCMP,
  WLAN_ALG_SMS4,
  WLAN_ALG_KRK,
  WLAN_ALG_GCMP_256,
  WLAN_ALG_CCMP_256,
  WLAN_ALG_BIP_GMAC_128,
  WLAN_ALG_BIP_GMAC_256,
  WLAN_ALG_BIP_CMAC_256
};

enum WLAN_HW_MODE_E
{
  WLAN_HW_MODE_IEEE80211B,
  WLAN_HW_MODE_IEEE80211G,
  WLAN_HW_MODE_IEEE80211A,
  WLAN_HW_MODE_IEEE80211AD,
  WLAN_HW_MODE_IEEE80211ANY,
  WLAN_HW_MODES
};

/* enum WLAN_CHAN_WIDTH_E - Channel width definitions */

enum WLAN_CHAN_WIDTH_E
{
  WLAN_CHAN_WIDTH_20_NOHT,
  WLAN_CHAN_WIDTH_20,
  WLAN_CHAN_WIDTH_40,
  WLAN_CHAN_WIDTH_80,
  WLAN_CHAN_WIDTH_80P80,
  WLAN_CHAN_WIDTH_160,
  WLAN_CHAN_WIDTH_2160,
  WLAN_CHAN_WIDTH_4320,
  WLAN_CHAN_WIDTH_6480,
  WLAN_CHAN_WIDTH_8640,
  WLAN_CHAN_WIDTH_320,
  WLAN_CHAN_WIDTH_UNKNOWN
};

enum WLAN_80211_BAND_E
{
  WLAN_80211_BAND_2GHZ,
  WLAN_80211_BAND_5GHZ,
  WLAN_80211_BAND_6GHZ,
  WLAN_80211_BAND_NUMS
};

enum WLAN_SCAN_STATE_E
{
  WLAN_SCAN_STATE_SCANNING,
  WLAN_SCAN_STATE_DONE,
};

enum WLAN_STA_STATE_E
{
  WLAN_STA_STATE_INIT,
  WLAN_STA_STATE_CONNECTING,
  WLAN_STA_STATE_CONNECTED,
};

enum WLAN_STA_CONNERR_E
{
  WLAN_STA_CONNERR_NO_BSS,
  WLAN_STA_CONNERR_NO_PSK,
  WLAN_STA_CONNERR_WRONG_KEY,
  WLAN_STA_CONNERR_FAILED,
};

/* for scan results */

struct iw_scan_result_s
{
  int      total_len;
  int       cur_len;
  FAR char *buf;                    /* iwr->u.data.pointer */
};

/* channel */

struct wlan_channel_s
{
  enum WLAN_80211_BAND_E band;
  uint32_t center_freq;
};

struct wifi_sim_bss_s
{
  char     bssid[ETH_ALEN];         /* bss BSSID */
  uint16_t capability;              /* Capability information */
  char     ssid[SSID_MAX_LEN + 1];  /* Capability information */
  uint8_t  ssid_len;                /* the length of ssid */
  int16_t  RSSI;                    /* receive signal strength (in dBm) */
  int8_t   phy_noise;               /* noise (in dBm) */
  int16_t  snr;                     /* average SNR of during frame reception */
  uint32_t freq;                    /* 802.11N BSS frequency */
  uint8_t  flags;                   /* flags */
  char     password[64];            /* password */
  char     security[128];           /* security type */
};

struct wifi_sim_sta_info_s
{
  uint16_t aid;
  char     mac_addr[ETH_ALEN];
  uint16_t capability;              /* Capability information */
  int      auth_alg;
  uint8_t  supported_rates[WLAN_SUPP_RATES_MAX];
  int      supported_rates_len;
  uint8_t  qosinfo;
  int16_t  RSSI;                    /* receive signal strength (in dBm) */
  uint32_t connected_time;          /* units: seconds */
  int16_t  deauth_reason;
  int16_t  disassoc_reason;
};

struct wifi_sim_s
{
  FAR struct netdev_lowerhalf_s *lower;

  char     ssid[SSID_MAX_LEN + 1];
  char     bssid[ETH_ALEN];
  uint8_t  mode;                        /* IW_MODE_INFRA/ IW_MODE_MASTER */
  enum WLAN_HW_MODE_E    hw_mode;       /* for hw mode */
  enum WLAN_CHAN_WIDTH_E band;
  uint16_t channel;
  uint32_t freq;
  char     password[64];
  int      key_mgmt;
  int      proto;
  int      auth_alg;
  int      pairwise_chiper;
  int      group_cipher;

  uint32_t bitrate;
  int32_t  txpower;
  char     country[4];
  int8_t   sensitivity;

  bool     psk_flag;                    /* for psk, 0: unset, 1: set */
  uint8_t  ssid_flag;                   /* for ssid, 0: unset, 1: set ssid
                                         *           2: set bssid  */
  enum WLAN_STA_STATE_E state;
  enum WLAN_SCAN_STATE_E scan_state;
  uint16_t status_code;                 /* status code */
  uint16_t reason_code;                 /* deauth and disassoc reason */
  uint16_t error_code;                  /* connect error reason */

  FAR struct wifi_sim_bss_s *connected_ap;  /* for sta mode */
};

/* for wireless event */

struct wireless_event_s
{
  struct nlmsghdr  hdr;                 /* netlink message header */
  struct ifinfomsg iface;               /* interface info */

  struct rtattr    attrevent;           /* IFLA_WIRELESS */
  struct iw_event  event;               /* wireless event */
};

struct wireless_event_list_s
{
  sq_entry_t flink;
  struct wireless_event_s payload;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int wifidriver_connect(FAR struct netdev_lowerhalf_s *dev);
static int wifidriver_disconnect(FAR struct netdev_lowerhalf_s *dev);
static int wifidriver_essid(FAR struct netdev_lowerhalf_s *dev,
                            FAR struct iwreq *iwr, bool set);
static int wifidriver_bssid(FAR struct netdev_lowerhalf_s *dev,
                            FAR struct iwreq *iwr, bool set);
static int wifidriver_passwd(FAR struct netdev_lowerhalf_s *dev,
                             FAR struct iwreq *iwr, bool set);
static int wifidriver_mode(FAR struct netdev_lowerhalf_s *dev,
                           FAR struct iwreq *iwr, bool set);
static int wifidriver_auth(FAR struct netdev_lowerhalf_s *dev,
                           FAR struct iwreq *iwr, bool set);
static int wifidriver_freq(FAR struct netdev_lowerhalf_s *dev,
                           FAR struct iwreq *iwr, bool set);
static int wifidriver_bitrate(FAR struct netdev_lowerhalf_s *dev,
                              FAR struct iwreq *iwr, bool set);
static int wifidriver_txpower(FAR struct netdev_lowerhalf_s *dev,
                              FAR struct iwreq *iwr, bool set);
static int wifidriver_country(FAR struct netdev_lowerhalf_s *dev,
                              FAR struct iwreq *iwr, bool set);
static int wifidriver_sensitivity(FAR struct netdev_lowerhalf_s *dev,
                                  FAR struct iwreq *iwr, bool set);
static int wifidriver_scan(FAR struct netdev_lowerhalf_s *dev,
                           FAR struct iwreq *iwr, bool set);
static int wifidriver_range(FAR struct netdev_lowerhalf_s *dev,
                            FAR struct iwreq *iwr);

static int wifidriver_get_bssinfo(FAR struct wifi_sim_bss_s *bss_info,
                                  FAR char *buf, int len);
static int wifidriver_start_disconnect(FAR struct wifi_sim_s *wifidev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct wlan_channel_s wifi_sim_channels_2ghz[] =
{
  CHAN2G(2412), /* Channel 1 */
  CHAN2G(2417), /* Channel 2 */
  CHAN2G(2422), /* Channel 3 */
  CHAN2G(2427), /* Channel 4 */
  CHAN2G(2432), /* Channel 5 */
  CHAN2G(2437), /* Channel 6 */
  CHAN2G(2442), /* Channel 7 */
  CHAN2G(2447), /* Channel 8 */
  CHAN2G(2452), /* Channel 9 */
  CHAN2G(2457), /* Channel 10 */
  CHAN2G(2462), /* Channel 11 */
  CHAN2G(2467), /* Channel 12 */
  CHAN2G(2472), /* Channel 13 */
  CHAN2G(2484), /* Channel 14 */
};

static const struct wlan_channel_s wifi_sim_channels_5ghz[] =
{
  CHAN5G(5180), /* Channel 36 */
  CHAN5G(5200), /* Channel 40 */
  CHAN5G(5220), /* Channel 44 */
  CHAN5G(5240), /* Channel 48 */
  CHAN5G(5260), /* Channel 52 */
  CHAN5G(5280), /* Channel 56 */
  CHAN5G(5300), /* Channel 60 */
  CHAN5G(5320), /* Channel 64 */
  CHAN5G(5745), /* Channel 149 */
  CHAN5G(5765), /* Channel 153 */
  CHAN5G(5785), /* Channel 157 */
  CHAN5G(5805), /* Channel 161 */
  CHAN5G(5825), /* Channel 165 */
};

static const struct wireless_ops_s g_iw_ops =
{
  wifidriver_connect,
  wifidriver_disconnect,
  wifidriver_essid,
  wifidriver_bssid,
  wifidriver_passwd,
  wifidriver_mode,
  wifidriver_auth,
  wifidriver_freq,
  wifidriver_bitrate,
  wifidriver_txpower,
  wifidriver_country,
  wifidriver_sensitivity,
  wifidriver_scan,
  wifidriver_range
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* utils */

static int mac_addr_a2n(FAR unsigned char *mac_addr, FAR char *arg)
{
  int i;

  for (i = 0; i < ETH_ALEN ; i++)
    {
      int temp;
      FAR char *cp = strchr(arg, ':');

      if (cp)
        {
          *cp = 0;
          cp++;
        }

      if (sscanf(arg, "%x", &temp) != 1)
        {
          return -EINVAL;
        }

      if (temp < 0 || temp > 255)
        {
          return -EINVAL;
        }

      mac_addr[i] = temp;
      if (!cp)
        {
          break;
        }

      arg = cp;
    }

  if (i < ETH_ALEN - 1)
    {
      return -EINVAL;
    }

  return OK;
}

static void mac_addr_n2a(FAR char *mac_addr, FAR unsigned char *arg)
{
  int i;
  int l;

  for (l = 0, i = 0; i < ETH_ALEN; i++)
    {
      if (i == 0)
        {
          sprintf(mac_addr + l, "%02x", arg[i]);
          l += 2;
        }
      else
        {
          sprintf(mac_addr + l, ":%02x", arg[i]);
          l += 3;
        }
    }
}

/* wifi interfaces */

static int wifi_send_event(FAR struct wifi_sim_s *wifidev,
                           unsigned int cmd, FAR union iwreq_data *wrqu)
{
  FAR struct net_driver_s *dev = &wifidev->lower->netdev;
  FAR struct wireless_event_list_s *alloc;
  FAR struct wireless_event_s *wev;

  DEBUGASSERT(dev != NULL);

  int up = IFF_IS_UP(dev->d_flags);

  /* Allocate the response buffer */

  alloc = (FAR struct wireless_event_list_s *)
          kmm_zalloc(RTA_SPACE(sizeof(struct wireless_event_list_s)));
  if (alloc == NULL)
    {
      nerr("ERROR: Failed to allocate wifi event buffer.\n");
      return -ENOMEM;
    }

  /* Initialize the response buffer */

  wev                   = &alloc->payload;

  wev->hdr.nlmsg_len    = sizeof(struct wireless_event_s);
  wev->hdr.nlmsg_type   = up ? RTM_NEWLINK : RTM_DELLINK;
  wev->hdr.nlmsg_flags  = 0;
  wev->hdr.nlmsg_seq    = 0;
  wev->hdr.nlmsg_pid    = 0;

  wev->iface.ifi_family = AF_UNSPEC;
  wev->iface.ifi_type   = ARPHRD_IEEE80211;
#ifdef CONFIG_NETDEV_IFINDEX
  wev->iface.ifi_index  = dev->d_ifindex;
#endif
  wev->iface.ifi_flags  = dev->d_flags;
  wev->iface.ifi_change = 0;

  /* add wireless event info */

  wev->attrevent.rta_len  = RTA_SPACE(sizeof(struct iw_event));
  wev->attrevent.rta_type = IFLA_WIRELESS;
  wev->event.len          = sizeof(union iwreq_data);
  wev->event.cmd          = cmd;

  memset(&wev->event.u, 0, sizeof(union iwreq_data));
  memcpy(&wev->event.u, ((FAR char *) wrqu), sizeof(union iwreq_data));

  netlink_add_broadcast(RTNLGRP_LINK,
                        (FAR struct netlink_response_s *)alloc);
  return OK;
}

static int read_bss_config(FAR char *buf, size_t len, FAR char *path)
{
  struct file filep;
  int ret;

  ret = file_open(&filep, path, O_RDWR);
  if (ret < 0)
    {
      nerr("open error");
      return ret;
    }

  ret = file_read(&filep, buf, len);
  if (ret == len)
    {
      ret = -E2BIG;
    }
  else if (ret > 0)
    {
      buf[ret] = '\0';
      ninfo("content: %s", buf);
    }

  file_close(&filep);

  return ret;
}

#if 0
uint32_t wifi_chan_to_freq_mhz(int chan, enum WLAN_80211_BAND_E band)
{
  /* see 802.11 17.3.8.3.2 and Annex J
   * there are overlapping channel numbers in 5GHz and 2GHz bands
   */

  if (chan <= 0)
    {
      return 0; /* not supported */
    }

  switch (band)
    {
      case WLAN_80211_BAND_2GHZ:
          if (chan == 14)
            {
              return (2484);
            }
          else if (chan < 14)
            {
              return (2407 + chan * 5);
            }
          break;
      case WLAN_80211_BAND_5GHZ:
          if (chan >= 182 && chan <= 196)
            {
              return (4000 + chan * 5);
            }
          else
            {
              return (5000 + chan * 5);
            }
          break;
      case WLAN_80211_BAND_6GHZ:

          /* see 802.11ax D6.1 27.3.23.2 */

          if (chan == 2)
            {
              return (5935);
            }

          if (chan <= 233)
            {
              return (5950 + chan * 5);
            }
          break;
      default:
          break;
    }

  return 0; /* not supported */
}
#endif

static int freq_to_channel(uint16_t freq)
{
  int channel = 0;

  /* If the freq is a valid channel, we think that
   * user wants to directly configure the channel.
   */

  if ((freq >= 1 && freq <= 14) ||
      (freq >= 36 && freq <= 165) ||
      (freq >= 182 && freq <= 196))
    {
      channel = freq;
      return channel;
    }

  if (freq >= 2412 && freq <= 2484)
    {
      if (freq == 2484)
        {
          channel = 14;
        }
      else
        {
          channel = freq - 2407;
          if (channel % 5)
            {
              return OK;
            }

          channel /= 5;
        }

      return channel;
    }

  if (freq >= 5005 && freq < 5900)
    {
      if (freq % 5)
        {
          return OK;
        }

      channel = (freq - 5000) / 5;
      return channel;
    }

  if (freq >= 4905 && freq < 5000)
    {
      if (freq % 5)
        {
          return OK;
        }

      channel = (freq - 4000) / 5;
      return channel;
    }

  return OK;
}

static FAR const char *get_cipherstr(int cipher)
{
  switch (cipher)
    {
      case IW_AUTH_CIPHER_NONE:
        return "NONE";

      case IW_AUTH_CIPHER_WEP40:
        return "WEP40";

      case IW_AUTH_CIPHER_WEP104:
        return "WEP104";

      case IW_AUTH_CIPHER_TKIP:
        return "TKIP";

      case IW_AUTH_CIPHER_CCMP:
        return "CCMP";

      case IW_AUTH_CIPHER_AES_CMAC:
        return "AES-128-CMAC";

      default:
        nerr("ERROR: Failed to transfer wifi cipher: %d", cipher);
        return NULL;
    }
}

static FAR const char *get_authstr(int auth)
{
  switch (auth)
    {
      case IW_AUTH_WPA_VERSION_DISABLED:
        return NULL;

      case IW_AUTH_WPA_VERSION_WPA:
        return "WPA";

      case IW_AUTH_WPA_VERSION_WPA2:
        return "WPA2";
      default:
        nerr("ERROR: Failed to transfer wifi auth: %d", auth);
        return NULL;
    }
}

/* implement iw_ops */

static FAR struct wifi_sim_bss_s *select_bss(FAR struct wifi_sim_s *sta,
                                             FAR char *bss_buf)
{
  char bss[256];
  FAR char *s;
  FAR char *p;
  struct wifi_sim_bss_s bss_info;
  FAR struct wifi_sim_bss_s *sbss = NULL;
  bool bss_found = false;

  for (p = bss_buf; *p != '\0'; p++)
    {
      s = p;
      while (*p != '\n')
        {
          p++;
        }

      memset(bss, 0, sizeof(bss));
      memcpy(bss, s, p - s + 1);
      wifidriver_get_bssinfo(&bss_info, bss, strlen(bss));

      /* compare bssid or essid, and security */

      if ((sta->ssid_flag == 2 && !memcmp(sta->bssid, bss_info.bssid, 6)) ||
          (sta->ssid_flag == 1 && !memcmp(sta->ssid, bss_info.ssid, 32)))
        {
          /* cmp security */

          if (sta->psk_flag == 0)
            {
              if (!strstr(bss_info.security, "WPA") &&
                  !strstr(bss_info.security, "WEP"))
                {
                  bss_found = true;
                }
              else
                {
                  nerr("security doesn't match");
                }

              break;
            }
          else if (get_cipherstr(sta->pairwise_chiper) &&
                   strstr(bss_info.security,
                          get_cipherstr(sta->pairwise_chiper)))
            {
              bss_found = true;
              break;
            }
          else
            {
              nerr("security doesn't match");
            }
        }
    }

  if (bss_found)
    {
      sbss = malloc(sizeof(struct wifi_sim_bss_s));
      if (sbss == NULL)
        {
          nerr("select bss  malloc failed!");
          return NULL;
        }

      memcpy(sbss, &bss_info, sizeof(struct wifi_sim_bss_s));
    }
  else
    {
      nerr("No BSS match.");
    }

  return sbss;
}

static int verify_password(FAR struct wifi_sim_s *sta,
                           FAR struct wifi_sim_bss_s *bss)
{
  int ret = ERROR;

  if (sta->psk_flag == 0)
    {
      ret = OK;
    }
  else if (!memcmp(sta->password, bss->password, strlen(sta->password)))
    {
      ret = OK;
    }

  return ret;
}

static int get_bss_from_file(FAR char **rbuf)
{
  int ret;
  int size = 4096;
  FAR char *p;

  *rbuf = malloc(size * sizeof(char));
  if (rbuf == NULL)
    {
      nerr("malloc failed!\n");
      return -ENOMEM;
    }

redo:
  ret = read_bss_config(*rbuf, size, BSS_FILE_PATH);
  if (ret == -E2BIG)
    {
      size += 1024;
      p = realloc(*rbuf, size);
      if (p == NULL)
        {
          nerr("read bss faied in realloc!\n");
          free(rbuf);
          return -ENOMEM;
        }

      *rbuf = p;
      goto redo;
    }
  else if (ret < 0)
    {
      nerr("read bss failed\n");
      return ret;
    }

  rbuf[ret] = '\0';

  return ret;
}

static bool wifidriver_sta_is_connected(FAR struct wifi_sim_s *wifidev)
{
  return (wifidev->state == WLAN_STA_STATE_CONNECTED);
}

static int wifidriver_start_connect(FAR struct wifi_sim_s *wifidev)
{
  int ret;
  FAR char *bss_buf = NULL;
  FAR struct wifi_sim_bss_s *bss_info;

  switch (wifidev->mode)
    {
      case IW_MODE_INFRA:
        {
          /* If wlan is connected, should be disconnect before connectting. */

          /* 1. check and disconnect */

          if (wifidev->state == WLAN_STA_STATE_CONNECTED)
            {
              wifidriver_start_disconnect(wifidev);
              wifidev->state = WLAN_STA_STATE_CONNECTING;
            }

          /* 2. get_bss, match (bssid/essid, security) */

          ret = get_bss_from_file(&bss_buf);
          if (ret < 0 || bss_buf == NULL)
            {
              goto error;
            }

          bss_info = select_bss(wifidev, bss_buf);
          if (bss_info == NULL)
            {
              nerr("select no bss.");
              wifidev->error_code = WLAN_STA_CONNERR_NO_BSS;

              ret = ERROR;
              goto error;
            }

          /* 3. verify the password and connect to the bss */

          ret = verify_password(wifidev, bss_info);
          if (ret == OK)
            {
              union iwreq_data wrqu;

              wifidev->state        = WLAN_STA_STATE_CONNECTED;
              wifidev->status_code  = WLAN_STATUS_SUCCESS;
              wifidev->connected_ap = bss_info;

              memset(&wrqu, 0, sizeof(wrqu));
              memcpy(wrqu.ap_addr.sa_data, bss_info->bssid, ETH_ALEN);
              wifi_send_event(wifidev, SIOCGIWAP, &wrqu);
            }
          else
            {
              wifidev->reason_code = WLAN_REASON_DEAUTH_LEAVING;
              wifidev->error_code  = WLAN_STA_CONNERR_WRONG_KEY;
              free(bss_info);

              ret = WLAN_REASON_CIPHER_SUITE_REJECTED;
            }

error:
          free(bss_buf);
          if (ret != OK)
            {
              wifidev->state = WLAN_STA_STATE_INIT;
            }

          wifidev->ssid_flag = 0;

          /* Enshure that the default is non-encrypted. */

          if (wifidev->psk_flag)
            {
              wifidev->psk_flag = 0;
            }
        }
        break;

      case IW_MODE_MASTER:
      default:
        ret = -EINVAL;
        break;
    }

  return ret;
}

static int wifidriver_start_disconnect(FAR struct wifi_sim_s *wifidev)
{
  int ret;
  union iwreq_data wrqu;

  switch (wifidev->mode)
    {
      case IW_MODE_INFRA:
        {
          if (wifidev->state == WLAN_STA_STATE_CONNECTED)
            {
              /* free the connected_ap */

              free(wifidev->connected_ap);

              netdev_lower_carrier_off(wifidev->lower);
              memset(&wrqu, 0, sizeof(wrqu));
              wrqu.ap_addr.sa_family = ARPHRD_ETHER;
              wifi_send_event(wifidev, SIOCGIWAP, &wrqu);
            }

          if (wifidev->psk_flag == 0)
            {
              memset(wifidev->password, 0, sizeof(wifidev->password));
              wifidev->pairwise_chiper = IW_AUTH_CIPHER_NONE;
            }

          wifidev->state = WLAN_STA_STATE_INIT;
          ret            = OK;
        }
        break;

      case IW_MODE_MASTER:
      default:
        ret = -ENOSYS;
        break;
    }

  return ret;
}

static int wifidriver_get_mode(FAR struct wifi_sim_s *wifidev,
                               FAR struct iwreq *pwrq)
{
  switch (wifidev->mode)
    {
      case IW_MODE_INFRA:
      case IW_MODE_MASTER:
        {
          pwrq->u.mode = wifidev->mode;
          ninfo("get %s mode", wifidev->lower->netdev.d_ifname);
        }
        break;

      default:
        return -EINVAL;
    }

  return OK;
}

static int wifidriver_set_mode(FAR struct wifi_sim_s *wifidev,
                               FAR struct iwreq *pwrq)
{
  switch (pwrq->u.mode)
    {
      case IW_MODE_INFRA:
      case IW_MODE_MASTER:
        {
          wifidev->mode = pwrq->u.mode;

          /* default country code CN */

          memcpy(wifidev->country, "CN", 2);
        }
        break;

      default:
        return -EINVAL;
    }

  return OK;
}

static int wifidriver_set_country(FAR struct wifi_sim_s *wifidev,
                                  FAR struct iwreq *pwrq)
{
  switch (wifidev->mode)
    {
      case IW_MODE_INFRA:
      case IW_MODE_MASTER:
        {
          memcpy(wifidev->country, pwrq->u.data.pointer,
                 pwrq->u.data.length);
          ninfo("set country is %s\n", wifidev->country);
        }
        break;

      default:
        return -EINVAL;
    }

  return OK;
}

static int wifidriver_get_country(FAR struct wifi_sim_s *wifidev,
                                  FAR struct iwreq *pwrq)
{
  switch (wifidev->mode)
    {
      case IW_MODE_INFRA:
      case IW_MODE_MASTER:
        {
          pwrq->u.data.length = strlen(wifidev->country);
          memcpy(pwrq->u.data.pointer, wifidev->country,
                 pwrq->u.data.length);
          ninfo("set country is %s\n", wifidev->country);
        }
        break;

      default:
        return -EINVAL;
    }

  return OK;
}

static int wifidriver_get_sensitivity(FAR struct wifi_sim_s *wifidev,
                                      FAR struct iwreq *pwrq)
{
  switch (wifidev->mode)
    {
      case IW_MODE_INFRA:
      case IW_MODE_MASTER:
        {
          if (wifidriver_sta_is_connected(wifidev))
            {
              pwrq->u.sens.value = -wifidev->connected_ap->RSSI;
            }

          ninfo("get rssi is %d\n", pwrq->u.sens.value);
        }
        break;

      default:
        return -EINVAL;
    }

  return OK;
}

int wifidriver_set_freq(FAR struct wifi_sim_s *wifidev,
                        FAR struct iwreq *pwrq)
{
  int ret;

  switch (wifidev->mode)
    {
      case IW_MODE_INFRA:
      case IW_MODE_MASTER:
        {
          ret = freq_to_channel(pwrq->u.freq.m);
          if (ret > 0)
            {
              wifidev->channel = ret;
              wifidev->freq = pwrq->u.freq.m;
              ninfo("set channel is %d\n", ret);
            }
        }
        break;

      default:
        ret = -EINVAL;
        break;
    }

  return ret;
}

static int wifidriver_get_freq(FAR struct wifi_sim_s *wifidev,
                               FAR struct iwreq *pwrq)
{
  switch (wifidev->mode)
    {
      case IW_MODE_INFRA:
        {
          if (wifidriver_sta_is_connected(wifidev))
            {
              pwrq->u.freq.flags = IW_FREQ_FIXED;
              pwrq->u.freq.e     = 0;
              pwrq->u.freq.m     = wifidev->connected_ap->freq;
            }
          else
            {
              pwrq->u.freq.flags = IW_FREQ_AUTO;
              pwrq->u.freq.e     = 0;
              pwrq->u.freq.m     = 2412;
            }
        }
        break;

      case IW_MODE_MASTER:
        {
          pwrq->u.freq.flags = IW_FREQ_FIXED;
          pwrq->u.freq.e     = 0;
          pwrq->u.freq.m     = wifidev->freq;
        }
        break;

      default:
        return -EINVAL;
    }

  return OK;
}

int wifidriver_set_txpower(FAR struct wifi_sim_s *wifidev,
                           FAR struct iwreq *pwrq)
{
  int ret;

  switch (wifidev->mode)
    {
      case IW_MODE_INFRA:
      case IW_MODE_MASTER:
        wifidev->txpower = pwrq->u.txpower.value;
        break;

      default:
        ret = -EINVAL;
        break;
    }

  return ret;
}

static int wifidriver_get_txpower(FAR struct wifi_sim_s *wifidev,
                                  FAR struct iwreq *pwrq)
{
  switch (wifidev->mode)
    {
      case IW_MODE_INFRA:
        {
          pwrq->u.txpower.value    = wifidev->txpower ? wifidev->txpower :
                                     WLAN_DEFAULT_TXPOWER;

          pwrq->u.txpower.fixed    = 0;
          pwrq->u.txpower.disabled = 0;
          pwrq->u.txpower.flags    = IW_TXPOW_DBM;
        }
        break;

      case IW_MODE_MASTER:
        break;
      default:
        return -EINVAL;
        break;
    }

  return OK;
}

static int wifidriver_get_bitrate(FAR struct wifi_sim_s *wifidev,
                                  FAR struct iwreq *pwrq)
{
  switch (wifidev->mode)
    {
      case IW_MODE_INFRA:
        if (wifidev->state != WLAN_STA_STATE_CONNECTED)
          {
            pwrq->u.bitrate.fixed = IW_FREQ_AUTO;
          }
        else
          {
            pwrq->u.bitrate.fixed = IW_FREQ_FIXED;

            /* default 2streams 40MHz */

            pwrq->u.bitrate.value = 150;
          }
        break;

      case IW_MODE_MASTER:
      default:
        return -EINVAL;
        break;
    }

  return OK;
}

static int wifidriver_get_range(FAR struct wifi_sim_s *wifidev,
                                FAR struct iwreq *pwrq)
{
  int k;
  int i;
  FAR struct iw_range *range = (FAR struct iw_range *)pwrq->u.data.pointer;

  /* default in china. */

  /* Add 2.4G channel */

  range->num_frequency = nitems(wifi_sim_channels_2ghz);
  for (k = 1; k <= range->num_frequency; k++)
    {
      range->freq[k - 1].i = k;
      range->freq[k - 1].e = 0;
      range->freq[k - 1].m = wifi_sim_channels_2ghz[k - 1].center_freq;
    }

  /* Add 5GHz channels */

  range->num_frequency += nitems(wifi_sim_channels_5ghz);
  for (i = 0; k <= range->num_frequency; k++)
    {
      range->freq[k - 1].i = k;
      range->freq[k - 1].e = 0;
      range->freq[k - 1].m = wifi_sim_channels_5ghz[i++].center_freq;
    }

  return OK;
}

static int wifidriver_start_scan(FAR struct wifi_sim_s *wifidev,
                                 FAR struct iwreq *pwrq)
{
  switch (wifidev->mode)
    {
      case IW_MODE_INFRA:
        {
          wifidev->scan_state = WLAN_SCAN_STATE_SCANNING;
          ninfo("scan %s\n", BSS_FILE_PATH);
        }
        break;

      case IW_MODE_MASTER:
      default:
        return -EINVAL;
        break;
    }

  return OK;
}

static int copy_scan_results(FAR struct iw_scan_result_s *scan_req,
                             FAR struct wifi_sim_bss_s *info)
{
  int need_len;
  FAR char *pointer;
  FAR struct iw_event *iwe;

  need_len = IW_EV_LEN(ap_addr) + IW_EV_LEN(qual) +
             IW_EV_LEN(freq) + IW_EV_LEN(data) + IW_EV_LEN(essid);

  if (scan_req->cur_len + need_len > scan_req->total_len)
    {
      scan_req->cur_len += need_len;
      return -E2BIG;
    }

  /* Copy scan result */

  pointer = scan_req->buf + scan_req->cur_len;

  /* 1.Copy BSSID */

  iwe                      = (FAR struct iw_event *)pointer;
  iwe->cmd                 = SIOCGIWAP;
  iwe->u.ap_addr.sa_family = ARPHRD_ETHER;
  memcpy(&iwe->u.ap_addr.sa_data, info->bssid, IFHWADDRLEN);
  iwe->len                 = IW_EV_LEN(ap_addr);
  pointer                 += iwe->len;

  /* 2.Copy ESSID */

  iwe                  = (FAR struct iw_event *)pointer;
  iwe->cmd             = SIOCGIWESSID;
  iwe->u.essid.flags   = 0;
  iwe->u.essid.length  = MIN(info->ssid_len, 32);
  iwe->u.essid.pointer = (FAR void *)sizeof(iwe->u.essid);
  memcpy(&iwe->u.essid + 1, info->ssid, iwe->u.essid.length);
  iwe->len             = IW_EV_LEN(essid) + ((iwe->u.essid.length + 3) & ~3);
  pointer             += iwe->len;

  /* 3.Copy link quality info */

  iwe                  = (FAR struct iw_event *)pointer;
  iwe->cmd             = IWEVQUAL;
  iwe->u.qual.qual     = info->snr;
  iwe->u.qual.level    = info->RSSI;
  iwe->u.qual.noise    = info->phy_noise;
  iwe->u.qual.updated  = IW_QUAL_DBM | IW_QUAL_ALL_UPDATED;
  iwe->len             = IW_EV_LEN(qual);
  pointer             += iwe->len;

  /* 4.Copy AP control channel */

  iwe                  = (FAR struct iw_event *)pointer;
  iwe->cmd             = SIOCGIWFREQ;
  iwe->u.freq.e        = -1;
  iwe->u.freq.m        = info->freq * 10;
  iwe->len             = IW_EV_LEN(freq);
  pointer             += iwe->len;

  /* 5.Copy AP encryption mode */

  iwe                  = (FAR struct iw_event *)pointer;
  iwe->cmd             = SIOCGIWENCODE;
  iwe->u.data.flags    = info->capability & IEEE80211_CAP_PRIVACY ?
                         IW_ENCODE_ENABLED | IW_ENCODE_NOKEY :
                         IW_ENCODE_DISABLED;
  iwe->u.data.length   = 0;
  iwe->u.essid.pointer = NULL;
  iwe->len             = IW_EV_LEN(data);
  pointer             += iwe->len;

  scan_req->cur_len    = pointer - scan_req->buf;

  return OK;
}

static int wifidriver_get_bssinfo(FAR struct wifi_sim_bss_s *bss_info,
                                  FAR char *buf, int len)
{
  unsigned char bssid[ETH_ALEN];
  char str[128];
  FAR char *p = NULL;
  FAR char *s = NULL;
  int i = 0;

  memset(bss_info, 0, sizeof(*bss_info));
  for (i = 0, p = buf; p - buf < len; p++, i++)
    {
      ninfo("  var_idx%d: ", i);
      s = p;
      while (*p != '\n' && *p != ',' && *p != '\0')
        {
          p++;
        }

      memset(str, 0, sizeof(str));
      memcpy(str, s, p - s);
      ninfo("%s", str);

      switch (i)
        {
          case 0:
            {
              /* bssid */

              mac_addr_a2n(bssid, str);
              memcpy(bss_info->bssid, bssid, IFHWADDRLEN);
            }
            break;

          case 1:
            {
              /* freq */

              bss_info->freq = atoi(str);
            }
            break;

          case 2:
            {
              /* signal */

              bss_info->RSSI = atoi(str);
            }
            break;

          case 3:
            {
              /* security */

              bss_info->capability |= strlen(str) > strlen("[ESS]") ?
                                      IEEE80211_CAP_PRIVACY : 0x01;
              memcpy(bss_info->security, str, p - s);
            }
            break;

          case 4:
            {
              /* ssid */

              memcpy(bss_info->ssid, str, p - s);
              bss_info->ssid_len = p - s;
            }
            break;

          case 5:
            {
              /* password */

              memcpy(bss_info->password, str, p - s);
            }
            break;

          default:
              break;
        }
    }

  ninfo("\n");

  return OK;
}

static int get_scan_results(FAR struct wifi_sim_s *wifidev,
                            FAR struct iw_scan_result_s *scan_reqs)
{
  int ret;
  FAR char *rbuf;
  char bss[128];
  FAR char *p;
  FAR char *s;
  struct wifi_sim_bss_s bss_info;

  ret = get_bss_from_file(&rbuf);
  if (ret < 0)
    {
      if (rbuf)
        {
          free(rbuf);
        }

      return ret;
    }

  ret = -EAGAIN;
  for (p = rbuf; *p != '\0'; p++)
    {
      s = p;
      while (*p != '\n')
        {
          p++;
        }

      memset(bss, 0, sizeof(bss));
      memcpy(bss, s, p - s + 1);
      ninfo("%s\n", bss);

      wifidriver_get_bssinfo(&bss_info, bss, strlen(bss));
      ret = copy_scan_results(scan_reqs, &bss_info);
      if (ret < 0)
        {
          break;
        }
    }

  free(rbuf);

  return ret;
}

static int wifidriver_scan_result(FAR struct wifi_sim_s *wifidev,
                                  FAR struct iwreq *pwrq)
{
  struct iw_scan_result_s scan_req;
  int ret = 0;

  if (wifidev->mode == IW_MODE_MASTER)
    {
      return OK;
    }
  else if (wifidev->mode == IW_MODE_INFRA)
    {
      if (wifidev->scan_state == WLAN_SCAN_STATE_SCANNING)
        {
          wifidev->scan_state = WLAN_SCAN_STATE_DONE;
          return -EAGAIN;
        }

      scan_req.buf       = pwrq->u.data.pointer;
      scan_req.total_len = pwrq->u.data.length;
      scan_req.cur_len   = 0;

      ret = get_scan_results(wifidev, &scan_req);

      pwrq->u.data.length = scan_req.cur_len;
    }

  return ret;
}

static int wifidriver_set_essid(FAR struct wifi_sim_s *wifidev,
                                FAR struct iwreq *pwrq)
{
  switch (wifidev->mode)
    {
      case IW_MODE_INFRA:
      case IW_MODE_MASTER:
        {
          memset(wifidev->ssid, 0, sizeof(wifidev->ssid));
          memcpy(wifidev->ssid, pwrq->u.essid.pointer, pwrq->u.essid.length);
          wifidev->ssid_flag = 1;
          ninfo("set essid = %s\n", wifidev->ssid);
        }
        break;

      default:
        return -EINVAL;
        break;
    }

  return OK;
}

static int wifidriver_get_essid(FAR struct wifi_sim_s *wifidev,
                                FAR struct iwreq *pwrq)
{
  FAR struct iw_point *essid = &pwrq->u.essid;

  switch (wifidev->mode)
    {
      case IW_MODE_INFRA:
        {
          /* get essid */

          pwrq->u.essid.length = strlen(wifidev->ssid);
          memcpy(pwrq->u.essid.pointer, wifidev->ssid, pwrq->u.essid.length);

          ninfo("get essid = %s\n", wifidev->ssid);

          /* get state */

          if (wifidev->state == WLAN_STA_STATE_CONNECTED)
            {
              essid->flags = IW_ESSID_ON;
            }
          else
            {
              essid->flags = IW_ESSID_OFF;
            }
        }
        break;

      case IW_MODE_MASTER:
      default:
        return -EINVAL;
    }

  return OK;
}

static int wifidriver_set_bssid(FAR struct wifi_sim_s *wifidev,
                                FAR struct iwreq *pwrq)
{
  FAR struct sockaddr *sockaddr = &pwrq->u.ap_addr;
  FAR unsigned char *pdata = (FAR unsigned char *)sockaddr->sa_data;
  char bssid_str[20];
  int ret;

  switch (wifidev->mode)
    {
      case IW_MODE_INFRA:
        {
          memcpy(wifidev->bssid, pdata, ETH_ALEN);
          mac_addr_n2a(bssid_str, pdata);
          ninfo("set bssid = %s\n", bssid_str);

          wifidev->ssid_flag = 2;
          ret = OK;
        }
        break;

      case IW_MODE_MASTER:
        ret = -ENOSYS;
        break;

      default:
        ret = -EINVAL;
        break;
    }

  return ret;
}

static int wifidriver_get_bssid(FAR struct wifi_sim_s *wifidev,
                                FAR struct iwreq *pwrq)
{
  FAR struct sockaddr *sockaddr = &pwrq->u.ap_addr;
  FAR unsigned char *pdata = (FAR unsigned char *)sockaddr->sa_data;
  int ret;

  switch (wifidev->mode)
    {
      case IW_MODE_INFRA:
        {
          if (wifidriver_sta_is_connected(wifidev))
            {
              memcpy(pdata, wifidev->connected_ap->bssid, ETH_ALEN);
              ret = OK;
            }
          else
            {
              ret = -ENOTTY;
            }
        }
        break;

      case IW_MODE_MASTER:
        ret = -ENOSYS;
        break;

      default:
        ret = -EINVAL;
        break;
    }

  return ret;
}

static int wifidriver_set_auth(FAR struct wifi_sim_s *wifidev,
                               FAR struct iwreq *pwrq)
{
  int flag  = pwrq->u.param.flags & IW_AUTH_INDEX;
  int value = pwrq->u.param.value;

  if (wifidev->mode != IW_MODE_INFRA)
    {
      return -ENOSYS;
    }

  switch (flag)
    {
      case IW_AUTH_WPA_VERSION:
        {
          /* record the value */

          wifidev->proto = value >> 1;

          ninfo("proto=%s\n", get_authstr(value));
        }
        break;

      case IW_AUTH_CIPHER_PAIRWISE:
        {
          /* record the value */

          wifidev->pairwise_chiper = value;

          ninfo("pairwise=%s\n", get_cipherstr(value));
        }
        break;

      default:
        nerr("ERROR: Unknown cmd %d\n", flag);
        return -ENOSYS;
    }

  return OK;
}

static int wifidriver_get_auth(FAR struct wifi_sim_s *wifidev,
                               FAR struct iwreq *pwrq)
{
  switch (wifidev->mode)
    {
      case IW_MODE_INFRA:
        break;
      case IW_MODE_MASTER:
        break;
      default:
        break;
    }

  return OK;
}

static int wifidriver_set_psk(FAR struct wifi_sim_s *wifidev,
                              FAR struct iwreq *pwrq)
{
  FAR struct iw_encode_ext *ext;
  int ret = OK;

  ext = (FAR struct iw_encode_ext *)pwrq->u.encoding.pointer;

  /* set auth_alg */

  switch (ext->alg)
    {
      case IW_ENCODE_ALG_TKIP:
      case IW_ENCODE_ALG_CCMP:
        break;
      case IW_ENCODE_ALG_NONE:
      case IW_ENCODE_ALG_WEP:
      default:
        return -ENOSYS;
    }

  switch (wifidev->mode)
    {
      case IW_MODE_INFRA:
        {
          wifidev->auth_alg = ext->alg;
          memset(wifidev->password, 0, sizeof(wifidev->password));
          memcpy(wifidev->password, ext->key, ext->key_len);

          ninfo("psk=%s, key_len= %d, alg=%u\n", wifidev->password,
                ext->key_len, ext->alg);

          /* Set the psk flag for security ap. */

          wifidev->psk_flag = 1;
        }
        break;

      case IW_MODE_MASTER:
      default:
        ret = -ENOSYS;
        break;
    }

  return ret ;
}

/* iw_ops */

static int wifidriver_connect(FAR struct netdev_lowerhalf_s *dev)
{
  int ret;

  ret = wifidriver_start_connect(LOWERDEV2WIFIDEV(dev));
  if (ret >= 0)
    {
      netdev_lower_carrier_on(dev);
    }

  return ret;
}

static int wifidriver_disconnect(FAR struct netdev_lowerhalf_s *dev)
{
  return wifidriver_start_disconnect(LOWERDEV2WIFIDEV(dev));
}

static int wifidriver_essid(FAR struct netdev_lowerhalf_s *dev,
                            FAR struct iwreq *iwr, bool set)
{
  FAR struct wifi_sim_s *wifidev = LOWERDEV2WIFIDEV(dev);

  if (set)
    {
      return wifidriver_set_essid(wifidev, iwr);
    }
  else
    {
      return wifidriver_get_essid(wifidev, iwr);
    }
}

static int wifidriver_bssid(FAR struct netdev_lowerhalf_s *dev,
                            FAR struct iwreq *iwr, bool set)
{
  int ret;
  FAR struct wifi_sim_s *wifidev = LOWERDEV2WIFIDEV(dev);

  if (set)
    {
      ret = wifidriver_set_bssid(wifidev, iwr);
      if (ret >= 0)
        {
           ret = wifidriver_start_connect(wifidev);
        }
    }
  else
    {
      ret = wifidriver_get_bssid(wifidev, iwr);
    }

  return ret;
}

static int wifidriver_passwd(FAR struct netdev_lowerhalf_s *dev,
                             FAR struct iwreq *iwr, bool set)
{
  if (set)
    {
      return wifidriver_set_psk(LOWERDEV2WIFIDEV(dev), iwr);
    }
  else
    {
      return -ENOTTY;
    }
}

static int wifidriver_mode(FAR struct netdev_lowerhalf_s *dev,
                           FAR struct iwreq *iwr, bool set)
{
  FAR struct wifi_sim_s *wifidev = LOWERDEV2WIFIDEV(dev);

  if (set)
    {
      return wifidriver_set_mode(wifidev, iwr);
    }
  else
    {
      return wifidriver_get_mode(wifidev, iwr);
    }
}

static int wifidriver_auth(FAR struct netdev_lowerhalf_s *dev,
                           FAR struct iwreq *iwr, bool set)
{
  FAR struct wifi_sim_s *wifidev = LOWERDEV2WIFIDEV(dev);

  if (set)
    {
      return wifidriver_set_auth(wifidev, iwr);
    }
  else
    {
      return wifidriver_get_auth(wifidev, iwr);
    }
}

static int wifidriver_freq(FAR struct netdev_lowerhalf_s *dev,
                           FAR struct iwreq *iwr, bool set)
{
  FAR struct wifi_sim_s *wifidev = LOWERDEV2WIFIDEV(dev);

  if (set)
    {
      return wifidriver_set_freq(wifidev, iwr);
    }
  else
    {
      return wifidriver_get_freq(wifidev, iwr);
    }
}

static int wifidriver_bitrate(FAR struct netdev_lowerhalf_s *dev,
                              FAR struct iwreq *iwr, bool set)
{
  FAR struct wifi_sim_s *wifidev = LOWERDEV2WIFIDEV(dev);

  if (set)
    {
      return -ENOTTY;
    }
  else
    {
      return wifidriver_get_bitrate(wifidev, iwr);
    }
}

static int wifidriver_txpower(FAR struct netdev_lowerhalf_s *dev,
                              FAR struct iwreq *iwr, bool set)
{
  FAR struct wifi_sim_s *wifidev = LOWERDEV2WIFIDEV(dev);

  if (set)
    {
      return wifidriver_set_txpower(wifidev, iwr);
    }
  else
    {
      return wifidriver_get_txpower(wifidev, iwr);
    }
}

static int wifidriver_country(FAR struct netdev_lowerhalf_s *dev,
                              FAR struct iwreq *iwr, bool set)
{
  FAR struct wifi_sim_s *wifidev = LOWERDEV2WIFIDEV(dev);

  if (set)
    {
      return wifidriver_set_country(wifidev, iwr);
    }
  else
    {
      return wifidriver_get_country(wifidev, iwr);
    }
}

static int wifidriver_sensitivity(FAR struct netdev_lowerhalf_s *dev,
                                  FAR struct iwreq *iwr, bool set)
{
  FAR struct wifi_sim_s *wifidev = LOWERDEV2WIFIDEV(dev);

  if (set)
    {
      return -ENOTTY;
    }
  else
    {
      return wifidriver_get_sensitivity(wifidev, iwr);
    }
}

static int wifidriver_scan(FAR struct netdev_lowerhalf_s *dev,
                           FAR struct iwreq *iwr, bool set)
{
  FAR struct wifi_sim_s *wifidev = LOWERDEV2WIFIDEV(dev);

  if (set)
    {
      return wifidriver_start_scan(wifidev, iwr);
    }
  else
    {
      return wifidriver_scan_result(wifidev, iwr);
    }
}

static int wifidriver_range(FAR struct netdev_lowerhalf_s *dev,
                            FAR struct iwreq *iwr)
{
  return wifidriver_get_range(LOWERDEV2WIFIDEV(dev), iwr);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wifi_sim_init
 ****************************************************************************/

int wifi_sim_init(FAR struct wifi_sim_lowerhalf_s *netdev)
{
  FAR struct wifi_sim_s *priv;

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      nerr("virt wifi driver priv alloc failed\n");
      return -ENOMEM;
    }

  netdev->lower.iw_ops = &g_iw_ops;
  priv->lower          = &netdev->lower;
  netdev->wifi         = priv;

  return OK;
}

/****************************************************************************
 * Name: wifi_sim_remove
 ****************************************************************************/

void wifi_sim_remove(FAR struct wifi_sim_lowerhalf_s *netdev)
{
  FAR struct wifi_sim_s *sta = (FAR struct wifi_sim_s *)netdev->wifi;

  if (sta && sta->state == WLAN_STA_STATE_CONNECTED)
    {
       wifidriver_start_disconnect(sta);
    }

  kmm_free(netdev->wifi);
}

