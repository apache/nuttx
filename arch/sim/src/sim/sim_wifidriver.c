/****************************************************************************
 * arch/sim/src/sim/sim_wifidriver.c
 * Manage the host wireless
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

#include <stdlib.h>
#include <stdio.h>
#include <debug.h>
#include <netinet/if_ether.h>
#include <nuttx/wireless/wireless.h>
#include <nuttx/net/netdev_lowerhalf.h>

#include "sim_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CMD_LEN 512
#define BUF_LEN 1024
#define SSID_MAX_LEN 33

/* ESSID flags */

#define IW_ESSID_OFF        0    /* Disconnect with access point */
#define IW_ESSID_ON         1    /* Connect  with access point */

#define IEEE80211_CAP_PRIVACY 0x0010
#define WLAN_EID_SSID 0

#define IW_EV_LEN(field) \
  (offsetof(struct iw_event, u) + sizeof(((union iwreq_data *)0)->field))

#define WPA_CTRL_PATH " -p /var/run/simwifi/wpa_supplicant "
#define WPA_CLI "/usr/bin/sudo /usr/sbin/wpa_cli "

#define HOSTAPD_CTRL_PATH " -p /var/run/simwifi/hostapd "
#define HOSTAPD_CLI "/usr/bin/sudo /usr/sbin/hostapd_cli "
#define SIMWIFI_FILE "/usr/bin/sim_wifi.sh "

#define WPA_SET_NETWORK(wifidev, fmt, args...) \
    set_cmd(wifidev,"set_network %d "fmt, (wifidev)->network_id, ##args)

#define get_cmd(wifidev, buf, buf_size, fmt, ...) \
    ({ \
      int ret_; \
      if (wifidev->mode == IW_MODE_INFRA) \
        { \
          ret_ = host_system(buf, buf_size, "%s %s -i wlan%d "fmt, WPA_CLI, \
                             WPA_CTRL_PATH, wifidev->devidx, ##__VA_ARGS__); \
        } \
      else if (wifidev->mode == IW_MODE_MASTER) \
        { \
          ret_ = host_system(buf, buf_size, "%s %s -i wlan%d "fmt, HOSTAPD_CLI, \
                            HOSTAPD_CTRL_PATH, wifidev->devidx, ##__VA_ARGS__); \
        } \
      else \
        { \
          ret_ = -EINVAL; \
        } \
      ret_; \
    })

#define set_cmd(wifidev, fmt, ...)                               \
  ({                                                             \
    char rbuf[BUF_LEN];                                          \
    int ret__ = -EINVAL;                                         \
    if (get_cmd(wifidev, rbuf, BUF_LEN, fmt, ##__VA_ARGS__) > 0) \
      {                                                          \
        if (!strncmp(rbuf, "OK", 2))                             \
          {                                                      \
            ninfo(fmt" is OK\n", ##__VA_ARGS__);                 \
            ret__ = 0;                                           \
          }                                                      \
        else if (!strncmp(rbuf, "FAIL", 4))                      \
          {                                                      \
            nerr("ERROR: "fmt" is failed!\n", ##__VA_ARGS__);    \
            ret__ = -EINVAL;                                     \
          }                                                      \
      }                                                          \
    ret__;                                                       \
  })

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sim_scan_result_s
{
  int total_len;
  int cur_len;
  char *buf;                    /* iwr->u.data.pointer */
};

struct sim_bss_info_s
{
  uint32_t version;             /* version field */
  uint32_t length;              /* byte length of data in this record, */
                                /* starting at version and including IEs */
  struct ether_addr BSSID;
  uint16_t beacon_period;       /* units are Kusec */
  uint16_t capability;          /* Capability information */
  uint8_t ssid_len;
  char ssid[32];
  struct
  {
    uint32_t count;             /* rates in this set */
    uint8_t rates[16];          /* rates in 500kbps units w/hi bit set if
                                 * basic */
  } rateset;                    /* supported rates */
  uint16_t atim_window;         /* units are Kusec */
  uint8_t dtim_period;          /* DTIM period */
  int16_t RSSI;                 /* receive signal strength (in dBm) */
  int8_t phy_noise;             /* noise (in dBm) */

  uint8_t n_cap;                /* BSS is 802.11N Capable */
  uint32_t nbss_cap;            /* 802.11N BSS Capabilities (based on *
                                 * HT_CAP_*) */
  uint32_t freq;                /* 802.11N BSS frequency */
  uint8_t ctl_ch;               /* 802.11N BSS control channel number */
  uint32_t reserved32[1];       /* Reserved for expansion of BSS *
                                 * properties */
  uint8_t flags;                /* flags */
  uint8_t reserved[3];          /* Reserved for expansion of BSS *
                                 * properties */
  uint8_t basic_mcs[16];        /* 802.11N BSS required MCS set */

  uint16_t ie_offset;           /* offset at which IEs start, from *
                                 * beginning */
  uint32_t ie_length;           /* byte length of Information Elements */
  int16_t snr;                  /* average SNR of during frame reception */
};

struct sim_netdev_s
{
  struct netdev_lowerhalf_s dev;
  uint8_t buf[SIM_NETDEV_BUFSIZE]; /* Used when packet buffer is fragmented */
  char ssid[SSID_MAX_LEN];
  char bssid[ETH_ALEN];
  uint16_t channel;
  uint32_t freq;
  uint8_t password[64];
  int key_mgmt;
  int proto;
  int auth_alg;
  int pairwise_chiper;
  int group_cipher;
  uint8_t mode;                 /* IW_MODE_INFRA/ IW_MODE_MASTER */
  uint32_t bitrate;
  uint8_t txpower;
  char country[4];
  int8_t sensitivity;
  uint8_t devidx;
  bool psk_flag;                /* for psk, 0: unset, 1: set */
  char host_ifname[IFNAMSIZ];   /* The wlan interface name on the host */
  uint8_t network_id;           /* for sta, default is 0 */
};

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

static int wifidriver_connect(struct netdev_lowerhalf_s *dev);
static int wifidriver_disconnect(struct netdev_lowerhalf_s *dev);
static int wifidriver_essid(struct netdev_lowerhalf_s *dev,
                            struct iwreq *iwr, bool set);
static int wifidriver_bssid(struct netdev_lowerhalf_s *dev,
                            struct iwreq *iwr, bool set);
static int wifidriver_passwd(struct netdev_lowerhalf_s *dev,
                             struct iwreq *iwr, bool set);
static int wifidriver_mode(struct netdev_lowerhalf_s *dev,
                           struct iwreq *iwr, bool set);
static int wifidriver_auth(struct netdev_lowerhalf_s *dev,
                           struct iwreq *iwr, bool set);
static int wifidriver_freq(struct netdev_lowerhalf_s *dev,
                           struct iwreq *iwr, bool set);
static int wifidriver_bitrate(struct netdev_lowerhalf_s *dev,
                              struct iwreq *iwr, bool set);
static int wifidriver_txpower(struct netdev_lowerhalf_s *dev,
                              struct iwreq *iwr, bool set);
static int wifidriver_country(struct netdev_lowerhalf_s *dev,
                              struct iwreq *iwr, bool set);
static int wifidriver_sensitivity(struct netdev_lowerhalf_s *dev,
                                  struct iwreq *iwr, bool set);
static int wifidriver_scan(struct netdev_lowerhalf_s *dev,
                           struct iwreq *iwr, bool set);
static int wifidriver_range(struct netdev_lowerhalf_s *dev,
                            struct iwreq *iwr);

/****************************************************************************
 * Private Data
 ****************************************************************************/

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

static int get_cmd_result_num(struct sim_netdev_s *wifidev, char *cmd)
{
  int num = 0;
  char rbuf[BUF_LEN];

  if (get_cmd(wifidev, rbuf, BUF_LEN, "%s", cmd) > 0)
    {
      num = atoi(rbuf);
    }
  else
    {
      nerr("ERROR: Failed to get num.\n");
      num = -EINVAL;
    }

  return num;
}

/* For sta, add an available network. */

static int wpa_add_network(struct sim_netdev_s *wifidev)
{
  return get_cmd_result_num(wifidev, "add_network");
}

/* For sta, get the number of available networks. */

static int wpa_get_network_num(struct sim_netdev_s *wifidev)
{
  return get_cmd_result_num(wifidev, "list_network | grep \"\\[\" | wc -l");
}

/* For sta, get the available network_id. */

static int wpa_get_last_network_id(struct sim_netdev_s *wifidev,
                                   int network_num)
{
  int num;
  int i = 0;
  char rbuf[BUF_LEN];

  num = get_cmd(wifidev, rbuf, BUF_LEN, "%s",
                "list_network | grep \"\\[\" | awk '{print $1}'");
  if (num > 0)
    {
      while (--network_num)
        {
          if (rbuf[i] == '\n')
            {
              i++;
            }

          while (rbuf[i] != '\n')
            {
              i++;
            }
        }

      num = atoi(rbuf + i);
    }
  else
    {
      nerr("ERROR: Failed to get the last network ID.");
    }

  return num;
}

static const char *get_auth_algstr(uint32_t auth_alg)
{
  switch (auth_alg)
    {
    case IW_ENCODE_ALG_TKIP:
    case IW_ENCODE_ALG_CCMP:
      return "OPEN";
    default:
      nerr("ERROR: Failed to transfer wireless auth alg: %u", auth_alg);
      return NULL;
    }
}

static const char *get_authstr(int auth)
{
  switch (auth)
    {
    case IW_AUTH_WPA_VERSION_DISABLED:
      return NULL;

    case IW_AUTH_WPA_VERSION_WPA:
      return "WPA";

    case IW_AUTH_WPA_VERSION_WPA2:
      return "RSN";
    default:
      nerr("ERROR: Failed to transfer wifi auth: %d", auth);
      return NULL;
    }
}

static const char *get_cipherstr(int cipher)
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

static void mac_addr_n2a(char *mac_addr, unsigned char *arg)
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

static int mac_addr_a2n(unsigned char *mac_addr, char *arg)
{
  int i;

  for (i = 0; i < ETH_ALEN ; i++)
    {
      int temp;
      char *cp = strchr(arg, ':');

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

static int utf8_escape(char *outp, int out_size,
                       const char *inp, size_t in_size)
{
  size_t res_size = 0;

  if (!inp || !outp)
    {
      return -EINVAL;
    }

  /* The inp may or may not be NUL terminated, but must be
   * if 0 size is specified.
   */

  if (!in_size)
    {
      in_size = strlen(inp);
    }

  while (in_size)
    {
      in_size--;

      if (res_size++ >= out_size)
        {
          return -EINVAL;
        }

      switch (*inp)
        {
          case '\\':
            if (*(inp + 1) == '\'')
              {
                inp++;
                break;
              }

          case '\"':
            if (res_size++ >= out_size)
              {
                return -EINVAL;
              }

            *outp++ = '\\';

          default:
            *outp++ = *inp++;
            break;
        }
    }

  /* NUL terminate if space allows */

  if (res_size < out_size)
    {
      *outp = '\0';
    }

  return 0;
}

static int utf8_unescape(char *outp, int out_size,
                         const char *inp, size_t in_size)
{
  size_t res_size = 0;

  if (!inp || !outp)
    {
      return -EINVAL;
    }

  if (!in_size)
    {
      in_size = strlen(inp);
    }

  for (in_size--; in_size >= 0; in_size--, res_size++)
    {
      if (res_size >= out_size)
        {
          return -EINVAL;
        }

      if (*inp == '\\')
        {
          in_size--;
          inp++;
        }

      *outp++ = *inp++;
    }

  /* NUL terminate if space allows */

  if (res_size < out_size)
    {
      *outp = '\0';
    }

  return res_size;
}

static int hex2nibble(char c)
{
  if (c >= '0' && c <= '9')
    {
      return c - '0';
    }

  if (c >= 'a' && c <= 'f')
    {
      return c - 'a' + 10;
    }

  if (c >= 'A' && c <= 'F')
    {
      return c - 'A' + 10;
    }

  return -EINVAL;
}

static int hex2byte(const char *hex)
{
  int a;
  int b;

  a = hex2nibble(*hex++);
  if (a < 0)
    {
      return -EINVAL;
    }

  b = hex2nibble(*hex++);
  if (b < 0)
    {
      return -EINVAL;
    }

  return (a << 4) | b;
}

static size_t wpa_ssid_decode(char *buf, size_t maxlen, const char *str)
{
  const char *pos = str;
  size_t len = 0;
  int val;

  while (*pos)
    {
      if (len + 1 >= maxlen)
        {
          break;
        }

      switch (*pos)
        {
          case '\\':
            pos++;
            switch (*pos)
              {
                case '\\':
                    buf[len++] = '\\';
                    pos++;
                    break;
                case '"':
                    buf[len++] = '"';
                    pos++;
                    break;
                case 'n':
                    buf[len++] = '\n';
                    pos++;
                    break;
                case 'r':
                    buf[len++] = '\r';
                    pos++;
                    break;
                case 't':
                    buf[len++] = '\t';
                    pos++;
                    break;
                case 'e':
                    buf[len++] = '\033';
                    pos++;
                    break;
                case 'x':
                    pos++;
                    val = hex2byte(pos);
                    if (val < 0)
                      {
                        val = hex2nibble(*pos);
                        if (val < 0)
                            break;
                        buf[len++] = val;
                        pos++;
                      }
                    else
                      {
                        buf[len++] = val;
                        pos += 2;
                      }
                    break;
                case '0':
                case '1':
                case '2':
                case '3':
                case '4':
                case '5':
                case '6':
                case '7':
                    val = *pos++ - '0';
                    if (*pos >= '0' && *pos <= '7')
                      {
                        val = val * 8 + (*pos++ - '0');
                      }

                    if (*pos >= '0' && *pos <= '7')
                      {
                        val = val * 8 + (*pos++ - '0');
                      }

                    buf[len++] = val;
                    break;
                default:
                    break;
              }
            break;
        default:
            buf[len++] = *pos++;
            break;
      }
  }

  return len;
}

static int copy_scan_results(struct sim_scan_result_s *scan_req,
                             struct sim_bss_info_s *info)
{
  int need_len;
  char *pointer;
  struct iw_event *iwe;

  need_len = IW_EV_LEN(ap_addr) + IW_EV_LEN(qual) +
             IW_EV_LEN(freq) + IW_EV_LEN(data) +
             IW_EV_LEN(essid) + ((MIN(info->ssid_len, 32) + 3) & ~3);

  if (scan_req->cur_len + need_len > scan_req->total_len)
    {
      scan_req->cur_len += need_len;
      return -E2BIG;
    }

  /* Copy scan result */

  pointer = scan_req->buf + scan_req->cur_len;

  /* 1.Copy BSSID */

  iwe = (struct iw_event *)pointer;
  iwe->cmd = SIOCGIWAP;
  iwe->u.ap_addr.sa_family = ARPHRD_ETHER;
  memcpy(&iwe->u.ap_addr.sa_data,
         info->BSSID.ether_addr_octet, IFHWADDRLEN);
  iwe->len = IW_EV_LEN(ap_addr);
  pointer += iwe->len;

  /* 2.Copy ESSID */

  iwe = (struct iw_event *)pointer;
  iwe->cmd = SIOCGIWESSID;
  iwe->u.essid.flags = 0;
  iwe->u.essid.length = MIN(info->ssid_len, 32);
  iwe->u.essid.pointer = (void *)sizeof(iwe->u.essid);
  memcpy(&iwe->u.essid + 1, info->ssid, iwe->u.essid.length);
  iwe->len = IW_EV_LEN(essid) + ((iwe->u.essid.length + 3) & ~3);
  pointer += iwe->len;

  /* 3.Copy link quality info */

  iwe = (struct iw_event *)pointer;
  iwe->cmd = IWEVQUAL;
  iwe->u.qual.qual = info->snr;
  iwe->u.qual.level = info->RSSI;
  iwe->u.qual.noise = info->phy_noise;
  iwe->u.qual.updated = IW_QUAL_DBM | IW_QUAL_ALL_UPDATED;
  iwe->len = IW_EV_LEN(qual);
  pointer += iwe->len;

  /* 4.Copy AP control channel */

  iwe = (struct iw_event *)pointer;
  iwe->cmd = SIOCGIWFREQ;
  iwe->u.freq.e = -1;
  iwe->u.freq.m = info->freq * 10;
  iwe->len = IW_EV_LEN(freq);
  pointer += iwe->len;

  /* 5.Copy AP encryption mode */

  iwe = (struct iw_event *)pointer;
  iwe->cmd = SIOCGIWENCODE;
  iwe->u.data.flags = info->capability & IEEE80211_CAP_PRIVACY ?
                      IW_ENCODE_ENABLED | IW_ENCODE_NOKEY :
                      IW_ENCODE_DISABLED;
  iwe->u.data.length = 0;
  iwe->u.essid.pointer = NULL;
  iwe->len = IW_EV_LEN(data);
  pointer += iwe->len;

  scan_req->cur_len = pointer - scan_req->buf;

  return OK;
}

static int get_bss_info(struct sim_bss_info_s *bss_info, char *buf, int len)
{
  unsigned char bssid[ETH_ALEN];
  char str[256];
  char *p = NULL;
  char *s = NULL;
  int i = 0;

  memset(bss_info, 0, sizeof(*bss_info));
  for (i = 0, p = buf; p - buf < len; p++, i++)
    {
      ninfo("  var_idx%d: ", i);
      s = p;
      while (*p != '\n' && *p != ' ' && *p != '\0')
        {
          p++;
        }

      memset(str, 0, sizeof(str));
      memcpy(str, s, p - s);
      ninfo("%s", str);

      switch (i)
        {
          case 0:    /* bssid */
              mac_addr_a2n(bssid, str);
              memcpy(bss_info->BSSID.ether_addr_octet,
                     bssid, IFHWADDRLEN);
              break;
          case 1:    /* freq */
              bss_info->freq = atoi(str);
              break;
          case 2:    /* signal */
              bss_info->RSSI = atoi(str);
              break;
          case 3:    /* security */
              bss_info->capability |= strlen(str) > strlen("[ESS]") ?
                                      IEEE80211_CAP_PRIVACY : 0x01;
              break;
          case 4:    /* ssid */
              if (p - s > SSID_MAX_LEN)
                {
                  wpa_ssid_decode(bss_info->ssid, SSID_MAX_LEN, str);
                }
              else
                {
                  memcpy(bss_info->ssid, str, p - s);
                }

              bss_info->ssid_len =
                      utf8_unescape(bss_info->ssid, sizeof(bss_info->ssid),
                                    bss_info->ssid, strlen(bss_info->ssid));

              break;
          default:
              break;
        }
    }

  return OK;
}

static int get_scan_results(struct sim_netdev_s *wifidev,
                            struct sim_scan_result_s *scan_reqs)
{
  int ret;
  int size = 4096;
  char *rbuf;
  char bss[512];
  char *p;
  char *s;
  struct sim_bss_info_s bss_info;

  rbuf = malloc(size * sizeof(char));
  if (rbuf == NULL)
    {
      nerr("malloc failed!\n");
      return -ENOMEM;
    }

get_scan:
  ret = host_system(rbuf, size, "%s %s -i%s %s", WPA_CLI, WPA_CTRL_PATH,
                    wifidev->host_ifname, "scan_results | grep \"\\[ESS\"|"
                    "awk '{print $1,$2,$3,$4,$5}'");
  if (ret < 0)
    {
      nerr("get scan failed\n");
      free(rbuf);
      return ret;
    }
  else if (ret == size)
    {
      size += 1024;
      p = realloc(rbuf, size);
      if (p == NULL)
        {
          nerr("get scan faied in realloc!\n");
          free(rbuf);
          return -ENOMEM;
        }

      rbuf = p;
      goto get_scan;
    }

  /* Add a terminator for the rbuf */

  rbuf[ret] = '\0';

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

      get_bss_info(&bss_info, bss, strlen(bss));
      ninfo("\n");

      ret = copy_scan_results(scan_reqs, &bss_info);
      if (ret < 0)
        {
          break;
        }
    }

  free(rbuf);

  return ret;
}

static bool get_wpa_state(struct sim_netdev_s *wifidev)
{
  int ret;
  char rbuf[BUF_LEN];

  ret = get_cmd(wifidev, rbuf, BUF_LEN, "%s",
                "status | grep wpa_state | awk -F'=' '{print $2}'");

  if (ret > 0 && !strncmp(rbuf, "COMPLETED", strlen("COMPLETED")))
    {
      return true;
    }

  return false;
}

static void get_wpa_ssid(struct sim_netdev_s *wifidev,
                         struct iw_point *essid)
{
  essid->length = strlen(wifidev->ssid);
  strlcpy(essid->pointer, wifidev->ssid, essid->length + 1);
}

static int get_wpa_freq(struct sim_netdev_s *wifidev)
{
  return get_cmd_result_num(wifidev,
                            "status | grep freq | awk -F'=' '{print $2}'");
}

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

static int wifidriver_start_scan(struct sim_netdev_s *wifidev,
                                 struct iwreq *pwrq)
{
  int ret;
  uint8_t  mode = wifidev->mode;

  if (mode == IW_MODE_MASTER)
    {
      return OK;
    }
  else if (mode == IW_MODE_INFRA)
    {
      ret = set_cmd(wifidev, "scan");
      if (ret == -EINVAL)
        {
          nerr("ERROR: scan is running!\n");
        }
    }
  else
    {
      nerr("ERROR: Scan do not support the mode %d ! \n", mode);
      ret = -ENOSYS;
    }

  return ret;
}

static int wifidriver_scan_result(struct sim_netdev_s *wifidev,
                               struct iwreq *pwrq)
{
  struct sim_scan_result_s scan_req;
  int ret = 0;

  if (wifidev->mode == IW_MODE_MASTER)
    {
      return OK;
    }
  else if (wifidev->mode == IW_MODE_INFRA)
    {
      scan_req.buf = pwrq->u.data.pointer;
      scan_req.total_len = pwrq->u.data.length;
      scan_req.cur_len = 0;

      ret = get_scan_results(wifidev, &scan_req);

      pwrq->u.data.length = scan_req.cur_len;
    }

  return ret;
}

static int wifidriver_set_auth(struct sim_netdev_s *wifidev,
                               struct iwreq *pwrq)
{
  int ret = 0;
  int flag = pwrq->u.param.flags & IW_AUTH_INDEX;
  int value = pwrq->u.param.value;

  switch (flag)
    {
    case IW_AUTH_WPA_VERSION:

      /* record the value */

      wifidev->proto = value >> 1;

      ninfo("proto=%s\n", get_authstr(value));

      if (wifidev->mode == IW_MODE_INFRA)
        {
          ret = WPA_SET_NETWORK(wifidev, "proto %s", get_authstr(value));
        }
      else if(wifidev->mode == IW_MODE_MASTER)
        {
          /* set ap value */

          ret = set_cmd(wifidev, "set wpa %d", wifidev->proto);
          if (value == IW_AUTH_WPA_VERSION_WPA ||
              value == IW_AUTH_WPA_VERSION_WPA2)
            {
              ret = set_cmd(wifidev, "set wpa_key_mgmt %s", "WPA-PSK");
            }
          else
            {
              ret = set_cmd(wifidev, "set wpa %d", wifidev->proto);
            }
        }
      break;

    case IW_AUTH_CIPHER_PAIRWISE:

      /* record the value */

      wifidev->pairwise_chiper = value;

      ninfo("pairwise=%s\n", get_cipherstr(value));

      if (wifidev->mode == IW_MODE_INFRA)
        {
          ret = WPA_SET_NETWORK(wifidev, "pairwise %s",
                                get_cipherstr(value));
        }
      else if(wifidev->mode == IW_MODE_MASTER)
        {
          /* set rsn_pairwise value */

          ret = set_cmd(wifidev, "set rsn_pairwise %s",
                        get_cipherstr(value));
        }
      break;
    default:
      nerr("ERROR: Unknown cmd %d\n", flag);
      return -ENOSYS;
    }

  return ret;
}

static int wifidriver_get_auth(struct sim_netdev_s *wifidev,
                               struct iwreq *pwrq)
{
  int ret = 0;

  switch (wifidev->mode)
    {
    case IW_MODE_INFRA:
      break;
    case IW_MODE_MASTER:
      break;
    default:
      break;
    }

  return ret;
}

static int wifidriver_set_psk(struct sim_netdev_s *wifidev,
                              struct iwreq *pwrq)
{
  char psk_buf[64];
  struct iw_encode_ext *ext;
  int ret = 0;

  ext = (struct iw_encode_ext *)pwrq->u.encoding.pointer;

  memset(psk_buf, 0, sizeof(psk_buf));
  memcpy(psk_buf, ext->key, ext->key_len);

  ninfo("psk=%s, key_len= %d, alg=%u\n", psk_buf, ext->key_len, ext->alg);

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
      WPA_SET_NETWORK(wifidev, "auth_alg %s", get_auth_algstr(ext->alg));
      WPA_SET_NETWORK(wifidev, "psk \\\"%s\\\"", psk_buf);
      WPA_SET_NETWORK(wifidev, "key_mgmt %s", "WPA-PSK WPA-EAP");

      /* Set the psk flag for security ap. */

      wifidev->psk_flag = 1;
      break;
    case IW_MODE_MASTER:

       /* set wpa_passphrase psk_buf */

      ret = set_cmd(wifidev, "set wpa_passphrase %s", psk_buf);
      wifidev->psk_flag = 1;
      break;
    default:
      break;
    }

  return ret ;
}

static int wifidriver_set_essid(struct sim_netdev_s *wifidev,
                             struct iwreq *pwrq)
{
  char ssid_buf[SSID_MAX_LEN];
  char out_ssid[256];
  int ret = 0;
  uint8_t ssid_len = pwrq->u.essid.length;

  memset(ssid_buf, 0, sizeof(ssid_buf));
  memcpy(ssid_buf, pwrq->u.essid.pointer, pwrq->u.essid.length);
  ninfo("ssid=%s, ssid_len=%d\n", ssid_buf, ssid_len);

  memset(wifidev->ssid, 0, SSID_MAX_LEN);
  memcpy(wifidev->ssid, ssid_buf, ssid_len);

  if (wifidev->mode == IW_MODE_INFRA)
    {
      ret = utf8_escape(out_ssid, sizeof(out_ssid), ssid_buf, ssid_len);
      if (ret < 0)
        {
          return ret;
        }

      WPA_SET_NETWORK(wifidev, "ssid \"\\\"%s\\\"\"", out_ssid);

      WPA_SET_NETWORK(wifidev, "scan_ssid 1");

      if (wifidev->psk_flag == 0)
        {
          /* should set the key_mgmt for open */

          WPA_SET_NETWORK(wifidev, "key_mgmt %s", "NONE");
          wifidev->key_mgmt = 0;
        }

      ret = set_cmd(wifidev, "save_config");
    }
  else if (wifidev->mode == IW_MODE_MASTER)
    {
      /* set ssid ssid_buf */

      ret = set_cmd(wifidev, "set ssid %s", ssid_buf);
      if (wifidev->psk_flag == 0)
        {
          /* should set the key_mgmt for open */

          ret = set_cmd(wifidev, "set wpa %d", 0);
          wifidev->proto = 0;
        }

      ret = set_cmd(wifidev, "reload");
    }

  return ret;
}

static int wifidriver_get_essid(struct sim_netdev_s *wifidev,
                             struct iwreq *pwrq)
{
  int ret = 0;
  struct iw_point *essid = &pwrq->u.essid;

  switch (wifidev->mode)
    {
    case IW_MODE_INFRA:
      {
        /* get essid */

        get_wpa_ssid(wifidev, essid);

        /* get wpa_state */

        if (get_wpa_state(wifidev))
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
      break;
    default:
      break;
    }

  return ret;
}

static int wifidriver_set_bssid(struct sim_netdev_s *wifidev,
                             struct iwreq *pwrq)
{
  int ret = 0;
  struct sockaddr *sockaddr = &pwrq->u.ap_addr;
  unsigned char *pdata = (unsigned char *)sockaddr->sa_data;
  char bssid_buf[20];

  memset(bssid_buf, 0, sizeof(bssid_buf));
  mac_addr_n2a(bssid_buf, pdata);

  ninfo("bssid=%s \n", bssid_buf);

  memset(wifidev->bssid, 0, 6);
  memcpy(wifidev->bssid, pdata, 6);

  if (wifidev->mode == IW_MODE_INFRA)
    {
      WPA_SET_NETWORK(wifidev, "bssid %s", bssid_buf);
      WPA_SET_NETWORK(wifidev, "scan_ssid 0");

      if (wifidev->psk_flag == 0)
        {
          /* should set the key_mgmt for open */

          WPA_SET_NETWORK(wifidev, "key_mgmt %s", "NONE");
          wifidev->key_mgmt = 0;
        }

      set_cmd(wifidev, "save_config");
    }
  else if (wifidev->mode == IW_MODE_MASTER)
    {
      ret = -ENOSYS;
    }

  return ret;
}

static int wifidriver_start_connect(struct sim_netdev_s *wifidev)
{
  switch (wifidev->mode)
    {
    case IW_MODE_INFRA:

      /* If wlan is connected, should be disconnect before connectting. */

      set_cmd(wifidev, "select_network %d", wifidev->network_id);
      set_cmd(wifidev, "disconnect");
      set_cmd(wifidev, "reconnect");

      /* Enshure that the default is non-encrypted. */

      if (wifidev->psk_flag)
        {
          wifidev->psk_flag = 0;
        }
      break;
    case IW_MODE_MASTER:

      set_cmd(wifidev, "disable");
      set_cmd(wifidev, "enable");

      if (wifidev->psk_flag)
        {
          wifidev->psk_flag = 0;
        }
      break;
    default:
        break;
    }

  return OK;
}

static int wifidriver_start_disconnect(struct sim_netdev_s *wifidev)
{
  int ret;

  switch (wifidev->mode)
    {
    case IW_MODE_INFRA:
      ret = set_cmd(wifidev, "disconnect");
      break;
    case IW_MODE_MASTER:
      ret = set_cmd(wifidev, "disable");
      break;
    default:
      ret = -ENOSYS;
      break;
    }

  return ret;
}

static int wifidriver_get_mode(struct sim_netdev_s *wifidev,
                               struct iwreq *pwrq)
{
  pwrq->u.mode = wifidev->mode;
  return OK;
}

static int wifidriver_set_mode(struct sim_netdev_s *wifidev,
                               struct iwreq *pwrq)
{
  int ret;

  /* IW_MODE_INFRA indicates station */

  wifidev->mode = pwrq->u.mode;
  switch (wifidev->mode)
    {
    case IW_MODE_INFRA:

      /* Start the sta config, including wpa_supplicant and udhcpc. */

      ret = host_system(NULL, 0,
                        "/usr/bin/sudo "SIMWIFI_FILE" start_sta %s",
                        wifidev->host_ifname);
      if (ret == 0)
        {
          /* Check the network number, if no network, should add new network.
          * Then, set the new network id to network_id.
          */

          int num;
          int network_id = 0;

          num = wpa_get_network_num(wifidev);
          if (num < 1)
            {
              network_id = wpa_add_network(wifidev);
            }
          else
            {
              network_id = wpa_get_last_network_id(wifidev, num);
            }

          wifidev->network_id = network_id;
        }
      break;
    case IW_MODE_MASTER:

      /* Start the hostapd. */

      ret = host_system(NULL, 0,
                        "/usr/bin/sudo "SIMWIFI_FILE" start_ap %s",
                        wifidev->host_ifname);

      break;
    default:
      ret = -ENOTTY;
      break;
    }

  return ret;
}

static int wifidriver_set_country(struct sim_netdev_s *wifidev,
                               struct iwreq *pwrq)
{
  char country[4] =
    {
      0
    };

  if (wifidev->mode != IW_MODE_INFRA)
    {
      return OK;
    }

  memcpy(country, pwrq->u.data.pointer, pwrq->u.data.length);

  ninfo("set country is %s\n", country);

  return set_cmd(wifidev, "set country %s", country);
}

static int wifidriver_get_country(struct sim_netdev_s *wifidev,
                               struct iwreq *pwrq)
{
  char country[128];
  int ret;

  if (wifidev->mode != IW_MODE_INFRA)
    {
      return OK;
    }

  memset(country, 0, sizeof(country));

  ret = host_system(country, sizeof(country),
                    "%s %s -i%s %s", WPA_CLI, WPA_CTRL_PATH,
                    wifidev->host_ifname, "get country");

  if (ret <= 0)
    {
      nerr("get country NULL.");
      return -ENODATA;
    }

  if (strncmp(country, "FAIL", 4))
    {
      memcpy(pwrq->u.data.pointer, country, 2);
      ((uint8_t *)pwrq->u.data.pointer)[2] = '\0';
      ret = 0;
    }
  else
    {
      nerr("get country FAILED.");
      ret = -ENODATA;
    }

  return ret;
}

int wifidriver_set_freq(struct sim_netdev_s *wifidev, struct iwreq *pwrq)
{
  int channel;
  int ret = 0;

  switch (wifidev->mode)
    {
    case IW_MODE_INFRA:
      break;
    case IW_MODE_MASTER:
      channel = freq_to_channel(pwrq->u.freq.m);
      if (channel > 0)
        {
          ret = set_cmd(wifidev, "set channel %d", channel);
          ret = set_cmd(wifidev, "disable");
          ret = set_cmd(wifidev, "enable");
        }
      break;
    default:
      break;
    }

  return ret;
}

static int wifidriver_get_freq(struct sim_netdev_s *wifidev,
                               struct iwreq *pwrq)
{
  switch (wifidev->mode)
    {
    case IW_MODE_INFRA:
      if (get_wpa_state(wifidev))
        {
          pwrq->u.freq.flags = IW_FREQ_FIXED;
          pwrq->u.freq.e     = 0;
          pwrq->u.freq.m     = get_wpa_freq(wifidev);
        }
      else
        {
          pwrq->u.freq.flags = IW_FREQ_AUTO;
          pwrq->u.freq.e     = 0;
          pwrq->u.freq.m     = 2412;
        }
      break;
    case IW_MODE_MASTER:
      break;
    default:
      break;
    }

  return OK;
}

static int wifidriver_get_range(struct sim_netdev_s *wifidev,
                             struct iwreq *pwrq)
{
  int k;
  struct iw_range *range = (struct iw_range *)pwrq->u.data.pointer;

  /* default in china. */

  range->num_frequency = 13;
  for (k = 1; k <= range->num_frequency; k++)
    {
      range->freq[k - 1].i = k;
      range->freq[k - 1].e = 0;
      range->freq[k - 1].m = 2407 + 5 * k;
    }

  return OK;
}

static int wifidriver_connect(struct netdev_lowerhalf_s *dev)
{
  int ret;

  ret = wifidriver_start_connect((struct sim_netdev_s *)dev);
  if (ret >= 0)
    {
      netdev_lower_carrier_on(dev);
    }

  return ret;
}

static int wifidriver_disconnect(struct netdev_lowerhalf_s *dev)
{
  int ret;

  ret = wifidriver_start_disconnect((struct sim_netdev_s *)dev);
  if (ret >= 0)
    {
      netdev_lower_carrier_off(dev);
    }

  return ret;
}

static int wifidriver_essid(struct netdev_lowerhalf_s *dev,
                            struct iwreq *iwr, bool set)
{
  struct sim_netdev_s *wifidev = (struct sim_netdev_s *)dev;

  if (set)
    {
      return wifidriver_set_essid(wifidev, iwr);
    }
  else
    {
      return wifidriver_get_essid(wifidev, iwr);
    }
}

static int wifidriver_bssid(struct netdev_lowerhalf_s *dev,
                            struct iwreq *iwr, bool set)
{
  int ret;
  struct sim_netdev_s *wifidev = (struct sim_netdev_s *)dev;

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
      return -ENOTTY;
    }

  return ret;
}

static int wifidriver_passwd(struct netdev_lowerhalf_s *dev,
                             struct iwreq *iwr, bool set)
{
  if (set)
    {
      return wifidriver_set_psk((struct sim_netdev_s *)dev, iwr);
    }
  else
    {
      return -ENOTTY;
    }
}

static int wifidriver_mode(struct netdev_lowerhalf_s *dev,
                           struct iwreq *iwr, bool set)
{
  struct sim_netdev_s *wifidev = (struct sim_netdev_s *)dev;

  if (set)
    {
      return wifidriver_set_mode(wifidev, iwr);
    }
  else
    {
      return wifidriver_get_mode(wifidev, iwr);
    }
}

static int wifidriver_auth(struct netdev_lowerhalf_s *dev,
                           struct iwreq *iwr, bool set)
{
  struct sim_netdev_s *wifidev = (struct sim_netdev_s *)dev;

  if (set)
    {
      return wifidriver_set_auth(wifidev, iwr);
    }
  else
    {
      return wifidriver_get_auth(wifidev, iwr);
    }
}

static int wifidriver_freq(struct netdev_lowerhalf_s *dev,
                           struct iwreq *iwr, bool set)
{
  struct sim_netdev_s *wifidev = (struct sim_netdev_s *)dev;

  if (set)
    {
      return wifidriver_set_freq(wifidev, iwr);
    }
  else
    {
      return wifidriver_get_freq(wifidev, iwr);
    }
}

static int wifidriver_bitrate(struct netdev_lowerhalf_s *dev,
                              struct iwreq *iwr, bool set)
{
  return -ENOTTY;
}

static int wifidriver_txpower(struct netdev_lowerhalf_s *dev,
                              struct iwreq *iwr, bool set)
{
  return -ENOTTY;
}

static int wifidriver_country(struct netdev_lowerhalf_s *dev,
                              struct iwreq *iwr, bool set)
{
  struct sim_netdev_s *wifidev = (struct sim_netdev_s *)dev;

  if (set)
    {
      return wifidriver_set_country(wifidev, iwr);
    }
  else
    {
      return wifidriver_get_country(wifidev, iwr);
    }
}

static int wifidriver_sensitivity(struct netdev_lowerhalf_s *dev,
                                  struct iwreq *iwr, bool set)
{
  return -ENOTTY;
}

static int wifidriver_scan(struct netdev_lowerhalf_s *dev,
                           struct iwreq *iwr, bool set)
{
  struct sim_netdev_s *wifidev = (struct sim_netdev_s *)dev;

  if (set)
    {
      return wifidriver_start_scan(wifidev, iwr);
    }
  else
    {
      return wifidriver_scan_result(wifidev, iwr);
    }
}

static int wifidriver_range(struct netdev_lowerhalf_s *dev,
                            struct iwreq *iwr)
{
  return wifidriver_get_range((struct sim_netdev_s *)dev, iwr);
}

static bool wifidriver_connected(struct netdev_lowerhalf_s *dev)
{
  struct sim_netdev_s *wifidev = (struct sim_netdev_s *)dev;

  if (wifidev->mode == IW_MODE_MASTER)
    {
      return true;
    }
  else if (wifidev->mode == IW_MODE_INFRA)
    {
      return get_wpa_state(wifidev);
    }

  return false;
}

static void wifidriver_init(struct netdev_lowerhalf_s *dev, int devidx)
{
  struct sim_netdev_s *wifidev = (struct sim_netdev_s *)dev;

  wifidev->mode = IW_MODE_AUTO;
  wifidev->devidx = devidx;

  /* The default host wlan interface name is corresponding to the nuttx
   * wlan interface name. If not, should modify the host wlan interface
   * name.
   */

  snprintf(wifidev->host_ifname, IFNAMSIZ, "wlan%d", devidx);

  /* Bind the wireless ops interfaces. */

  dev->iw_ops = &g_iw_ops;
}
