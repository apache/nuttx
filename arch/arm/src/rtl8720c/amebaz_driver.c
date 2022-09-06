/****************************************************************************
 * arch/arm/src/rtl8720c/amebaz_driver.c
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
#include <stdio.h>
#include <nuttx/kmalloc.h>
#include <nuttx/net/arp.h>
#include "amebaz_netdev.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AMEBAZ_SCAN_TIMEOUT_TICK    (10 * CLOCKS_PER_SEC)
#define AMEBAZ_CONNECT_TIMEOUT_TICK (10 * CLOCKS_PER_SEC)
#define AMEBAZ_DEVICE_COUNT         (2)
#define AMEBAZ_SCAN_ENTRY_COUNT     (5)
#define AMEBAZ_DEFAULT_COUNTRY      "CN"
#define SIOCDEVPRIVATE              0x89f0
#define SIOCGIWPRIVPASSPHRASE       0x8bfc
#define SIOCSIWPRIVCOUNTRY          0x8bfd
#define SIOCSIWPRIVAPESSID          0x8bfe
#define SIOCSIWPRIVPASSPHRASE       0x8bff
struct _sockaddr_t
{
  uint8_t sa_len;
  uint8_t sa_family;
  char sa_data[14];
};

static struct amebaz_dev_s *gp_wlan_dev[AMEBAZ_DEVICE_COUNT + 1];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

static void amebaz_state_timeout(wdparm_t arg)
{
  struct amebaz_state_s *state = (struct amebaz_state_s *)arg;
  if (state->status < AMEBAZ_STATUS_RUN)
    {
      return;
    }

  state->status = AMEBAZ_STATUS_TIMEOUT;
  nxsem_post(&state->sem);
}

static int amebaz_state_run(struct amebaz_state_s *state, int32_t delay)
{
  if (state->status == AMEBAZ_STATUS_RUN)
    {
      return -EBUSY;
    }

  state->status = AMEBAZ_STATUS_RUN;
  return wd_start(&state->timeout, delay,
                  amebaz_state_timeout, (wdparm_t)state);
}

static int amebaz_state_wait(struct amebaz_state_s *state)
{
  int ret = 0;
  while (state->status == AMEBAZ_STATUS_RUN)
    {
      ret = nxsem_wait_uninterruptible(&state->sem);
      if (ret != 0)
        {
          break;
        }
    }

  return ret;
}

static void amebaz_state_post(struct amebaz_state_s *state, int status)
{
  int _status = state->status;
  state->status = status;
  if (_status == AMEBAZ_STATUS_RUN)
    {
      wd_cancel(&state->timeout);
      nxsem_post(&state->sem);
    }
}

static void amebaz_state_deinit(struct amebaz_state_s *state)
{
  wd_cancel(&state->timeout);
}

static int amebaz_state_init(struct amebaz_state_s *state)
{
  if (nxsem_init(&state->sem, 0, 0) != OK)
    {
      return -ENOMEM;
    }

  state->status = AMEBAZ_STATUS_DISABLED;
  return 0;
}

void amebaz_wl_scan_handler(int index, union iwreq_data *wrqu, char *extra)
{
  struct amebaz_dev_s *priv = gp_wlan_dev[index];
  struct amebaz_state_s *state = &priv->scan;
  rtw_scan_result_t *res, *cache, *tmp;
  rtw_scan_result_t ap_res;
  rtw_scan_ap_result_t *ap;
  struct iwreq iwr;
  int i;
  int j;
  if (state->status != AMEBAZ_STATUS_RUN)
    {
      return;
    }

  if (wrqu->data.pointer == NULL)
    {
      memset(&iwr, 0, sizeof(iwr));
      snprintf(iwr.ifr_name, sizeof(iwr.ifr_name), "wlan%d", index);
      iwr.u.data.pointer = priv->scan_data;
      iwr.u.data.length = AMEBAZ_SCAN_AP_COUNT * sizeof(rtw_scan_result_t);
      if (rltk_wlan_control(SIOCGIWSCAN, &iwr) != 0 ||
          iwr.u.data.flags != 1)
        {
          amebaz_state_post(state, AMEBAZ_STATUS_DONE);
          return;
        }

      ap = iwr.u.data.pointer;
      ap_res.signal_strength  = ap->signal_strength;
      ap_res.bss_type         = 0;
      ap_res.security         = ap->security;
      ap_res.wps_type         = ap->wps_type;
      ap_res.channel          = ap->channel;
      ap_res.SSID.len         = ap->len - sizeof(*ap) + 1;
      memcpy(ap_res.SSID.val, ap->SSID, ap_res.SSID.len);
      memcpy(ap_res.BSSID.octet, ap->BSSID, IFHWADDRLEN);
      ap_res.SSID.val[ap_res.SSID.len - 1]  = '\0';
      memcpy(priv->scan_data, &ap_res, sizeof(ap_res));
      priv->scan_count = 1;
      amebaz_state_post(state, AMEBAZ_STATUS_DONE);
      return;
    }

  for (i = 0; i < wrqu->data.length / sizeof(rtw_scan_result_t); i++)
    {
      res = ((rtw_scan_result_t **)wrqu->data.pointer)[i];
      for (j = 0; j < priv->scan_count; j++)
        {
          cache = &priv->scan_data[j];
          if (memcmp(cache->BSSID.octet, res->BSSID.octet, IFHWADDRLEN))
            {
              continue;
            }

          if (cache->signal_strength < res->signal_strength)
            {
              memcpy(cache, res, sizeof(rtw_scan_result_t));
            }

          break;
        }

      if (j != priv->scan_count)
        {
          continue;
        }

      if (priv->scan_count >= AMEBAZ_SCAN_AP_COUNT)
        {
          tmp = NULL;
          for (j = 0; j < priv->scan_count; j++)
            {
              cache = &priv->scan_data[j];
              if (!tmp || cache->signal_strength < tmp->signal_strength)
                {
                  tmp = cache;
                }
            }

          memcpy(tmp, res, sizeof(rtw_scan_result_t));
        }

      else
        {
          cache = &priv->scan_data[priv->scan_count++];
          memcpy(cache, res, sizeof(rtw_scan_result_t));
        }
    }
}

static void amebaz_wl_post_connection_event(struct amebaz_dev_s *priv,
    int status)
{
  struct amebaz_state_s *state = &priv->conn;
  if (status == AMEBAZ_STATUS_DONE)
    {
      netdev_carrier_on(&priv->dev);
    }

  else
    {
      netdev_carrier_off(&priv->dev);
    }

  amebaz_state_post(state, status);
}

void amebaz_wl_connection_handler(int index,
                                  union iwreq_data *wrqu, char *extra)
{
  const unsigned char fourway_done[] = "WPA/WPA2 handshake done";
  const unsigned char no_assoc_network[] =
    "No Assoc Network After Scan Done";
  struct amebaz_dev_s *priv = gp_wlan_dev[0];
  struct amebaz_state_s *state = &priv->conn;
  unsigned char null_mac[IFHWADDRLEN] =
  {
  };

  bool mac_avalid;
  if (gp_wlan_dev[1]->conn.status == AMEBAZ_STATUS_RUN)
    {
      priv = gp_wlan_dev[1];
    }

  mac_avalid = memcmp(wrqu->ap_addr.sa_data, null_mac, sizeof(null_mac));
  if (extra)
    {
      if (!memcmp(fourway_done, extra, sizeof(fourway_done)))
        {
          amebaz_wl_post_connection_event(priv, AMEBAZ_STATUS_DONE);
        }

      else if (!memcmp(no_assoc_network, extra, sizeof(no_assoc_network)) ||
               !mac_avalid)
        {
          amebaz_wl_post_connection_event(priv, AMEBAZ_STATUS_DISABLED);
        }
    }

  else if (priv->assoc.alg == IW_ENCODE_ALG_NONE ||
           priv->mode == RTW_MODE_AP || priv->mode == RTW_MODE_STA_AP)
    {
      if (mac_avalid)
        {
          amebaz_wl_post_connection_event(priv, AMEBAZ_STATUS_DONE);
        }

      else
        {
          amebaz_wl_post_connection_event(priv, AMEBAZ_STATUS_DISABLED);
        }
    }

  else if (state->status == AMEBAZ_STATUS_DONE && !mac_avalid)
    {
      amebaz_wl_post_connection_event(priv, AMEBAZ_STATUS_DISABLED);
    }
}

void amebaz_wl_netif_info_handler(int index, void *dev, unsigned char *addr)
{
  struct amebaz_dev_s *priv = gp_wlan_dev[index];
  if (!priv || index != priv->devnum)
    {
      return;
    }

  memcpy(priv->dev.d_mac.ether.ether_addr_octet, addr, IFHWADDRLEN);
}

void amebaz_wl_notify_rx_handler(int index, unsigned int len)
{
  struct amebaz_dev_s *priv = gp_wlan_dev[index];
  if (!priv || index != priv->devnum || !len)
    {
      return;
    }

  amebaz_netdev_notify_receive(priv, index, len);
}

static int amebaz_wl_set_channels(struct amebaz_dev_s *priv,
                                  struct iw_freq *freqs,
                                  uint8_t num_channels)
{
  uint8_t param[12 + 1 + 15 * 2] =
  {
    "PartialScan"
  };

  struct iwreq iwr =
  {
  };

  int i;
  if (!freqs || num_channels == 0)
    {
      return OK;
    }

  if (num_channels > 15)
    {
      return -EINVAL;
    }

  *(param + 12) = num_channels;
  for (i = 0; i < num_channels; i++)
    {
      *(param + 13 + i) = freqs[i].m;
      *(param + 13 + num_channels + i) = PSCAN_ENABLE;
    }

  iwr.u.data.pointer = param;
  iwr.u.data.length  = 12 + 1 + num_channels * 2;
  snprintf(iwr.ifr_name, IFNAMSIZ, "wlan%d", priv->devnum);
  return rltk_wlan_control(SIOCDEVPRIVATE, &iwr);
}

int amebaz_wl_start_scan(struct amebaz_dev_s *priv, struct iwreq *iwr)
{
  enum
    {
      RTW_SCAN_COMMAMD = 0x01
    };

  struct amebaz_state_s *state = &priv->scan;
  int scan_type = IW_SCAN_TYPE_ACTIVE;
  int bss_type = RTW_BSS_TYPE_ANY;
  struct iw_scan_req *req;
  void *pointer = NULL;
  uint16_t length;
  int essid_len;
  int ret;
  iwr->u.data.flags = scan_type | (bss_type << 8);
  if (iwr->u.data.pointer != NULL &&
      iwr->u.data.length == sizeof(*req))
    {
      req = iwr->u.data.pointer;
      ret = amebaz_wl_set_channels(priv,
                                   req->channel_list, req->num_channels);
      if (ret < 0)
        {
          return ret;
        }

      essid_len = req->essid_len;
      memcpy(priv->scan_data, &essid_len, sizeof(int));
      memcpy((char *)priv->scan_data + sizeof(int),
             req->essid, req->essid_len);
      pointer = iwr->u.data.pointer;
      length = iwr->u.data.length;
      iwr->u.data.pointer = priv->scan_data;
      iwr->u.data.length = AMEBAZ_SCAN_AP_COUNT * sizeof(rtw_scan_result_t);
    }

  else
    {
      iwr->u.data.flags |= RTW_SCAN_COMMAMD << 4;
    }

  priv->scan_count = 0;
  ret = amebaz_state_run(state, AMEBAZ_SCAN_TIMEOUT_TICK);
  if (ret < 0)
    {
      return ret;
    }

  ret = rltk_wlan_control(SIOCSIWSCAN, iwr);
  if (pointer)
    {
      iwr->u.data.pointer = pointer;
      iwr->u.data.length = length;
    }

  if (ret < 0)
    {
      state->status = AMEBAZ_STATUS_DISABLED;
    }

  return ret;
}

static char *amebaz_wl_iwe_add_event(char *stream, char *stop,
                                     struct iw_event *iwe, int event_len)
{
  if (stream + event_len > stop)
    {
      return stream;
    }

  iwe->len = event_len;
  return stream + event_len;
}

static int amebaz_wl_format_scan_results(struct amebaz_dev_s *priv,
    struct iwreq *iwr)
{
  rtw_scan_result_t *cache;
  struct iw_event *iwe;
  char *start;
  char *stop;
  int i;
  start = iwr->u.data.pointer;
  stop = (char *)iwr->u.data.pointer + iwr->u.data.length;
  for (i = 0; i < priv->scan_count; i++)
    {
      cache = &priv->scan_data[i];
      iwe = (struct iw_event *)start;
      iwe->cmd = SIOCGIWAP;
      iwe->u.ap_addr.sa_family = ARPHRD_ETHER;
      memcpy(&iwe->u.ap_addr.sa_data, cache->BSSID.octet, IFHWADDRLEN);
      start = amebaz_wl_iwe_add_event(start, stop, iwe, IW_EV_LEN(ap_addr));
      iwe = (struct iw_event *)start;
      iwe->cmd = SIOCGIWESSID;
      iwe->u.essid.flags = 0;
      iwe->u.essid.length = cache->SSID.len;
      iwe->u.essid.pointer = (void *)sizeof(iwe->u.essid);
      memcpy(&iwe->u.essid + 1, cache->SSID.val, cache->SSID.len);
      start = amebaz_wl_iwe_add_event(start, stop, iwe,
                IW_EV_LEN(essid) + ((cache->SSID.len + 3) & -4));
      iwe = (struct iw_event *)start;
      iwe->cmd = IWEVQUAL;
      iwe->u.qual.qual = 0;
      iwe->u.qual.level = cache->signal_strength;
      iwe->u.qual.noise = 0;
      iwe->u.qual.updated |= IW_QUAL_DBM;
      start = amebaz_wl_iwe_add_event(start, stop, iwe, IW_EV_LEN(qual));
      iwe = (struct iw_event *)start;
      iwe->cmd = SIOCGIWFREQ;
      iwe->u.freq.e = 0;
      iwe->u.freq.m = cache->channel;
      start = amebaz_wl_iwe_add_event(start, stop, iwe, IW_EV_LEN(freq));
      iwe = (struct iw_event *)start;
      iwe->cmd = SIOCGIWENCODE;
      iwe->u.data.flags = IW_ENCODE_DISABLED;
      iwe->u.data.length = 0;
      iwe->u.essid.pointer = NULL;
      start = amebaz_wl_iwe_add_event(start, stop, iwe, IW_EV_LEN(data));
    }

  return start - (char *)iwr->u.data.pointer;
}

int amebaz_wl_get_scan_results(struct amebaz_dev_s *priv, struct iwreq *iwr)
{
  struct amebaz_state_s *state = &priv->scan;
  int request_size;
  int ret = OK;
  if (state->status == AMEBAZ_STATUS_RUN)
    {
      ret = -EAGAIN;
      goto exit_failed;
    }

  if (state->status != AMEBAZ_STATUS_DONE)
    {
      ret = -EINVAL;
      goto exit_failed;
    }

  if ((ret = amebaz_state_wait(state)) < 0)
    {
      goto exit_failed;
    }

  if (priv->scan_count <= 0)
    {
      ret = OK;
      iwr->u.data.length = 0;
      goto exit_sem_post;
    }

  request_size = priv->scan_count *
                 AMEBAZ_SCAN_ENTRY_COUNT *
                 sizeof(struct iw_event);
  if (iwr->u.data.pointer == NULL ||
      iwr->u.data.length < request_size)
    {
      ret = -E2BIG;
      iwr->u.data.pointer = NULL;
      iwr->u.data.length = request_size;
      goto exit_sem_post;
    }

  iwr->u.data.length = amebaz_wl_format_scan_results(priv, iwr);
exit_sem_post:
  nxsem_post(&state->sem);
exit_failed:
  if (ret < 0)
    {
      iwr->u.data.length = 0;
    }

  return ret;
}

int amebaz_wl_process_command(struct amebaz_dev_s *priv, int cmd, void *req)
{
  return rltk_wlan_control(cmd, req);
}

int amebaz_wl_set_encode_ext(struct amebaz_dev_s *priv, struct iwreq *iwr)
{
  struct iw_encode_ext *ext;
  struct iwreq _iwr =
  {
  };

  int ret;
  iwr->u.encoding.flags = IW_ENCODE_INDEX & 1;
  iwr->u.encoding.flags |= IW_ENCODE_TEMP;
  ret = rltk_wlan_control(SIOCSIWENCODEEXT, iwr);
  if (ret < 0)
    {
      return ret;
    }

  ext = iwr->u.encoding.pointer;
  _iwr.u.data.pointer = (void *)ext->key;
  _iwr.u.data.length = ext->key_len;
  _iwr.u.data.flags = (ext->key_len != 0);
  memcpy(_iwr.ifr_name, iwr->ifr_name, strlen(iwr->ifr_name));
  ret = rltk_wlan_control(SIOCSIWPRIVPASSPHRASE, &_iwr);
  if (ret < 0)
    {
      return ret;
    }

  priv->assoc.alg = ext->alg;
  priv->assoc.mask |= AMEBAZ_ASSOCIATE_ALG;
  return OK;
}

int amebaz_wl_get_encode_ext(struct amebaz_dev_s *priv, struct iwreq *iwr)
{
  struct iw_encode_ext *ext;
  struct iwreq _iwr =
  {
  };

  int ret;
  ret = rltk_wlan_control(SIOCGIWENCODEEXT, iwr);
  if (ret < 0)
    {
      return ret;
    }

  ext = iwr->u.encoding.pointer;
  _iwr.u.data.pointer = (void *)ext->key;
  memcpy(_iwr.ifr_name, iwr->ifr_name, strlen(iwr->ifr_name));
  ret = rltk_wlan_control(SIOCGIWPRIVPASSPHRASE, &_iwr);
  if (ret < 0)
    {
      return ret;
    }

  ext->key_len = _iwr.u.data.length;
  ext->key[ext->key_len] = '\0';
  return ret;
}

static int amebaz_wl_add_custom_ie(struct amebaz_dev_s *priv,
                                   int devnum)
{
  struct rtw_custom_ie_t
  {
    uint8_t *ie;
    uint8_t type;
  };

  enum
    {
      PROBE_REQ = 0x0001,
      PROBE_RSP = 0x0002,
      BEACON    = 0x0004,
      ASSOC_REQ = 0x0008,
    };

  uint8_t ie[8] =
  {
    7, 6, 'C', 'N', '\0', 1, 13, 20
  };

  struct rtw_custom_ie_t cie =
  {
    ie, PROBE_RSP | BEACON
  };

  char param[16] = "SetCusIE";
  uint32_t *cmd = (uint32_t *)(param + strlen(param) + 1);
  struct iwreq iwr =
  {
  };

  memcpy(&ie[2], priv->country, 2);
  *cmd = (intptr_t)&cie;
  *(cmd + 1) = (intptr_t)1;
  iwr.u.data.pointer = param;
  iwr.u.data.length = sizeof(param);
  snprintf(iwr.ifr_name, IFNAMSIZ, "wlan%d", devnum);
  return rltk_wlan_control(SIOCDEVPRIVATE, &iwr);
}

int amebaz_wl_associate(struct amebaz_dev_s *priv, struct iwreq *iwr)
{
  struct amebaz_state_s *state = &priv->conn;
  rtw_network_info_t info =
    {
    };

  rtw_network_info_t *pinfo;
  struct iw_freq freq =
  {
  };

  struct _sockaddr_t *addr;
  bool bssid_conn = false;
  struct iwreq _iwr =
  {
  };

  char *data;
  int ret;
  if (priv->mode == RTW_MODE_STA)
    {
      if (priv->assoc.mask !=
          (AMEBAZ_ASSOCIATE_MASK & ~AMEBAZ_ASSOCIATE_ALG) &&
          priv->assoc.mask != AMEBAZ_ASSOCIATE_MASK)
        {
          return 0;
        }

      if ((priv->assoc.mask & AMEBAZ_ASSOCIATE_ALG) == 0)
        {
          priv->assoc.alg = IW_ENCODE_ALG_NONE;
        }

      snprintf(_iwr.ifr_name, IFNAMSIZ, "wlan%d", priv->devnum);
      addr = (struct _sockaddr_t *)&_iwr.u.ap_addr;
      addr->sa_family = ARPHRD_ETHER;
      data = addr->sa_data;
      memcpy(_iwr.u.ap_addr.sa_data, priv->assoc.mac.octet, IFHWADDRLEN);
      freq.m = priv->assoc.channel;
      if (amebaz_wl_set_channels(priv, &freq, 1) < 0)
        {
          return -EINVAL;
        }

      data[IFHWADDRLEN] = '#';
      data[IFHWADDRLEN + 1] = '@';
      info.ssid.len = priv->assoc.ssid.len;
      memcpy(info.ssid.val, priv->assoc.ssid.val, info.ssid.len);
      memcpy(info.bssid.octet, priv->assoc.mac.octet, IFHWADDRLEN);
      switch (priv->assoc.alg)
        {
        case IW_ENCODE_ALG_NONE:
          info.security_type = RTW_SECURITY_OPEN;
          break;
        case IW_ENCODE_ALG_WEP:
          info.security_type = RTW_SECURITY_WEP_PSK;
          break;
        case IW_ENCODE_ALG_CCMP:
        default:
          info.security_type = RTW_SECURITY_WPA2_MIXED_PSK;
        }

      pinfo = &info;
      memcpy(data + (IFHWADDRLEN + 2), &pinfo, sizeof(pinfo));
      bssid_conn = true;
    }

  ret = amebaz_state_run(state, AMEBAZ_CONNECT_TIMEOUT_TICK);
  if (ret < 0)
    {
      return ret;
    }

  if ((priv->mode == RTW_MODE_AP && priv->devnum == 0) ||
      (priv->mode == RTW_MODE_STA_AP && priv->devnum == 1))
    {
      ret = amebaz_wl_add_custom_ie(priv, priv->devnum);
      if (ret < 0)
        {
          wlwarn("unable to update the custom ie\n");
        }
    }

  if (priv->mode == RTW_MODE_AP || priv->mode == RTW_MODE_STA_AP)
    {
      ret = rltk_wlan_control(SIOCSIWPRIVAPESSID, iwr);
    }

  else if (priv->mode == RTW_MODE_STA)
    {
      if (bssid_conn)
        {
          ret = rltk_wlan_control(SIOCSIWAP, &_iwr);
        }

      else
        {
          ret = rltk_wlan_control(SIOCSIWESSID, iwr);
        }
    }

  else
    {
      ret = -EINVAL;
    }

  if (ret < 0)
    {
      goto exit_failed;
    }

  if ((ret = amebaz_state_wait(state)) < 0)
    {
      goto exit_failed;
    }

  if (state->status == AMEBAZ_STATUS_TIMEOUT)
    {
      ret = -ETIME;
    }

  else if (state->status != AMEBAZ_STATUS_DONE)
    {
      ret = -ENXIO;
    }

exit_failed:
  if (ret < 0)
    {
      amebaz_state_post(state, AMEBAZ_STATUS_DISABLED);
      priv->assoc.alg = IW_ENCODE_ALG_NONE;
    }

  else
    {
      state->status = AMEBAZ_STATUS_DONE;
    }

  priv->assoc.mask = 0;
  return ret;
}

int amebaz_wl_set_ssid(struct amebaz_dev_s *priv, struct iwreq *iwr)
{
  if (!iwr->u.essid.flags)
    {
      return rltk_wlan_control(SIOCSIWESSID, iwr);
    }

  memcpy(priv->assoc.ssid.val, iwr->u.essid.pointer, iwr->u.essid.length);
  priv->assoc.ssid.len = iwr->u.essid.length;
  priv->assoc.mask |= AMEBAZ_ASSOCIATE_SSID;
  return amebaz_wl_associate(priv, iwr);
}

int amebaz_wl_set_bssid(struct amebaz_dev_s *priv, struct iwreq *iwr)
{
  struct _sockaddr_t *addr = (struct _sockaddr_t *)&iwr->u.ap_addr;
  unsigned char null_mac[IFHWADDRLEN] =
  {
  };

  char *data;
  addr->sa_family = ARPHRD_ETHER;
  data = addr->sa_data;
  if (!memcmp(data, null_mac, sizeof(null_mac)))
    {
      /* set MAC address last byte to 1
       * since driver will filter the mac with all 0x00 or 0xff
       */

      data[IFHWADDRLEN - 1] = 1;
    }

  else
    {
      memcpy(priv->assoc.mac.octet, data, IFHWADDRLEN);
      priv->assoc.mask |= AMEBAZ_ASSOCIATE_BSSID;
      return amebaz_wl_associate(priv, iwr);
    }

  return rltk_wlan_control(SIOCSIWAP, iwr);
}

static int amebaz_wl_disable_powersave(int devnum)
{
  char control[7 + 6] = "pm_set";
  struct iwreq iwr =
  {
  };

  snprintf(iwr.ifr_name, IFNAMSIZ, "wlan%d", devnum);
  /* Command format:
   *  Entry[0...n]:
   *     1. Type
   *     2. Set/Unset
   *     3. Value
   */

  enum
    {
      AMEBA_PMSET_MODE_IPS,
      AMEBA_PMSET_MODE_LPS,
      AMEBA_PMSET_MODE_TDMA,
      AMEBA_PMSET_MODE_DTIM,
      AMEBA_PMSET_MODE_BEACON,
      AMEBA_PMSET_MODE_LPS_LEVEL,
      AMEBA_PMSET_MODE_LPS_THRESHOLD,
      AMEBA_PMSET_MODE_LPS_RF,
      AMEBA_PMSET_MODE_RESUME,
    };

  control[7]  = AMEBA_PMSET_MODE_IPS;
  control[8]  = 1;
  control[9]  = 0;
  control[10] = AMEBA_PMSET_MODE_LPS;
  control[11] = 1;
  control[12] = 0;
  iwr.u.data.pointer = control;
  iwr.u.data.length = sizeof(control);
  return rltk_wlan_control(SIOCDEVPRIVATE, &iwr);
}

int amebaz_wl_set_mode(struct amebaz_dev_s *priv, struct iwreq *iwr)
{
  int mode;
  int ret;
  int count;
  int i;
  if (priv->devnum == 1)
    {
      if (iwr->u.mode == IW_MODE_MASTER)
        {
          mode = RTW_MODE_STA_AP;
        }

      else
        {
          return -EINVAL;
        }
    }

  else
    {
      if (rltk_wlan_running(1))
        {
          mode = RTW_MODE_STA;
          ret = 0;
          goto errout;
        }

      if (iwr->u.mode == IW_MODE_MASTER)
        {
          mode = RTW_MODE_AP;
        }

      else if (iwr->u.mode == IW_MODE_INFRA)
        {
          mode = RTW_MODE_STA;
        }

      else
        {
          return -EINVAL;
        }
    }

  if (priv->mode == mode)
    {
      return OK;
    }

  if (priv->mode != RTW_MODE_NONE)
    {
      if (priv->mode == RTW_MODE_STA && mode == RTW_MODE_AP)
        {
          if (priv->conn.status == AMEBAZ_STATUS_DONE)
            {
              return -EINVAL;
            }
        }

      else if (priv->mode == RTW_MODE_AP && mode == RTW_MODE_STA)
        {
          return -EINVAL;
        }
    }

  if (priv->mode == RTW_MODE_NONE &&
      rltk_wlan_running(priv->devnum) == false)
    {
      if (priv->devnum == 1 && rltk_wlan_running(0) == true)
        {
          ret = rltk_set_mode_prehandle(RTW_MODE_STA,
                                        RTW_MODE_STA_AP, "wlan0");
          if (ret < 0)
            {
              goto errout;
            }

          rtw_msleep_os(50);
          ret = rltk_set_mode_posthandle(RTW_MODE_STA,
                                         RTW_MODE_STA_AP, "wlan0");
          if (ret < 0)
            {
              goto errout;
            }
        }

      else
        {
          count = (mode == RTW_MODE_STA_AP) ? AMEBAZ_DEVICE_COUNT : 1;
          for (i = 0; i < count; i++)
            {
              ret = rltk_wlan_init(i, mode);
              if (ret < 0)
                {
                  goto errout;
                }

              extern void up_irq_attach_workaround(void);
              up_irq_attach_workaround();
            }

          for (i = 0; i < count; i++)
            {
              ret = rltk_wlan_start(i);
              if (ret < 0)
                {
                  goto errout;
                }
            }

          while (!rltk_wlan_running(priv->devnum))
            {
              usleep(1000);
            }

          ret = amebaz_wl_disable_powersave(0);
          if (ret < 0)
            {
              goto errout;
            }
        }
    }

  ret = rltk_wlan_control(SIOCSIWMODE, iwr);
errout:
  if (ret)
    {
      rltk_wlan_deinit();
      mode = RTW_MODE_NONE;
    }

  priv->mode = mode;
  return ret;
}

int amebaz_wl_set_country(struct amebaz_dev_s *priv, struct iwreq *iwr)
{
  const char *country = iwr->u.essid.pointer;
  int32_t cc = RTW_COUNTRY_WORLD1;
  int ret;
  if (!strncmp(country, "CN", 2))
    {
    }

  else if (!strncmp(country, "CA", 2))
    {
      cc = RTW_COUNTRY_CA;
    }

  else if (!strncmp(country, "CO", 2))
    {
      cc = RTW_COUNTRY_CO;
    }

  else if (!strncmp(country, "DO", 2))
    {
      cc = RTW_COUNTRY_DO;
    }

  else if (!strncmp(country, "GT", 2))
    {
      cc = RTW_COUNTRY_GT;
    }

  else if (!strncmp(country, "MX", 2))
    {
      cc = RTW_COUNTRY_MX;
    }

  else if (!strncmp(country, "PA", 2))
    {
      cc = RTW_COUNTRY_PA;
    }

  else if (!strncmp(country, "PR", 2))
    {
      cc = RTW_COUNTRY_PR;
    }

  else if (!strncmp(country, "US", 2))
    {
      cc = RTW_COUNTRY_US;
    }

  else if (!strncmp(country, "TW", 2))
    {
      cc = RTW_COUNTRY_TW;
    }

  else if (!strncmp(country, "JP", 2))
    {
      cc = RTW_COUNTRY_JP;
    }

  else if (!strncmp(country, "IL", 2))
    {
      cc = RTW_COUNTRY_IL;
    }

  iwr->u.essid.pointer = NULL;
  iwr->u.param.value = cc;
  ret = rltk_wlan_control(SIOCSIWPRIVCOUNTRY, iwr);
  if (ret == 0)
    {
      memcpy(priv->country, country, 2);
    }

  iwr->u.essid.pointer = (void *)country;
  return ret;
}

int amebaz_wl_set_freq(struct amebaz_dev_s *priv, struct iwreq *iwr)
{
  priv->assoc.channel = iwr->u.freq.m;
  priv->assoc.mask |= AMEBAZ_ASSOCIATE_CHANNEL;
  return 0;
}

int amebaz_wl_get_freq(struct amebaz_dev_s *priv, struct iwreq *iwr)
{
  int ret;
  ret = rltk_wlan_control(SIOCGIWFREQ, iwr);
  if (ret == 0)
    {
      iwr->u.freq.m = iwr->u.freq.i;
    }

  return ret;
}

static struct amebaz_dev_s *amebaz_allocate_device(int devnum)
{
  struct amebaz_dev_s *priv;
  int ret;
  priv = (struct amebaz_dev_s *)kmm_zalloc(sizeof(*priv));
  if (!priv)
    {
      return NULL;
    }

  ret = amebaz_state_init(&priv->scan);
  ret |= amebaz_state_init(&priv->conn);
  if (ret)
    {
      kmm_free(priv);
      return NULL;
    }

  memcpy(priv->country, AMEBAZ_DEFAULT_COUNTRY, 2);
  priv->devnum = devnum;
  return priv;
}

static void amebaz_free_device(struct amebaz_dev_s *priv)
{
  amebaz_state_deinit(&priv->scan);
  amebaz_state_deinit(&priv->conn);
  kmm_free(priv);
}

static int amebaz_wl_on(int mode)
{
  int ret = -1;
  for (int i = 0; i < 2; i++)
    {
      ret = rltk_wlan_init(i, mode);
      if (ret < 0)
        {
          return ret;
        }
    }

  extern void up_irq_attach_workaround(void);
  up_irq_attach_workaround();
  for (int i = 0; i < 2; i++)
    {
      ret = rltk_wlan_start(i);
      if (ret < 0)
        {
          return ret;
        }

      while (!rltk_wlan_running(gp_wlan_dev[i]->devnum))
        {
          usleep(1000);
        }
    }

  ret = amebaz_wl_disable_powersave(0);
  return ret;
}

int amebaz_wl_initialize(unsigned char mode)
{
  struct amebaz_dev_s *priv;
  struct iwreq wrq =
  {
  };

  int ret;
  int i;
  for (i = 0; i < 2; i++)
    {
      priv = amebaz_allocate_device(i);
      if (!priv)
        {
          ret = -ENOMEM;
          goto free_dev;
        }

      ret = amebaz_netdev_register(priv);
      if (ret < 0)
        {
          amebaz_free_device(priv);
          goto free_dev;
        }

      gp_wlan_dev[i] = priv;
    }

  if (mode == RTW_MODE_STA_AP)
    {
      return amebaz_wl_on(RTW_MODE_STA_AP);
    }

  else
    {
      strlcpy(wrq.ifr_name, "wlan0", IFNAMSIZ);
      wrq.u.mode = IW_MODE_INFRA;
      return amebaz_wl_set_mode(gp_wlan_dev[0], &wrq);
    }

free_dev:
  for (i = 0; gp_wlan_dev[i]; i++)
    {
      netdev_unregister(&gp_wlan_dev[i]->dev);
      amebaz_free_device(gp_wlan_dev[i]);
      gp_wlan_dev[i] = NULL;
    }

  return ret;
}

