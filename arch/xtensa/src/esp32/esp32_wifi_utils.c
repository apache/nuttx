/****************************************************************************
 * arch/xtensa/src/esp32/esp32_wifi_utils.c
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

#include <assert.h>
#include <debug.h>
#include <netinet/arp.h>
#include <sys/param.h>

#include <nuttx/kmalloc.h>
#include <nuttx/wireless/wireless.h>

#include "esp32_wifi_adapter.h"
#include "esp32_wifi_utils.h"
#include "esp32_wireless.h"
#include "espidf_wifi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Helper to get iw_event size */

#define ESP_IW_EVENT_SIZE(field) \
  (offsetof(struct iw_event, u) + sizeof(((union iwreq_data *)0)->field))

#ifdef CONFIG_ESP32_WIFI_SCAN_RESULT_SIZE
#  define WIFI_SCAN_RESULT_SIZE      CONFIG_ESP32_WIFI_SCAN_RESULT_SIZE
#else
#  define WIFI_SCAN_RESULT_SIZE      (4096)
#endif

#define SCAN_TIME_SEC                (5)
#define SSID_LEN                     (33)

/* Maximum number of channels for Wi-Fi 2.4Ghz */

#define CHANNEL_MAX_NUM              (14)

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum scan_status_e
{
  ESP_SCAN_DISABLED = 0,
  ESP_SCAN_RUN,
  ESP_SCAN_DONE
};

/* Wi-Fi scan result information */

struct wifi_scan_result
{
  enum scan_status_e scan_status;      /* Scan status */
  sem_t scan_signal;                   /* Scan notification signal */
  uint8_t *scan_result;                /* Temp buffer that holds results */
  unsigned int scan_result_size;       /* Current size of temp buffer */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct wifi_scan_result g_scan_priv =
{
  .scan_signal = SEM_INITIALIZER(0),
};
static uint8_t g_channel_num = 0;
static uint8_t g_channel_list[CHANNEL_MAX_NUM];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_wifi_start_scan
 *
 * Description:
 *   Scan all available APs.
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_start_scan(struct iwreq *iwr)
{
  struct wifi_scan_result *priv = &g_scan_priv;
  wifi_scan_config_t *config = NULL;
  uint8_t target_ssid[SSID_LEN];
  struct iw_scan_req *req;
  int ret = 0;
  int i;
  uint8_t target_mac[MAC_LEN];

  memset(target_ssid, 0x0, sizeof(SSID_LEN));
  if (iwr == NULL)
    {
      wlerr("ERROR: Invalid ioctl cmd.\n");
      return -EINVAL;
    }

  if (g_scan_priv.scan_status != ESP_SCAN_DISABLED)
    {
      return OK;
    }

  config = kmm_malloc(sizeof(wifi_scan_config_t));
  if (config == NULL)
    {
      wlerr("ERROR: Cannot allocate result buffer\n");
      return -ENOMEM;
    }

  g_channel_num = 0;
  memset(g_channel_list, 0x0, CHANNEL_MAX_NUM);
  memset(config, 0x0, sizeof(wifi_scan_config_t));
  if (iwr->u.data.pointer &&
      iwr->u.data.length >= sizeof(struct iw_scan_req))
    {
      req = (struct iw_scan_req *)iwr->u.data.pointer;
      config->scan_type = (req->scan_type == IW_SCAN_TYPE_ACTIVE ?
            WIFI_SCAN_TYPE_ACTIVE : WIFI_SCAN_TYPE_PASSIVE);
      if (iwr->u.data.flags & IW_SCAN_THIS_ESSID &&
          req->essid_len < sizeof(target_ssid))
        {
          /* Scan specific ESSID */

          memcpy(&target_ssid[0], req->essid, req->essid_len);
          config->ssid = &target_ssid[0];
          config->ssid[req->essid_len] = '\0';
        }

      if (iwr->u.data.flags & IW_SCAN_THIS_FREQ &&
          req->num_channels > 0)
        {
          /* Scan specific channels */

          DEBUGASSERT(req->num_channels <= CHANNEL_MAX_NUM);
          g_channel_num = req->num_channels;
          if (req->num_channels == 1)
            {
              config->channel = req->channel_list[0].m;
            }
          else
            {
              for (i = 0; i < req->num_channels; i++)
                {
                  g_channel_list[i] = req->channel_list[i].m;
                }
            }
        }

      memset(target_mac, 0xff, MAC_LEN);
      if (memcmp(req->bssid.sa_data, target_mac, MAC_LEN) != 0)
        {
          /* Scan specific bssid */

          memcpy(target_mac, req->bssid.sa_data, MAC_LEN);
          config->bssid = &target_mac[0];
        }
    }
  else
    {
      /* Default scan parameters */

      wlinfo("INFO: Use default scan parameters\n");
      config->scan_type = WIFI_SCAN_TYPE_ACTIVE; /* Active scan */
    }

  esp_wifi_start();

  esp_wifi_scan_stop();
  ret = esp_wifi_scan_start(config, false);
  if (ret != OK)
    {
      wlerr("ERROR: Scan error, ret: %d\n", ret);
    }
  else
    {
      /* Allocate buffer to store scan result */

      if (priv->scan_result == NULL)
        {
          priv->scan_result = kmm_malloc(WIFI_SCAN_RESULT_SIZE);
          if (priv->scan_result == NULL)
            {
              wlerr("ERROR: Cannot allocate result buffer\n");
              ret = -ENOMEM;
            }
          else
            {
              memset(priv->scan_result, 0x0, WIFI_SCAN_RESULT_SIZE);
            }
        }
    }

  if (config)
    {
      kmm_free(config);
      config = NULL;
      wlinfo("INFO: start scan\n");
    }

  g_scan_priv.scan_status = ESP_SCAN_RUN;

  return ret;
}

/****************************************************************************
 * Name: esp_wifi_get_scan_results
 *
 * Description:
 *   Get scan result
 *
 * Input Parameters:
 *   iwr - The argument of the ioctl cmd
 *
 * Returned Value:
 *   OK on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

int esp_wifi_get_scan_results(struct iwreq *iwr)
{
  int ret = OK;
  static bool scan_block = false;
  struct wifi_scan_result *priv = &g_scan_priv;

  if (g_scan_priv.scan_status == ESP_SCAN_RUN)
    {
      if (scan_block == false)
        {
          scan_block = true;
          nxsem_tickwait(&priv->scan_signal, SEC2TICK(SCAN_TIME_SEC));
          scan_block = false;
        }
      else
        {
          ret = -EINVAL;
          goto exit_failed;
        }
    }

  if ((iwr == NULL) || (g_scan_priv.scan_status != ESP_SCAN_DONE))
    {
      ret = -EINVAL;
      goto exit_failed;
    }

  if (!priv->scan_result)
    {
      /* Result have already been requested */

      ret = OK;
      iwr->u.data.length = 0;
      goto exit_failed;
    }

  if (iwr->u.data.pointer == NULL ||
      iwr->u.data.length < priv->scan_result_size)
    {
      /* Stat request, return scan_result_size */

      ret = -E2BIG;
      iwr->u.data.pointer = NULL;
      iwr->u.data.length = priv->scan_result_size;
      goto exit_failed;
    }

  if (priv->scan_result_size <= 0)
    {
      ret = OK;
      iwr->u.data.length = 0;
      goto exit_free_buffer;
    }

  /* Copy result to user buffer */

  if (iwr->u.data.length > priv->scan_result_size)
    {
      iwr->u.data.length = priv->scan_result_size;
    }

  memcpy(iwr->u.data.pointer, priv->scan_result, iwr->u.data.length);

exit_free_buffer:

  /* Free scan result buffer */

  kmm_free(priv->scan_result);
  priv->scan_result = NULL;
  priv->scan_result_size = 0;
  g_scan_priv.scan_status = ESP_SCAN_DISABLED;

exit_failed:
  if (ret < 0)
    {
      iwr->u.data.length = 0;
    }

  return ret;
}

/****************************************************************************
 * Name: esp_wifi_scan_event_parse
 *
 * Description:
 *   Parse scan information
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *     None
 *
 ****************************************************************************/

void esp_wifi_scan_event_parse(void)
{
  struct wifi_scan_result *priv = &g_scan_priv;
  wifi_ap_record_t *ap_list_buffer = NULL;
  uint16_t bss_total = 0;
  uint8_t bss_count = 0;
  bool parse_done = false;

  esp_wifi_scan_get_ap_num(&bss_total);
  if (bss_total == 0)
    {
      priv->scan_status = ESP_SCAN_DONE;
      wlinfo("INFO: None AP is scanned\n");
      return;
    }

  ap_list_buffer = kmm_zalloc(bss_total * sizeof(wifi_ap_record_t));
  if (ap_list_buffer == NULL)
    {
      priv->scan_status = ESP_SCAN_DONE;
      wlerr("ERROR: Failed to malloc buffer to print scan results");
      return;
    }

  if (esp_wifi_scan_get_ap_records(&bss_total,
              (wifi_ap_record_t *)ap_list_buffer) == OK)
    {
      struct iw_event *iwe;
      unsigned int result_size;
      size_t essid_len;
      size_t essid_len_aligned;
      bool is_target_channel = true;
      int i;
      for (bss_count = 0; bss_count < bss_total; bss_count++)
        {
          if (g_channel_num > 1)
            {
              is_target_channel = false;
              for (i = 0; i < g_channel_num; i++)
                {
                  if (g_channel_list[i] == ap_list_buffer[bss_count].primary)
                    {
                      is_target_channel = true;
                      break;
                    }
                }
            }
          else
            {
              is_target_channel = true;
            }

          if (is_target_channel == true)
            {
              result_size = WIFI_SCAN_RESULT_SIZE - priv->scan_result_size;

              /* Copy BSSID */

              if (result_size < ESP_IW_EVENT_SIZE(ap_addr))
                {
                  goto scan_result_full;
                }

              iwe = (struct iw_event *)
                            &priv->scan_result[priv->scan_result_size];
              iwe->len = ESP_IW_EVENT_SIZE(ap_addr);
              iwe->cmd = SIOCGIWAP;
              memcpy(&iwe->u.ap_addr.sa_data,
                     ap_list_buffer[bss_count].bssid,
                     sizeof(ap_list_buffer[bss_count].bssid));
              iwe->u.ap_addr.sa_family = ARPHRD_ETHER;
              priv->scan_result_size += ESP_IW_EVENT_SIZE(ap_addr);
              result_size -= ESP_IW_EVENT_SIZE(ap_addr);

              /* Copy ESSID */

              essid_len = MIN(strlen((const char *)
                                     ap_list_buffer[bss_count].ssid), 32);
              essid_len_aligned = (essid_len + 3) & -4;
              if (result_size < ESP_IW_EVENT_SIZE(essid)+essid_len_aligned)
                {
                  goto scan_result_full;
                }

              iwe = (struct iw_event *)
                    &priv->scan_result[priv->scan_result_size];
              iwe->len = ESP_IW_EVENT_SIZE(essid)+essid_len_aligned;
              iwe->cmd = SIOCGIWESSID;
              iwe->u.essid.flags = 0;
              iwe->u.essid.length = essid_len;

              /* Special processing for iw_point, set offset
               * in pointer field.
               */

              iwe->u.essid.pointer = (void *)sizeof(iwe->u.essid);
              memcpy(&iwe->u.essid + 1,
                    ap_list_buffer[bss_count].ssid, essid_len);
              wlinfo("INFO: ssid %s\n", ap_list_buffer[bss_count].ssid);
              priv->scan_result_size +=
                    ESP_IW_EVENT_SIZE(essid)+essid_len_aligned;
              result_size -= ESP_IW_EVENT_SIZE(essid)+essid_len_aligned;

              /* Copy link quality info */

              if (result_size < ESP_IW_EVENT_SIZE(qual))
                {
                  goto scan_result_full;
                }

              iwe = (struct iw_event *)
                    &priv->scan_result[priv->scan_result_size];
              iwe->len = ESP_IW_EVENT_SIZE(qual);
              iwe->cmd = IWEVQUAL;
              iwe->u.qual.qual = 0x00;
              wlinfo("INFO: signal %d\n", ap_list_buffer[bss_count].rssi);
              iwe->u.qual.level = ap_list_buffer[bss_count].rssi;
              iwe->u.qual.noise = 0x00;
              iwe->u.qual.updated = IW_QUAL_DBM | IW_QUAL_ALL_UPDATED;

              priv->scan_result_size += ESP_IW_EVENT_SIZE(qual);
              result_size -= ESP_IW_EVENT_SIZE(qual);

              /* Copy AP mode */

              if (result_size < ESP_IW_EVENT_SIZE(mode))
                {
                  goto scan_result_full;
                }

              iwe = (struct iw_event *)
                    &priv->scan_result[priv->scan_result_size];
              iwe->len = ESP_IW_EVENT_SIZE(mode);
              iwe->cmd = SIOCGIWMODE;
              iwe->u.mode = IW_MODE_MASTER;
              priv->scan_result_size += ESP_IW_EVENT_SIZE(mode);
              result_size -= ESP_IW_EVENT_SIZE(mode);

              /* Copy AP encryption mode */

              if (result_size < ESP_IW_EVENT_SIZE(data))
                {
                  goto scan_result_full;
                }

              iwe = (struct iw_event *)
                    &priv->scan_result[priv->scan_result_size];
              iwe->len = ESP_IW_EVENT_SIZE(data);
              iwe->cmd = SIOCGIWENCODE;
              iwe->u.data.flags = IW_ENCODE_ENABLED | IW_ENCODE_NOKEY;
              iwe->u.data.length = 0;
              iwe->u.essid.pointer = NULL;

              priv->scan_result_size += ESP_IW_EVENT_SIZE(data);
              result_size -= ESP_IW_EVENT_SIZE(data);

              /* Copy AP channel */

              if (result_size < ESP_IW_EVENT_SIZE(freq))
                {
                  goto scan_result_full;
                }

              iwe = (struct iw_event *)
                    &priv->scan_result[priv->scan_result_size];
              iwe->len = ESP_IW_EVENT_SIZE(freq);
              iwe->cmd = SIOCGIWFREQ;
              iwe->u.freq.e = 0;
              iwe->u.freq.m = ap_list_buffer[bss_count].primary;

              priv->scan_result_size += ESP_IW_EVENT_SIZE(freq);
              result_size -= ESP_IW_EVENT_SIZE(freq);
            }
        }

      parse_done = true;
    }

scan_result_full:

  /* Continue instead of break to log dropped AP results */

  if (parse_done == false)
    {
      wlerr("ERROR: No more space in scan_result buffer\n");
    }

  if (ap_list_buffer)
    {
      kmm_free(ap_list_buffer);
      ap_list_buffer = NULL;
    }

  priv->scan_status = ESP_SCAN_DONE;
  nxsem_post(&priv->scan_signal);
}
