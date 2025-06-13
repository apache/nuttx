/****************************************************************************
 * arch/xtensa/src/common/espressif/esp_wifi_utils.c
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
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>
#include <netinet/arp.h>
#include <sys/param.h>

#include <nuttx/kmalloc.h>
#include <nuttx/wireless/wireless.h>
#include <nuttx/signal.h>
#include <nuttx/spinlock.h>

#ifdef CONFIG_ARCH_CHIP_ESP32
#include "esp32_wifi_adapter.h"
#endif
#ifdef CONFIG_ARCH_CHIP_ESP32S2
#include "esp32s2_wifi_adapter.h"
#endif
#ifdef CONFIG_ARCH_CHIP_ESP32S3
#include "esp32s3_wifi_adapter.h"
#endif

#ifdef CONFIG_ESPRESSIF_BLE
#  ifdef CONFIG_ARCH_CHIP_ESP32
#    include "esp32_ble_adapter.h"
#  endif
#  ifdef CONFIG_ARCH_CHIP_ESP32S3
#    include "esp32s3_ble_adapter.h"
#  endif
#  ifdef CONFIG_ESPRESSIF_WIFI_BT_COEXIST
#    include "private/esp_coexist_internal.h"
#  endif
#endif

#include "espressif/esp_wlan.h"

#include "esp_wifi_utils.h"
#include "esp_wireless.h"

#include "esp_log.h"
#include "esp_mac.h"
#include "esp_private/phy.h"
#include "esp_private/wifi.h"
#include "esp_random.h"
#include "esp_timer.h"
#ifdef CONFIG_ESPRESSIF_WIFI
#  include "esp_wpa.h"
#endif
#include "rom/ets_sys.h"
#include "soc/soc_caps.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Helper to get iw_event size */

#define ESP_IW_EVENT_SIZE(field) \
  (offsetof(struct iw_event, u) + sizeof(((union iwreq_data *)0)->field))

#ifdef CONFIG_ESPRESSIF_WIFI_SCAN_RESULT_SIZE
#  define WIFI_SCAN_RESULT_SIZE      CONFIG_ESPRESSIF_WIFI_SCAN_RESULT_SIZE
#else
#  define WIFI_SCAN_RESULT_SIZE      (4096)
#endif

#define SCAN_TIME_SEC                (5)

/* Maximum number of channels for Wi-Fi 2.4Ghz */

#define CHANNEL_MAX_NUM              (14)

/* CONFIG_POWER_SAVE_MODEM */

#if defined(CONFIG_ESPRESSIF_POWER_SAVE_MIN_MODEM)
#  define DEFAULT_PS_MODE WIFI_PS_MIN_MODEM
#elif defined(CONFIG_ESPRESSIF_POWER_SAVE_MAX_MODEM)
#  define DEFAULT_PS_MODE WIFI_PS_MAX_MODEM
#elif defined(CONFIG_ESPRESSIF_POWER_SAVE_NONE)
#  define DEFAULT_PS_MODE WIFI_PS_NONE
#else
#  define DEFAULT_PS_MODE WIFI_PS_NONE
#endif

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

/* Wi-Fi event ID */

enum wifi_adpt_evt_e
{
  WIFI_ADPT_EVT_SCAN_DONE = 0,
  WIFI_ADPT_EVT_STA_START,
  WIFI_ADPT_EVT_STA_CONNECT,
  WIFI_ADPT_EVT_STA_DISCONNECT,
  WIFI_ADPT_EVT_STA_AUTHMODE_CHANGE,
  WIFI_ADPT_EVT_STA_STOP,
  WIFI_ADPT_EVT_AP_START,
  WIFI_ADPT_EVT_AP_STOP,
  WIFI_ADPT_EVT_AP_STACONNECTED,
  WIFI_ADPT_EVT_AP_STADISCONNECTED,
  WIFI_ADPT_EVT_MAX,
};

/* Wi-Fi event callback function */

typedef void (*wifi_evt_cb_t)(void *p);

/* Wi-Fi event private data */

struct evt_adpt
{
  sq_entry_t entry;         /* Sequence entry */
  int32_t id;               /* Event ID */
  uint8_t buf[0];           /* Event private data */
};

/* Wi-Fi event notification private data */

struct wifi_notify
{
  bool assigned;            /* Flag indicate if it is used */
  pid_t pid;                /* Signal's target thread PID */
  struct sigevent event;    /* Signal event private data */
  struct sigwork_s work;    /* Signal work private data */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int esp_event_id_map(int event_id);

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Wi-Fi interface configuration */

#ifdef ESPRESSIF_WLAN_HAS_STA

extern wifi_config_t g_sta_wifi_cfg;

#endif /* ESPRESSIF_WLAN_HAS_STA */

#ifdef ESPRESSIF_WLAN_HAS_SOFTAP

extern wifi_config_t g_softap_wifi_cfg;

#endif /* ESPRESSIF_WLAN_HAS_SOFTAP */

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct wifi_scan_result g_scan_priv =
{
  .scan_signal = SEM_INITIALIZER(0),
};
static uint8_t g_channel_num;
static uint8_t g_channel_list[CHANNEL_MAX_NUM];

/* Wi-Fi event private data */

static spinlock_t g_lock_event;
static struct work_s g_wifi_evt_work;
static sq_queue_t g_wifi_evt_queue;
static struct wifi_notify g_wifi_notify[WIFI_ADPT_EVT_MAX];

static mutex_t g_wifiexcl_lock = NXMUTEX_INITIALIZER;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_event_id_map
 *
 * Description:
 *   Transform from esp-idf event ID to Wi-Fi adapter event ID
 *
 * Input Parameters:
 *   event_id - esp-idf event ID
 *
 * Returned Value:
 *   Wi-Fi adapter event ID
 *
 ****************************************************************************/

static int esp_event_id_map(int event_id)
{
  int id;

  switch (event_id)
    {
      case WIFI_EVENT_SCAN_DONE:
        id = WIFI_ADPT_EVT_SCAN_DONE;
        break;

#ifdef ESPRESSIF_WLAN_HAS_STA
      case WIFI_EVENT_STA_START:
        id = WIFI_ADPT_EVT_STA_START;
        break;

      case WIFI_EVENT_STA_CONNECTED:
        id = WIFI_ADPT_EVT_STA_CONNECT;
        break;

      case WIFI_EVENT_STA_DISCONNECTED:
        id = WIFI_ADPT_EVT_STA_DISCONNECT;
        break;

      case WIFI_EVENT_STA_AUTHMODE_CHANGE:
        id = WIFI_ADPT_EVT_STA_AUTHMODE_CHANGE;
        break;

      case WIFI_EVENT_STA_STOP:
        id = WIFI_ADPT_EVT_STA_STOP;
        break;
#endif /* ESPRESSIF_WLAN_HAS_STA */

#ifdef ESPRESSIF_WLAN_HAS_SOFTAP
      case WIFI_EVENT_AP_START:
        id = WIFI_ADPT_EVT_AP_START;
        break;

      case WIFI_EVENT_AP_STOP:
        id = WIFI_ADPT_EVT_AP_STOP;
        break;

      case WIFI_EVENT_AP_STACONNECTED:
        id = WIFI_ADPT_EVT_AP_STACONNECTED;
        break;

      case WIFI_EVENT_AP_STADISCONNECTED:
        id = WIFI_ADPT_EVT_AP_STADISCONNECTED;
        break;
#endif /* ESPRESSIF_WLAN_HAS_SOFTAP */

      default:
        wlerr("ERROR: Unknown event ID: %d\n", event_id);
        return -1;
    }

  return id;
}

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
  struct iw_scan_req *req;
  int ret = 0;
  int i;
  uint8_t target_mac[MAC_LEN];
  uint8_t target_ssid[SSID_MAX_LEN + 1];
  memset(target_ssid, 0x0, sizeof(SSID_MAX_LEN + 1));

  if (iwr == NULL)
    {
      wlerr("ERROR: Invalid ioctl cmd.\n");
      return -EINVAL;
    }

  if (g_scan_priv.scan_status != ESP_SCAN_DISABLED)
    {
      return OK;
    }

  config = kmm_calloc(1, sizeof(wifi_scan_config_t));
  if (config == NULL)
    {
      wlerr("ERROR: Cannot allocate result buffer\n");
      return -ENOMEM;
    }

  g_channel_num = 0;
  memset(g_channel_list, 0x0, CHANNEL_MAX_NUM);

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

          config->show_hidden = true;
          config->bssid = NULL;
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
  ret = esp_wifi_scan_start(config, false);
  if (ret != OK)
    {
      wlerr("ERROR: Scan error, ret: %d\n", ret);
      ret = ERROR;
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
    }

  if (ret == OK)
    {
      wlinfo("INFO: start scan\n");
      g_scan_priv.scan_status = ESP_SCAN_RUN;
    }

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
      irqstate_t irqstate = enter_critical_section();
      if (!scan_block)
        {
          scan_block = true;
          leave_critical_section(irqstate);
          nxsem_tickwait(&priv->scan_signal, SEC2TICK(SCAN_TIME_SEC));
          scan_block = false;
        }
      else
        {
          leave_critical_section(irqstate);
          ret = -EINVAL;
          goto exit_failed;
        }
    }
  else if (g_scan_priv.scan_status == ESP_SCAN_DISABLED)
    {
      ret = -EINVAL;
      goto exit_failed;
    }

  if ((iwr == NULL) || (g_scan_priv.scan_status != ESP_SCAN_DONE))
    {
      ret = -EINVAL;
      goto exit_failed;
    }

  if (priv->scan_result == NULL)
    {
      /* Result have already been requested */

      ret = OK;
      iwr->u.data.length = 0;
      goto exit_failed;
    }

  if (priv->scan_result_size <= 0)
    {
      ret = OK;
      iwr->u.data.length = 0;
      goto exit_free_buffer;
    }

  if (iwr->u.data.pointer == NULL ||
      iwr->u.data.length < priv->scan_result_size)
    {
      /* Stat request, return scan_result_size */

      ret = -E2BIG;
      iwr->u.data.pointer = NULL;
      iwr->u.data.length = priv->scan_result_size;
      return ret;
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

  if (priv->scan_status != ESP_SCAN_RUN)
    {
      return;
    }

  esp_wifi_scan_get_ap_num(&bss_total);
  if (bss_total == 0)
    {
      priv->scan_status = ESP_SCAN_DONE;
      wlinfo("INFO: None AP is scanned\n");
      nxsem_post(&priv->scan_signal);
      return;
    }

  ap_list_buffer = kmm_calloc(bss_total, sizeof(wifi_ap_record_t));
  if (ap_list_buffer == NULL)
    {
      priv->scan_status = ESP_SCAN_DONE;
      wlerr("ERROR: Failed to calloc buffer to print scan results");
      nxsem_post(&priv->scan_signal);
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

          if (is_target_channel)
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
                              ap_list_buffer[bss_count].ssid), SSID_MAX_LEN);
              essid_len_aligned = (essid_len + 3) & -4;
              if (result_size < ESP_IW_EVENT_SIZE(essid) + essid_len_aligned)
                {
                  goto scan_result_full;
                }

              iwe = (struct iw_event *)
                    &priv->scan_result[priv->scan_result_size];
              iwe->len = ESP_IW_EVENT_SIZE(essid) + essid_len_aligned;
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
                    ESP_IW_EVENT_SIZE(essid) + essid_len_aligned;
              result_size -= ESP_IW_EVENT_SIZE(essid) + essid_len_aligned;

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

  if (!parse_done)
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

/****************************************************************************
 * Name: esp_evt_work_cb
 *
 * Description:
 *   Process the cached event
 *
 * Input Parameters:
 *   arg - No mean
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp_evt_work_cb(void *arg)
{
  int ret;
  irqstate_t flags;
  struct evt_adpt *evt_adpt;
  struct wifi_notify *notify;
  wifi_ps_type_t ps_type = DEFAULT_PS_MODE;

  while (1)
    {
      flags = spin_lock_irqsave(&g_lock_event);
      evt_adpt = (struct evt_adpt *)sq_remfirst(&g_wifi_evt_queue);
      spin_unlock_irqrestore(&g_lock_event, flags);
      if (!evt_adpt)
        {
          break;
        }

      /* Some of the following logic (eg. esp32s3_wlan_sta_set_linkstatus)
       * can take net_lock(). To maintain the consistent locking order,
       * we take net_lock() here before taking esp_wifi_lock. Note that
       * net_lock() is a recursive lock.
       */

      net_lock();
      esp_wifi_lock(true);

      switch (evt_adpt->id)
        {
          case WIFI_ADPT_EVT_SCAN_DONE:
            esp_wifi_scan_event_parse();
            break;

#ifdef ESPRESSIF_WLAN_HAS_STA
          case WIFI_ADPT_EVT_STA_START:
            wlinfo("Wi-Fi sta start\n");

            g_sta_connected = false;

#ifdef CONFIG_ESPRESSIF_BLE
            if (esp_bt_controller_get_status() !=
                ESP_BT_CONTROLLER_STATUS_IDLE)
              {
                if (ps_type == WIFI_PS_NONE)
                  {
                    ps_type = WIFI_PS_MIN_MODEM;
                  }
              }
#endif

            ret = esp_wifi_set_ps(ps_type);

            if (ret)
              {
                wlerr("Failed to set power save type\n");
                break;
              }
            else
              {
                wlinfo("INFO: Set ps type=%d\n", ps_type);
              }

            ret = esp_wifi_get_config(WIFI_IF_STA, &g_sta_wifi_cfg);
            if (ret)
              {
                wlerr("Failed to get Wi-Fi config data ret=%d\n", ret);
              }
            break;

          case WIFI_ADPT_EVT_STA_CONNECT:
            wlinfo("Wi-Fi sta connect\n");
            g_sta_connected = true;
            ret = esp_wlan_sta_set_linkstatus(true);
            if (ret < 0)
              {
                wlerr("ERROR: Failed to set Wi-Fi station link status\n");
              }

            break;

          case WIFI_ADPT_EVT_STA_DISCONNECT:
            wlinfo("Wi-Fi sta disconnect\n");
            g_sta_connected = false;
            ret = esp_wlan_sta_set_linkstatus(false);
            if (ret < 0)
              {
                wlerr("ERROR: Failed to set Wi-Fi station link status\n");
              }

            if (g_sta_reconnect)
              {
                ret = esp_wifi_connect();
                if (ret)
                  {
                    wlerr("Failed to connect AP error=%d\n", ret);
                  }
              }
            break;

          case WIFI_ADPT_EVT_STA_STOP:
            wlinfo("Wi-Fi sta stop\n");
            g_sta_connected = false;
            break;
#endif /* ESPRESSIF_WLAN_HAS_STA */

#ifdef ESPRESSIF_WLAN_HAS_SOFTAP
          case WIFI_ADPT_EVT_AP_START:
            wlinfo("INFO: Wi-Fi softap start\n");

#ifdef CONFIG_ESPRESSIF_BLE
            if (esp_bt_controller_get_status() !=
                ESP_BT_CONTROLLER_STATUS_IDLE)
              {
                if (ps_type == WIFI_PS_NONE)
                  {
                    ps_type = WIFI_PS_MIN_MODEM;
                  }
              }
#endif

            ret = esp_wifi_set_ps(ps_type);

            if (ret)
              {
                wlerr("Failed to set power save type\n");
                break;
              }
            else
              {
                wlinfo("INFO: Set ps type=%d\n", ps_type);
              }

            ret = esp_wifi_get_config(WIFI_IF_AP, &g_softap_wifi_cfg);
            if (ret)
              {
                wlerr("Failed to get Wi-Fi config data ret=%d\n", ret);
              }
            break;

          case WIFI_ADPT_EVT_AP_STOP:
            wlinfo("INFO: Wi-Fi softap stop\n");
            break;

          case WIFI_ADPT_EVT_AP_STACONNECTED:
            wlinfo("INFO: Wi-Fi station join\n");
            break;

          case WIFI_ADPT_EVT_AP_STADISCONNECTED:
            wlinfo("INFO: Wi-Fi station leave\n");
            break;
#endif /* ESPRESSIF_WLAN_HAS_SOFTAP */
          default:
            break;
        }

      notify = &g_wifi_notify[evt_adpt->id];
      if (notify->assigned)
        {
          notify->event.sigev_value.sival_ptr = evt_adpt->buf;

          ret = nxsig_notification(notify->pid, &notify->event,
                                   SI_QUEUE, &notify->work);
          if (ret < 0)
            {
              wlwarn("nxsig_notification event ID=%" PRId32 " failed: %d\n",
                     evt_adpt->id, ret);
            }
        }

      esp_wifi_lock(false);
      net_unlock();

      kmm_free(evt_adpt);
    }
}

/****************************************************************************
 * Name: esp_event_post
 *
 * Description:
 *   Active work queue and let the work to process the cached event
 *
 * Input Parameters:
 *   event_base      - Event set name
 *   event_id        - Event ID
 *   event_data      - Event private data
 *   event_data_size - Event data size
 *   ticks           - Waiting system ticks
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int esp_event_post(esp_event_base_t event_base,
                   int32_t event_id,
                   void *event_data,
                   size_t event_data_size,
                   uint32_t ticks)
{
  size_t size;
  int32_t id;
  irqstate_t flags;
  struct evt_adpt *evt_adpt;

  wlinfo("Event: base=%s id=%" PRId32 " data=%p data_size=%d "
         "ticks=%" PRIu32 "\n",
         event_base, event_id, event_data, event_data_size, ticks);

  id = esp_event_id_map(event_id);
  if (id < 0)
    {
      wlinfo("No process event %" PRId32 "\n", event_id);
      return -1;
    }

  size = event_data_size + sizeof(struct evt_adpt);
  evt_adpt = kmm_malloc(size);
  if (!evt_adpt)
    {
      wlerr("Failed to alloc %d memory\n", size);
      return -1;
    }

  evt_adpt->id = id;
  memcpy(evt_adpt->buf, event_data, event_data_size);

  flags = spin_lock_irqsave(&g_lock_event);
  sq_addlast(&evt_adpt->entry, &g_wifi_evt_queue);
  spin_unlock_irqrestore(&g_lock_event, flags);

  work_queue(LPWORK, &g_wifi_evt_work, esp_evt_work_cb, NULL, 0);

  return 0;
}

/****************************************************************************
 * Name: esp_init_event_queue
 *
 * Description:
 *   Initialize the Wi-Fi event queue that holds pending events to be
 *   processed.  This queue is used to store Wi-Fi events like scan
 *   completion, station connect/disconnect etc. before they are handled by
 *   the event work callback.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp_init_event_queue(void)
{
  sq_init(&g_wifi_evt_queue);
}

/****************************************************************************
 * Name: esp_wifi_lock
 *
 * Description:
 *   Lock or unlock the event process
 *
 * Input Parameters:
 *   lock - true: Lock event process, false: unlock event process
 *
 * Returned Value:
 *   The result of lock or unlock the event process
 *
 ****************************************************************************/

int esp_wifi_lock(bool lock)
{
  int ret;

  if (lock)
    {
      ret = nxmutex_lock(&g_wifiexcl_lock);
      if (ret < 0)
        {
          wlinfo("Failed to lock Wi-Fi ret=%d\n", ret);
        }
    }
  else
    {
      ret = nxmutex_unlock(&g_wifiexcl_lock);
      if (ret < 0)
        {
          wlinfo("Failed to unlock Wi-Fi ret=%d\n", ret);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: esp_wifi_notify_subscribe
 *
 * Description:
 *   Enable event notification
 *
 * Input Parameters:
 *   pid   - Task PID
 *   event - Signal event data pointer
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int esp_wifi_notify_subscribe(pid_t pid, struct sigevent *event)
{
  int id;
  struct wifi_notify *notify;
  int ret = -1;

  wlinfo("PID=%d event=%p\n", pid, event);

  esp_wifi_lock(true);

  if (event->sigev_notify == SIGEV_SIGNAL)
    {
      id = esp_event_id_map(event->sigev_signo);
      if (id < 0)
        {
          wlerr("No process event %d\n", event->sigev_signo);
        }
      else
        {
          notify = &g_wifi_notify[id];

          if (notify->assigned)
            {
              wlerr("sigev_signo %d has subscribed\n",
                    event->sigev_signo);
            }
          else
            {
              if (pid == 0)
                {
                  pid = nxsched_gettid();
                  wlinfo("Actual PID=%d\n", pid);
                }

              notify->pid = pid;
              notify->event = *event;
              notify->assigned = true;

              ret = 0;
            }
        }
    }
  else if (event->sigev_notify == SIGEV_NONE)
    {
      id = esp_event_id_map(event->sigev_signo);
      if (id < 0)
        {
          wlerr("No process event %d\n", event->sigev_signo);
        }
      else
        {
          notify = &g_wifi_notify[id];

          if (!notify->assigned)
            {
              wlerr("sigev_signo %d has not subscribed\n",
                    event->sigev_signo);
            }
          else
            {
              notify->assigned = false;

              ret = 0;
            }
        }
    }
  else
    {
      wlerr("sigev_notify %d is invalid\n", event->sigev_signo);
    }

  esp_wifi_lock(false);

  return ret;
}
