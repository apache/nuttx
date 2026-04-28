/****************************************************************************
 * arch/xtensa/src/common/espressif/esp_wifi_event_handler.c
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

#include <nuttx/debug.h>
#include <nuttx/signal.h>
#include <nuttx/wqueue.h>

#include "esp_event.h"
#include "esp_wifi.h"

#include "esp_wifi_utils.h"
#include "esp_wifi_api.h"

#ifndef CONFIG_SCHED_LPWORK
#  error "CONFIG_SCHED_LPWORK must be defined"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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

/* Wi-Fi event notification private data */

struct wifi_notify
{
  bool assigned;            /* Flag indicate if it is used */
  pid_t pid;                /* Signal's target thread PID */
  struct sigevent event;    /* Signal event private data */
  struct sigwork_s work;    /* Signal work private data */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct wifi_notify g_wifi_notify[WIFI_EVENT_MAX];
static struct work_s g_wifi_reconnect_work;
static bool g_wifi_handler_registered;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_reconnect_work_cb
 *
 * Description:
 *   Function called by a work queue to reconnect to Wi-Fi in case of
 *   a disconnection event and WIFI_REASON_ASSOC_LEAVE reason.
 *   Must check if the failure_retry_cnt is not 0, otherwise it may
 *   reconnect when not desired, such as when the user has actually
 *   asked to disconnect from the AP.
 *
 * Input Parameters:
 *   arg - Unused work queue argument.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef ESP_WLAN_HAS_STA
static void esp_reconnect_work_cb(void *arg)
{
  UNUSED(arg);
  int ret;
  wifi_config_t wifi_config;

  esp_wifi_get_config(WIFI_IF_STA, &wifi_config);
  if (wifi_config.sta.failure_retry_cnt == 0)
    {
      wlinfo("Reconnect to Wi-Fi on callback: failure_retry_cnt is 0\n");
      return;
    }

  ret = esp_wifi_sta_connect();
  if (ret < 0)
    {
      wlerr("Failed to reconnect to Wi-Fi on callback\n");
    }
}
#endif

/****************************************************************************
 * Name: esp_wifi_event_handler
 *
 * Description:
 *   Handler registered against WIFI_EVENT / ESP_EVENT_ANY_ID with the
 *   generic esp_event dispatcher (see esp_event.c). Translates Wi-Fi
 *   driver events into NuttX-visible actions (link-layer hooks and the
 *   optional sigevent notification subsystem).
 *
 * Input Parameters:
 *   arg        - Handler-specific argument registered with the event
 *                dispatcher (unused).
 *   event_base - Event base identifier; always WIFI_EVENT here (unused).
 *   event_id   - Wi-Fi event ID as defined by the ESP-IDF wifi_event_t
 *                enumeration.
 *   event_data - Pointer to the event-specific payload, whose concrete
 *                type depends on event_id.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp_wifi_event_handler(void *arg, esp_event_base_t event_base,
                                   int32_t event_id, void *event_data)
{
  int ret;
  struct wifi_notify *notify;
  wifi_ps_type_t ps_type = DEFAULT_PS_MODE;

  UNUSED(arg);
  UNUSED(event_base);

  net_lock();
  esp_wifi_lock(true);

  switch (event_id)
    {
#ifdef ESP_WLAN_DEVS
      case WIFI_EVENT_SCAN_DONE:
        esp_wifi_scan_event_parse();
        break;
#endif
      case WIFI_EVENT_HOME_CHANNEL_CHANGE:
        wlinfo("Wi-Fi home channel change\n");
        break;

#ifdef ESP_WLAN_HAS_STA
      case WIFI_EVENT_STA_START:
        {
          wlinfo("Wi-Fi sta start\n");

          ret = esp_wifi_set_ps(ps_type);
          if (ret != 0)
            {
              wlerr("Failed to set power save type\n");
              break;
            }
        }
        break;

      case WIFI_EVENT_STA_STOP:
        wlinfo("Wi-Fi station stopped\n");
        break;

      case WIFI_EVENT_STA_CONNECTED:
        {
          wlinfo("Wi-Fi station connected\n");
          esp_wlan_sta_connect_success_hook();
        }
        break;

      case WIFI_EVENT_STA_DISCONNECTED:
        {
          wifi_event_sta_disconnected_t *event =
              (wifi_event_sta_disconnected_t *)event_data;
          wifi_err_reason_t reason = event->reason;

          wlinfo("Wi-Fi station disconnected, reason: %u\n", reason);
          esp_wlan_sta_disconnect_hook();
          if (reason == WIFI_REASON_ASSOC_LEAVE)
            {
              work_queue(LPWORK, &g_wifi_reconnect_work,
                         esp_reconnect_work_cb, NULL, 0);
            }
        }
        break;

      case WIFI_EVENT_STA_AUTHMODE_CHANGE:
        wlinfo("Wi-Fi station auth mode change\n");
        break;
#endif /* ESP_WLAN_HAS_STA */

#ifdef ESP_WLAN_HAS_SOFTAP
      case WIFI_EVENT_AP_START:
        {
          wlinfo("INFO: Wi-Fi softap start\n");
          esp_wlan_softap_connect_success_hook();
          ret = esp_wifi_set_ps(ps_type);
          if (ret != 0)
            {
              wlerr("Failed to set power save type\n");
              break;
            }
        }
        break;

      case WIFI_EVENT_AP_STOP:
        {
          wlinfo("Wi-Fi softap stop\n");
          esp_wlan_softap_disconnect_hook();
        }
        break;

      case WIFI_EVENT_AP_STACONNECTED:
        wlinfo("Wi-Fi station joined AP\n");
        break;

      case WIFI_EVENT_AP_STADISCONNECTED:
        wlinfo("Wi-Fi station left AP\n");
        break;
#endif /* ESP_WLAN_HAS_SOFTAP */

      default:
        break;
    }

  if (event_id >= 0 && event_id < WIFI_EVENT_MAX)
    {
      notify = &g_wifi_notify[event_id];
      if (notify->assigned)
        {
          notify->event.sigev_value.sival_ptr = event_data;

          ret = nxsig_notification(notify->pid, &notify->event,
                                   SI_QUEUE, &notify->work);
          if (ret < 0)
            {
              wlwarn("nxsig_notification event ID=%d failed: %d\n",
                     (int)event_id, ret);
            }
        }
    }

  esp_wifi_lock(false);
  net_unlock();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_wifi_evt_work_init
 *
 * Description:
 *   Register the Wi-Fi event handler against the generic esp_event loop.
 *   Kept under the original name so existing callers (esp_wifi_api.c)
 *   continue to compile unchanged. Idempotent: subsequent calls after the
 *   first successful registration are silently ignored.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_wifi_evt_work_init(void)
{
  if (g_wifi_handler_registered)
    {
      return;
    }

  esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                             esp_wifi_event_handler, NULL);
  g_wifi_handler_registered = true;
}
