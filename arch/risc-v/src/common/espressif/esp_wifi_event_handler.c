/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_wifi_event_handler.c
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

#include <debug.h>
#include <nuttx/spinlock.h>
#include <nuttx/signal.h>

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

/* Wi-Fi event private data */

struct evt_adpt
{
  sq_entry_t entry;         /* Sequence entry */
  wifi_event_t id;          /* Event ID */
  uint8_t buf[0];           /* Event private data */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct wifi_notify g_wifi_notify[WIFI_EVENT_MAX];
static struct work_s g_wifi_evt_work;
static sq_queue_t g_wifi_evt_queue;
static spinlock_t g_lock;

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
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

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

/****************************************************************************
 * Name: esp_evt_work_cb
 *
 * Description:
 *   Process Wi-Fi events.
 *
 * Input Parameters:
 *   arg - Not used.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp_evt_work_cb(void *arg)
{
  int ret;
  irqstate_t flags;
  struct evt_adpt *evt_adpt;
  struct wifi_notify *notify;
  wifi_ps_type_t ps_type = DEFAULT_PS_MODE;

  while (1)
    {
      flags = spin_lock_irqsave(&g_lock);
      evt_adpt = (struct evt_adpt *)sq_remfirst(&g_wifi_evt_queue);
      spin_unlock_irqrestore(&g_lock, flags);
      if (!evt_adpt)
        {
          break;
        }

      /* Some of the following logic (eg. esp_wlan_sta_set_linkstatus)
       * can take net_lock(). To maintain the consistent locking order,
       * we take net_lock() here before taking esp_wifi_lock. Note that
       * net_lock() is a recursive lock.
       */

      net_lock();
      esp_wifi_lock(true);

      switch (evt_adpt->id)
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
            wlinfo("Wi-Fi sta start\n");

            ret = esp_wifi_set_ps(ps_type);
            if (ret)
              {
                wlerr("Failed to set power save type\n");
                break;
              }
            break;

          case WIFI_EVENT_STA_STOP:
            wlinfo("Wi-Fi station stopped\n");
            break;

          case WIFI_EVENT_STA_CONNECTED:
            wlinfo("Wi-Fi station connected\n");
            esp_wlan_sta_connect_success_hook();
            break;

          case WIFI_EVENT_STA_DISCONNECTED:
            wifi_event_sta_disconnected_t *event =
            (wifi_event_sta_disconnected_t *)evt_adpt->buf;
            wifi_err_reason_t reason = event->reason;

            wlinfo("Wi-Fi station disconnected, reason: %u\n", reason);
            esp_wlan_sta_disconnect_hook();
            if (reason == WIFI_REASON_ASSOC_LEAVE)
              {
                work_queue(LPWORK, &g_wifi_evt_work, esp_reconnect_work_cb,
                           NULL, 0);
              }

            break;

          case WIFI_EVENT_STA_AUTHMODE_CHANGE:
            wlinfo("Wi-Fi station auth mode change\n");
            break;
#endif /* ESP_WLAN_HAS_STA */

#ifdef ESP_WLAN_HAS_SOFTAP
          case WIFI_EVENT_AP_START:
            wlinfo("INFO: Wi-Fi softap start\n");
            esp_wlan_softap_connect_success_hook();
            ret = esp_wifi_set_ps(ps_type);
            if (ret)
              {
                wlerr("Failed to set power save type\n");
                break;
              }
            break;

          case WIFI_EVENT_AP_STOP:
            wlinfo("Wi-Fi softap stop\n");
            esp_wlan_softap_disconnect_hook();
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

      notify = &g_wifi_notify[evt_adpt->id];
      if (notify->assigned)
        {
          notify->event.sigev_value.sival_ptr = evt_adpt->buf;

          ret = nxsig_notification(notify->pid, &notify->event,
                                   SI_QUEUE, &notify->work);
          if (ret < 0)
            {
              wlwarn("nxsig_notification event ID=%d failed: %d\n",
                     evt_adpt->id, ret);
            }
        }

      esp_wifi_lock(false);
      net_unlock();

      kmm_free(evt_adpt);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_event_post
 *
 * Description:
 *   Posts an event to the event loop system. The event is queued in a FIFO
 *   and processed asynchronously in the low-priority work queue.
 *
 * Input Parameters:
 *   event_base      - Identifier for the event category (e.g. WIFI_EVENT)
 *   event_id        - Event ID within the event base category
 *   event_data      - Pointer to event data structure
 *   event_data_size - Size of event data structure
 *   ticks           - Number of ticks to wait (currently unused)
 *
 * Returned Value:
 *   0 on success
 *   -1 on failure with following error conditions:
 *      - Invalid event ID
 *      - Memory allocation failure
 *
 * Assumptions/Limitations:
 *   - Event data is copied into a new buffer, so the original can be freed
 *   - Events are processed in FIFO order in the low priority work queue
 *   - The function is thread-safe and can be called from interrupt context
 *
 ****************************************************************************/

int esp_event_post(const char *event_base,
                         int32_t event_id,
                         void *event_data,
                         size_t event_data_size,
                         uint32_t ticks)
{
  size_t size;
  int32_t id;
  irqstate_t flags;
  struct evt_adpt *evt_adpt;

  wlinfo("Event: base=%s id=%ld data=%p data_size=%u ticks=%lu\n",
         event_base, event_id, event_data, event_data_size, ticks);

  size = event_data_size + sizeof(struct evt_adpt);
  evt_adpt = kmm_malloc(size);
  if (!evt_adpt)
    {
      wlerr("ERROR: Failed to alloc %d memory\n", size);
      return -1;
    }

  evt_adpt->id = event_id;
  memcpy(evt_adpt->buf, event_data, event_data_size);

  flags = enter_critical_section();
  sq_addlast(&evt_adpt->entry, &g_wifi_evt_queue);
  leave_critical_section(flags);

  work_queue(LPWORK, &g_wifi_evt_work, esp_evt_work_cb, NULL, 0);

  return 0;
}

/****************************************************************************
 * Name: esp_evt_work_init
 *
 * Description:
 *   Initialize the event work queue
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_evt_work_init(void)
{
  sq_init(&g_wifi_evt_queue);
}
