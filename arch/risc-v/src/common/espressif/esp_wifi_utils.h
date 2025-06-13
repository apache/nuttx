/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_wifi_utils.h
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

#ifndef __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_WIFI_UTILS_H
#define __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_WIFI_UTILS_H

#include <nuttx/config.h>
#include <nuttx/net/netdev.h>
#include <nuttx/signal.h>

#ifndef __ASSEMBLY__

#include <stdint.h>

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

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
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_WIFI

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

int esp_wifi_start_scan(struct iwreq *iwr);

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

int esp_wifi_get_scan_results(struct iwreq *iwr);

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
 *   None
 *
 ****************************************************************************/

void esp_wifi_scan_event_parse(void);

#endif

/****************************************************************************
 * Name: esp_wifi_to_errno
 *
 * Description:
 *   Transform from ESP Wi-Fi error code to NuttX error code
 *
 * Input Parameters:
 *   err - ESP Wi-Fi error code
 *
 * Returned Value:
 *   NuttX error code defined in errno.h
 *
 ****************************************************************************/

int32_t esp_wifi_to_errno(int err);

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

int esp_event_id_map(int event_id);

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

int esp_wifi_lock(bool lock);

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

void esp_evt_work_cb(void *arg);

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
                         uint32_t ticks);

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

int esp_wifi_notify_subscribe(pid_t pid, struct sigevent *event);

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_WIFI_UTILS_H */
