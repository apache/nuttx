/****************************************************************************
 * arch/xtensa/src/common/espressif/esp_wifi_utils.h
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

#ifndef __ARCH_XTENSA_SRC_COMMON_ESPRESSIF_ESP_WIFI_UTILS_H
#define __ARCH_XTENSA_SRC_COMMON_ESPRESSIF_ESP_WIFI_UTILS_H

#include <nuttx/config.h>
#include <nuttx/net/netdev.h>

#include "esp_private/wifi.h"

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif /* __ASSEMBLY__ */

#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
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

int esp_wifi_start_scan(struct iwreq *iwr);

/****************************************************************************
 * Name: esp_wifi_get_scan_results
 *
 * Description:
 *   Get scan result
 *
 * Input Parameters:
 *   req      The argument of the ioctl cmd
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
 *     None
 *
 ****************************************************************************/

void esp_wifi_scan_event_parse(void);

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

int esp_event_post(const char *event_base,
                   int32_t event_id,
                   void *event_data,
                   size_t event_data_size,
                   uint32_t ticks);

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

void esp_init_event_queue(void);

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

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_XTENSA_SRC_COMMON_ESPRESSIF_ESP_WIFI_UTILS_H */
