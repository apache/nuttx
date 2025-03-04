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

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_WIFI_UTILS_H */
