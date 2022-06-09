/****************************************************************************
 * arch/xtensa/src/esp32/esp32_wlan.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32_ESP32_WLAN_H
#define __ARCH_XTENSA_SRC_ESP32_ESP32_WLAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "esp32_wifi_adapter.h"

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#ifdef CONFIG_ESP32_WIFI

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_wlan_sta_initialize
 *
 * Description:
 *   Initialize the esp32 WLAN station netcard driver
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

#ifdef ESP32_WLAN_HAS_STA
int esp32_wlan_sta_initialize(void);
#endif

/****************************************************************************
 * Name: esp32_wlan_softap_initialize
 *
 * Description:
 *   Initialize the esp32 WLAN softAP netcard driver
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

#ifdef ESP32_WLAN_HAS_SOFTAP
int esp32_wlan_softap_initialize(void);
#endif

#endif /* CONFIG_ESP32_WIFI */
#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_SRC_ESP32_ESP32_WLAN_H */
