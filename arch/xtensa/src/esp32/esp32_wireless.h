/****************************************************************************
 * arch/xtensa/src/esp32/esp32_wireless.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32_ESP32_WIRELESS_H
#define __ARCH_XTENSA_SRC_ESP32_ESP32_WIRELESS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "xtensa_attr.h"

#include "espidf_wifi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Note: Don't remove these definitions, they are needed by the 3rdparty IDF
 * headers
 */

#ifdef CONFIG_ESP32_SUPPORT_MULTIPLE_PHY_INIT_DATA
#  undef CONFIG_ESP32_SUPPORT_MULTIPLE_PHY_INIT_DATA_BIN
#  define CONFIG_ESP32_SUPPORT_MULTIPLE_PHY_INIT_DATA_BIN 1
#endif
#define CONFIG_MAC_BB_PD                                0
#define SOC_COEX_HW_PTI                                 0

#define MAC_LEN                                       (6)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp_read_mac
 *
 * Description:
 *   Read MAC address from efuse
 *
 * Input Parameters:
 *   mac  - MAC address buffer pointer
 *   type - MAC address type
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int32_t esp_read_mac(uint8_t *mac, esp_mac_type_t type);

/****************************************************************************
 * Name: esp32_phy_enable
 *
 * Description:
 *   Initialize PHY hardware
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32_phy_enable(void);

/****************************************************************************
 * Name: esp32_phy_disable
 *
 * Description:
 *   Deinitialize PHY hardware
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32_phy_disable(void);

/****************************************************************************
 * Name: esp32_phy_enable_clock
 *
 * Description:
 *   Enable PHY clock
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32_phy_enable_clock(void);

/****************************************************************************
 * Name: esp32_phy_disable_clock
 *
 * Description:
 *   Disable PHY clock
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32_phy_disable_clock(void);

/****************************************************************************
 * Functions needed by libphy.a
 ****************************************************************************/

/****************************************************************************
 * Name: esp_dport_access_reg_read
 *
 * Description:
 *   Read register value safely in SMP
 *
 * Input Parameters:
 *   reg - Register address
 *
 * Returned Value:
 *   Register value
 *
 ****************************************************************************/

uint32_t IRAM_ATTR esp_dport_access_reg_read(uint32_t reg);

/****************************************************************************
 * Name: phy_enter_critical
 *
 * Description:
 *   Enter critical state
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   CPU PS value
 *
 ****************************************************************************/

uint32_t IRAM_ATTR phy_enter_critical(void);

/****************************************************************************
 * Name: phy_exit_critical
 *
 * Description:
 *   Exit from critical state
 *
 * Input Parameters:
 *   level - CPU PS value
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void IRAM_ATTR phy_exit_critical(uint32_t level);

/****************************************************************************
 * Name: phy_printf
 *
 * Description:
 *   Output format string and its arguments
 *
 * Input Parameters:
 *   format - format string
 *
 * Returned Value:
 *   0
 *
 ****************************************************************************/

int phy_printf(const char *format, ...) printf_like(1, 2);

/****************************************************************************
 * Name: esp32_phy_update_country_info
 *
 * Description:
 *   Update PHY init data according to country code
 *
 * Input Parameters:
 *   country - PHY init data type
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

int esp32_phy_update_country_info(const char *country);

#endif /* __ARCH_XTENSA_SRC_ESP32_ESP32_WIRELESS_H */
