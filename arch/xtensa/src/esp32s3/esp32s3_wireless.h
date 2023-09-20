/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_wireless.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_WIRELESS_H
#define __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_WIRELESS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <semaphore.h>
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/config.h>
#include <nuttx/list.h>

#include "xtensa_attr.h"
#include "esp32s3_rt_timer.h"

#include "esp_hal_wifi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Note: Don't remove these definitions, they are needed by the 3rdparty IDF
 * headers
 */

#define CONFIG_ESP32_SUPPORT_MULTIPLE_PHY_INIT_DATA_BIN     0
#define CONFIG_MAC_BB_PD                                    0

#define MAC_LEN                                       (6)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Semaphore Cache Data */

struct esp_semcache_s
{
  struct list_node node;

  sem_t *sem;
  uint32_t count;
};

/* Queue Cache Data */

struct esp_queuecache_s
{
  struct list_node node;

  struct file *mq_ptr;
  size_t size;
  uint8_t *buffer;
};

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
 * Name: esp32s3_phy_enable
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

void esp32s3_phy_enable(void);

/****************************************************************************
 * Name: esp32s3_phy_disable
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

void IRAM_ATTR esp32s3_phy_disable(void);

/****************************************************************************
 * Name: esp32s3_phy_enable_clock
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

void IRAM_ATTR esp32s3_phy_enable_clock(void);

/****************************************************************************
 * Name: esp32s3_phy_disable_clock
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

void esp32s3_phy_disable_clock(void);

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
 * Name: esp_wifi_bt_power_domain_on
 *
 * Description:
 *   Initialize Bluetooth and Wi-Fi power domain
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void IRAM_ATTR esp_wifi_bt_power_domain_on(void);

/****************************************************************************
 * Name: esp_timer_create
 *
 * Description:
 *   Create timer with given arguments
 *
 * Input Parameters:
 *   create_args - Timer arguments data pointer
 *   out_handle  - Timer handle pointer
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int32_t esp_timer_create(const esp_timer_create_args_t *create_args,
                         esp_timer_handle_t *out_handle);

/****************************************************************************
 * Name: esp_timer_start_once
 *
 * Description:
 *   Start timer with one shot mode
 *
 * Input Parameters:
 *   timer      - Timer handle pointer
 *   timeout_us - Timeout value by micro second
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int32_t esp_timer_start_once(esp_timer_handle_t timer, uint64_t timeout_us);

/****************************************************************************
 * Name: esp_timer_start_periodic
 *
 * Description:
 *   Start timer with periodic mode
 *
 * Input Parameters:
 *   timer  - Timer handle pointer
 *   period - Timeout value by micro second
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int32_t esp_timer_start_periodic(esp_timer_handle_t timer, uint64_t period);

/****************************************************************************
 * Name: esp_timer_stop
 *
 * Description:
 *   Stop timer
 *
 * Input Parameters:
 *   timer  - Timer handle pointer
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int32_t esp_timer_stop(esp_timer_handle_t timer);

/****************************************************************************
 * Name: esp_timer_delete
 *
 * Description:
 *   Delete timer and free resource
 *
 * Input Parameters:
 *   timer  - Timer handle pointer
 *
 * Returned Value:
 *   0 if success or -1 if fail
 *
 ****************************************************************************/

int32_t esp_timer_delete(esp_timer_handle_t timer);

/****************************************************************************
 * Name: esp_init_semcache
 *
 * Description:
 *   Initialize semaphore cache.
 *
 * Parameters:
 *   sc  - Semaphore cache data pointer
 *   sem - Semaphore data pointer
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_init_semcache(struct esp_semcache_s *sc, sem_t *sem);

/****************************************************************************
 * Name: esp_post_semcache
 *
 * Description:
 *   Store posting semaphore action into semaphore cache.
 *
 * Parameters:
 *   sc  - Semaphore cache data pointer
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_post_semcache(struct esp_semcache_s *sc);

/****************************************************************************
 * Name: esp_init_queuecache
 *
 * Description:
 *   Initialize queue cache.
 *
 * Parameters:
 *   qc     - Queue cache data pointer
 *   mq_ptr - Queue data pointer
 *   buffer - Queue cache buffer pointer
 *   size   - Queue cache buffer size
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_init_queuecache(struct esp_queuecache_s *qc,
                         struct file *mq_ptr,
                         uint8_t *buffer,
                         size_t size);

/****************************************************************************
 * Name: esp32_wl_send_queuecache
 *
 * Description:
 *   Store posting queue action and data into queue cache.
 *
 * Parameters:
 *   qc     - Queue cache data pointer
 *   buffer - Data buffer
 *   size   - Buffer size
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_send_queuecache(struct esp_queuecache_s *qc,
                         uint8_t *buffer,
                         int size);

/****************************************************************************
 * Name: esp_wireless_init
 *
 * Description:
 *   Initialize ESP32 wireless common components for both BT and Wi-Fi.
 *
 * Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int esp_wireless_init(void);

/****************************************************************************
 * Name: esp_wireless_deinit
 *
 * Description:
 *   De-initialize ESP32 wireless common components.
 *
 * Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

int esp_wireless_deinit(void);

#endif /* __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_WIRELESS_H */
