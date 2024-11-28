/****************************************************************************
 * arch/xtensa/src/common/espressif/esp_wireless.h
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

#ifndef __ARCH_XTENSA_SRC_COMMON_ESPRESSIF_ESP_WIRELESS_H
#define __ARCH_XTENSA_SRC_COMMON_ESPRESSIF_ESP_WIRELESS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <semaphore.h>
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/config.h>
#include <nuttx/list.h>

#ifdef CONFIG_ARCH_CHIP_ESP32
#include "xtensa_attr.h"
#include "esp32_rt_timer.h"
#elif CONFIG_ARCH_CHIP_ESP32S2
#include "esp_attr.h"
#include "esp32s2_rt_timer.h"
#elif CONFIG_ARCH_CHIP_ESP32S3
#include "esp_attr.h"
#include "esp32s3_rt_timer.h"
#endif

#include "esp_log.h"
#include "esp_mac.h"
#include "esp_private/phy.h"
#include "esp_private/wifi.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"
#include "soc/soc_caps.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Note: Don't remove these definitions, they are needed by the 3rdparty IDF
 * headers
 */

#ifdef CONFIG_ARCH_CHIP_ESP32
#  ifdef CONFIG_ESP32_SUPPORT_MULTIPLE_PHY_INIT_DATA
#    undef CONFIG_ESP32_SUPPORT_MULTIPLE_PHY_INIT_DATA_BIN
#    define CONFIG_ESP32_SUPPORT_MULTIPLE_PHY_INIT_DATA_BIN 1
#  endif
#  define SOC_COEX_HW_PTI                               0
#endif

#define CONFIG_MAC_BB_PD                              (0)
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
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nuttx_err_to_freertos
 *
 * Description:
 *   Transform from Nuttx OS error code to FreeRTOS's pdTRUE or pdFALSE.
 *
 * Input Parameters:
 *   ret - NuttX error code
 *
 * Returned Value:
 *   Wi-Fi adapter error code
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_CHIP_ESP32S2
static inline int32_t nuttx_err_to_freertos(int ret)
{
  return ret >= 0;
}
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

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

#ifndef CONFIG_ARCH_CHIP_ESP32S2
int32_t esp_wifi_to_errno(int err);
#endif

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
 * Name: esp_phy_update_country_info
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

int esp_phy_update_country_info(const char *country);

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
 *   len    - Queue cache max length
 *   size   - Queue cache buffer size
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_CHIP_ESP32
void esp_init_queuecache(struct esp_queuecache_s *qc,
                         struct file *mq_ptr,
                         uint8_t *buffer,
                         size_t len,
                         size_t size);
#else
void esp_init_queuecache(struct esp_queuecache_s *qc,
                         struct file *mq_ptr,
                         uint8_t *buffer,
                         size_t size);
#endif

/****************************************************************************
 * Name: esp_send_queuecache
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

#ifdef CONFIG_ARCH_CHIP_ESP32
void esp_send_queuecache(void *qc, uint8_t *buffer, int size);
#else
void esp_send_queuecache(struct esp_queuecache_s *qc,
                         uint8_t *buffer,
                         int size);
#endif

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

#endif /* __ARCH_XTENSA_SRC_COMMON_ESPRESSIF_ESP_WIRELESS_H */
