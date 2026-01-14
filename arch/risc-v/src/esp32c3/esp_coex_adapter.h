/****************************************************************************
 * arch/risc-v/src/esp32c3/esp_coex_adapter.h
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

#ifndef __ARCH_RISCV_SRC_ESP32C3_ESP_COEX_ADAPTER_H
#define __ARCH_RISCV_SRC_ESP32C3_ESP_COEX_ADAPTER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <inttypes.h>

#include "esp_attr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef __ASSEMBLY__

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

bool IRAM_ATTR esp_coex_common_env_is_chip_wrapper(void);
void *esp_coex_common_spin_lock_create_wrapper(void);
uint32_t IRAM_ATTR esp_coex_common_int_disable_wrapper(void *wifi_int_mux);
void IRAM_ATTR esp_coex_common_int_restore_wrapper(void *wifi_int_mux,
                                                   uint32_t tmp);
void IRAM_ATTR esp_coex_common_task_yield_from_isr_wrapper(void);
void *esp_coex_common_semphr_create_wrapper(uint32_t max, uint32_t init);
void esp_coex_common_semphr_delete_wrapper(void *semphr);
int32_t esp_coex_common_semphr_take_wrapper(void *semphr,
                                            uint32_t block_time_tick);
int32_t esp_coex_common_semphr_give_wrapper(void *semphr);
void IRAM_ATTR esp_coex_common_timer_disarm_wrapper(void *timer);
void esp_coex_common_timer_done_wrapper(void *ptimer);
void esp_coex_common_timer_setfn_wrapper(void *ptimer,
                                         void *pfunction,
                                         void *parg);
void IRAM_ATTR esp_coex_common_timer_arm_us_wrapper(void *ptimer,
                                                    uint32_t us,
                                                    bool repeat);
IRAM_ATTR void *esp_coex_common_malloc_internal_wrapper(size_t size);
uint32_t esp_coex_common_clk_slowclk_cal_get_wrapper(void);

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_ESP32C3_ESP_COEX_ADAPTER_H */
