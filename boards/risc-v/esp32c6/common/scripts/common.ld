/****************************************************************************
 * boards/risc-v/esp32c6/common/scripts/common.ld
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
#include <nuttx/config.h>

#if CONFIG_ESPRESSIF_SOC_RTC_MEM_SUPPORTED
#  define ESP_BOOTLOADER_RESERVE_RTC 0

   /* rtc timer data (s_rtc_timer_retain_mem, see esp_clk.c files).
    * For rtc_timer_data_in_rtc_mem section.
    */
#  define RTC_TIMER_RESERVE_RTC (24)

#  ifdef CONFIG_ARCH_CHIP_ESP32
#    define RESERVE_RTC_MEM (RTC_TIMER_RESERVE_RTC)
#  else
#    define RESERVE_RTC_MEM (ESP_BOOTLOADER_RESERVE_RTC + RTC_TIMER_RESERVE_RTC)
#  endif /* CONFIG_ARCH_CHIP_ESP32 */
#endif /* CONFIG_ESPRESSIF_SOC_RTC_MEM_SUPPORTED */
