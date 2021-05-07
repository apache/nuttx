/****************************************************************************
 * arch/xtensa/src/esp32/esp32_clockconfig.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32_ESP32_CLOCKCONFIG_H
#define __ARCH_XTENSA_SRC_ESP32_ESP32_CLOCKCONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name:  esp32_update_cpu_freq
 *
 * Description:
 *   Set the real CPU ticks per us to the ets, so that ets_delay_us
 *   will be accurate. Call this function when CPU frequency is changed.
 *
 * Input Parameters:
 *   ticks_per_us - CPU ticks per us
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32_update_cpu_freq(uint32_t ticks_per_us);

/****************************************************************************
 * Name: esp32_set_cpu_freq
 *
 * Description:
 *   Switch to one of PLL-based frequencies.
 *   Current frequency can be XTAL or PLL.
 *
 * Input Parameters:
 *   cpu_freq_mhz      - new CPU frequency
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32_set_cpu_freq(int cpu_freq_mhz);

/****************************************************************************
 * Name: esp32_clockconfig
 *
 * Description:
 *   Called to initialize the ESP32.  This does whatever setup is needed to
 *   put the  SoC in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.
 *
 ****************************************************************************/

void esp32_clockconfig(void);

/****************************************************************************
 * Name:  esp_clk_cpu_freq
 *
 * Description:
 *   Get CPU frequency
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   CPU frequency
 *
 ****************************************************************************/

int esp_clk_cpu_freq(void);

/****************************************************************************
 * Name:  esp_clk_apb_freq
 *
 * Description:
 *   Return current APB clock frequency.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   APB clock frequency, in Hz
 *
 ****************************************************************************/

int esp_clk_apb_freq(void);

#endif /* __ARCH_XTENSA_SRC_ESP32_ESP32_CLOCKCONFIG_H */
