/****************************************************************************
 * arch/risc-v/src/esp32c6/esp32c6_clockconfig.h
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

#ifndef __ARCH_RISCV_SRC_ESP32C6_ESP32C6_CLOCKCONFIG_H
#define __ARCH_RISCV_SRC_ESP32C6_ESP32C6_CLOCKCONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name:  esp32c6_update_cpu_freq
 *
 * Description:
 *   Set the real CPU ticks per us to the ets, so that ets_delay_us
 *   will be accurate. Call this function when CPU frequency is changed.
 *
 * Input Parameters:
 *   ticks_per_us - CPU ticks per us
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32c6_update_cpu_freq(uint32_t ticks_per_us);

/****************************************************************************
 * Name: esp32c6_set_cpu_freq
 *
 * Description:
 *   Switch to one of PLL-based frequencies.
 *   Current frequency can be XTAL or PLL.
 *
 * Input Parameters:
 *   cpu_freq_mhz - new CPU frequency
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32c6_set_cpu_freq(int cpu_freq_mhz);

/****************************************************************************
 * Name: esp32c6_clockconfig
 *
 * Description:
 *   Called to initialize the ESP32-C6. This does whatever setup is needed to
 *   put the SoC in a usable state.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32c6_clockconfig(void);

/****************************************************************************
 * Name:  esp_clk_cpu_freq
 *
 * Description:
 *   Get the current CPU frequency.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   CPU frequency in Hz.
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
 *   None.
 *
 * Returned Value:
 *   APB clock frequency in Hz.
 *
 ****************************************************************************/

int esp_clk_apb_freq(void);

#endif /* __ARCH_RISCV_SRC_ESP32C6_ESP32C6_CLOCKCONFIG_H */
