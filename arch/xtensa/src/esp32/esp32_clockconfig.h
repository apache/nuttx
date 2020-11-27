/****************************************************************************
 * arch/xtensa/src/esp32/esp32_clockconfig.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
