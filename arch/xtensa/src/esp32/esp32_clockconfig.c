/****************************************************************************
 * arch/xtensa/src/esp32/esp32_clockconfig.C
 *
 * Mofidifed by use in NuttX by:
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Derives from software originally provided by Expressif Systems:
 *
 * Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include "xtensa.h"

#ifndef CONFIG_SUPPRESS_CLOCK_CONFIG
#warning REVISIT ... function prototypes

void phy_get_romfunc_addr(void);
void rtc_init_lite(void);
void rtc_set_cpu_freq(xtal_freq_t xtal_freq, enum xtal_freq_e cpu_freq);
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifndef CONFIG_SUPPRESS_CLOCK_CONFIG
enum xtal_freq_e
{
  XTAL_40M = 40,
  XTAL_26M = 26,
  XTAL_24M = 24,
  XTAL_AUTO = 0
};

enum xtal_freq_e
{
  CPU_80M = 1,
  CPU_160M = 2,
  CPU_240M = 3,
};
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_clockconfig
 *
 * Description:
 *   Called to initialize the ESP32.  This does whatever setup is needed to
 *   put the  SoC in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.
 *
 ****************************************************************************/

void esp32_clockconfig(void)
{
#ifdef CONFIG_SUPPRESS_CLOCK_CONFIG
#  warning WARNING: Clock configuration disabled
#else
  uint32_t freq_mhz = CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ;
  enum xtal_freq_e freq;

  phy_get_romfunc_addr();

  /* Frequency will be changed to 40MHz in rtc_init_lite */

  rtc_init_lite();

  freq = CPU_80M;
  switch (freq_mhz)
    {
    case 240:
      freq = CPU_240M;
      break;
    case 160:
      freq = CPU_160M;
      break;
    default:
      freq_mhz = 80;
      /* no break */
    case 80:
      freq = CPU_80M;
      break;
    }

  /* Frequency will be changed to freq in rtc_set_cpu_freq */

  rtc_set_cpu_freq(XTAL_AUTO, freq);
  ets_update_cpu_frequency(freq_mhz);
#endif
}
