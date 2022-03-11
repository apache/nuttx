/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_clockconfig.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "riscv_internal.h"
#include "hardware/esp32c3_system.h"

#include "esp32c3.h"
#include "esp32c3_clockconfig.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum esp32c3_clksrc_e
{
  ESP32C3_CLKSRC_XTAL = 0,
  ESP32C3_CLKSRC_PLL
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline void esp32c3_clksrc(int src);
static inline void esp32c3_sysprediv(int div);
static inline void esp32c3_pllcfg(int pllfreq, int cpuprd);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_clksrc
 ****************************************************************************/

static inline void esp32c3_clksrc(int src)
{
  uint32_t set = 0;
  uint32_t clear = 0;

  set   |= src << SYSTEM_SOC_CLK_SEL_S;
  clear |= SYSTEM_SOC_CLK_SEL_M;

  modifyreg32(SYSTEM_SYSCLK_CONF_REG, clear, set);
}

/****************************************************************************
 * Name: esp32c3_sysprediv
 ****************************************************************************/

static inline void esp32c3_sysprediv(int div)
{
  uint32_t set   = 0;
  uint32_t clear = 0;

  clear |= SYSTEM_PRE_DIV_CNT_M;
  set   |= div << SYSTEM_PRE_DIV_CNT_S;

  modifyreg32(SYSTEM_SYSCLK_CONF_REG, clear, set);
}

/****************************************************************************
 * Name: esp32c3_pllcfg
 ****************************************************************************/

static inline void esp32c3_pllcfg(int pllfreq, int cpuprd)
{
  uint32_t set   = 0;
  uint32_t clear = 0;

  clear |= SYSTEM_PLL_FREQ_SEL_M | SYSTEM_CPUPERIOD_SEL_M;
  set   |= pllfreq << SYSTEM_PLL_FREQ_SEL_S;
  set   |= cpuprd << SYSTEM_CPUPERIOD_SEL_S;

  modifyreg32(SYSTEM_CPU_PER_CONF_REG, clear, set);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_clockconfig
 ****************************************************************************/

void esp32c3_clockconfig(void)
{
  uint32_t freq_mhz = CONFIG_ESP32C3_CPU_FREQ_MHZ;

  switch (freq_mhz)
    {
      case 40:
        /* 40 MHz is obtained from the XTAL.
         * In this case CPU_CLK = XTAL / (DIV + 1).  Set the divider as 0 and
         * the source as XTAL.
         */

        esp32c3_sysprediv(0);
        esp32c3_clksrc(ESP32C3_CLKSRC_XTAL);
        break;

      case 80:
        /* 80 MHz is obtained from the 480 MHz PLL.
         * In this case CPU_CLK = PLL_CLK / 6.  Config the PLL as 480 MHz
         * with a 6 divider and set the source clock as PLL_CLK.
         */

        esp32c3_pllcfg(1, 0);
        esp32c3_clksrc(ESP32C3_CLKSRC_PLL);
        break;

      case 160:
        /* 160 MHz is obtained from the 480 MHz PLL.
         * In this case CPU_CLK = PLL_CLK / 3.  Config the PLL as 480 MHz
         * with a 3 divider and set the source clock as PLL_CLK.
         */

        esp32c3_pllcfg(1, 1);
        esp32c3_clksrc(ESP32C3_CLKSRC_PLL);
        break;

      default:

        /* Unsupported clock config. */

        return;
    }
}

/****************************************************************************
 * Name:  esp32c3_clk_cpu_freq
 *
 * Description:
 *   Get CPU frequency in Hz.
 *
 ****************************************************************************/

int esp32c3_clk_cpu_freq(void)
{
  return CONFIG_ESP32C3_CPU_FREQ_MHZ * 1000000;
}

/****************************************************************************
 * Name: esp32c3_clk_apb_freq
 *
 * Description:
 *   Returns ABP frequency in Hertz.
 *
 ****************************************************************************/

int esp32c3_clk_apb_freq(void)
{
  uint32_t cpufreq = CONFIG_ESP32C3_CPU_FREQ_MHZ;

  /* APB frequency is 80 MHz if PLL is selected as CPU source, otherwise it's
   * the same as the CPU frequency.
   */

  return (cpufreq == 40) ? (40 * 1000000) : (80 * 1000000);
}

/****************************************************************************
 * Name: esp32c3_clk_crypto_freq
 *
 * Description:
 *   Returns crypto engine frequency in Hertz.
 *
 ****************************************************************************/

int esp32c3_clk_crypto_freq(void)
{
  uint32_t cpufreq = CONFIG_ESP32C3_CPU_FREQ_MHZ;

  /* Crypto frequency is 160 MHz if PLL is selected as CPU source, otherwise
   * it's the same as the CPU frequency.
   */

  return (cpufreq == 40) ? (40 * 1000000) : (160 * 1000000);
}

/****************************************************************************
 * Name: esp32c3_cpu_cycle_count
 *
 * Description:
 *   Get the current value of the internal counter that increments
 *   every processor-clock cycle.
 *
 ****************************************************************************/

uint32_t IRAM_ATTR esp32c3_cpu_cycle_count(void)
{
  uint32_t result;

  result = READ_CSR(CSR_PCCR_MACHINE);
  return result;
}
