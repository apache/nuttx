/****************************************************************************
 * arch/risc-v/src/esp32c6/esp32c6_clockconfig.c
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
#include <sys/param.h>
#include <stdint.h>

#include "riscv_internal.h"
#include "hardware/esp32c6_soc.h"
#include "hardware/esp32c6_pcr.h"

#include "esp32c6_attr.h"
#include "esp32c6_clockconfig.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum cpu_clksrc_e
{
  XTAL_CLK,
  PLL_CLK,
  FOSC_CLK
};

/****************************************************************************
 * ROM Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ets_update_cpu_frequency
 *
 * Description:
 *   Set the real CPU ticks per us to the ets, so that ets_delay_us will be
 *   accurate. Call this function when CPU frequency is changed.
 *
 * Input Parameters:
 *   ticks_per_us - CPU ticks per us.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

extern void ets_update_cpu_frequency(uint32_t ticks_per_us);

/****************************************************************************
 * Name: ets_get_cpu_frequency
 *
 * Description:
 *   Get the real CPU ticks per us to the ets.
 *   This function do not return real CPU ticks per us, just the record in
 *   ets. It can be used to check with the real CPU frequency.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   CPU ticks per us record in ets.
 *
 ****************************************************************************/

extern uint32_t ets_get_cpu_frequency(void);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c6_cpuclksrc
 *
 * Description:
 *   Select a clock source for CPU clock.
 *
 * Input Parameters:
 *   src - Any source from cpu_clksrc_e.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void esp32c6_cpuclksrc(enum cpu_clksrc_e src)
{
  uint32_t value;
  value = VALUE_TO_FIELD(src, PCR_SOC_CLK_SEL);
  modifyreg32(PCR_SYSCLK_CONF_REG, PCR_SOC_CLK_SEL_M, value);
}

/****************************************************************************
 * Name: esp32c6_cpudiv
 *
 * Description:
 *   Select a divider for the CPU clk.
 *   NOTE: The divider is not necessarily the real divisor. See TRM for the
 *   equivalences.
 *
 * Input Parameters:
 *   divider - A value between 3 to 6.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void esp32c6_cpudiv(uint8_t divider)
{
  uint32_t value = (divider / 3) - 1;
  bool force_120m = (divider == 4) ? 1 : 0;
  value = VALUE_TO_FIELD(value, PCR_CPU_HS_DIV_NUM);
  modifyreg32(PCR_CPU_FREQ_CONF_REG, PCR_CPU_HS_DIV_NUM_M, value);
  value = VALUE_TO_FIELD(force_120m, PCR_CPU_HS_120M_FORCE);
  modifyreg32(PCR_CPU_FREQ_CONF_REG, PCR_CPU_HS_120M_FORCE_M, value);
}

/****************************************************************************
 * Public Functions
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
 *   None
 *
 ****************************************************************************/

void IRAM_ATTR esp32c6_update_cpu_freq(uint32_t ticks_per_us)
{
  /* Update scale factors used by esp_rom_delay_us */

  ets_update_cpu_frequency(ticks_per_us);
}

/****************************************************************************
 * Name: esp32c6_set_cpu_freq
 *
 * Description:
 *   Switch to one of PLL-based frequencies.
 *
 * Input Parameters:
 *   cpu_freq_mhz - Target CPU frequency
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void IRAM_ATTR esp32c6_set_cpu_freq(int cpu_freq_mhz)
{
  switch (cpu_freq_mhz)
    {
      case 80:
        /* 80 MHz is obtained from the 480 MHz PLL.
         * In this case CPU_CLK = PLL_CLK / 6.  Config the PLL as 480 MHz
         * with a 6 divider and set the source clock as PLL_CLK.
         */

        esp32c6_cpudiv(6);
        break;

      case 120:
        /* 120 MHz is obtained from the 480 MHz PLL.
         * In this case CPU_CLK = PLL_CLK / 4.  Config the PLL as 480 MHz
         * with a 4 divider and set the source clock as PLL_CLK.
         */

        esp32c6_cpudiv(4);
        break;

      case 160:
        /* 160 MHz is obtained from the 480 MHz PLL.
         * In this case CPU_CLK = PLL_CLK / 3.  Config the PLL as 480 MHz
         * with a 3 divider and set the source clock as PLL_CLK.
         */

        esp32c6_cpudiv(3);
        break;

      default:

        /* Unsupported clock config. */

        return;
    }

  esp32c6_cpuclksrc(PLL_CLK);
  esp32c6_update_cpu_freq(cpu_freq_mhz);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

void esp32c6_clockconfig(void)
{
  /* Configure the CPU frequency */

  esp32c6_set_cpu_freq(CONFIG_ESP32C6_CPU_FREQ_MHZ);
}

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

int IRAM_ATTR esp_clk_cpu_freq(void)
{
  return (int)ets_get_cpu_frequency() * MHZ;
}

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

int IRAM_ATTR esp_clk_apb_freq(void)
{
  return MIN(ets_get_cpu_frequency(), 80) * MHZ;
}
