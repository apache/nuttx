/****************************************************************************
 * arch/xtensa/src/esp32s2/esp32s2_clockconfig.c
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

#include "xtensa.h"
#include "xtensa_attr.h"
#include "hardware/esp32s2_soc.h"
#include "hardware/esp32s2_uart.h"
#include "hardware/esp32s2_rtccntl.h"
#include "hardware/esp32s2_system.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef CONFIG_ESP_CONSOLE_UART_NUM
#define CONFIG_ESP_CONSOLE_UART_NUM 0
#endif

#define DEFAULT_CPU_FREQ  80

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum cpu_freq_e
{
  CPU_80M = 0,
  CPU_160M = 1,
  CPU_240M = 2,
};

enum cpu_clksrc_e
{
  XTAL_CLK,
  PLL_CLK,
  RTC8M_CLK,
  APLL_CLK
};

enum pll_freq_e
{
  PLL_320,
  PLL_480
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s2_cpuclksrc
 *
 * Description:
 *   Select a clock source for CPU clock.
 *
 * Input Parameters:
 *   src             - Any source from cpu_clksrc_e.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void esp32s2_cpuclksrc(enum cpu_clksrc_e src)
{
  uint32_t value;
  value = VALUE_TO_FIELD(src, SYSTEM_SOC_CLK_SEL);
  modifyreg32(SYSTEM_SYSCLK_CONF_REG, SYSTEM_SOC_CLK_SEL_M, value);
}

/****************************************************************************
 * Name: esp32s2_cpudiv
 *
 * Description:
 *   Select a divider for the CPU clk.
 *   NOTE: The divider is not necessarily the real divisor. See TRM for the
 *   equivalences.
 *
 * Input Parameters:
 *   divider          - A value between 0 to 2.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void esp32s2_cpudiv(uint8_t divider)
{
  uint32_t value;
  value = VALUE_TO_FIELD(divider,  SYSTEM_CPUPERIOD_SEL);
  modifyreg32(SYSTEM_CPU_PER_CONF_REG,  SYSTEM_CPUPERIOD_SEL_M, value);
}

/****************************************************************************
 * Name: esp32s2_pllfreqsel
 *
 * Description:
 *   Select the PLL frequency.
 *
 * Input Parameters:
 *   freq             - Any clock from enum pll_freq_e
 *
 * Returned Value:
 *   None
 ****************************************************************************/

static inline void esp32s2_pllfreqsel(enum pll_freq_e freq)
{
  uint32_t value;
  value = VALUE_TO_FIELD(freq, SYSTEM_PLL_FREQ_SEL);
  modifyreg32(SYSTEM_CPU_PER_CONF_REG, SYSTEM_PLL_FREQ_SEL_M, value);
}

/****************************************************************************
 * Name: esp32s2_uart_tx_wait_idle
 *
 * Description:
 *   Wait until uart tx full empty and the last char send ok.
 *
 * Input Parameters:
 *   uart_no   - 0 for UART0, 1 for UART1, 2 for UART2
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void esp32s2_uart_tx_wait_idle(uint8_t uart_no)
{
  uint32_t status;
  do
    {
      status = getreg32(UART_STATUS_REG(uart_no));

      /* tx count is non-zero */
    }
  while ((status & UART_TXFIFO_CNT_M) != 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

extern uint32_t g_ticks_per_us;

/****************************************************************************
 * Name:  esp32s2_update_cpu_freq
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

void IRAM_ATTR esp32s2_update_cpu_freq(uint32_t ticks_per_us)
{
  /* Update scale factors used by esp_rom_delay_us */

  g_ticks_per_us = ticks_per_us;
}

/****************************************************************************
 * Name: esp32s2_set_cpu_freq
 *
 * Description:
 *   Switch to one of PLL-based frequencies.
 *
 * Input Parameters:
 *   cpu_freq_mhz     - Target CPU frequency
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void IRAM_ATTR esp32s2_set_cpu_freq(int cpu_freq_mhz)
{
  uint32_t dbias;
  uint32_t value;
  switch (cpu_freq_mhz)
    {
      case 80:
        /* 80 MHz is obtained from the 480 MHz PLL.
         * In this case CPU_CLK = PLL_CLK / 6.  Config the PLL as 480 MHz
         * with a 6 divider and set the source clock as PLL_CLK.
         */

        dbias = DIG_DBIAS_80M_160M;
        esp32s2_cpudiv(0);
        break;

      case 160:
        /* 160 MHz is obtained from the 480 MHz PLL.
         * In this case CPU_CLK = PLL_CLK / 3.  Config the PLL as 480 MHz
         * with a 3 divider and set the source clock as PLL_CLK.
         */

        dbias = DIG_DBIAS_80M_160M;
        esp32s2_cpudiv(1);
        break;

      case 240:
        /* 160 MHz is obtained from the 480 MHz PLL.
         * In this case CPU_CLK = PLL_CLK / 2.  Config the PLL as 480 MHz
         * with a 2 divider and set the source clock as PLL_CLK.
         */

        dbias = DIG_DBIAS_240M;
        esp32s2_cpudiv(2);
        break;

      default:

        /* Unsupported clock config. */

        return;
    }

  value = (((80 * MHZ) >> 12) & UINT16_MAX) |
          ((((80 * MHZ) >> 12) & UINT16_MAX) << 16);
  esp32s2_pllfreqsel(PLL_480);
  REG_SET_FIELD(RTC_CNTL_REG, RTC_CNTL_DIG_DBIAS_WAK, dbias);
  esp32s2_cpuclksrc(PLL_CLK);
  putreg32(value, RTC_APB_FREQ_REG);
  esp32s2_update_cpu_freq(cpu_freq_mhz);
}

/****************************************************************************
 * Name: esp32s2_clockconfig
 *
 * Description:
 *   Called to initialize the ESP32S2.  This does whatever setup is needed to
 *   put the  SoC in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.
 *
 ****************************************************************************/

void esp32s2_clockconfig(void)
{
  /* Wait for the TX FIFO to unload data */

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
  esp32s2_uart_tx_wait_idle(0);
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
  esp32s2_uart_tx_wait_idle(1);
#endif

  /* Configure the CPU frequency */

  esp32s2_set_cpu_freq(CONFIG_ESP32S2_DEFAULT_CPU_FREQ_MHZ);
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
  return g_ticks_per_us * MHZ;
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
  return MIN(g_ticks_per_us, 80) * MHZ;
}

