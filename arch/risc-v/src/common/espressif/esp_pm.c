/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_pm.c
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

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/power/pm.h>

#include <debug.h>

#include "esp_pm.h"
#ifdef CONFIG_SCHED_TICKLESS
#  include "esp_tickless.h"
#endif
#include "esp_sleep.h"
#include "soc/rtc.h"
#include "esp_sleep_internal.h"
#include "esp_pmu.h"
#ifdef CONFIG_PM_EXT1_WAKEUP
#  include "driver/rtc_io.h"
#endif
#ifdef CONFIG_PM_GPIO_WAKEUP
#  include "driver/gpio.h"
#  include "espressif/esp_gpio.h"
#  include "hal/gpio_types.h"
#endif
#ifdef CONFIG_PM_UART_WAKEUP
#  include "driver/uart_wakeup.h"
#  include "hal/uart_types.h"
#  include "hal/uart_hal.h"
#  include "hal/uart_ll.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_ARCH_CHIP_ESP32C3_GENERIC) || \
    defined(CONFIG_ARCH_CHIP_ESP32C6)
#  define CHECK_VDD_SPI 1
#  if defined(CONFIG_ARCH_CHIP_ESP32C6)
/* GPIO20 - GPIO26 are powered by VDD_SPI Powered */
#    define CHECK_VDD_SPI_PIN_MASKS 133169152
#  else
/* GPIO19 - GPIO24 are powered by VDD_SPI Powered */
#    define CHECK_VDD_SPI_PIN_MASKS 33030144
#  endif /* CONFIG_ARCH_CHIP_ESP32C6 */
#endif /* CONFIG_ARCH_CHIP_ESP32C3_GENERIC || CONFIG_ARCH_CHIP_ESP32C6 */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Wakeup reasons string. */

const char *g_wakeup_reasons[] =
{
  "undefined",
  "",
  "",
  "EXT1",
  "Timer",
  "",
  "ULP",
  "GPIO",
  "UART",
  "",
  "",
  "",
  "",
  "",
  ""
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_PM_EXT1_WAKEUP
/****************************************************************************
 * Name: esp_pm_get_ext1_io_mask
 *
 * Description:
 *   Get ext1 IO mask value from configured wakeup pins.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   A 64-bit unsigned integer where each bit corresponds to an RTC GPIO pin
 *   that has been configured as an ext1 wakeup source.
 *
 ****************************************************************************/

static uint64_t IRAM_ATTR esp_pm_get_ext1_io_mask(void)
{
  uint64_t io_mask = 0;
#ifdef CONFIG_PM_EXT1_WAKEUP_RTC_GPIO0
  io_mask |= BIT(0);
#endif
#ifdef CONFIG_PM_EXT1_WAKEUP_RTC_GPIO1
  io_mask |= BIT(1);
#endif
#ifdef CONFIG_PM_EXT1_WAKEUP_RTC_GPIO2
  io_mask |= BIT(2);
#endif
#ifdef CONFIG_PM_EXT1_WAKEUP_RTC_GPIO3
  io_mask |= BIT(3);
#endif
#ifdef CONFIG_PM_EXT1_WAKEUP_RTC_GPIO4
  io_mask |= BIT(4);
#endif
#ifdef CONFIG_PM_EXT1_WAKEUP_RTC_GPIO5
  io_mask |= BIT(5);
#endif
#ifdef CONFIG_PM_EXT1_WAKEUP_RTC_GPIO6
  io_mask |= BIT(6);
#endif
#ifdef CONFIG_PM_EXT1_WAKEUP_RTC_GPIO7
  io_mask |= BIT(7);
#endif
#ifdef CONFIG_PM_EXT1_WAKEUP_RTC_GPIO8
  io_mask |= BIT(8);
#endif
#ifdef CONFIG_PM_EXT1_WAKEUP_RTC_GPIO9
  io_mask |= BIT(9);
#endif
#ifdef CONFIG_PM_EXT1_WAKEUP_RTC_GPIO10
  io_mask |= BIT(10);
#endif
#ifdef CONFIG_PM_EXT1_WAKEUP_RTC_GPIO11
  io_mask |= BIT(11);
#endif
#ifdef CONFIG_PM_EXT1_WAKEUP_RTC_GPIO12
  io_mask |= BIT(12);
#endif
#ifdef CONFIG_PM_EXT1_WAKEUP_RTC_GPIO13
  io_mask |= BIT(13);
#endif
#ifdef CONFIG_PM_EXT1_WAKEUP_RTC_GPIO14
  io_mask |= BIT(14);
#endif

  return io_mask;
}

/****************************************************************************
 * Name: esp_pm_ext1_wakeup_prepare
 *
 * Description:
 *   Configure ext1 gpios to use as wakeup source.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void IRAM_ATTR esp_pm_ext1_wakeup_prepare(void)
{
  int pin_mask;
  uint64_t io_mask;
#  ifdef CONFIG_PM_EXT1_WAKEUP_TRIGGER_ANY_LOW
  esp_sleep_ext1_wakeup_mode_t level_mode = ESP_EXT1_WAKEUP_ANY_LOW;
#  else
  esp_sleep_ext1_wakeup_mode_t level_mode = ESP_EXT1_WAKEUP_ANY_HIGH;
#  endif /* CONFIG_PM_EXT1_WAKEUP */
  io_mask = esp_pm_get_ext1_io_mask();
  for (int i = 0; i < CONFIG_SOC_GPIO_PIN_COUNT; i++)
    {
      pin_mask = BIT(i);
      if ((io_mask & pin_mask) != 0)
        {
          esp_sleep_enable_ext1_wakeup_io(pin_mask, level_mode);
        }
    }

  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
}
#endif /* CONFIG_PM_EXT1_WAKEUP */

#ifdef CONFIG_PM_GPIO_WAKEUP
/****************************************************************************
 * Name: esp_pm_get_gpio_mask
 *
 * Description:
 *   Get GPIO mask value from configured wakeup pins.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   A 64-bit unsigned integer where each bit corresponds to an GPIO pin
 *   that has been configured as an GPIO wakeup source.
 *
 ****************************************************************************/

static uint64_t IRAM_ATTR esp_pm_get_gpio_mask(void)
{
  uint64_t io_mask = 0;
#ifdef CONFIG_PM_GPIO_WAKEUP_GPIO0
  io_mask |= BIT(0);
#endif
#ifdef CONFIG_PM_GPIO_WAKEUP_GPIO1
  io_mask |= BIT(1);
#endif
#ifdef CONFIG_PM_GPIO_WAKEUP_GPIO2
  io_mask |= BIT(2);
#endif
#ifdef CONFIG_PM_GPIO_WAKEUP_GPIO3
  io_mask |= BIT(3);
#endif
#ifdef CONFIG_PM_GPIO_WAKEUP_GPIO4
  io_mask |= BIT(4);
#endif
#ifdef CONFIG_PM_GPIO_WAKEUP_GPIO5
  io_mask |= BIT(5);
#endif
#ifdef CONFIG_PM_GPIO_WAKEUP_GPIO6
  io_mask |= BIT(6);
#endif
#ifdef CONFIG_PM_GPIO_WAKEUP_GPIO7
  io_mask |= BIT(7);
#endif
#ifdef CONFIG_PM_GPIO_WAKEUP_GPIO8
  io_mask |= BIT(8);
#endif
#ifdef CONFIG_PM_GPIO_WAKEUP_GPIO9
  io_mask |= BIT(9);
#endif
#ifdef CONFIG_PM_GPIO_WAKEUP_GPIO10
  io_mask |= BIT(10);
#endif
#ifdef CONFIG_PM_GPIO_WAKEUP_GPIO11
  io_mask |= BIT(11);
#endif
#ifdef CONFIG_PM_GPIO_WAKEUP_GPIO12
  io_mask |= BIT(12);
#endif
#ifdef CONFIG_PM_GPIO_WAKEUP_GPIO13
  io_mask |= BIT(13);
#endif
#ifdef CONFIG_PM_GPIO_WAKEUP_GPIO14
  io_mask |= BIT(14);
#endif
#ifdef CONFIG_PM_GPIO_WAKEUP_GPIO15
  io_mask |= BIT(15);
#endif
#ifdef CONFIG_PM_GPIO_WAKEUP_GPIO16
  io_mask |= BIT(16);
#endif
#ifdef CONFIG_PM_GPIO_WAKEUP_GPIO17
  io_mask |= BIT(17);
#endif
#ifdef CONFIG_PM_GPIO_WAKEUP_GPIO18
  io_mask |= BIT(18);
#endif
#ifdef CONFIG_PM_GPIO_WAKEUP_GPIO19
  io_mask |= BIT(19);
#endif
#ifdef CONFIG_PM_GPIO_WAKEUP_GPIO20
  io_mask |= BIT(20);
#endif
#ifdef CONFIG_PM_GPIO_WAKEUP_GPIO21
  io_mask |= BIT(21);
#endif
#ifdef CONFIG_PM_GPIO_WAKEUP_GPIO22
  io_mask |= BIT(22);
#endif
#ifdef CONFIG_PM_GPIO_WAKEUP_GPIO23
  io_mask |= BIT(23);
#endif
#ifdef CONFIG_PM_GPIO_WAKEUP_GPIO24
  io_mask |= BIT(24);
#endif
#ifdef CONFIG_PM_GPIO_WAKEUP_GPIO25
  io_mask |= BIT(25);
#endif
#ifdef CONFIG_PM_GPIO_WAKEUP_GPIO26
  io_mask |= BIT(26);
#endif
#ifdef CONFIG_PM_GPIO_WAKEUP_GPIO27
  io_mask |= BIT(27);
#endif
#ifdef CONFIG_PM_GPIO_WAKEUP_GPIO28
  io_mask |= BIT(28);
#endif
#ifdef CONFIG_PM_GPIO_WAKEUP_GPIO29
  io_mask |= BIT(29);
#endif
#ifdef CONFIG_PM_GPIO_WAKEUP_GPIO30
  io_mask |= BIT(30);
#endif

  return io_mask;
}

/****************************************************************************
 * Name: esp_pm_gpio_wakeup_prepare
 *
 * Description:
 *   Configure gpios to use as gpio wakeup source.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void IRAM_ATTR esp_pm_gpio_wakeup_prepare(void)
{
  uint64_t mask_value = esp_pm_get_gpio_mask();
  int pin_mask = 0;
#  ifdef CONFIG_PM_GPIO_WAKEUP_TRIGGER_ANY_LOW
  gpio_int_type_t level_mode = GPIO_INTR_LOW_LEVEL;
#  else
  gpio_int_type_t level_mode = GPIO_INTR_HIGH_LEVEL;
#  endif /* CONFIG_PM_EXT1_WAKEUP */

  for (int i = 0; i < CONFIG_SOC_GPIO_PIN_COUNT; i++)
    {
      pin_mask = BIT(i);
      if ((mask_value & pin_mask) != 0)
        {
          esp_configgpio(i, INPUT);
          gpio_wakeup_enable(i, level_mode);
        }
    }

#ifdef CHECK_VDD_SPI
  if ((mask_value & CHECK_VDD_SPI_PIN_MASKS) != 0)
    {
      esp_sleep_pd_config(ESP_PD_DOMAIN_VDDSDIO, ESP_PD_OPTION_ON);
    }
#endif /* CHECK_VDD_SPI */

  esp_sleep_enable_gpio_wakeup();
}
#endif /* CONFIG_PM_GPIO_WAKEUP */

#ifdef CONFIG_PM_UART_WAKEUP
/****************************************************************************
 * Name: esp_pm_uart_wakeup_prepare
 *
 * Description:
 *   Configure UART wake-up mode
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void IRAM_ATTR esp_pm_uart_wakeup_prepare(void)
{
#  if defined(CONFIG_PM_UART_WAKEUP_UART0)
  int uart_num = 0;
#  elif defined(CONFIG_PM_UART_WAKEUP_UART1)
  int uart_num = 1;
#  endif
  uart_wakeup_cfg_t wake_up_cfg =
    {
#ifdef CONFIG_PM_UART_WAKEUP_ACTIVE_EDGE_THRESHOLD_MODE
      .wakeup_mode = UART_WK_MODE_ACTIVE_THRESH,
      .rx_edge_threshold = CONFIG_PM_UART_WAKEUP_ACTIVE_EDGE_THRESHOLD
#endif
#ifdef CONFIG_PM_UART_WAKEUP_FIFO_THRESHOLD_MODE
      .wakeup_mode = UART_WK_MODE_FIFO_THRESH,
      .rx_fifo_threshold = CONFIG_PM_UART_WAKEUP_FIFO_THRESHOLD
#endif
#ifdef CONFIG_PM_UART_WAKEUP_START_BIT_MODE
      .wakeup_mode = UART_WK_MODE_START_BIT,
#endif
#ifdef CONFIG_PM_UART_WAKEUP_CHAR_SEQ_MODE
      .wakeup_mode = UART_WK_MODE_CHAR_SEQ,
      .wake_chars_seq = CONFIG_PM_UART_WAKEUP_CHAR_SEQ
#endif
    };

  uart_wakeup_setup(uart_num, &wake_up_cfg);
  esp_sleep_enable_uart_wakeup(uart_num);
}
#endif /* CONFIG_PM_UART_WAKEUP */

/****************************************************************************
 * Name: esp_pm_sleep_enable_timer_wakeup
 *
 * Description:
 *   Configure wakeup interval
 *
 * Input Parameters:
 *   time_in_us - Sleep duration in microseconds.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp_pm_sleep_enable_timer_wakeup(uint64_t time_in_us)
{
  esp_sleep_enable_timer_wakeup(time_in_us);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_pm_light_sleep_start
 *
 * Description:
 *   Enter light sleep mode
 *
 * Input Parameters:
 *   sleep_time - Reference of uint64_t value to return actual sleep duration
 *                in microseconds. Use NULL if not needed.
 *
 * Returned Value:
 *   OK on success or a negated errno value if fails.
 *
 ****************************************************************************/

int esp_pm_light_sleep_start(uint64_t *sleep_time)
{
  int ret = OK;
  int sleep_start = rtc_time_get();
  int sleep_return = 0;

  ret = esp_light_sleep_start();
  if (sleep_time != NULL)
    {
      sleep_return = rtc_time_get();
      *sleep_time = sleep_return - sleep_start;
    }

  return ret;
}

/****************************************************************************
 * Name:  esp_pm_deep_sleep_start
 *
 * Description:
 *   Enter deep sleep mode
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp_pm_deep_sleep_start(void)
{
  esp_deep_sleep_start();

  /* Because RTC is in a slower clock domain than the CPU, it
   * can take several CPU cycles for the sleep mode to start.
   */

  while (1);
}

/****************************************************************************
 * Name: esp_pmstandby
 *
 * Description:
 *   Enter pm standby (light sleep) mode.
 *
 * Input Parameters:
 *   time_in_us - The maximum time to sleep in microseconds.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp_pmstandby(uint64_t time_in_us)
{
  uint64_t rtc_diff_us;
  esp_sleep_wakeup_cause_t cause;
#ifdef CONFIG_PM_GPIO_WAKEUP
  int64_t gpio_mask;
#endif
#ifdef CONFIG_PM_EXT1_WAKEUP
  int64_t ext1_mask;
  esp_pm_ext1_wakeup_prepare();
#endif
#ifdef CONFIG_PM_GPIO_WAKEUP
  esp_pm_gpio_wakeup_prepare();
#endif
#ifdef CONFIG_PM_ULP_WAKEUP
  esp_sleep_enable_ulp_wakeup();
#endif
#ifdef CONFIG_PM_UART_WAKEUP
  esp_pm_uart_wakeup_prepare();
#endif /* CONFIG_PM_UART_WAKEUP */

  esp_pm_sleep_enable_timer_wakeup(time_in_us);

  esp_pm_light_sleep_start(&rtc_diff_us);

#ifdef CONFIG_SCHED_TICKLESS
  up_step_idletime((uint32_t)time_in_us);
#endif

  cause = esp_sleep_get_wakeup_cause();
  pwrinfo("Returned from light-sleep with: %s, slept for %" PRIu32 " ms\n",
          g_wakeup_reasons[cause],
          (uint32_t)(rtc_diff_us) / 1000);

#ifdef CONFIG_PM_EXT1_WAKEUP
  if (cause == ESP_SLEEP_WAKEUP_EXT1)
    {
      ext1_mask = esp_sleep_get_ext1_wakeup_status();
      pwrinfo("EXT1 wakeup mask: %" PRIu64 "\n", ext1_mask);
    }
#endif

#ifdef CONFIG_PM_GPIO_WAKEUP
  if (cause == ESP_SLEEP_WAKEUP_GPIO)
    {
      gpio_mask = esp_sleep_get_gpio_wakeup_status();
      pwrinfo("GPIO wakeup mask: %" PRIu64 "\n", gpio_mask);
    }
#endif
}

/****************************************************************************
 * Name: esp_pmsleep
 *
 * Description:
 *   Enter pm sleep (deep sleep) mode.
 *
 * Input Parameters:
 *   time_in_us - The maximum time to sleep in microseconds.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp_pmsleep(uint64_t time_in_us)
{
#ifdef CONFIG_PM_EXT1_WAKEUP
  esp_pm_ext1_wakeup_prepare();
#endif
#ifdef CONFIG_PM_ULP_WAKEUP
  esp_sleep_enable_ulp_wakeup();
#endif
  esp_pm_sleep_enable_timer_wakeup(time_in_us);
  esp_pm_deep_sleep_start();
}
