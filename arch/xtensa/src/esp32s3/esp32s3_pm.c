/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_pm.c
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

#ifdef CONFIG_PM

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <sys/param.h>
#include <sys/types.h>
#include <arch/xtensa/core_macros.h>

#include "chip.h"
#include "xtensa.h"
#include "hardware/esp32s3_rtccntl.h"
#include "hardware/esp32s3_system.h"
#include "hardware/esp32s3_syscon.h"
#include "hardware/esp32s3_soc.h"
#include "hardware/esp32s3_uart.h"
#include "hardware/esp32s3_gpio.h"

#include "esp32s3_rtc.h"
#include "esp32s3_pm.h"

#include "soc/periph_defs.h"
#include "hal/clk_gate_ll.h"
#include "esp_private/esp_clk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If light sleep time is less than that, don't power down flash */

#define FLASH_PD_MIN_SLEEP_TIME_US         (2000)

/* Minimal amount of time we can sleep for. */

#define LIGHT_SLEEP_MIN_TIME_US            (200)

#define RTC_MODULE_SLEEP_PREPARE_CYCLES    (6)

/* Time from VDD_SDIO power up to first flash read in ROM code */

#define VDD_SDIO_POWERUP_TO_FLASH_READ_US  (700)

/* Extra time it takes to enter and exit light sleep and deep sleep */

#define LIGHT_SLEEP_TIME_OVERHEAD_US       (133)

#ifdef CONFIG_ESP32S3_DEFAULT_CPU_FREQ_MHZ
#define DEFAULT_CPU_FREQ_MHZ CONFIG_ESP32S3_DEFAULT_CPU_FREQ_MHZ
#else
#define DEFAULT_CPU_FREQ_MHZ               (160)
#endif

#define DEFAULT_SLEEP_OUT_OVERHEAD_US      (382)
#define DEEP_SLEEP_TIME_OVERHEAD_US        (250 + 100 * 240 / DEFAULT_CPU_FREQ_MHZ)

#define DEEP_SLEEP_WAKEUP_DELAY            (2000)

#define RTC_VDDSDIO_TIEH_1_8V     0  /* TIEH field value for 1.8V VDDSDIO */
#define RTC_VDDSDIO_TIEH_3_3V     1  /* TIEH field value for 3.3V VDDSDIO */

#define RTC_GPIO_TRIG_EN          BIT(2)  /* GPIO wakeup */
#define RTC_TIMER_TRIG_EN         BIT(3)  /* Timer wakeup */
#define RTC_WIFI_TRIG_EN          BIT(5)  /* Wi-Fi wakeup (light sleep only) */
#define RTC_UART0_TRIG_EN         BIT(6)  /* UART0 wakeup (light sleep only) */
#define RTC_UART1_TRIG_EN         BIT(7)  /* UART1 wakeup (light sleep only) */
#define RTC_BT_TRIG_EN            BIT(10) /* BT wakeup (light sleep only) */
#define RTC_XTAL32K_DEAD_TRIG_EN  BIT(12)
#define RTC_USB_TRIG_EN           BIT(14)
#define RTC_BROWNOUT_DET_TRIG_EN  BIT(16)

#define PERIPH_INFORM_OUT_SLEEP_OVERHEAD_NO (1)
#define PERIPH_SKIP_SLEEP_NO                (1)

#define UART_FSM_IDLE                       (0x0)
#define UART_FSM_TX_WAIT_SEND               (0xf)

#define ESP32S3_NUARTS                      (3) /* UART0-2 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Power down options */

enum esp32s3_sleep_pd_option_e
{
  /* Power down the power domain in sleep mode */

  ESP_PD_OPTION_OFF,

  /* Keep power domain enabled during sleep mode */

  ESP_PD_OPTION_ON,

  /* Keep power domain enabled in sleep mode if it is needed
   * by one of the wakeup options, otherwise power it down.
   */

  ESP_PD_OPTION_AUTO
};

/* Power domains which can be powered down in sleep mode. */

enum esp32s3_sleep_pd_domain_e
{
  ESP_PD_DOMAIN_RTC_PERIPH = 0,  /* RTC IO, sensors */
  ESP_PD_DOMAIN_RTC_SLOW_MEM,    /* RTC slow memory */
  ESP_PD_DOMAIN_RTC_FAST_MEM,    /* RTC fast memory */
  ESP_PD_DOMAIN_XTAL,            /* XTAL oscillator */
  ESP_PD_DOMAIN_CPU,             /* CPU core */
  ESP_PD_DOMAIN_RTC8M,           /* Internal 8M oscillator */
  ESP_PD_DOMAIN_VDDSDIO,         /* VDD_SDIO */
  ESP_PD_DOMAIN_MAX              /* Number of domains */
};

/* Internal structure which holds all requested deep sleep parameters. */

struct esp32s3_sleep_config_s
{
  enum esp32s3_sleep_pd_option_e pd_options[ESP_PD_DOMAIN_MAX];
  uint64_t sleep_duration;
  uint32_t wakeup_triggers : 15;
  uint32_t ext1_trigger_mode : 1;
  uint32_t ext1_rtc_gpio_mask : 22;
  uint32_t ext0_trigger_level : 1;
  uint32_t ext0_rtc_gpio_num : 5;
  uint32_t gpio_wakeup_mask : 6;
  uint32_t gpio_trigger_mode : 6;
  uint32_t sleep_time_adjustment;
  uint32_t ccount_ticks_record;
  uint32_t sleep_time_overhead_out;
  uint32_t rtc_clk_cal_period;
  uint64_t rtc_ticks_at_sleep_start;
};

/* Structure describing vddsdio configuration. */

struct esp32s3_rtc_vddsdio_config_s
{
  uint32_t force :  1; /* If 1, use configuration from RTC registers;
                        * if 0, use EFUSE/bootstrapping pins.
                        */
  uint32_t enable : 1; /* Enable VDDSDIO regulator */
  uint32_t tieh  :  1; /* Select VDDSDIO voltage. One of
                        * RTC_VDDSDIO_TIEH_1_8V, RTC_VDDSDIO_TIEH_3_3V
                        */
  uint32_t drefh :  2; /* Tuning parameter for VDDSDIO regulator */
  uint32_t drefm :  2; /* Tuning parameter for VDDSDIO regulator */
  uint32_t drefl :  2; /* Tuning parameter for VDDSDIO regulator */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline void IRAM_ATTR esp32s3_uart_tx_wait_idle(uint8_t uart_no);
static void IRAM_ATTR esp32s3_flush_uarts(void);
static void IRAM_ATTR esp32s3_suspend_uarts(void);
static void IRAM_ATTR esp32s3_resume_uarts(void);
static void IRAM_ATTR esp32s3_timer_wakeup_prepare(void);
static uint32_t IRAM_ATTR esp32s3_get_power_down_flags(void);
static void IRAM_ATTR esp32s3_set_vddsdio_config(
                      struct esp32s3_rtc_vddsdio_config_s config);
static int IRAM_ATTR esp32s3_get_vddsdio_config(
                      struct esp32s3_rtc_vddsdio_config_s *config);
static int IRAM_ATTR esp32s3_light_sleep_inner(uint32_t pd_flags,
            uint32_t time_us, struct esp32s3_rtc_vddsdio_config_s config);
static int IRAM_ATTR esp32s3_sleep_start(uint32_t pd_flags);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct esp32s3_sleep_config_s g_config =
{
  .pd_options =
            { ESP_PD_OPTION_AUTO, ESP_PD_OPTION_AUTO,
              ESP_PD_OPTION_AUTO, ESP_PD_OPTION_AUTO,
              ESP_PD_OPTION_AUTO, ESP_PD_OPTION_AUTO,
              ESP_PD_OPTION_AUTO
            },
  .ccount_ticks_record = 0,
  .sleep_time_overhead_out = DEFAULT_SLEEP_OUT_OVERHEAD_US,
  .wakeup_triggers = 0
};

static _Atomic uint32_t pm_wakelock = 0;

/* Inform peripherals of light sleep wakeup overhead time */

inform_out_sleep_overhead_cb_t
  g_periph_inform_out_sleep_overhead_cb[PERIPH_INFORM_OUT_SLEEP_OVERHEAD_NO];

/* Indicates if light sleep shoule be skipped by peripherals. */

skip_light_sleep_cb_t g_periph_skip_sleep_cb[PERIPH_SKIP_SLEEP_NO];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Pauses execution for us microseconds. */

extern void esp_rom_delay_us(uint32_t us);

/****************************************************************************
 * Name: esp32s3_uart_tx_wait_idle
 *
 * Description:
 *   Wait until uart tx full empty and the last char send ok.
 *
 * Input Parameters:
 *   uart_no - 0 for UART0, 1 for UART1, 2 for UART2
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void IRAM_ATTR esp32s3_uart_tx_wait_idle(uint8_t uart_no)
{
  uint32_t status;
  do
    {
      status = getreg32(UART_STATUS_REG(uart_no));
    }
  while ((status & (UART_ST_UTX_OUT_M | UART_TXFIFO_CNT_M)) != 0);
}

/****************************************************************************
 * Name: esp32s3_flush_uarts
 *
 * Description:
 *   Wait until UART0/UART1 tx full empty and the last char send ok
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void IRAM_ATTR esp32s3_flush_uarts(void)
{
  int i;

  for (i = 0; i < ESP32S3_NUARTS; ++i)
    {
      if (periph_ll_periph_enabled(PERIPH_UART0_MODULE + i))
        {
          esp32s3_uart_tx_wait_idle(i);
        }
    }
}

/****************************************************************************
 * Name: esp32s3_suspend_uarts
 *
 * Description:
 *   Suspend UART0/UART1 output
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void IRAM_ATTR esp32s3_suspend_uarts(void)
{
  int i;
  uint32_t uart_fsm = 0;

  for (i = 0; i < ESP32S3_NUARTS; ++i)
    {
      if (!periph_ll_periph_enabled(PERIPH_UART0_MODULE + i))
        {
          continue;
        }

      modifyreg32(UART_FLOW_CONF_REG(i), UART_FORCE_XON,
                  UART_SW_FLOW_CON_EN | UART_FORCE_XOFF);
      do
        {
          uart_fsm = REG_GET_FIELD(UART_STATUS_REG(i), UART_ST_UTX_OUT);
        }
      while (!(uart_fsm == UART_FSM_IDLE ||
               uart_fsm == UART_FSM_TX_WAIT_SEND));
    }
}

/****************************************************************************
 * Name: esp32s3_resume_uarts
 *
 * Description:
 *   Re-enable UART0/UART1/UART2 output
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void IRAM_ATTR esp32s3_resume_uarts(void)
{
  int i;

  for (i = 0; i < ESP32S3_NUARTS; ++i)
    {
      if (!periph_ll_periph_enabled(PERIPH_UART0_MODULE + i))
        {
          continue;
        }

      modifyreg32(UART_FLOW_CONF_REG(i), UART_FORCE_XOFF, 0);
      modifyreg32(UART_FLOW_CONF_REG(i), 0, UART_FORCE_XON);
      modifyreg32(UART_FLOW_CONF_REG(i), UART_SW_FLOW_CON_EN |
                  UART_FORCE_XON, 0);
    }
}

/****************************************************************************
 * Name:  esp32s3_get_power_down_flags
 *
 * Description:
 *   Get power domains that can be powered down
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Power domains
 *
 ****************************************************************************/

static uint32_t IRAM_ATTR esp32s3_get_power_down_flags(void)
{
  uint32_t pd_flags = 0;

  g_config.pd_options[ESP_PD_DOMAIN_RTC_FAST_MEM] = ESP_PD_OPTION_ON;

  if (g_config.pd_options[ESP_PD_DOMAIN_RTC_PERIPH] == ESP_PD_OPTION_AUTO)
    {
      if (g_config.wakeup_triggers & RTC_GPIO_TRIG_EN)
        {
          g_config.pd_options[ESP_PD_DOMAIN_RTC_PERIPH] =
                                              ESP_PD_OPTION_ON;
        }
      else
        {
          g_config.pd_options[ESP_PD_DOMAIN_RTC_PERIPH] =
                                              ESP_PD_OPTION_OFF;
        }
    }

  g_config.pd_options[ESP_PD_DOMAIN_CPU] = ESP_PD_OPTION_ON;

  /* Prepare flags based on the selected options */

  if (g_config.pd_options[ESP_PD_DOMAIN_RTC_FAST_MEM] != ESP_PD_OPTION_ON)
    {
      pd_flags |= RTC_SLEEP_PD_RTC_FAST_MEM;
    }

  if (g_config.pd_options[ESP_PD_DOMAIN_RTC_SLOW_MEM] != ESP_PD_OPTION_ON)
    {
      pd_flags |= RTC_SLEEP_PD_RTC_SLOW_MEM;
    }

  if (g_config.pd_options[ESP_PD_DOMAIN_RTC_PERIPH] != ESP_PD_OPTION_ON)
    {
      pd_flags |= RTC_SLEEP_PD_RTC_PERIPH;
    }

  if (g_config.pd_options[ESP_PD_DOMAIN_CPU] != ESP_PD_OPTION_ON)
    {
      pd_flags |= RTC_SLEEP_PD_CPU;
    }

  if (g_config.pd_options[ESP_PD_DOMAIN_RTC8M] != ESP_PD_OPTION_ON)
    {
      pd_flags |= RTC_SLEEP_PD_INT_8M;
    }

  if (g_config.pd_options[ESP_PD_DOMAIN_XTAL] != ESP_PD_OPTION_ON)
    {
      pd_flags |= RTC_SLEEP_PD_XTAL;
    }

  g_config.pd_options[ESP_PD_DOMAIN_VDDSDIO] = ESP_PD_OPTION_ON;

  return pd_flags;
}

/****************************************************************************
 * Name:  esp32s3_timer_wakeup_prepare
 *
 * Description:
 *   Configure timer to wake-up
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void IRAM_ATTR esp32s3_timer_wakeup_prepare(void)
{
  int64_t ticks;
  int64_t sleep_duration = (int64_t)g_config.sleep_duration -
                           (int64_t)g_config.sleep_time_adjustment;
  if (sleep_duration < 0)
    {
      sleep_duration = 0;
    }

  ticks = esp32s3_rtc_time_us_to_slowclk(sleep_duration,
                                         g_config.rtc_clk_cal_period);
  esp32s3_rtc_sleep_set_wakeup_time(g_config.rtc_ticks_at_sleep_start
                                    + ticks);
}

/****************************************************************************
 * Name: esp32s3_set_vddsdio_config
 *
 * Description:
 *   Set new VDDSDIO configuration using RTC registers.
 *
 * Input Parameters:
 *   config - New VDDSDIO configuration
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void IRAM_ATTR esp32s3_set_vddsdio_config(
                      struct esp32s3_rtc_vddsdio_config_s config)
{
  uint32_t val = 0;
  val |= (config.force << RTC_CNTL_SDIO_FORCE_S);
  val |= (config.enable << RTC_CNTL_XPD_SDIO_REG_S);
  val |= (config.drefh << RTC_CNTL_DREFH_SDIO_S);
  val |= (config.drefm << RTC_CNTL_DREFM_SDIO_S);
  val |= (config.drefl << RTC_CNTL_DREFL_SDIO_S);
  val |= (config.tieh << RTC_CNTL_SDIO_TIEH_S);
  val |= RTC_CNTL_SDIO_REG_PD_EN;
  putreg32((uint32_t)val, RTC_CNTL_RTC_SDIO_CONF_REG);
}

/****************************************************************************
 * Name: esp32s3_get_vddsdio_config
 *
 * Description:
 *   Get current VDDSDIO configuration.
 *
 * Input Parameters:
 *   config - Incoming parameter address of VDDSDIO configuration to be saved
 *
 * Returned Value:
 *   Zero (OK) is returned on success.
 *
 ****************************************************************************/

static int IRAM_ATTR esp32s3_get_vddsdio_config(
                     struct esp32s3_rtc_vddsdio_config_s *config)
{
  struct esp32s3_rtc_vddsdio_config_s *result = config;
  uint32_t strap_reg;
  uint32_t sdio_conf_reg = getreg32(RTC_CNTL_RTC_SDIO_CONF_REG);

  result->drefh = (sdio_conf_reg & RTC_CNTL_DREFH_SDIO_M)
                  >> RTC_CNTL_DREFH_SDIO_S;
  result->drefm = (sdio_conf_reg & RTC_CNTL_DREFM_SDIO_M)
                  >> RTC_CNTL_DREFM_SDIO_S;
  result->drefl = (sdio_conf_reg & RTC_CNTL_DREFL_SDIO_M)
                  >> RTC_CNTL_DREFL_SDIO_S;

  if (sdio_conf_reg & RTC_CNTL_SDIO_FORCE)
    {
      /* Get configuration from RTC */

      result->force = 1;
      result->enable = (sdio_conf_reg & RTC_CNTL_XPD_SDIO_REG_M)
                       >> RTC_CNTL_XPD_SDIO_REG_S;
      result->tieh = (sdio_conf_reg & RTC_CNTL_SDIO_TIEH_M)
                     >> RTC_CNTL_SDIO_TIEH_S;

      return OK;
    }

  /* Otherwise, VDD_SDIO is controlled by bootstrapping pin */

  strap_reg = getreg32(GPIO_STRAP_REG);
  result->force = 0;
  result->tieh = (strap_reg & BIT(5)) ? RTC_VDDSDIO_TIEH_1_8V
                                      : RTC_VDDSDIO_TIEH_3_3V;
  result->enable = 1;

  return OK;
}

/****************************************************************************
 * Name: esp32s3_sleep_start
 *
 * Description:
 *   Enter low power mode.
 *
 * Input Parameters:
 *   pd_flags - Power domains
 *
 * Returned Value:
 *   0 is returned on success or a negated errno value is returned
 *
 ****************************************************************************/

static int IRAM_ATTR esp32s3_sleep_start(uint32_t pd_flags)
{
  int result;
  struct esp32s3_cpu_freq_config_s cpu_freq_config;
  bool deep_sleep = pd_flags & RTC_SLEEP_PD_DIG;

  /* Stop UART output so that output is not lost due to APB frequency change.
   * For light sleep, suspend UART output - it will resume after wakeup.
   * For deep sleep, wait for the contents of UART FIFO to be sent.
   */

  if (deep_sleep == true)
    {
      esp32s3_flush_uarts();
    }
  else
    {
      esp32s3_suspend_uarts();
    }

  /* Save current frequency and switch to XTAL */

  esp32s3_rtc_clk_cpu_freq_get_config(&cpu_freq_config);
  esp32s3_rtc_cpu_freq_set_xtal();

  /* Enter sleep */

  esp32s3_rtc_sleep_init(pd_flags | RTC_SLEEP_NO_ULTRA_LOW);

  /* Set state machine time for light sleep */

  if (deep_sleep == false)
    {
      esp32s3_rtc_sleep_low_init(g_config.rtc_clk_cal_period);
    }

  /* Configure timer wakeup */

  if (g_config.wakeup_triggers & RTC_TIMER_TRIG_EN)
    {
      esp32s3_timer_wakeup_prepare();
    }

  result = esp32s3_rtc_sleep_start(g_config.wakeup_triggers, 0);

  /* Restore CPU frequency */

  esp32s3_rtc_clk_cpu_freq_set_config(&cpu_freq_config);

  if (deep_sleep == false)
    {
      g_config.ccount_ticks_record = XTHAL_GET_CCOUNT();
    }

  /* Re-enable UART output */

  esp32s3_resume_uarts();

  return result;
}

/****************************************************************************
 * Name: esp32s3_light_sleep_inner
 *
 * Description:
 *   Enter low power mode, then wait for flash to be ready on wakeup
 *
 * Input Parameters:
 *   pd_flags - Power domains
 *   time_us  - Time to wait for spi_flash become ready
 *   config   - VDDSDIO configuration
 *
 * Returned Value:
 *   0 is returned on success or a negated errno value is returned
 *
 ****************************************************************************/

static int IRAM_ATTR esp32s3_light_sleep_inner(uint32_t pd_flags,
        uint32_t time_us, struct esp32s3_rtc_vddsdio_config_s config)
{
  /* Enter sleep */

  int err = esp32s3_sleep_start(pd_flags);

  /* If VDDSDIO regulator was controlled by RTC registers before sleep.
   * restore the configuration.
   */

  if (config.force)
    {
      esp32s3_set_vddsdio_config(config);
    }

  /* If SPI flash was powered down, wait for it to become ready. */

  if (pd_flags & RTC_SLEEP_PD_VDDSDIO)
    {
      /* Wait for the flash chip to start up. */

      esp_rom_delay_us(time_us);
    }

  return err;
}

/****************************************************************************
 * Name:  esp32s3_periph_should_skip_sleep
 *
 * Description:
 *   Indicates if light sleep shoule be skipped by peripherals
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   True is returned on success. Otherwise false.
 *
 ****************************************************************************/

static inline bool IRAM_ATTR esp32s3_periph_should_skip_sleep(void)
{
  for (int i = 0; i < PERIPH_SKIP_SLEEP_NO; i++)
    {
      if (g_periph_skip_sleep_cb[i])
        {
          if (g_periph_skip_sleep_cb[i]() == true)
            {
              return true;
            }
        }
    }

  return false;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  esp32s3_pm_register_skip_sleep_callback
 *
 * Description:
 *   Unregister callback function of skipping light sleep.
 *
 * Input Parameters:
 *   cb - Callback function
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise -1 (ERROR).
 *
 ****************************************************************************/

int esp32s3_pm_register_skip_sleep_callback(skip_light_sleep_cb_t cb)
{
  for (int i = 0; i < PERIPH_SKIP_SLEEP_NO; i++)
    {
      if (g_periph_skip_sleep_cb[i] == cb)
        {
          return OK;
        }
      else if (g_periph_skip_sleep_cb[i] == NULL)
        {
          g_periph_skip_sleep_cb[i] = cb;
          return OK;
        }
    }

  return ERROR;
}

/****************************************************************************
 * Name:  esp32s3_pm_unregister_skip_sleep_callback
 *
 * Description:
 *   Register callback function of skipping light sleep.
 *
 * Input Parameters:
 *   cb - Callback function
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise -1 (ERROR).
 *
 ****************************************************************************/

int esp32s3_pm_unregister_skip_sleep_callback(skip_light_sleep_cb_t cb)
{
  for (int i = 0; i < PERIPH_SKIP_SLEEP_NO; i++)
    {
      if (g_periph_skip_sleep_cb[i] == cb)
        {
          g_periph_skip_sleep_cb[i] = NULL;
          return OK;
        }
    }

  return ERROR;
}

/****************************************************************************
 * Name:  esp32s3_should_skip_light_sleep
 *
 * Description:
 *   Indicates if light sleep shoule be skipped.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   True is returned on success. Otherwise false.
 *
 ****************************************************************************/

bool IRAM_ATTR esp32s3_should_skip_light_sleep(void)
{
  if (esp32s3_periph_should_skip_sleep() == true)
    {
      return true;
    }

  return false;
}

/****************************************************************************
 * Name:  esp32s3_pm_register_inform_out_sleep_overhead_callback
 *
 * Description:
 *   Register informing peripherals of light sleep wakeup overhead time
 *   callback function.
 *
 * Input Parameters:
 *   cb - Callback function
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise -1 (ERROR).
 *
 ****************************************************************************/

int esp32s3_pm_register_inform_out_sleep_overhead_callback(
                            inform_out_sleep_overhead_cb_t cb)
{
  for (int i = 0; i < PERIPH_INFORM_OUT_SLEEP_OVERHEAD_NO; i++)
    {
      if (g_periph_inform_out_sleep_overhead_cb[i] == cb)
        {
          return ERROR;
        }
      else if (g_periph_inform_out_sleep_overhead_cb[i] == NULL)
        {
          g_periph_inform_out_sleep_overhead_cb[i] = cb;
          return OK;
        }
    }

  return ERROR;
}

/****************************************************************************
 * Name:  esp32s3_pm_unregister_inform_out_sleep_overhead_callback
 *
 * Description:
 *   Unregister informing peripherals of light sleep wakeup overhead time
 *   callback function.
 *
 * Input Parameters:
 *   cb - Callback function
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise -1 (ERROR).
 *
 ****************************************************************************/

int esp32s3_pm_unregister_inform_out_sleep_overhead_callback(
                              inform_out_sleep_overhead_cb_t cb)
{
  for (int i = 0; i < PERIPH_INFORM_OUT_SLEEP_OVERHEAD_NO; i++)
    {
      if (g_periph_inform_out_sleep_overhead_cb[i] == cb)
        {
          g_periph_inform_out_sleep_overhead_cb[i] = NULL;
          return OK;
        }
    }

  return ERROR;
}

/****************************************************************************
 * Name:  esp32s3_periph_inform_out_sleep_overhead
 *
 * Description:
 *   Inform peripherals of light sleep wakeup overhead time
 *
 * Input Parameters:
 *   us - Light sleep wakeup overhead time
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void IRAM_ATTR esp32s3_periph_inform_out_sleep_overhead(uint32_t us)
{
  for (int i = 0; i < PERIPH_INFORM_OUT_SLEEP_OVERHEAD_NO; i++)
    {
      if (g_periph_inform_out_sleep_overhead_cb[i])
        {
          g_periph_inform_out_sleep_overhead_cb[i](us);
        }
    }
}

/****************************************************************************
 * Name:  esp32s3_sleep_enable_timer_wakeup
 *
 * Description:
 *   Enable wakeup by timer
 *
 * Input Parameters:
 *   time_in_us - Configure wake-up time interval
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void IRAM_ATTR esp32s3_sleep_enable_timer_wakeup(uint64_t time_in_us)
{
  g_config.wakeup_triggers |= RTC_TIMER_TRIG_EN;
  g_config.sleep_duration = time_in_us;
}

/****************************************************************************
 * Name:  esp32s3_sleep_enable_wifi_wakeup
 *
 * Description:
 *   Configure Wi-Fi wake-up source
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32s3_sleep_enable_wifi_wakeup(void)
{
  g_config.wakeup_triggers |= RTC_WIFI_TRIG_EN;
}

/****************************************************************************
 * Name:  esp32s3_light_sleep_start
 *
 * Description:
 *   Enter light sleep mode
 *
 * Input Parameters:
 *   sleep_time - Actual sleep time
 *
 * Returned Value:
 *   0 is returned on success or a negated errno value is returned
 *
 ****************************************************************************/

int IRAM_ATTR esp32s3_light_sleep_start(uint64_t *sleep_time)
{
  int ret = OK;
  irqstate_t flags;
  uint32_t pd_flags;
  uint32_t flash_enable_time_us;
  uint32_t vddsdio_pd_sleep_duration;
  struct esp32s3_rtc_vddsdio_config_s vddsdio_config;
  uint32_t rtc_cntl_xtl_buf_wait_cycles;
  uint32_t sleep_time_overhead_in;
  uint32_t ccount_at_sleep_start;
  int64_t final_sleep_us;
  int64_t min_sleep_us;

  flags = enter_critical_section();

  g_config.ccount_ticks_record = XTHAL_GET_CCOUNT();
  g_config.rtc_ticks_at_sleep_start = esp32s3_rtc_time_get();
  ccount_at_sleep_start = XTHAL_GET_CCOUNT();
  sleep_time_overhead_in = (ccount_at_sleep_start -
    g_config.ccount_ticks_record) / (esp_clk_cpu_freq() / 1000000ULL);

  /* Decide which power domains can be powered down */

  pd_flags = esp32s3_get_power_down_flags();
  pd_flags &= ~RTC_SLEEP_PD_RTC_PERIPH;
  g_config.rtc_clk_cal_period =
        esp32s3_rtc_clk_cal(RTC_CAL_RTC_MUX, RTC_CLK_SRC_CAL_CYCLES);

  /* Adjustment time consists of parts below:
   * 1. Hardware time waiting for internal 8M oscilate clock and XTAL;
   * 2. Hardware state swithing time of the rtc main state machine;
   * 3. Code execution time when clock is not stable;
   * 4. Code execution time which can be measured;
   */

  rtc_cntl_xtl_buf_wait_cycles = esp32s3_rtc_time_us_to_slowclk(
        RTC_CNTL_XTL_BUF_WAIT_SLP_US, g_config.rtc_clk_cal_period);

  g_config.sleep_time_adjustment = LIGHT_SLEEP_TIME_OVERHEAD_US +
        sleep_time_overhead_in + g_config.sleep_time_overhead_out
        + esp32s3_rtc_time_slowclk_to_us(rtc_cntl_xtl_buf_wait_cycles +
        RTC_CNTL_CK8M_WAIT_SLP_CYCLES + RTC_CNTL_WAKEUP_DELAY_CYCLES,
        g_config.rtc_clk_cal_period);

  flash_enable_time_us = VDD_SDIO_POWERUP_TO_FLASH_READ_US
                         + DEEP_SLEEP_WAKEUP_DELAY;

  esp32s3_periph_inform_out_sleep_overhead(
    g_config.sleep_time_adjustment - sleep_time_overhead_in);

  esp32s3_get_vddsdio_config(&vddsdio_config);

  final_sleep_us = (int64_t)g_config.sleep_duration -
                   (int64_t)g_config.sleep_time_adjustment;
  min_sleep_us = esp32s3_rtc_time_slowclk_to_us(RTC_CNTL_MIN_SLP_VAL_MIN,
                                                g_config.rtc_clk_cal_period);

  /* If rtc timer wakeup source is enabled, need to compare final
   *  sleep duration and min sleep duration to avoid late wakeup
   */

  if ((g_config.wakeup_triggers & RTC_TIMER_TRIG_EN) &&
      (final_sleep_us <= min_sleep_us))
    {
      ret = ERROR;
    }
  else
    {
      /* Enter sleep, then wait for flash to be ready on wakeup */

      ret = esp32s3_light_sleep_inner(pd_flags, flash_enable_time_us,
                                      vddsdio_config);
    }

  if (sleep_time != NULL)
    {
      *sleep_time = esp32s3_rtc_time_slowclk_to_us(esp32s3_rtc_time_get() -
          g_config.rtc_ticks_at_sleep_start, g_config.rtc_clk_cal_period);
    }

  g_config.sleep_time_overhead_out = (XTHAL_GET_CCOUNT() -
       g_config.ccount_ticks_record) / (esp_clk_cpu_freq() / 1000000ULL);

  leave_critical_section(flags);

  return ret;
}

/****************************************************************************
 * Name: esp32s3_pmstandby
 *
 * Description:
 *   Enter force sleep.
 *
 * Input Parameters:
 *   time_in_us - Force sleep time interval
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32s3_pmstandby(uint64_t time_in_us)
{
  uint64_t rtc_diff_us;

  /* Don't power down XTAL - powering it up takes different time on. */

  esp32s3_sleep_enable_timer_wakeup(time_in_us);
  esp32s3_light_sleep_start(&rtc_diff_us);
  pwrinfo("Returned from auto-sleep, slept for %" PRIu32 " ms\n",
            (uint32_t)(rtc_diff_us) / 1000);
}

/****************************************************************************
 * Name:  esp32s3_deep_sleep_start
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

void IRAM_ATTR esp32s3_deep_sleep_start(void)
{
  uint32_t pd_flags;

  /* Record current RTC time */

  g_config.rtc_ticks_at_sleep_start = esp32s3_rtc_time_get();

  /* Decide which power domains can be powered down */

  pd_flags = esp32s3_get_power_down_flags();
  g_config.rtc_clk_cal_period = getreg32(RTC_SLOW_CLK_CAL_REG);

  /* Correct the sleep time */

  g_config.sleep_time_adjustment = DEEP_SLEEP_TIME_OVERHEAD_US;

  pd_flags |= RTC_SLEEP_PD_DIG | RTC_SLEEP_PD_VDDSDIO |
              RTC_SLEEP_PD_INT_8M | RTC_SLEEP_PD_XTAL;

  /* Enter deep sleep */

  esp32s3_sleep_start(pd_flags);

  /* Because RTC is in a slower clock domain than the CPU, it
   * can take several CPU cycles for the sleep mode to start.
   */

  while (1);
}

/****************************************************************************
 * Name: esp32s3_pmsleep
 *
 * Description:
 *   Enter deep sleep.
 *
 * Input Parameters:
 *   time_in_us - Deep sleep time interval
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32s3_pmsleep(uint64_t time_in_us)
{
  esp32s3_sleep_enable_timer_wakeup(time_in_us);
  esp32s3_deep_sleep_start();
}

/****************************************************************************
 * Name: esp32s3_pm_lockacquire
 *
 * Description:
 *   Take a power management lock
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void IRAM_ATTR esp32s3_pm_lockacquire(void)
{
  ++pm_wakelock;
}

/****************************************************************************
 * Name: esp32s3_pm_lockrelease
 *
 * Description:
 *   Release the lock taken using esp32s3_pm_lockacquire.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void IRAM_ATTR esp32s3_pm_lockrelease(void)
{
  --pm_wakelock;
}

/****************************************************************************
 * Name: esp32s3_pm_lockstatus
 *
 * Description:
 *   Return power management lock status.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Current pm_wakelock count
 *
 ****************************************************************************/

uint32_t IRAM_ATTR esp32s3_pm_lockstatus(void)
{
  return pm_wakelock;
}

#endif /* CONFIG_PM */
