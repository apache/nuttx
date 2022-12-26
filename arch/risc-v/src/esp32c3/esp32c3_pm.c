/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_pm.c
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
#include <sys/types.h>

#include "chip.h"
#include "riscv_internal.h"
#include "hardware/esp32c3_rtccntl.h"
#include "hardware/esp32c3_system.h"
#include "hardware/esp32c3_syscon.h"
#include "hardware/esp32c3_soc.h"
#include "hardware/esp32c3_uart.h"
#include "hardware/esp32c3_gpio.h"
#include "hardware/apb_ctrl_reg.h"

#include "esp32c3_attr.h"
#include "esp32c3_rtc.h"
#include "esp32c3_clockconfig.h"
#include "esp32c3_pm.h"

#ifdef CONFIG_ESP32C3_RT_TIMER
#include "esp32c3_rt_timer.h"
#endif

#ifdef CONFIG_SCHED_TICKLESS
#include "esp32c3_tickless.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If light sleep time is less than that, don't power down flash */

#define FLASH_PD_MIN_SLEEP_TIME_US 2000

/* Minimal amount of time we can sleep for. */

#define LIGHT_SLEEP_MIN_TIME_US    200

#define RTC_MODULE_SLEEP_PREPARE_CYCLES (6)

#ifndef MAX
#  define MAX(a,b) ((a) > (b) ? (a) : (b))
#endif

/* Time from VDD_SDIO power up to first flash read in ROM code */

#define VDD_SDIO_POWERUP_TO_FLASH_READ_US (700)

/* Extra time it takes to enter and exit light sleep and deep sleep */

#define LIGHT_SLEEP_TIME_OVERHEAD_US      (37)

#ifdef CONFIG_ESP32C3_CPU_FREQ_MHZ
#define DEFAULT_CPU_FREQ_MHZ CONFIG_ESP32C3_CPU_FREQ_MHZ
#else
#define DEFAULT_CPU_FREQ_MHZ (160)
#endif

#define DEFAULT_SLEEP_OUT_OVERHEAD_US       (105)
#define DEEP_SLEEP_TIME_OVERHEAD_US         (250 + 100 * 240 / DEFAULT_CPU_FREQ_MHZ)

#define DEEP_SLEEP_WAKEUP_DELAY     0

#define RTC_VDDSDIO_TIEH_1_8V       0  /* TIEH field value for 1.8V VDDSDIO */
#define RTC_VDDSDIO_TIEH_3_3V       1  /* TIEH field value for 3.3V VDDSDIO */

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

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Power down options */

enum esp32c3_sleep_pd_option_e
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

enum esp32c3_sleep_pd_domain_e
{
  ESP_PD_DOMAIN_RTC_PERIPH = 0,  /* RTC IO, sensors */
  ESP_PD_DOMAIN_RTC_SLOW_MEM,    /* RTC slow memory */
  ESP_PD_DOMAIN_RTC_FAST_MEM,    /* RTC fast memory */
  ESP_PD_DOMAIN_XTAL,            /* XTAL oscillator */
  ESP_PD_DOMAIN_CPU,             /* CPU core */
  ESP_PD_DOMAIN_MAX              /* Number of domains */
};

/* Internal structure which holds all requested deep sleep parameters. */

struct esp32c3_sleep_config_s
{
  enum esp32c3_sleep_pd_option_e pd_options[ESP_PD_DOMAIN_MAX];
  uint64_t sleep_duration;
  uint32_t wakeup_triggers : 15;
  uint32_t ext1_trigger_mode : 1;
  uint32_t ext1_rtc_gpio_mask : 18;
  uint32_t ext0_trigger_level : 1;
  uint32_t ext0_rtc_gpio_num : 5;
  uint32_t gpio_wakeup_mask : 6;
  uint32_t gpio_trigger_mode : 6;
  uint32_t sleep_time_adjustment;
  uint32_t ccount_ticks_record;
  uint32_t sleep_time_overhead_out;
  uint32_t rtc_clk_cal_period;
  uint64_t rtc_ticks_at_sleep_start;
  void     *cpu_pd_mem;
};

/* Structure describing vddsdio configuration. */

struct esp32c3_rtc_vddsdio_config_s
{
  uint32_t force : 1;     /* If 1, use configuration from RTC registers;
                           * if 0, use EFUSE/bootstrapping pins.
                           */
  uint32_t enable : 1;    /* Enable VDDSDIO regulator */
  uint32_t tieh  : 1;     /* Select VDDSDIO voltage. One of
                           * RTC_VDDSDIO_TIEH_1_8V, RTC_VDDSDIO_TIEH_3_3V
                           */
  uint32_t drefh : 2;     /* Tuning parameter for VDDSDIO regulator */
  uint32_t drefm : 2;     /* Tuning parameter for VDDSDIO regulator */
  uint32_t drefl : 2;     /* Tuning parameter for VDDSDIO regulator */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline void IRAM_ATTR esp32c3_uart_tx_wait_idle(uint8_t uart_no);
static void IRAM_ATTR esp32c3_flush_uarts(void);
static void IRAM_ATTR esp32c3_suspend_uarts(void);
static void IRAM_ATTR esp32c3_resume_uarts(void);
static void IRAM_ATTR esp32c3_timer_wakeup_prepare(void);
static uint32_t IRAM_ATTR esp32c3_get_power_down_flags(void);
static void IRAM_ATTR esp32c3_set_vddsdio_config(
                      struct esp32c3_rtc_vddsdio_config_s config);
static int IRAM_ATTR esp32c3_get_vddsdio_config(
                      struct esp32c3_rtc_vddsdio_config_s *config);
static int IRAM_ATTR esp32c3_light_sleep_inner(uint32_t pd_flags,
            uint32_t time_us, struct esp32c3_rtc_vddsdio_config_s config);
static int IRAM_ATTR esp32c3_sleep_start(uint32_t pd_flags);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct esp32c3_sleep_config_s s_config =
{
  .pd_options =
            { ESP_PD_OPTION_AUTO, ESP_PD_OPTION_AUTO,
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

/* Set the real CPU ticks per us to the ets,
 * so that ets_delay_us will be accurate.
 */

extern void ets_update_cpu_frequency(uint32_t ticks_per_us);

/* Pauses execution for us microseconds. */

extern void esp_rom_delay_us(uint32_t us);

/****************************************************************************
 * Name: esp32c3_uart_tx_wait_idle
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

static inline void IRAM_ATTR esp32c3_uart_tx_wait_idle(uint8_t uart_no)
{
  uint32_t status;
  do
    {
      status = getreg32(UART_STATUS_REG(uart_no));
    }
  while ((status & (UART_ST_UTX_OUT_M | UART_TXFIFO_CNT_M)) != 0);
}

/****************************************************************************
 * Name: esp32c3_flush_uarts
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

static void IRAM_ATTR esp32c3_flush_uarts(void)
{
  int i;

  for (i = 0; i < ESP32C3_NUARTS; ++i)
    {
      if (esp32c3_periph_ll_periph_enabled(PERIPH_UART0_MODULE + i))
        {
          esp32c3_uart_tx_wait_idle(i);
        }
    }
}

/****************************************************************************
 * Name: esp32c3_suspend_uarts
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

static void IRAM_ATTR esp32c3_suspend_uarts(void)
{
  int i;
  uint32_t uart_fsm = 0;

  for (i = 0; i < ESP32C3_NUARTS; ++i)
    {
      if (!esp32c3_periph_ll_periph_enabled(PERIPH_UART0_MODULE + i))
        {
          continue;
        }

      modifyreg32(UART_FLOW_CONF_REG(i), 0, UART_FORCE_XOFF);
      do
        {
          uart_fsm = REG_GET_FIELD(UART_STATUS_REG(i), UART_ST_UTX_OUT);
        }
      while (!(uart_fsm == UART_FSM_IDLE ||
               uart_fsm == UART_FSM_TX_WAIT_SEND));
    }
}

/****************************************************************************
 * Name: esp32c3_resume_uarts
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

static void IRAM_ATTR esp32c3_resume_uarts(void)
{
  int i;

  for (i = 0; i < ESP32C3_NUARTS; ++i)
    {
      if (!esp32c3_periph_ll_periph_enabled(PERIPH_UART0_MODULE + i))
        {
          continue;
        }

      REG_CLR_BIT(UART_FLOW_CONF_REG(i), UART_FORCE_XOFF);
      REG_SET_BIT(UART_FLOW_CONF_REG(i), UART_FORCE_XON);
      REG_CLR_BIT(UART_FLOW_CONF_REG(i),
                  UART_SW_FLOW_CON_EN | UART_FORCE_XON);
      REG_SET_BIT(UART_ID_REG(i), UART_UPDATE);
    }
}

/****************************************************************************
 * Name:  esp32c3_get_power_down_flags
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

static uint32_t IRAM_ATTR esp32c3_get_power_down_flags(void)
{
  uint32_t pd_flags = 0;

  s_config.pd_options[ESP_PD_DOMAIN_RTC_FAST_MEM] = ESP_PD_OPTION_ON;

  if (s_config.pd_options[ESP_PD_DOMAIN_RTC_PERIPH] == ESP_PD_OPTION_AUTO)
    {
      if (s_config.wakeup_triggers & RTC_GPIO_TRIG_EN)
        {
          s_config.pd_options[ESP_PD_DOMAIN_RTC_PERIPH] =
                                              ESP_PD_OPTION_ON;
        }
      else
        {
          s_config.pd_options[ESP_PD_DOMAIN_RTC_PERIPH] =
                                              ESP_PD_OPTION_OFF;
        }
    }

  if (s_config.cpu_pd_mem == NULL)
    {
      s_config.pd_options[ESP_PD_DOMAIN_CPU] = ESP_PD_OPTION_ON;
    }

  if (s_config.pd_options[ESP_PD_DOMAIN_XTAL] == ESP_PD_OPTION_AUTO)
    {
      s_config.pd_options[ESP_PD_DOMAIN_XTAL] = ESP_PD_OPTION_OFF;
    }

  /* Prepare flags based on the selected options */

  if (s_config.pd_options[ESP_PD_DOMAIN_RTC_FAST_MEM] != ESP_PD_OPTION_ON)
    {
      pd_flags |= RTC_SLEEP_PD_RTC_FAST_MEM;
    }

  if (s_config.pd_options[ESP_PD_DOMAIN_RTC_PERIPH] != ESP_PD_OPTION_ON)
    {
      pd_flags |= RTC_SLEEP_PD_RTC_PERIPH;
    }

  if (s_config.pd_options[ESP_PD_DOMAIN_CPU] != ESP_PD_OPTION_ON)
    {
      pd_flags |= RTC_SLEEP_PD_CPU;
    }

  return pd_flags;
}

/****************************************************************************
 * Name:  esp32c3_timer_wakeup_prepare
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

static void IRAM_ATTR esp32c3_timer_wakeup_prepare(void)
{
  int64_t ticks;
  int64_t sleep_duration = (int64_t)s_config.sleep_duration -
                           (int64_t) s_config.sleep_time_adjustment;
  if (sleep_duration < 0)
    {
      sleep_duration = 0;
    }

  ticks = esp32c3_rtc_time_us_to_slowclk(sleep_duration,
                           s_config.rtc_clk_cal_period);
  esp32c3_rtc_sleep_set_wakeup_time(s_config.rtc_ticks_at_sleep_start
                                                             + ticks);
}

/****************************************************************************
 * Name: esp32c3_set_vddsdio_config
 *
 * Description:
 *   Set new VDDSDIO configuration using RTC registers.
 *
 * Input Parameters:
 *   New VDDSDIO configuration
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void IRAM_ATTR esp32c3_set_vddsdio_config(
                      struct esp32c3_rtc_vddsdio_config_s config)
{
  uint32_t val = 0;
  val |= (config.force << RTC_CNTL_SDIO_FORCE_S);
  val |= (config.enable << RTC_CNTL_XPD_SDIO_REG_S);
  val |= (config.drefh << RTC_CNTL_DREFH_SDIO_S);
  val |= (config.drefm << RTC_CNTL_DREFM_SDIO_S);
  val |= (config.drefl << RTC_CNTL_DREFL_SDIO_S);
  val |= (config.tieh << RTC_CNTL_SDIO_TIEH_S);
  val |= RTC_CNTL_SDIO_PD_EN;
  putreg32((uint32_t)val, RTC_CNTL_SDIO_CONF_REG);
}

/****************************************************************************
 * Name: esp32c3_get_vddsdio_config
 *
 * Description:
 *   Get current VDDSDIO configuration.
 *
 * Input Parameters:
 *   Incoming parameter address of VDDSDIO configuration to be saved
 *
 * Returned Value:
 *   Zero (OK) is returned on success.
 *
 ****************************************************************************/

static int IRAM_ATTR esp32c3_get_vddsdio_config(
                     struct esp32c3_rtc_vddsdio_config_s *config)
{
  struct esp32c3_rtc_vddsdio_config_s *result = config;
  uint32_t strap_reg;
  uint32_t sdio_conf_reg = getreg32(RTC_CNTL_SDIO_CONF_REG);

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
 * Name: esp32c3_sleep_start
 *
 * Description:
 *   Enter low power mode.
 *
 * Input Parameters:
 *   Power domains
 *
 * Returned Value:
 *   0 is returned on success or a negated errno value is returned
 *
 ****************************************************************************/

static int IRAM_ATTR esp32c3_sleep_start(uint32_t pd_flags)
{
  int result;
  struct esp32c3_cpu_freq_config_s cpu_freq_config;
  bool deep_sleep = pd_flags & RTC_SLEEP_PD_DIG;

  /* Stop UART output so that output is not lost due to APB frequency change.
   * For light sleep, suspend UART output — it will resume after wakeup.
   * For deep sleep, wait for the contents of UART FIFO to be sent.
   */

  if (deep_sleep)
    {
      esp32c3_flush_uarts();
    }
  else
    {
      esp32c3_suspend_uarts();
    }

  /* Save current frequency and switch to XTAL */

  esp32c3_rtc_clk_cpu_freq_get_config(&cpu_freq_config);
  esp32c3_rtc_cpu_freq_set_xtal();

  /* Enter sleep */

  esp32c3_rtc_sleep_init(pd_flags);

  esp32c3_rtc_sleep_low_init(s_config.rtc_clk_cal_period);

  /* Set state machine time for light sleep */

  if (deep_sleep == false)
    {
      esp32c3_rtc_sleep_low_init(s_config.rtc_clk_cal_period);
    }

  /* Configure timer wakeup */

  if ((s_config.wakeup_triggers & RTC_TIMER_TRIG_EN)
                     && s_config.sleep_duration > 0)
    {
      esp32c3_timer_wakeup_prepare();
    }

  if (deep_sleep)
    {
      /* Otherwise, need to call the dedicated soc function for this */

      result = esp32c3_rtc_deep_sleep_start(s_config.wakeup_triggers, 0);
    }
  else
    {
      result = esp32c3_rtc_sleep_start(s_config.wakeup_triggers, 0, 1);
    }

  /* Restore CPU frequency */

  esp32c3_rtc_clk_cpu_freq_set_config(&cpu_freq_config);

  if (!deep_sleep)
    {
      s_config.ccount_ticks_record = esp32c3_cpu_cycle_count();
    }

  REG_CLR_BIT(RTC_CNTL_RETENTION_CTRL_REG, RTC_CNTL_RETENTION_EN);

  /* Re-enable UART output */

  esp32c3_resume_uarts();

  return result;
}

/****************************************************************************
 * Name: esp32c3_light_sleep_inner
 *
 * Description:
 *   Enter low power mode, then wait for flash to be ready on wakeup
 *
 * Input Parameters:
 *   pd_flags - Power domains
 *   time_us  - Time to wait for spi_flash become ready
 *   config   -  VDDSDIO configuration
 *
 * Returned Value:
 *   0 is returned on success or a negated errno value is returned
 *
 ****************************************************************************/

static int IRAM_ATTR esp32c3_light_sleep_inner(uint32_t pd_flags,
        uint32_t time_us, struct esp32c3_rtc_vddsdio_config_s config)
{
  /* Enter sleep */

  int err = esp32c3_sleep_start(pd_flags);

  /* If VDDSDIO regulator was controlled by RTC registers before sleep.
   * restore the configuration.
   */

  if (config.force)
    {
      esp32c3_set_vddsdio_config(config);
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
 * Name:  esp32c3_periph_should_skip_sleep
 *
 * Description:
 *   Indicates if light sleep shoule be skipped by peripherals
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   True is returned on success.  Otherwise false.
 *
 ****************************************************************************/

static inline bool IRAM_ATTR esp32c3_periph_should_skip_sleep(void)
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
 * Name:  esp32c3_pm_register_skip_sleep_callback
 *
 * Description:
 *   Unregister callback function of skipping light sleep.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise -1 (ERROR).
 *
 ****************************************************************************/

int esp32c3_pm_register_skip_sleep_callback(skip_light_sleep_cb_t cb)
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
 * Name:  esp32c3_pm_unregister_skip_sleep_callback
 *
 * Description:
 *   Register callback function of skipping light sleep.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise -1 (ERROR).
 *
 ****************************************************************************/

int esp32c3_pm_unregister_skip_sleep_callback(skip_light_sleep_cb_t cb)
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
 * Name:  esp32c3_should_skip_light_sleep
 *
 * Description:
 *   Indicates if light sleep shoule be skipped.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   True is returned on success.  Otherwise false.
 *
 ****************************************************************************/

bool IRAM_ATTR esp32c3_should_skip_light_sleep(void)
{
  if (esp32c3_periph_should_skip_sleep() == true)
    {
      return true;
    }

  return false;
}

/****************************************************************************
 * Name:  esp32c3_pm_register_inform_out_sleep_overhead_callback
 *
 * Description:
 *   Register informing peripherals of light sleep wakeup overhead time
 *   callback function.
 *
 * Input Parameters:
 *   cb - callback function
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise -1 (ERROR).
 *
 ****************************************************************************/

int esp32c3_pm_register_inform_out_sleep_overhead_callback(
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
 * Name:  esp32c3_pm_unregister_inform_out_sleep_overhead_callback
 *
 * Description:
 *   Unregister informing peripherals of light sleep wakeup overhead time
 *   callback function.
 *
 * Input Parameters:
 *   cb - callback function
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

int esp32c3_pm_unregister_inform_out_sleep_overhead_callback(
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
 * Name:  esp32c3_periph_inform_out_sleep_overhead
 *
 * Description:
 *   Inform peripherals of light sleep wakeup overhead time
 *
 * Input Parameters:
 *   us - light sleep wakeup overhead time
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void IRAM_ATTR esp32c3_periph_inform_out_sleep_overhead(uint32_t us)
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
 * Name:  esp32c3_sleep_enable_rtc_timer_wakeup
 *
 * Description:
 *   Configure RTC TIMER wake-up interval
 *
 * Input Parameters:
 *   time_in_us - Configure wake-up time interval
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void IRAM_ATTR esp32c3_sleep_enable_rtc_timer_wakeup(uint64_t time_in_us)
{
  s_config.wakeup_triggers |= RTC_TIMER_TRIG_EN;
  s_config.sleep_duration = time_in_us;
}

/****************************************************************************
 * Name:  esp32c3_sleep_enable_wifi_wakeup
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

void esp32c3_sleep_enable_wifi_wakeup(void)
{
  s_config.wakeup_triggers |= RTC_WIFI_TRIG_EN;
}

/****************************************************************************
 * Name:  esp32c3_light_sleep_start
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

int IRAM_ATTR esp32c3_light_sleep_start(uint64_t *sleep_time)
{
  int ret = OK;
  irqstate_t flags;
  uint32_t pd_flags;
  uint32_t flash_enable_time_us;
  uint32_t vddsdio_pd_sleep_duration;
  struct esp32c3_rtc_vddsdio_config_s vddsdio_config;
  uint32_t rtc_cntl_xtl_buf_wait_cycles;
  uint32_t sleep_time_overhead_in;
  uint32_t ccount_at_sleep_start;

  flags = enter_critical_section();

  s_config.ccount_ticks_record = esp32c3_cpu_cycle_count();
  s_config.rtc_ticks_at_sleep_start = esp32c3_rtc_time_get();
  ccount_at_sleep_start = esp32c3_cpu_cycle_count();
  sleep_time_overhead_in = (ccount_at_sleep_start -
    s_config.ccount_ticks_record) / (esp32c3_clk_cpu_freq() / 1000000ULL);

  /* Decide which power domains can be powered down */

  pd_flags = esp32c3_get_power_down_flags();

  s_config.rtc_clk_cal_period =
        esp32c3_rtc_clk_cal(RTC_CAL_RTC_MUX, RTC_CLK_SRC_CAL_CYCLES);

  /* Adjustment time consists of parts below:
   * 1. Hardware time waiting for internal 8M oscilate clock and XTAL;
   * 2. Hardware state swithing time of the rtc main state machine;
   * 3. Code execution time when clock is not stable;
   * 4. Code execution time which can be measured;
   */

  rtc_cntl_xtl_buf_wait_cycles = esp32c3_rtc_time_us_to_slowclk(
        RTC_CNTL_XTL_BUF_WAIT_SLP_US, s_config.rtc_clk_cal_period);

  s_config.sleep_time_adjustment = LIGHT_SLEEP_TIME_OVERHEAD_US +
        sleep_time_overhead_in + s_config.sleep_time_overhead_out
        + esp32c3_rtc_time_slowclk_to_us(rtc_cntl_xtl_buf_wait_cycles +
        RTC_CNTL_CK8M_WAIT_SLP_CYCLES + RTC_CNTL_WAKEUP_DELAY_CYCLES,
        s_config.rtc_clk_cal_period);

  /* Decide if VDD_SDIO needs to be powered down;
   * If it needs to be powered down, adjust sleep time.
   */

  flash_enable_time_us = VDD_SDIO_POWERUP_TO_FLASH_READ_US
                         + DEEP_SLEEP_WAKEUP_DELAY;

  /* When SPIRAM is disabled in menuconfig, the minimum sleep time of the
   * system needs to meet the sum below:
   * 1. Wait time for the flash power-on after waking up;
   * 2. The execution time of codes between RTC Timer get start time
   *    with hardware starts to switch state to sleep;
   * 3. The hardware state switching time of the rtc state machine during
   *    sleep and wake-up. This process requires 6 cycles to complete.
   *    The specific hardware state switching process and the cycles
   *    consumed are rtc_cpu_run_stall(1), cut_pll_rtl(2), cut_8m(1),
   *    min_protect(2);
   * 4. All the adjustment time which is
   *    s_config.sleep_time_adjustment below.
   */

  vddsdio_pd_sleep_duration = MAX(FLASH_PD_MIN_SLEEP_TIME_US,
      flash_enable_time_us + LIGHT_SLEEP_MIN_TIME_US +
      s_config.sleep_time_adjustment + esp32c3_rtc_time_slowclk_to_us(
        RTC_MODULE_SLEEP_PREPARE_CYCLES, s_config.rtc_clk_cal_period));

  if (s_config.sleep_duration > vddsdio_pd_sleep_duration)
    {
      pd_flags |= RTC_SLEEP_PD_VDDSDIO;
      if (s_config.sleep_time_overhead_out < flash_enable_time_us)
        {
          s_config.sleep_time_adjustment += flash_enable_time_us;
        }
    }
  else
    {
      if (s_config.sleep_time_overhead_out > flash_enable_time_us)
        {
          s_config.sleep_time_adjustment -= flash_enable_time_us;
        }
    }

  esp32c3_periph_inform_out_sleep_overhead(
    s_config.sleep_time_adjustment - sleep_time_overhead_in);

  esp32c3_get_vddsdio_config(&vddsdio_config);

  /* Enter sleep, then wait for flash to be ready on wakeup */

  ret = esp32c3_light_sleep_inner(pd_flags, flash_enable_time_us,
                                               vddsdio_config);

  if (sleep_time != NULL)
    {
      *sleep_time = esp32c3_rtc_time_slowclk_to_us(esp32c3_rtc_time_get() -
          s_config.rtc_ticks_at_sleep_start, s_config.rtc_clk_cal_period);
    }

  s_config.sleep_time_overhead_out = (esp32c3_cpu_cycle_count() -
       s_config.ccount_ticks_record) / (esp32c3_clk_cpu_freq() / 1000000ULL);

  leave_critical_section(flags);

  return ret;
}

/****************************************************************************
 * Name: esp32c3_pmstandby
 *
 * Description:
 *   Enter force sleep.
 *
 * Input Parameters:
 *   time_in_us - force sleep time interval
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32c3_pmstandby(uint64_t time_in_us)
{
  uint64_t rtc_diff_us;
#ifdef CONFIG_ESP32C3_RT_TIMER
  uint64_t hw_start_us;
  uint64_t hw_end_us;
  uint64_t hw_diff_us;
#endif

  /* don't power down XTAL — powering it up takes different time on. */

  esp32c3_sleep_enable_rtc_timer_wakeup(time_in_us);
#ifdef CONFIG_ESP32C3_RT_TIMER
  /* Get rt-timer timestamp before entering sleep */

  hw_start_us = rt_timer_time_us();
#endif

  esp32c3_light_sleep_start(&rtc_diff_us);

#ifdef CONFIG_ESP32C3_RT_TIMER
  /* Get rt-timer timestamp after waking up from sleep */

  hw_end_us = rt_timer_time_us();
  hw_diff_us = hw_end_us - hw_start_us;
  DEBUGASSERT(rtc_diff_us > hw_diff_us);

  rt_timer_calibration(rtc_diff_us - hw_diff_us);
#endif

#ifdef CONFIG_SCHED_TICKLESS
  up_step_idletime((uint32_t)time_in_us);
#endif

  pwrinfo("Returned from auto-sleep, slept for %" PRIu32 " ms\n",
            (uint32_t)(rtc_diff_us) / 1000);
}

/****************************************************************************
 * Name:  esp32c3_deep_sleep_start
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

void IRAM_ATTR esp32c3_deep_sleep_start(void)
{
  uint32_t pd_flags;

  /* record current RTC time */

  s_config.rtc_ticks_at_sleep_start = esp32c3_rtc_time_get();

  /* Decide which power domains can be powered down */

  pd_flags = esp32c3_get_power_down_flags();
  s_config.rtc_clk_cal_period = getreg32(RTC_SLOW_CLK_CAL_REG);

  /* Correct the sleep time */

  s_config.sleep_time_adjustment = DEEP_SLEEP_TIME_OVERHEAD_US;

  /* Enter deep sleep */

  esp32c3_sleep_start(RTC_SLEEP_PD_DIG | RTC_SLEEP_PD_VDDSDIO | pd_flags);

  /* Because RTC is in a slower clock domain than the CPU, it
   * can take several CPU cycles for the sleep mode to start.
   */

  while (1);
}

/****************************************************************************
 * Name: esp32c3_pmsleep
 *
 * Description:
 *   Enter deep sleep.
 *
 * Input Parameters:
 *   time_in_us - deep sleep time interval
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32c3_pmsleep(uint64_t time_in_us)
{
  esp32c3_sleep_enable_rtc_timer_wakeup(time_in_us);
  esp32c3_deep_sleep_start();
}

/****************************************************************************
 * Name: esp32c3_pm_lockacquire
 *
 * Description:
 *   Take a power management lock
 *
 ****************************************************************************/

void IRAM_ATTR esp32c3_pm_lockacquire(void)
{
  ++pm_wakelock;
}

/****************************************************************************
 * Name: esp32c3_pm_lockrelease
 *
 * Description:
 *   Release the lock taken using esp32c3_pm_lockacquire.
 *
 ****************************************************************************/

void IRAM_ATTR esp32c3_pm_lockrelease(void)
{
  --pm_wakelock;
}

/****************************************************************************
 * Name: esp32c3_pm_lockstatus
 *
 * Description:
 *   Return power management lock status.
 *
 ****************************************************************************/

uint32_t IRAM_ATTR esp32c3_pm_lockstatus(void)
{
  return pm_wakelock;
}

#endif /* CONFIG_PM */
