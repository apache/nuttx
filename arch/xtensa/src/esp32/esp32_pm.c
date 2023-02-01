/****************************************************************************
 * arch/xtensa/src/esp32/esp32_pm.c
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
#include <nuttx/power/pm.h>

#ifdef CONFIG_PM

#include <arch/chip/chip.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <sys/param.h>
#include <sys/time.h>
#include "hardware/esp32_rtccntl.h"
#include "hardware/esp32_uart.h"
#include "hardware/esp32_dport.h"
#include "xtensa.h"
#include "xtensa_attr.h"
#include "esp32_rtc.h"
#include "esp32_clockconfig.h"
#include "esp32_pm.h"
#include "esp32_resetcause.h"

#ifdef CONFIG_ESP32_RT_TIMER
#include "esp32_rt_timer.h"
#endif

#ifdef CONFIG_SCHED_TICKLESS
#include "esp32_tickless.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If light sleep time is less than that, don't power down flash */

#define FLASH_PD_MIN_SLEEP_TIME_US 2000

/* Minimal amount of time we can sleep for. */

#define LIGHT_SLEEP_MIN_TIME_US    200

/* Time from VDD_SDIO power up to first flash read in ROM code */

#define VDD_SDIO_POWERUP_TO_FLASH_READ_US 700

/* Extra time it takes to enter and exit light sleep and deep sleep */

#define LIGHT_SLEEP_TIME_OVERHEAD_US (250 + 30 * 240 / CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ)
#define DEEP_SLEEP_TIME_OVERHEAD_US  (250 + 100 * 240 / CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ)

#define CONFIG_ESP32_DEEP_SLEEP_WAKEUP_DELAY 2000

#define RTC_VDDSDIO_TIEH_1_8V       0  /* TIEH field value for 1.8V VDDSDIO */
#define RTC_VDDSDIO_TIEH_3_3V       1  /* TIEH field value for 3.3V VDDSDIO */

#define RTC_EXT0_TRIG_EN      BIT(0)  /* EXT0 GPIO wakeup */
#define RTC_EXT1_TRIG_EN      BIT(1)  /* EXT1 GPIO wakeup */
#define RTC_GPIO_TRIG_EN      BIT(2)  /* GPIO wakeup (light sleep only) */
#define RTC_TIMER_TRIG_EN     BIT(3)  /* Timer wakeup */
#define RTC_SDIO_TRIG_EN      BIT(4)  /* SDIO wakeup (light sleep only) */
#define RTC_MAC_TRIG_EN       BIT(5)  /* MAC wakeup (light sleep only) */
#define RTC_UART0_TRIG_EN     BIT(6)  /* UART0 wakeup (light sleep only) */
#define RTC_UART1_TRIG_EN     BIT(7)  /* UART1 wakeup (light sleep only) */
#define RTC_TOUCH_TRIG_EN     BIT(8)  /* Touch wakeup */
#define RTC_ULP_TRIG_EN       BIT(9)  /* ULP wakeup */
#define RTC_BT_TRIG_EN        BIT(10) /* BT wakeup (light sleep only) */

#define RTC_ENTRY_ADDR_REG    RTC_CNTL_STORE6_REG
#define RTC_MEMORY_CRC_REG    RTC_CNTL_STORE7_REG

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Power down options */

enum esp32_sleep_pd_option_e
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

enum esp_sleep_pd_domain_e
{
  ESP_PD_DOMAIN_RTC_PERIPH,      /* RTC IO, sensors and ULP co-processor */
  ESP_PD_DOMAIN_RTC_SLOW_MEM,    /* RTC slow memory */
  ESP_PD_DOMAIN_RTC_FAST_MEM,    /* RTC fast memory */
  ESP_PD_DOMAIN_XTAL,            /* XTAL oscillator */
  ESP_PD_DOMAIN_MAX              /* Number of domains */
};

/* Internal structure which holds all requested deep sleep parameters. */

struct esp32_sleep_config_t
{
  enum esp32_sleep_pd_option_e pd_options[ESP_PD_DOMAIN_MAX];
  uint64_t sleep_duration;
  uint32_t wakeup_triggers : 11;
  uint32_t ext1_trigger_mode : 1;
  uint32_t ext1_rtc_gpio_mask : 18;
  uint32_t ext0_trigger_level : 1;
  uint32_t ext0_rtc_gpio_num : 5;
  uint32_t sleep_time_adjustment;
  uint64_t rtc_ticks_at_sleep_start;
};

/* Structure describing vddsdio configuration. */

struct rtc_vddsdio_config_s
{
  uint32_t force : 1;     /* If 1, use configuration from RTC registers; if 0, use EFUSE/bootstrapping pins. */
  uint32_t enable : 1;    /* Enable VDDSDIO regulator */
  uint32_t tieh  : 1;     /* Select VDDSDIO voltage. One of RTC_VDDSDIO_TIEH_1_8V, RTC_VDDSDIO_TIEH_3_3V */
  uint32_t drefh : 2;     /* Tuning parameter for VDDSDIO regulator */
  uint32_t drefm : 2;     /* Tuning parameter for VDDSDIO regulator */
  uint32_t drefl : 2;     /* Tuning parameter for VDDSDIO regulator */
};

/* Function type for stub to run on wake from sleep */

typedef void (*esp_deep_sleep_wake_stub_fn_t)(void);

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void esp32_timer_wakeup_prepare(void);
static void IRAM_ATTR esp32_flush_uarts(void);
static void IRAM_ATTR esp32_suspend_uarts(void);
static void IRAM_ATTR esp32_resume_uarts(void);
static uint32_t esp32_get_power_down_flags(void);
static inline void esp32_uart_tx_wait_idle(uint8_t uart_no);
static void IRAM_ATTR esp32_set_vddsdio_config(
                      struct rtc_vddsdio_config_s config);
static int IRAM_ATTR esp32_get_vddsdio_config(
                      struct rtc_vddsdio_config_s *config);
int IRAM_ATTR esp32_light_sleep_inner(uint32_t pd_flags,
                      uint32_t time_us, struct rtc_vddsdio_config_s config);
static int IRAM_ATTR esp32_configure_cpu_freq(uint32_t cpu_freq_mhz);
static inline bool IRAM_ATTR esp32_ptr_executable(const void *p);
static void esp32_set_deep_sleep_wake_stub(
                      esp_deep_sleep_wake_stub_fn_t new_stub);
static void RTC_IRAM_ATTR esp32_wake_deep_sleep(void);
static esp_deep_sleep_wake_stub_fn_t esp32_get_deep_sleep_wake_stub(void);

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern uint32_t g_ticks_per_us_pro;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct esp32_sleep_config_t s_config =
{
    .pd_options =
              { ESP_PD_OPTION_AUTO, ESP_PD_OPTION_AUTO, ESP_PD_OPTION_AUTO },
    .wakeup_triggers = 0
};

static _Atomic uint32_t pm_wakelock = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* CPU do while loop for some time. */

extern void ets_delay_us(uint32_t us);

/* Set CRC of Fast RTC memory 0-0x7ff into RTC STORE7. */

extern void set_rtc_memory_crc(void);

/* Set the real CPU ticks per us to the ets,
 * so that ets_delay_us will be accurate.
 */

extern void ets_update_cpu_frequency_rom(uint32_t ticks_per_us);

/* Get xtal_freq value, If value not stored in RTC_STORE5, than store. */

extern uint32_t ets_get_detected_xtal_freq(void);

/****************************************************************************
 * Name: esp32_uart_tx_wait_idle
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

static inline void esp32_uart_tx_wait_idle(uint8_t uart_no)
{
  uint32_t status;
  do
    {
      status = getreg32(UART_STATUS_REG(uart_no));

      /* either tx count or state is non-zero */
    }
  while ((status & (UART_ST_UTX_OUT_M | UART_TXFIFO_CNT_M)) != 0);
}

/****************************************************************************
 * Name: esp32_flush_uarts
 *
 * Description:
 *   Wait until UART0/UART1/UART2 tx full empty and the last char send ok
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void IRAM_ATTR esp32_flush_uarts(void)
{
  int i;

  for (i = 0; i < ESP32_NUARTS; ++i)
    {
      esp32_uart_tx_wait_idle(i);
    }
}

/****************************************************************************
 * Name: esp32_suspend_uarts
 *
 * Description:
 *   Suspend UART0/UART1/UART2 output
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void IRAM_ATTR esp32_suspend_uarts(void)
{
  int i;

  for (i = 0; i < ESP32_NUARTS; ++i)
    {
      modifyreg32(UART_FLOW_CONF_REG(i), 0, UART_FORCE_XOFF);
      while (REG_GET_FIELD(UART_STATUS_REG(i), UART_ST_UTX_OUT) != 0);
    }
}

/****************************************************************************
 * Name: esp32_resume_uarts
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

static void IRAM_ATTR esp32_resume_uarts(void)
{
  int i;

  for (i = 0; i < ESP32_NUARTS; ++i)
    {
      modifyreg32(UART_FLOW_CONF_REG(i), UART_FORCE_XOFF, 0);
      modifyreg32(UART_FLOW_CONF_REG(i), 0, UART_FORCE_XON);
      modifyreg32(UART_FLOW_CONF_REG(i), UART_FORCE_XON, 0);
    }
}

/****************************************************************************
 * Name:  esp32_get_power_down_flags
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

static uint32_t esp32_get_power_down_flags(void)
{
  uint32_t pd_flags = 0;

  if (s_config.pd_options[ESP_PD_DOMAIN_RTC_FAST_MEM] == ESP_PD_OPTION_AUTO)
    {
      s_config.pd_options[ESP_PD_DOMAIN_RTC_FAST_MEM] = ESP_PD_OPTION_ON;
    }

  if (s_config.pd_options[ESP_PD_DOMAIN_RTC_PERIPH] == ESP_PD_OPTION_AUTO)
    {
      if (s_config.wakeup_triggers & (RTC_EXT0_TRIG_EN | RTC_GPIO_TRIG_EN))
        {
          s_config.pd_options[ESP_PD_DOMAIN_RTC_PERIPH] = ESP_PD_OPTION_ON;
        }
      else if (s_config.wakeup_triggers &
              (RTC_TOUCH_TRIG_EN | RTC_ULP_TRIG_EN))
        {
          /* In both rev. 0 and rev. 1 of ESP32,
           * forcing power up of prevents ULP timer
           * and touch FSMs from working correctly.
           */

          s_config.pd_options[ESP_PD_DOMAIN_RTC_PERIPH] = ESP_PD_OPTION_OFF;
        }
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

  if (s_config.pd_options[ESP_PD_DOMAIN_RTC_SLOW_MEM] != ESP_PD_OPTION_ON)
    {
      pd_flags |= RTC_SLEEP_PD_RTC_SLOW_MEM;
    }

  if (s_config.pd_options[ESP_PD_DOMAIN_RTC_PERIPH] != ESP_PD_OPTION_ON)
    {
      pd_flags |= RTC_SLEEP_PD_RTC_PERIPH;
    }

  if (s_config.pd_options[ESP_PD_DOMAIN_XTAL] != ESP_PD_OPTION_ON)
    {
      pd_flags |= RTC_SLEEP_PD_XTAL;
    }

  return pd_flags;
}

/****************************************************************************
 * Name:  esp32_timer_wakeup_prepare
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

static void esp32_timer_wakeup_prepare(void)
{
  uint32_t period;
  int64_t sleep_duration;
  int64_t rtc_count_delta;

  period = getreg32(RTC_SLOW_CLK_CAL_REG);
  sleep_duration = (int64_t) s_config.sleep_duration -
                   (int64_t) s_config.sleep_time_adjustment;

  if (sleep_duration < 0)
    {
      sleep_duration = 0;
    }

  rtc_count_delta = esp32_rtc_time_us_to_slowclk(sleep_duration, period);
  esp32_rtc_sleep_set_wakeup_time(s_config.rtc_ticks_at_sleep_start +
                                  rtc_count_delta);
}

/****************************************************************************
 * Name: esp32_set_vddsdio_config
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

static void IRAM_ATTR esp32_set_vddsdio_config(
                      struct rtc_vddsdio_config_s config)
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
 * Name: esp32_get_vddsdio_config
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

static int IRAM_ATTR esp32_get_vddsdio_config(
                     struct rtc_vddsdio_config_s *config)
{
  struct rtc_vddsdio_config_s *result = config;
  uint32_t efuse_reg;
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

  efuse_reg = getreg32(EFUSE_BLK0_RDATA4_REG);

  if (efuse_reg & EFUSE_RD_SDIO_FORCE)
    {
      /* Get configuration from EFUSE */

      result->force = 0;
      result->enable = (efuse_reg & EFUSE_RD_XPD_SDIO_REG_M)
                                  >> EFUSE_RD_XPD_SDIO_REG_S;
      result->tieh = (efuse_reg & EFUSE_RD_SDIO_TIEH_M)
                                >> EFUSE_RD_SDIO_TIEH_S;

      if (REG_GET_FIELD(EFUSE_BLK0_RDATA3_REG,
              EFUSE_RD_BLK3_PART_RESERVE) == 0)
        {
          result->drefh = (efuse_reg & EFUSE_RD_SDIO_DREFH_M)
                                     >> EFUSE_RD_SDIO_DREFH_S;
          result->drefm = (efuse_reg & EFUSE_RD_SDIO_DREFM_M)
                                     >> EFUSE_RD_SDIO_DREFM_S;
          result->drefl = (efuse_reg & EFUSE_RD_SDIO_DREFL_M)
                                     >> EFUSE_RD_SDIO_DREFL_S;
        }

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
 * Name: esp32_sleep_start
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

static int IRAM_ATTR esp32_sleep_start(uint32_t pd_flags)
{
  int result;
  uint32_t cur_freq;

  /* Stop UART output so that output is not lost due to APB frequency change.
   * For light sleep, suspend UART output — it will resume after wakeup.
   * For deep sleep, wait for the contents of UART FIFO to be sent.
   */

  if (pd_flags & RTC_SLEEP_PD_DIG)
    {
      esp32_flush_uarts();
    }
  else
    {
      esp32_suspend_uarts();
    }

  /* Save current frequency and switch to XTAL */

  cur_freq = esp_rtc_clk_get_cpu_freq();
  esp32_rtc_cpu_freq_set_xtal();

  /* Enter sleep */

  esp32_rtc_sleep_init(pd_flags);

  /* Configure timer wakeup */

  if ((s_config.wakeup_triggers & RTC_TIMER_TRIG_EN)
                     && s_config.sleep_duration > 0)
    {
      esp32_timer_wakeup_prepare();
    }

  result = esp32_rtc_sleep_start(s_config.wakeup_triggers, 0);

  /* Restore CPU frequency */

  if (esp32_configure_cpu_freq(cur_freq) != OK)
    {
      pwrwarn("WARNING: Failed to restore CPU frequency"
              "Configure cpu frequency %d.\n", cur_freq);
    }

  /* Re-enable UART output */

  esp32_resume_uarts();

  return result;
}

/****************************************************************************
 * Name: esp32_light_sleep_inner
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

int esp32_light_sleep_inner(uint32_t pd_flags,
        uint32_t time_us, struct rtc_vddsdio_config_s config)
{
  /* Enter sleep */

  int err = esp32_sleep_start(pd_flags);

  /* If VDDSDIO regulator was controlled by RTC registers before sleep.
   * restore the configuration.
   */

  if (config.force)
    {
      esp32_set_vddsdio_config(config);
    }

  /* If SPI flash was powered down, wait for it to become ready. */

  if (pd_flags & RTC_SLEEP_PD_VDDSDIO)
    {
      /* Wait for the flash chip to start up. */

      ets_delay_us(time_us);
    }

  return err;
}

/****************************************************************************
 * Name: esp32_configure_cpu_freq
 *
 * Description:
 *   Switch to new CPU frequencies.
 *
 * Input Parameters:
 *   cpu_freq_mhz - new CPU frequency
 *
 * Returned Value:
 *   0 is returned on success or a negated errno value is returned
 *
 ****************************************************************************/

static int IRAM_ATTR esp32_configure_cpu_freq(uint32_t cpu_freq_mhz)
{
  uint32_t soc_clk_sel;
  uint32_t source_freq_mhz;
  enum esp32_rtc_xtal_freq_e xtal_freq;

  if (cpu_freq_mhz == 240)
    {
      source_freq_mhz = RTC_PLL_FREQ_480M;
    }
  else if(cpu_freq_mhz == 80 || cpu_freq_mhz == 160)
    {
      source_freq_mhz = RTC_PLL_FREQ_320M;
    }
  else
    {
      return -EINVAL;
    }

  xtal_freq = esp32_rtc_clk_xtal_freq_get();
  soc_clk_sel = REG_GET_FIELD(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_SOC_CLK_SEL);

  if (soc_clk_sel != RTC_CNTL_SOC_CLK_SEL_XTL)
    {
      esp32_rtc_update_to_xtal(xtal_freq, 1);
      esp32_rtc_wait_for_slow_cycle();
    }

  if (soc_clk_sel == RTC_CNTL_SOC_CLK_SEL_PLL)
    {
      esp32_rtc_bbpll_disable();
    }

  esp32_rtc_bbpll_enable();
  esp32_rtc_wait_for_slow_cycle();
  esp32_rtc_bbpll_configure(xtal_freq, source_freq_mhz);
  esp32_set_cpu_freq(cpu_freq_mhz);

  return OK;
}

/****************************************************************************
 * Name:  esp32_ptr_executable
 *
 * Description:
 *   Check if point p in a compatible memory area of IRAM
 *
 * Input Parameters:
 *   p - Memory address
 *
 * Returned Value:
 *    True if in memory area or false if not.
 *
 ****************************************************************************/

static inline bool IRAM_ATTR esp32_ptr_executable(const void *p)
{
  intptr_t ip = (intptr_t) p;

  return (ip >= SOC_IROM_LOW && ip < SOC_IROM_HIGH)
        || (ip >= SOC_IRAM_LOW && ip < SOC_IRAM_HIGH)
        || (ip >= SOC_IROM_MASK_LOW && ip < SOC_IROM_MASK_HIGH)
#if defined(SOC_CACHE_APP_LOW)
        || (ip >= SOC_CACHE_APP_LOW && ip < SOC_CACHE_APP_HIGH)
#endif
        || (ip >= SOC_RTC_IRAM_LOW && ip < SOC_RTC_IRAM_HIGH);
}

/****************************************************************************
 * Name:  esp32_set_deep_sleep_wake_stub
 *
 * Description:
 *   Install a new stub at runtime to run on wake from deep sleep.
 *
 * Input Parameters:
 *   new_stub - Function type for stub to run on wake from sleep
 *
 * Returned Value:
 *    None.
 *
 ****************************************************************************/

static void esp32_set_deep_sleep_wake_stub(
                      esp_deep_sleep_wake_stub_fn_t new_stub)
{
  putreg32((uint32_t)new_stub, RTC_ENTRY_ADDR_REG);
  set_rtc_memory_crc();
}

/****************************************************************************
 * Name:  esp32_get_deep_sleep_wake_stub
 *
 * Description:
 *   Get current wake from deep sleep stub.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *    Current wake from deep sleep stub, or NULL if no stub is installed.
 *
 ****************************************************************************/

static esp_deep_sleep_wake_stub_fn_t esp32_get_deep_sleep_wake_stub(void)
{
  uint32_t stored_crc = 0;
  uint32_t calc_crc = 0;
  esp_deep_sleep_wake_stub_fn_t stub_ptr = NULL;

  stored_crc = getreg32(RTC_MEMORY_CRC_REG);
  set_rtc_memory_crc();
  calc_crc = getreg32(RTC_MEMORY_CRC_REG);
  putreg32(stored_crc, RTC_MEMORY_CRC_REG);

  if (stored_crc != calc_crc)
    {
      return NULL;
    }

  stub_ptr = (esp_deep_sleep_wake_stub_fn_t) getreg32(RTC_ENTRY_ADDR_REG);
  if (!esp32_ptr_executable(stub_ptr))
    {
      return NULL;
    }

  return stub_ptr;
}

/****************************************************************************
 * Name:  esp32_wake_deep_sleep
 *
 * Description:
 *   Default stub to run on wake from deep sleep.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *    None
 *
 ****************************************************************************/

static void RTC_IRAM_ATTR esp32_wake_deep_sleep(void)
{
  /* Clear MMU for CPU 0 */

  putreg32(getreg32(DPORT_PRO_CACHE_CTRL1_REG) | DPORT_PRO_CACHE_MMU_IA_CLR,
                    DPORT_PRO_CACHE_CTRL1_REG);
  putreg32(getreg32(DPORT_PRO_CACHE_CTRL1_REG) &
                   (~DPORT_PRO_CACHE_MMU_IA_CLR), DPORT_PRO_CACHE_CTRL1_REG);

#if CONFIG_ESP32_DEEP_SLEEP_WAKEUP_DELAY > 0
  /* ROM code has not started yet, so we need to set delay factor
   * used by ets_delay_us first.
   */

  ets_update_cpu_frequency_rom(ets_get_detected_xtal_freq() / 1000000);

  /* This delay is configured in menuconfig, it can be used to give
   * the flash chip some time to become ready.
   */

  ets_delay_us(CONFIG_ESP32_DEEP_SLEEP_WAKEUP_DELAY);
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  esp32_sleep_enable_timer_wakeup
 *
 * Description:
 *   Configure wake-up interval
 *
 * Input Parameters:
 *   time_in_us - Configure wake-up time interval
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void IRAM_ATTR esp32_sleep_enable_timer_wakeup(uint64_t time_in_us)
{
  s_config.wakeup_triggers |= RTC_TIMER_TRIG_EN;
  s_config.sleep_duration = time_in_us;
}

/****************************************************************************
 * Name:  esp32_light_sleep_start
 *
 * Description:
 *   Enter sleep mode
 *
 * Input Parameters:
 *   sleep_time - Actual sleep time
 *
 * Returned Value:
 *   0 is returned on success or a negated errno value is returned
 *
 ****************************************************************************/

int esp32_light_sleep_start(uint64_t *sleep_time)
{
  irqstate_t flags;
  uint32_t pd_flags;
  uint32_t flash_enable_time_us;
#ifndef CONFIG_ESP32_SPIRAM
  uint32_t vddsdio_pd_sleep_duration;
#endif
  struct rtc_vddsdio_config_s vddsdio_config;
  int ret = OK;

  flags = enter_critical_section();
  s_config.rtc_ticks_at_sleep_start = esp32_rtc_time_get();

  /* Decide which power domains can be powered down */

  pd_flags = esp32_get_power_down_flags();

  /* Amount of time to subtract from actual sleep time.
   * This is spent on entering and leaving light sleep.
   */

  s_config.sleep_time_adjustment = LIGHT_SLEEP_TIME_OVERHEAD_US;

  /* Decide if VDD_SDIO needs to be powered down;
   * If it needs to be powered down, adjust sleep time.
   */

  flash_enable_time_us = VDD_SDIO_POWERUP_TO_FLASH_READ_US
                         + CONFIG_ESP32_DEEP_SLEEP_WAKEUP_DELAY;

#ifndef CONFIG_ESP32_SPIRAM
  vddsdio_pd_sleep_duration = MAX(FLASH_PD_MIN_SLEEP_TIME_US,
              flash_enable_time_us + LIGHT_SLEEP_TIME_OVERHEAD_US
                                        + LIGHT_SLEEP_MIN_TIME_US);

  if (s_config.sleep_duration > vddsdio_pd_sleep_duration)
    {
      pd_flags |= RTC_SLEEP_PD_VDDSDIO;
      s_config.sleep_time_adjustment += flash_enable_time_us;
    }
#endif

  esp32_get_vddsdio_config(&vddsdio_config);

  /* Enter sleep, then wait for flash to be ready on wakeup */

  ret = esp32_light_sleep_inner(pd_flags, flash_enable_time_us,
                                               vddsdio_config);
  if (sleep_time != NULL)
    {
      *sleep_time = esp32_rtc_time_slowclk_to_us(esp32_rtc_time_get() -
          s_config.rtc_ticks_at_sleep_start, esp32_clk_slowclk_cal_get());
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: esp32_pminit
 *
 * Description:
 *   Initialize force sleep parameters.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32_pminit(void)
{
  /* Initialize RTC parameters */

  esp32_rtc_init();
  esp32_rtc_clk_set();
}

/****************************************************************************
 * Name: esp32_pmstandby
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

void esp32_pmstandby(uint64_t time_in_us)
{
  uint64_t rtc_diff_us;
#ifdef CONFIG_ESP32_RT_TIMER
  uint64_t hw_start_us;
  uint64_t hw_end_us;
  uint64_t hw_diff_us;
#endif

  /* don't power down XTAL — powering it up takes different time on. */

  esp32_sleep_enable_timer_wakeup(time_in_us);

#ifdef CONFIG_ESP32_RT_TIMER
  /* Get rt-timer timestamp before entering sleep */

  hw_start_us = rt_timer_time_us();
#endif

  esp32_light_sleep_start(&rtc_diff_us);

#ifdef CONFIG_ESP32_RT_TIMER
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
 * Name: esp32_sleep_get_wakeup_cause
 *
 * Description:
 *   Get the wakeup source which caused wakeup from sleep.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Cause of wake up from last sleep
 *   (one of enum esp32_sleep_source_e values).
 *
 ****************************************************************************/

enum esp32_sleep_source_e esp32_sleep_get_wakeup_cause(void)
{
  uint32_t wakeup_cause;
  if (esp32_resetcause(0) != ESP32_RESETCAUSE_CORE_DPSP)
    {
      return ESP_SLEEP_WAKEUP_UNDEFINED;
    }

  wakeup_cause = REG_GET_FIELD(RTC_CNTL_WAKEUP_STATE_REG,
                                    RTC_CNTL_WAKEUP_CAUSE);
  if (wakeup_cause & RTC_EXT0_TRIG_EN)
    {
      return ESP_SLEEP_WAKEUP_EXT0;
    }
  else if (wakeup_cause & RTC_EXT1_TRIG_EN)
    {
      return ESP_SLEEP_WAKEUP_EXT1;
    }
  else if (wakeup_cause & RTC_TIMER_TRIG_EN)
    {
      return ESP_SLEEP_WAKEUP_TIMER;
    }
  else if (wakeup_cause & RTC_TOUCH_TRIG_EN)
    {
      return ESP_SLEEP_WAKEUP_TOUCHPAD;
    }
  else if (wakeup_cause & RTC_ULP_TRIG_EN)
    {
      return ESP_SLEEP_WAKEUP_ULP;
    }
  else if (wakeup_cause & RTC_GPIO_TRIG_EN)
    {
      return ESP_SLEEP_WAKEUP_GPIO;
    }
  else if (wakeup_cause & (RTC_UART0_TRIG_EN | RTC_UART1_TRIG_EN))
    {
      return ESP_SLEEP_WAKEUP_UART;
    }
  else
    {
      return ESP_SLEEP_WAKEUP_UNDEFINED;
    }
}

/****************************************************************************
 * Name:  esp32_deep_sleep_start
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

void esp32_deep_sleep_start(void)
{
  uint32_t pd_flags;

  /* record current RTC time */

  s_config.rtc_ticks_at_sleep_start = esp32_rtc_time_get();

  /* Configure wake stub */

  if (esp32_get_deep_sleep_wake_stub() == NULL)
    {
      esp32_set_deep_sleep_wake_stub(esp32_wake_deep_sleep);
    }

  /* Decide which power domains can be powered down */

  pd_flags = esp32_get_power_down_flags();

  /* Correct the sleep time */

  s_config.sleep_time_adjustment = DEEP_SLEEP_TIME_OVERHEAD_US;

  /* Enter deep sleep */

  esp32_sleep_start(RTC_SLEEP_PD_DIG | RTC_SLEEP_PD_VDDSDIO
                              | RTC_SLEEP_PD_XTAL | pd_flags);

  /* Because RTC is in a slower clock domain than the CPU, it
   * can take several CPU cycles for the sleep mode to start.
   */

  while (1);
}

/****************************************************************************
 * Name: esp32_pmsleep
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

void esp32_pmsleep(uint64_t time_in_us)
{
  esp32_sleep_enable_timer_wakeup(time_in_us);
  esp32_deep_sleep_start();
}

/****************************************************************************
 * Name: esp32_pm_lockacquire
 *
 * Description:
 *   Take a power management lock
 *
 ****************************************************************************/

void IRAM_ATTR esp32_pm_lockacquire(void)
{
  ++pm_wakelock;
}

/****************************************************************************
 * Name: esp32_pm_lockrelease
 *
 * Description:
 *   Release the lock taken using esp32_pm_lockacquire.
 *
 ****************************************************************************/

void IRAM_ATTR esp32_pm_lockrelease(void)
{
  --pm_wakelock;
}

/****************************************************************************
 * Name: esp32_pm_lockstatus
 *
 * Description:
 *   Return power management lock status.
 *
 ****************************************************************************/

uint32_t IRAM_ATTR esp32_pm_lockstatus(void)
{
  return pm_wakelock;
}

#endif /* CONFIG_PM */
