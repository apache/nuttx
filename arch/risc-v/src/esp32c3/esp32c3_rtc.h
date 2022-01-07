/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_rtc.h
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

#ifndef __ARCH_RISCV_SRC_ESP32C3_ESP32C3_RTC_H
#define __ARCH_RISCV_SRC_ESP32C3_ESP32C3_RTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/timers/rtc.h>
#include <sys/types.h>
#include <time.h>
#include "hardware/esp32c3_soc.h"

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Number of cycles to wait from the 32k XTAL oscillator to
 * consider it running. Larger values increase startup delay.
 * Smaller values may cause false positive detection
 * (i.e. oscillator runs for a few cycles and then stops).
 */

#define SLOW_CLK_CAL_CYCLES         1024

/* Cycles for RTC Timer clock source (internal oscillator) calibrate */

#define RTC_CLK_SRC_CAL_CYCLES           (10)

/* Various delays to be programmed into power control state machines */

#define RTC_CNTL_XTL_BUF_WAIT_SLP_US            (250)
#define RTC_CNTL_PLL_BUF_WAIT_SLP_CYCLES        (1)
#define RTC_CNTL_CK8M_WAIT_SLP_CYCLES           (4)
#define RTC_CNTL_WAKEUP_DELAY_CYCLES            (5)

#define RTC_SLOW_CLK_CAL_REG    RTC_CNTL_STORE1_REG
#define RTC_BOOT_TIME_LOW_REG   RTC_CNTL_STORE2_REG
#define RTC_BOOT_TIME_HIGH_REG  RTC_CNTL_STORE3_REG
#define RTC_XTAL_FREQ_REG       RTC_CNTL_STORE4_REG
#define RTC_APB_FREQ_REG        RTC_CNTL_STORE5_REG
#define RTC_ENTRY_ADDR_REG      RTC_CNTL_STORE6_REG
#define RTC_RESET_CAUSE_REG     RTC_CNTL_STORE6_REG
#define RTC_MEMORY_CRC_REG      RTC_CNTL_STORE7_REG

#define RTC_SLEEP_PD_DIG                BIT(0)  /* Deep sleep */
#define RTC_SLEEP_PD_RTC_PERIPH         BIT(1)  /* Power down RTC peripherals */
#define RTC_SLEEP_PD_RTC_SLOW_MEM       BIT(2)  /* Power down RTC SLOW memory */
#define RTC_SLEEP_PD_RTC_FAST_MEM       BIT(3)  /* Power down RTC FAST memory */

/* RTC FAST and SLOW memories are automatically
 * powered up and down along with the CPU
 */

#define RTC_SLEEP_PD_RTC_MEM_FOLLOW_CPU BIT(4)
#define RTC_SLEEP_PD_VDDSDIO            BIT(5)  /* Power down VDDSDIO regulator */
#define RTC_SLEEP_PD_WIFI               BIT(6)  /* Power down Wi-Fi */
#define RTC_SLEEP_PD_BT                 BIT(7)  /* Power down BT */
#define RTC_SLEEP_PD_CPU                BIT(8)  /* Power down CPU when in light-sleep */
#define RTC_SLEEP_PD_DIG_PERIPH         BIT(9)  /* Power down DIG peripherals */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Clock source to be calibrated using rtc_clk_cal function */

enum esp32c3_rtc_cal_sel_e
{
  RTC_CAL_RTC_MUX = 0,       /* Currently selected RTC SLOW_CLK */
  RTC_CAL_8MD256 = 1,        /* Internal 8 MHz RC oscillator, divided by 256 */
  RTC_CAL_32K_XTAL = 2       /* External 32 kHz XTAL */
};

/* CPU clock source */

enum esp32c3_rtc_cpu_freq_src_e
{
  RTC_CPU_FREQ_SRC_XTAL,  /* XTAL */
  RTC_CPU_FREQ_SRC_PLL,   /* PLL (480M or 320M) */
  RTC_CPU_FREQ_SRC_8M,    /* Internal 8M RTC oscillator */
  RTC_CPU_FREQ_SRC_APLL   /* APLL */
};

/* Possible main XTAL frequency values.
 * Enum values should be equal to frequency in MHz.
 */

enum esp32c3_rtc_xtal_freq_e
{
  RTC_XTAL_FREQ_32M = 32,
  RTC_XTAL_FREQ_40M = 40,
};

/* RTC SLOW_CLK frequency values */

enum esp32c3_rtc_slow_freq_e
{
  RTC_SLOW_FREQ_RTC = 0,      /* Internal 150 kHz RC oscillator */
  RTC_SLOW_FREQ_32K_XTAL = 1, /* External 32 kHz XTAL */
  RTC_SLOW_FREQ_8MD256 = 2,   /* Internal 8 MHz RC oscillator, divided by 256 */
};

/* Periph module values */

enum esp32c3_periph_module_e
{
  PERIPH_LEDC_MODULE = 0,
  PERIPH_UART0_MODULE,
  PERIPH_UART1_MODULE,
  PERIPH_USB_DEVICE_MODULE,
  PERIPH_I2C0_MODULE,
  PERIPH_I2S1_MODULE,
  PERIPH_TIMG0_MODULE,
  PERIPH_TIMG1_MODULE,
  PERIPH_UHCI0_MODULE,
  PERIPH_RMT_MODULE,
  PERIPH_SPI_MODULE,
  PERIPH_SPI2_MODULE,
  PERIPH_TWAI_MODULE,
  PERIPH_RNG_MODULE,
  PERIPH_WIFI_MODULE,
  PERIPH_BT_MODULE,
  PERIPH_WIFI_BT_COMMON_MODULE,
  PERIPH_BT_BASEBAND_MODULE,
  PERIPH_BT_LC_MODULE,
  PERIPH_RSA_MODULE,
  PERIPH_AES_MODULE,
  PERIPH_SHA_MODULE,
  PERIPH_HMAC_MODULE,
  PERIPH_DS_MODULE,
  PERIPH_GDMA_MODULE,
  PERIPH_SYSTIMER_MODULE,
  PERIPH_SARADC_MODULE,
  PERIPH_MODULE_MAX
};

/* CPU clock configuration structure */

struct esp32c3_cpu_freq_config_s
{
  /* The clock from which CPU clock is derived */

  enum esp32c3_rtc_cpu_freq_src_e source;
  uint32_t source_freq_mhz;    /* Source clock frequency */
  uint32_t div;                /* Divider, freq_mhz = source_freq_mhz / div */
  uint32_t freq_mhz;           /* CPU clock frequency */
};

#ifdef CONFIG_RTC_ALARM

/* The form of an alarm callback */

typedef void (*alm_callback_t)(void *arg, unsigned int alarmid);

enum alm_id_e
{
  RTC_ALARM0 = 0,           /* RTC ALARM 0 */
  RTC_ALARM1 = 1,           /* RTC ALARM 1 */
  RTC_ALARM_LAST,
};

/* Structure used to pass parameters to set an alarm */

struct alm_setalarm_s
{
  int               as_id;     /* enum alm_id_e */
  struct timespec   as_time;   /* Alarm expiration time */
  alm_callback_t    as_cb;     /* Callback (if non-NULL) */
  void          *as_arg;       /* Argument for callback */
};

#endif /* CONFIG_RTC_ALARM */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_rtc_clk_xtal_freq_get
 *
 * Description:
 *   Get main XTAL frequency
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   XTAL frequency (one of enum esp32c3_rtc_xtal_freq_e values)
 *
 ****************************************************************************/

enum esp32c3_rtc_xtal_freq_e esp32c3_rtc_clk_xtal_freq_get(void);

/****************************************************************************
 * Name: esp32c3_rtc_clk_slow_freq_get
 *
 * Description:
 *   Get the RTC_SLOW_CLK source
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Currently selected clock source
 *   (one of enum esp32c3_rtc_slow_freq_e values)
 *
 ****************************************************************************/

enum esp32c3_rtc_slow_freq_e esp32c3_rtc_clk_slow_freq_get(void);

/****************************************************************************
 * Name: esp32c3_rtc_clk_slow_freq_set
 *
 * Description:
 *   Select source for RTC_SLOW_CLK
 *
 * Input Parameters:
 *   slow_freq - clock source (one of esp32c3_rtc_slow_freq_e values)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32c3_rtc_clk_slow_freq_set(enum esp32c3_rtc_slow_freq_e slow_freq);

/****************************************************************************
 * Name: esp32c3_rtc_clk_set
 *
 * Description:
 *   Set RTC CLK frequency.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32c3_rtc_clk_set(void);

/****************************************************************************
 * Name: esp32c3_rtc_init
 *
 * Description:
 *   Initialize RTC clock and power control related functions.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32c3_rtc_init(void);

/****************************************************************************
 * Name: esp32c3_periph_ll_periph_enabled
 *
 * Description:
 *   Whether the current Periph module is enabled
 *
 * Input Parameters:
 *   periph - Periph module (one of enum esp32c3_periph_module_e values)
 *
 * Returned Value:
 *   Periph module is enabled or not
 *
 ****************************************************************************/

bool IRAM_ATTR esp32c3_periph_ll_periph_enabled(
                                     enum esp32c3_periph_module_e periph);

/****************************************************************************
 * Name:  esp32c3_perip_clk_init
 *
 * Description:
 *   This function disables clock of useless peripherals when cpu starts.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32c3_perip_clk_init(void);

/****************************************************************************
 * Name: esp32c3_rtc_time_get
 *
 * Description:
 *   Get current value of RTC counter.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   current value of RTC counter
 *
 ****************************************************************************/

uint64_t esp32c3_rtc_time_get(void);

/****************************************************************************
 * Name: esp32c3_rtc_time_us_to_slowclk
 *
 * Description:
 *   Convert time interval from microseconds to RTC_SLOW_CLK cycles.
 *
 * Input Parameters:
 *   time_in_us      - Time interval in microseconds
 *   slow_clk_period -  Period of slow clock in microseconds
 *
 * Returned Value:
 *   number of slow clock cycles
 *
 ****************************************************************************/

uint64_t esp32c3_rtc_time_us_to_slowclk(uint64_t time_in_us,
                                        uint32_t period);

/****************************************************************************
 * Name: esp32c3_rtc_cpu_freq_set_xtal
 *
 * Description:
 *   Switch CPU clock source to XTAL
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32c3_rtc_cpu_freq_set_xtal(void);

/****************************************************************************
 * Name: esp32c3_rtc_sleep_init
 *
 * Description:
 *   Prepare the chip to enter sleep mode
 *
 * Input Parameters:
 *   flags - sleep mode configuration
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32c3_rtc_sleep_init(uint32_t flags);

/****************************************************************************
 * Name: esp32c3_rtc_sleep_start
 *
 * Description:
 *   Enter force sleep mode.
 *
 * Input Parameters:
 *   wakeup_opt - bit mask wake up reasons to enable
 *   reject_opt - bit mask of sleep reject reasons.
 *
 * Returned Value:
 *   non-zero if sleep was rejected by hardware
 *
 ****************************************************************************/

uint32_t esp32c3_rtc_sleep_start(uint32_t wakeup_opt, uint32_t reject_opt,
                                 uint32_t lslp_mem_inf_fpu);

/****************************************************************************
 * Name: esp32c3_rtc_clk_cal
 *
 * Description:
 *   Measure RTC slow clock's period, based on main XTAL frequency
 *
 * Input Parameters:
 *   cal_clk        - clock to be measured
 *   slowclk_cycles - number of slow clock cycles to average
 *
 * Returned Value:
 *   Average slow clock period in microseconds, Q13.19 fixed point format
 *   or 0 if calibration has timed out
 *
 ****************************************************************************/

uint32_t esp32c3_rtc_clk_cal(enum esp32c3_rtc_cal_sel_e cal_clk,
                             uint32_t slowclk_cycles);

/****************************************************************************
 * Name: esp32c3_rtc_time_slowclk_to_us
 *
 * Description:
 *   Convert time interval from RTC_SLOW_CLK to microseconds
 *
 * Input Parameters:
 *   rtc_cycles - Time interval in RTC_SLOW_CLK cycles
 *   period     - Period of slow clock in microseconds
 *
 * Returned Value:
 *   time interval in microseconds
 *
 ****************************************************************************/

uint64_t esp32c3_rtc_time_slowclk_to_us(uint64_t rtc_cycles,
                                                  uint32_t period);

/****************************************************************************
 * Name: esp32c3_rtc_deep_sleep_start
 *
 * Description:
 *   Enter deep sleep mode.
 *
 * Input Parameters:
 *   wakeup_opt - bit mask wake up reasons to enable
 *   reject_opt - bit mask of sleep reject reasons.
 *
 * Returned Value:
 *   non-zero if sleep was rejected by hardware
 *
 ****************************************************************************/

uint32_t esp32c3_rtc_deep_sleep_start(uint32_t wakeup_opt,
                                                uint32_t reject_opt);

/****************************************************************************
 * Name: esp32c3_rtc_clk_cpu_freq_set_config
 *
 * Description:
 *   Set CPU frequency configuration.
 *
 * Input Parameters:
 *   config - CPU frequency configuration
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32c3_rtc_clk_cpu_freq_set_config(
               const struct esp32c3_cpu_freq_config_s *config);

/****************************************************************************
 * Name: esp32c3_rtc_sleep_low_init
 *
 * Description:
 *   Low level initialize for rtc state machine waiting
 *   cycles after waking up.
 *
 * Input Parameters:
 *   slowclk_period - Re-calibrated slow clock period
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32c3_rtc_sleep_low_init(uint32_t slowclk_period);

/****************************************************************************
 * Name: esp32c3_rtc_clk_cpu_freq_get_config
 *
 * Description:
 *   Get the currently used CPU frequency configuration.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   CPU clock configuration structure
 *
 ****************************************************************************/

void esp32c3_rtc_clk_cpu_freq_get_config(
             struct esp32c3_cpu_freq_config_s *out_config);

/****************************************************************************
 * Name: esp32c3_rtc_sleep_set_wakeup_time
 *
 * Description:
 *   Set target value of RTC counter for RTC_TIMER_TRIG_EN wakeup source.
 *
 * Input Parameters:
 *   t - value of RTC counter at which wakeup from sleep will happen.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32c3_rtc_sleep_set_wakeup_time(uint64_t t);

/****************************************************************************
 * Name: esp32c3_rtc_get_time_us
 *
 * Description:
 *   Get current value of RTC counter in microseconds
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Current value of RTC counter in microseconds
 *
 ****************************************************************************/

uint64_t esp32c3_rtc_get_time_us(void);

/****************************************************************************
 * Name: esp32c3_rtc_set_boot_time
 *
 * Description:
 *   Set time to RTC register to replace the original boot time.
 *
 * Input Parameters:
 *   time_us - set time in microseconds.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp32c3_rtc_set_boot_time(uint64_t time_us);

/****************************************************************************
 * Name: esp32c3_rtc_get_boot_time
 *
 * Description:
 *   Get time of RTC register to indicate the original boot time.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   time_us - get time in microseconds.
 *
 ****************************************************************************/

uint64_t esp32c3_rtc_get_boot_time(void);

#ifdef CONFIG_RTC_DRIVER

/****************************************************************************
 * Name: up_rtc_time
 *
 * Description:
 *   Get the current time in seconds.  This is similar to the standard time()
 *   function.  This interface is only required if the low-resolution
 *   RTC/counter hardware implementation selected.  It is only used by the
 *   RTOS during initialization to set up the system time when CONFIG_RTC is
 *   set but neither CONFIG_RTC_HIRES nor CONFIG_RTC_DATETIME are set.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The current time in seconds
 *
 ****************************************************************************/

#ifndef CONFIG_RTC_HIRES
time_t up_rtc_time(void);
#endif

/****************************************************************************
 * Name: up_rtc_settime
 *
 * Description:
 *   Set the RTC to the provided time.  All RTC implementations must be
 *   able to set their time based on a standard timespec.
 *
 * Input Parameters:
 *   tp - the time to use
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int up_rtc_settime(const struct timespec *ts);

/****************************************************************************
 * Name: up_rtc_initialize
 *
 * Description:
 *   Initialize the hardware RTC per the selected configuration.
 *   This function is called once during the OS initialization sequence
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int up_rtc_initialize(void);

/****************************************************************************
 * Name: up_rtc_gettime
 *
 * Description:
 *   Get the current time from the high resolution RTC clock/counter.  This
 *   interface is only supported by the high-resolution RTC/counter hardware
 *   implementation.  It is used to replace the system timer.
 *
 * Input Parameters:
 *   tp - The location to return the high resolution time value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_HIRES
int up_rtc_gettime(struct timespec *tp);
#endif

#ifdef CONFIG_RTC_ALARM

/****************************************************************************
 * Name: up_rtc_setalarm
 *
 * Description:
 *   Set up an alarm.
 *
 * Input Parameters:
 *   alminfo - Information about the alarm configuration.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int up_rtc_setalarm(struct alm_setalarm_s *alminfo);

/****************************************************************************
 * Name: up_rtc_cancelalarm
 *
 * Description:
 *   Cancel an alaram.
 *
 * Input Parameters:
 *  alarmid - Identifies the alarm to be cancelled
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int up_rtc_cancelalarm(enum alm_id_e alarmid);

/****************************************************************************
 * Name: up_rtc_rdalarm
 *
 * Description:
 *   Query an alarm configured in hardware.
 *
 * Input Parameters:
 *  tp      - Location to return the timer match register.
 *  alarmid - Identifies the alarm to be cancelled
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int up_rtc_rdalarm(struct timespec *tp, uint32_t alarmid);

#endif /* CONFIG_RTC_ALARM */

/****************************************************************************
 * Name: up_rtc_timer_init
 *
 * Description:
 *   Init RTC timer.
 *
 * Input Parameters:
 *  None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int up_rtc_timer_init(void);

#endif /* CONFIG_RTC_DRIVER */

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_ESP32C3_ESP32C3_RTC_H */
