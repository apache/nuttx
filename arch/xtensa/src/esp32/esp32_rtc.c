/****************************************************************************
 * arch/xtensa/src/esp32/esp32_rtc.c
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

#include <stdint.h>

#include "esp32_rtc.h"
#include "hardware/esp32_rtccntl.h"
#include "hardware/esp32_dport.h"
#include "hardware/esp32_i2s.h"
#include "xtensa.h"
#include "xtensa_attr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Various delays to be programmed into power control state machines */

#define RTC_CNTL_XTL_BUF_WAIT_SLP   2
#define RTC_CNTL_CK8M_WAIT_SLP      4
#define OTHER_BLOCKS_POWERUP        1
#define OTHER_BLOCKS_WAIT           1

#define ROM_RAM_POWERUP_CYCLES   OTHER_BLOCKS_POWERUP
#define ROM_RAM_WAIT_CYCLES      OTHER_BLOCKS_WAIT

#define WIFI_POWERUP_CYCLES      OTHER_BLOCKS_POWERUP
#define WIFI_WAIT_CYCLES         OTHER_BLOCKS_WAIT

#define RTC_POWERUP_CYCLES       OTHER_BLOCKS_POWERUP
#define RTC_WAIT_CYCLES          OTHER_BLOCKS_WAIT

#define DG_WRAP_POWERUP_CYCLES   OTHER_BLOCKS_POWERUP
#define DG_WRAP_WAIT_CYCLES      OTHER_BLOCKS_WAIT

#define RTC_MEM_POWERUP_CYCLES   OTHER_BLOCKS_POWERUP
#define RTC_MEM_WAIT_CYCLES      OTHER_BLOCKS_WAIT

#define RTC_CNTL_PLL_BUF_WAIT_SLP   2

#define DELAY_FAST_CLK_SWITCH       3

#define XTAL_32K_DAC_VAL            3
#define XTAL_32K_DRES_VAL           3
#define XTAL_32K_DBIAS_VAL          0

#define DELAY_SLOW_CLK_SWITCH       300

/* Number of fractional bits in values returned by rtc_clk_cal */

#define RTC_CLK_CAL_FRACT           19

/* With the default value of CK8M_DFREQ,
 * 8M clock frequency is 8.5 MHz +/- 7%
 */

#define RTC_FAST_CLK_FREQ_APPROX    8500000

/* Number of cycles to wait from the 32k XTAL oscillator to
 * consider it running. Larger values increase startup delay.
 * Smaller values may cause false positive detection
 * (i.e. oscillator runs for a few cycles and then stops).
 */

#define SLOW_CLK_CAL_CYCLES         1024

/* Disable logging from the ROM code. */

#define RTC_DISABLE_ROM_LOG ((1 << 0) | (1 << 16))
#define EXT_OSC_FLAG    BIT(3)

/* Default initializer for esp32_rtc_sleep_config_t
 * This initializer sets all fields to "reasonable" values
 * (e.g. suggested for production use) based on a combination
 * of RTC_SLEEP_PD_x flags.
 */

#define RTC_SLEEP_CONFIG_DEFAULT(sleep_flags) { \
  .lslp_mem_inf_fpu = 0, \
  .rtc_mem_inf_fpu = 0, \
  .rtc_mem_inf_follow_cpu = ((sleep_flags) & RTC_SLEEP_PD_RTC_MEM_FOLLOW_CPU) ? 1 : 0, \
  .rtc_fastmem_pd_en = ((sleep_flags) & RTC_SLEEP_PD_RTC_FAST_MEM) ? 1 : 0, \
  .rtc_slowmem_pd_en = ((sleep_flags) & RTC_SLEEP_PD_RTC_SLOW_MEM) ? 1 : 0, \
  .rtc_peri_pd_en = ((sleep_flags) & RTC_SLEEP_PD_RTC_PERIPH) ? 1 : 0, \
  .wifi_pd_en = 0, \
  .rom_mem_pd_en = 0, \
  .deep_slp = ((sleep_flags) & RTC_SLEEP_PD_DIG) ? 1 : 0, \
  .wdt_flashboot_mod_en = 0, \
  .dig_dbias_wak = RTC_CNTL_DBIAS_1V10, \
  .dig_dbias_slp = RTC_CNTL_DBIAS_0V90, \
  .rtc_dbias_wak = RTC_CNTL_DBIAS_1V10, \
  .rtc_dbias_slp = RTC_CNTL_DBIAS_0V90, \
  .lslp_meminf_pd = 1, \
  .vddsdio_pd_en = ((sleep_flags) & RTC_SLEEP_PD_VDDSDIO) ? 1 : 0, \
  .xtal_fpu = ((sleep_flags) & RTC_SLEEP_PD_XTAL) ? 0 : 1 \
}

/* Initializer for rtc_sleep_pd_config_t which
 * sets all flags to the same value
 */

#define RTC_SLEEP_PD_CONFIG_ALL(val) {\
  .dig_pd = (val), \
  .rtc_pd = (val), \
  .cpu_pd = (val), \
  .i2s_pd = (val), \
  .bb_pd = (val), \
  .nrx_pd = (val), \
  .fe_pd = (val), \
}

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* RTC SLOW_CLK frequency values */

enum esp32_rtc_slow_freq_e
{
  RTC_SLOW_FREQ_RTC = 0,      /* Internal 150 kHz RC oscillator */
  RTC_SLOW_FREQ_32K_XTAL = 1, /* External 32 kHz XTAL */
  RTC_SLOW_FREQ_8MD256 = 2,   /* Internal 8 MHz RC oscillator, divided by 256 */
};

/* RTC FAST_CLK frequency values */

enum esp32_rtc_fast_freq_e
{
  RTC_FAST_FREQ_XTALD4 = 0,   /* Main XTAL, divided by 4 */
  RTC_FAST_FREQ_8M = 1,       /* Internal 8 MHz RC oscillator */
};

/* This is almost the same as esp32_rtc_slow_freq_e, except that we define
 * an extra enum member for the external 32k oscillator.For convenience,
 * lower 2 bits should correspond to esp32_rtc_slow_freq_e values.
 */

enum esp32_slow_clk_sel_e
{
  /* Internal 150 kHz RC oscillator */

  SLOW_CLK_150K = RTC_SLOW_FREQ_RTC,

  /* External 32 kHz XTAL */

  SLOW_CLK_32K_XTAL = RTC_SLOW_FREQ_32K_XTAL,

  /* Internal 8 MHz RC oscillator, divided by 256 */

  SLOW_CLK_8MD256 = RTC_SLOW_FREQ_8MD256,

  /* External 32k oscillator connected to 32K_XP pin */

  SLOW_CLK_32K_EXT_OSC = RTC_SLOW_FREQ_32K_XTAL | EXT_OSC_FLAG
};

/* Clock source to be calibrated using rtc_clk_cal function */

enum esp32_rtc_cal_sel_e
{
  RTC_CAL_RTC_MUX = 0,       /* Currently selected RTC SLOW_CLK */
  RTC_CAL_8MD256 = 1,        /* Internal 8 MHz RC oscillator, divided by 256 */
  RTC_CAL_32K_XTAL = 2       /* External 32 kHz XTAL */
};

/* RTC power and clock control initialization settings */

struct esp32_rtc_priv_s
{
  uint32_t ck8m_wait : 8;         /* Number of rtc_fast_clk cycles to wait for 8M clock to be ready */
  uint32_t xtal_wait : 8;         /* Number of rtc_fast_clk cycles to wait for XTAL clock to be ready */
  uint32_t pll_wait : 8;          /* Number of rtc_fast_clk cycles to wait for PLL to be ready */
  uint32_t clkctl_init : 1;       /* Perform clock control related initialization */
  uint32_t pwrctl_init : 1;       /* Perform power control related initialization */
  uint32_t rtc_dboost_fpd : 1;    /* Force power down RTC_DBOOST */
};

/* sleep configuration for rtc_sleep_init function */

struct esp32_rtc_sleep_config_s
{
  uint32_t lslp_mem_inf_fpu : 1;       /* force normal voltage in sleep mode (digital domain memory) */
  uint32_t rtc_mem_inf_fpu : 1;        /* force normal voltage in sleep mode (RTC memory) */
  uint32_t rtc_mem_inf_follow_cpu : 1; /* keep low voltage in sleep mode (even if ULP/touch is used) */
  uint32_t rtc_fastmem_pd_en : 1;      /* power down RTC fast memory */
  uint32_t rtc_slowmem_pd_en : 1;      /* power down RTC slow memory */
  uint32_t rtc_peri_pd_en : 1;         /* power down RTC peripherals */
  uint32_t wifi_pd_en : 1;             /* power down WiFi */
  uint32_t rom_mem_pd_en : 1;          /* power down main RAM and ROM */
  uint32_t deep_slp : 1;               /* power down digital domain */
  uint32_t wdt_flashboot_mod_en : 1;   /* enable WDT flashboot mode */
  uint32_t dig_dbias_wak : 3;          /* set bias for digital domain, in active mode */
  uint32_t dig_dbias_slp : 3;          /* set bias for digital domain, in sleep mode */
  uint32_t rtc_dbias_wak : 3;          /* set bias for RTC domain, in active mode */
  uint32_t rtc_dbias_slp : 3;          /* set bias for RTC domain, in sleep mode */
  uint32_t lslp_meminf_pd : 1;         /* remove all peripheral force power up flags */
  uint32_t vddsdio_pd_en : 1;          /* power down VDDSDIO regulator */
  uint32_t xtal_fpu : 1;               /* keep main XTAL powered up in sleep */
};

/* Power down flags for rtc_sleep_pd function */

struct esp32_rtc_sleep_pd_config_s
{
  uint32_t dig_pd : 1;    /* Set to 1 to power down digital part in sleep */
  uint32_t rtc_pd : 1;    /* Set to 1 to power down RTC memories in sleep */
  uint32_t cpu_pd : 1;    /* Set to 1 to power down digital memories and CPU in sleep */
  uint32_t i2s_pd : 1;    /* Set to 1 to power down I2S in sleep */
  uint32_t bb_pd : 1;     /* Set to 1 to power down WiFi in sleep */
  uint32_t nrx_pd : 1;    /* Set to 1 to power down WiFi in sleep */
  uint32_t fe_pd : 1;     /* Set to 1 to power down WiFi in sleep */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void IRAM_ATTR esp32_rtc_sleep_pd(
                      struct esp32_rtc_sleep_pd_config_s cfg);
static inline bool esp32_clk_val_is_valid(uint32_t val);
static void IRAM_ATTR esp32_rtc_clk_fast_freq_set(
                      enum esp32_rtc_fast_freq_e fast_freq);
static uint32_t IRAM_ATTR esp32_rtc_clk_cal_internal(
                enum esp32_rtc_cal_sel_e cal_clk, uint32_t slowclk_cycles);
static uint32_t IRAM_ATTR esp32_rtc_clk_cal(enum esp32_rtc_cal_sel_e cal_clk,
                                                    uint32_t slowclk_cycles);
static void IRAM_ATTR esp32_rtc_clk_slow_freq_set(
                      enum esp32_rtc_slow_freq_e slow_freq);
static void esp32_select_rtc_slow_clk(enum esp32_slow_clk_sel_e slow_clk);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct esp32_rtc_priv_s esp32_rtc_priv =
{
  .ck8m_wait      = RTC_CNTL_CK8M_WAIT_DEFAULT,
  .xtal_wait      = RTC_CNTL_XTL_BUF_WAIT_DEFAULT,
  .pll_wait       = RTC_CNTL_PLL_BUF_WAIT_DEFAULT,
  .clkctl_init    = 1,
  .pwrctl_init    = 1,
  .rtc_dboost_fpd = 1
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

extern void ets_delay_us(uint32_t us);

/****************************************************************************
 * Name: esp32_rtc_sleep_pd
 *
 * Description:
 *   Configure whether certain peripherals are powered down in deep sleep.
 *
 * Input Parameters:
 *   cfg - power down flags as rtc_sleep_pd_config_t structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void IRAM_ATTR esp32_rtc_sleep_pd(
                      struct esp32_rtc_sleep_pd_config_s cfg)
{
  REG_SET_FIELD(RTC_CNTL_DIG_PWC_REG,
                RTC_CNTL_LSLP_MEM_FORCE_PU, ~cfg.dig_pd);
  REG_SET_FIELD(RTC_CNTL_PWC_REG, RTC_CNTL_SLOWMEM_FORCE_LPU, ~cfg.rtc_pd);
  REG_SET_FIELD(RTC_CNTL_PWC_REG, RTC_CNTL_FASTMEM_FORCE_LPU, ~cfg.rtc_pd);
  REG_SET_FIELD(DPORT_MEM_PD_MASK_REG, DPORT_LSLP_MEM_PD_MASK, ~cfg.cpu_pd);
  REG_SET_FIELD(I2S_PD_CONF_REG(0), I2S_PLC_MEM_FORCE_PU, ~cfg.i2s_pd);
  REG_SET_FIELD(I2S_PD_CONF_REG(0), I2S_FIFO_FORCE_PU, ~cfg.i2s_pd);
  REG_SET_FIELD(BBPD_CTRL, BB_FFT_FORCE_PU, ~cfg.bb_pd);
  REG_SET_FIELD(BBPD_CTRL, BB_DC_EST_FORCE_PU, ~cfg.bb_pd);
  REG_SET_FIELD(NRXPD_CTRL, NRX_RX_ROT_FORCE_PU, ~cfg.nrx_pd);
  REG_SET_FIELD(NRXPD_CTRL, NRX_VIT_FORCE_PU, ~cfg.nrx_pd);
  REG_SET_FIELD(NRXPD_CTRL, NRX_DEMAP_FORCE_PU, ~cfg.nrx_pd);
  REG_SET_FIELD(FE_GEN_CTRL, FE_IQ_EST_FORCE_PU, ~cfg.fe_pd);
  REG_SET_FIELD(FE2_TX_INTERP_CTRL, FE2_TX_INF_FORCE_PU, ~cfg.fe_pd);
}

/****************************************************************************
 * Name: esp32_rtc_clk_fast_freq_set
 *
 * Description:
 *   Select source for RTC_FAST_CLK.
 *
 * Input Parameters:
 *   cfg - Clock source (one of enum esp32_rtc_fast_freq_e values)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void IRAM_ATTR esp32_rtc_clk_fast_freq_set(
                      enum esp32_rtc_fast_freq_e fast_freq)
{
  REG_SET_FIELD(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_FAST_CLK_RTC_SEL, fast_freq);
  ets_delay_us(DELAY_FAST_CLK_SWITCH);
}

/****************************************************************************
 * Name: esp32_clk_val_is_valid
 *
 * Description:
 *   Values of RTC_XTAL_FREQ_REG and RTC_APB_FREQ_REG are
 *   stored as two copies in lower and upper 16-bit halves.
 *   These are the routines to work with such a representation.
 *
 * Input Parameters:
 *   val   - register value
 *
 * Returned Value:
 *   true:  Valid register value.
 *   false: Invalid register value.
 *
 ****************************************************************************/

static inline bool esp32_clk_val_is_valid(uint32_t val)
{
  return (val & 0xffff) == ((val >> 16) & 0xffff)
                        && val != 0 && val != UINT32_MAX;
}

/****************************************************************************
 * Name: esp32_rtc_clk_cal_internal
 *
 * Description:
 *   Clock calibration function used by rtc_clk_cal and rtc_clk_cal_ratio
 *
 * Input Parameters:
 *   cal_clk        - which clock to calibrate
 *   slowclk_cycles - number of slow clock cycles to count.
 *
 * Returned Value:
 *   Number of XTAL clock cycles within the given number of slow clock cycles
 *
 ****************************************************************************/

static uint32_t IRAM_ATTR esp32_rtc_clk_cal_internal(
                enum esp32_rtc_cal_sel_e cal_clk, uint32_t slowclk_cycles)
{
  uint32_t expected_freq;
  uint32_t us_time_estimate;
  uint32_t us_timer_max;
  int timeout_us;
  enum esp32_rtc_slow_freq_e slow_freq;
  enum esp32_rtc_xtal_freq_e xtal_freq;

  /* Enable requested clock (150k clock is always on) */

  int dig_32k_xtal_state = REG_GET_FIELD(RTC_CNTL_CLK_CONF_REG,
                                         RTC_CNTL_DIG_XTAL32K_EN);

  if (cal_clk == RTC_CAL_32K_XTAL && !dig_32k_xtal_state)
    {
      REG_SET_FIELD(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_DIG_XTAL32K_EN, 1);
    }

  if (cal_clk == RTC_CAL_8MD256)
    {
      modifyreg32(RTC_CNTL_CLK_CONF_REG, 0, RTC_CNTL_DIG_CLK8M_D256_EN);
    }

  /* Prepare calibration */

  REG_SET_FIELD(TIMG_RTCCALICFG_REG(0), TIMG_RTC_CALI_CLK_SEL, cal_clk);
  modifyreg32(TIMG_RTCCALICFG_REG(0), TIMG_RTC_CALI_START_CYCLING, 0);
  REG_SET_FIELD(TIMG_RTCCALICFG_REG(0), TIMG_RTC_CALI_MAX, slowclk_cycles);

  /* Figure out how long to wait for calibration to finish */

  slow_freq = REG_GET_FIELD(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_ANA_CLK_RTC_SEL);

  if (cal_clk == RTC_CAL_32K_XTAL ||
        (cal_clk == RTC_CAL_RTC_MUX && slow_freq == RTC_SLOW_FREQ_32K_XTAL))
    {
      expected_freq = 32768; /* standard 32k XTAL */
    }
  else if (cal_clk == RTC_CAL_8MD256 ||
          (cal_clk == RTC_CAL_RTC_MUX && slow_freq == RTC_SLOW_FREQ_8MD256))
    {
      expected_freq = RTC_FAST_CLK_FREQ_APPROX / 256;
    }
  else
    {
      expected_freq = 150000; /* 150k internal oscillator */
    }

  us_time_estimate = (uint32_t) (((uint64_t) slowclk_cycles) *
                                           MHZ / expected_freq);

  /* Check if the required number of slowclk_cycles
   * may result in an overflow of TIMG_RTC_CALI_VALUE.
   */

  xtal_freq = esp32_rtc_clk_xtal_freq_get();
  if (xtal_freq == RTC_XTAL_FREQ_AUTO)
    {
      /* XTAL frequency is not known yet; assume worst case (40 MHz) */

      xtal_freq = RTC_XTAL_FREQ_40M;
    }

  us_timer_max =  TIMG_RTC_CALI_VALUE / (uint32_t) xtal_freq;

  if (us_time_estimate >= us_timer_max)
    {
      return 0;
    }

  /* Start calibration */

  modifyreg32(TIMG_RTCCALICFG_REG(0), TIMG_RTC_CALI_START, 0);
  modifyreg32(TIMG_RTCCALICFG_REG(0), 0, TIMG_RTC_CALI_START);

  /* Wait the expected time calibration should take.
   * TODO: if running under RTOS, and us_time_estimate > RTOS tick, use the
   * RTOS delay function.
   */

  ets_delay_us(us_time_estimate);

  /* Wait for calibration to finish up to another us_time_estimate */

  timeout_us = us_time_estimate;
  while (!(getreg32(TIMG_RTCCALICFG_REG(0)) &
           TIMG_RTC_CALI_RDY) && (timeout_us > 0))
    {
      timeout_us--;
      ets_delay_us(1);
    }

  REG_SET_FIELD(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_DIG_XTAL32K_EN,
                                          dig_32k_xtal_state);

  if (cal_clk == RTC_CAL_8MD256)
    {
      modifyreg32(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_DIG_CLK8M_D256_EN, 0);
    }

  if (timeout_us == 0)
    {
      /* timed out waiting for calibration */

      return 0;
    }

  return REG_GET_FIELD(TIMG_RTCCALICFG1_REG(0), TIMG_RTC_CALI_VALUE);
}

/****************************************************************************
 * Name: esp32_rtc_clk_cal
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

static uint32_t IRAM_ATTR esp32_rtc_clk_cal(enum esp32_rtc_cal_sel_e cal_clk,
                                                     uint32_t slowclk_cycles)
{
  enum esp32_rtc_xtal_freq_e xtal_freq;
  uint64_t xtal_cycles;
  uint64_t divider;
  uint64_t period_64;
  uint32_t period;

  xtal_freq = esp32_rtc_clk_xtal_freq_get();
  xtal_cycles = esp32_rtc_clk_cal_internal(cal_clk, slowclk_cycles);
  divider = ((uint64_t)xtal_freq) * slowclk_cycles;
  period_64 = ((xtal_cycles << RTC_CLK_CAL_FRACT) + divider / 2 - 1)
                                                          / divider;
  period = (uint32_t)(period_64 & UINT32_MAX);

  return period;
}

/****************************************************************************
 * Name: esp32_rtc_clk_slow_freq_set
 *
 * Description:
 *   Select source for RTC_SLOW_CLK
 *
 * Input Parameters:
 *   slow_freq - Select source for RTC_SLOW_CLK
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void IRAM_ATTR esp32_rtc_clk_slow_freq_set(
                      enum esp32_rtc_slow_freq_e slow_freq)
{
  REG_SET_FIELD(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_ANA_CLK_RTC_SEL, slow_freq);

  REG_SET_FIELD(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_DIG_XTAL32K_EN,
               (slow_freq == RTC_SLOW_FREQ_32K_XTAL) ? 1 : 0);

  ets_delay_us(DELAY_SLOW_CLK_SWITCH);
}

/****************************************************************************
 * Name: esp32_select_rtc_slow_clk
 *
 * Description:
 *   Selects an clock source for RTC.
 *
 * Input Parameters:
 *   slow_clk - RTC SLOW_CLK frequency values
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32_select_rtc_slow_clk(enum esp32_slow_clk_sel_e slow_clk)
{
  uint32_t cal_val = 0;
  enum esp32_rtc_slow_freq_e rtc_slow_freq = slow_clk &
                                             RTC_CNTL_ANA_CLK_RTC_SEL_V;

  do
    {
      esp32_rtc_clk_slow_freq_set(rtc_slow_freq);

      /* TODO: 32k XTAL oscillator has some frequency drift at startup.
       * Improve calibration routine to wait until the frequency is stable.
       */

      cal_val = esp32_rtc_clk_cal(RTC_CAL_RTC_MUX, SLOW_CLK_CAL_CYCLES);
    }
  while (cal_val == 0);

  putreg32((uint32_t)cal_val, RTC_SLOW_CLK_CAL_REG);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

enum esp32_rtc_xtal_freq_e rtc_get_xtal(void)
                __attribute__((alias("esp32_rtc_clk_xtal_freq_get")));

/****************************************************************************
 * Name: esp32_rtc_clk_xtal_freq_get
 *
 * Description:
 *   Get main XTAL frequency
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   XTAL frequency (one of enum esp32_rtc_xtal_freq_e values)
 *
 ****************************************************************************/

enum esp32_rtc_xtal_freq_e IRAM_ATTR esp32_rtc_clk_xtal_freq_get(void)
{
  /* We may have already written XTAL value into RTC_XTAL_FREQ_REG */

  uint32_t xtal_freq_reg = getreg32(RTC_XTAL_FREQ_REG);

  if (!esp32_clk_val_is_valid(xtal_freq_reg))
    {
      return RTC_XTAL_FREQ_AUTO;
    }

  return (xtal_freq_reg & ~RTC_DISABLE_ROM_LOG) & UINT16_MAX;
}

/****************************************************************************
 * Name: esp32_rtc_update_to_xtal
 *
 * Description:
 *   Switch to XTAL frequency, does not disable the PLL
 *
 * Input Parameters:
 *   freq -  XTAL frequency
 *   div  -  REF_TICK divider
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

void IRAM_ATTR esp32_rtc_update_to_xtal(int freq, int div)
{
  uint32_t value = (((freq * MHZ) >> 12) & UINT16_MAX)
                   | ((((freq * MHZ) >> 12) & UINT16_MAX) << 16);
  putreg32(value, RTC_APB_FREQ_REG);

  /* set divider from XTAL to APB clock */

  REG_SET_FIELD(APB_CTRL_SYSCLK_CONF_REG, APB_CTRL_PRE_DIV_CNT, div - 1);

  /* switch clock source */

  REG_SET_FIELD(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_SOC_CLK_SEL,
                RTC_CNTL_SOC_CLK_SEL_XTL);

  /* adjust ref_tick */

  modifyreg32(APB_CTRL_XTAL_TICK_CONF_REG, 0,
             (freq * MHZ) / REF_CLK_FREQ - 1);

  /* lower the voltage */

  if (freq <= 2)
    {
      REG_SET_FIELD(RTC_CNTL_REG, RTC_CNTL_DIG_DBIAS_WAK, DIG_DBIAS_2M);
    }
  else
    {
      REG_SET_FIELD(RTC_CNTL_REG, RTC_CNTL_DIG_DBIAS_WAK, DIG_DBIAS_XTAL);
    }
}

/****************************************************************************
 * Name: esp32_rtc_bbpll_enable
 *
 * Description:
 *   Reset BBPLL configuration.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void IRAM_ATTR esp32_rtc_bbpll_enable(void)
{
  modifyreg32(RTC_CNTL_OPTIONS0_REG,
              RTC_CNTL_BIAS_I2C_FORCE_PD | RTC_CNTL_BB_I2C_FORCE_PD |
              RTC_CNTL_BBPLL_FORCE_PD | RTC_CNTL_BBPLL_I2C_FORCE_PD, 0);

  /* reset BBPLL configuration */

  I2C_WRITEREG_RTC(I2C_BBPLL, I2C_BBPLL_IR_CAL_DELAY,
                   BBPLL_IR_CAL_DELAY_VAL);
  I2C_WRITEREG_RTC(I2C_BBPLL, I2C_BBPLL_IR_CAL_EXT_CAP,
                   BBPLL_IR_CAL_EXT_CAP_VAL);
  I2C_WRITEREG_RTC(I2C_BBPLL, I2C_BBPLL_OC_ENB_FCAL,
                   BBPLL_OC_ENB_FCAL_VAL);
  I2C_WRITEREG_RTC(I2C_BBPLL, I2C_BBPLL_OC_ENB_VCON,
                   BBPLL_OC_ENB_VCON_VAL);
  I2C_WRITEREG_RTC(I2C_BBPLL, I2C_BBPLL_BBADC_CAL_7_0,
                   BBPLL_BBADC_CAL_7_0_VAL);
}

/****************************************************************************
 * Name: esp32_rtc_bbpll_configure
 *
 * Description:
 *   Configure main XTAL frequency values according to pll_freq.
 *
 * Input Parameters:
 *   xtal_freq -    XTAL frequency values
 *   pll_freq  -    PLL frequency values
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void IRAM_ATTR esp32_rtc_bbpll_configure(
                     enum esp32_rtc_xtal_freq_e xtal_freq, int pll_freq)
{
  static uint8_t div_ref = 0;
  static uint8_t div7_0 = 0;
  static uint8_t div10_8 = 0;
  static uint8_t lref = 0 ;
  static uint8_t dcur = 0;
  static uint8_t bw = 0;
  uint8_t i2c_bbpll_lref  = 0;
  uint8_t i2c_bbpll_div_7_0 = 0;
  uint8_t i2c_bbpll_dcur = 0;

  if (pll_freq == RTC_PLL_FREQ_320M)
    {
      /* Raise the voltage, if needed */

      REG_SET_FIELD(RTC_CNTL_REG, RTC_CNTL_DIG_DBIAS_WAK,
                    DIG_DBIAS_80M_160M);

      /* Configure 320M PLL */

      switch (xtal_freq)
        {
          case RTC_XTAL_FREQ_40M:
            {
              div_ref = 0;
              div7_0 = 32;
              div10_8 = 0;
              lref = 0;
              dcur = 6;
              bw = 3;
            }
            break;

          case RTC_XTAL_FREQ_26M:
            {
              div_ref = 12;
              div7_0 = 224;
              div10_8 = 4;
              lref = 1;
              dcur = 0;
              bw = 1;
            }
            break;

          case RTC_XTAL_FREQ_24M:
            {
              div_ref = 11;
              div7_0 = 224;
              div10_8 = 4;
              lref = 1;
              dcur = 0;
              bw = 1;
            }
            break;

          default:
            {
              div_ref = 12;
              div7_0 = 224;
              div10_8 = 4;
              lref = 0;
              dcur = 0;
              bw = 0;
            }
            break;
        }

      I2C_WRITEREG_RTC(I2C_BBPLL, I2C_BBPLL_ENDIV5, BBPLL_ENDIV5_VAL_320M);
      I2C_WRITEREG_RTC(I2C_BBPLL, I2C_BBPLL_BBADC_DSMP,
                       BBPLL_BBADC_DSMP_VAL_320M);
    }
  else
    {
      /* Raise the voltage */

      REG_SET_FIELD(RTC_CNTL_REG, RTC_CNTL_DIG_DBIAS_WAK, DIG_DBIAS_240M);
      ets_delay_us(DELAY_PLL_DBIAS_RAISE);

      /* Configure 480M PLL */

      switch (xtal_freq)
        {
          case RTC_XTAL_FREQ_40M:
            {
              div_ref = 0;
              div7_0 = 28;
              div10_8 = 0;
              lref = 0;
              dcur = 6;
              bw = 3;
            }
            break;

          case RTC_XTAL_FREQ_26M:
            {
              div_ref = 12;
              div7_0 = 144;
              div10_8 = 4;
              lref = 1;
              dcur = 0;
              bw = 1;
            }
            break;

          case RTC_XTAL_FREQ_24M:
            {
              div_ref = 11;
              div7_0 = 144;
              div10_8 = 4;
              lref = 1;
              dcur = 0;
              bw = 1;
            }
            break;

          default:
            {
              div_ref = 12;
              div7_0 = 224;
              div10_8 = 4;
              lref = 0;
              dcur = 0;
              bw = 0;
            }
            break;
        }

      I2C_WRITEREG_RTC(I2C_BBPLL, I2C_BBPLL_ENDIV5, BBPLL_ENDIV5_VAL_480M);
      I2C_WRITEREG_RTC(I2C_BBPLL,
                       I2C_BBPLL_BBADC_DSMP, BBPLL_BBADC_DSMP_VAL_480M);
    }

  i2c_bbpll_lref  = (lref << 7) | (div10_8 << 4) | (div_ref);
  i2c_bbpll_div_7_0 = div7_0;
  i2c_bbpll_dcur = (bw << 6) | dcur;
  I2C_WRITEREG_RTC(I2C_BBPLL, I2C_BBPLL_OC_LREF, i2c_bbpll_lref);
  I2C_WRITEREG_RTC(I2C_BBPLL, I2C_BBPLL_OC_DIV_7_0, i2c_bbpll_div_7_0);
  I2C_WRITEREG_RTC(I2C_BBPLL, I2C_BBPLL_OC_DCUR, i2c_bbpll_dcur);
}

/****************************************************************************
 * Name: esp32_rtc_clk_set
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

void esp32_rtc_clk_set()
{
  enum esp32_rtc_fast_freq_e fast_freq = RTC_FAST_FREQ_8M;
  enum esp32_slow_clk_sel_e slow_clk = RTC_SLOW_FREQ_RTC;
  esp32_rtc_clk_fast_freq_set(fast_freq);
  esp32_select_rtc_slow_clk(slow_clk);
}

/****************************************************************************
 * Name: esp32_rtc_init
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

void IRAM_ATTR esp32_rtc_init()
{
  struct esp32_rtc_priv_s *priv = &esp32_rtc_priv;

  modifyreg32(RTC_CNTL_ANA_CONF_REG, RTC_CNTL_PVTMON_PU |
              RTC_CNTL_TXRF_I2C_PU | RTC_CNTL_RFRX_PBUS_PU |
              RTC_CNTL_CKGEN_I2C_PU | RTC_CNTL_PLL_I2C_PU, 0);

  REG_SET_FIELD(RTC_CNTL_TIMER1_REG, RTC_CNTL_PLL_BUF_WAIT, priv->pll_wait);
  REG_SET_FIELD(RTC_CNTL_TIMER1_REG, RTC_CNTL_XTL_BUF_WAIT, priv->xtal_wait);
  REG_SET_FIELD(RTC_CNTL_TIMER1_REG, RTC_CNTL_CK8M_WAIT, priv->ck8m_wait);
  REG_SET_FIELD(RTC_CNTL_BIAS_CONF_REG, RTC_CNTL_DBG_ATTEN,
                RTC_CNTL_DBG_ATTEN_DEFAULT);

  modifyreg32(RTC_CNTL_BIAS_CONF_REG, 0,
              RTC_CNTL_DEC_HEARTBEAT_WIDTH | RTC_CNTL_INC_HEARTBEAT_PERIOD);

  /* Reset RTC bias to default value (needed if waking up from deep sleep) */

  REG_SET_FIELD(RTC_CNTL_REG, RTC_CNTL_DBIAS_WAK, RTC_CNTL_DBIAS_1V10);
  REG_SET_FIELD(RTC_CNTL_REG, RTC_CNTL_DBIAS_SLP, RTC_CNTL_DBIAS_1V10);
  if (priv->clkctl_init)
    {
      /* clear CMMU clock force on */

      modifyreg32(DPORT_PRO_CACHE_CTRL1_REG, DPORT_PRO_CMMU_FORCE_ON, 0);
      modifyreg32(DPORT_APP_CACHE_CTRL1_REG, DPORT_APP_CMMU_FORCE_ON, 0);

      /* clear rom clock force on */

      modifyreg32(DPORT_ROM_FO_CTRL_REG,
                 (DPORT_SHARE_ROM_FO << DPORT_SHARE_ROM_FO_S), 0);
      modifyreg32(DPORT_ROM_FO_CTRL_REG, DPORT_APP_ROM_FO |
                  DPORT_PRO_ROM_FO, 0);

      /* clear sram clock force on */

      modifyreg32(DPORT_SRAM_FO_CTRL_0_REG, DPORT_SRAM_FO_0, 0);
      modifyreg32(DPORT_SRAM_FO_CTRL_1_REG, DPORT_SRAM_FO_1, 0);

      /* clear tag clock force on */

      modifyreg32(DPORT_TAG_FO_CTRL_REG, DPORT_APP_CACHE_TAG_FORCE_ON |
                  DPORT_PRO_CACHE_TAG_FORCE_ON, 0);
    }

  if (priv->pwrctl_init)
    {
      modifyreg32(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_CK8M_FORCE_PU, 0);

      /* cancel xtal force pu */

      modifyreg32(RTC_CNTL_OPTIONS0_REG, RTC_CNTL_XTL_FORCE_PU, 0);

      /* cancel BIAS force pu */

      modifyreg32(RTC_CNTL_OPTIONS0_REG, RTC_CNTL_BIAS_CORE_FORCE_PU |
                RTC_CNTL_BIAS_I2C_FORCE_PU | RTC_CNTL_BIAS_FORCE_NOSLEEP, 0);

      /* bias follow 8M */

      modifyreg32(RTC_CNTL_OPTIONS0_REG, 0, RTC_CNTL_BIAS_CORE_FOLW_8M |
                    RTC_CNTL_BIAS_I2C_FOLW_8M | RTC_CNTL_BIAS_SLEEP_FOLW_8M);

      /* CLEAR APLL close */

      modifyreg32(RTC_CNTL_ANA_CONF_REG, RTC_CNTL_PLLA_FORCE_PU,
                  RTC_CNTL_PLLA_FORCE_PD);
      modifyreg32(RTC_CNTL_OPTIONS0_REG, RTC_CNTL_BBPLL_FORCE_PU |
                  RTC_CNTL_BBPLL_I2C_FORCE_PU, 0);

      /* cancel RTC REG force PU */

      modifyreg32(RTC_CNTL_REG, RTC_CNTL_FORCE_PU |
                  RTC_CNTL_DBOOST_FORCE_PU, 0);
      if (priv->rtc_dboost_fpd)
        {
          modifyreg32(RTC_CNTL_REG, 0, RTC_CNTL_DBOOST_FORCE_PD);
        }
      else
        {
          modifyreg32(RTC_CNTL_REG, RTC_CNTL_DBOOST_FORCE_PD, 0);
        }

      /* cancel digital pu force */

      modifyreg32(RTC_CNTL_DIG_PWC_REG, RTC_CNTL_LSLP_MEM_FORCE_PU |
                  RTC_CNTL_DG_WRAP_FORCE_PU | RTC_CNTL_WIFI_FORCE_PU |
                  RTC_CNTL_CPU_ROM_RAM_FORCE_PU , 0);
      modifyreg32(RTC_CNTL_PWC_REG, RTC_CNTL_MEM_FORCE_PU |
                  RTC_CNTL_PWC_FORCE_PU, 0);
      modifyreg32(RTC_CNTL_DIG_ISO_REG, RTC_CNTL_DG_WRAP_FORCE_NOISO |
          RTC_CNTL_WIFI_FORCE_NOISO | RTC_CNTL_CPU_ROM_RAM_FORCE_NOISO, 0);
      modifyreg32(RTC_CNTL_PWC_REG, RTC_CNTL_MEM_FORCE_NOISO |
                  RTC_CNTL_FORCE_NOISO, 0);

      /* cancel digital PADS force no iso */

      modifyreg32(RTC_CNTL_DIG_ISO_REG, RTC_CNTL_DG_PAD_FORCE_UNHOLD |
                  RTC_CNTL_DG_PAD_FORCE_NOISO, 0);
    }
}

/****************************************************************************
 * Name: esp32_rtc_time_get
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

uint64_t IRAM_ATTR esp32_rtc_time_get(void)
{
  uint64_t rtc_time;

  modifyreg32(RTC_CNTL_TIME_UPDATE_REG, 0, RTC_CNTL_TIME_UPDATE);

  /* might take 1 RTC slowclk period, don't flood RTC bus */

  while ((getreg32(RTC_CNTL_TIME_UPDATE_REG) & RTC_CNTL_TIME_VALID) == 0)
    {
      ets_delay_us(1);
    }

  modifyreg32(RTC_CNTL_INT_CLR_REG, 0, RTC_CNTL_TIME_VALID_INT_CLR);
  rtc_time = getreg32(RTC_CNTL_TIME0_REG);
  rtc_time |= ((uint64_t) getreg32(RTC_CNTL_TIME1_REG)) << 32;

  return rtc_time;
}

/****************************************************************************
 * Name: esp32_rtc_time_us_to_slowclk
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

uint64_t IRAM_ATTR esp32_rtc_time_us_to_slowclk(uint64_t time_in_us,
                                                uint32_t period)
{
  /* Overflow will happen in this function if time_in_us >= 2^45,
   * which is about 400 days. TODO: fix overflow.
   */

  return (time_in_us << RTC_CLK_CAL_FRACT) / period;
}

/****************************************************************************
 * Name: esp32_rtc_bbpll_disable
 *
 * Description:
 *   disable BBPLL.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void IRAM_ATTR esp32_rtc_bbpll_disable(void)
{
  uint32_t apll_fpd;
  modifyreg32(RTC_CNTL_OPTIONS0_REG, 0, RTC_CNTL_BB_I2C_FORCE_PD |
              RTC_CNTL_BBPLL_FORCE_PD | RTC_CNTL_BBPLL_I2C_FORCE_PD);

  /* is APLL under force power down? */

  apll_fpd = REG_GET_FIELD(RTC_CNTL_ANA_CONF_REG, RTC_CNTL_PLLA_FORCE_PD);

  if (apll_fpd)
    {
      /* then also power down the internal I2C bus */

      modifyreg32(RTC_CNTL_OPTIONS0_REG, 0, RTC_CNTL_BIAS_I2C_FORCE_PD);
    }
}

/****************************************************************************
 * Name: esp32_rtc_sleep_set_wakeup_time
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

void IRAM_ATTR esp32_rtc_sleep_set_wakeup_time(uint64_t t)
{
  putreg32(t & UINT32_MAX, RTC_CNTL_SLP_TIMER0_REG);
  putreg32((uint32_t)(t >> 32), RTC_CNTL_SLP_TIMER1_REG);
}

/****************************************************************************
 * Name: esp32_rtc_wait_for_slow_cycle
 *
 * Description:
 *   Busy loop until next RTC_SLOW_CLK cycle.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

void IRAM_ATTR esp32_rtc_wait_for_slow_cycle(void)
{
  modifyreg32(TIMG_RTCCALICFG_REG(0), TIMG_RTC_CALI_START_CYCLING |
              TIMG_RTC_CALI_START, 0);
  modifyreg32(TIMG_RTCCALICFG_REG(0), TIMG_RTC_CALI_RDY, 0);
  REG_SET_FIELD(TIMG_RTCCALICFG_REG(0), TIMG_RTC_CALI_CLK_SEL,
                RTC_CAL_RTC_MUX);

  /* Request to run calibration for 0 slow clock cycles.
   * RDY bit will be set on the nearest slow clock cycle.
   */

  REG_SET_FIELD(TIMG_RTCCALICFG_REG(0), TIMG_RTC_CALI_MAX, 0);
  modifyreg32(TIMG_RTCCALICFG_REG(0), 0, TIMG_RTC_CALI_START);

  /* RDY needs some time to go low */

  ets_delay_us(1);

  while (!(getreg32(TIMG_RTCCALICFG_REG(0)) & TIMG_RTC_CALI_RDY))
    {
      ets_delay_us(1);
    }
}

/****************************************************************************
 * Name: esp32_rtc_cpu_freq_set_xtal
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

void IRAM_ATTR esp32_rtc_cpu_freq_set_xtal(void)
{
  int freq_mhz = (int) esp32_rtc_clk_xtal_freq_get();
  esp32_rtc_update_to_xtal(freq_mhz, 1);
  esp32_rtc_wait_for_slow_cycle();
  esp32_rtc_bbpll_disable();
}

/****************************************************************************
 * Name: esp32_rtc_sleep_init
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

void IRAM_ATTR esp32_rtc_sleep_init(uint32_t flags)
{
  struct esp32_rtc_sleep_config_s cfg = RTC_SLEEP_CONFIG_DEFAULT(flags);

  struct esp32_rtc_sleep_pd_config_s pd_cfg =
                           RTC_SLEEP_PD_CONFIG_ALL(cfg.lslp_meminf_pd);

  /* set 5 PWC state machine times to fit in main state machine time */

  REG_SET_FIELD(RTC_CNTL_TIMER1_REG, RTC_CNTL_PLL_BUF_WAIT,
                RTC_CNTL_PLL_BUF_WAIT_SLP);
  REG_SET_FIELD(RTC_CNTL_TIMER1_REG, RTC_CNTL_XTL_BUF_WAIT,
                RTC_CNTL_XTL_BUF_WAIT_SLP);
  REG_SET_FIELD(RTC_CNTL_TIMER1_REG, RTC_CNTL_CK8M_WAIT,
                RTC_CNTL_CK8M_WAIT_SLP);

  /* set shortest possible sleep time limit */

  REG_SET_FIELD(RTC_CNTL_TIMER5_REG, RTC_CNTL_MIN_SLP_VAL,
                RTC_CNTL_MIN_SLP_VAL_MIN);

  /* set rom&ram timer */

  REG_SET_FIELD(RTC_CNTL_TIMER3_REG, RTC_CNTL_ROM_RAM_POWERUP_TIMER,
                ROM_RAM_POWERUP_CYCLES);
  REG_SET_FIELD(RTC_CNTL_TIMER3_REG, RTC_CNTL_ROM_RAM_WAIT_TIMER,
                ROM_RAM_WAIT_CYCLES);

  /* set wifi timer */

  REG_SET_FIELD(RTC_CNTL_TIMER3_REG, RTC_CNTL_WIFI_POWERUP_TIMER,
                WIFI_POWERUP_CYCLES);
  REG_SET_FIELD(RTC_CNTL_TIMER3_REG, RTC_CNTL_WIFI_WAIT_TIMER,
                WIFI_WAIT_CYCLES);

  /* set rtc peri timer */

  REG_SET_FIELD(RTC_CNTL_TIMER4_REG, RTC_CNTL_POWERUP_TIMER,
                RTC_POWERUP_CYCLES);
  REG_SET_FIELD(RTC_CNTL_TIMER4_REG, RTC_CNTL_WAIT_TIMER,
                RTC_WAIT_CYCLES);

  /* set digital wrap timer */

  REG_SET_FIELD(RTC_CNTL_TIMER4_REG, RTC_CNTL_DG_WRAP_POWERUP_TIMER,
                DG_WRAP_POWERUP_CYCLES);
  REG_SET_FIELD(RTC_CNTL_TIMER4_REG, RTC_CNTL_DG_WRAP_WAIT_TIMER,
                DG_WRAP_WAIT_CYCLES);

  /* set rtc memory timer */

  REG_SET_FIELD(RTC_CNTL_TIMER5_REG, RTC_CNTL_RTCMEM_POWERUP_TIMER,
                RTC_MEM_POWERUP_CYCLES);
  REG_SET_FIELD(RTC_CNTL_TIMER5_REG, RTC_CNTL_RTCMEM_WAIT_TIMER,
                RTC_MEM_WAIT_CYCLES);

  REG_SET_FIELD(RTC_CNTL_DIG_PWC_REG, RTC_CNTL_LSLP_MEM_FORCE_PU,
                cfg.lslp_mem_inf_fpu);
  esp32_rtc_sleep_pd(pd_cfg);

  if (cfg.rtc_mem_inf_fpu)
    {
      modifyreg32(RTC_CNTL_PWC_REG, 0, RTC_CNTL_MEM_FORCE_PU);
    }
  else
    {
      modifyreg32(RTC_CNTL_PWC_REG, RTC_CNTL_MEM_FORCE_PU, 0);
    }

  if (cfg.rtc_mem_inf_follow_cpu)
    {
      modifyreg32(RTC_CNTL_PWC_REG, 0, RTC_CNTL_MEM_FOLW_CPU);
    }
  else
    {
      modifyreg32(RTC_CNTL_PWC_REG, RTC_CNTL_MEM_FOLW_CPU, 0);
    }

  if (cfg.rtc_fastmem_pd_en)
    {
      modifyreg32(RTC_CNTL_PWC_REG, RTC_CNTL_FASTMEM_FORCE_PU |
                  RTC_CNTL_FASTMEM_FORCE_NOISO, RTC_CNTL_FASTMEM_PD_EN);
    }
  else
    {
      modifyreg32(RTC_CNTL_PWC_REG, RTC_CNTL_FASTMEM_PD_EN,
                  RTC_CNTL_FASTMEM_FORCE_PU | RTC_CNTL_FASTMEM_FORCE_NOISO);
    }

  if (cfg.rtc_slowmem_pd_en)
    {
      modifyreg32(RTC_CNTL_PWC_REG, RTC_CNTL_SLOWMEM_FORCE_PU |
                  RTC_CNTL_SLOWMEM_FORCE_NOISO, RTC_CNTL_SLOWMEM_PD_EN);
    }
  else
    {
      modifyreg32(RTC_CNTL_PWC_REG, RTC_CNTL_SLOWMEM_PD_EN,
                  RTC_CNTL_SLOWMEM_FORCE_PU | RTC_CNTL_SLOWMEM_FORCE_NOISO);
    }

  if (cfg.rtc_peri_pd_en)
    {
      modifyreg32(RTC_CNTL_PWC_REG, 0, RTC_CNTL_PD_EN);
    }
  else
    {
      modifyreg32(RTC_CNTL_PWC_REG, RTC_CNTL_PD_EN, 0);
    }

  if (cfg.wifi_pd_en)
    {
      modifyreg32(RTC_CNTL_DIG_PWC_REG, 0, RTC_CNTL_WIFI_PD_EN);
    }
  else
    {
      modifyreg32(RTC_CNTL_DIG_PWC_REG, RTC_CNTL_WIFI_PD_EN, 0);
    }

  if (cfg.rom_mem_pd_en)
    {
      modifyreg32(RTC_CNTL_DIG_PWC_REG, 0, RTC_CNTL_CPU_ROM_RAM_PD_EN);
    }
  else
    {
      modifyreg32(RTC_CNTL_DIG_PWC_REG, RTC_CNTL_CPU_ROM_RAM_PD_EN, 0);
    }

  if (cfg.deep_slp)
    {
      modifyreg32(RTC_CNTL_DIG_ISO_REG, RTC_CNTL_DG_PAD_FORCE_ISO |
                  RTC_CNTL_DG_PAD_FORCE_NOISO, 0);
      modifyreg32(RTC_CNTL_DIG_PWC_REG, RTC_CNTL_DG_WRAP_FORCE_PU |
                  RTC_CNTL_DG_WRAP_FORCE_PD, RTC_CNTL_DG_WRAP_PD_EN);
      modifyreg32(RTC_CNTL_OPTIONS0_REG, RTC_CNTL_BIAS_FORCE_NOSLEEP, 0);

      /* Shut down parts of RTC which may have been left
       * enabled by the wireless drivers.
       */

      modifyreg32(RTC_CNTL_ANA_CONF_REG, RTC_CNTL_CKGEN_I2C_PU |
                  RTC_CNTL_PLL_I2C_PU | RTC_CNTL_RFRX_PBUS_PU |
                  RTC_CNTL_TXRF_I2C_PU, 0);
    }
  else
    {
      modifyreg32(RTC_CNTL_DIG_PWC_REG, RTC_CNTL_DG_WRAP_PD_EN, 0);
      REG_SET_FIELD(RTC_CNTL_BIAS_CONF_REG, RTC_CNTL_DBG_ATTEN, 0);
    }

  REG_SET_FIELD(RTC_CNTL_OPTIONS0_REG, RTC_CNTL_XTL_FORCE_PU, cfg.xtal_fpu);

  if (REG_GET_FIELD(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_ANA_CLK_RTC_SEL) ==
                                                      RTC_SLOW_FREQ_8MD256)
    {
      modifyreg32(RTC_CNTL_CLK_CONF_REG, 0, RTC_CNTL_CK8M_FORCE_PU);
    }
  else
    {
      modifyreg32(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_CK8M_FORCE_PU, 0);
    }

  /* enable VDDSDIO control by state machine */

  modifyreg32(RTC_CNTL_SDIO_CONF_REG, RTC_CNTL_SDIO_FORCE, 0);
  REG_SET_FIELD(RTC_CNTL_SDIO_CONF_REG, RTC_CNTL_SDIO_PD_EN,
                cfg.vddsdio_pd_en);
  REG_SET_FIELD(RTC_CNTL_REG, RTC_CNTL_DBIAS_SLP, cfg.rtc_dbias_slp);
  REG_SET_FIELD(RTC_CNTL_REG, RTC_CNTL_DBIAS_WAK, cfg.rtc_dbias_wak);
  REG_SET_FIELD(RTC_CNTL_REG, RTC_CNTL_DIG_DBIAS_WAK, cfg.dig_dbias_wak);
  REG_SET_FIELD(RTC_CNTL_REG, RTC_CNTL_DIG_DBIAS_SLP, cfg.dig_dbias_slp);
}

/****************************************************************************
 * Name: esp32_rtc_sleep_start
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

void IRAM_ATTR esp32_rtc_sleep_start(uint32_t wakeup_opt,
                                     uint32_t reject_opt)
{
  REG_SET_FIELD(RTC_CNTL_WAKEUP_STATE_REG, RTC_CNTL_WAKEUP_ENA, wakeup_opt);
  putreg32((uint32_t)reject_opt, RTC_CNTL_SLP_REJECT_CONF_REG);

  /* Start entry into sleep mode */

  modifyreg32(RTC_CNTL_STATE0_REG, 0, RTC_CNTL_SLEEP_EN);

  while ((getreg32(RTC_CNTL_INT_RAW_REG) &
         (RTC_CNTL_SLP_REJECT_INT_RAW | RTC_CNTL_SLP_WAKEUP_INT_RAW)) == 0);

  /* In deep sleep mode, we never get here */

  modifyreg32(RTC_CNTL_INT_CLR_REG, 0,
              RTC_CNTL_SLP_REJECT_INT_CLR | RTC_CNTL_SLP_WAKEUP_INT_CLR);

  /* restore DBG_ATTEN to the default value */

  REG_SET_FIELD(RTC_CNTL_BIAS_CONF_REG, RTC_CNTL_DBG_ATTEN,
                RTC_CNTL_DBG_ATTEN_DEFAULT);
}
