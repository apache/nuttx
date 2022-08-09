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
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/spinlock.h>

#include "clock/clock.h"

#include "esp32_clockconfig.h"
#include "esp32_rt_timer.h"

#include "hardware/esp32_rtccntl.h"
#include "hardware/esp32_dport.h"
#include "hardware/esp32_i2s.h"

#include "xtensa.h"
#include "xtensa_attr.h"

#include "esp32_rtc.h"

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

#define XTAL_32K_DAC_VAL            1
#define XTAL_32K_DRES_VAL           3
#define XTAL_32K_DBIAS_VAL          0

#define XTAL_32K_EXT_DAC_VAL        2
#define XTAL_32K_EXT_DRES_VAL       3
#define XTAL_32K_EXT_DBIAS_VAL      1

#define DELAY_SLOW_CLK_SWITCH       300

#define DELAY_8M_ENABLE             50

#define RETRY_CAL_EXT               1

/* Lower threshold for a reasonably-looking calibration value for a 32k XTAL.
 * The ideal value (assuming 32768 Hz frequency)
 * is 1000000/32768*(2**19) = 16*10^6.
 */

#define MIN_32K_XTAL_CAL_VAL        15000000L

/* Frequency of the 8M oscillator is 8.5MHz +/- 5%, at the default DCAP
 * setting
 */

#define RTC_FAST_CLK_FREQ_8M        8500000
#define RTC_SLOW_CLK_FREQ_150K      150000
#define RTC_SLOW_CLK_FREQ_8MD256    (RTC_FAST_CLK_FREQ_8M / 256)
#define RTC_SLOW_CLK_FREQ_32K       32768

/* Number of fractional bits in values returned by rtc_clk_cal */

#define RTC_CLK_CAL_FRACT           19

/* With the default value of CK8M_DFREQ,
 * 8M clock frequency is 8.5 MHz +/- 7%
 */

#define RTC_FAST_CLK_FREQ_APPROX    8500000

/* Disable logging from the ROM code. */

#define RTC_DISABLE_ROM_LOG ((1 << 0) | (1 << 16))

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

/* The magic data for the struct esp32_rtc_backup_s that is in RTC slow
 * memory.
 */

#define MAGIC_RTC_SAVE UINT64_C(0x11223344556677)

/* RTC Memory & Store Register usage */

#define RTC_SLOW_CLK_CAL_REG    RTC_CNTL_STORE1_REG /* RTC_SLOW_CLK calibration value */
#define RTC_BOOT_TIME_LOW_REG   RTC_CNTL_STORE2_REG /* Boot time, low word */
#define RTC_BOOT_TIME_HIGH_REG  RTC_CNTL_STORE3_REG /* Boot time, high word */
#define RTC_XTAL_FREQ_REG       RTC_CNTL_STORE4_REG /* External XTAL frequency */
#define RTC_APB_FREQ_REG        RTC_CNTL_STORE5_REG /* APB bus frequency */
#define RTC_ENTRY_ADDR_REG      RTC_CNTL_STORE6_REG /* FAST_RTC_MEMORY_ENTRY */
#define RTC_RESET_CAUSE_REG     RTC_CNTL_STORE6_REG
#define RTC_MEMORY_CRC_REG      RTC_CNTL_STORE7_REG /* FAST_RTC_MEMORY_CRC */

/****************************************************************************
 * Private Types
 ****************************************************************************/

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
  uint32_t wifi_pd_en : 1;             /* power down Wi-Fi */
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
  uint32_t bb_pd : 1;     /* Set to 1 to power down Wi-Fi in sleep */
  uint32_t nrx_pd : 1;    /* Set to 1 to power down Wi-Fi in sleep */
  uint32_t fe_pd : 1;     /* Set to 1 to power down Wi-Fi in sleep */
};

#ifdef CONFIG_RTC_ALARM
struct alm_cbinfo_s
{
  struct rt_timer_s *alarm_hdl;  /* Timer id point to here */
  volatile alm_callback_t ac_cb; /* Client callback function */
  volatile void *ac_arg;         /* Argument to pass with the callback function */
  uint64_t deadline_us;
  uint8_t index;
};
#endif

struct esp32_rtc_backup_s
{
  uint64_t magic;
  int64_t  offset;              /* Offset time from RTC HW value */
  int64_t  reserved0;
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
static void IRAM_ATTR esp32_rtc_clk_slow_freq_set(
                      enum esp32_rtc_slow_freq_e slow_freq);
static void esp32_select_rtc_slow_clk(enum esp32_slow_clk_sel_e slow_clk);
static void esp32_rtc_clk_32k_enable(int ac, int res, int bias);
static void IRAM_ATTR esp32_rtc_clk_8m_enable(bool clk_8m_en, bool d256_en);
static uint32_t IRAM_ATTR esp32_rtc_clk_slow_freq_get_hz(void);

#ifdef CONFIG_RTC_ALARM
static void IRAM_ATTR esp32_rt_cb_handler(void *arg);
#endif

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

/* Callback to use when the alarm expires */

#ifdef CONFIG_RTC_ALARM
static struct alm_cbinfo_s g_alarmcb[RTC_ALARM_LAST];
#endif

static RTC_DATA_ATTR struct esp32_rtc_backup_s rtc_saved_data;

/* Saved data for persistent RTC time */

static struct esp32_rtc_backup_s *g_rtc_save;
static bool g_rt_timer_enabled = false;

/****************************************************************************
 * Public Data
 ****************************************************************************/

volatile bool g_rtc_enabled = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
  up_udelay(DELAY_FAST_CLK_SWITCH);
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
 *   Number of XTAL clock cycles within the given number of slow clock
 *   cycles.
 *   In case of error, return 0 cycle.
 *
 ****************************************************************************/

static uint32_t IRAM_ATTR esp32_rtc_clk_cal_internal(
                enum esp32_rtc_cal_sel_e cal_clk, uint32_t slowclk_cycles)
{
  uint32_t expected_freq;
  uint32_t us_time_estimate;
  uint32_t us_timer_max;
  uint32_t clks_state;
  uint32_t clks_mask;
  int timeout_us;
  enum esp32_rtc_slow_freq_e slow_freq;
  enum esp32_rtc_xtal_freq_e xtal_freq;

  /* Get the current state */

  clks_mask  = (RTC_CNTL_DIG_XTAL32K_EN_M | RTC_CNTL_DIG_CLK8M_D256_EN_M);
  clks_state = getreg32(RTC_CNTL_CLK_CONF_REG);
  clks_state &= clks_mask;

  /* Enable requested clock (150k clock is always on) */

  if (cal_clk == RTC_CAL_32K_XTAL && !(clks_state & RTC_CNTL_DIG_XTAL32K_EN))
    {
      REG_SET_FIELD(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_DIG_XTAL32K_EN, 1);
    }
  else if (cal_clk == RTC_CAL_8MD256 &&
           !(clks_state & RTC_CNTL_DIG_CLK8M_D256_EN))
    {
      modifyreg32(RTC_CNTL_CLK_CONF_REG, 0, RTC_CNTL_DIG_CLK8M_D256_EN);
    }

  /* Prepare calibration */

  REG_SET_FIELD(TIMG_RTCCALICFG_REG(0), TIMG_RTC_CALI_CLK_SEL, cal_clk);
  modifyreg32(TIMG_RTCCALICFG_REG(0), TIMG_RTC_CALI_START_CYCLING, 0);
  REG_SET_FIELD(TIMG_RTCCALICFG_REG(0), TIMG_RTC_CALI_MAX, slowclk_cycles);

  /* Figure out how long to wait for calibration to finish */

  slow_freq = REG_GET_FIELD(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_ANA_CLK_RTC_SEL);

  if (cal_clk == RTC_CAL_32K_XTAL || slow_freq == RTC_SLOW_FREQ_32K_XTAL)
    {
      expected_freq = 32768; /* standard 32k XTAL */
    }
  else if (cal_clk == RTC_CAL_8MD256 || slow_freq == RTC_SLOW_FREQ_8MD256)
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
      rtcerr("Estimated time overflows TIMG_RTC_CALI_VALUE\n");
      return 0;
    }

  /* Start calibration */

  modifyreg32(TIMG_RTCCALICFG_REG(0), TIMG_RTC_CALI_START, 0);
  modifyreg32(TIMG_RTCCALICFG_REG(0), 0, TIMG_RTC_CALI_START);

  /* Wait the expected time calibration should take */

  up_udelay(us_time_estimate);

  /* Wait for calibration to finish up to another us_time_estimate */

  timeout_us = us_time_estimate;
  while (!(getreg32(TIMG_RTCCALICFG_REG(0)) &
           TIMG_RTC_CALI_RDY) && (timeout_us > 0))
    {
      timeout_us--;
      up_udelay(1);
    }

  /* Restore the previous clocks states */

  modifyreg32(RTC_CNTL_CLK_CONF_REG, clks_mask, clks_state);

  /* Verify if this calibration occured within the timeout */

  if (timeout_us == 0)
    {
      /* Timed out waiting for calibration */

      rtcerr("Timed out waiting for calibration\n");
      return 0;
    }

  return REG_GET_FIELD(TIMG_RTCCALICFG1_REG(0), TIMG_RTC_CALI_VALUE);
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

  up_udelay(DELAY_SLOW_CLK_SWITCH);
}

/****************************************************************************
 * Name: esp32_rtc_clk_32k_enable
 *
 * Description:
 *   Enable 32 kHz XTAL oscillator
 *
 * Input Parameters:
 *   ac   - The current of XTAL oscillator.
 *   res  - The resistance of XTAL oscillator.
 *   bias - The bias voltage of XTAL oscillator.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void esp32_rtc_clk_32k_enable(int ac, int res, int bias)
{
  modifyreg32(RTC_IO_XTAL_32K_PAD_REG, RTC_IO_X32P_RDE | RTC_IO_X32P_RUE |
              RTC_IO_X32N_RUE | RTC_IO_X32N_RDE | RTC_IO_X32N_FUN_IE |
              RTC_IO_X32P_FUN_IE, RTC_IO_X32N_MUX_SEL | RTC_IO_X32P_MUX_SEL);

  /* Set the parameters of xtal */

  REG_SET_FIELD(RTC_IO_XTAL_32K_PAD_REG, RTC_IO_DAC_XTAL_32K, ac);
  REG_SET_FIELD(RTC_IO_XTAL_32K_PAD_REG, RTC_IO_DRES_XTAL_32K, res);
  REG_SET_FIELD(RTC_IO_XTAL_32K_PAD_REG, RTC_IO_DBIAS_XTAL_32K, bias);

  /* Power up external xtal */

  modifyreg32(RTC_IO_XTAL_32K_PAD_REG, 0, RTC_IO_XPD_XTAL_32K_M);
}

/****************************************************************************
 * Name: esp32_rtc_clk_8m_enable
 *
 * Description:
 *   Enable or disable 8 MHz internal oscillator
 *
 * Input Parameters:
 *   clk_8m_en - true to enable 8MHz generator, false to disable
 *   d256_en   - true to enable /256 divider, false to disable
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void IRAM_ATTR esp32_rtc_clk_8m_enable(bool clk_8m_en, bool d256_en)
{
  if (clk_8m_en)
    {
      modifyreg32(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_ENB_CK8M, 0);

      /* no need to wait once enabled by software */

      REG_SET_FIELD(RTC_CNTL_TIMER1_REG, RTC_CNTL_CK8M_WAIT, 1);
      if (d256_en)
        {
          modifyreg32(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_ENB_CK8M_DIV, 0);
        }
      else
        {
          modifyreg32(RTC_CNTL_CLK_CONF_REG, 0, RTC_CNTL_ENB_CK8M_DIV);
        }

      up_udelay(DELAY_8M_ENABLE);
    }
  else
    {
      modifyreg32(RTC_CNTL_CLK_CONF_REG, 0, RTC_CNTL_ENB_CK8M);
      REG_SET_FIELD(RTC_CNTL_TIMER1_REG, RTC_CNTL_CK8M_WAIT,
                    RTC_CNTL_CK8M_WAIT_DEFAULT);
    }
}

/****************************************************************************
 * Name: esp32_rtc_clk_slow_freq_get_hz
 *
 * Description:
 *   Get the approximate frequency of RTC_SLOW_CLK, in Hz
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   slow_clk_freq - RTC_SLOW_CLK frequency, in Hz
 *
 ****************************************************************************/

static uint32_t IRAM_ATTR esp32_rtc_clk_slow_freq_get_hz(void)
{
  enum esp32_rtc_slow_freq_e slow_clk_freq =
              REG_GET_FIELD(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_ANA_CLK_RTC_SEL);
  switch (slow_clk_freq)
    {
      case RTC_SLOW_FREQ_RTC:
        return RTC_SLOW_CLK_FREQ_150K;

      case RTC_SLOW_FREQ_32K_XTAL:
        return RTC_SLOW_CLK_FREQ_32K;

      case RTC_SLOW_FREQ_8MD256:
        return RTC_SLOW_CLK_FREQ_8MD256;
    }

  return OK;
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
  /* Number of times to repeat 32k XTAL calibration before giving up and
   * switching to the internal RC.
   */

  int retry_32k_xtal = RETRY_CAL_EXT;
  uint32_t cal_val = 0;
  uint64_t cal_dividend;
  enum esp32_rtc_slow_freq_e rtc_slow_freq = slow_clk &
                                             RTC_CNTL_ANA_CLK_RTC_SEL_V;

  do
    {
      if (rtc_slow_freq == RTC_SLOW_FREQ_32K_XTAL)
        {
          /* 32k XTAL oscillator needs to be enabled and running before
           * it can be used. Hardware doesn't have a direct way of checking
           * if the oscillator is running. Here we use rtc_clk_cal function
           * to count the number of main XTAL cycles in the given number of
           * 32k XTAL oscillator cycles. If the 32k XTAL has not started up,
           * calibration will time out, returning 0.
           */

          rtcinfo("Waiting for 32k oscillator to start up\n");
          if (slow_clk == SLOW_CLK_32K_XTAL)
            {
              esp32_rtc_clk_32k_enable(XTAL_32K_DAC_VAL, XTAL_32K_DRES_VAL,
                                       XTAL_32K_DBIAS_VAL);
            }
          else if (slow_clk == SLOW_CLK_32K_EXT_OSC)
            {
              esp32_rtc_clk_32k_enable(XTAL_32K_EXT_DAC_VAL,
                            XTAL_32K_EXT_DRES_VAL, XTAL_32K_EXT_DBIAS_VAL);
            }

          if (SLOW_CLK_CAL_CYCLES > 0)
            {
              cal_val = esp32_rtc_clk_cal(RTC_CAL_32K_XTAL,
                                            SLOW_CLK_CAL_CYCLES);
              if (cal_val == 0 || cal_val < MIN_32K_XTAL_CAL_VAL)
                {
                  if (retry_32k_xtal-- > 0)
                    {
                      continue;
                    }

                  rtc_slow_freq = RTC_SLOW_FREQ_RTC;
                }
            }
        }
      else if (rtc_slow_freq == RTC_SLOW_FREQ_8MD256)
        {
          esp32_rtc_clk_8m_enable(true, true);
        }

      esp32_rtc_clk_slow_freq_set(rtc_slow_freq);
      if (SLOW_CLK_CAL_CYCLES > 0)
        {
          /* 32k XTAL oscillator has some frequency drift at startup. Improve
           * calibration routine to wait until the frequency is stable.
           */

          cal_val = esp32_rtc_clk_cal(RTC_CAL_RTC_MUX,
                                      SLOW_CLK_CAL_CYCLES);
        }
      else
        {
          cal_dividend = (1ULL << RTC_CLK_CAL_FRACT) * 1000000ULL;
          cal_val = (uint32_t) (cal_dividend /
                                esp32_rtc_clk_slow_freq_get_hz());
        }
    }
  while (cal_val == 0);
  rtcinfo("RTC_SLOW_CLK calibration value: %d\n", cal_val);
  putreg32((uint32_t)cal_val, RTC_SLOW_CLK_CAL_REG);
}

#ifdef CONFIG_RTC_ALARM

/****************************************************************************
 * Name: esp32_rt_cb_handler
 *
 * Description:
 *   RT-Timer service routine
 *
 * Input Parameters:
 *   arg - Information about the RT-Timer configuration.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void IRAM_ATTR esp32_rt_cb_handler(void *arg)
{
  struct alm_cbinfo_s *cbinfo = (struct alm_cbinfo_s *)arg;
  alm_callback_t cb;
  void *cb_arg;
  int alminfo_id;

  DEBUGASSERT(cbinfo != NULL);
  alminfo_id = cbinfo->index;
  DEBUGASSERT((RTC_ALARM0 <= alminfo_id) &&
              (alminfo_id < RTC_ALARM_LAST));

  if (cbinfo->ac_cb != NULL)
    {
      /* Alarm callback */

      cb = cbinfo->ac_cb;
      cb_arg = (void *)cbinfo->ac_arg;
      cbinfo->ac_cb  = NULL;
      cbinfo->ac_arg = NULL;
      cbinfo->deadline_us = 0;
      cb(cb_arg, alminfo_id);
    }
}

#endif /* CONFIG_RTC_ALARM */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_rtc_get_slow_clk_rtc
 *
 * Description:
 *   Get slow_clk_rtc source.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The clock source:
 *   -  SLOW_CK
 *   -  CK_XTAL_32K
 *   -  CK8M_D256_OUT
 *
 ****************************************************************************/

enum esp32_rtc_slow_freq_e IRAM_ATTR esp32_rtc_get_slow_clk(void)
{
  enum esp32_rtc_slow_freq_e slow_freq;

  /* Get the clock source for slow_clk_rtc */

  slow_freq = REG_GET_FIELD(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_ANA_CLK_RTC_SEL);

  return slow_freq;
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

uint32_t IRAM_ATTR esp32_rtc_clk_cal(enum esp32_rtc_cal_sel_e cal_clk,
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
  esp32_update_cpu_freq(freq);

  /* set divider from XTAL to APB clock */

  REG_SET_FIELD(APB_CTRL_SYSCLK_CONF_REG, APB_CTRL_PRE_DIV_CNT, div - 1);

  /* adjust ref_tick */

  modifyreg32(APB_CTRL_XTAL_TICK_CONF_REG, 0,
             (freq * MHZ) / REF_CLK_FREQ - 1);

  /* switch clock source */

  REG_SET_FIELD(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_SOC_CLK_SEL,
                RTC_CNTL_SOC_CLK_SEL_XTL);
  putreg32(value, RTC_APB_FREQ_REG);

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
      up_udelay(DELAY_PLL_DBIAS_RAISE);

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

void esp32_rtc_clk_set(void)
{
  enum esp32_rtc_fast_freq_e fast_freq = RTC_FAST_FREQ_8M;
  enum esp32_slow_clk_sel_e slow_clk = SLOW_CLK_150K;

#if defined(CONFIG_ESP32_RTC_CLK_SRC_EXT_CRYS)
  slow_clk = SLOW_CLK_32K_XTAL;
#elif defined(CONFIG_ESP32_RTC_CLK_SRC_EXT_OSC)
  slow_clk = SLOW_CLK_32K_EXT_OSC;
#elif defined(CONFIG_ESP32_RTC_CLK_SRC_INT_8MD256)
  slow_clk = SLOW_CLK_8MD256;
#endif

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

void IRAM_ATTR esp32_rtc_init(void)
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
      up_udelay(1);
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
 *   slow_clk_period - Period of slow clock in microseconds
 *
 * Returned Value:
 *   Number of slow clock cycles
 *
 ****************************************************************************/

uint64_t IRAM_ATTR esp32_rtc_time_us_to_slowclk(uint64_t time_in_us,
                                                uint32_t period)
{
  uint64_t slow_clk_cycles = 0;
  uint64_t max_time_in_us = (UINT64_C(1) << 45) - 1;

  /* Handle overflow that would happen if time_in_us >= 2^45 */

  while (time_in_us > max_time_in_us)
    {
      time_in_us      -= max_time_in_us;
      slow_clk_cycles += ((max_time_in_us << RTC_CLK_CAL_FRACT) / period);
    }

  slow_clk_cycles += ((time_in_us << RTC_CLK_CAL_FRACT) / period);

  return slow_clk_cycles;
}

/****************************************************************************
 * Name: esp32_rtc_time_slowclk_to_us
 *
 * Description:
 *   Convert time interval from RTC_SLOW_CLK to microseconds
 *
 * Input Parameters:
 *   rtc_cycles - Time interval in RTC_SLOW_CLK cycles
 *   period     - Period of slow clock in microseconds
 *
 * Returned Value:
 *   Time interval in microseconds
 *
 ****************************************************************************/

uint64_t IRAM_ATTR esp32_rtc_time_slowclk_to_us(uint64_t rtc_cycles,
                                                uint32_t period)
{
  return (rtc_cycles * period) >> RTC_CLK_CAL_FRACT;
}

/****************************************************************************
 * Name: esp32_clk_slowclk_cal_get
 *
 * Description:
 *   Get the calibration value of RTC slow clock.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   the calibration value obtained using rtc_clk_cal
 *
 ****************************************************************************/

uint32_t IRAM_ATTR esp32_clk_slowclk_cal_get(void)
{
  return getreg32(RTC_SLOW_CLK_CAL_REG);
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

  up_udelay(1);

  while (!(getreg32(TIMG_RTCCALICFG_REG(0)) & TIMG_RTC_CALI_RDY))
    {
      up_udelay(1);
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
 * Name: esp_rtc_clk_get_cpu_freq
 *
 * Description:
 *   Get the currently used CPU frequency configuration.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   CPU frequency
 *
 ****************************************************************************/

int IRAM_ATTR esp_rtc_clk_get_cpu_freq(void)
{
  uint32_t source_freq_mhz;
  uint32_t div;
  uint32_t soc_clk_sel;
  uint32_t cpuperiod_sel;
  int freq_mhz = 0;

  soc_clk_sel = REG_GET_FIELD(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_SOC_CLK_SEL);
  switch (soc_clk_sel)
    {
      case RTC_CNTL_SOC_CLK_SEL_XTL:
        {
          div = REG_GET_FIELD(APB_CTRL_SYSCLK_CONF_REG,
                              APB_CTRL_PRE_DIV_CNT) + 1;
          source_freq_mhz = (uint32_t) esp32_rtc_clk_xtal_freq_get();
          freq_mhz = source_freq_mhz / div;
        }
        break;

      case RTC_CNTL_SOC_CLK_SEL_PLL:
        {
          cpuperiod_sel = REG_GET_FIELD(DPORT_CPU_PER_CONF_REG,
                                        DPORT_CPUPERIOD_SEL);
          if (cpuperiod_sel == DPORT_CPUPERIOD_SEL_80)
            {
              freq_mhz = 80;
            }
          else if (cpuperiod_sel == DPORT_CPUPERIOD_SEL_160)
            {
              freq_mhz = 160;
            }
          else if (cpuperiod_sel == DPORT_CPUPERIOD_SEL_240)
            {
              freq_mhz = 240;
            }
          else
            {
              DEBUGPANIC();
            }
        }
        break;

      case RTC_CNTL_SOC_CLK_SEL_8M:
        {
          freq_mhz = 8;
        }
        break;

      case RTC_CNTL_SOC_CLK_SEL_APLL:
        default:
          DEBUGPANIC();
    }

  return freq_mhz;
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

  /* Keep the RTC8M_CLK on in light_sleep mode if the
   * ledc low-speed channel is clocked by RTC8M_CLK.
   */

  if (!cfg.deep_slp && GET_PERI_REG_MASK(RTC_CNTL_CLK_CONF_REG,
                                         RTC_CNTL_DIG_CLK8M_EN_M))
    {
      REG_CLR_BIT(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_CK8M_FORCE_PD);
      REG_SET_BIT(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_CK8M_FORCE_PU);
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

int IRAM_ATTR esp32_rtc_sleep_start(uint32_t wakeup_opt,
                                     uint32_t reject_opt)
{
  int reject;
  REG_SET_FIELD(RTC_CNTL_WAKEUP_STATE_REG, RTC_CNTL_WAKEUP_ENA, wakeup_opt);
  putreg32((uint32_t)reject_opt, RTC_CNTL_SLP_REJECT_CONF_REG);

  /* Start entry into sleep mode */

  modifyreg32(RTC_CNTL_STATE0_REG, 0, RTC_CNTL_SLEEP_EN);

  while ((getreg32(RTC_CNTL_INT_RAW_REG) &
         (RTC_CNTL_SLP_REJECT_INT_RAW | RTC_CNTL_SLP_WAKEUP_INT_RAW)) == 0);

  /* In deep sleep mode, we never get here */

  reject = REG_GET_FIELD(RTC_CNTL_INT_RAW_REG, RTC_CNTL_SLP_REJECT_INT_RAW);

  modifyreg32(RTC_CNTL_INT_CLR_REG, 0,
              RTC_CNTL_SLP_REJECT_INT_CLR | RTC_CNTL_SLP_WAKEUP_INT_CLR);

  /* restore DBG_ATTEN to the default value */

  REG_SET_FIELD(RTC_CNTL_BIAS_CONF_REG, RTC_CNTL_DBG_ATTEN,
                RTC_CNTL_DBG_ATTEN_DEFAULT);
  return reject;
}

/****************************************************************************
 * Name: esp32_rtc_get_time_us
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

uint64_t esp32_rtc_get_time_us(void)
{
  const uint32_t cal = getreg32(RTC_SLOW_CLK_CAL_REG);
  const uint64_t rtc_this_ticks = esp32_rtc_time_get();

  /* RTC counter result is up to 2^48, calibration factor is up to 2^24,
   * for a 32kHz clock. We need to calculate (assuming no overflow):
   * (ticks * cal) >> RTC_CLK_CAL_FRACT. An overflow in the (ticks * cal)
   * multiplication would cause time to wrap around after approximately
   * 13 days, which is probably not enough for some applications.
   * Therefore multiplication is split into two terms, for the lower 32-bit
   * and the upper 16-bit parts of "ticks", i.e.:
   * ((ticks_low + 2^32 * ticks_high) * cal) >> RTC_CLK_CAL_FRACT
   */

  const uint64_t ticks_low = rtc_this_ticks & UINT32_MAX;
  const uint64_t ticks_high = rtc_this_ticks >> 32;
  const uint64_t delta_time_us = ((ticks_low * cal) >> RTC_CLK_CAL_FRACT) +
          ((ticks_high * cal) << (32 - RTC_CLK_CAL_FRACT));

  return delta_time_us;
}

/****************************************************************************
 * Name: esp32_rtc_set_boot_time
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

void IRAM_ATTR esp32_rtc_set_boot_time(uint64_t time_us)
{
  putreg32((uint32_t)(time_us & UINT32_MAX), RTC_BOOT_TIME_LOW_REG);
  putreg32((uint32_t)(time_us >> 32), RTC_BOOT_TIME_HIGH_REG);
}

/****************************************************************************
 * Name: esp32_rtc_get_boot_time
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

uint64_t IRAM_ATTR esp32_rtc_get_boot_time(void)
{
  return ((uint64_t)getreg32(RTC_BOOT_TIME_LOW_REG))
        + (((uint64_t)getreg32(RTC_BOOT_TIME_HIGH_REG)) << 32);
}

/****************************************************************************
 * Name: up_rtc_time
 *
 * Description:
 *   Get the current time in seconds.  This is similar to the standard time()
 *   function.  This interface is only required if the low-resolution
 *   RTC/counter hardware implementation is selected.  It is only used by the
 *   RTOS during initialization to set up the system time when CONFIG_RTC is
 *   set but CONFIG_RTC_HIRES is not set.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The current time in seconds
 *
 ****************************************************************************/

#ifndef CONFIG_RTC_HIRES
time_t up_rtc_time(void)
{
  uint64_t time_us;
  irqstate_t flags;

  flags = spin_lock_irqsave(NULL);

  /* NOTE: RT-Timer starts to work after the board is initialized, and the
   * RTC controller starts works after up_rtc_initialize is initialized.
   * Since the system clock starts to work before the board is initialized,
   * if CONFIG_RTC is enabled, the system time must be matched by the time
   * of the RTC controller (up_rtc_initialize has already been initialized,
   * and RT-Timer cannot work).
   */

  /* Determine if RT-Timer is started */

  if (g_rt_timer_enabled == true)
    {
      /* Get the time from RT-Timer, the time interval between RTC
       * controller and RT-Timer is stored in g_rtc_save->offset.
       */

      time_us = rt_timer_time_us() + g_rtc_save->offset +
                              esp32_rtc_get_boot_time();
    }
  else
    {
      /* Get the time from RTC controller. */

      time_us = esp32_rtc_get_time_us() +
                  esp32_rtc_get_boot_time();
    }

  spin_unlock_irqrestore(NULL, flags);

  return (time_t)(time_us / USEC_PER_SEC);
}
#endif /* !CONFIG_RTC_HIRES */

/****************************************************************************
 * Name: up_rtc_settime
 *
 * Description:
 *   Set the RTC to the provided time. All RTC implementations must be
 *   able to set their time based on a standard timespec.
 *
 * Input Parameters:
 *   ts - the time to use
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int up_rtc_settime(const struct timespec *ts)
{
  irqstate_t flags;
  uint64_t now_us;
  uint64_t rtc_offset_us;

  DEBUGASSERT(ts != NULL && ts->tv_nsec < NSEC_PER_SEC);
  flags = spin_lock_irqsave(NULL);

  now_us = ((uint64_t) ts->tv_sec) * USEC_PER_SEC +
          ts->tv_nsec / NSEC_PER_USEC;
  if (g_rt_timer_enabled == true)
    {
      /* Set based on RT-Timer offset value. */

      rtc_offset_us = now_us - rt_timer_time_us();
    }
  else
    {
      /* Set based on the offset value of the RT controller. */

      rtc_offset_us = now_us - esp32_rtc_get_time_us();
    }

  g_rtc_save->offset = 0;
  esp32_rtc_set_boot_time(rtc_offset_us);

  spin_unlock_irqrestore(NULL, flags);

  return OK;
}

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

int up_rtc_initialize(void)
{
#ifndef CONFIG_PM
  /* Initialize RTC controller parameters */

  esp32_rtc_init();
  esp32_rtc_clk_set();
#endif

  g_rtc_save = &rtc_saved_data;

  /* If saved data is invalid, clear offset information */

  if (g_rtc_save->magic != MAGIC_RTC_SAVE)
    {
      g_rtc_save->magic = MAGIC_RTC_SAVE;
      g_rtc_save->offset = 0;
      esp32_rtc_set_boot_time(0);
    }

#ifdef CONFIG_RTC_HIRES
  /* Synchronize the base time to the RTC time */

  up_rtc_gettime(&g_basetime);
#endif

  g_rtc_enabled = true;

  return OK;
}

/****************************************************************************
 * Name: up_rtc_gettime
 *
 * Description:
 *   Get the current time from the high resolution RTC time or RT-Timer. This
 *   interface is only supported by the high-resolution RTC/counter hardware
 *   implementation. It is used to replace the system timer.
 *
 * Input Parameters:
 *   tp - The location to return the RTC time or RT-Timer value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_HIRES
int up_rtc_gettime(struct timespec *tp)
{
  irqstate_t flags;
  uint64_t time_us;

  flags = spin_lock_irqsave(NULL);

  if (g_rt_timer_enabled == true)
    {
      time_us = rt_timer_time_us() + g_rtc_save->offset +
                              esp32_rtc_get_boot_time();
    }
  else
    {
      time_us = esp32_rtc_get_time_us() + esp32_rtc_get_boot_time();
    }

  tp->tv_sec  = time_us / USEC_PER_SEC;
  tp->tv_nsec = (time_us % USEC_PER_SEC) * NSEC_PER_USEC;

  spin_unlock_irqrestore(NULL, flags);

  return OK;
}
#endif /* CONFIG_RTC_HIRES */

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

int up_rtc_setalarm(struct alm_setalarm_s *alminfo)
{
  struct rt_timer_args_s rt_timer_args;
  struct alm_cbinfo_s *cbinfo;
  irqstate_t flags;
  int ret = -EBUSY;
  int id;

  DEBUGASSERT(alminfo != NULL);
  DEBUGASSERT((RTC_ALARM0 <= alminfo->as_id) &&
              (alminfo->as_id < RTC_ALARM_LAST));

  /* Set the alarm in RT-Timer */

  id = alminfo->as_id;
  cbinfo = &g_alarmcb[id];

  if (cbinfo->ac_cb == NULL)
    {
      /* Create the RT-Timer alarm */

      flags = spin_lock_irqsave(NULL);

      if (cbinfo->alarm_hdl == NULL)
        {
          cbinfo->index = id;
          rt_timer_args.arg = cbinfo;
          rt_timer_args.callback = esp32_rt_cb_handler;
          ret = rt_timer_create(&rt_timer_args, &cbinfo->alarm_hdl);
          if (ret < 0)
            {
              rtcerr("ERROR: Failed to create rt_timer error=%d\n", ret);
              spin_unlock_irqrestore(NULL, flags);
              return ret;
            }
        }

      cbinfo->ac_cb  = alminfo->as_cb;
      cbinfo->ac_arg = alminfo->as_arg;
      cbinfo->deadline_us = alminfo->as_time.tv_sec * USEC_PER_SEC +
                            alminfo->as_time.tv_nsec / NSEC_PER_USEC;

      if (cbinfo->alarm_hdl == NULL)
        {
          rtcerr("ERROR: failed to create alarm timer\n");
        }
      else
        {
          rtcinfo("Start RTC alarm.\n");
          rt_timer_start(cbinfo->alarm_hdl, cbinfo->deadline_us, false);
          ret = OK;
        }

      spin_unlock_irqrestore(NULL, flags);
    }

  return ret;
}

/****************************************************************************
 * Name: up_rtc_cancelalarm
 *
 * Description:
 *   Cancel an alarm.
 *
 * Input Parameters:
 *   alarmid - Identifies the alarm to be cancelled
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int up_rtc_cancelalarm(enum alm_id_e alarmid)
{
  struct alm_cbinfo_s *cbinfo;
  irqstate_t flags;
  int ret = -ENODATA;

  DEBUGASSERT((RTC_ALARM0 <= alarmid) &&
              (alarmid < RTC_ALARM_LAST));

  /* Set the alarm in hardware and enable interrupts */

  cbinfo = &g_alarmcb[alarmid];

  if (cbinfo->ac_cb != NULL)
    {
      flags = spin_lock_irqsave(NULL);

      /* Stop and delete the alarm */

      rtcinfo("Cancel RTC alarm.\n");
      rt_timer_stop(cbinfo->alarm_hdl);
      rt_timer_delete(cbinfo->alarm_hdl);
      cbinfo->ac_cb = NULL;
      cbinfo->deadline_us = 0;
      cbinfo->alarm_hdl = NULL;

      spin_unlock_irqrestore(NULL, flags);

      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: up_rtc_rdalarm
 *
 * Description:
 *   Query an alarm configured in hardware.
 *
 * Input Parameters:
 *   tp      - Location to return the timer match register.
 *   alarmid - Identifies the alarm to get.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int up_rtc_rdalarm(struct timespec *tp, uint32_t alarmid)
{
  irqstate_t flags;
  struct alm_cbinfo_s *cbinfo;
  DEBUGASSERT(tp != NULL);
  DEBUGASSERT((RTC_ALARM0 <= alarmid) &&
              (alarmid < RTC_ALARM_LAST));

  flags = spin_lock_irqsave(NULL);

  /* Get the alarm according to the alarmid */

  cbinfo = &g_alarmcb[alarmid];

  tp->tv_sec = (rt_timer_time_us() + g_rtc_save->offset +
              cbinfo->deadline_us) / USEC_PER_SEC;
  tp->tv_nsec = ((rt_timer_time_us() + g_rtc_save->offset +
              cbinfo->deadline_us) % USEC_PER_SEC) * NSEC_PER_USEC;

  spin_unlock_irqrestore(NULL, flags);

  return OK;
}

#endif /* CONFIG_RTC_ALARM */

/****************************************************************************
 * Name: up_rtc_timer_init
 *
 * Description:
 *   Init RTC timer.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int up_rtc_timer_init(void)
{
  /* RT-Timer enabled */

  g_rt_timer_enabled = true;

  /* Get the time difference between rt_timer and RTC timer */

  g_rtc_save->offset = esp32_rtc_get_time_us() - rt_timer_time_us();

  return OK;
}
