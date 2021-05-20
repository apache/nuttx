/****************************************************************************
 * arch/xtensa/src/esp32s2/esp32s2_rtc.c
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

#include "esp32s2_rtc.h"
#include "esp32s2_clockconfig.h"
#include "hardware/esp32s2_i2s.h"
#include "hardware/esp32s2_rtccntl.h"
#include "hardware/esp32s2_i2cbbpll.h"
#include "hardware/esp32s2_system.h"
#include "esp32s2_rtc.h"
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

/* Disable logging from the ROM code. */

#define RTC_DISABLE_ROM_LOG ((1 << 0) | (1 << 16))

/* Default initializer for esp32s2_rtc_sleep_config_t
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

/* RTC power and clock control initialization settings */

struct esp32s2_rtc_priv_s
{
  uint32_t ck8m_wait : 8;         /* Number of rtc_fast_clk cycles to wait for 8M clock to be ready */
  uint32_t xtal_wait : 8;         /* Number of rtc_fast_clk cycles to wait for XTAL clock to be ready */
  uint32_t pll_wait : 8;          /* Number of rtc_fast_clk cycles to wait for PLL to be ready */
  uint32_t clkctl_init : 1;       /* Perform clock control related initialization */
  uint32_t pwrctl_init : 1;       /* Perform power control related initialization */
  uint32_t rtc_dboost_fpd : 1;    /* Force power down RTC_DBOOST */
};

/* sleep configuration for rtc_sleep_init function */

struct esp32s2_rtc_sleep_config_s
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

struct esp32s2_rtc_sleep_pd_config_s
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

static void IRAM_ATTR esp32s2_rtc_sleep_pd(
                      struct esp32s2_rtc_sleep_pd_config_s cfg);
static inline bool esp32s2_clk_val_is_valid(uint32_t val);
static void IRAM_ATTR esp32s2_rtc_clk_fast_freq_set(
                      enum esp32s2_rtc_fast_freq_e fast_freq);
static uint32_t IRAM_ATTR esp32s2_rtc_clk_cal_internal(
                enum esp32s2_rtc_cal_sel_e cal_clk, uint32_t slowclk_cycles);
static void IRAM_ATTR esp32s2_rtc_clk_slow_freq_set(
                      enum esp32s2_rtc_slow_freq_e slow_freq);
static void esp32s2_select_rtc_slow_clk(enum esp32s2_slow_clk_sel_e
                                        slow_clk);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct esp32s2_rtc_priv_s esp32s2_rtc_priv =
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
 * Name: esp32s2_clk_val_is_valid
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

static inline bool esp32s2_clk_val_is_valid(uint32_t val)
{
  return (val & 0xffff) == ((val >> 16) & 0xffff)
                        && val != 0 && val != UINT32_MAX;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

enum esp32s2_rtc_xtal_freq_e rtc_get_xtal(void)
                __attribute__((alias("esp32s2_rtc_clk_xtal_freq_get")));

/****************************************************************************
 * Name: esp32s2_rtc_clk_xtal_freq_get
 *
 * Description:
 *   Get main XTAL frequency
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   XTAL frequency (one of enum esp32s2_rtc_xtal_freq_e values)
 *
 ****************************************************************************/

enum esp32s2_rtc_xtal_freq_e IRAM_ATTR esp32s2_rtc_clk_xtal_freq_get(void)
{
  /* We may have already written XTAL value into RTC_XTAL_FREQ_REG */

  uint32_t xtal_freq_reg = getreg32(RTC_XTAL_FREQ_REG);

  if (!esp32s2_clk_val_is_valid(xtal_freq_reg))
    {
      return RTC_XTAL_FREQ_AUTO;
    }

  return (xtal_freq_reg & ~RTC_DISABLE_ROM_LOG) & UINT16_MAX;
}

/****************************************************************************
 * Name: esp32s2_rtc_update_to_xtal
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

void IRAM_ATTR esp32s2_rtc_update_to_xtal(int freq, int div)
{
  uint32_t value = (((freq * MHZ) >> 12) & UINT16_MAX)
                   | ((((freq * MHZ) >> 12) & UINT16_MAX) << 16);
  esp32s2_update_cpu_freq(freq);

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
 * Name: esp32s2_rtc_bbpll_enable
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

void IRAM_ATTR esp32s2_rtc_bbpll_enable(void)
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
 * Name: esp32s2_rtc_bbpll_configure
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

void IRAM_ATTR esp32s2_rtc_bbpll_configure(
                     enum esp32s2_rtc_xtal_freq_e xtal_freq, int pll_freq)
{
  static uint8_t div_ref = 0;
  static uint8_t div7_0 = 0;
  static uint8_t dr1 = 0;
  static uint8_t dr3 = 0;
  static uint8_t dchgp = 0;
  static uint8_t dcur = 0;
  uint8_t i2c_bbpll_lref  = 0;
  uint8_t i2c_bbpll_div_7_0 = 0;
  uint8_t i2c_bbpll_dcur = 0;

  if (pll_freq == RTC_PLL_FREQ_480M)
    {
      /* Clear this register to let the digital part know 480M PLL is used */

      SET_PERI_REG_MASK(DPORT_CPU_PER_CONF_REG, SYSTEM_PLL_FREQ_SEL);

      /* Configure 480M PLL */

      div_ref = 0;
      div7_0 = 8;
      dr1 = 0;
      dr3 = 0;
      dchgp = 5;
      dcur = 4;

      I2C_WRITEREG_RTC(I2C_BBPLL, I2C_BBPLL_MODE_HF, 0x6b);
    }
  else
    {
      /* Clear this register to let the digital part know 320M PLL is used */

      CLEAR_PERI_REG_MASK(DPORT_CPU_PER_CONF_REG, SYSTEM_PLL_FREQ_SEL);

      /* Configure 320M PLL */

      div_ref = 0;
      div7_0 = 4;
      dr1 = 0;
      dr3 = 0;
      dchgp = 5;
      dcur = 5;

      I2C_WRITEREG_RTC(I2C_BBPLL, I2C_BBPLL_MODE_HF, 0x69);
    }

  i2c_bbpll_lref = (dchgp << I2C_BBPLL_OC_DCHGP_LSB) | (div_ref);
  i2c_bbpll_div_7_0 = div7_0;
  i2c_bbpll_dcur = (2 << I2C_BBPLL_OC_DLREF_SEL_LSB) |
                   (1 << I2C_BBPLL_OC_DHREF_SEL_LSB) | dcur;

  I2C_WRITEREG_RTC(I2C_BBPLL, I2C_BBPLL_OC_REF_DIV, i2c_bbpll_lref);
  I2C_WRITEREG_RTC(I2C_BBPLL, I2C_BBPLL_OC_DIV_7_0, i2c_bbpll_div_7_0);
  I2C_WRITEREG_MASK_RTC(I2C_BBPLL, I2C_BBPLL_OC_DR1, dr1);
  I2C_WRITEREG_MASK_RTC(I2C_BBPLL, I2C_BBPLL_OC_DR3, dr3);
  I2C_WRITEREG_RTC(I2C_BBPLL, I2C_BBPLL_OC_DCUR, i2c_bbpll_dcur);

  /* Enable calibration by software */

  I2C_WRITEREG_MASK_RTC(I2C_BBPLL, I2C_BBPLL_IR_CAL_ENX_CAP, 1);

  for (int ext_cap = 0; ext_cap < 16; ext_cap++)
    {
      uint8_t cal_result;

      I2C_WRITEREG_MASK_RTC(I2C_BBPLL, I2C_BBPLL_IR_CAL_EXT_CAP, ext_cap);
      cal_result = I2C_READREG_MASK_RTC(I2C_BBPLL, I2C_BBPLL_OR_CAL_CAP);
      if (cal_result == 0)
        {
          break;
        }

      if (ext_cap == 15)
        {
          ets_printf("BBPLL SOFTWARE CAL FAIL\n");
          abort();
        }
    }
}

/****************************************************************************
 * Name: esp32s2_rtc_wait_for_slow_cycle
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

void IRAM_ATTR esp32s2_rtc_wait_for_slow_cycle(void)
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
          source_freq_mhz = (uint32_t) esp32s2_rtc_clk_xtal_freq_get();
          freq_mhz = source_freq_mhz / div;
        }
        break;

      case RTC_CNTL_SOC_CLK_SEL_PLL:
        {
          cpuperiod_sel = REG_GET_FIELD(DPORT_CPU_PER_CONF_REG,
                                        SYSTEM_CPUPERIOD_SEL);
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
              DEBUGASSERT(0);
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
          DEBUGASSERT(0);
    }

  return freq_mhz;
}

