/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_rtc.c
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

#include "riscv_arch.h"

#include "hardware/esp32c3_rtccntl.h"
#include "hardware/esp32c3_soc.h"
#include "hardware/esp32c3_system.h"
#include "hardware/esp32c3_syscon.h"
#include "hardware/esp32c3_tim.h"
#include "hardware/apb_ctrl_reg.h"
#include "hardware/bb_reg.h"
#include "hardware/nrx_reg.h"
#include "hardware/fe_reg.h"
#include "hardware/regi2c_lp_bias.h"
#include "hardware/regi2c_dig_reg.h"
#include "hardware/regi2c_bbpll.h"
#include "hardware/regi2c_ctrl.h"
#include "hardware/extmem_reg.h"
#include "hardware/spi_mem_reg.h"

#include "esp32c3_rtc.h"
#include "esp32c3_clockconfig.h"
#include "esp32c3_attr.h"
#include "esp32c3_rt_timer.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OTHER_BLOCKS_POWERUP        1
#define OTHER_BLOCKS_WAIT           1

#define STR2(X) #X
#define STR(X) STR2(X)

#define MHZ                        (1000000)

/* Lower threshold for a reasonably-looking calibration value for a 32k XTAL.
 * The ideal value (assuming 32768 Hz frequency)
 * is 1000000/32768*(2**19) = 16*10^6.
 */

#define MIN_32K_XTAL_CAL_VAL        15000000L
#define DELAY_FAST_CLK_SWITCH       3
#define DELAY_SLOW_CLK_SWITCH       300
#define DELAY_8M_ENABLE             50

/* Indicates that this 32k oscillator gets input from external oscillator,
 * rather than a crystal.
 */

#define EXT_OSC_FLAG    BIT(3)

#define RTC_FAST_CLK_FREQ_8M        8500000

/* With the default value of CK8M_DFREQ,
 * 8M clock frequency is 8.5 MHz +/- 7%.
 */

#define RTC_FAST_CLK_FREQ_APPROX    8500000

#define RTC_PLL_FREQ_320M   320
#define RTC_PLL_FREQ_480M   480

#define RTC_SLOW_CLK_FREQ_90K      90000
#define RTC_SLOW_CLK_FREQ_8MD256    (RTC_FAST_CLK_FREQ_APPROX / 256)
#define RTC_SLOW_CLK_FREQ_32K       32768

#define RTC_SLOW_CLK_X32K_CAL_TIMEOUT_THRES(cycles)   (cycles << 12)
#define RTC_SLOW_CLK_8MD256_CAL_TIMEOUT_THRES(cycles) (cycles << 12)
#define RTC_SLOW_CLK_150K_CAL_TIMEOUT_THRES(cycles)   (cycles << 10)

/* Number of fractional bits in values returned by rtc_clk_cal */

#define RTC_CLK_CAL_FRACT  19

/* set sleep_init default param. */

#define RTC_CNTL_DBG_ATTEN_LIGHTSLEEP_DEFAULT  3
#define RTC_CNTL_DBG_ATTEN_DEEPSLEEP_DEFAULT  15
#define RTC_CNTL_DBG_ATTEN_MONITOR_DEFAULT  0
#define RTC_CNTL_BIASSLP_MONITOR_DEFAULT  0
#define RTC_CNTL_BIASSLP_SLEEP_DEFAULT  1
#define RTC_CNTL_PD_CUR_MONITOR_DEFAULT  0
#define RTC_CNTL_PD_CUR_SLEEP_DEFAULT  1
#define RTC_CNTL_DG_VDD_DRV_B_SLP_DEFAULT 254

#define RTC_CK8M_ENABLE_WAIT_DEFAULT 5

/* Approximate mapping of voltages to RTC_CNTL_DBIAS_WAK,
 * RTC_CNTL_DBIAS_SLP, RTC_CNTL_DIG_DBIAS_WAK,
 * RTC_CNTL_DIG_DBIAS_SLP values. Valid if RTC_CNTL_DBG_ATTEN is 0.
 */

#define RTC_CNTL_DBIAS_SLP  0
#define RTC_CNTL_DBIAS_0V90 13
#define RTC_CNTL_DBIAS_0V95 16
#define RTC_CNTL_DBIAS_1V00 18
#define RTC_CNTL_DBIAS_1V05 20
#define RTC_CNTL_DBIAS_1V10 23
#define RTC_CNTL_DBIAS_1V15 25
#define RTC_CNTL_DBIAS_1V20 28
#define RTC_CNTL_DBIAS_1V25 30
#define RTC_CNTL_DBIAS_1V30 31

#define DIG_DBIAS_XTAL      RTC_CNTL_DBIAS_1V10
#define DIG_DBIAS_2M        RTC_CNTL_DBIAS_1V00

#define DIG_DBIAS_80M   RTC_CNTL_DBIAS_1V20
#define DIG_DBIAS_160M  RTC_CNTL_DBIAS_1V20

/* Default initializer for esp32c3_rtc_sleep_config_s
 * This initializer sets all fields to "reasonable" values
 * (e.g. suggested for production use) based on a combination
 * of RTC_SLEEP_PD_x flags.
 */

#define RTC_SLEEP_CONFIG_DEFAULT(sleep_flags) { \
  .lslp_mem_inf_fpu = 0, \
  .rtc_mem_inf_follow_cpu = ((sleep_flags) & RTC_SLEEP_PD_RTC_MEM_FOLLOW_CPU) ? 1 : 0, \
  .rtc_fastmem_pd_en = ((sleep_flags) & RTC_SLEEP_PD_RTC_FAST_MEM) ? 1 : 0, \
  .rtc_slowmem_pd_en = ((sleep_flags) & RTC_SLEEP_PD_RTC_SLOW_MEM) ? 1 : 0, \
  .rtc_peri_pd_en = ((sleep_flags) & RTC_SLEEP_PD_RTC_PERIPH) ? 1 : 0, \
  .wifi_pd_en = ((sleep_flags) & RTC_SLEEP_PD_WIFI) ? 1 : 0, \
  .bt_pd_en = ((sleep_flags) & RTC_SLEEP_PD_BT) ? 1 : 0, \
  .cpu_pd_en = ((sleep_flags) & RTC_SLEEP_PD_CPU) ? 1 : 0, \
  .dig_peri_pd_en = ((sleep_flags) & RTC_SLEEP_PD_DIG_PERIPH) ? 1 : 0, \
  .deep_slp = ((sleep_flags) & RTC_SLEEP_PD_DIG) ? 1 : 0, \
  .wdt_flashboot_mod_en = 0, \
  .dig_dbias_wak = RTC_CNTL_DBIAS_1V10, \
  .dig_dbias_slp = RTC_CNTL_DBIAS_SLP, \
  .rtc_dbias_wak = RTC_CNTL_DBIAS_1V10, \
  .rtc_dbias_slp = RTC_CNTL_DBIAS_SLP, \
  .vddsdio_pd_en = ((sleep_flags) & RTC_SLEEP_PD_VDDSDIO) ? 1 : 0, \
  .deep_slp_reject = 1, \
  .light_slp_reject = 1 \
};

#define X32K_CONFIG_DEFAULT() { \
  .dac = 3, \
  .dres = 3, \
  .dgm = 3, \
  .dbuf = 1, \
}

/* Initializer for struct esp32c3_rtc_sleep_pu_config_s
 * which sets all flags to the same value
 */

#define RTC_SLEEP_PU_CONFIG_ALL(val) {\
  .dig_fpu = (val), \
  .rtc_fpu = (val), \
  .cpu_fpu = (val), \
  .i2s_fpu = (val), \
  .bb_fpu = (val), \
  .nrx_fpu = (val), \
  .fe_fpu = (val), \
  .sram_fpu = (val), \
  .rom_ram_fpu = (val), \
}

#define RTC_INIT_CONFIG_DEFAULT() { \
  .wifi_powerup_cycles = OTHER_BLOCKS_POWERUP, \
  .wifi_wait_cycles = OTHER_BLOCKS_WAIT, \
  .bt_powerup_cycles = OTHER_BLOCKS_POWERUP, \
  .bt_wait_cycles = OTHER_BLOCKS_WAIT, \
  .cpu_top_powerup_cycles = OTHER_BLOCKS_POWERUP, \
  .cpu_top_wait_cycles = OTHER_BLOCKS_WAIT, \
  .dg_wrap_powerup_cycles = OTHER_BLOCKS_POWERUP, \
  .dg_wrap_wait_cycles = OTHER_BLOCKS_WAIT, \
  .dg_peri_powerup_cycles = OTHER_BLOCKS_POWERUP, \
  .dg_peri_wait_cycles = OTHER_BLOCKS_WAIT, \
}

/* Default initializer of struct esp32c3_rtc_config_s.
 * This initializer sets all fields to "reasonable" values
 * (e.g. suggested for production use).
 */

#define RTC_CONFIG_DEFAULT() {\
  .ck8m_wait = RTC_CNTL_CK8M_WAIT_DEFAULT, \
  .xtal_wait = RTC_CNTL_XTL_BUF_WAIT_DEFAULT, \
  .pll_wait  = RTC_CNTL_PLL_BUF_WAIT_DEFAULT, \
  .clkctl_init = 1, \
  .pwrctl_init = 1, \
  .rtc_dboost_fpd = 1, \
  .xtal_fpu = 0, \
  .bbpll_fpu = 0, \
  .cpu_waiti_clk_gate = 1, \
  .cali_ocode = 0\
}

#ifdef CONFIG_RTC_DRIVER
/* The magic data for the struct esp32c3_rtc_backup_s that is in RTC slow
 * memory.
 */

#  define MAGIC_RTC_SAVE (UINT64_C(0x11223344556677))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This is almost the same as esp32c3_rtc_slow_freq_e, except that we define
 * an extra enum member for the external 32k oscillator. For convenience,
 * lower 2 bits should correspond to esp32c3_rtc_slow_freq_e values.
 */

enum esp32c3_slow_clk_sel_e
{
  /* Internal 90 kHz RC oscillator */

  SLOW_CLK_RTC = RTC_SLOW_FREQ_RTC,

  /* External 32 kHz XTAL */

  SLOW_CLK_32K_XTAL = RTC_SLOW_FREQ_32K_XTAL,

  /* Internal 8 MHz RC oscillator, divided by 256 */

  SLOW_CLK_8MD256 = RTC_SLOW_FREQ_8MD256,

  /* External 32k oscillator connected to 32K_XP pin */

  SLOW_CLK_32K_EXT_OSC = RTC_SLOW_FREQ_32K_XTAL | EXT_OSC_FLAG
};

/* RTC FAST_CLK frequency values */

enum esp32c3_rtc_fast_freq_e
{
  RTC_FAST_FREQ_XTALD4 = 0,   /* Main XTAL, divided by 4 */
  RTC_FAST_FREQ_8M = 1,       /* Internal 8 MHz RC oscillator */
};

struct esp32c3_x32k_config_s
{
  uint32_t dac : 6;
  uint32_t dres : 3;
  uint32_t dgm : 3;
  uint32_t dbuf: 1;
};

/* Power down flags for rtc_sleep_pd function */

struct esp32c3_rtc_sleep_pu_config_s
{
  uint32_t dig_fpu : 1;     /* Set to 1 to power UP digital part in sleep */
  uint32_t rtc_fpu : 1;     /* Set to 1 to power UP RTC memories in sleep */
  uint32_t cpu_fpu : 1;     /* Set to 1 to power UP digital memories and
                             * CPU in sleep
                             */
  uint32_t i2s_fpu : 1;     /* Set to 1 to power UP I2S in sleep */
  uint32_t bb_fpu : 1;      /* Set to 1 to power UP Wi-Fi in sleep */
  uint32_t nrx_fpu : 1;     /* Set to 1 to power UP Wi-Fi in sleep */
  uint32_t fe_fpu : 1;      /* Set to 1 to power UP Wi-Fi in sleep */
  uint32_t sram_fpu : 1;    /* Set to 1 to power UP SRAM in sleep */
  uint32_t rom_ram_fpu : 1; /* Set to 1 to power UP ROM/IRAM0_DRAM0 in sleep */
};

struct esp32c3_rtc_init_config_s
{
  uint16_t wifi_powerup_cycles : 7;
  uint16_t wifi_wait_cycles : 9;
  uint16_t bt_powerup_cycles : 7;
  uint16_t bt_wait_cycles : 9;
  uint16_t cpu_top_powerup_cycles : 7;
  uint16_t cpu_top_wait_cycles : 9;
  uint16_t dg_wrap_powerup_cycles : 7;
  uint16_t dg_wrap_wait_cycles : 9;
  uint16_t dg_peri_powerup_cycles : 7;
  uint16_t dg_peri_wait_cycles : 9;
};

/* RTC power and clock control initialization settings */

struct esp32c3_rtc_priv_s
{
    uint32_t ck8m_wait : 8;       /* Number of rtc_fast_clk cycles to
                                   * wait for 8M clock to be ready
                                   */
    uint32_t xtal_wait : 8;       /* Number of rtc_fast_clk cycles to
                                   * wait for XTAL clock to be ready
                                   */
    uint32_t pll_wait : 8;        /* Number of rtc_fast_clk cycles to
                                   * wait for PLL to be ready
                                   */
    uint32_t clkctl_init : 1;     /* Perform clock control related initialization */
    uint32_t pwrctl_init : 1;     /* Perform power control related initialization */
    uint32_t rtc_dboost_fpd : 1;  /* Force power down RTC_DBOOST */
    uint32_t xtal_fpu : 1;
    uint32_t bbpll_fpu : 1;
    uint32_t cpu_waiti_clk_gate : 1;
    uint32_t cali_ocode : 1;      /* Calibrate Ocode to make bangap voltage more precise. */
};

/* sleep configuration for rtc_sleep_init function */

struct esp32c3_rtc_sleep_config_s
{
  uint32_t lslp_mem_inf_fpu : 1;       /* force normal voltage in sleep mode */
  uint32_t rtc_mem_inf_follow_cpu : 1; /* keep low voltage in sleep mode */
  uint32_t rtc_fastmem_pd_en : 1;      /* power down RTC fast memory */
  uint32_t rtc_slowmem_pd_en : 1;      /* power down RTC slow memory */
  uint32_t rtc_peri_pd_en : 1;         /* power down RTC peripherals */
  uint32_t wifi_pd_en : 1;             /* power down Wi-Fi */
  uint32_t bt_pd_en : 1;               /* power down BT */
  uint32_t cpu_pd_en : 1;              /* power down CPU, but not
                                        * restart when lightsleep.
                                        */
  uint32_t dig_peri_pd_en : 1;         /* power down digital peripherals */
  uint32_t deep_slp : 1;               /* power down digital domain */
  uint32_t wdt_flashboot_mod_en : 1;   /* enable WDT flashboot mode */
  uint32_t dig_dbias_wak : 5;          /* set bias for digital domain,
                                        * in active mode
                                        */
  uint32_t dig_dbias_slp : 5;          /* set bias for digital domain,
                                        * in sleep mode
                                        */
  uint32_t rtc_dbias_wak : 5;          /* set bias for RTC domain, in active mode */
  uint32_t rtc_dbias_slp : 5;          /* set bias for RTC domain, in sleep mode */
  uint32_t vddsdio_pd_en : 1;          /* power down VDDSDIO regulator */
  uint32_t deep_slp_reject : 1;
  uint32_t light_slp_reject : 1;
};

#ifdef CONFIG_RTC_DRIVER

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

struct esp32c3_rtc_backup_s
{
  uint64_t magic;
  int64_t  offset;              /* Offset time from RTC HW value */
  int64_t  reserved0;
};

#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_RTC_DRIVER

/* Callback to use when the alarm expires */

#ifdef CONFIG_RTC_ALARM
static struct alm_cbinfo_s g_alarmcb[RTC_ALARM_LAST];
#endif

static RTC_DATA_ATTR struct esp32c3_rtc_backup_s rtc_saved_data;

/* Saved data for persistent RTC time */

static struct esp32c3_rtc_backup_s *g_rtc_save;
static bool g_rt_timer_enabled = false;

#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void IRAM_ATTR esp32c3_rtc_sleep_pu(
                      struct esp32c3_rtc_sleep_pu_config_s cfg);
static inline bool esp32c3_clk_val_is_valid(uint32_t val);
static uint32_t IRAM_ATTR esp32c3_rtc_clk_cal_internal(
                enum esp32c3_rtc_cal_sel_e cal_clk, uint32_t slowclk_cycles);
static void IRAM_ATTR esp32c3_rtc_clk_32k_enable(bool enable);
static void IRAM_ATTR esp32c3_rtc_clk_8m_enable(
                                bool clk_8m_en, bool d256_en);
static uint32_t IRAM_ATTR esp32c3_rtc_clk_slow_freq_get_hz(void);
static void IRAM_ATTR esp32c3_select_rtc_slow_clk(
                      enum esp32c3_slow_clk_sel_e slow_clk);
static void IRAM_ATTR esp32c3_rtc_clk_fast_freq_set(
                      enum esp32c3_rtc_fast_freq_e fast_freq);
static uint32_t IRAM_ATTR esp32c3_rtc_sleep_finish(uint32_t
                                          lslp_mem_inf_fpu);
static void IRAM_ATTR esp32c3_rtc_clk_apb_freq_update(uint32_t apb_freq);
static void IRAM_ATTR esp32c3_wait_dig_dbias_valid(uint64_t rtc_cycles);
static void IRAM_ATTR esp32c3_rtc_update_to_xtal(int freq, int div);
static void IRAM_ATTR esp32c3_rtc_clk_bbpll_disable(void);
static void IRAM_ATTR esp32c3_rtc_clk_set_xtal_wait(void);
static void IRAM_ATTR esp32c3_rtc_bbpll_configure(
                     enum esp32c3_rtc_xtal_freq_e xtal_freq, int pll_freq);
static void IRAM_ATTR esp32c3_rtc_clk_cpu_freq_to_8m(void);
static void IRAM_ATTR esp32c3_rtc_clk_cpu_freq_to_pll_mhz(
                                             int cpu_freq_mhz);

#ifdef CONFIG_RTC_DRIVER
static void IRAM_ATTR esp32c3_rt_cb_handler(void *arg);
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_RTC_DRIVER
volatile bool g_rtc_enabled = false;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Pauses execution for us microseconds */

extern void esp_rom_delay_us(uint32_t us);

/* Set the real CPU ticks per us to the ets, so that ets_delay_us
 * will be accurate. Call this function when CPU frequency is changed.
 */

extern void ets_update_cpu_frequency(uint32_t ticks_per_us);

/****************************************************************************
 * Name: esp32c3_rtc_sleep_pu
 *
 * Description:
 *   Configure whether certain peripherals are powered down in deep sleep.
 *
 * Input Parameters:
 *   cfg - power down flags as esp32c3_rtc_sleep_pu_config_s structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void IRAM_ATTR esp32c3_rtc_sleep_pu(
                 struct esp32c3_rtc_sleep_pu_config_s cfg)
{
  REG_SET_FIELD(RTC_CNTL_DIG_PWC_REG,
                RTC_CNTL_LSLP_MEM_FORCE_PU, cfg.dig_fpu);
  REG_SET_FIELD(RTC_CNTL_PWC_REG, RTC_CNTL_FASTMEM_FORCE_LPU, cfg.rtc_fpu);
  REG_SET_FIELD(APB_CTRL_FRONT_END_MEM_PD_REG,
                APB_CTRL_DC_MEM_FORCE_PU, cfg.fe_fpu);
  REG_SET_FIELD(APB_CTRL_FRONT_END_MEM_PD_REG,
                APB_CTRL_PBUS_MEM_FORCE_PU, cfg.fe_fpu);
  REG_SET_FIELD(APB_CTRL_FRONT_END_MEM_PD_REG,
                APB_CTRL_AGC_MEM_FORCE_PU, cfg.fe_fpu);
  REG_SET_FIELD(BBPD_CTRL, BB_FFT_FORCE_PU, cfg.bb_fpu);
  REG_SET_FIELD(BBPD_CTRL, BB_DC_EST_FORCE_PU, cfg.bb_fpu);
  REG_SET_FIELD(NRXPD_CTRL, NRX_RX_ROT_FORCE_PU, cfg.nrx_fpu);
  REG_SET_FIELD(NRXPD_CTRL, NRX_VIT_FORCE_PU, cfg.nrx_fpu);
  REG_SET_FIELD(NRXPD_CTRL, NRX_DEMAP_FORCE_PU, cfg.nrx_fpu);
  REG_SET_FIELD(FE_GEN_CTRL, FE_IQ_EST_FORCE_PU, cfg.fe_fpu);
  REG_SET_FIELD(FE2_TX_INTERP_CTRL, FE2_TX_INF_FORCE_PU, cfg.fe_fpu);
  if (cfg.sram_fpu)
    {
      REG_SET_FIELD(APB_CTRL_MEM_POWER_UP_REG,
                    APB_CTRL_SRAM_POWER_UP, APB_CTRL_SRAM_POWER_UP);
    }
  else
    {
      REG_SET_FIELD(APB_CTRL_MEM_POWER_UP_REG,
                    APB_CTRL_SRAM_POWER_UP, 0);
    }

  if (cfg.rom_ram_fpu)
    {
      REG_SET_FIELD(APB_CTRL_MEM_POWER_UP_REG,
                    APB_CTRL_ROM_POWER_UP, APB_CTRL_ROM_POWER_UP);
    }
  else
    {
      REG_SET_FIELD(APB_CTRL_MEM_POWER_UP_REG,
                    APB_CTRL_ROM_POWER_UP, 0);
    }
}

/****************************************************************************
 * Name: esp32c3_rtc_clk_fast_freq_set
 *
 * Description:
 *   Select source for RTC_FAST_CLK.
 *
 * Input Parameters:
 *   fast_freq - Clock source (one of enum esp32c3_rtc_fast_freq_e values)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void IRAM_ATTR esp32c3_rtc_clk_fast_freq_set(
                      enum esp32c3_rtc_fast_freq_e fast_freq)
{
  REG_SET_FIELD(RTC_CNTL_CLK_CONF_REG,
                RTC_CNTL_FAST_CLK_RTC_SEL, fast_freq);
  esp_rom_delay_us(DELAY_FAST_CLK_SWITCH);
}

/****************************************************************************
 * Name: esp32c3_clk_val_is_valid
 *
 * Description:
 *   Values of RTC_XTAL_FREQ_REG and RTC_APB_FREQ_REG are
 *   stored as two copies in lower and upper 16-bit halves.
 *   These are the routines to work with such a representation.
 *
 * Input Parameters:
 *   val - register value
 *
 * Returned Value:
 *   true:  Valid register value.
 *   false: Invalid register value.
 *
 ****************************************************************************/

static inline bool esp32c3_clk_val_is_valid(uint32_t val)
{
  return (val & 0xffff) == ((val >> 16) & 0xffff)
                        && val != 0 && val != UINT32_MAX;
}

/****************************************************************************
 * Name: esp32c3_rtc_clk_cal_internal
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

static uint32_t IRAM_ATTR esp32c3_rtc_clk_cal_internal(
                enum esp32c3_rtc_cal_sel_e cal_clk, uint32_t slowclk_cycles)
{
  uint32_t cal_val;
  uint32_t expected_freq;
  uint32_t us_time_estimate;
  int dig_32k_xtal_state;
  enum esp32c3_rtc_slow_freq_e slow_freq;
  if (cal_clk == RTC_CAL_RTC_MUX)
    {
      slow_freq = REG_GET_FIELD(RTC_CNTL_CLK_CONF_REG,
                                RTC_CNTL_ANA_CLK_RTC_SEL);
      if (slow_freq == RTC_SLOW_FREQ_32K_XTAL)
        {
          cal_clk = RTC_CAL_32K_XTAL;
        }
      else if (slow_freq == RTC_SLOW_FREQ_8MD256)
        {
          cal_clk = RTC_CAL_8MD256;
        }
    }

  /* Enable requested clock (150k clock is always on) */

  dig_32k_xtal_state = REG_GET_FIELD(RTC_CNTL_CLK_CONF_REG,
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

  /* There may be another calibration process already running during we
   * call this function, so we should wait the last process is done.
   */

  if ((getreg32(TIMG_RTCCALICFG2_REG(0)) & TIMG_RTC_CALI_TIMEOUT) == 0)
    {
      if (getreg32(TIMG_RTCCALICFG_REG(0)) & TIMG_RTC_CALI_START_CYCLING)
        {
          while ((getreg32(TIMG_RTCCALICFG_REG(0)) &
                  TIMG_RTC_CALI_RDY) == 0);
        }
    }

  modifyreg32(TIMG_RTCCALICFG_REG(0), TIMG_RTC_CALI_START_CYCLING, 0);
  REG_SET_FIELD(TIMG_RTCCALICFG_REG(0), TIMG_RTC_CALI_MAX, slowclk_cycles);

  /* Set timeout reg and expect time delay */

  if (cal_clk == RTC_CAL_32K_XTAL)
    {
      REG_SET_FIELD(TIMG_RTCCALICFG2_REG(0), TIMG_RTC_CALI_TIMEOUT_THRES,
                    RTC_SLOW_CLK_X32K_CAL_TIMEOUT_THRES(slowclk_cycles));
      expected_freq = RTC_SLOW_CLK_FREQ_32K;
    }
  else if (cal_clk == RTC_CAL_8MD256)
    {
      REG_SET_FIELD(TIMG_RTCCALICFG2_REG(0), TIMG_RTC_CALI_TIMEOUT_THRES,
                    RTC_SLOW_CLK_8MD256_CAL_TIMEOUT_THRES(slowclk_cycles));
      expected_freq = RTC_SLOW_CLK_FREQ_8MD256;
    }
  else
    {
      REG_SET_FIELD(TIMG_RTCCALICFG2_REG(0), TIMG_RTC_CALI_TIMEOUT_THRES,
                    RTC_SLOW_CLK_150K_CAL_TIMEOUT_THRES(slowclk_cycles));
      expected_freq = RTC_SLOW_CLK_FREQ_90K;
    }

  us_time_estimate = (uint32_t) (((uint64_t) slowclk_cycles) *
                                          MHZ / expected_freq);

  /* Start calibration */

  modifyreg32(TIMG_RTCCALICFG_REG(0), TIMG_RTC_CALI_START, 0);
  modifyreg32(TIMG_RTCCALICFG_REG(0), 0, TIMG_RTC_CALI_START);

  /* Wait for calibration to finish up to another us_time_estimate */

  esp_rom_delay_us(us_time_estimate);
  while (1)
    {
      if (getreg32(TIMG_RTCCALICFG_REG(0)) & TIMG_RTC_CALI_RDY)
        {
          cal_val = REG_GET_FIELD(TIMG_RTCCALICFG1_REG(0),
                                  TIMG_RTC_CALI_VALUE);
          break;
        }

      if (getreg32(TIMG_RTCCALICFG2_REG(0)) & TIMG_RTC_CALI_TIMEOUT)
        {
          cal_val = 0;
          break;
        }
    }

  modifyreg32(TIMG_RTCCALICFG_REG(0), TIMG_RTC_CALI_START, 0);
  REG_SET_FIELD(RTC_CNTL_CLK_CONF_REG,
                RTC_CNTL_DIG_XTAL32K_EN, dig_32k_xtal_state);
  if (cal_clk == RTC_CAL_8MD256)
    {
      modifyreg32(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_DIG_CLK8M_D256_EN, 0);
    }

  return cal_val;
}

/****************************************************************************
 * Name: esp32c3_rtc_clk_32k_enable
 *
 * Description:
 *   Enable or disable 32 kHz XTAL oscillator
 *
 * Input Parameters:
 *   enable - true to enable, false to disable
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void IRAM_ATTR esp32c3_rtc_clk_32k_enable(bool enable)
{
  struct esp32c3_x32k_config_s cfg = X32K_CONFIG_DEFAULT();
  if (enable)
    {
      REG_SET_FIELD(RTC_CNTL_EXT_XTL_CONF_REG,
                    RTC_CNTL_DAC_XTAL_32K, cfg.dac);
      REG_SET_FIELD(RTC_CNTL_EXT_XTL_CONF_REG,
                    RTC_CNTL_DRES_XTAL_32K, cfg.dres);
      REG_SET_FIELD(RTC_CNTL_EXT_XTL_CONF_REG,
                    RTC_CNTL_DGM_XTAL_32K, cfg.dgm);
      REG_SET_FIELD(RTC_CNTL_EXT_XTL_CONF_REG,
                    RTC_CNTL_DBUF_XTAL_32K, cfg.dbuf);
      modifyreg32(RTC_CNTL_EXT_XTL_CONF_REG, 0,
                  RTC_CNTL_XPD_XTAL_32K);
    }
  else
    {
      modifyreg32(RTC_CNTL_EXT_XTL_CONF_REG, RTC_CNTL_XPD_XTAL_32K,
                  RTC_CNTL_XTAL32K_XPD_FORCE);
    }
}

/****************************************************************************
 * Name: esp32c3_rtc_clk_8m_enable
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

static void IRAM_ATTR esp32c3_rtc_clk_8m_enable(bool clk_8m_en, bool d256_en)
{
  if (clk_8m_en)
    {
      modifyreg32(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_ENB_CK8M, 0);

      /* no need to wait once enabled by software */

      REG_SET_FIELD(RTC_CNTL_TIMER1_REG, RTC_CNTL_CK8M_WAIT,
                    RTC_CK8M_ENABLE_WAIT_DEFAULT);
      esp_rom_delay_us(DELAY_8M_ENABLE);
    }
  else
    {
      modifyreg32(RTC_CNTL_CLK_CONF_REG, 0, RTC_CNTL_ENB_CK8M);
      REG_SET_FIELD(RTC_CNTL_TIMER1_REG, RTC_CNTL_CK8M_WAIT,
                    RTC_CNTL_CK8M_WAIT_DEFAULT);
    }

  /* d256 should be independent configured with 8M
   * Maybe we can split this function into 8m and dmd256
   */

  if (d256_en)
    {
      modifyreg32(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_ENB_CK8M_DIV, 0);
    }
  else
    {
      modifyreg32(RTC_CNTL_CLK_CONF_REG, 0, RTC_CNTL_ENB_CK8M_DIV);
    }
}

/****************************************************************************
 * Name: esp32c3_rtc_clk_slow_freq_get_hz
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

static uint32_t IRAM_ATTR esp32c3_rtc_clk_slow_freq_get_hz(void)
{
  enum esp32c3_rtc_slow_freq_e slow_clk_freq =
                               esp32c3_rtc_clk_slow_freq_get();
  switch (slow_clk_freq)
    {
      case RTC_SLOW_FREQ_RTC:
        return RTC_SLOW_CLK_FREQ_90K;

      case RTC_SLOW_FREQ_32K_XTAL:
        return RTC_SLOW_CLK_FREQ_32K;

      case RTC_SLOW_FREQ_8MD256:
        return RTC_SLOW_CLK_FREQ_8MD256;
    }

  return 0;
}

/****************************************************************************
 * Name: esp32c3_select_rtc_slow_clk
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

static void IRAM_ATTR esp32c3_select_rtc_slow_clk(
                      enum esp32c3_slow_clk_sel_e slow_clk)
{
  uint32_t cal_val = 0;
  int retry_32k_xtal = 3;
  uint64_t cal_dividend;
  enum esp32c3_rtc_slow_freq_e rtc_slow_freq = slow_clk &
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

          if (slow_clk == SLOW_CLK_32K_XTAL ||
              slow_clk == SLOW_CLK_32K_EXT_OSC)
            {
              esp32c3_rtc_clk_32k_enable(true);
            }

          if (SLOW_CLK_CAL_CYCLES > 0)
            {
              cal_val = esp32c3_rtc_clk_cal(RTC_CAL_32K_XTAL,
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
          esp32c3_rtc_clk_8m_enable(true, true);
        }

      esp32c3_rtc_clk_slow_freq_set(rtc_slow_freq);
      if (SLOW_CLK_CAL_CYCLES > 0)
        {
          /* 32k XTAL oscillator has some frequency drift at startup. Improve
           * calibration routine to wait until the frequency is stable.
           */

          cal_val = esp32c3_rtc_clk_cal(RTC_CAL_RTC_MUX,
                                        SLOW_CLK_CAL_CYCLES);
        }
      else
        {
          cal_dividend = (1ULL << RTC_CLK_CAL_FRACT) * 1000000ULL;
          cal_val = (uint32_t) (cal_dividend /
                                esp32c3_rtc_clk_slow_freq_get_hz());
        }
    }
  while (cal_val == 0);

  putreg32((uint32_t)cal_val, RTC_SLOW_CLK_CAL_REG);
}

/****************************************************************************
 * Name: esp32c3_rtc_sleep_finish
 *
 * Description:
 *   Wake up from sleep.
 *
 * Input Parameters:
 *   lslp_mem_inf_fpu - If non-zero then the low power config
 *                      is restored immediately on wake
 *
 * Returned Value:
 *   non-zero if sleep was rejected by hardware
 *
 ****************************************************************************/

static uint32_t IRAM_ATTR esp32c3_rtc_sleep_finish(uint32_t lslp_mem_inf_fpu)
{
  uint32_t reject;
  struct esp32c3_rtc_sleep_pu_config_s pu_cfg = RTC_SLEEP_PU_CONFIG_ALL(1);

  /* In deep sleep mode, we never get here */

  reject = REG_GET_FIELD(RTC_CNTL_INT_RAW_REG, RTC_CNTL_SLP_REJECT_INT_RAW);
  modifyreg32(RTC_CNTL_INT_CLR_REG, 0,
              RTC_CNTL_SLP_REJECT_INT_CLR | RTC_CNTL_SLP_WAKEUP_INT_CLR);

  /* restore config if it is a light sleep */

  if (lslp_mem_inf_fpu)
    {
      esp32c3_rtc_sleep_pu(pu_cfg);
    }

  return reject;
}

/****************************************************************************
 * Name: esp32c3_rtc_clk_apb_freq_update
 *
 * Description:
 *   Store new APB frequency value into RTC_APB_FREQ_REG
 *
 * Input Parameters:
 *   apb_freq - New APB frequency, in Hz
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void IRAM_ATTR esp32c3_rtc_clk_apb_freq_update(uint32_t apb_freq)
{
  uint32_t val = ((apb_freq >> 12) & UINT16_MAX) |
                 (((apb_freq >> 12) & UINT16_MAX) << 16);
  putreg32(val, RTC_APB_FREQ_REG);
}

/****************************************************************************
 * Name: esp32c3_wait_dig_dbias_valid
 *
 * Description:
 *   Wait digtial dbias valid
 *
 * Input Parameters:
 *   rtc_cycles - RTC count
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void IRAM_ATTR esp32c3_wait_dig_dbias_valid(uint64_t rtc_cycles)
{
  enum esp32c3_rtc_slow_freq_e slow_clk_freq =
                                    esp32c3_rtc_clk_slow_freq_get();
  enum esp32c3_rtc_cal_sel_e cal_clk = RTC_CAL_RTC_MUX;
  if (slow_clk_freq == RTC_SLOW_FREQ_32K_XTAL)
    {
      cal_clk = RTC_CAL_32K_XTAL;
    }
  else if (slow_clk_freq == RTC_SLOW_FREQ_8MD256)
    {
      cal_clk = RTC_CAL_8MD256;
    }

  esp32c3_rtc_clk_cal(cal_clk, rtc_cycles);
}

/****************************************************************************
 * Name: esp32c3_rtc_update_to_xtal
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

static void IRAM_ATTR esp32c3_rtc_update_to_xtal(int freq, int div)
{
  int origin_soc_clk = REG_GET_FIELD(SYSTEM_SYSCLK_CONF_REG,
                                     SYSTEM_SOC_CLK_SEL);
  int origin_div_cnt = REG_GET_FIELD(SYSTEM_SYSCLK_CONF_REG,
                                     SYSTEM_PRE_DIV_CNT);
  ets_update_cpu_frequency(freq);

  /* lower the voltage */

  if (freq <= 2)
    {
      REGI2C_WRITE_MASK(I2C_DIG_REG,
                        I2C_DIG_REG_EXT_DIG_DREG, DIG_DBIAS_2M);
    }
  else
    {
      REGI2C_WRITE_MASK(I2C_DIG_REG,
                        I2C_DIG_REG_EXT_DIG_DREG, DIG_DBIAS_XTAL);
    }

  if ((DPORT_SOC_CLK_SEL_XTAL == origin_soc_clk)
                        && (origin_div_cnt > 0))
    {
      esp32c3_wait_dig_dbias_valid(2);
    }

  /* Set divider from XTAL to APB clock.
   * Need to set divider to 1 (reg. value 0) first.
   */

  REG_SET_FIELD(SYSTEM_SYSCLK_CONF_REG, SYSTEM_PRE_DIV_CNT, 0);
  REG_SET_FIELD(SYSTEM_SYSCLK_CONF_REG, SYSTEM_PRE_DIV_CNT, div - 1);

  /* No need to adjust the REF_TICK.
   * Switch clock source.
   */

  REG_SET_FIELD(SYSTEM_SYSCLK_CONF_REG,
                SYSTEM_SOC_CLK_SEL, DPORT_SOC_CLK_SEL_XTAL);
  esp32c3_rtc_clk_apb_freq_update(freq * MHZ);
}

/****************************************************************************
 * Name: esp32c3_rtc_clk_bbpll_disable
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

static void IRAM_ATTR esp32c3_rtc_clk_bbpll_disable(void)
{
  modifyreg32(RTC_CNTL_OPTIONS0_REG, 0, RTC_CNTL_BB_I2C_FORCE_PD |
              RTC_CNTL_BBPLL_FORCE_PD | RTC_CNTL_BBPLL_I2C_FORCE_PD);
}

/****************************************************************************
 * Name: esp32c3_rtc_clk_set_xtal_wait
 *
 * Description:
 *   Set XTAL wait cycles by RTC slow clock's period
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void IRAM_ATTR esp32c3_rtc_clk_set_xtal_wait(void)
{
  uint32_t slow_clk_period;

  /* the `xtal_wait` time need 1ms, so we need calibrate slow clk period,
   * and `RTC_CNTL_XTL_BUF_WAIT` depend on it.
   */

  uint32_t xtal_wait_1ms = 100;
  enum esp32c3_rtc_slow_freq_e slow_clk_freq =
                                      esp32c3_rtc_clk_slow_freq_get();
  enum esp32c3_rtc_cal_sel_e cal_clk = RTC_CAL_RTC_MUX;
  if (slow_clk_freq == RTC_SLOW_FREQ_32K_XTAL)
    {
      cal_clk = RTC_CAL_32K_XTAL;
    }
  else if (slow_clk_freq == RTC_SLOW_FREQ_8MD256)
    {
      cal_clk  = RTC_CAL_8MD256;
    }

  slow_clk_period = esp32c3_rtc_clk_cal(cal_clk, 2000);
  if (slow_clk_period)
    {
      xtal_wait_1ms = (1000 << RTC_CLK_CAL_FRACT) / slow_clk_period;
    }

  REG_SET_FIELD(RTC_CNTL_TIMER1_REG, RTC_CNTL_XTL_BUF_WAIT, xtal_wait_1ms);
}

/****************************************************************************
 * Name: esp32c3_rtc_bbpll_configure
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

static void IRAM_ATTR esp32c3_rtc_bbpll_configure(
                      enum esp32c3_rtc_xtal_freq_e xtal_freq, int pll_freq)
{
  uint8_t div_ref;
  uint8_t div7_0;
  uint8_t dr1;
  uint8_t dr3;
  uint8_t dchgp;
  uint8_t dcur;
  uint8_t dbias;
  uint8_t i2c_bbpll_lref;
  uint8_t i2c_bbpll_div_7_0;
  uint8_t i2c_bbpll_dcur;

  modifyreg32(I2C_MST_ANA_CONF0_REG,
              I2C_MST_BBPLL_STOP_FORCE_HIGH,  0);
  modifyreg32(I2C_MST_ANA_CONF0_REG, 0, I2C_MST_BBPLL_STOP_FORCE_LOW);
  if (pll_freq == RTC_PLL_FREQ_480M)
    {
      /* Set this register to let the digital part know 480M PLL is used */

      modifyreg32(SYSTEM_CPU_PER_CONF_REG, 0, SYSTEM_PLL_FREQ_SEL);

      /* Configure 480M PLL */

      switch (xtal_freq)
        {
          case RTC_XTAL_FREQ_40M:
            {
              div_ref = 0;
              div7_0 = 8;
              dr1 = 0;
              dr3 = 0;
              dchgp = 5;
              dcur = 3;
              dbias = 2;
            }
            break;

          case RTC_XTAL_FREQ_32M:
            {
              div_ref = 1;
              div7_0 = 26;
              dr1 = 1;
              dr3 = 1;
              dchgp = 4;
              dcur = 0;
              dbias = 2;
            }
            break;

          default:
            {
              div_ref = 0;
              div7_0 = 8;
              dr1 = 0;
              dr3 = 0;
              dchgp = 5;
              dcur = 3;
              dbias = 2;
            }
            break;
        }

      REGI2C_WRITE(I2C_BBPLL, I2C_BBPLL_MODE_HF, 0x6b);
    }
  else
    {
      /* Clear this register to let the digital part know 320M PLL is used */

      modifyreg32(SYSTEM_CPU_PER_CONF_REG, SYSTEM_PLL_FREQ_SEL,  0);

      /* Configure 320M PLL */

      switch (xtal_freq)
        {
          case RTC_XTAL_FREQ_40M:
            {
              div_ref = 0;
              div7_0 = 4;
              dr1 = 0;
              dr3 = 0;
              dchgp = 5;
              dcur = 3;
              dbias = 2;
            }
            break;

          case RTC_XTAL_FREQ_32M:
            {
              div_ref = 1;
              div7_0 = 6;
              dr1 = 0;
              dr3 = 0;
              dchgp = 5;
              dcur = 3;
              dbias = 2;
            }
            break;

          default:
            {
              div_ref = 0;
              div7_0 = 4;
              dr1 = 0;
              dr3 = 0;
              dchgp = 5;
              dcur = 3;
              dbias = 2;
            }
            break;
        }

      REGI2C_WRITE(I2C_BBPLL, I2C_BBPLL_MODE_HF, 0x69);
    }

  i2c_bbpll_lref  = (dchgp << I2C_BBPLL_OC_DCHGP_LSB) | (div_ref);
  i2c_bbpll_div_7_0 = div7_0;
  i2c_bbpll_dcur = (2 << I2C_BBPLL_OC_DLREF_SEL_LSB)
                    | (1 << I2C_BBPLL_OC_DHREF_SEL_LSB) | dcur;
  REGI2C_WRITE(I2C_BBPLL, I2C_BBPLL_OC_REF_DIV, i2c_bbpll_lref);
  REGI2C_WRITE(I2C_BBPLL, I2C_BBPLL_OC_DIV_7_0, i2c_bbpll_div_7_0);
  REGI2C_WRITE_MASK(I2C_BBPLL, I2C_BBPLL_OC_DR1, dr1);
  REGI2C_WRITE_MASK(I2C_BBPLL, I2C_BBPLL_OC_DR3, dr3);
  REGI2C_WRITE(I2C_BBPLL, I2C_BBPLL_OC_DCUR, i2c_bbpll_dcur);
  REGI2C_WRITE_MASK(I2C_BBPLL, I2C_BBPLL_OC_VCO_DBIAS, dbias);
}

/****************************************************************************
 * Name: esp32c3_rtc_clk_cpu_freq_to_8m
 *
 * Description:
 *   Switch CPU frequency to 8 Mhz.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void IRAM_ATTR esp32c3_rtc_clk_cpu_freq_to_8m(void)
{
  int origin_soc_clk = REG_GET_FIELD(SYSTEM_SYSCLK_CONF_REG,
                                        SYSTEM_SOC_CLK_SEL);
  int origin_div_cnt = REG_GET_FIELD(SYSTEM_SYSCLK_CONF_REG,
                                        SYSTEM_PRE_DIV_CNT);
  ets_update_cpu_frequency(8);
  REGI2C_WRITE_MASK(I2C_DIG_REG, I2C_DIG_REG_EXT_DIG_DREG,
                                          DIG_DBIAS_XTAL);
  if ((DPORT_SOC_CLK_SEL_XTAL == origin_soc_clk)
                            && (origin_div_cnt > 4))
    {
      esp32c3_wait_dig_dbias_valid(2);
    }

  REG_SET_FIELD(SYSTEM_SYSCLK_CONF_REG, SYSTEM_PRE_DIV_CNT, 0);
  REG_SET_FIELD(SYSTEM_SYSCLK_CONF_REG, SYSTEM_SOC_CLK_SEL,
                DPORT_SOC_CLK_SEL_8M);
  esp32c3_rtc_clk_apb_freq_update(RTC_FAST_CLK_FREQ_8M);
}

/****************************************************************************
 * Name: esp32c3_rtc_clk_cpu_freq_to_pll_mhz
 *
 * Description:
 *   Switch to one of PLL-based frequencies.
 *
 * Input Parameters:
 *   cpu_freq_mhz - CPU frequency
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void IRAM_ATTR esp32c3_rtc_clk_cpu_freq_to_pll_mhz(
                                             int cpu_freq_mhz)
{
  int origin_soc_clk = REG_GET_FIELD(SYSTEM_SYSCLK_CONF_REG,
                                        SYSTEM_SOC_CLK_SEL);
  int origin_cpuperiod_sel = REG_GET_FIELD(SYSTEM_CPU_PER_CONF_REG,
                                              SYSTEM_CPUPERIOD_SEL);
  int dbias = DIG_DBIAS_80M;
  int per_conf = DPORT_CPUPERIOD_SEL_80;
  if (cpu_freq_mhz == 160)
    {
      dbias = DIG_DBIAS_160M;
      per_conf = DPORT_CPUPERIOD_SEL_160;
    }
  else
    {
      ASSERT(cpu_freq_mhz != 80);
    }

  REGI2C_WRITE_MASK(I2C_DIG_REG, I2C_DIG_REG_EXT_DIG_DREG, dbias);
  if ((origin_soc_clk == DPORT_SOC_CLK_SEL_XTAL)
      || (origin_soc_clk == DPORT_SOC_CLK_SEL_8M)
      || (((origin_soc_clk == DPORT_SOC_CLK_SEL_PLL)
      && (0 == origin_cpuperiod_sel))))
    {
      esp32c3_wait_dig_dbias_valid(2);
    }

  REG_SET_FIELD(SYSTEM_CPU_PER_CONF_REG, SYSTEM_CPUPERIOD_SEL, per_conf);
  REG_SET_FIELD(SYSTEM_SYSCLK_CONF_REG, SYSTEM_PRE_DIV_CNT, 0);
  REG_SET_FIELD(SYSTEM_SYSCLK_CONF_REG, SYSTEM_SOC_CLK_SEL,
                DPORT_SOC_CLK_SEL_PLL);
  esp32c3_rtc_clk_apb_freq_update(80 * MHZ);
  ets_update_cpu_frequency(cpu_freq_mhz);
}

#ifdef CONFIG_RTC_DRIVER

/****************************************************************************
 * Name: esp32c3_rt_cb_handler
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

static void IRAM_ATTR esp32c3_rt_cb_handler(void *arg)
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

#endif /* CONFIG_RTC_DRIVER */

/****************************************************************************
 * Public Functions
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

enum esp32c3_rtc_xtal_freq_e IRAM_ATTR
            esp32c3_rtc_clk_xtal_freq_get(void)
{
  /* We may have already written XTAL value into RTC_XTAL_FREQ_REG */

  uint32_t xtal_freq_reg = getreg32(RTC_XTAL_FREQ_REG);
  if (!esp32c3_clk_val_is_valid(xtal_freq_reg))
    {
      return RTC_XTAL_FREQ_40M;
    }

  return (xtal_freq_reg & UINT16_MAX);
}

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

void IRAM_ATTR esp32c3_rtc_clk_slow_freq_set(
                           enum esp32c3_rtc_slow_freq_e slow_freq)
{
  REG_SET_FIELD(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_ANA_CLK_RTC_SEL, slow_freq);

  /* Why we need to connect this clock to digital? Or maybe this clock
   * should be connected to digital when xtal 32k clock is enabled instead?
   */

  REG_SET_FIELD(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_DIG_XTAL32K_EN,
                (slow_freq == RTC_SLOW_FREQ_32K_XTAL) ? 1 : 0);

  /* The clk_8m_d256 will be closed when rtc_state in SLEEP,
   * so if the slow_clk is 8md256, clk_8m must be force power on
   */

  REG_SET_FIELD(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_CK8M_FORCE_PU,
               (slow_freq == RTC_SLOW_FREQ_8MD256) ? 1 : 0);
  esp32c3_rtc_clk_set_xtal_wait();
  esp_rom_delay_us(DELAY_SLOW_CLK_SWITCH);
}

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

enum esp32c3_rtc_slow_freq_e IRAM_ATTR esp32c3_rtc_clk_slow_freq_get(void)
{
  return REG_GET_FIELD(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_ANA_CLK_RTC_SEL);
}

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

uint32_t IRAM_ATTR esp32c3_rtc_clk_cal(enum esp32c3_rtc_cal_sel_e cal_clk,
                                                     uint32_t slowclk_cycles)
{
  enum esp32c3_rtc_xtal_freq_e xtal_freq;
  uint64_t xtal_cycles;
  uint64_t divider;
  uint64_t period_64;
  uint32_t period;

  xtal_freq = esp32c3_rtc_clk_xtal_freq_get();
  xtal_cycles = esp32c3_rtc_clk_cal_internal(cal_clk, slowclk_cycles);
  divider = ((uint64_t)xtal_freq) * slowclk_cycles;
  period_64 = ((xtal_cycles << RTC_CLK_CAL_FRACT) + divider / 2 - 1)
                                                          / divider;
  period = (uint32_t)(period_64 & UINT32_MAX);

  return period;
}

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

void esp32c3_rtc_clk_set(void)
{
  esp32c3_rtc_clk_fast_freq_set(RTC_FAST_FREQ_8M);
  esp32c3_select_rtc_slow_clk(RTC_SLOW_FREQ_RTC);
}

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

void IRAM_ATTR esp32c3_rtc_init(void)
{
  struct esp32c3_rtc_priv_s cfg = RTC_CONFIG_DEFAULT();
  struct esp32c3_rtc_init_config_s rtc_init_cfg = RTC_INIT_CONFIG_DEFAULT();

  /* If this pd_cfg is set to 1, all memory won't
   * enter low power mode during light sleep.
   * If this pd_cfg is set to 0, all memory will
   * enter low power mode during light sleep
   */

  struct esp32c3_rtc_sleep_pu_config_s pu_cfg = RTC_SLEEP_PU_CONFIG_ALL(0);
  REGI2C_WRITE_MASK(I2C_DIG_REG, I2C_DIG_REG_XPD_DIG_REG, 0);
  REGI2C_WRITE_MASK(I2C_DIG_REG, I2C_DIG_REG_XPD_RTC_REG, 0);
  modifyreg32(RTC_CNTL_ANA_CONF_REG, RTC_CNTL_PVTMON_PU, 0);
  esp32c3_rtc_clk_set_xtal_wait();
  REG_SET_FIELD(RTC_CNTL_TIMER1_REG, RTC_CNTL_PLL_BUF_WAIT, cfg.pll_wait);
  REG_SET_FIELD(RTC_CNTL_TIMER1_REG, RTC_CNTL_CK8M_WAIT, cfg.ck8m_wait);
  REG_SET_FIELD(RTC_CNTL_TIMER5_REG, RTC_CNTL_MIN_SLP_VAL,
                RTC_CNTL_MIN_SLP_VAL_MIN);

  /* set default powerup & wait time */

  REG_SET_FIELD(RTC_CNTL_TIMER3_REG, RTC_CNTL_WIFI_POWERUP_TIMER,
                rtc_init_cfg.wifi_powerup_cycles);
  REG_SET_FIELD(RTC_CNTL_TIMER3_REG, RTC_CNTL_WIFI_WAIT_TIMER,
                rtc_init_cfg.wifi_wait_cycles);
  REG_SET_FIELD(RTC_CNTL_TIMER3_REG, RTC_CNTL_BT_POWERUP_TIMER,
                rtc_init_cfg.bt_powerup_cycles);
  REG_SET_FIELD(RTC_CNTL_TIMER3_REG, RTC_CNTL_BT_WAIT_TIMER,
                rtc_init_cfg.bt_wait_cycles);
  REG_SET_FIELD(RTC_CNTL_TIMER4_REG, RTC_CNTL_CPU_TOP_POWERUP_TIMER,
                rtc_init_cfg.cpu_top_powerup_cycles);
  REG_SET_FIELD(RTC_CNTL_TIMER4_REG, RTC_CNTL_CPU_TOP_WAIT_TIMER,
                rtc_init_cfg.cpu_top_wait_cycles);
  REG_SET_FIELD(RTC_CNTL_TIMER4_REG, RTC_CNTL_DG_WRAP_POWERUP_TIMER,
                rtc_init_cfg.dg_wrap_powerup_cycles);
  REG_SET_FIELD(RTC_CNTL_TIMER4_REG, RTC_CNTL_DG_WRAP_WAIT_TIMER,
                rtc_init_cfg.dg_wrap_wait_cycles);
  REG_SET_FIELD(RTC_CNTL_TIMER6_REG, RTC_CNTL_DG_PERI_POWERUP_TIMER,
                rtc_init_cfg.dg_peri_powerup_cycles);
  REG_SET_FIELD(RTC_CNTL_TIMER6_REG, RTC_CNTL_DG_PERI_WAIT_TIMER,
                rtc_init_cfg.dg_peri_wait_cycles);

  /* Reset RTC bias to default value (needed if waking up from deep sleep) */

  REGI2C_WRITE_MASK(I2C_DIG_REG, I2C_DIG_REG_EXT_RTC_DREG_SLEEP,
                    RTC_CNTL_DBIAS_1V10);
  REGI2C_WRITE_MASK(I2C_DIG_REG, I2C_DIG_REG_EXT_RTC_DREG,
                    RTC_CNTL_DBIAS_1V10);
  if (cfg.clkctl_init)
    {
      /* clear CMMU clock force on */

      modifyreg32(EXTMEM_CACHE_MMU_POWER_CTRL_REG,
                  EXTMEM_CACHE_MMU_MEM_FORCE_ON, 0);

      /* clear tag clock force on */

      modifyreg32(EXTMEM_ICACHE_TAG_POWER_CTRL_REG,
                  EXTMEM_ICACHE_TAG_MEM_FORCE_ON, 0);

      /* clear register clock force on */

      modifyreg32(SPI_MEM_CLOCK_GATE_REG(0), SPI_MEM_CLK_EN, 0);
      modifyreg32(SPI_MEM_CLOCK_GATE_REG(1), SPI_MEM_CLK_EN, 0);
    }

  if (cfg.pwrctl_init)
    {
      modifyreg32(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_CK8M_FORCE_PU, 0);

      /* cancel xtal force pu if no need to force power up
       * cannot cancel xtal force pu if pll is force power on
       */

      if (!(cfg.xtal_fpu | cfg.bbpll_fpu))
        {
          modifyreg32(RTC_CNTL_OPTIONS0_REG, RTC_CNTL_XTL_FORCE_PU, 0);
        }
      else
        {
          modifyreg32(RTC_CNTL_OPTIONS0_REG, 0, RTC_CNTL_XTL_FORCE_PU);
        }

      /* force pd APLL */

      modifyreg32(RTC_CNTL_ANA_CONF_REG, RTC_CNTL_PLLA_FORCE_PU, 0);
      modifyreg32(RTC_CNTL_ANA_CONF_REG, 0, RTC_CNTL_PLLA_FORCE_PD);

      /* open sar_i2c protect function to avoid sar_i2c
       * reset when rtc_ldo is low.
       */

      modifyreg32(RTC_CNTL_ANA_CONF_REG, RTC_CNTL_I2C_RESET_POR_FORCE_PD, 0);

      /* cancel bbpll force pu if setting no force power up */

      if (!cfg.bbpll_fpu)
        {
          modifyreg32(RTC_CNTL_OPTIONS0_REG, RTC_CNTL_BBPLL_FORCE_PU |
              RTC_CNTL_BBPLL_I2C_FORCE_PU | RTC_CNTL_BB_I2C_FORCE_PU, 0);
        }
      else
        {
          modifyreg32(RTC_CNTL_OPTIONS0_REG, 0, RTC_CNTL_BBPLL_FORCE_PU |
              RTC_CNTL_BBPLL_I2C_FORCE_PU | RTC_CNTL_BB_I2C_FORCE_PU);
        }

      modifyreg32(RTC_CNTL_REG, RTC_CNTL_REGULATOR_FORCE_PU |
                  RTC_CNTL_DBOOST_FORCE_PU, 0);
      if (cfg.rtc_dboost_fpd)
        {
          modifyreg32(RTC_CNTL_REG, 0, RTC_CNTL_DBOOST_FORCE_PD);
        }
      else
        {
          modifyreg32(RTC_CNTL_REG, RTC_CNTL_DBOOST_FORCE_PD, 0);
        }

      /* If this mask is enabled, all soc memories
       * cannot enter power down mode.
       * We should control soc memory power down mode from RTC,
       * so we will not touch this register any more.
       */

      modifyreg32(SYSTEM_MEM_PD_MASK_REG, SYSTEM_LSLP_MEM_PD_MASK, 0);
      esp32c3_rtc_sleep_pu(pu_cfg);
      modifyreg32(RTC_CNTL_DIG_PWC_REG, RTC_CNTL_DG_WRAP_FORCE_PU |
                  RTC_CNTL_WIFI_FORCE_PU | RTC_CNTL_BT_FORCE_PU |
                  RTC_CNTL_CPU_TOP_FORCE_PU | RTC_CNTL_DG_PERI_FORCE_PU , 0);

      modifyreg32(RTC_CNTL_DIG_ISO_REG, RTC_CNTL_DG_WRAP_FORCE_NOISO |
            RTC_CNTL_WIFI_FORCE_NOISO | RTC_CNTL_BT_FORCE_NOISO |
            RTC_CNTL_CPU_TOP_FORCE_NOISO | RTC_CNTL_DG_PERI_FORCE_NOISO , 0);

      /* cancel digital PADS force no iso */

      if (cfg.cpu_waiti_clk_gate)
        {
          modifyreg32(SYSTEM_CPU_PER_CONF_REG,
                      SYSTEM_CPU_WAIT_MODE_FORCE_ON, 0);
        }
      else
        {
          modifyreg32(SYSTEM_CPU_PER_CONF_REG, 0,
                      SYSTEM_CPU_WAIT_MODE_FORCE_ON);
        }

      /* if SYSTEM_CPU_WAIT_MODE_FORCE_ON == 0 ,
       * the cpu clk will be closed when cpu enter WAITI mode.
       */

      modifyreg32(RTC_CNTL_DIG_ISO_REG, RTC_CNTL_DG_PAD_FORCE_UNHOLD |
                  RTC_CNTL_DG_PAD_FORCE_NOISO, 0);
    }
}

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

uint64_t esp32c3_rtc_get_time_us(void)
{
  const uint32_t cal = getreg32(RTC_SLOW_CLK_CAL_REG);
  const uint64_t rtc_this_ticks = esp32c3_rtc_time_get();

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

uint64_t IRAM_ATTR esp32c3_rtc_time_get(void)
{
  uint64_t rtc_time;

  modifyreg32(RTC_CNTL_TIME_UPDATE_REG, 0, RTC_CNTL_TIME_UPDATE);
  rtc_time = getreg32(RTC_CNTL_TIME0_REG);
  rtc_time |= ((uint64_t) getreg32(RTC_CNTL_TIME1_REG)) << 32;

  return rtc_time;
}

/****************************************************************************
 * Name: esp32c3_rtc_time_us_to_slowclk
 *
 * Description:
 *   Convert time interval from microseconds to RTC_SLOW_CLK cycles.
 *
 * Input Parameters:
 *   time_in_us      - Time interval in microseconds
 *   slow_clk_period - Period of slow clock in microseconds
 *
 * Returned Value:
 *   number of slow clock cycles
 *
 ****************************************************************************/

uint64_t IRAM_ATTR esp32c3_rtc_time_us_to_slowclk(uint64_t time_in_us,
                                                  uint32_t period)
{
  /* Overflow will happen in this function if time_in_us >= 2^45,
   * which is about 400 days. TODO: fix overflow.
   */

  return (time_in_us << RTC_CLK_CAL_FRACT) / period;
}

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

uint64_t IRAM_ATTR esp32c3_rtc_time_slowclk_to_us(uint64_t rtc_cycles,
                                                  uint32_t period)
{
  return (rtc_cycles * period) >> RTC_CLK_CAL_FRACT;
}

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

void IRAM_ATTR esp32c3_rtc_cpu_freq_set_xtal(void)
{
  int freq_mhz = (int) esp32c3_rtc_clk_xtal_freq_get();
  esp32c3_rtc_update_to_xtal(freq_mhz, 1);
  esp32c3_rtc_clk_bbpll_disable();
}

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

void IRAM_ATTR esp32c3_rtc_sleep_init(uint32_t flags)
{
  struct esp32c3_rtc_sleep_config_s cfg = RTC_SLEEP_CONFIG_DEFAULT(flags);
  struct esp32c3_rtc_sleep_pu_config_s pu_cfg = RTC_SLEEP_PU_CONFIG_ALL(1);
  if (cfg.lslp_mem_inf_fpu)
    {
      esp32c3_rtc_sleep_pu(pu_cfg);
    }

  if (cfg.wifi_pd_en)
    {
      modifyreg32(RTC_CNTL_DIG_PWC_REG, 0, RTC_CNTL_WIFI_PD_EN);
    }
  else
    {
      modifyreg32(RTC_CNTL_DIG_PWC_REG, RTC_CNTL_WIFI_PD_EN, 0);
    }

  if (cfg.bt_pd_en)
    {
      modifyreg32(RTC_CNTL_DIG_PWC_REG, 0, RTC_CNTL_BT_PD_EN);
    }
  else
    {
      modifyreg32(RTC_CNTL_DIG_PWC_REG, RTC_CNTL_BT_PD_EN, 0);
    }

  if (cfg.cpu_pd_en)
    {
      modifyreg32(RTC_CNTL_DIG_PWC_REG, 0, RTC_CNTL_CPU_TOP_PD_EN);
    }
  else
    {
      modifyreg32(RTC_CNTL_DIG_PWC_REG, RTC_CNTL_CPU_TOP_PD_EN, 0);
    }

  if (cfg.dig_peri_pd_en)
    {
      modifyreg32(RTC_CNTL_DIG_PWC_REG, 0, RTC_CNTL_DG_PERI_PD_EN);
    }
  else
    {
      modifyreg32(RTC_CNTL_DIG_PWC_REG, RTC_CNTL_DG_PERI_PD_EN, 0);
    }

  REG_SET_FIELD(RTC_CNTL_BIAS_CONF_REG, RTC_CNTL_DBG_ATTEN_MONITOR,
                RTC_CNTL_DBG_ATTEN_MONITOR_DEFAULT);
  REG_SET_FIELD(RTC_CNTL_BIAS_CONF_REG, RTC_CNTL_BIAS_SLEEP_MONITOR,
                RTC_CNTL_BIASSLP_MONITOR_DEFAULT);
  REG_SET_FIELD(RTC_CNTL_BIAS_CONF_REG, RTC_CNTL_BIAS_SLEEP_DEEP_SLP,
                RTC_CNTL_BIASSLP_SLEEP_DEFAULT);
  REG_SET_FIELD(RTC_CNTL_BIAS_CONF_REG, RTC_CNTL_PD_CUR_MONITOR,
                RTC_CNTL_PD_CUR_MONITOR_DEFAULT);
  REG_SET_FIELD(RTC_CNTL_BIAS_CONF_REG, RTC_CNTL_PD_CUR_DEEP_SLP,
                RTC_CNTL_PD_CUR_SLEEP_DEFAULT);
  if (cfg.deep_slp)
    {
      REGI2C_WRITE_MASK(I2C_ULP, I2C_ULP_IR_FORCE_XPD_CK, 0);
      modifyreg32(RTC_CNTL_REG, RTC_CNTL_REGULATOR_FORCE_PU, 0);

      /* It's only a temporary configuration to set dbg 0 to make
       * deepsleep run successfully when in high temperature.
       * We will restore it to RTC_CNTL_DBG_ATTEN_DEEPSLEEP_DEFAULT
       * when ECO chip come back. TODO ESP32-C3 IDF-2568
       */

      REG_SET_FIELD(RTC_CNTL_BIAS_CONF_REG, RTC_CNTL_DBG_ATTEN_DEEP_SLP, 0);
      modifyreg32(RTC_CNTL_DIG_PWC_REG, 0, RTC_CNTL_DG_WRAP_PD_EN);
      modifyreg32(RTC_CNTL_ANA_CONF_REG, RTC_CNTL_CKGEN_I2C_PU |
      RTC_CNTL_PLL_I2C_PU | RTC_CNTL_RFRX_PBUS_PU | RTC_CNTL_TXRF_I2C_PU, 0);
    }
  else
    {
      modifyreg32(RTC_CNTL_BIAS_CONF_REG, 0, RTC_CNTL_DG_VDD_DRV_B_SLP_EN);
      REG_SET_FIELD(RTC_CNTL_BIAS_CONF_REG, RTC_CNTL_DG_VDD_DRV_B_SLP,
                    RTC_CNTL_DG_VDD_DRV_B_SLP_DEFAULT);
      modifyreg32(RTC_CNTL_REG, 0, RTC_CNTL_REGULATOR_FORCE_PU);
      modifyreg32(RTC_CNTL_DIG_PWC_REG, RTC_CNTL_DG_WRAP_PD_EN, 0);
      REG_SET_FIELD(RTC_CNTL_BIAS_CONF_REG, RTC_CNTL_DBG_ATTEN_DEEP_SLP,
                    RTC_CNTL_DBG_ATTEN_LIGHTSLEEP_DEFAULT);
    }

  /* enable VDDSDIO control by state machine */

  REG_CLR_BIT(RTC_CNTL_SDIO_CONF_REG, RTC_CNTL_SDIO_FORCE);
  REG_SET_FIELD(RTC_CNTL_SDIO_CONF_REG, RTC_CNTL_SDIO_PD_EN,
                cfg.vddsdio_pd_en);
  REGI2C_WRITE_MASK(I2C_DIG_REG, I2C_DIG_REG_EXT_RTC_DREG_SLEEP,
                    cfg.rtc_dbias_slp);
  REGI2C_WRITE_MASK(I2C_DIG_REG, I2C_DIG_REG_EXT_RTC_DREG,
                    cfg.rtc_dbias_wak);
  REGI2C_WRITE_MASK(I2C_DIG_REG, I2C_DIG_REG_EXT_DIG_DREG_SLEEP,
                    cfg.dig_dbias_slp);
  REGI2C_WRITE_MASK(I2C_DIG_REG, I2C_DIG_REG_EXT_DIG_DREG,
                    cfg.dig_dbias_wak);
  REG_SET_FIELD(RTC_CNTL_SLP_REJECT_CONF_REG, RTC_CNTL_DEEP_SLP_REJECT_EN,
                cfg.deep_slp_reject);
  REG_SET_FIELD(RTC_CNTL_SLP_REJECT_CONF_REG, RTC_CNTL_LIGHT_SLP_REJECT_EN,
                cfg.light_slp_reject);

  /* gating XTAL clock */

  REG_CLR_BIT(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_XTAL_GLOBAL_FORCE_NOGATING);
}

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

uint32_t IRAM_ATTR esp32c3_rtc_deep_sleep_start(uint32_t wakeup_opt,
                                                uint32_t reject_opt)
{
  /* Values used to set the SYSTEM_RTC_FASTMEM_CONFIG_REG value */

  const unsigned CRC_START_ADDR = 0;
  const unsigned CRC_LEN = 0x7ff;

  REG_SET_FIELD(RTC_CNTL_WAKEUP_STATE_REG, RTC_CNTL_WAKEUP_ENA, wakeup_opt);
  putreg32(reject_opt, RTC_CNTL_SLP_REJECT_CONF_REG);

  /* Calculate RTC Fast Memory CRC (for wake stub) & go to deep sleep
   * Because we may be running from RTC memory as stack, we can't easily
   * call any functions to do this (as registers will spill to stack,
   * corrupting the CRC). Instead, load all the values we need into registers
   * then use register ops only to calculate the CRC value, write it to the
   * RTC CRC value register, and immediately go into deep sleep.
   */

  asm volatile(

  /* Start CRC calculation */

    "sw %1, 0(%0)\n"
    "or t0, %1, %2\n"
    "sw t0, 0(%0)\n"

  /* Wait for the CRC calculation to finish */

    ".Lwaitcrc:\n"
    "fence\n"
    "lw t0, 0(%0)\n"
    "li t1, "STR(SYSTEM_RTC_MEM_CRC_FINISH)"\n"
    "and t0, t0, t1\n"
    "beqz t0, .Lwaitcrc\n"
    "not %2, %2\n"
    "and t0, t0, %2\n"
    "sw t0, 0(%0)\n"
    "fence\n"
    "not %2, %2\n"

  /* Store the calculated value in RTC_MEM_CRC_REG */

    "lw t0, 0(%3)\n"
    "sw t0, 0(%4)\n"
    "fence\n"

  /* Set register bit to go into deep sleep */

    "lw t0, 0(%5)\n"
    "or   t0, t0, %6\n"
    "sw t0, 0(%5)\n"
    "fence\n"

  /* Wait for sleep reject interrupt (never finishes if successful) */

    ".Lwaitsleep:"
    "fence\n"
    "lw t0, 0(%7)\n"
    "and t0, t0, %8\n"
    "beqz t0, .Lwaitsleep\n"

    :
    :
      "r" (SYSTEM_RTC_FASTMEM_CONFIG_REG),
      "r" ((CRC_START_ADDR << SYSTEM_RTC_MEM_CRC_START_S)
            | (CRC_LEN << SYSTEM_RTC_MEM_CRC_LEN_S)),
      "r" (SYSTEM_RTC_MEM_CRC_START),
      "r" (SYSTEM_RTC_FASTMEM_CRC_REG),
      "r" (RTC_MEMORY_CRC_REG),
      "r" (RTC_CNTL_STATE0_REG),
      "r" (RTC_CNTL_SLEEP_EN),
      "r" (RTC_CNTL_INT_RAW_REG),
      "r" (RTC_CNTL_SLP_REJECT_INT_RAW | RTC_CNTL_SLP_WAKEUP_INT_RAW)
    : "t0", "t1"
  );

  return esp32c3_rtc_sleep_finish(0);
}

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

uint32_t IRAM_ATTR esp32c3_rtc_sleep_start(uint32_t wakeup_opt,
                            uint32_t reject_opt, uint32_t lslp_mem_inf_fpu)
{
  REG_SET_FIELD(RTC_CNTL_WAKEUP_STATE_REG, RTC_CNTL_WAKEUP_ENA, wakeup_opt);
  REG_SET_FIELD(RTC_CNTL_SLP_REJECT_CONF_REG,
                RTC_CNTL_SLEEP_REJECT_ENA, reject_opt);

  /* Start entry into sleep mode */

  modifyreg32(RTC_CNTL_STATE0_REG, 0, RTC_CNTL_SLEEP_EN);

  while ((getreg32(RTC_CNTL_INT_RAW_REG) &
        (RTC_CNTL_SLP_REJECT_INT_RAW | RTC_CNTL_SLP_WAKEUP_INT_RAW)) == 0);

  return esp32c3_rtc_sleep_finish(lslp_mem_inf_fpu);
}

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

void IRAM_ATTR esp32c3_rtc_clk_cpu_freq_set_config(
               const struct esp32c3_cpu_freq_config_s *config)
{
  uint32_t soc_clk_sel = REG_GET_FIELD(SYSTEM_SYSCLK_CONF_REG,
                                          SYSTEM_SOC_CLK_SEL);
  if (config->source == RTC_CPU_FREQ_SRC_XTAL)
    {
      esp32c3_rtc_update_to_xtal(config->freq_mhz, config->div);
      if (soc_clk_sel == DPORT_SOC_CLK_SEL_PLL)
        {
          esp32c3_rtc_clk_bbpll_disable();
        }
    }
  else if (config->source == RTC_CPU_FREQ_SRC_PLL)
    {
      if (soc_clk_sel != DPORT_SOC_CLK_SEL_PLL)
        {
          modifyreg32(RTC_CNTL_OPTIONS0_REG, RTC_CNTL_BB_I2C_FORCE_PD |
                RTC_CNTL_BBPLL_FORCE_PD | RTC_CNTL_BBPLL_I2C_FORCE_PD,  0);
          esp32c3_rtc_bbpll_configure(esp32c3_rtc_clk_xtal_freq_get(),
                                             config->source_freq_mhz);
        }

      esp32c3_rtc_clk_cpu_freq_to_pll_mhz(config->freq_mhz);
    }
  else if (config->source == RTC_CPU_FREQ_SRC_8M)
    {
      esp32c3_rtc_clk_cpu_freq_to_8m();
      if (soc_clk_sel == DPORT_SOC_CLK_SEL_PLL)
        {
          esp32c3_rtc_clk_bbpll_disable();
        }
    }
}

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

void IRAM_ATTR esp32c3_rtc_sleep_low_init(uint32_t slowclk_period)
{
  /* Set 5 PWC state machine times to fit in main state machine time */

  REG_SET_FIELD(RTC_CNTL_TIMER1_REG, RTC_CNTL_PLL_BUF_WAIT,
                          RTC_CNTL_PLL_BUF_WAIT_SLP_CYCLES);
  REG_SET_FIELD(RTC_CNTL_TIMER1_REG, RTC_CNTL_XTL_BUF_WAIT,
                          esp32c3_rtc_time_us_to_slowclk(
                          RTC_CNTL_XTL_BUF_WAIT_SLP_US, slowclk_period));
  REG_SET_FIELD(RTC_CNTL_TIMER1_REG, RTC_CNTL_CK8M_WAIT,
                          RTC_CNTL_CK8M_WAIT_SLP_CYCLES);
}

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

void IRAM_ATTR esp32c3_rtc_clk_cpu_freq_get_config(
                           struct esp32c3_cpu_freq_config_s *out_config)
{
  uint32_t div = 3;
  uint32_t freq_mhz = 160;
  uint32_t source_freq_mhz = RTC_PLL_FREQ_480M;
  enum esp32c3_rtc_cpu_freq_src_e source = RTC_CPU_FREQ_SRC_PLL;
  uint32_t soc_clk_sel = REG_GET_FIELD(SYSTEM_SYSCLK_CONF_REG,
                                       SYSTEM_SOC_CLK_SEL);
  switch (soc_clk_sel)
    {
      case DPORT_SOC_CLK_SEL_XTAL:
        {
          source = RTC_CPU_FREQ_SRC_XTAL;
          div = REG_GET_FIELD(SYSTEM_SYSCLK_CONF_REG,
                              SYSTEM_PRE_DIV_CNT) + 1;
          source_freq_mhz = (uint32_t) esp32c3_rtc_clk_xtal_freq_get();
          freq_mhz = source_freq_mhz / div;
        }
        break;

      case DPORT_SOC_CLK_SEL_PLL:
        {
          uint32_t cpuperiod_sel = REG_GET_FIELD(SYSTEM_CPU_PER_CONF_REG,
                                                   SYSTEM_CPUPERIOD_SEL);
          uint32_t pllfreq_sel = REG_GET_FIELD(SYSTEM_CPU_PER_CONF_REG,
                                                   SYSTEM_PLL_FREQ_SEL);
          source = RTC_CPU_FREQ_SRC_PLL;
          source_freq_mhz = (pllfreq_sel) ?
                             RTC_PLL_FREQ_480M : RTC_PLL_FREQ_320M;
          if (cpuperiod_sel == DPORT_CPUPERIOD_SEL_80)
            {
              div = (source_freq_mhz == RTC_PLL_FREQ_480M) ? 6 : 4;
              freq_mhz = 80;
            }
          else if (cpuperiod_sel == DPORT_CPUPERIOD_SEL_160)
            {
              div = (source_freq_mhz == RTC_PLL_FREQ_480M) ? 3 : 2;
              div = 3;
              freq_mhz = 160;
            }
          else
            {
              ASSERT(0);
            }
        }
        break;

      case DPORT_SOC_CLK_SEL_8M:
        {
          source = RTC_CPU_FREQ_SRC_8M;
          source_freq_mhz = 8;
          div = 1;
          freq_mhz = source_freq_mhz;
        }
        break;

      default:
        ASSERT(0);
    }

  *out_config = (struct esp32c3_cpu_freq_config_s)
    {
      .source = source,
      .source_freq_mhz = source_freq_mhz,
      .div = div,
      .freq_mhz = freq_mhz
    };
}

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

void IRAM_ATTR esp32c3_rtc_sleep_set_wakeup_time(uint64_t t)
{
  putreg32(t & UINT32_MAX, RTC_CNTL_SLP_TIMER0_REG);
  putreg32((uint32_t)(t >> 32), RTC_CNTL_SLP_TIMER1_REG);
  modifyreg32(RTC_CNTL_INT_CLR_REG, 0, RTC_CNTL_MAIN_TIMER_INT_CLR_M);
  modifyreg32(RTC_CNTL_SLP_TIMER1_REG, 0, RTC_CNTL_MAIN_TIMER_ALARM_EN_M);
}

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

void IRAM_ATTR esp32c3_rtc_set_boot_time(uint64_t time_us)
{
  putreg32((uint32_t)(time_us & UINT32_MAX), RTC_BOOT_TIME_LOW_REG);
  putreg32((uint32_t)(time_us >> 32), RTC_BOOT_TIME_HIGH_REG);
}

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

uint64_t IRAM_ATTR esp32c3_rtc_get_boot_time(void)
{
  return ((uint64_t)getreg32(RTC_BOOT_TIME_LOW_REG))
        + (((uint64_t)getreg32(RTC_BOOT_TIME_HIGH_REG)) << 32);
}

#ifdef CONFIG_RTC_DRIVER

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
                              esp32c3_rtc_get_boot_time();
    }
  else
    {
      /* Get the time from RTC controller. */

      time_us = esp32c3_rtc_get_time_us() +
                  esp32c3_rtc_get_boot_time();
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

      rtc_offset_us = now_us - esp32c3_rtc_get_time_us();
    }

  g_rtc_save->offset = 0;
  esp32c3_rtc_set_boot_time(rtc_offset_us);

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

  esp32c3_rtc_init();
  esp32c3_rtc_clk_set();
#endif

  g_rtc_save = &rtc_saved_data;

  /* If saved data is invalid, clear offset information */

  if (g_rtc_save->magic != MAGIC_RTC_SAVE)
    {
      g_rtc_save->magic = MAGIC_RTC_SAVE;
      g_rtc_save->offset = 0;
      esp32c3_rtc_set_boot_time(0);
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
                              esp32c3_rtc_get_boot_time();
    }
  else
    {
      time_us = = esp32c3_rtc_get_time_us() +
                    esp32c3_rtc_get_boot_time();
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
          rt_timer_args.callback = esp32c3_rt_cb_handler;
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

  g_rtc_save->offset = esp32c3_rtc_get_time_us() - rt_timer_time_us();

  return OK;
}

#endif /* CONFIG_RTC_DRIVER */

