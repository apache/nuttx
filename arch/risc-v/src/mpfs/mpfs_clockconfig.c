/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_clockconfig.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>
#include <arch/board/board_liberodefs.h>

#include "mpfs_clockconfig.h"
#include "riscv_arch.h"
#include "hardware/mpfs_sysreg.h"
#include "hardware/mpfs_sgmii.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OSC_FREQ 80000000UL

/* Read & Write Memory barrier */

#define RISCV_FENCE(p, s) \
        __asm__ __volatile__ ("fence " #p "," #s : : : "memory")

#define mb() RISCV_FENCE(iorw, iorw)

/* Module reset / clock enable defines */

#define MPFS_SYSREG_SOFT_RESET_CR     (MPFS_SYSREG_BASE + \
                                       MPFS_SYSREG_SOFT_RESET_CR_OFFSET)
#define MPFS_SYSREG_SUBBLK_CLOCK_CR   (MPFS_SYSREG_BASE + \
                                       MPFS_SYSREG_SUBBLK_CLOCK_CR_OFFSET)

/* PLL bit defines */

#define PLL_CTRL_LOCK_BIT             (1 << 25)
#define PLL_INIT_AND_OUT_OF_RESET     0x3
#define PLL_CTRL_REG_POWERDOWN_B_MASK 0x1

/* eNVM clock frequency settled poll bit */

#define ENVM_CR_CLOCK_OKAY_MASK       (1 << 6)

/* Silicon version lookup bitmasks */

#define ARO_REF_PCODE_MASK            0x3f
#define ARO_REF_PCODE_REVC_THRESHOLD  0x40
#define MIN_DLL_90_CODE_VALUE_INDICATING_TT_PART_REVB  13

#define MPFS_SCB_ACCESS_CONFIG        ((160 << 8) | (0x80))

/* Poll loop default retries */

#define MPFS_DEFAULT_RETRIES          0xffff

/* Eye width defines */

#define EARLY_EYE_WIDTH_PART_PART_NOT_DETERMINED       6
#define EARLY_EYE_WIDTH_PART_REVC_OR_LATER             6
#define EARLY_EYE_WIDTH_PART_REVC_OR_LATER_PRE_TEST    7
#define EARLY_EYE_WIDTH_SS_PART_REVB                   5
#define EARLY_TT_PART_REVB                             6

#define LATE_EYE_WIDTH_PART_NOT_DETERMINED             7
#define LATE_EYE_WIDTH_PART_REVC_OR_LATER              7
#define LATE_EYE_WIDTH_PART_REVC_OR_LATER_PRE_TEST     6
#define LATE_EYE_WIDTH_SS_PART_REVB                    6
#define LATE_TT_PART_REVB                              7

#define SHIFT_TO_REG_RX0_EYEWIDTH     21
#define REG_RX0_EYEWIDTH_P_MASK       (~(0x7 << SHIFT_TO_REG_RX0_EYEWIDTH))

#define SHIFT_TO_REG_RX1_EYEWIDTH     21
#define REG_RX1_EYEWIDTH_P_MASK       (~(0x7 << SHIFT_TO_REG_RX1_EYEWIDTH))

#define SHIFT_TO_CH0_N_EYE_VALUE      26
#define SHIFT_TO_CH1_N_EYE_VALUE      29
#define N_EYE_MASK                    0x03ffffff

#define REG_RX0_EN_FLAG_N             (1 << 31)
#define REG_RX1_EN_FLAG_N             (1 << 31)

#define MSSIO_CONTROL_CR_MSS_IO_EN                     (1 << 13)
#define MSSIO_CONTROL_CR_MSS_FLASH_VALID               (1 << 12)
#define MSSIO_CONTROL_CR_MSS_CORE_UP                   (1 << 11)
#define MSSIO_CONTROL_CR_MSS_DCE                       (0x7 << 8)

#define SGMII_NV_MAP_DDR_PHY                           (1 << 0)
#define SGMII_ARO_PLL0_LOCK                            (1 << 7)

/* MPFS_CFG_DDR_SGMII_PHY_DYN_CNTL bit defines */

#define SGMII_REG_LANE1_SOFT_RESET_PERIPH              (1 << 14)
#define SGMII_REG_LANE0_SOFT_RESET_PERIPH              (1 << 13)
#define SGMII_REG_PVT_SOFT_RESET_PERIPH                (1 << 10)

/* MPFS_IOSCB_CALIB_SGMII_IOC_REG0 */

#define SGMII_IOC_REG0_REG_CALIB_LOCK                  (1 << 14)

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum part_type_e
{
    PART_NOT_DETERMINED   = 0x00,
    PART_REVC_OR_LATER    = 0x01,
    SS_PART_REVB          = 0x02,
    TT_PART_REVB          = 0x03,
    PART_TYPE_ARRAY_SIZE  = 0x04,
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint64_t g_cpu_clock = MPFS_MSS_EXT_SGMII_REF_CLK;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_wait_cycles
 *
 * Description:
 *   Wait n number of cycles. This mimics the vendor wait loops in the
 *   reference code.
 *
 * Parameters:
 *   n   - number of cycles to loop
 *
 ****************************************************************************/

static void mpfs_wait_cycles(uint32_t n)
{
  volatile uint32_t count = n;

  while (--count);
}

/****************************************************************************
 * Name: mpfs_set_rtc_divisor
 *
 * Description:
 *   Set the RTC divisor based on MSS Configurator setting.
 *   This will always be calculated so that RTC clock is 1MHz.
 *
 ****************************************************************************/

static void mpfs_set_rtc_divisor(void)
{
  /* Disable RTC clock */

  modifyreg32(MPFS_SYSREG_RTC_CLOCK_CR, SYSREG_RTC_CLOCK_CR_ENABLE, 0);

  putreg32((LIBERO_SETTING_MSS_EXT_SGMII_REF_CLK / \
           LIBERO_SETTING_MSS_RTC_TOGGLE_CLK),
           MPFS_SYSREG_RTC_CLOCK_CR);

  /* Enable RTC Clock */

  modifyreg32(MPFS_SYSREG_RTC_CLOCK_CR, 0, SYSREG_RTC_CLOCK_CR_ENABLE);
}

/****************************************************************************
 * Name: mpfs_sgmii_mux_config
 *
 * Description:
 *   Soft resets and loads RPC settings.
 *
 ****************************************************************************/

void mpfs_sgmii_mux_config(void)
{
  /* This will load the APB registers, set via SGMII TIP */

  putreg32(1, MPFS_IOSCB_SGMII_MUX_SOFT_RESET);
}

/****************************************************************************
 * Name: mpfs_sgmii_pll_config_scb
 *
 * Description:
 *   Soft resets and loads RPC settings
 *
 ****************************************************************************/

static void mpfs_sgmii_pll_config_scb(void)
{
  /* Set the NV map reset. This will load the APB registers, set
   * via SGMII TIP.
   */

  putreg32(1, MPFS_IOSCB_SGMII_PLL_SOFT_RESET);
}

/****************************************************************************
 * Name: mpfs_set_early_late_thresholds
 *
 * Description:
 *   Sets the N and P eye width values.
 *
 * Parameters:
 *   n_late_threshold   - N eye width
 *   p_early_threshold  - P eye width
 *
 ****************************************************************************/

static void mpfs_set_early_late_thresholds(uint8_t n_late_threshold,
                                           uint8_t p_early_threshold)
{
  uint32_t n_eye_values;
  uint32_t p_eye_value;

  /* Set the N eye width value, bits 31:29 for CH1, bits 28:26
   * for CH0 in spare control (N eye width value)
   */

  n_eye_values = (n_late_threshold << SHIFT_TO_CH0_N_EYE_VALUE);
  n_eye_values |= (n_late_threshold << SHIFT_TO_CH1_N_EYE_VALUE);

  putreg32((LIBERO_SETTING_SPARE_CNTL & N_EYE_MASK) | n_eye_values,
           MPFS_CFG_DDR_SGMII_PHY_SPARE_STAT);

  /* Set CH0/CH1 P values */

  p_eye_value = (p_early_threshold << SHIFT_TO_REG_RX0_EYEWIDTH);

  putreg32(((LIBERO_SETTING_CH0_CNTL & REG_RX0_EYEWIDTH_P_MASK) |
           p_eye_value) | REG_RX0_EN_FLAG_N,
           MPFS_CFG_DDR_SGMII_PHY_CH0_CNTL);
  putreg32(((LIBERO_SETTING_CH1_CNTL & REG_RX1_EYEWIDTH_P_MASK) |
           p_eye_value) | REG_RX1_EN_FLAG_N,
           MPFS_CFG_DDR_SGMII_PHY_CH1_CNTL);
}

/****************************************************************************
 * Name: mss_mux_pre_mss_pll_config
 *
 * Description:
 *   Apply required reference clks to PLL, configure PLL and wait for lock.
 *
 ****************************************************************************/

static void mss_mux_pre_mss_pll_config(void)
{
  putreg32(LIBERO_SETTING_MSS_PLL_CKMUX, MPFS_IOSCB_MSS_MUX_PLL_CKMUX);
  putreg32(LIBERO_SETTING_SGMII_CLK_XCVR, MPFS_IOSCB_SGMII_MUX_CLK_XCVR);
  putreg32(LIBERO_SETTING_MSS_BCLKMUX, MPFS_IOSCB_MSS_MUX_BCLKMUX);
  putreg32(LIBERO_SETTING_MSS_FMETER_ADDR, MPFS_IOSCB_MSS_MUX_FMETER_ADDR);
  putreg32(LIBERO_SETTING_MSS_FMETER_DATAW, MPFS_IOSCB_MSS_MUX_FMETER_DATAW);
  putreg32(LIBERO_SETTING_MSS_FMETER_DATAR, MPFS_IOSCB_MSS_MUX_FMETER_DATAR);

  mpfs_wait_cycles(400);
}

/****************************************************************************
 * Name: mss_mux_pre_mss_pll_config
 *
 * Description:
 *   We must run this code from RAM, as we need to modify the clock of the
 *   eNVM. The first thing we do is change the eNVM clock, to prevent L1
 *   cache accessing eNVM as it will do as we approach the return
 *   instruction. The mb() makes sure order of processing is not changed by
 *   the compiler
 *
 * Assumptions:
 *   This should be run outside of eNVM, although apparently it works even
 *   from eNVM.
 *
 ****************************************************************************/

__attribute__((section(".ram"))) void mpfs_mux_post_mss_pll_config(void)
{
  /* Modify the eNVM clock, so it matches the new MSS clock
   *
   * 7 will generate a 40ns period 25MHz clock if the AHB clock is 200MHz
   * 11 will generate a 40ns period 25MHz clock if the AHB clock is 250MHz
   * 15 will generate a 40ns period 25MHz clock if the AHB clock is 400MHz
   */

  putreg32(LIBERO_SETTING_MSS_ENVM_CR, MPFS_SYSREG_ENVM_CR);

  /* Make sure we change clock in eNVM first so all is ready by the time we
   * leave.
   */

  mb();

  /* When changing the eNVM clock frequency, there is a bit
   * (ENVM_CR_clock_okay) in the eNVM_CR which can be polled to check that
   * the frequency change has happened before bumping up the AHB frequency.
   */

  while ((getreg32(MPFS_SYSREG_ENVM_CR) & ENVM_CR_CLOCK_OKAY_MASK) != \
         ENVM_CR_CLOCK_OKAY_MASK);

  /* Change the MSS clock as required */

  putreg32(LIBERO_SETTING_MSS_CLOCK_CONFIG_CR, MPFS_SYSREG_CLOCK_CONFIG_CR);

  /* Feed clock from MSS PLL to MSS, using glitchless mux */

  putreg32(LIBERO_SETTING_MSS_MSSCLKMUX, MPFS_IOSCB_MSS_MUX_MSSCLKMUX);

  /* Change the RTC clock divisor, so RTC clock is 1MHz */

  mpfs_set_rtc_divisor();
}

/****************************************************************************
 * Name: mpfs_pll_config
 *
 * Description:
 *   On startup, MSS is supplied with 80MHz SCB clock.
 *   Power on procedure for the MSS PLL clock:
 *
 *   During POR:
 *     Keep PLL in power down mode. Powerdown_int_b = 0
 *
 *   After POR, Power-On steps:
 *   1)  mssclk_mux_sel_int = 0 & powerdown_int_b = 0 & clk_standby_sel = 0
 *       MSS PLL is powered down and selects clk_standby = scb_clk
 *   2)  PFC Processor writes powerdown_int_b = 1 & divq0_int_en = 1
 *       MSS PLL powers up, then lock asserts when locked.
 *   3)  PFC Processor switches mssclk_mux_sel_int = 1
 *       MSS PLL clock is now sent to MSS.
 *   4)  When BOOT authentication is complete
 *       a.  PFC processor writes mssclk_mux_sel_int = 0 to select
 *           clk_standby.
 *       b.  PFC Processor writes powerdown_int_b = 0 to power down the PLL
 *       c.  MSS Processor writes new parameters to the MSS PLL
 *   5)  MSS Processor writes powerdown_int_b = 1
 *       Start up the PLL with new parameters.
 *       Wait for PLL to lock.
 *   6)  MSS Processor enables all 4 PLL outputs.
 *   7)  MSS Processor writes mssclk_mux_sel_int = 1 to select the MSS PLL
 *       clock.
 *
 ****************************************************************************/

static int mpfs_pll_config(void)
{
  uint32_t retries = MPFS_DEFAULT_RETRIES;

  putreg32(PLL_INIT_AND_OUT_OF_RESET, MPFS_IOSCB_MSS_PLL_SOFT_RESET);
  putreg32(PLL_INIT_AND_OUT_OF_RESET, MPFS_IOSCB_DDR_PLL_SOFT_RESET);

  putreg32(LIBERO_SETTING_MSS_PLL_CTRL & ~(PLL_CTRL_REG_POWERDOWN_B_MASK),
           MPFS_IOSCB_MSS_PLL_CTRL);

  putreg32(LIBERO_SETTING_MSS_PLL_REF_FB, MPFS_IOSCB_MSS_PLL_REF_FB);

  putreg32(LIBERO_SETTING_MSS_PLL_DIV_0_1, MPFS_IOSCB_MSS_PLL_DIV_0_1);
  putreg32(LIBERO_SETTING_MSS_PLL_DIV_2_3, MPFS_IOSCB_MSS_PLL_DIV_2_3);
  putreg32(LIBERO_SETTING_MSS_PLL_CTRL2, MPFS_IOSCB_MSS_PLL_CTRL2);
  putreg32(LIBERO_SETTING_MSS_PLL_FRACN, MPFS_IOSCB_MSS_PLL_FRACN);
  putreg32(LIBERO_SETTING_MSS_SSCG_REG_0, MPFS_IOSCB_MSS_PLL_SSCG_REG_0);
  putreg32(LIBERO_SETTING_MSS_SSCG_REG_1, MPFS_IOSCB_MSS_PLL_SSCG_REG_1);
  putreg32(LIBERO_SETTING_MSS_SSCG_REG_2, MPFS_IOSCB_MSS_PLL_SSCG_REG_2);
  putreg32(LIBERO_SETTING_MSS_SSCG_REG_3, MPFS_IOSCB_MSS_PLL_SSCG_REG_3);

  /* PLL phase register */

  putreg32(LIBERO_SETTING_MSS_PLL_PHADJ, MPFS_IOSCB_MSS_PLL_PHADJ);

  /* Write powerdown_int_b = 1, start up the PLL with new parameters
   * and wait for it to lock.
   */

  mss_mux_pre_mss_pll_config();

  putreg32(LIBERO_SETTING_MSS_PLL_CTRL | 0x01, MPFS_IOSCB_MSS_PLL_CTRL);

  /* Start up the PLL with new parameters. Wait for it to lock. */

  while (!(getreg32(MPFS_IOSCB_MSS_PLL_CTRL) & PLL_CTRL_LOCK_BIT) &&
        --retries);

  DEBUGASSERT(retries > 0);

  /* Processor enables all 4 PLL outputs. Set mssclk_mux_sel_int = 1 to
   * select the MSS PLL clock.
   */

  mpfs_mux_post_mss_pll_config();

  return 0;
}

/****************************************************************************
 * Name: mpfs_sgmii_setup
 *
 * Description:
 *   Setup the Serial Gigabit Media-Independent Interface (SGMII)
 *
 ****************************************************************************/

static void mpfs_sgmii_setup(void)
{
  putreg32(((0x01 << 8) | 0x1), MPFS_CFG_DDR_SGMII_PHY_SOFT_RESET_DDR_PHY);
  putreg32(0x1, MPFS_CFG_DDR_SGMII_PHY_SOFT_RESET_DDR_PHY);

  putreg32(LIBERO_SETTING_SGMII_MODE & ~(1 << 22),
           MPFS_CFG_DDR_SGMII_PHY_SGMII_MODE);
  putreg32(LIBERO_SETTING_CH0_CNTL, MPFS_CFG_DDR_SGMII_PHY_CH0_CNTL);
  putreg32(LIBERO_SETTING_CH1_CNTL, MPFS_CFG_DDR_SGMII_PHY_CH1_CNTL);
  putreg32(LIBERO_SETTING_RECAL_CNTL, MPFS_CFG_DDR_SGMII_PHY_RECAL_CNTL);
  putreg32(LIBERO_SETTING_CLK_CNTL, MPFS_CFG_DDR_SGMII_PHY_CLK_CNTL);
  putreg32(LIBERO_SETTING_SPARE_CNTL, MPFS_CFG_DDR_SGMII_PHY_SPARE_CNTL);
  putreg32(LIBERO_SETTING_PLL_CNTL, MPFS_CFG_DDR_SGMII_PHY_PLL_CNTL);

  /* Enable the Bank controller:
   *  - Set soft reset on IP to load RPC to SCB regs (dynamic mode)
   *  - Bring the sgmii bank controller out of reset =- ioscb_bank_ctrl_sgmii
   */

  putreg32(0x01, MPFS_IOSCB_BANK_CNTL_SGMII_SOFT_RESET);

  /* Check the IO_EN signal here. This is an output from the bank controller
   * power detector, which are turned on using MSS_IO_EN.
   */

  while (!(getreg32(MPFS_CFG_DDR_SGMII_PHY_PVT_STAT) & (1 << 6)));

  /* IO power ramp wait time: after IOEN is received from power detectors
   * DDR and SGMII, extra time is required for voltage to ramp.
   */

  mpfs_wait_cycles(10);

  putreg32(0, MPFS_SYSREGSCB_MSS_RESET_CR);

  /* Reset SGMII DLL */

  putreg32(1 << 0, MPFS_IOSCB_DLL_SGMII_SOFT_RESET);

  /* SGMII Lane 01 and 23 soft-reset */

  putreg32(1, MPFS_IOSCB_SGMII_LANE01_SOFT_RESET);
  putreg32(1, MPFS_IOSCB_SGMII_LANE23_SOFT_RESET);

  putreg32(SGMII_REG_PVT_SOFT_RESET_PERIPH | 0x7f,
           MPFS_CFG_DDR_SGMII_PHY_DYN_CNTL);
  putreg32(0x7f, MPFS_CFG_DDR_SGMII_PHY_DYN_CNTL);

  putreg32(1, MPFS_IOSCB_CALIB_SGMII_SOFT_RESET);
  putreg32(0, MPFS_IOSCB_CALIB_SGMII_SOFT_RESET);

  /* Verify calibration */

  while (!(getreg32(MPFS_CFG_DDR_SGMII_PHY_PVT_STAT) & (1 << 14)));

  modifyreg32(MPFS_CFG_DDR_SGMII_PHY_PVT_STAT, 0, 0x40000000);
  modifyreg32(MPFS_IOSCB_CALIB_SGMII_IOC_REG0, 0,
              SGMII_IOC_REG0_REG_CALIB_LOCK);

  mpfs_sgmii_mux_config();
  mpfs_sgmii_pll_config_scb();

  /* SGMII wait for MSS lock */

  while (!((getreg32(MPFS_CFG_DDR_SGMII_PHY_PLL_CNTL) &
         SGMII_ARO_PLL0_LOCK)));

  /* SGMII wait for DLL lock */

  while (!((getreg32(MPFS_CFG_DDR_SGMII_PHY_RECAL_CNTL) & (1 << 23))));

  modifyreg32(MPFS_SYSREG_SUBBLK_CLOCK_CR, 0,
              SYSREG_SUBBLK_CLOCK_CR_MAC0 | SYSREG_SUBBLK_CLOCK_CR_MAC1);

  modifyreg32(MPFS_SYSREG_SOFT_RESET_CR, SYSREG_SOFT_RESET_CR_MAC0 |
              SYSREG_SOFT_RESET_CR_MAC1, 0);

  /* SGMII gigabit mode enable */

  putreg32((1 << 10) | (1 << 11), MPFS_GEM0_NETWORK_CONFIG);
  putreg32((1 << 10) | (1 << 11), MPFS_GEM1_NETWORK_CONFIG);

  /* Determine silicon variant from generated sro_dll_90_code */

  uint32_t sro_dll_90_code = ((getreg32(MPFS_CFG_DDR_SGMII_PHY_RECAL_CNTL) \
                               >> 16) & 0x7f);
  uint32_t silicon_variant;

  /* Post rev B silicon */

  if (getreg32(MPFS_CFG_DDR_SGMII_PHY_SPARE_STAT) & (1 << 31))
    {
      silicon_variant = PART_REVC_OR_LATER;
      mpfs_set_early_late_thresholds(
                                LATE_EYE_WIDTH_PART_REVC_OR_LATER_PRE_TEST,
                                EARLY_EYE_WIDTH_PART_REVC_OR_LATER_PRE_TEST);
    }
  else
    {
      if (sro_dll_90_code < MIN_DLL_90_CODE_VALUE_INDICATING_TT_PART_REVB)
        {
          silicon_variant = SS_PART_REVB;
          mpfs_set_early_late_thresholds(LATE_EYE_WIDTH_SS_PART_REVB,
                                         EARLY_EYE_WIDTH_SS_PART_REVB);
        }
      else
        {
          silicon_variant = TT_PART_REVB;
          mpfs_set_early_late_thresholds(LATE_TT_PART_REVB,
                                         EARLY_TT_PART_REVB);
        }
    }

  /* DLL soft reset                   - Already configured
   * PVT soft reset                   - Already configured
   * Bank controller soft reset       - Already configured
   * CLKMUX soft reset                - Already configured
   * Lane0 soft reset                 - must be soft reset here
   * Lane1 soft reset                 - must be soft reset here
   */

  putreg32(SGMII_REG_LANE1_SOFT_RESET_PERIPH |
           SGMII_REG_LANE0_SOFT_RESET_PERIPH | 0x7f,
           MPFS_CFG_DDR_SGMII_PHY_DYN_CNTL);
  putreg32(0x7f, MPFS_CFG_DDR_SGMII_PHY_DYN_CNTL);

  if (silicon_variant == PART_REVC_OR_LATER)
    {
      mpfs_wait_cycles(0xfff);

      if ((getreg32(MPFS_CFG_DDR_SGMII_PHY_SPARE_STAT) & ARO_REF_PCODE_MASK)
          <= ARO_REF_PCODE_REVC_THRESHOLD)
        {
          /* Need to adjust eye values */

          mpfs_set_early_late_thresholds(LATE_EYE_WIDTH_PART_REVC_OR_LATER,
                                         EARLY_EYE_WIDTH_PART_REVC_OR_LATER);

          /* Now reset the channels */

          putreg32(SGMII_REG_LANE1_SOFT_RESET_PERIPH |
                   SGMII_REG_LANE0_SOFT_RESET_PERIPH | 0x7f,
                   MPFS_CFG_DDR_SGMII_PHY_DYN_CNTL);
          putreg32(0x7f, MPFS_CFG_DDR_SGMII_PHY_DYN_CNTL);
        }
    }

  mpfs_pll_config();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_clock_init
 *
 * Description:
 *   Setup the system clocks
 *
 ****************************************************************************/

void mpfs_clockconfig(void)
{
  mpfs_set_rtc_divisor();

  putreg32(MPFS_SCB_ACCESS_CONFIG, MPFS_IOSCBCFG_TIMER);

  putreg32(0x01, MPFS_SYSREG_DFIAPB_CR);

  putreg32((0x3f << 16) | (0x1f << 8), MPFS_CFG_DDR_SGMII_PHY_STARTUP);
  putreg32(SGMII_REG_PVT_SOFT_RESET_PERIPH | 0x7f,
           MPFS_CFG_DDR_SGMII_PHY_DYN_CNTL);

  /* DCE:111, CORE_UP:1, FLASH_VALID:0, mss_io_en:0 */

  putreg32(MSSIO_CONTROL_CR_MSS_DCE | MSSIO_CONTROL_CR_MSS_CORE_UP,
           MPFS_SYSREGSCB_MSSIO_CONTROL_CR);

  mpfs_wait_cycles(10);

  /* DCE:000, CORE_UP:1, FLASH_VALID:0, mss_io_en:0 */

  putreg32(MSSIO_CONTROL_CR_MSS_CORE_UP, MPFS_SYSREGSCB_MSSIO_CONTROL_CR);

  mpfs_wait_cycles(10);

  /* DCE:000, CORE_UP:1, FLASH_VALID:1, mss_io_en:0 */

  putreg32(MSSIO_CONTROL_CR_MSS_CORE_UP | MSSIO_CONTROL_CR_MSS_FLASH_VALID,
           MPFS_SYSREGSCB_MSSIO_CONTROL_CR);

  mpfs_wait_cycles(10);

  /* DCE:000, CORE_UP:1, FLASH_VALID:1, mss_io_en:1  */

  putreg32(MSSIO_CONTROL_CR_MSS_CORE_UP | MSSIO_CONTROL_CR_MSS_FLASH_VALID |
           MSSIO_CONTROL_CR_MSS_IO_EN, MPFS_SYSREGSCB_MSSIO_CONTROL_CR);

  /* Setup SGMII */

  mpfs_sgmii_setup();

  /* Setup the MSS PLL */

  mpfs_pll_config();
}

/****************************************************************************
 * Name: mpfs_get_cpuclk
 ****************************************************************************/

uint64_t mpfs_get_cpuclk(void)
{
  return g_cpu_clock;
}

/****************************************************************************
 * Name: mpfs_get_pll0clk
 ****************************************************************************/

#ifndef CONFIG_MPFS_WITH_QEMU
uint64_t mpfs_get_pll0clk(void)
{
  return 0;
}
#endif
