/****************************************************************************
 * arch/arm/src/imxrt/imxrt_clockconfig_ver3.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM_SRC_IMXRT_IMXRT_CLOCKCONFIG_VER3_H
#define __ARCH_ARM_SRC_IMXRT_IMXRT_CLOCKCONFIG_VER3_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include "hardware/rt118x/imxrt118x_ccm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CCM_CLOCK_ROOT_DISABLE \
  { .action = CCM_CLOCK_ROOT_ACTION_DISABLE }
#define CCM_CLOCK_ROOT_IGNORE \
  { .action = CCM_CLOCK_ROOT_ACTION_IGNORE }

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum ccm_clock_root_action_e
{
  CCM_CLOCK_ROOT_ACTION_DISABLE = 0,
  CCM_CLOCK_ROOT_CONFIGURE,
  CCM_CLOCK_ROOT_ACTION_IGNORE
};

struct ccm_clock_root
{
  uint8_t action;
  uint8_t div;
  uint8_t mux;
};

struct ccm_clock_root_config_s
{
  union
  {
    struct
    {
      struct ccm_clock_root m7_clk_root;
      struct ccm_clock_root m33_clk_root;
      struct ccm_clock_root edgelock_clk_root;
      struct ccm_clock_root bus_aon_clk_root;
      struct ccm_clock_root bus_wakeup_clk_root;
      struct ccm_clock_root wakeup_axi_clk_root;
      struct ccm_clock_root swo_trace_clk_root;
      struct ccm_clock_root m33_systick_clk_root;
      struct ccm_clock_root m7_systick_clk_root;
      struct ccm_clock_root flexio1_clk_root;
      struct ccm_clock_root flexio2_clk_root;
      struct ccm_clock_root lpit3_clk_root;
      struct ccm_clock_root lptimer1_clk_root;
      struct ccm_clock_root lptimer2_clk_root;
      struct ccm_clock_root lptimer3_clk_root;
      struct ccm_clock_root tpm2_clk_root;
      struct ccm_clock_root tpm4_clk_root;
      struct ccm_clock_root tpm5_clk_root;
      struct ccm_clock_root tpm6_clk_root;
      struct ccm_clock_root gpt1_clk_root;
      struct ccm_clock_root gpt2_clk_root;
      struct ccm_clock_root flexspi1_clk_root;
      struct ccm_clock_root flexspi2_clk_root;
      struct ccm_clock_root flexspi_slv_clk_root;
      struct ccm_clock_root can1_clk_root;
      struct ccm_clock_root can2_clk_root;
      struct ccm_clock_root can3_clk_root;
      struct ccm_clock_root lpuart0102_clk_root;
      struct ccm_clock_root lpuart0304_clk_root;
      struct ccm_clock_root lpuart0506_clk_root;
      struct ccm_clock_root lpuart0708_clk_root;
      struct ccm_clock_root lpuart0910_clk_root;
      struct ccm_clock_root lpuart1112_clk_root;
      struct ccm_clock_root lpi2c0102_clk_root;
      struct ccm_clock_root lpi2c0304_clk_root;
      struct ccm_clock_root lpi2c0506_clk_root;
      struct ccm_clock_root lpspi0102_clk_root;
      struct ccm_clock_root lpspi0304_clk_root;
      struct ccm_clock_root lpspi0506_clk_root;
      struct ccm_clock_root i3c1_clk_root;
      struct ccm_clock_root i3c2_clk_root;
      struct ccm_clock_root usdhc1_clk_root;
      struct ccm_clock_root usdhc2_clk_root;
      struct ccm_clock_root semc_clk_root;
      struct ccm_clock_root adc1_clk_root;
      struct ccm_clock_root adc2_clk_root;
      struct ccm_clock_root acmp_clk_root;
      struct ccm_clock_root ecat_clk_root;
      struct ccm_clock_root enet_clk_root;
      struct ccm_clock_root tmr_1588_clk_root;
      struct ccm_clock_root netc_clk_root;
      struct ccm_clock_root mac0_clk_root;
      struct ccm_clock_root mac1_clk_root;
      struct ccm_clock_root mac2_clk_root;
      struct ccm_clock_root mac3_clk_root;
      struct ccm_clock_root mac4_clk_root;
      struct ccm_clock_root serdes0_clk_root;
      struct ccm_clock_root serdes1_clk_root;
      struct ccm_clock_root serdes2_clk_root;
      struct ccm_clock_root serdes0_1g_clk_root;
      struct ccm_clock_root serdes1_1g_clk_root;
      struct ccm_clock_root serdes2_1g_clk_root;
      struct ccm_clock_root xcelbusx_clk_root;
      struct ccm_clock_root xriocu4_clk_root;
      struct ccm_clock_root mctrl_clk_root;
      struct ccm_clock_root sai1_clk_root;
      struct ccm_clock_root sai2_clk_root;
      struct ccm_clock_root sai3_clk_root;
      struct ccm_clock_root sai4_clk_root;
      struct ccm_clock_root spdif_clk_root;
      struct ccm_clock_root asrc_clk_root;
      struct ccm_clock_root mic_clk_root;
      struct ccm_clock_root cko1_clk_root;
      struct ccm_clock_root cko2_clk_root;
    };
    struct ccm_clock_root clock_root[CCM_CR_COUNT];
  };
};

struct ccm_arm_pll
{
  uint8_t post_div;
  uint8_t loop_div;
};

struct ccm_sys_pll1
{
  uint32_t enable;
  uint8_t div;
  uint32_t num;
  uint32_t denom;
};

struct ccm_sys_pll2
{
  uint32_t mfd;
  uint32_t ss;
  uint8_t ss_enable;
  uint16_t ss_stop;
  uint16_t ss_step;
  uint32_t pfd0;
  uint32_t pfd1;
  uint32_t pfd2;
  uint32_t pfd3;
};

struct ccm_sys_pll3
{
  uint32_t pfd0;
  uint32_t pfd1;
  uint32_t pfd2;
  uint32_t pfd3;
};

struct clock_configuration_s
{
  struct ccm_clock_root_config_s ccm;
  struct ccm_arm_pll             arm_pll;
  struct ccm_sys_pll1            sys_pll1;
  struct ccm_sys_pll2            sys_pll2;
  struct ccm_sys_pll3            sys_pll3;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern const struct clock_configuration_s g_initial_clkconfig;

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LPCG helpers so the shared code can enable/disable peripheral clock gates
 * with the same names as on other i.MX RT families.
 */

#define imxrt_clockall_lpuart1()    imxrt_ccm_gate_on(CCM_LPCG_LPUART1, true)
#define imxrt_clockall_lpuart2()    imxrt_ccm_gate_on(CCM_LPCG_LPUART2, true)
#define imxrt_clockall_lpuart3()    imxrt_ccm_gate_on(CCM_LPCG_LPUART3, true)
#define imxrt_clockall_lpuart4()    imxrt_ccm_gate_on(CCM_LPCG_LPUART4, true)
#define imxrt_clockall_lpuart5()    imxrt_ccm_gate_on(CCM_LPCG_LPUART5, true)
#define imxrt_clockall_lpuart6()    imxrt_ccm_gate_on(CCM_LPCG_LPUART6, true)
#define imxrt_clockall_lpuart7()    imxrt_ccm_gate_on(CCM_LPCG_LPUART7, true)
#define imxrt_clockall_lpuart8()    imxrt_ccm_gate_on(CCM_LPCG_LPUART8, true)
#define imxrt_clockall_lpuart9()    imxrt_ccm_gate_on(CCM_LPCG_LPUART9, true)
#define imxrt_clockall_lpuart10()   imxrt_ccm_gate_on(CCM_LPCG_LPUART10, true)
#define imxrt_clockall_lpuart11()   imxrt_ccm_gate_on(CCM_LPCG_LPUART11, true)
#define imxrt_clockall_lpuart12()   imxrt_ccm_gate_on(CCM_LPCG_LPUART12, true)

#define imxrt_clockall_xbar1()      imxrt_ccm_gate_on(CCM_LPCG_XBAR1, true)
#define imxrt_clockall_xbar2()      imxrt_ccm_gate_on(CCM_LPCG_XBAR2, true)
#define imxrt_clockall_xbar3()      imxrt_ccm_gate_on(CCM_LPCG_XBAR3, true)

#define imxrt_clockall_gpio1()      imxrt_ccm_gate_on(CCM_LPCG_GPIO1, true)
#define imxrt_clockall_gpio2()      imxrt_ccm_gate_on(CCM_LPCG_GPIO2, true)
#define imxrt_clockall_gpio3()      imxrt_ccm_gate_on(CCM_LPCG_GPIO3, true)
#define imxrt_clockall_gpio4()      imxrt_ccm_gate_on(CCM_LPCG_GPIO4, true)
#define imxrt_clockall_gpio5()      imxrt_ccm_gate_on(CCM_LPCG_GPIO5, true)
#define imxrt_clockall_gpio6()      imxrt_ccm_gate_on(CCM_LPCG_GPIO6, true)

#define imxrt_clockall_ocotp_ctrl() imxrt_ccm_gate_on(CCM_LPCG_OCOTP, true)

#define imxrt_clockall_gpt_bus()    imxrt_ccm_gate_on(CCM_LPCG_GPT1, true)
#define imxrt_clockall_gpt2_bus()   imxrt_ccm_gate_on(CCM_LPCG_GPT2, true)

#define imxrt_clockall_pwm1()       imxrt_ccm_gate_on(CCM_LPCG_PWM1, true)
#define imxrt_clockall_pwm2()       imxrt_ccm_gate_on(CCM_LPCG_PWM2, true)
#define imxrt_clockall_pwm3()       imxrt_ccm_gate_on(CCM_LPCG_PWM3, true)
#define imxrt_clockall_pwm4()       imxrt_ccm_gate_on(CCM_LPCG_PWM4, true)

#define imxrt_clockall_lpi2c1()     imxrt_ccm_gate_on(CCM_LPCG_LPI2C1, true)
#define imxrt_clockall_lpi2c2()     imxrt_ccm_gate_on(CCM_LPCG_LPI2C2, true)
#define imxrt_clockall_lpi2c3()     imxrt_ccm_gate_on(CCM_LPCG_LPI2C3, true)
#define imxrt_clockall_lpi2c4()     imxrt_ccm_gate_on(CCM_LPCG_LPI2C4, true)
#define imxrt_clockall_lpi2c5()     imxrt_ccm_gate_on(CCM_LPCG_LPI2C5, true)
#define imxrt_clockall_lpi2c6()     imxrt_ccm_gate_on(CCM_LPCG_LPI2C6, true)

#define imxrt_clockoff_lpi2c1()     imxrt_ccm_gate_on(CCM_LPCG_LPI2C1, false)
#define imxrt_clockoff_lpi2c2()     imxrt_ccm_gate_on(CCM_LPCG_LPI2C2, false)
#define imxrt_clockoff_lpi2c3()     imxrt_ccm_gate_on(CCM_LPCG_LPI2C3, false)
#define imxrt_clockoff_lpi2c4()     imxrt_ccm_gate_on(CCM_LPCG_LPI2C4, false)
#define imxrt_clockoff_lpi2c5()     imxrt_ccm_gate_on(CCM_LPCG_LPI2C5, false)
#define imxrt_clockoff_lpi2c6()     imxrt_ccm_gate_on(CCM_LPCG_LPI2C6, false)

#define imxrt_clockall_lpspi1()     imxrt_ccm_gate_on(CCM_LPCG_LPSPI1, true)
#define imxrt_clockall_lpspi2()     imxrt_ccm_gate_on(CCM_LPCG_LPSPI2, true)
#define imxrt_clockall_lpspi3()     imxrt_ccm_gate_on(CCM_LPCG_LPSPI3, true)
#define imxrt_clockall_lpspi4()     imxrt_ccm_gate_on(CCM_LPCG_LPSPI4, true)
#define imxrt_clockall_lpspi5()     imxrt_ccm_gate_on(CCM_LPCG_LPSPI5, true)
#define imxrt_clockall_lpspi6()     imxrt_ccm_gate_on(CCM_LPCG_LPSPI6, true)

#define imxrt_clockoff_lpspi1()     imxrt_ccm_gate_on(CCM_LPCG_LPSPI1, false)
#define imxrt_clockoff_lpspi2()     imxrt_ccm_gate_on(CCM_LPCG_LPSPI2, false)
#define imxrt_clockoff_lpspi3()     imxrt_ccm_gate_on(CCM_LPCG_LPSPI3, false)
#define imxrt_clockoff_lpspi4()     imxrt_ccm_gate_on(CCM_LPCG_LPSPI4, false)
#define imxrt_clockoff_lpspi5()     imxrt_ccm_gate_on(CCM_LPCG_LPSPI5, false)
#define imxrt_clockoff_lpspi6()     imxrt_ccm_gate_on(CCM_LPCG_LPSPI6, false)

#define imxrt_clockall_usboh3()     imxrt_ccm_gate_on(CCM_LPCG_USB, true)
#define imxrt_clockoff_usboh3()     imxrt_ccm_gate_on(CCM_LPCG_USB, false)
#define imxrt_clockrun_usboh3()     imxrt_ccm_gate_on(CCM_LPCG_USB, true)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Initialise the RT1180 clock tree. */

void imxrt_clockconfig(void);

/* Program a clock root's MUX + DIV using a clock source name from
 * ccm_clock_name_e.  Returns OK, or -EINVAL if the (root, src) pair is
 * not valid.
 */

int imxrt_ccm_configure_root_clock(int root, int src, uint32_t div);

/* Enable or disable a low-power clock gate (LPCG). */

int imxrt_ccm_gate_on(int gate, bool enabled);

/* Compatibility shim for existing callers that use CCM_LPCG_DIR_ON /
 * CCM_LPCG_DIR_OFF as the value argument.
 */

void imxrt_periphclk_configure(unsigned int index, unsigned int value);

/* Return the frequency in Hz of the named clock source (OSC / PLL / PFD)
 * or zero if unknown.
 */

int imxrt_get_clock(int clkname, uint32_t *frequency);

/* Return the frequency in Hz of the selected clock root (after MUX and
 * DIV are applied).  Signature matches RT117x's imxrt_get_rootclock() so
 * that shared drivers do not need to distinguish between families.
 */

int imxrt_get_rootclock(uint32_t clkroot, uint32_t *frequency);

#endif /* __ARCH_ARM_SRC_IMXRT_IMXRT_CLOCKCONFIG_VER3_H */
