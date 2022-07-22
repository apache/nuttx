/****************************************************************************
 * arch/arm/src/s32k3xx/s32k3xx_clockconfig.h
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

/* Copyright 2022 NXP */

#ifndef __ARCH_ARM_SRC_S32K3XX_S32K3XX_CLKCONFIG_H
#define __ARCH_ARM_SRC_S32K3XX_S32K3XX_CLKCONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "s32k3xx_clocknames.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Nominal frequencies of internal clocks */

#define CGM_FIRC_LOWRANGE_FREQUENCY   3000000  /* 3MHz */

/* Only possible using HSE_B.CONFIG_REG_GPR[FIRC_DIV_SEL] */

#define CGM_FIRC_HIGHRANGE_FREQUENCY 48000000  /* 48MHz */
#define CGM_SIRC_FREQUENCY0             32000  /* 32kHz */

#define NUMBER_OF_TCLK_INPUTS        3

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Clock Configuration ******************************************************/

enum cgm_system_clock_type_e
{
  CGM_SYSTEM_CLOCK_CORE,               /* Core clock  */
  CGM_SYSTEM_CLOCK_AIPS_PLAT_CLK,      /* medium-speed peripheral clock */
  CGM_SYSTEM_CLOCK_AIPS_SLOW_CLK,      /* slow-speed peripheral clock */
  CGM_SYSTEM_CLOCK_HSE_CLK,            /* HSE clock */
  CGM_SYSTEM_CLOCK_DCM_CLK,            /* DCM clock */
  CGM_SYSTEM_CLOCK_LBIST_CLK,          /* LBIST clock */
  CGM_SYSTEM_CLOCK_QSPI_MEM_CLK,       /* QSPI clock */
};

enum cgm_csc_src_e
{
  CGM_CLK_SRC_FIRC             = 0,
  CGM_CLK_SRC_SIRC             = 1,
  CGM_CLK_SRC_FXOSC            = 2,
  CGM_CLK_SRC_SXOSC            = 4,
  CGM_CLK_SRC_PLL_PHI0_CLK     = 8,
  CGM_CLK_SRC_PLL_PHI1_CLK     = 9,
  CGM_CLK_SRC_CORE_CLK         = 16,
  CGM_CLK_SRC_HSE_CLK          = 19,
  CGM_CLK_SRC_AIPS_PLAT_CLK    = 22,
  CGM_CLK_SRC_AIPS_SLOW_CLK    = 23,
  CGM_CLK_SRC_EMAC_RMII_TX_CLK = 24,
  CGM_CLK_SRC_EMAC_RX_CLK      = 25
};

/* These structure are used to define the clock configuration. */

/* CGM SIRC clock configuration */

enum cgm_sirc_range_e
{
  CGM_FIRC_RANGE_32K = 1,              /* Slow IRC is trimmed to 32kHz */
};

struct cgm_sirc_config_s
{
  enum cgm_sirc_range_e range;         /* Slow IRC frequency range */
};

/* CGM FIRC clock configuration */

enum cgm_firc_range_e
{
  CGM_FIRC_RANGE_HIGH = 0,             /* Slow IRC high range clock (48 MHz). */
};

enum cgm_firc_clock_div_e
{
  CGM_CLOCK_DISABLE   = 0,       /* Clock output is disabled */
  CGM_CLOCK_DIV_BY_1  = 1,       /* Divided by 1 */
  CGM_CLOCK_DIV_BY_2  = 2,       /* Divided by 2 */
  CGN_CLOCK_DIV_BY_16 = 5,       /* Divided by 16 */
};

struct cgm_firc_config_s
{
  enum cgm_firc_range_e range;         /* Fast IRC frequency range */
  enum cgm_firc_clock_div_e div;       /* HSE FIRC DIV SEL */
};

/* CGM SOSC Clock Configuration */

enum cgm_mux_div_e
{
  CGM_MUX_DISABLE   = 0,       /* Clock output is disabled */
  CGM_MUX_DIV_BY_1  = 1,       /* Divided by 1 */
  CGM_MUX_DIV_BY_2  = 2,       /* Divided by 2 */
  CGM_MUX_DIV_BY_3  = 3,       /* Divided by 3 */
  CGM_MUX_DIV_BY_4  = 4,       /* Divided by 4 */
  CGM_MUX_DIV_BY_5  = 5,       /* Divided by 5 */
  CGM_MUX_DIV_BY_6  = 6,       /* Divided by 6 */
  CGM_MUX_DIV_BY_7  = 7,       /* Divided by 7 */
  CGM_MUX_DIV_BY_8  = 8,       /* Divided by 8 */
};

enum cgm_mux_div_slow_e
{
  CGM_MUX_SLOW_DISABLE   = 0,       /* Clock output is disabled */
  CGM_MUX_DIV_SLOW_BY_1  = 1,       /* Divided by 1 */
  CGM_MUX_DIV_SLOW_BY_2  = 2,       /* Divided by 2 */
  CGM_MUX_DIV_SLOW_BY_3  = 3,       /* Divided by 3 */
  CGM_MUX_DIV_SLOW_BY_4  = 4,       /* Divided by 4 */
  CGM_MUX_DIV_SLOW_BY_5  = 5,       /* Divided by 5 */
  CGM_MUX_DIV_SLOW_BY_6  = 6,       /* Divided by 6 */
  CGM_MUX_DIV_SLOW_BY_7  = 7,       /* Divided by 7 */
  CGM_MUX_DIV_SLOW_BY_8  = 8,       /* Divided by 8 */
  CGM_MUX_DIV_SLOW_BY_9  = 8,       /* Divided by 9 */
  CGM_MUX_DIV_SLOW_BY_10  = 8,      /* Divided by 10 */
  CGM_MUX_DIV_SLOW_BY_11  = 8,      /* Divided by 11 */
  CGM_MUX_DIV_SLOW_BY_12  = 8,      /* Divided by 12 */
};

enum cgm_scs_source_e
{
  CGM_SCS_SOURCE_FIRC      = 0,
  CGM_SCS_SOURCE_PLL_PHI0  = 1
};

struct cgm_mux_config_s
{
  enum cgm_mux_div_e div;      /* Asynchronous peripheral source */
  bool trigger;                /* true: Common Trigger Divider update */
};

struct cgm_mux_slow_config_s
{
  enum cgm_mux_div_slow_e div; /* Asynchronous peripheral source */
  bool trigger;                /* true: Common Trigger Divider update */
};

struct cgm_mux_src_config_s
{
  enum cgm_csc_src_e source;  /* MUX_X Clock src */
  enum cgm_mux_div_e div;     /* Note div ranges from 1..4 */
};

struct cgm_scs_config_s
{
  enum cgm_scs_source_e        scs_source;
  struct cgm_mux_config_s      core_clk;
  struct cgm_mux_config_s      aips_plat_clk;
  struct cgm_mux_slow_config_s aips_slow_clk;
  struct cgm_mux_config_s      hse_clk;
  struct cgm_mux_config_s      dcm_clk;
  struct cgm_mux_config_s      lbist_clk;
#ifdef CONFIG_S32K3XX_QSPI
  struct cgm_mux_config_s      qspi_mem_clk;
#endif
  struct cgm_mux_src_config_s    mux_1_stm0;
  struct cgm_mux_src_config_s    mux_3;
  struct cgm_mux_src_config_s    mux_4;
#ifdef CONFIG_S32K3XX_ENET
  struct cgm_mux_src_config_s    mux_7_emac_rx;
  struct cgm_mux_src_config_s    mux_8_emac_tx;
  struct cgm_mux_src_config_s    mux_9_emac_ts;
#endif
#ifdef CONFIG_S32K3XX_QSPI
  struct cgm_mux_src_config_s    mux_10_qspi_sfck;
#endif
};

/* CGM PLL Clock Configuration */

enum scg_spll_monitor_mode_e
{
  CGM_SPLL_MONITOR_DISABLE = 0,        /* Monitor disable */
  CGM_SPLL_MONITOR_INT     = 1,        /* Interrupt when system PLL error detected */
  CGM_SPLL_MONITOR_RESET   = 2         /* Reset when system PLL error detected */
};

enum cgm_pll_core_mode_e
{
  CGM_PLL_INTEGER_MODE    = 0,
  CGM_PLL_FRACTIONAL_MODE = 1,
  CGM_PLL_SSCG_MODE       = 2
};

enum cgm_pll_sigma_delta_e
{
  CGM_PLL_SIGMA_DELTA         = 0,
  CGM_PLL_SIGMA_DELTA_ORDER_2 = 1,
  CGM_PLL_SIGMA_DELTA_ORDER_3 = 2
};

enum cgm_pll_postdiv_e
{
  CGM_PLL_POSTDIV_DISABLE   = 0,   /* Clock output is disabled */
  CGM_PLL_POSTDIV_BY_1  = 1,       /* Divided by 1 */
  CGM_PLL_POSTDIV_BY_2  = 2,       /* Divided by 2 */
  CGM_PLL_POSTDIV_BY_3  = 3,       /* Divided by 3 */
  CGM_PLL_POSTDIV_BY_4  = 4,       /* Divided by 4 */
  CGM_PLL_POSTDIV_BY_5  = 5,       /* Divided by 5 */
  CGM_PLL_POSTDIV_BY_6  = 6,       /* Divided by 6 */
  CGM_PLL_POSTDIV_BY_7  = 7,       /* Divided by 7 */
  CGM_PLL_POSTDIV_BY_8  = 8,       /* Divided by 8 */
  CGM_PLL_POSTDIV_BY_9  = 9,       /* Divided by 9 */
  CGM_PLL_POSTDIV_BY_10  = 10,     /* Divided by 10 */
  CGM_PLL_POSTDIV_BY_11  = 11,     /* Divided by 11 */
};

enum cgm_pll_phi_div_e
{
  CGM_PLL_PHI_DIV_DISABLE   = 0,   /* Clock output is disabled */
  CGM_PLL_PHI_DIV_BY_1  = 1,       /* Divided by 1 */
  CGM_PLL_PHI_DIV_BY_2  = 2,       /* Divided by 2 */
  CGM_PLL_PHI_DIV_BY_3  = 3,       /* Divided by 3 */
  CGM_PLL_PHI_DIV_BY_4  = 4,       /* Divided by 4 */
  CGM_PLL_PHI_DIV_BY_5  = 5,       /* Divided by 5 */
  CGM_PLL_PHI_DIV_BY_6  = 6,       /* Divided by 6 */
  CGM_PLL_PHI_DIV_BY_7  = 7,       /* Divided by 7 */
  CGM_PLL_PHI_DIV_BY_8  = 8,       /* Divided by 8 */
  CGM_PLL_PHI_DIV_BY_9  = 9,       /* Divided by 9 */
  CGM_PLL_PHI_DIV_BY_10  = 10,     /* Divided by 10 */
  CGM_PLL_PHI_DIV_BY_11  = 11,     /* Divided by 11 */
  CGM_PLL_PHI_DIV_BY_12  = 12,     /* Divided by 12 */
};

struct cgm_pll_config_s
{
    uint32_t modul_freq;
    uint32_t modul_depth;
    bool     core_pll_power;
    bool     modulation_type;          /* true: modulation spread below */
    enum     cgm_pll_sigma_delta_e sigma_delta;
    bool     enable_dither;
    enum     cgm_pll_core_mode_e mode; /* Core PLL mode */
    uint8_t  prediv;                   /* PLL reference clock divider */
    uint8_t  mult;                     /* PLL multiplier */
    enum     cgm_pll_postdiv_e postdiv;
    enum     cgm_pll_phi_div_e phi0;
    enum     cgm_pll_phi_div_e phi1;
};

enum cgm_clkout_div_e
{
  CGM_CLKOUT_DIV_DISABLE = 0,     /* Clock output is disabled */
  CGM_CLKOUT_DIV_BY_1  = 1,       /* Divided by 1 */
  CGM_CLKOUT_DIV_BY_2  = 2,       /* Divided by 2 */
  CGM_CLKOUT_DIV_BY_3  = 3,       /* Divided by 3 */
  CGM_CLKOUT_DIV_BY_4  = 4,       /* Divided by 4 */
  CGM_CLKOUT_DIV_BY_5  = 5,       /* Divided by 5 */
  CGM_CLKOUT_DIV_BY_6  = 6,       /* Divided by 6 */
  CGM_CLKOUT_DIV_BY_7  = 7,       /* Divided by 7 */
  CGM_CLKOUT_DIV_BY_8  = 8,       /* Divided by 8 */
  CGM_CLKOUT_DIV_BY_9  = 9,       /* Divided by 9 */
  CGM_CLKOUT_DIV_BY_10  = 10,     /* Divided by 10 */
  CGM_CLKOUT_DIV_BY_11  = 11,     /* Divided by 11 */
  CGM_CLKOUT_DIV_BY_12  = 12,     /* Divided by 12 */
};

struct cgm_clkout_config_s
{
    enum cgm_csc_src_e    source;
    enum cgm_clkout_div_e div;
};

/* Overall CGM Configuration */

struct cgm_config_s
{
  struct cgm_sirc_config_s sirc;             /* Slow internal reference clock configuration */
  struct cgm_firc_config_s firc;             /* Fast internal reference clock configuration */
  struct cgm_scs_config_s scs;               /* System oscillator configuration */
  struct cgm_pll_config_s pll;               /* Phase locked loop configuration */
  struct cgm_clkout_config_s clkout;         /* Phase locked loop configuration */
};

/* PCC clock configuration */

struct peripheral_clock_config_s;      /* Forward reference */
struct pcc_config_s
{
  unsigned int count;                            /* Number of peripherals to be configured */
  const struct peripheral_clock_config_s *pclks; /* The peripheral clock configuration array */
};

/* Overall clock configuration */

struct clock_configuration_s
{
  struct cgm_config_s cgm;             /* CGM Clock configuration */
  struct pcc_config_s pcc;             /* PCC Clock configuration */
};

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: s32k3xx_clockconfig
 *
 * Description:
 *   Called to initialize the S32K3XX.  This does whatever setup is needed
 *   to put the MCU in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.  This function also performs
 *   other low-level chip as necessary.
 *
 * Input Parameters:
 *   clkcfg - Describes the new clock configuration
 *
 * Returned Value:
 *   Zero (OK) is returned a success;  A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int s32k3xx_clockconfig(const struct clock_configuration_s *clkcfg);

/****************************************************************************
 * Name: s32k3xx_get_coreclk
 *
 * Description:
 *   Return the current value of the CORE clock frequency.
 *
 * Input Parameters:
 *   None
 *
 * Returned Values:
 *   The current value of the CORE clock frequency.  Zero is returned on any
 *   failure.
 *
 ****************************************************************************/

uint32_t s32k3xx_get_coreclk(void);

/****************************************************************************
 * Name: s32k3xx_get_sysclk
 *
 * Description:
 *   Return the current value of an CGM system clock frequency, these clocks
 *   are used for core, platform, external and bus clock domains..
 *
 * Input Parameters:
 *   type - Identifies the system clock of interest
 *
 * Returned Values:
 *   The current value of the system clock frequency.  Zero is returned on
 *   any failure.
 *
 ****************************************************************************/

uint32_t s32k3xx_get_sysclk(enum cgm_system_clock_type_e type);

/****************************************************************************
 * Name: s32k3xx_get_freq
 *
 * Description:
 *    clock frequency from a clock source.
 *
 * Input Parameters:
 *   clksrc - The requested clock source.
 *
 * Returned Value:
 *   The frequency of the requested clock source.
 *
 ****************************************************************************/

uint32_t s32k3xx_get_freq(enum clock_names_e clksrc);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_S32K3XX_S32K3XX_CLKCONFIG_H */
