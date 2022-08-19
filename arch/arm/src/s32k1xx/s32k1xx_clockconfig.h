/****************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_clockconfig.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Some of the definitions within this file derive from NXP sample code for
 * the S32K1xx MCUs.  That sample code has this licensing information:
 *
 *   Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 *   Copyright 2016-2018 NXP
 *   All rights reserved.
 *
 * THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_S32K1XX_S32K1XX_CLKCONFIG_H
#define __ARCH_ARM_SRC_S32K1XX_S32K1XX_CLKCONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#if defined(CONFIG_ARCH_CHIP_S32K11X)
#  include "s32k11x/s32k11x_clocknames.h"
#elif defined(CONFIG_ARCH_CHIP_S32K14X)
#  include "s32k14x/s32k14x_clocknames.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Nomial frequencies of internal clocks */

#define SCG_SIRC_LOWRANGE_FREQUENCY   2000000  /* 2MHz */
#define SCG_SIRC_HIGHRANGE_FREQUENCY  8000000  /* 8MHz */
#define SCG_FIRC_FREQUENCY0          48000000  /* 48MHz */

#define NUMBER_OF_TCLK_INPUTS        3

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Clock Configuration ******************************************************/

enum scg_system_clock_type_e
{
  SCG_SYSTEM_CLOCK_CORE,               /* Core clock  */
  SCG_SYSTEM_CLOCK_BUS,                /* BUS clock */
  SCG_SYSTEM_CLOCK_SLOW,               /* System slow clock */
  SCG_SYSTEM_CLOCK_MAX                 /* Max value */
};

enum scg_async_clock_type_e
{
  SCG_ASYNC_CLOCK_DIV1 = 0,            /* Clock divider 1 */
  SCG_ASYNC_CLOCK_DIV2 = 1,            /* Clock divider 2 */
  SCG_ASYNC_CLOCK_MAX  = 2,            /* Max value */
};

/* These structure are used to define the clock configuration. */

/* SCG SIRC clock configuration */

enum scg_sirc_range_e
{
  SCG_SIRC_RANGE_HIGH = 1,             /* Slow IRC high range clock (8 MHz). */
};

enum scg_async_clock_div_e
{
  SCG_ASYNC_CLOCK_DISABLE   = 0,       /* Clock output is disabled */
  SCG_ASYNC_CLOCK_DIV_BY_1  = 1,       /* Divided by 1 */
  SCG_ASYNC_CLOCK_DIV_BY_2  = 2,       /* Divided by 2 */
  SCG_ASYNC_CLOCK_DIV_BY_4  = 3,       /* Divided by 4 */
  SCG_ASYNC_CLOCK_DIV_BY_8  = 4,       /* Divided by 8 */
  SCG_ASYNC_CLOCK_DIV_BY_16 = 5,       /* Divided by 16 */
  SCG_ASYNC_CLOCK_DIV_BY_32 = 6,       /* Divided by 32 */
  SCG_ASYNC_CLOCK_DIV_BY_64 = 7        /* Divided by 64 */
};

struct scg_sirc_config_s
{
  enum scg_sirc_range_e range;         /* Slow IRC frequency range */
  enum scg_async_clock_div_e div1;     /* Asynchronous peripheral source */
  enum scg_async_clock_div_e div2;     /* Asynchronous peripheral source */
  bool initialize;                     /* true: Initialize the SIRC module */
  bool stopmode;                       /* true: Enable SIRC in stop mode */
  bool lowpower;                       /* true: Enable SIRC in low power mode */
  bool locked;                         /* true: Lock SIRC Control Register */
};

/* SCG FIRC clock configuration */

enum scg_firc_range_e
{
  SCG_FIRC_RANGE_48M = 0,              /* Fast IRC is trimmed to 48MHz */
};

struct scg_firc_config_s
{
  enum scg_firc_range_e range;         /* Fast IRC frequency range */
  enum scg_async_clock_div_e div1;     /* Asynchronous peripheral source */
  enum scg_async_clock_div_e div2;     /* Asynchronous peripheral source */
  bool initialize;                     /* true: Initialize the FIRC module */
  bool stopmode;                       /* true: Enable FIRC in stop mode */
  bool lowpower;                       /* true: Enable FIRC in low power mode */
  bool regulator;                      /* true: Enable FIRC regulator */
  bool locked;                         /* true: Lock FIRC Control Register */
};

/* SCG SOSC Clock Configuration */

enum scg_sosc_monitor_mode_e
{
  SCG_SOSC_MONITOR_DISABLE = 0,        /* Monitor disable */
  SCG_SOSC_MONITOR_INT     = 1,        /* Interrupt when system OSC error detected */
  SCG_SOSC_MONITOR_RESET   = 2,        /* Reset when system OSC error detected */
};

enum scg_sosc_ext_ref_e
{
  SCG_SOSC_REF_EXT         = 0,        /* External reference clock requested */
  SCG_SOSC_REF_OSC         = 1,        /* Internal oscillator of OSC requested */
};

enum scg_sosc_gain_e
{
  SCG_SOSC_GAIN_LOW        = 0,        /* Configure crystal oscillator for low-power operation */
  SCG_SOSC_GAIN_HIGH       = 1,        /* Configure crystal oscillator for high-gain operation */
};

enum scg_sosc_range_e
{
  SCG_SOSC_RANGE_MID       = 2,        /* Medium frequency range selected for the crystal OSC (4 MHz to 8 MHz). */
  SCG_SOSC_RANGE_HIGH      = 3,        /* High frequency range selected for the crystal OSC (8 MHz to 40 MHz). */
};

struct scg_sosc_config_s
{
  enum scg_sosc_monitor_mode_e mode;   /* System OSC Clock monitor mode */
  enum scg_sosc_ext_ref_e extref;      /* System OSC External Reference Select */
  enum scg_sosc_gain_e gain;           /* System OSC high-gain operation */
  enum scg_sosc_range_e range;         /* System OSC frequency range */
  enum scg_async_clock_div_e div1;     /* Asynchronous peripheral source */
  enum scg_async_clock_div_e div2;     /* Asynchronous peripheral source */
  bool initialize;                     /* true: Initialize the System OSC module */
  bool stopmode;                       /* true: Enable System OSC in stop mode */
  bool lowpower;                       /* true: Enable System OSC in low power mode */
  bool locked;                         /* true: Lock System OSC Control Register */
};

#ifdef CONFIG_S32K1XX_HAVE_SPLL
/* SCG SPLL Clock Configuration */

enum scg_spll_monitor_mode_e
{
  SCG_SPLL_MONITOR_DISABLE = 0,        /* Monitor disable */
  SCG_SPLL_MONITOR_INT     = 1,        /* Interrupt when system PLL error detected */
  SCG_SPLL_MONITOR_RESET   = 2         /* Reset when system PLL error detected */
};

struct scg_spll_config_s
{
  enum scg_spll_monitor_mode_e mode;   /* Clock monitor mode selected */
  enum scg_async_clock_div_e div1;     /* Asynchronous peripheral source */
  enum scg_async_clock_div_e div2;     /* Asynchronous peripheral source */
  uint8_t prediv;                      /* PLL reference clock divider */
  uint8_t mult;                        /* System PLL multiplier */
  uint8_t src;                         /* System PLL source */
  bool initialize;                     /* true: Initialize or not the System PLL clock */
  bool stopmode;                       /* true: Enable System PLL clock in stop mode */
  bool locked;                         /* true: Lock System PLL clock Control Register */
};
#endif

/* SCG RTC Clock Configuration */

struct scg_rtc_config_s
{
  uint32_t clkin;                      /* RTC_CLKIN frequency */
  bool initialize;                     /* true: Initialize the RTC module */
};

/* SCG ClockOut Configuration */

enum scg_clockout_src_e
{
  SCG_CLOCKOUT_SRC_SCG_SLOW = 0,       /* SCG SLOW */
  SCG_CLOCKOUT_SRC_SOSC     = 1,       /* System OSC */
  SCG_CLOCKOUT_SRC_SIRC     = 2,       /* Slow IRC */
  SCG_CLOCKOUT_SRC_FIRC     = 3,       /* Fast IRC */
  SCG_CLOCKOUT_SRC_SPLL     = 6        /* System PLL */
};

struct scg_clockout_config_s
{
  enum scg_clockout_src_e source;      /* ClockOut source select */
  bool initialize;                     /* true: Initialize the ClockOut */
};

/* SCG Clock Mode Configuration */

enum scg_system_clock_src_e
{
  SCG_SYSTEM_CLOCK_SRC_SYS_OSC  = 1,   /* System OSC */
  SCG_SYSTEM_CLOCK_SRC_SIRC     = 2,   /* Slow IRC */
  SCG_SYSTEM_CLOCK_SRC_FIRC     = 3,   /* Fast IRC */
#ifdef CONFIG_S32K1XX_HAVE_SPLL
  SCG_SYSTEM_CLOCK_SRC_SYS_PLL  = 6,   /* System PLL */
#endif
  SCG_SYSTEM_CLOCK_SRC_NONE     = 255  /* MAX value */
};

struct scg_system_clock_config_s
{
  enum scg_system_clock_src_e src;     /* System clock source */
  uint8_t divslow;                     /* Slow clock divider, range=1..16 */
  uint8_t divbus;                      /* BUS clock divider, range=1..16 */
  uint8_t divcore;                     /* Core clock divider, range=1..16 */
};

struct scg_clock_mode_config_s
{
  struct scg_system_clock_config_s rccr; /* Run Clock Control configuration */
  struct scg_system_clock_config_s vccr; /* VLPR Clock Control configuration */
#ifdef CONFIG_S32K1XX_HAVE_HSRUN
  struct scg_system_clock_config_s hccr; /* HSRUN Clock Control configuration */
#endif
  enum scg_system_clock_src_e altclk;    /* Alternate clock used during initialization */
  bool initialize;                       /* true: Initialize the Clock Mode Configuration */
};

/* Overall SCG Configuration */

struct scg_config_s
{
  struct scg_sirc_config_s sirc;             /* Slow internal reference clock configuration */
  struct scg_firc_config_s firc;             /* Fast internal reference clock configuration */
  struct scg_sosc_config_s sosc;             /* System oscillator configuration */
#ifdef CONFIG_S32K1XX_HAVE_SPLL
  struct scg_spll_config_s spll;             /* System Phase locked loop configuration */
#endif
  struct scg_rtc_config_s rtc;               /* Real Time Clock configuration */
  struct scg_clockout_config_s clockout;     /* SCG ClockOut Configuration */
  struct scg_clock_mode_config_s clockmode;  /* SCG Clock Mode Configuration */
};

/* SIM ClockOut configuration */

enum sim_clkout_src_e
{
  SIM_CLKOUT_SEL_SYSTEM_SCG_CLKOUT     = 0,  /* SCG CLKOUT */
  SIM_CLKOUT_SEL_SYSTEM_SOSC_DIV2_CLK  = 2,  /* SOSC DIV2 CLK */
  SIM_CLKOUT_SEL_SYSTEM_SIRC_DIV2_CLK  = 4,  /* SIRC DIV2 CLK */
  SIM_CLKOUT_SEL_SYSTEM_FIRC_DIV2_CLK  = 6,  /* FIRC DIV2 CLK */
  SIM_CLKOUT_SEL_SYSTEM_HCLK           = 7,  /* HCLK */
  SIM_CLKOUT_SEL_SYSTEM_SPLL_DIV2_CLK  = 8,  /* SPLL DIV2 CLK */
  SIM_CLKOUT_SEL_SYSTEM_BUS_CLK        = 9,  /* BUS_CLK */
  SIM_CLKOUT_SEL_SYSTEM_LPO_128K_CLK   = 10, /* LPO_CLK 128 Khz */
  SIM_CLKOUT_SEL_SYSTEM_LPO_CLK        = 12, /* LPO_CLK as selected by SIM LPO CLK Select */
  SIM_CLKOUT_SEL_SYSTEM_RTC_CLK        = 14, /* RTC CLK as selected by SIM CLK 32 KHz Select */
#ifdef CONFIG_ARCH_CHIP_S32K14X
  SIM_CLKOUT_SEL_SYSTEM_SFIF_CLK_HYP   = 5,  /* SFIF_CLK_HYP */
  SIM_CLKOUT_SEL_SYSTEM_IPG_CLK        = 11, /* IPG_CLK */
  SIM_CLKOUT_SEL_SYSTEM_IPG_CLK_SFIF   = 13, /* IPG_CLK_SFIF */
  SIM_CLKOUT_SEL_SYSTEM_IPG_CLK_2XSFIF = 15  /* IP_CLK_2XSFIF */
#endif
};

struct sim_clock_out_config_s
{
  enum sim_clkout_src_e source;        /* SIM ClockOut source select */
  uint8_t divider;                     /* SIM ClockOut divide ratio, range 1..8 */
  bool initialize;                     /* true: Initialize the ClockOut clock */
  bool enable;                         /* true: Enable the ClockOut clock */
};

/* SIM LPO clock configuration */

enum sim_rtc_clk_sel_src_e
{
  SIM_RTCCLK_SEL_SOSCDIV1_CLK = 0,     /* SOSCDIV1 clock */
  SIM_RTCCLK_SEL_LPO_32K      = 1,     /* 32 kHz LPO clock */
  SIM_RTCCLK_SEL_RTC_CLKIN    = 2,     /* RTC_CLKIN clock */
  SIM_RTCCLK_SEL_FIRCDIV1_CLK = 3,     /* FIRCDIV1 clock */
};

enum sim_lpoclk_sel_src_e
{
  SIM_LPO_CLK_SEL_LPO_128K = 0,        /* 128 kHz LPO clock */
  SIM_LPO_CLK_SEL_NO_CLOCK = 1,        /* No clock */
  SIM_LPO_CLK_SEL_LPO_32K  = 2,        /* 32 kHz LPO clock which is divided by the 128 kHz LPO clock */
  SIM_LPO_CLK_SEL_LPO_1K   = 3,        /* 1 kHz LPO clock which is divided by the 128 kHz LPO clock */
};

struct sim_lpo_clock_config_s
{
  enum sim_rtc_clk_sel_src_e rtc_source; /* RTC_CLK source select */
  enum sim_lpoclk_sel_src_e  lpo_source; /* LPO clock source select */
  bool initialize;                       /* true: Initialize the LPO clock */
  bool lpo32k;                           /* MSCM Clock Gating Control enable */
  bool lpo1k;                            /* MSCM Clock Gating Control enable */
};

/* SIM TCLK clock configuration */

struct sim_tclk_config_s
{
  uint32_t tclkfreq[NUMBER_OF_TCLK_INPUTS]; /* TCLKx frequency */
  bool initialize;                          /* true: Initialize the TCLKx clock */
};

/* SIM clock gating control configuration */

struct sim_plat_gate_config_s
{
  bool initialize;                     /* true: Initialize the clock gating control */
  bool mscm;                           /* true: Enable MSCM Clock Gating Control */
  bool mpu;                            /* true: Enable MPU Clock Gating Control */
  bool dma;                            /* true: Enable DMA Clock Gating Control */
  bool erm;                            /* true: Enable ERM Clock Gating Control */
  bool eim;                            /* true: Enable EIM Clock Gating Control */
};

/* SIM trace clock configuration */

enum clock_trace_src_e
{
  CLOCK_TRACE_SRC_CORE_CLK = 0x0       /* Core clock */
};

struct sim_trace_clock_config_s
{
  enum clock_trace_src_e  source;       /* Trace clock select */
  uint8_t divider;                      /* Trace clock divider divisor, range 1..8 */
  bool initialize;                      /* true: Initialize the Trace clock */
  bool enable;                          /* true: Enable Trace clock divider */
  bool fraction;                        /* true: EnableTrace clock divider fraction */
};

#ifdef CONFIG_S32K1XX_HAVE_QSPI
/* SIM QSPI clock configuration */

struct sim_qspi_ref_clk_gating_s
{
  bool refclk;                          /* true: Enable QSPI internal reference clock gating */
};
#endif

/* Overall SIM clock configuration */

struct sim_clock_config_s
{
  struct sim_clock_out_config_s clockout;      /* Clock Out configuration */
  struct sim_lpo_clock_config_s lpoclk;        /* Low Power Clock configuration */
  struct sim_tclk_config_s tclk;               /* Platform Gate Clock configuration */
  struct sim_plat_gate_config_s platgate;      /* Platform Gate Clock configuration */
  struct sim_trace_clock_config_s traceclk;    /* Trace clock configuration */
#ifdef CONFIG_S32K1XX_HAVE_QSPI
  struct sim_qspi_ref_clk_gating_s qspirefclk; /* Qspi Reference Clock Gating */
#endif
};

/* PCC clock configuration */

struct peripheral_clock_config_s;      /* Forward reference */
struct pcc_config_s
{
  unsigned int count;                            /* Number of peripherals to be configured */
  const struct peripheral_clock_config_s *pclks; /* The peripheral clock configuration array */
};

/* PMC clock configuration */

struct pmc_lpo_clock_config_s
{
  int8_t trim;                         /* LPO trimming value */
  bool initialize;                     /* true: Initialize the LPO */
  bool enable;                         /* true: Enable the LPO */
};

struct pmc_config_s
{
  struct pmc_lpo_clock_config_s lpoclk; /* Low Power Clock configuration */
};

/* Overall clock configuration */

struct clock_configuration_s
{
  struct scg_config_s scg;             /* SCG Clock configuration */
  struct sim_clock_config_s sim;       /* SIM Clock configuration */
  struct pcc_config_s pcc;             /* PCC Clock configuration */
  struct pmc_config_s pmc;             /* PMC Clock configuration */
};

enum scg_system_clock_mode_e
{
  SCG_SYSTEM_CLOCK_MODE_CURRENT = 0,  /* Current mode. */
  SCG_SYSTEM_CLOCK_MODE_RUN     = 1,  /* Run mode. */
  SCG_SYSTEM_CLOCK_MODE_VLPR    = 2,  /* Very Low Power Run mode. */
  SCG_SYSTEM_CLOCK_MODE_HSRUN   = 3,  /* High Speed Run mode. */
  SCG_SYSTEM_CLOCK_MODE_NONE          /* MAX value. */
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
 * Name: s32k1xx_get_runmode
 *
 * Description:
 *   Get the current running mode.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The current running mode.
 *
 ****************************************************************************/

enum scg_system_clock_mode_e s32k1xx_get_runmode(void);

/****************************************************************************
 * Name: s32k1xx_set_runmode
 *
 * Description:
 *   Set the running mode.
 *
 * Input Parameters:
 *   next_run_mode - The next running mode.
 *
 * Returned Value:
 *   The current running mode.
 *
 ****************************************************************************/

enum scg_system_clock_mode_e s32k1xx_set_runmode(enum scg_system_clock_mode_e
  next_run_mode);

/****************************************************************************
 * Name: s32k1xx_clockconfig
 *
 * Description:
 *   Called to initialize the S32K1XX.  This does whatever setup is needed
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

int s32k1xx_clockconfig(const struct clock_configuration_s *clkcfg);

/****************************************************************************
 * Name: s32k1xx_clock_pm_register
 *
 * Description:
 *   This function is called after OS and PM init in order to register to
 *   receive power management event callbacks.
 *
 * Input Parameters:
 *   None
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/
#ifdef CONFIG_PM
void s32k1xx_clock_pm_register(void);
#endif

/****************************************************************************
 * Name: s32k1xx_get_coreclk
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

uint32_t s32k1xx_get_coreclk(void);

/****************************************************************************
 * Name: s32k1xx_get_sysclk
 *
 * Description:
 *   Return the current value of an SCG system clock frequency, these clocks
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

uint32_t s32k1xx_get_sysclk(enum scg_system_clock_type_e type);

/****************************************************************************
 * Name: s32k1xx_get_asnchfreq
 *
 * Description:
 *   Gets SCG asynchronous clock frequency from a clock source.
 *
 * Input Parameters:
 *   clksrc - The requested clock source.
 *   type   - The requested clock type.
 *
 * Returned Value:
 *   The frequency of the requested asynchronous clock source.
 *
 ****************************************************************************/

uint32_t s32k1xx_get_asnchfreq(enum clock_names_e clksrc,
                               enum scg_async_clock_type_e type);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_S32K1XX_S32K1XX_CLKCONFIG_H */
