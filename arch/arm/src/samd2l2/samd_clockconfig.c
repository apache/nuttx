/****************************************************************************
 * arch/arm/src/samd2l2/samd_clockconfig.c
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

/* References:
 *   1. "Atmel SAM D20J / SAM D20G / SAM D20E ARM-Based Microcontroller
 *      Datasheet", 42129J-SAM-12/2013
 *   2. "Atmel SAM D21E / SAM D21G / SAM D21J SMART ARM-Based Microcontroller
 *      Datasheet", Atmel-42181E-SAM-D21_Datasheet-02/2015
 *   3. Atmel sample code for the SAMD20.  This code has an ASF license
 *      with is compatible with the NuttX BSD license, but includes the
 *      provision that this code not be used in non-Atmel products.  That
 *      sample code was used only as a reference so I believe that only the
 *      NuttX BSD license applies.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#include "arm_internal.h"
#include "hardware/samd_pm.h"
#include "hardware/samd_sysctrl.h"
#include "hardware/samd_gclk.h"
#include "hardware/samd_nvmctrl.h"
#include "sam_fuses.h"
#include "sam_gclk.h"

#include <arch/board/board.h>

#include "samd_periphclks.h"
#include "sam_clockconfig.h"

#if defined(CONFIG_ARCH_FAMILY_SAMD20) || defined(CONFIG_ARCH_FAMILY_SAMD21)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This structure describes the configuration of every enabled GCLK */

#ifdef BOARD_GCLK_ENABLE
static const struct sam_gclkconfig_s g_gclkconfig[] =
{
  /* GCLK generator 0 (Main Clock) */

  {
    .gclk       = 0,
#ifdef BOARD_GCLK0_RUN_IN_STANDBY
    .runstandby = true,
#endif
#ifdef BOARD_GCLK0_OUTPUT_ENABLE
    .output     = true,
#endif
    .prescaler  = BOARD_GCLK0_PRESCALER,
    .clksrc     = (uint8_t)(BOARD_GCLK0_CLOCK_SOURCE >>
                            GCLK_GENCTRL_SRC_SHIFT),
  }

  /* GCLK generator 1 */

#ifdef BOARD_GCLK1_ENABLE
  ,
  {
    .gclk       = 1,
#ifdef BOARD_GCLK1_RUN_IN_STANDBY
    .runstandby = true,
#endif
#ifdef BOARD_GCLK1_OUTPUT_ENABLE
    .output     = true,
#endif
    .prescaler  = BOARD_GCLK1_PRESCALER,
    .clksrc     = (uint8_t)(BOARD_GCLK1_CLOCK_SOURCE >>
                            GCLK_GENCTRL_SRC_SHIFT),
  }
#endif

  /* GCLK generator 2 (RTC) */

#ifdef BOARD_GCLK2_ENABLE
  ,
  {
    .gclk       = 2,
#ifdef BOARD_GCLK2_RUN_IN_STANDBY
    .runstandby = true,
#endif
#ifdef BOARD_GCLK2_OUTPUT_ENABLE
    .output     = true,
#endif
    .prescaler  = BOARD_GCLK2_PRESCALER,
    .clksrc     = (uint8_t)(BOARD_GCLK2_CLOCK_SOURCE >>
                            GCLK_GENCTRL_SRC_SHIFT),
  }
#endif

  /* GCLK generator 3 */

#ifdef BOARD_GCLK3_ENABLE
  ,
  {
    .gclk       = 3,
#ifdef BOARD_GCLK3_RUN_IN_STANDBY
    .runstandby = true,
#endif
#ifdef BOARD_GCLK3_OUTPUT_ENABLE
    .output     = true,
#endif
    .prescaler  = BOARD_GCLK3_PRESCALER,
    .clksrc     = (uint8_t)(BOARD_GCLK3_CLOCK_SOURCE >>
                            GCLK_GENCTRL_SRC_SHIFT),
  }
#endif

  /* GCLK generator 4 */

#ifdef BOARD_GCLK4_ENABLE
  ,
  {
    .gclk       = 4,
#ifdef BOARD_GCLK4_RUN_IN_STANDBY
    .runstandby = true,
#endif
#ifdef BOARD_GCLK4_OUTPUT_ENABLE
    .output     = true,
#endif
    .prescaler  = BOARD_GCLK4_PRESCALER,
    .clksrc     = (uint8_t)(BOARD_GCLK4_CLOCK_SOURCE >>
                            GCLK_GENCTRL_SRC_SHIFT),
  }
#endif

  /* GCLK generator 5 */

#ifdef BOARD_GCLK5_ENABLE
  ,
  {
    .gclk       = 5,
#ifdef BOARD_GCLK5_RUN_IN_STANDBY
    .runstandby = true,
#endif
#ifdef BOARD_GCLK5_OUTPUT_ENABLE
    .output     = true,
#endif
    .prescaler  = BOARD_GCLK5_PRESCALER,
    .clksrc     = (uint8_t)(BOARD_GCLK5_CLOCK_SOURCE >>
                            GCLK_GENCTRL_SRC_SHIFT),
  }
#endif

  /* GCLK generator 6 */

#ifdef BOARD_GCLK6_ENABLE
  ,
  {
    .gclk       = 6,
#ifdef BOARD_GCLK6_RUN_IN_STANDBY
    .runstandby = true,
#endif
#ifdef BOARD_GCLK6_OUTPUT_ENABLE
    .output     = true,
#endif
    .prescaler  = BOARD_GCLK6_PRESCALER,
    .clksrc     = (uint8_t)(BOARD_GCLK6_CLOCK_SOURCE >>
                            GCLK_GENCTRL_SRC_SHIFT),
  }
#endif

  /* GCLK generator 7 */

#ifdef BOARD_GCLK7_ENABLE
  ,
  {
    .gclk       = 7,
#ifdef BOARD_GCLK7_RUN_IN_STANDBY
    .runstandby = true,
#endif
#ifdef BOARD_GCLK7_OUTPUT_ENABLE
    .output     = true,
#endif
    .prescaler  = BOARD_GCLK7_PRESCALER,
    .clksrc     = (uint8_t)(BOARD_GCLK7_CLOCK_SOURCE >>
                            GCLK_GENCTRL_SRC_SHIFT),
  }
#endif

  /* GCLK generator 8 */

#ifdef BOARD_GCLK8_ENABLE
  ,
  {
    .gclk       = 8,
#ifdef BOARD_GCLK8_RUN_IN_STANDBY
    .runstandby = true,
#endif
#ifdef BOARD_GCLK8_OUTPUT_ENABLE
    .output     = true,
#endif
    .prescaler  = BOARD_GCLK8_PRESCALER,
    .clksrc     = (uint8_t)(BOARD_GCLK8_CLOCK_SOURCE >>
                            GCLK_GENCTRL_SRC_SHIFT),
  }
#endif
};

#define NGCLKS_ENABLED (sizeof(g_gclkconfig) / sizeof(struct sam_gclkconfig_s))
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_flash_waitstates
 *
 * Description:
 *   Set the FLASH wait states based on settings in the board.h header file
 *   Depends on:
 *
 *     BOARD_FLASH_WAITSTATES - Number of wait states
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void sam_flash_waitstates(void)
{
  uint32_t regval;

  regval = getreg32(SAM_NVMCTRL_CTRLB);
  regval &= ~NVMCTRL_CTRLB_RWS_MASK;
  regval |= NVMCTRL_CTRLB_RWS(BOARD_FLASH_WAITSTATES);
  putreg32(regval, SAM_NVMCTRL_CTRLB);
}

/****************************************************************************
 * Name: sam_xosc_config
 *
 * Description:
 *   Configure XOSC based on settings in the board.h header file
 *   Depends on:
 *
 *     BOARD_XOSC_ENABLE       - Boolean (defined / not defined)
 *     BOARD_XOSC_FREQUENCY    - In Hz
 *     BOARD_XOSC_STARTUPTIME  - See SYSCTRL_XOSC_STARTUP_* definitions
 *     BOARD_XOSC_ISCRYSTAL    - Boolean (defined / not defined)
 *     BOARD_XOSC_AMPGC        - Boolean (defined / not defined)
 *     BOARD_XOSC_ONDEMAND     - Boolean (defined / not defined)
 *     BOARD_XOSC_RUNINSTANDBY - Boolean (defined / not defined)
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef BOARD_XOSC_ENABLE
static inline void sam_xosc_config(void)
{
  uint16_t regval;

  /* Configure the XOSC clock */

  regval = BOARD_XOSC_STARTUPTIME;

#ifdef BOARD_XOSC_ISCRYSTAL
  /* XOSC is a crystal */

  regval |= SYSCTRL_XOSC_XTALEN;
#endif

#ifdef BOARD_XOSC_AMPGC
  /* Enable automatic gain control */

  regval |= SYSCTRL_XOSC_AMPGC;

#else
  /* Set gain if automatic gain control is not selected */

#if BOARD_XOSC_FREQUENCY <= 2000000
  regval |= SYSCTRL_XOSC_GAIN_2MHZ;
#elif BOARD_XOSC_FREQUENCY <= 4000000
  regval |= SYSCTRL_XOSC_GAIN_4MHZ;
#elif BOARD_XOSC_FREQUENCY <= 8000000
  regval |= SYSCTRL_XOSC_GAIN_8MHZ;
#elif BOARD_XOSC_FREQUENCY <= 16000000
  regval |= SYSCTRL_XOSC_GAIN_16MHZ;
#elif BOARD_XOSC_FREQUENCY <= 30000000
  regval |= SYSCTRL_XOSC_GAIN_30MHZ;
#else
#  error BOARD_XOSC_FREQUENCY out of range
#endif
#endif /* BOARD_XOSC_AMPGC */

#ifdef BOARD_XOSC_ONDEMAND
  regval |= SYSCTRL_XOSC_ONDEMAND;
#endif

#ifdef BOARD_XOSC_RUNINSTANDBY
  regval |= SYSCTRL_XOSC_RUNSTDBY;
#endif

  putreg16(regval, SAM_SYSCTRL_XOSC);

  /* Then enable the XOSC clock */

  regval |= SYSCTRL_XOSC_ENABLE;
  putreg16(regval, SAM_SYSCTRL_XOSC);
}
#else
#  define sam_xosc_config()
#endif

/****************************************************************************
 * Name: sam_xosc32k_config
 *
 * Description:
 *   Configure XOSC32K based on settings in the board.h header file.
 *   Depends on:
 *
 *     BOARD_XOSC32K_ENABLE       - Boolean (defined / not defined)
 *     BOARD_XOSC32K_FREQUENCY    - In Hz
 *     BOARD_XOSC32K_STARTUPTIME  - See SYSCTRL_XOSC32K_STARTUP_* definitions
 *     BOARD_XOSC32K_ISCRYSTAL    - Boolean (defined / not defined)
 *     BOARD_XOSC32K_AAMPEN       - Boolean (defined / not defined)
 *     BOARD_XOSC32K_EN1KHZ       - Boolean (defined / not defined)
 *     BOARD_XOSC32K_EN32KHZ      - Boolean (defined / not defined)
 *     BOARD_XOSC32K_ONDEMAND     - Boolean (defined / not defined)
 *     BOARD_XOSC32K_RUNINSTANDBY - Boolean (defined / not defined)
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef BOARD_XOSC32K_ENABLE
static inline void sam_xosc32k_config(void)
{
  uint16_t regval;

  /* Configure XOSC32K */

  regval = BOARD_XOSC32K_STARTUPTIME;

#ifdef BOARD_XOSC32K_ISCRYSTAL
  regval |= SYSCTRL_XOSC32K_XTALEN;
#endif

#ifdef BOARD_XOSC32K_AAMPEN
  regval |= SYSCTRL_XOSC32K_AAMPEN;
#endif

#ifdef BOARD_XOSC32K_EN1KHZ
  regval |= SYSCTRL_XOSC32K_EN1K;
#endif

#ifdef BOARD_XOSC32K_EN32KHZ
  regval |= SYSCTRL_XOSC32K_EN32K;
#endif

#ifdef BOARD_XOSC32K_ONDEMAND
  regval |= SYSCTRL_XOSC32K_ONDEMAND;
#endif

#ifdef BOARD_XOSC32K_RUNINSTANDBY
  regval |= SYSCTRL_XOSC32K_RUNSTDBY;
#endif

  putreg16(regval, SAM_SYSCTRL_XOSC32K);

  /* Then enable the XOSC clock */

  regval |= SYSCTRL_XOSC32K_ENABLE;
  putreg16(regval, SAM_SYSCTRL_XOSC32K);
}
#else
#  define sam_xosc32k_config()
#endif

/****************************************************************************
 * Name: sam_osc32k_config
 *
 * Description:
 *   Configure OSC32K based on settings in the board.h header file.
 *   Depends on:
 *
 *     BOARD_OSC32K_ENABLE       - Boolean (defined / not defined)
 *     BOARD_OSC32K_FREQUENCY    - In Hz
 *     BOARD_OSC32K_STARTUPTIME  - See SYSCTRL_OSC32K_STARTUP_* definitions
 *     BOARD_OSC32K_EN1KHZ       - Boolean (defined / not defined)
 *     BOARD_OSC32K_EN32KHZ      - Boolean (defined / not defined)
 *     BOARD_OSC32K_ONDEMAND     - Boolean (defined / not defined)
 *     BOARD_OSC32K_RUNINSTANDBY - Boolean (defined / not defined)
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef BOARD_OSC32K_ENABLE
static inline void sam_osc32k_config(void)
{
  uint32_t regval;
  uint32_t calib;

  /* Recover OSC32K calibration data from OTP "fuse" memory */

  regval  = getreg32(SYSCTRL_FUSES_OSC32KCAL_ADDR);
  calib   = (regval & SYSCTRL_FUSES_OSC32KCAL_MASK) >>
             SYSCTRL_FUSES_OSC32KCAL_SHIFT;

  /* Configure OSC32K */

  regval = BOARD_OSC32K_STARTUPTIME;

#ifdef BOARD_OSC32K_EN1KHZ
  regval |= SYSCTRL_OSC32K_EN1K;
#endif

#ifdef BOARD_OSC32K_EN32KHZ
  regval |= SYSCTRL_OSC32K_EN32K;
#endif

#ifdef BOARD_OSC32K_ONDEMAND
  regval |= SYSCTRL_OSC32K_ONDEMAND;
#endif

#ifdef BOARD_OSC32K_RUNINSTANDBY
  regval |= SYSCTRL_OSC32K_RUNSTDBY;
#endif

  putreg32(regval, SAM_SYSCTRL_OSC32K);

  /* Then enable OSC32K */

  regval |= SYSCTRL_OSC32K_ENABLE;
  putreg32(regval, SAM_SYSCTRL_OSC32K);

  /* From the datasheet on page 157:
   * "When writing to the Calibration bits, the user must wait for the
   * PCLKSR.OSC32KRDY bit to go high before the value is committed
   * to the oscillator."
   */

  while ((getreg32(SAM_SYSCTRL_PCLKSR) & SYSCTRL_INT_OSC32KRDY) == 0);

  regval = getreg32(SAM_SYSCTRL_OSC32K);
  regval |= calib << SYSCTRL_OSC32K_CALIB_SHIFT;
  putreg32(regval, SAM_SYSCTRL_OSC32K);
}
#else
#  define sam_osc32k_config()
#endif

/****************************************************************************
 * Name: sam_osc8m_config
 *
 * Description:
 *   Configure OSC8M based on settings in the board.h header file.
 *   Depends on:
 *
 *     BOARD_OSC8M_PRESCALER     - See SYSCTRL_OSC8M_PRESC_DIV* definitions
 *     BOARD_OSC8M_ONDEMAND      - Boolean (defined / not defined)
 *     BOARD_OSC8M_RUNINSTANDBY  - Boolean (defined / not defined)
 *
 *   On any reset the synchronous clocks start to their initial state:
 *
 *     OSC8M is enabled and divided by 8
 *     GCLK_MAIN uses OSC8M as source
 *     CPU and BUS clocks are undivided
 *
 *   The reset state of the OSC8M register is:
 *
 *     FFxx CCCC CCCC CCCC xxxx xxPP ORxx xxEx
 *     xx00 xxxx xxxx xxxx 0000 0011 1000 0010
 *
 *     FRANGE   FF      Loaded from FLASH calibration at startup
 *     CALIB    CCC...C Loaded from FLASH calibration at startup
 *     PRESC    PP      3 = Divide by 8
 *     ONDEMAND O       1
 *     RUNSTBY  R       0
 *     ENABLE   1       1
 *
 *   NOTE that since we are running from OSC8M, it cannot be disable!
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void sam_osc8m_config(void)
{
  uint32_t regval;

  /* Configure OSC8M */

  regval  = getreg32(SAM_SYSCTRL_OSC8M);
  regval &= ~(SYSCTRL_OSC8M_PRESC_MASK | SYSCTRL_OSC8M_ONDEMAND |
              SYSCTRL_OSC8M_RUNSTDBY);

  /* Select the prescaler */

  regval |= (BOARD_OSC8M_PRESCALER | SYSCTRL_OSC8M_ENABLE);

#ifdef BOARD_OSC8M_ONDEMAND
  /* Select on-demand oscillator controls */

  regval |= SYSCTRL_OSC8M_ONDEMAND;
#endif

#ifdef BOARD_OSC8M_RUNINSTANDBY
  /* The oscillator continues to run in standby sleep mode  */

  regval |= SYSCTRL_OSC8M_RUNSTDBY;
#endif

  /* Set the OSC8M configuration */

  putreg32(regval, SAM_SYSCTRL_OSC8M);
}

#ifdef BOARD_DPLL_ENABLE
static inline void sam_dpll_config(void)
{
  uint8_t ctrla;
  uint32_t ctrlb;
  uint32_t ratio;

  ctrla = SYSCTRL_DPLLCTRLA_ENABLE;    /* Enable the FDPLL */
  ctrlb = 0;
  ratio = 0;

#ifdef BOARD_DPLL_RUNINSTANDBY
  ctrla |= SYSCTRL_DPLLCTRLA_RUNSTDBY; /* Run in standby */
#endif

#ifdef BOARD_DPLL_ONDEMAND
  ctrla |= SYSCTRL_DPLLCTRLA_ONDEMAND; /* On demand mode */
#endif

#ifdef BOARD_DPLL_DIV
  ctrlb |= SYSCTRL_DPLLCTRLB_DIV(BOARD_DPLL_DIV);
#endif

#ifdef BOARD_DPLL_LBYPASS
  ctrlb |= SYSCTRL_DPLLCTRLB_LBYPASS;
#endif

#ifdef BOARD_DPLL_LTIME
  ctrlb |= BOARD_DPLL_LTIME;
#endif

#ifdef BOARD_DPLL_REFCLK
  ctrlb |= BOARD_DPLL_REFCLK;
  ratio = SYSCTRL_DPLLRATIO_LDR(BOARD_DPLL_LDR);
#ifdef BOARD_DPLL_LDRFRAC
  ratio |= SYSCTRL_DPLLRATIO_LDRFRAC(BOARD_DPLL_LDRFRAC);
#endif

  /* If a GCLK reference was requested, we must initialize the GCLK first */

  if (BOARD_DPLL_REFCLK == SYSCTRL_DPLLCTRLB_REFCLK_GCLKDPLL)
    {
      putreg16(GCLK_CLKCTRL_ID_DPLL | GCLK_CLKCTRL_GEN(2) |
               GCLK_CLKCTRL_CLKEN, SAM_GCLK_CLKCTRL);
    }

  putreg32(ratio, SAM_SYSCTRL_DPLLRATIO);
#else

  /* If no reference clock was specified, default to using
   * the external 32KHz crystal and output of 96MHz
   */

  ctrlb |= SYSCTRL_DPLLCTRLB_REFCLK_XOSC32;
  ratio = SYSCTRL_DPLLRATIO_LDR(3000);
  putreg32(ratio, SAM_SYSCTRL_DPLLRATIO);
#endif

#ifdef BOARD_DPLL_WUF
  ctrlb |= BOARD_DPLL_WUF;
#endif

#ifdef BOARD_DPLL_LPEN
  ctrlb |= BOARD_DPLL_LPEN;
#endif

#ifdef BOARD_DPLL_FILTER
  ctrlb |= BOARD_DPLL_FILTER;
#endif

  /* Write Control B register */

  putreg32(ctrlb, SAM_SYSCTRL_DPLLCTRLB);

  /* Write Control A register */

  putreg8(ctrla, SAM_SYSCTRL_DPLLCTRLA);

  /* Wait for the DPLL to synchronize */

  while ((getreg8(SAM_SYSCTRL_DPLLSTATUS) & SYSCTRL_DPLLSTATUS_CLKRDY) == 0);
}
#else
#  define sam_dpll_config()
#endif

/****************************************************************************
 * Name: sam_dfll_config
 *
 * Description:
 *   Configure the DFLL based on settings in the board.h header file.
 *   Depends on:
 *
 *   BOARD_DFLL_OPENLOOP            - Boolean (defined / not defined)
 *   BOARD_DFLL_TRACKAFTERFINELOCK  - Boolean (defined / not defined)
 *   BOARD_DFLL_KEEPLOCKONWAKEUP    - Boolean (defined / not defined)
 *   BOARD_DFLL_ENABLECHILLCYCLE    - Boolean (defined / not defined)
 *   BOARD_DFLL_QUICKLOCK           - Boolean (defined / not defined)
 *   BOARD_DFLL_ONDEMAND            - Boolean (defined / not defined)
 *
 * Open Loop mode only:
 *   BOARD_DFLL_COARSEVALUE         - Value
 *   BOARD_DFLL_FINEVALUE           - Value
 *
 * Closed loop mode only:
 *   BOARD_DFLL_SRCGCLKGEN          - GCLK index
 *   BOARD_DFLL_MULTIPLIER          - Value
 *   BOARD_DFLL_MAXCOARSESTEP       - Value
 *   BOARD_DFLL_MAXFINESTEP         - Value
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef BOARD_DFLL_ENABLE
static inline void sam_dfll_config(void)
{
  uint16_t  control;
  uint32_t  regval;
  uint32_t *nvm_cal = (uint32_t *)SAM_NVMCALIB_AREA;
  uint32_t  coarse  = ((nvm_cal[1]) >> 26) & 0x3f;
  uint32_t  fine    = ((nvm_cal[2]) >> 0) & 0x7ff;

  /* Set up the DFLL control register */

  control  = SYSCTRL_DFLLCTRL_ENABLE;   /* Enable the DFLL */

#ifndef BOARD_DFLL_OPENLOOP
  control |= SYSCTRL_DFLLCTRL_MODE;     /* Closed loop mode */
#endif

#ifndef BOARD_DFLL_TRACKAFTERFINELOCK
  control |= SYSCTRL_DFLLCTRL_STABLE;   /* FINE calibration fixed after a fine lock */
#endif

#ifndef BOARD_DFLL_KEEPLOCKONWAKEUP
  control |= SYSCTRL_DFLLCTRL_LLAW;     /* Lose lock after wake */
#endif

#ifndef BOARD_DFLL_ENABLECHILLCYCLE
  control |= SYSCTRL_DFLLCTRL_CCDIS;    /* Chill cycle disable */
#endif

#ifndef BOARD_DFLL_QUICKLOCK
  control |= SYSCTRL_DFLLCTRL_QLDIS; /* Quick lock disable */
#endif

  /* Then enable the DFLL (with ONDEMAND set to zero). */

  putreg16(control, SAM_SYSCTRL_DFLLCTRL);

  /* Wait for the DFLL to synchronize */

  while ((getreg32(SAM_SYSCTRL_PCLKSR) & SYSCTRL_INT_DFLLRDY) == 0);

  /* Set up the open loop mode multiplier register */

#ifndef BOARD_DFLL_OPENLOOP
  regval = SYSCTRL_DFLLMUL_CSTEP(BOARD_DFLL_MAXCOARSESTEP) |
           SYSCTRL_DFLLMUL_FSTEP(BOARD_DFLL_MAXFINESTEP) |
           SYSCTRL_DFLLMUL_MUL(BOARD_DFLL_MULTIPLIER);
  putreg32(regval, SAM_SYSCTRL_DFLLMUL);
#else
  putreg32(0, SAM_SYSCTRL_DFLLMUL);
#endif

  /* Set up the DFLL value register */

  regval = SYSCTRL_DFLLVAL_COARSE(coarse) |
           SYSCTRL_DFLLVAL_FINE(fine);
  putreg32(regval, SAM_SYSCTRL_DFLLVAL);

  /* Finally, set the state of the ONDEMAND bit if necessary */

#ifdef BOARD_DFLL_ONDEMAND
  control |= SYSCTRL_DFLLCTRL_ONDEMAND; /* On demand control */
  putreg16(control, SAM_SYSCTRL_DFLLCTRL);
#endif
}
#else
#  define sam_dfll_config()
#endif

/****************************************************************************
 * Name: sam_dfll_reference
 *
 * Description:
 *   Enable DFLL reference clock if in closed loop mode.
 *   Depends on:
 *
 *   BOARD_DFLL_SRCGCLKGEN - GCLK index
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(BOARD_GCLK_ENABLE) && defined(BOARD_DFLL_ENABLE) && \
   !defined(BOARD_DFLL_OPENLOOP)
static inline void sam_dfll_reference(void)
{
  uint16_t regval;

  /* Disabled the DFLL reference clock */

  regval = GCLK_CLKCTRL_ID_DFLL48M;
  putreg16(regval, SAM_GCLK_CLKCTRL);

  /* Wait for the clock to become disabled */

  while ((getreg16(SAM_GCLK_CLKCTRL) & GCLK_CLKCTRL_CLKEN) != 0);

  /* Select the configured clock generator as the source for the DFLL
   * reference clock.
   *
   * NOTE: We could enable write lock here to prevent further modification
   */

  regval = ((BOARD_DFLL_SRCGCLKGEN << GCLK_CLKCTRL_GEN_SHIFT) |
            GCLK_CLKCTRL_ID_DFLL48M);
  putreg16(regval, SAM_GCLK_CLKCTRL);

  /* Enable the DFLL reference clock */

  regval |= GCLK_CLKCTRL_CLKEN;
  putreg16(regval, SAM_GCLK_CLKCTRL);

  /* The CLKCTRL.CLKEN bit must be synchronized to the generic clock domain.
   * CLKCTRL.CLKEN will continue to read as its previous state until the
   * synchronization is complete.
   */

  while ((getreg16(SAM_GCLK_CLKCTRL) & GCLK_CLKCTRL_CLKEN) == 0);
}
#else
#  define sam_dfll_reference()
#endif

/****************************************************************************
 * Name: sam_config_gclks
 *
 * Description:
 *   Configure GCLK(s) based on settings in the board.h header file.
 *   Depends on:
 *
 *   Global enable/disable.
 *
 *     BOARD_GCLK_ENABLE            - Boolean (defined / not defined)
 *
 *   For n=1-7:
 *     BOARD_GCLKn_ENABLE           - Boolean (defined / not defined)
 *
 *   For n=0-8:
 *     BOARD_GCLKn_RUN_IN_STANDBY   - Boolean (defined / not defined)
 *     BOARD_GCLKn_CLOCK_SOURCE     - See GCLK_GENCTRL_SRC_* definitions
 *     BOARD_GCLKn_PRESCALER        - Value
 *     BOARD_GCLKn_OUTPUT_ENABLE    - Boolean (defined / not defined)
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef BOARD_GCLK_ENABLE
static inline void sam_config_gclks(void)
{
  int i;

  /* Turn on the GCLK interface clock */

  sam_gclk_enableperiph();

  /* Reset the GCLK module */

  putreg8(GCLK_CTRL_SWRST, SAM_GCLK_CTRL);

  /* Wait for the reset to complete */

  while ((getreg8(SAM_GCLK_CTRL) & GCLK_CTRL_SWRST) != 0);

  /* Configure all GCLK generators, skipping GLCK_MAIN which is configured
   * below.
   */

  for (i = 1; i < NGCLKS_ENABLED; i++)
    {
      sam_gclk_config(&g_gclkconfig[i]);
    }

  /* Enable DFLL reference clock if the DFLL is enabled in closed loop mode */

  sam_dfll_reference();

  /* Configure the GCLK_MAIN last as it may depend on the DFLL or other
   * generators
   */

  sam_gclk_config(&g_gclkconfig[0]);
}
#else
#  define sam_config_gclks()
#endif

/****************************************************************************
 * Name: sam_dividers
 *
 * Description:
 *   Setup PM main clock dividers to generate CPU, AHB, and APB clocks.
 *   Depends on:
 *
 *  BOARD_CPU_DIVIDER   - See PM_CPUSEL_CPUDIV_* definitions
 *  BOARD_CPU_FRQUENCY  - In Hz
 *  BOARD_CPU_FAILDECT  - Boolean (defined / not defined)
 *  BOARD_APBA_DIVIDER  - See M_APBASEL_APBADIV_* definitions
 *  BOARD_APBA_FRQUENCY - In Hz
 *  BOARD_APBB_DIVIDER  - See M_APBBSEL_APBBDIV_* definitions
 *  BOARD_APBB_FRQUENCY - In Hz
 *  BOARD_APBC_DIVIDER  - See M_APBCSEL_APBCDIV_* definitions
 *  BOARD_APBC_FRQUENCY - In Hz
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void sam_dividers(void)
{
#ifdef PM_CTRL_CFDEN
  uint8_t regval;
#endif

  /* Set the CPU divider using the divider value from the board.h header
   * file
   */

  putreg8(BOARD_CPU_DIVIDER, SAM_PM_CPUSEL);

#ifdef PM_CTRL_CFDEN
  /* Optionally, enable failure detection */

  regval  = getreg8(SAM_PM_CTRL);
#ifdef BOARD_CPU_FAILDECT
  regval |= PM_CTRL_CFDEN;
#else
  regval &= ~PM_CTRL_CFDEN;
#endif
  putreg8(regval, SAM_PM_CTRL);
#endif

  /* Set the APBA, B, and C dividers */

  putreg8(BOARD_APBA_DIVIDER, SAM_PM_APBASEL);
  putreg8(BOARD_APBB_DIVIDER, SAM_PM_APBBSEL);
  putreg8(BOARD_APBC_DIVIDER, SAM_PM_APBCSEL);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_clockconfig
 *
 * Description:
 *   Called to establish the clock settings based on the values in board.h.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sam_clockconfig(void)
{
  /* Clear pending interrupt status. */

  putreg32(SYSCTRL_INT_ALL, SAM_SYSCTRL_INTFLAG);

  /* Set FLASH wait states */

  sam_flash_waitstates();

  /* Configure XOSC */

  putreg16(0, SAM_SYSCTRL_XOSC);
  sam_xosc_config();

  /* Configure XOSC32K */

  putreg16(0, SAM_SYSCTRL_XOSC32K);
  sam_xosc32k_config();

  /* Configure OSCK32K */

  sam_osc32k_config();

  /* Configure DFLL */

  sam_dfll_config();

  /* Configure OSC8M */

  sam_osc8m_config();

  /* Configure DPLL */

  sam_dpll_config();

  /* Configure GCLK(s) */

  sam_config_gclks();

  /* Set CPU and BUS clock dividers */

  sam_dividers();
}

#endif /* CONFIG_ARCH_FAMILY_SAMD20 || CONFIG_ARCH_FAMILY_SAMD21*/
