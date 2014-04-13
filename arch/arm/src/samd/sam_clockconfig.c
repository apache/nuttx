/****************************************************************************
 * arch/arm/src/samd/sam_clockconfig.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   1. "Atmel SAM D20J / SAM D20G / SAM D20E ARM-Based Microcontroller
 *      Datasheet", 42129J–SAM–12/2013
 *   2. Atmel sample code.  This code has an ASF license with is compatible
 *      with the NuttX BSD license, but includes the provision that this
 *      code not be used in non-Atmel products.  That sample code was used
 *      only as a reference so I believe that only the NuttX BSD license
 *      applies.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#include "up_arch.h"

#include "chip/sam_pm.h"
#include "chip/sam_sysctrl.h"
#include "chip/sam_gclk.h"
#include "chip/sam_nvmctrl.h"
#include "chip/sam_fuses.h"

#include <arch/board/board.h>

#include "sam_clockconfig.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the configuration of on GCLK */

#ifdef BOARD_GCLK_ENABLE
struct sam_gclkconfig_s
{
  uint8_t  gclk;        /* Clock generator */
  bool     runstandby;  /* Run clock in standby */
  bool     output;      /* Output enable */
  uint8_t  clksrc;      /* Encoded clock source */
  uint16_t prescaler;   /* Prescaler value */
};
#endif

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
    .clksrc     = (uint8_t)(BOARD_GCLK0_CLOCK_SOURCE >> GCLK_GENCTRL_SRC_SHIFT),
  }

  /* GCLK generator 1 */

#ifdef BOARD_GCLK1_ENABLE
  ,
  {
    .gclk       = 1,
#ifdef BOARD_GCLK1_RUN_IN_STANDBY
    .runstandby = true;
#endif
#ifdef BOARD_GCLK1_OUTPUT_ENABLE
    .output     = true;
#endif
    .prescaler  = BOARD_GCLK1_PRESCALER,
    .clksrc     = (uint8_t)(BOARD_GCLK1_CLOCK_SOURCE >> GCLK_GENCTRL_SRC_SHIFT),
  }
#endif

  /* GCLK generator 2 (RTC) */

#ifdef BOARD_GCLK2_ENABLE
  ,
  {
    .gclk       = 2,
#ifdef BOARD_GCLK2_RUN_IN_STANDBY
    .runstandby = true;
#endif
#ifdef BOARD_GCLK2_OUTPUT_ENABLE
    .output     = true;
#endif
    .prescaler  = BOARD_GCLK2_PRESCALER,
    .clksrc     = (uint8_t)(BOARD_GCLK2_CLOCK_SOURCE >> GCLK_GENCTRL_SRC_SHIFT),
  }
#endif

  /* GCLK generator 3 */

#ifdef BOARD_GCLK3_ENABLE
  ,
  {
    .gclk       = 3,
#ifdef BOARD_GCLK3_RUN_IN_STANDBY
    .runstandby = true;
#endif
#ifdef BOARD_GCLK3_OUTPUT_ENABLE
    .output     = true;
#endif
    .prescaler  = BOARD_GCLK3_PRESCALER,
    .clksrc     = (uint8_t)(BOARD_GCLK3_CLOCK_SOURCE >> GCLK_GENCTRL_SRC_SHIFT),
  }
#endif

  /* GCLK generator 4 */

#ifdef BOARD_GCLK4_ENABLE
  ,
  {
    .gclk       = 4,
#ifdef BOARD_GCLK4_RUN_IN_STANDBY
    .runstandby = true;
#endif
#ifdef BOARD_GCLK4_OUTPUT_ENABLE
    .output     = true;
#endif
    .prescaler  = BOARD_GCLK4_PRESCALER,
    .clksrc     = (uint8_t)(BOARD_GCLK4_CLOCK_SOURCE >> GCLK_GENCTRL_SRC_SHIFT),
  }
#endif

  /* GCLK generator 5 */

#ifdef BOARD_GCLK5_ENABLE
  ,
  {
    .gclk       = 5,
#ifdef BOARD_GCLK5_RUN_IN_STANDBY
    .runstandby = true;
#endif
#ifdef BOARD_GCLK5_OUTPUT_ENABLE
    .output     = true;
#endif
    .prescaler  = BOARD_GCLK5_PRESCALER,
    .clksrc     = (uint8_t)(BOARD_GCLK5_CLOCK_SOURCE >> GCLK_GENCTRL_SRC_SHIFT),
  }
#endif

  /* GCLK generator 6 */

#ifdef BOARD_GCLK6_ENABLE
  ,
  {
    .gclk       = 6,
#ifdef BOARD_GCLK6_RUN_IN_STANDBY
    .runstandby = true;
#endif
#ifdef BOARD_GCLK6_OUTPUT_ENABLE
    .output     = true;
#endif
    .prescaler  = BOARD_GCLK6_PRESCALER,
    .clksrc     = (uint8_t)(BOARD_GCLK6_CLOCK_SOURCE >> GCLK_GENCTRL_SRC_SHIFT),
  }
#endif

  /* GCLK generator 7 */

#ifdef BOARD_GCLK7_ENABLE
  ,
  {
    .gclk       = 7,
#ifdef BOARD_GCLK7_RUN_IN_STANDBY
    .runstandby = true;
#endif
#ifdef BOARD_GCLK7_OUTPUT_ENABLE
    .output     = true;
#endif
    .prescaler  = BOARD_GCLK7_PRESCALER,
    .clksrc     = (uint8_t)(BOARD_GCLK7_CLOCK_SOURCE >> GCLK_GENCTRL_SRC_SHIFT),
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

  regval = BOARD_XOSC_STARTUPTIME

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

  regval = BOARD_XOSC32K_STARTUPTIME

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

  regval  = getreg32(SYSCTRL_FUSES_OSC32KCAL_ADDR)
  calib   = (regval & SYSCTRL_FUSES_OSC32KCAL_MASK) >> SYSCTRL_FUSES_OSC32KCAL_SHIFT;
  regval  = calib << SYSCTRL_OSC32K_CALIB_SHIFT;

  /* Configure OSC32K */

  regval |= BOARD_OSC32K_STARTUPTIME;

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
 *   BOARD_DFLL_COARSEVALUE         - Value
 *   BOARD_DFLL_FINEVALUE           - Value
 *
 * Open Loop mode only:
 *   BOARD_DFLL_COARSEVALUE         - Value
 *   BOARD_DFLL_FINEVALUE           - Value
 *
 * Closed loop mode only:
 *   BOARD_DFLL_SRCGCLKGEN          - See GCLK_CLKCTRL_GEN* definitions
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
  uint16_t control;
  uint32_t regval;

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

  regval = SYSCTRL_DFLLVAL_COARSE(BOARD_DFLL_COARSEVALUE) |
           SYSCTRL_DFLLVAL_FINE(BOARD_DFLL_FINEVALUE);
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
 *   BOARD_DFLL_SRCGCLKGEN - See GCLK_CLKCTRL_GEN* definitions
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

  regval = (BOARD_DFLL_SRCGCLKGEN | GCLK_CLKCTRL_ID_DFLL48M);
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
 * Name: sam_gclck_waitsyncbusy
 *
 * Description:
 *   What until the SYNCBUSY bit is cleared.  This bit is cleared when the
 *   synchronization of registers between the clock domains is complete.
 *   This bit is set when the synchronization of registers between clock
 *   domains is started.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam_gclck_waitsyncbusy(void)
{
  while ((getreg8(SAM_GCLK_STATUS) & GCLK_STATUS_SYNCBUSY) != 0);
}

/****************************************************************************
 * Name: sam_config_gclks
 *
 * Description:
 *   Configure a single GCLK(s) based on settings in the board.h header file.
 *   Depends on:
 *
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
static inline void sam_gclk_config(FAR const struct sam_gclkconfig_s *config)
{
  uint32_t genctrl;
  uint32_t gendiv;

  /* Select the requested source clock for the generator */

  genctrl = ((uint32_t)config->gclk << GCLK_GENCTRL_ID_SHIFT) |
            ((uint32_t)config->clksrc << GCLK_GENCTRL_SRC_SHIFT);
  gendiv  = ((uint32_t)config->gclk << GCLK_GENDIV_ID_SHIFT);

#if 0 /* Not yet supported */
  /* Configure the clock to be either high or low when disabled */

  if (config->level)
    {
      genctrl |= GCLK_GENCTRL_OOV;
    }
#endif

  /* Configure if the clock output to I/O pin should be enabled */

  if (config->output)
    {
      genctrl |= GCLK_GENCTRL_OE;
    }

  /* Set the prescaler division factor */

  if (config->prescaler > 1)
    {
      /* Check if division is a power of two */

      if (((config->prescaler & (config->prescaler - 1)) == 0))
        {
          /* Determine the index of the highest bit set to get the
           * division factor that must be loaded into the division
           * register.
           */

          uint32_t count = 0;
          uint32_t mask;

          for (mask = 2; mask < (uint32_t)config->prescaler; mask <<= 1)
            {
              count++;
            }

          /* Set binary divider power of 2 division factor */

          gendiv  |= count << GCLK_GENDIV_DIV_SHIFT;
          genctrl |= GCLK_GENCTRL_DIVSEL;
        }
      else
        {
          /* Set integer division factor */

          gendiv  |= GCLK_GENDIV_DIV((uint32_t)config->prescaler);

          /* Enable non-binary division with increased duty cycle accuracy */

          genctrl |= GCLK_GENCTRL_IDC;
        }
    }

  /* Enable or disable the clock in standby mode */

  if (config->runstandby)
    {
      genctrl |= GCLK_GENCTRL_RUNSTDBY;
    }

  /* Wait for synchronization */

  sam_gclck_waitsyncbusy();

  /* Select the generator */

  putreg32(((uint32_t)config->gclk << GCLK_GENDIV_ID_SHIFT),
           SAM_GCLK_GENDIV);

  /* Wait for synchronization */

  sam_gclck_waitsyncbusy();

  /* Write the new generator configuration */

  putreg32(gendiv, SAM_GCLK_GENDIV);

  /* Wait for synchronization */

  sam_gclck_waitsyncbusy();

  /* Enable the clock generator */

  genctrl |= GCLK_GENCTRL_GENEN;
  putreg32(genctrl, SAM_GCLK_GENCTRL);

  /* Wait for synchronization */

  sam_gclck_waitsyncbusy();
}
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
  uint32_t regval;
  int i;

  /* Turn on the GCLK interface clock */

  regval  = getreg32(SAM_PM_APBAMASK);
  regval |= PM_APBAMASK_GCLK;
  putreg32(regval, SAM_PM_APBAMASK);

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
  uint8_t regval;

  /* Set CPU divider and, optionally, enable failure detection */

  putreg8(BOARD_CPU_DIVIDER, SAM_PM_CPUSEL);

  regval  = getreg8(SAM_PM_CTRL);
#ifdef BOARD_CPU_FAILDECT
  regval |= PM_CTRL_CFDEN;
#else
  regval &= ~PM_CTRL_CFDEN;
#endif
  putreg8(regval, SAM_PM_CTRL);

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

  sam_xosc_config();

  /* Configure XOSC32K */

  sam_xosc32k_config();

  /* Configure OSCK32K */

  sam_osc32k_config();

  /* Configure DFLL */

  sam_dfll_config();

  /* Configure OSC8M */

  sam_osc8m_config();

  /* Configure GCLK(s) */

  sam_config_gclks();

  /* Set CPU and BUS clock dividers */

  sam_dividers();
}
