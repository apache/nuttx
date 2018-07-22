/****************************************************************************
 * arch/arm/src/samd2l2/saml_clockconfig.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   1. "Atmel SAM L21E / SAM L21G / SAM L21J Smart ARM-Based Microcontroller
 *       Datasheet", Atmel-42385C-SAML21_Datasheet_Preliminary-03/20/15
 *   2. The SAMD20 samd_clockconfig.c file.  See that file for additional
 *      references.
 *   3. Atmel sample code for the SAML21.  This code has an ASF license
 *      with is compatible with the NuttX BSD license, but includes the
 *      provision that this code not be used in non-Atmel products.  That
 *      sample code was used only as a reference so I believe that only the
 *      NuttX BSD license applies.
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

#include "chip/saml_pm.h"
#include "chip/saml_supc.h"
#include "chip/saml_oscctrl.h"
#include "chip/saml_osc32kctrl.h"
#include "chip/saml_gclk.h"
#include "chip/saml_nvmctrl.h"
#include "sam_gclk.h"

#include <arch/board/board.h>

#include "saml_periphclks.h"
#include "sam_clockconfig.h"

#ifdef CONFIG_ARCH_FAMILY_SAML21

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* BOARD_GCLK_ENABLE looks optional, but it is not */

#ifndef BOARD_GCLK_ENABLE
#  warning BOARD_GCLK_ENABLE must be defined
#  define BOARD_GCLK_ENABLE 1
#endif

/* Force enabling of the FDPLL reference clock */

#ifdef BOARD_FDPLL96M_ENABLE
#  if BOARD_FDPLL96M_REFCLK == OSCCTRL_DPLLCTRLB_REFLCK_XOSC && \
      !defined(BOARD_XOSC_ENABLE)
#    warning Forcing BOARD_XOSC_ENABLE for FDPLL96M
#    define BOARD_XOSC_ENABLE 1
#  elif BOARD_FDPLL96M_REFCLK == OSCCTRL_DPLLCTRLB_REFLCK_XOSCK32K && \
        !defined(BOARD_XOSC32K_ENABLE)
#    warning Forcing BOARD_XOSC32K_ENABLE for FDPLL96M
#    define BOARD_XOSC32K_ENABLE 1
#  elif BOARD_FDPLL96M_REFCLK == OSCCTRL_DPLLCTRLB_REFLCK_GLCK && \
        !defined(BOARD_GCLK_ENABLE)
#    warning Forcing BOARD_GCLK_ENABLE for FDPLL96M
#    define BOARD_GCLK_ENABLE 1
#  endif
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline void sam_flash_waitstates(void);
static void sam_performance_level(uint8_t level);
#ifdef BOARD_XOSC_ENABLE
static inline void sam_xosc_config(void);
#endif
#ifdef BOARD_XOSC32K_ENABLE
static inline void sam_xosc32k_config(void);
#endif
#ifdef BOARD_OSC32K_ENABLE
static inline void sam_osc32k_config(void);
#endif
static inline void sam_osc16m_config(void);
#ifdef BOARD_DFLL48M_ENABLE
static inline void sam_dfll48m_config(void);
static inline void sam_dfll48m_enable(void);
#endif
#if defined(BOARD_GCLK_ENABLE) && defined(BOARD_DFLL48M_ENABLE) && \
   !defined(BOARD_DFLL48M_OPENLOOP)
static inline void sam_dfll48m_refclk(void);
#endif
#ifdef BOARD_FDPLL96M_ENABLE
static inline void sam_fdpll96m_config(void);
static inline void sam_fdpll96m_refclk(void);
#endif
#ifdef BOARD_GCLK_ENABLE
static inline void sam_config_gclks(void);
#endif
static inline void sam_cpu_dividers(void);
static inline void sam_periph_clocks(void);

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

  /* GCLK generator 8 */

#ifdef BOARD_GCLK8_ENABLE
  ,
  {
    .gclk       = 8,
#ifdef BOARD_GCLK8_RUN_IN_STANDBY
    .runstandby = true;
#endif
#ifdef BOARD_GCLK8_OUTPUT_ENABLE
    .output     = true;
#endif
    .prescaler  = BOARD_GCLK8_PRESCALER,
    .clksrc     = (uint8_t)(BOARD_GCLK8_CLOCK_SOURCE >> GCLK_GENCTRL_SRC_SHIFT),
  }
#endif
};

#define NGCLKS_ENABLED (sizeof(g_gclkconfig) / sizeof(struct sam_gclkconfig_s))
#endif

/* These are temporary GLCK0 configuration that may be needed at power up */

static const struct sam_gclkconfig_s g_gclk0_default =
{
  .gclk       = 0,
  .prescaler  = 1,
  .clksrc     = (uint8_t)(GCLK_GENCTRL_SRC_OSC16M >> GCLK_GENCTRL_SRC_SHIFT),
};

static const struct sam_gclkconfig_s g_gclk0_ulp32kconfig =
{
  .gclk       = 0,
  .prescaler  = 1,
  .clksrc     = (uint8_t)(GCLK_GENCTRL_SRC_OSCULP32K >> GCLK_GENCTRL_SRC_SHIFT),
};

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

  /* Errate 13134: Correct the default value of the NVMCTRL.CTRLB.MANW bit */

  regval  = getreg32(SAM_NVMCTRL_CTRLB);
  regval |= NVMCTRL_CTRLB_MANW;
  putreg32(regval, SAM_NVMCTRL_CTRLB);

  /* Set the configured number of flash wait states */

  regval &= ~NVMCTRL_CTRLB_RWS_MASK;
  regval |= NVMCTRL_CTRLB_RWS(BOARD_FLASH_WAITSTATES);
  putreg32(regval, SAM_NVMCTRL_CTRLB);
}

/****************************************************************************
 * Name: sam_performance_level
 *
 * Description:
 *   "When scaling down the performance level, the bus frequency should be
 *    first scaled down in order to not exceed the maximum frequency allowed
 *    for the low performance level.
 *
 *   "When scaling up the performance level (for example from PL0 to PL2),
 *    the bus frequency can be increased only once the performance level
 *    transition is completed, check the performance level status.
 *
 * Input Parameters:
 *   level - The new performance level
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sam_performance_level(uint8_t level)
{
  /* Check if we are already at this performance level */

  if (level != (getreg8(SAM_PM_PLCFG) & PM_PLCFG_PLSEL_MASK))
    {
      /* Clear performance level status and set the new performance level */

      putreg8(PM_INT_PLRDY, SAM_PM_INTFLAG);
      putreg8(level, SAM_PM_PLCFG);

      /* Wait for the new performance level to be ready */

      while ((getreg16(SAM_PM_INTFLAG) & PM_INT_PLRDY) == 0);
    }
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
 *     BOARD_XOSC_STARTUPTIME  - See OSCCTRL_XOSCCTRL_STARTUP_* definitions
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

  regval  = getreg16(SAM_OSCCTRL_XOSCCTRL);
  regval &= ~(OSCCTRL_XOSCCTRL_RUNSTDBY  | OSCCTRL_XOSCCTRL_ONDEMAND |
              OSCCTRL_XOSCCTRL_GAIN_MASK | OSCCTRL_XOSCCTRL_XTALEN   |
              OSCCTRL_XOSCCTRL_AMPGC    | OSCCTRL_XOSCCTRL_STARTUP_MASK);
  regval |= BOARD_XOSC_STARTUPTIME

#ifdef BOARD_XOSC_ISCRYSTAL
  /* XOSC is a crystal */

  regval |= OSCCTRL_XOSCCTRL_XTALEN;
#endif

#ifdef BOARD_XOSC_AMPGC
  /* Enable automatic gain control */

  regval |= OSCCTRL_XOSCCTRL_AMPGC;

#else
  /* Set gain if automatic gain control is not selected */

#if BOARD_XOSC_FREQUENCY <= 2000000
  regval |= OSCCTRL_XOSCCTRL_GAIN_2MHZ;
#elif BOARD_XOSC_FREQUENCY <= 4000000
  regval |= OSCCTRL_XOSCCTRL_GAIN_4MHZ;
#elif BOARD_XOSC_FREQUENCY <= 8000000
  regval |= OSCCTRL_XOSCCTRL_GAIN_8MHZ;
#elif BOARD_XOSC_FREQUENCY <= 16000000
  regval |= OSCCTRL_XOSCCTRL_GAIN_16MHZ;
#elif BOARD_XOSC_FREQUENCY <= 30000000
  regval |= OSCCTRL_XOSCCTRL_GAIN_30MHZ;
#else
#  error BOARD_XOSC_FREQUENCY out of range
#endif
#endif /* BOARD_XOSC_AMPGC */

#ifdef BOARD_XOSC_ONDEMAND
  regval |= OSCCTRL_XOSCCTRL_ONDEMAND;
#endif

#ifdef BOARD_XOSC_RUNINSTANDBY
  regval |= OSCCTRL_XOSCCTRL_RUNSTDBY;
#endif

  putreg16(regval, SAM_OSCCTRL_XOSCCTRL);

  /* Then enable the XOSC clock */

  regval |= OSCCTRL_XOSCCTRL_ENABLE;
  putreg16(regval, SAM_OSCCTRL_XOSCCTRL);
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
 *     BOARD_XOSC32K_STARTUPTIME  - See OSC32KCTRL_XOSC32K_STARTUP_* definitions
 *     BOARD_XOSC32K_ISCRYSTAL    - Boolean (defined / not defined)
 *     BOARD_XOSC32K_AAMPEN       - Boolean (defined / not defined)
 *     BOARD_XOSC32K_EN1KHZ       - Boolean (defined / not defined)
 *     BOARD_XOSC32K_EN32KHZ      - Boolean (defined / not defined)
 *     BOARD_XOSC32K_ONDEMAND     - Boolean (defined / not defined)
 *     BOARD_XOSC32K_RUNINSTANDBY - Boolean (defined / not defined)
 *     BOARD_XOSC32K_WRITELOCK    - Boolean (defined / not defined)
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

  /* Configure XOSC32K (skipping the ONDEMANC SETTING until last) */

  regval  = getreg16(SAM_OSC32KCTRL_XOSC32K);
  regval &= ~(OSC32KCTRL_XOSC32K_XTALEN   | OSC32KCTRL_XOSC32K_EN32K        |
              OSC32KCTRL_XOSC32K_EN1K     | OSC32KCTRL_XOSC32K_RUNSTDBY     |
              OSC32KCTRL_XOSC32K_ONDEMAND | OSC32KCTRL_XOSC32K_STARTUP_MASK |
              OSC32KCTRL_XOSC32K_WRTLOCK);
  regval |= BOARD_XOSC32K_STARTUPTIME;

#ifdef BOARD_XOSC32K_ISCRYSTAL
  regval |= OSC32KCTRL_XOSC32K_XTALEN;
#endif

#ifdef BOARD_XOSC32K_EN1KHZ
  regval |= OSC32KCTRL_XOSC32K_EN1K;
#endif

#ifdef BOARD_XOSC32K_EN32KHZ
  regval |= OSC32KCTRL_XOSC32K_EN32K;
#endif

#ifdef BOARD_XOSC32K_RUNINSTANDBY
  regval |= OSC32KCTRL_XOSC32K_RUNSTDBY;
#endif

  putreg16(regval, SAM_OSC32KCTRL_XOSC32K);

  /* Then enable the XOSC clock */

  regval |= OSC32KCTRL_XOSC32K_ENABLE;
  putreg16(regval, SAM_OSC32KCTRL_XOSC32K);

  /* Wait for XOSC32K to be ready */

  while ((getreg32(SAM_OSC32KCTRL_STATUS) & OSC32KCTRL_INT_XOSC32KRDY) == 0);

#ifdef BOARD_XOSC32K_ONDEMAND
  /* Set the on-demand bit */

  regval |= OSC32KCTRL_XOSC32K_ONDEMAND;
  putreg16(regval, SAM_OSC32KCTRL_XOSC32K);
#endif

#ifdef BOARD_XOSC32K_WRITELOCK
  /* Lock this configuration until the next power up */

  regval |= OSC32KCTRL_XOSC32K_WRTLOCK;
  putreg16(regval, SAM_OSC32KCTRL_XOSC32K);
#endif
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
 *     BOARD_OSC32K_STARTUPTIME  - See OSC32KCTRL_OSC32K_STARTUP_* definitions
 *     BOARD_OSC32K_EN1KHZ       - Boolean (defined / not defined)
 *     BOARD_OSC32K_EN32KHZ      - Boolean (defined / not defined)
 *     BOARD_OSC32K_ONDEMAND     - Boolean (defined / not defined)
 *     BOARD_OSC32K_RUNINSTANDBY - Boolean (defined / not defined)
 *     BOARD_OSC32K_WRITELOCK    - Boolean (defined / not defined)
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

  /* Configure OSC32K */

  regval  = getreg32(SAM_OSC32KCTRL_OSC32K);
  regval &= ~(OSC32KCTRL_OSC32K_EN32K        | OSC32KCTRL_OSC32K_EN1K     |
              OSC32KCTRL_OSC32K_RUNSTDBY     | OSC32KCTRL_OSC32K_ONDEMAND |
              OSC32KCTRL_OSC32K_STARTUP_MASK | OSC32KCTRL_OSC32K_WRTLOCK);
  regval |= BOARD_OSC32K_STARTUPTIME;

#ifdef BOARD_OSC32K_EN32KHZ
  regval |= OSC32KCTRL_OSC32K_EN32K;
#endif

#ifdef BOARD_OSC32K_EN1KHZ
  regval |= OSC32KCTRL_OSC32K_EN1K;
#endif

#ifdef BOARD_OSC32K_RUNINSTANDBY
  regval |= OSC32KCTRL_OSC32K_RUNSTDBY;
#endif

#ifdef BOARD_OSC32K_ONDEMAND
  regval |= OSC32KCTRL_OSC32K_ONDEMAND;
#endif

  putreg32(regval, SAM_OSC32KCTRL_OSC32K);

  /* Then enable OSC32K */

  regval |= OSC32KCTRL_OSC32K_ENABLE;
  putreg32(regval, SAM_OSC32KCTRL_OSC32K);

#ifdef BOARD_XOSC32K_WRITELOCK
  /* Lock this configuration until the next power up */

  regval |= OSC32KCTRL_OSC32K_WRTLOCK;
  putreg16(regval, SAM_OSC32KCTRL_OSC32K);
#endif
}
#else
#  define sam_osc32k_config()
#endif

/****************************************************************************
 * Name: sam_osculp32k_config
 *
 * Description:
 *   Configure OSCULP32K
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#define sam_osculp32k_config()

/****************************************************************************
 * Name: sam_osc16m_config
 *
 * Description:
 *   Configure OSC16M based on settings in the board.h header file.
 *   Depends on:
 *
 *     BOARD_OSC16M_FSEL          - See OSCCTRL_OSC16MCTRL_FSEL_* definitions
 *     BOARD_OSC16M_ONDEMAND      - Boolean (defined / not defined)
 *     BOARD_OSC16M_RUNINSTANDBY  - Boolean (defined / not defined)
 *
 *   On any reset the synchronous clocks start to their initial state:
 *
 *     OSC16M is enabled and divided by 8
 *     GCLK_MAIN uses OSC16M as source
 *     CPU and BUS clocks are undivided
 *
 *   The reset state of the OSC16M register is:
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
 *   NOTE that since we are running from OSC16M, it cannot be disable!
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void sam_osc16m_config(void)
{
  uint32_t regval;
  bool enabled;

  /* After reset, OSC16M is enabled and serve4s as the default clock source
   * at 4MHz.  Since this particular logic only runs on reset, there is
   * some additional unnecessary logic in the following.
   */

  /* Configure OSC16M */

  regval  = getreg32(SAM_OSCCTRL_OSC16MCTRL);

  /* Is OSC16M already enabled? Is it already running at the requested
   * frequency?
   */

  enabled = ((regval & OSCCTRL_OSC16MCTRL_ENABLE) !=  0);
  if (enabled && (regval & OSCCTRL_OSC16MCTRL_FSEL_MASK) == BOARD_OSC16M_FSEL)
    {
      regval &= ~(OSCCTRL_OSC16MCTRL_ONDEMAND | OSCCTRL_OSC16MCTRL_RUNSTDBY);

#ifdef BOARD_OSC16M_ONDEMAND
      /* Select on-demand oscillator controls */

      regval |= OSCCTRL_OSC16MCTRL_ONDEMAND;
#endif

#ifdef BOARD_OSC16M_RUNINSTANDBY
      /* The oscillator continues to run in standby sleep mode  */

      regval |= OSCCTRL_OSC16MCTRL_RUNSTDBY;
#endif

      /* Save the new OSC16M configuration */

      putreg32(regval, SAM_OSCCTRL_OSC16MCTRL);
    }

  /* Either the OSC16M is not running (which is not possible in this
   * context) or else OSC16M is configured to run at a different frequency.
   */

  else
    {
      /* If it is enabled, then we are probably running on OSC16M now.
       * Select OSCULP32K as new clock source for main clock temporarily.
       * This depends on the fact the GCLK0 is enabled at reset.
       */

      if (enabled)
        {
          sam_gclk_config(&g_gclk0_ulp32kconfig);

          /* Disable OSC16M clock */

          regval &= ~OSCCTRL_OSC16MCTRL_ENABLE;
          putreg32(regval, SAM_OSCCTRL_OSC16MCTRL);
        }

      /* Set the new OSC16M configuration */

      regval &= ~(OSCCTRL_OSC16MCTRL_FSEL_MASK | OSCCTRL_OSC16MCTRL_RUNSTDBY |
                  OSCCTRL_OSC16MCTRL_ONDEMAND);
      regval |= BOARD_OSC16M_FSEL;

#ifdef BOARD_OSC16M_RUNINSTANDBY
      /* The oscillator continues to run in standby sleep mode  */

      regval |= OSCCTRL_OSC16MCTRL_RUNSTDBY;
#endif

      /* Save the new OSC16M configuration */

      putreg32(regval, SAM_OSCCTRL_OSC16MCTRL);

      /* Enable OSC16M */

      regval |= OSCCTRL_OSC16MCTRL_ENABLE;
      putreg32(regval, SAM_OSCCTRL_OSC16MCTRL);

      /* Wait for OSC16M to be ready */

      while ((getreg32(SAM_OSCCTRL_STATUS) & OSCCTRL_INT_OSC16MRDY) == 0);

#ifdef BOARD_OSC16M_ONDEMAND
      /* Select on-demand oscillator controls */

      regval |= OSCCTRL_OSC16MCTRL_ONDEMAND;
      putreg32(regval, SAM_OSCCTRL_OSC16MCTRL);
#endif

      /* Re-select OSC16M for main clock again */

      if (enabled)
        {
          sam_gclk_config(&g_gclk0_default);
        }
    }
}

/****************************************************************************
 * Name: sam_dfll48m_config
 *
 * Description:
 *   Configure the DFLL48M based on settings in the board.h header file.
 *   Depends on:
 *
 *   BOARD_DFLL48M_CLOSEDLOOP          - Boolean (defined / not defined)
 *   BOARD_DFLL48M_OPENLOOP            - Boolean (defined / not defined)
 *   BOARD_DFLL48M_RECOVERY            - Boolean (defined / not defined)
 *   BOARD_DFLL48M_TRACKAFTERFINELOCK  - Boolean (defined / not defined)
 *   BOARD_DFLL48M_KEEPLOCKONWAKEUP    - Boolean (defined / not defined)
 *   BOARD_DFLL48M_ENABLECHILLCYCLE    - Boolean (defined / not defined)
 *   BOARD_DFLL48M_QUICKLOCK           - Boolean (defined / not defined)
 *   BOARD_DFLL48M_RUNINSTDBY          - Boolean (defined / not defined)
 *   BOARD_DFLL48M_ONDEMAND            - Boolean (defined / not defined)
 *   BOARD_DFLL48M_COARSEVALUE         - Value
 *   BOARD_DFLL48M_FINEVALUE           - Value
 *
 * Open Loop mode only:
 *   BOARD_DFLL48M_COARSEVALUE         - Value
 *   BOARD_DFLL48M_FINEVALUE           - Value
 *
 * Closed loop mode only:
 *   BOARD_DFLL48M_REFCLK_CLKGEN       - GCLK index in the range {0..8}
 *   BOARD_DFLL48M_MULTIPLIER          - Value
 *   BOARD_DFLL48M_MAXCOARSESTEP       - Value
 *   BOARD_DFLL48M_MAXFINESTEP         - Value
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef BOARD_DFLL48M_ENABLE
static inline void sam_dfll48m_config(void)
{
  uint16_t control;
  uint32_t regval;

  /* Disable ONDEMAND mode while writing configurations (Errata 9905).  This
   * is probably not necessary on the first time configuration after reset.
   */

  control  = getreg16(SAM_OSCCTRL_DFLLCTRL);
  control &= ~(OSCCTRL_DFLLCTRL_ENABLE | OSCCTRL_DFLLCTRL_ONDEMAND);
  putreg16(control, SAM_OSCCTRL_DFLLCTRL);

  /* Wait for the DFLL to synchronize */

  while ((getreg32(SAM_OSCCTRL_STATUS) & OSCCTRL_INT_DFLLRDY) == 0);

  /* Set up the DFLL control register */

  control &= ~(OSCCTRL_DFLLCTRL_MODE     | OSCCTRL_DFLLCTRL_STABLE |
               OSCCTRL_DFLLCTRL_LLAW     | OSCCTRL_DFLLCTRL_USBCRM |
               OSCCTRL_DFLLCTRL_RUNSTDBY | OSCCTRL_DFLLCTRL_CCDIS  |
               OSCCTRL_DFLLCTRL_QLDIS    | OSCCTRL_DFLLCTRL_BPLCKC |
               OSCCTRL_DFLLCTRL_WAITLOCK);

#if defined(BOARD_DFLL48M_CLOSEDLOOP)
  control |= OSCCTRL_DFLLCTRL_MODE;     /* Closed loop mode */
#elif defined(BOARD_DFLL48M_RECOVERY)
  control |= OSCCTRL_DFLLCTRL_USBCRM;   /* USB clock recovery mode */
#endif

#ifndef BOARD_DFLL48M_TRACKAFTERFINELOCK
  control |= OSCCTRL_DFLLCTRL_STABLE;   /* FINE calibration fixed after a fine lock */
#endif

#ifndef BOARD_DFLL48M_KEEPLOCKONWAKEUP
  control |= OSCCTRL_DFLLCTRL_LLAW;     /* Lose lock after wake */
#endif

#ifdef BOARD_DFLL48M_RUNINSTDBY
  control |= OSCCTRL_DFLLCTRL_RUNSTDBY; /* Run in standby */
#endif

#ifndef BOARD_DFLL48M_ENABLECHILLCYCLE
  control |= OSCCTRL_DFLLCTRL_CCDIS;    /* Chill cycle disable */
#endif

#ifndef BOARD_DFLL48M_QUICKLOCK
  control |= OSCCTRL_DFLLCTRL_QLDIS;    /* Quick lock disable */
#endif

#ifdef BOARD_DFLL48M_BPLCKC
  control |= OSCCTRL_DFLLCTRL_BPLCKC;   /* Bypass coarse clock */
#endif

#ifdef BOARD_DFLL48M_WAITLOCK
  control |= OSCCTRL_DFLLCTRL_WAITLOCK; /*  Wait lock */
#endif

  /* Then enable the DFLL (with ONDEMAND set to zero). */

  putreg16(control, SAM_OSCCTRL_DFLLCTRL);

  /* Wait for the DFLL to synchronize */

  while ((getreg32(SAM_OSCCTRL_STATUS) & OSCCTRL_INT_DFLLRDY) == 0);

  /* Set up the open loop mode multiplier register */

#ifndef BOARD_DFLL48M_OPENLOOP
  regval = OSCCTRL_DFLLMUL_CSTEP(BOARD_DFLL48M_MAXCOARSESTEP) |
           OSCCTRL_DFLLMUL_FSTEP(BOARD_DFLL48M_MAXFINESTEP) |
           OSCCTRL_DFLLMUL_MUL(BOARD_DFLL48M_MULTIPLIER);
  putreg32(regval, SAM_OSCCTRL_DFLLMUL);
#else
  putreg32(0, SAM_OSCCTRL_DFLLMUL);
#endif

  /* Set up the DFLL value register */

  regval = OSCCTRL_DFLLVAL_COARSE(BOARD_DFLL48M_COARSEVALUE) |
           OSCCTRL_DFLLVAL_FINE(BOARD_DFLL48M_FINEVALUE);
  putreg32(regval, SAM_OSCCTRL_DFLLVAL);
}
#else
#  define sam_dfll48m_config()
#endif

/****************************************************************************
 * Name: sam_dfll48m_enable
 *
 * Description:
 *   Enable the DFLL48M.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef BOARD_DFLL48M_ENABLE
static inline void sam_dfll48m_enable(void)
{
  uint16_t control;

  /* Enable the DFLL48M (with ONDEMAND still set to zero). */

  control  = getreg16(SAM_OSCCTRL_DFLLCTRL);
  control |= OSCCTRL_DFLLCTRL_ENABLE;   /* Enable the DFLL */
  putreg16(control, SAM_OSCCTRL_DFLLCTRL);

  /* Wait for the DFLL to synchronize */

  while ((getreg32(SAM_OSCCTRL_STATUS) & OSCCTRL_INT_DFLLRDY) == 0);

  /* Finally, set the state of the ONDEMAND bit if necessary */

#ifdef BOARD_DFLL48M_ONDEMAND
  control |= OSCCTRL_DFLLCTRL_ONDEMAND; /* On demand control */
  putreg16(control, SAM_OSCCTRL_DFLLCTRL);
#endif
}
#else
#  define sam_dfll48m_enable()
#endif

/****************************************************************************
 * Name: sam_dfll48m_refclk
 *
 * Description:
 *   Enable DFLL reference clock if in closed loop mode.
 *   Depends on:
 *
 *   BOARD_DFLL48M_REFCLK_CLKGEN - GCLK index in the range {0..8}
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(BOARD_GCLK_ENABLE) && defined(BOARD_DFLL48M_ENABLE) && \
   !defined(BOARD_DFLL48M_OPENLOOP)
static inline void sam_dfll48m_refclk(void)
{
  sam_gclk_chan_enable(GCLK_CHAN_DFLL48M_REF, BOARD_DFLL48M_REFCLK_CLKGEN);
}
#else
#  define sam_dfll48m_refclk()
#endif

/****************************************************************************
 * Name: sam_fdpll96m_config
 *
 * Description:
 *   Configure and enable the DFLL based on settings in the board.h header
 *   file.
 *   Depends on:
 *
 *     BOARD_FDPLL96M_ENABLE          - Boolean (defined / not defined)
 *     BOARD_FDPLL96M_RUNINSTDBY      - Boolean (defined / not defined)
 *     BOARD_FDPLL96M_ONDEMAND        - Boolean (defined / not defined)
 *     BOARD_FDPLL96M_LBYPASS         - Boolean (defined / not defined)
 *     BOARD_FDPLL96M_WUF             - Boolean (defined / not defined)
 *     BOARD_FDPLL96M_LPEN            - Boolean (defined / not defined)
 *     BOARD_FDPLL96M_FILTER          - See OSCCTRL_DPLLCTRLB_FILTER_* definitions
 *     BOARD_FDPLL96M_REFCLK          - See  OSCCTRL_DPLLCTRLB_REFLCK_* definitions
 *     BOARD_FDPLL96M_LOCKTIME        - See OSCCTRL_DPLLCTRLB_LTIME_* definitions
 *     BOARD_FDPLL96M_REFDIV          - Numeric value, 1 - 2047
 *     BOARD_FDPLL96M_PRESCALER       - See OSCCTRL_DPLLPRESC_* definitions
 *     BOARD_FDPLL96M_REFFREQ         - Numeric value
 *     BOARD_FDPLL96M_FREQUENCY       - Numeric value
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef BOARD_FDPLL96M_ENABLE
static inline void sam_fdpll96m_config(void)
{
  uint32_t ldr;
  uint32_t refclk;
  uint32_t regval;
  uint8_t  ldrfrac;
  uint8_t  ctrla;

  /* Get the reference clock frequency */

  refclk  = BOARD_FDPLL96M_REFFREQ;

#if BOARD_FDPLL96M_REFCLK == OSCCTRL_DPLLCTRLB_REFLCK_XOSC
  /* Only XOSC reference clock can be divided */

  refclk  = refclk / (2 * (BOARD_FDPLL96M_REFDIV + 1));
#endif

  /* Calculate LDRFRAC and LDR */

  ldr     = (BOARD_FDPLL96M_FREQUENCY << 4) / refclk;
  ldrfrac = (uint8_t)(ldr & 0x0f);
  ldr     = (ldr >> 4) - 1;

  /* Set DPLLCTRLA configuration (ut not the ONDEMAND bit) */

  ctrla   = 0;

#ifdef BOARD_FDPLL96M_RUNINSTDBY
  ctrla  |= OSCCTRL_DPLLCTRLA_RUNSTDBY;
#endif

  putreg8(ctrla, SAM_OSCCTRL_DPLLCTRLA);

  /* Set the FDPLL96M ration register */

  regval = OSCCTRL_DPLLRATIO_LDR(ldr) | OSCCTRL_DPLLRATIO_LDRFRAC(ldrfrac)
  putreg32(reval, SAM_OSCCTRL_DPLLRATIO);

  /* Wait for synchronization */

  while ((getreg8(SAM_OSCCTRL_DPLLSYNCBUSY) & OSCCTRL_DPLLSYNCBUSY_DPLLRATIO) != 0);

  /* Set DPLLCTRLB configuration */

  regval  = BOARD_FDPLL96M_FILTER | BOARD_FDPLL96M_LOCKTIME |
            BOARD_FDPLL96M_REFCLK |
            OSCCTRL_DPLLCTRLB_DIV(BOARD_FDPLL96M_REFDIV);

#ifdef BOARD_FDPLL96M_LBYPASS
  regval |= OSCCTRL_DPLLCTRLB_LBYPASS;
#endif
#ifdef BOARD_FDPLL96M_WUF
  regval |= OSCCTRL_DPLLCTRLB_WUF;
#endif
#ifdef BOARD_FDPLL96M_LPEN
  regval |= OSCCTRL_DPLLCTRLB_LPEN;
#endif

  putreg8(regval, SAM_OSCCTRL_DPLLCTRLA);

  /* Set the prescaler value */

  putreg8(BOARD_FDPLL96M_PRESCALER, SAM_ OSCCTRL_DPLLPRESC);

  /* Wait for synchronization */

  while ((getreg8(SAM_OSCCTRL_DPLLSYNCBUSY) & OSCCTRL_DPLLSYNCBUSY_DPLLPRESC) != 0);

  /* Enable the FDPLL96M output */

  ctrla |= OSCCTRL_DPLLCTRLA_ENABLE;
  putreg8(ctrla, SAM_OSCCTRL_DPLLCTRLA);

  /* Wait for synchronization */

  while ((getreg8(SAM_OSCCTRL_DPLLSYNCBUSY) & OSCCTRL_DPLLSYNCBUSY_ENABLE) != 0);

  /* Wait for the FPDLL96M to become locked and ready */

  while ((getreg8(SAM_OSCCTRL_DPLLSTATUS) &
         (OSCCTRL_DPLLSTATUS_CLKRDY | OSCCTRL_DPLLSTATUS_LOCK)) !=
         (OSCCTRL_DPLLSTATUS_CLKRDY | OSCCTRL_DPLLSTATUS_LOCK));

#ifdef BOARD_FDPLL96M_ONDEMAND
  /* Now set the ONDEMAND bit if so configured */

  ctrla |= OSCCTRL_DPLLCTRLA_ONDEMAND;
  putreg8(ctrla, SAM_OSCCTRL_DPLLCTRLA);
#endif
}
#else
#  define sam_fdpll96m_config()
#endif

/****************************************************************************
 * Name: sam_fdpll96m_refclk
 *
 * Description:
 *   Enable FDPLL96M internal lock timer and reference clock.
 *   Depends on:
 *
 *     BOARD_FDPLL96M_ENABLE          - Boolean (defined / not defined)
 *     BOARD_FDPLL96M_REFCLK          - See  OSCCTRL_DPLLCTRLB_REFLCK_* definitions
 *     BOARD_FDPLL96M_REFCLK_CLKGEN   - GCLK index in the range {0..8}
 *     BOARD_FDPLL96M_LOCKTIME_ENABLE - Boolean (defined / not defined)
 *     BOARD_FDPLL96M_LOCKTIME_CLKGEN - GCLK index in the range {0..8}
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(BOARD_GCLK_ENABLE) && defined(BOARD_FDPLL96M_ENABLE)
static inline void sam_fdpll96m_refclk(void)
{
#ifdef BOARD_FDPLL96M_LOCKTIME_ENABLE
  /* Enable the GCLK that is configured to the FDPLL lock timer */

  sam_gclk_chan_enable(GCLK_CHAN_DPLL_32K, BOARD_FDPLL96M_LOCKTIME_CLKGEN);
#endif

#if BOARD_FDPLL96M_REFCLK == OSCCTRL_DPLLCTRLB_REFLCK_GLCK
  /* Enable the GCLK that is configured to be the FDPLL reference clock */

  sam_gclk_chan_enable(GCLK_CHAN_DPLL, BOARD_FDPLL96M_REFCLK_CLKGEN);
#endif
}
#else
#  define sam_fdpll96m_refclk()
#endif

/****************************************************************************
 * Name: sam_cpu_dividers
 *
 * Description:
 *   Setup PM main clock dividers to generate CPU and AHB.
 *   Depends on:
 *
 *     BOARD_CPU_DIVIDER        - See MCLK_CPUDIV_DIV* definitions
 *     BOARD_CPU_FRQUENCY       - In Hz
 *     BOARD_CPU_FAILDECT       - Boolean (defined / not defined)
 *     BOARD_LOWPOWER_DIVIDER   - See MCLK_LPDIV_DIV_* definitions
 *     BOARD_LOWPOWER_FREQUENCY - In Hz
 *     BOARD_BACKUP_DIVIDER     - See MCLK_BUPDIV_DIV_* definitions
 *     BOARD_BACKUP_FREQUENCY   - In Hz
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void sam_cpu_dividers(void)
{
  uint8_t regval;

  /* Set CPU divider and, optionally, enable failure detection */

  putreg8(BOARD_CPU_DIVIDER, SAM_MCLK_CPUDIV);

  /* Enable/disabled clock failure detection */

  regval  = getreg8(SAM_MCLK_CTRLA);
#ifdef BOARD_CPU_FAILDECT
  regval |= MCLK_CTRLA_CFDEN;
#else
  regval &= ~MCLK_CTRLA_CFDEN;
#endif
  putreg8(regval, SAM_MCLK_CTRLA);

  /* Setup up lower power and backup dividers */

  putreg8(BOARD_LOWPOWER_DIVIDER, SAM_MCLK_LPDIV);
  putreg8(BOARD_BACKUP_DIVIDER, SAM_MCLK_BUPDIV);
}

/****************************************************************************
 * Name: sam_config_gclks
 *
 * Description:
 *   Configure GCLK(s) based on settings in the board.h header file.
 *   Depends on:
 *
 *   Global enable/disable.
 *
 *     BOARD_GCLK_ENABLE            - *MUST* be defined
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

  putreg8(GCLK_CTRLA_SWRST, SAM_GCLK_CTRLA);

  /* Wait for the reset to complete */

  while ((getreg8(SAM_GCLK_CTRLA) & GCLK_CTRLA_SWRST) != 0);

  /* Configure all GCLK generators, skipping GLCK_MAIN which is configured
   * below.
   */

  for (i = 1; i < NGCLKS_ENABLED; i++)
    {
      sam_gclk_config(&g_gclkconfig[i]);
    }
}
#else
#  define sam_config_gclks()
#endif

/****************************************************************************
 * Name: sam_periph_clocks
 *
 * Description:
 *   Setup initial peripheral clocking:
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void sam_periph_clocks(void)
{
#warning Missing logic
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

  putreg32(OSCCTRL_INT_ALL, SAM_OSCCTRL_INTFLAG);
  putreg32(OSC32KCTRL_INT_ALL, SAM_OSC32KCTRL_INTFLAG);
  putreg32(SUPC_INT_ALL, SAM_SUPC_INTFLAG);

  /* Set FLASH wait states */

  sam_flash_waitstates();

  /* Switch to PL2 to be sure configuration of GCLK0 is safe */

  sam_performance_level(PM_PLCFG_PLSEL_PL2);

  /* Configure XOSC */

  sam_xosc_config();

  /* Configure XOSC32K */

  sam_xosc32k_config();

  /* Configure OSCK32K */

  sam_osc32k_config();

  /* Configure OSCULPK32K */

  sam_osculp32k_config();

  /* Configure OSC16M */

  sam_osc16m_config();

  /* Configure DFLL48M */

  sam_dfll48m_config();

  /* Configure GCLK(s) */

  sam_config_gclks();

  /* Enable DFLL reference clock if the DFLL is enabled in closed loop mode */

  sam_dfll48m_refclk();

  /* Enable DFLL48M */

  sam_dfll48m_enable();

  /* Enable FDPLL reference clock if the DFLL is enabled */

  sam_fdpll96m_refclk();

  /* Configure and enable FDPLL96M */

  sam_fdpll96m_config();

  /* Setup CPU and BUS clocks */

  sam_cpu_dividers();

  /* Configure the GCLK_MAIN last as it may depend on the DFLL, FDPLL or
   * other generators
   */

  sam_gclk_config(&g_gclkconfig[0]);

#if BOARD_CPU_FREQUENCY <= 12000000
  /* If CPU frequency is less than 12MHz, scale down performance level to
   * PL0.
   */

  sam_performance_level(PM_PLCFG_PLSEL_PL0);
#endif

  /* Set up initial peripheral clocking */

  sam_periph_clocks();
}

#endif /* CONFIG_ARCH_FAMILY_SAML21 */
