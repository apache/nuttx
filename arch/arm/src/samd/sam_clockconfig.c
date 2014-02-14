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
 * Private Data
 ****************************************************************************/

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

#if defined(CONFIG_SAMD_XOSC) || defined(BOARD_XOSC_ENABLE)
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

#if defined(CONFIG_SAMD_XOSC32K) || defined(BOARD_XOSC32K_ENABLE)
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

#if defined(CONFIG_SAMD_OSC32K) || defined(BOARD_OSC32K_ENABLE)
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

  regval = BOARD_OSC8M_PRESCALER;

#ifdef BOARD_OSC8M_ONDEMAND
  regval |= SYSCTRL_OSC8M_ONDEMAND;
#endif

#ifdef BOARD_OSC8M_RUNINSTANDBY
  regval |= SYSCTRL_OSC8M_RUNSTDBY;
#endif

  putreg32(regval, SAM_SYSCTRL_OSC8M);

  /* Then enable OSC8M */

  regval |= SYSCTRL_OSC8M_ENABLE;
  putreg32(regval, SAM_SYSCTRL_OSC8M);
}

/****************************************************************************
 * Name: sam_dfll_config
 *
 * Description:
 *   Configure the DFLL based on settings in the board.h header file.
 *   Depends on:
 *
 *     
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_SAMD_DFLL) || defined(BOARD_DFLL_ENABLE)
static inline void sam_dfll_config(void)
{
#warning Missing logic
}
#else
#  define sam_dfll_config()
#endif

/****************************************************************************
 * Name: sam_gclk_config
 *
 * Description:
 *   Configure GCLK(s) based on settings in the board.h header file.
 *   Depends on:
 *
 *     
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_SAMD_GCLK) || defined(BOARD_GCLK_ENABLE)
static inline void sam_gclk_config(void)
{
#warning Missing logic
}
#else
#  define sam_gclk_config()
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

  sam_gclk_config();

  /* Set CPU and BUS clock dividers */

  sam_dividers();
}
