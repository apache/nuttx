/****************************************************************************
 * arch/arm/src/samdl/saml_clockconfig.c
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

#include <arch/board/board.h>

#include "saml_periphclks.h"
#include "sam_clockconfig.h"

#ifdef CONFIG_ARCH_FAMILY_SAML21

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
static void sam_gclck_waitsyncbusy(void);
static void sam_gclk_config(FAR const struct sam_gclkconfig_s *config);
#ifdef BOARD_GCLK_ENABLE
static inline void sam_config_gclks(void);
#endif
static inline void sam_dividers(void);

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

  regval  = getreg32(SAM_NVMCTRL_CTRLB);
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

  /* Configure XOSC32K */

  regval  = getreg16(SAM_OSC32KCTRL_XOSC32K);
  regval &= ~(OSC32KCTRL_XOSC32K_XTALEN   | OSC32KCTRL_XOSC32K_EN32K        |
              OSC32KCTRL_XOSC32K_EN1K     | OSC32KCTRL_XOSC32K_RUNSTDBY     |
              OSC32KCTRL_XOSC32K_ONDEMAND | OSC32KCTRL_XOSC32K_STARTUP_MASK |
              OSC32KCTRL_XOSC32K_WRTLOCK);
  regval |= BOARD_XOSC32K_STARTUPTIME

#ifdef BOARD_XOSC32K_ISCRYSTAL
  regval |= OSC32KCTRL_XOSC32K_XTALEN;
#endif

#ifdef BOARD_XOSC32K_AAMPEN
  regval |= OSC32KCTRL_XOSC32K_AAMPEN;
#endif

#ifdef BOARD_XOSC32K_EN1KHZ
  regval |= OSC32KCTRL_XOSC32K_EN1K;
#endif

#ifdef BOARD_XOSC32K_EN32KHZ
  regval |= OSC32KCTRL_XOSC32K_EN32K;
#endif

#ifdef BOARD_XOSC32K_ONDEMAND
  regval |= OSC32KCTRL_XOSC32K_ONDEMAND;
#endif

#ifdef BOARD_XOSC32K_RUNINSTANDBY
  regval |= OSC32KCTRL_XOSC32K_RUNSTDBY;
#endif

  putreg16(regval, SAM_OSC32KCTRL_XOSC32K);

  /* Then enable the XOSC clock */

  regval |= OSC32KCTRL_XOSC32K_ENABLE;
  putreg16(regval, SAM_OSC32KCTRL_XOSC32K);

  /* Wait for XOSC32K to be ready */

  while ((getreg32(SAM_OSC32CTRL_STATUS) & OSC32KCTRL_INT_XOSC32KRDY) == 0);

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

      /* Enable OSC16M */

      regval |= OSCCTRL_OSC16MCTRL_ENABLE;
      putreg32(regval, SAM_OSCCTRL_OSC16MCTRL);

      /* Wait for OSC16M to be ready */

      while ((getreg32(SAM_OSCCTRL_STATUS) & OSCCTRL_INT_OSC16MRDY) == 0);

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
 *   BOARD_DFLL48M_SRCGCLKGEN          - See GCLK_CLKCTRL_GEN* definitions
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

#if defined(BOARD_DFLL48M_CLOSELOOP)
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

#ifndef BOARD_DFLL48M_RUNINSTDBY
  control |= OSCCTRL_DFLLCTRL_RUNSTDBY;    /* Chill cycle disable */
#endif

#ifndef BOARD_DFLL48M_ENABLECHILLCYCLE
  control |= OSCCTRL_DFLLCTRL_CCDIS;    /* Chill cycle disable */
#endif

#ifndef BOARD_DFLL48M_QUICKLOCK
  control |= OSCCTRL_DFLLCTRL_QLDIS; /* Quick lock disable */
#endif

#ifndef BOARD_DFLL48M_BPLCKC
  control |= OSCCTRL_DFLLCTRL_BPLCKC; /* Bypass coarse clock */
#endif

#ifndef BOARD_DFLL48M_WAITLOCK
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
 *   BOARD_DFLL48M_SRCGCLKGEN - See GCLK_CLKCTRL_GEN* definitions
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

  regval = (BOARD_DFLL48M_SRCGCLKGEN | GCLK_CLKCTRL_ID_DFLL48M);
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
 *   BOARD_FDPLL96M_OPENLOOP            - Boolean (defined / not defined)
 *   BOARD_FDPLL96M_TRACKAFTERFINELOCK  - Boolean (defined / not defined)
 *   BOARD_FDPLL96M_KEEPLOCKONWAKEUP    - Boolean (defined / not defined)
 *   BOARD_FDPLL96M_ENABLECHILLCYCLE    - Boolean (defined / not defined)
 *   BOARD_FDPLL96M_QUICKLOCK           - Boolean (defined / not defined)
 *   BOARD_FDPLL96M_ONDEMAND            - Boolean (defined / not defined)
 *   BOARD_FDPLL96M_COARSEVALUE         - Value
 *   BOARD_FDPLL96M_FINEVALUE           - Value
 *
 * Open Loop mode only:
 *   BOARD_FDPLL96M_COARSEVALUE         - Value
 *   BOARD_FDPLL96M_FINEVALUE           - Value
 *
 * Closed loop mode only:
 *   BOARD_FDPLL96M_SRCGCLKGEN          - See GCLK_CLKCTRL_GEN* definitions
 *   BOARD_FDPLL96M_MULTIPLIER          - Value
 *   BOARD_FDPLL96M_MAXCOARSESTEP       - Value
 *   BOARD_FDPLL96M_MAXFINESTEP         - Value
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
#error Missing logic
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
 *   BOARD_FDPLL96M_SRCGCLKGEN - See GCLK_CLKCTRL_GEN* definitions
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
}
#else
#  define sam_fdpll96m_enable()
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
  while ((getreg8(SAM_GCLK_SYNCHBUSY) & GCLK_SYNCHBUSY_SYNCBUSY) != 0);
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

static void sam_gclk_config(FAR const struct sam_gclkconfig_s *config)
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

  /* Enable DFLL reference clock if the DFLL is enabled in closed loop mode */

  sam_dfll48m_refclk();

  /* Enable FDPLL reference clock if the DFLL is enabled */

  sam_fdpll96m_refclk();

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

  regval  = getreg8(SAM_PM_CTRLA);
#ifdef BOARD_CPU_FAILDECT
  regval |= PM_CTRLA_CFDEN;
#else
  regval &= ~PM_CTRLA_CFDEN;
#endif
  putreg8(regval, SAM_PM_CTRLA);

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

  /* Enable DFLL48M */

  sam_dfll48m_enable();

  /* Configure and enable FDPLL96M */

  sam_fdpll96m_config();

  /* Set CPU and BUS clock dividers */

  sam_dividers();
}

#endif /* CONFIG_ARCH_FAMILY_SAML21 */
