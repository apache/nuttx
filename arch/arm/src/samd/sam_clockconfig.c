/****************************************************************************
 * arch/arm/src/samd/sam_clockconfig.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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
#warning Missing logic
}
#else
#  define sam_xosc_config()
#endif

/****************************************************************************
 * Name: sam_xosc32k_config
 *
 * Description:
 *   Configure XOSC32K based on settings in the board.h header file
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_SAMD_XOSC32K) || defined(BOARD_XOSC32_ENABLE)
static inline void sam_xosc32k_config(void)
{
#warning Missing logic
}
#else
#  define sam_xosc32k_config()
#endif

/****************************************************************************
 * Name: sam_osc32k_config
 *
 * Description:
 *   Configure OSC32K based on settings in the board.h header file
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_SAMD_OSC32K) || defined(BOARD_OSC32_ENABLE)
static inline void sam_osc32k_config(void)
{
#warning Missing logic
}
#else
#  define sam_osc32k_config()
#endif

/****************************************************************************
 * Name: sam_dfll_config
 *
 * Description:
 *   Configure the DFLL based on settings in the board.h header file
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
 * Name: sam_osc8m_config
 *
 * Description:
 *   Configure OSC8M based on settings in the board.h header file
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
#warning Missing logic
}

/****************************************************************************
 * Name: sam_gclk_config
 *
 * Description:
 *   Configure GCLK(s) based on settings in the board.h header file
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
