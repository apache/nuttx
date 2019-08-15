/****************************************************************************
 * arch/arm/src/s32k1xx/s32k1xx_clockconfig.c
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
 * Portions of the logic within this file derives from NXP sample code for
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <debug.h>

#include <nuttx/arch.h>

#include "up_arch.h"
#include "up_internal.h"

#include "hardware/s32k1xx_scg.h"
#include "s32k1xx_clockconfig.h"

#include <arch/board/board.h>  /* Include last.  May have dependencies */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k1xx_clockconfig
 *
 * Description:
 *   Called to initialize the S32K1XX.  This does whatever setup is needed
 *   to put the MCU in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.  This function also performs
 *   other low-level chip as necessary.
 *
 *****************************************************************************/

void s32k1xx_clockconfig(FAR const struct clock_configuration_s *clkcfg)
{
#warning Missing logic
}

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
 *****************************************************************************/

uint32_t s32k1xx_get_coreclk(void)
{
  uint32_t coreclk = 0;
  uint32_t regval;
  uint32_t divider;
#ifdef CONFIG_ARCH_CHIP_S32K14X
  uint32_t prediv;
  uint32_t mult;
#endif

  /* Get the core clock divider */

  regval  = getreg32(S32K1XX_SCG_CSR);
  divider = ((regval & SCG_CSR_DIVCORE_MASK) >> SCG_CSR_DIVCORE_SHIFT) + 1;

  /* Handle according to the selection clock source */

  switch (regval & SCG_CSR_SCS_MASK)
    {
      case SCG_CSR_SCS_SOSC:  /* System OSC */

        coreclk = BOARD_XTAL_FREQUENCY;
        break;

      case SCG_CSR_SCS_SIRC:  /* Slow IRC */
        regval = getreg32(S32K1XX_SCG_SIRCCFG) & SCG_SIRCCFG_RANGE;
        if (regval == SCG_SIRCCFG_LOWRANGE)
          {
            /* Slow IRC low range clock (2 MHz) */

            return 0;
          }

        /* Slow IRC high range clock (8 MHz ) */

        coreclk = SCG_SIRQ_HIGHRANGE_FREQUENCY;
        break;

      case SCG_CSR_SCS_FIRC:  /* Fast IRC */
        regval = getreg32(S32K1XX_SCG_FIRCCFG) & SCG_FIRCCFG_RANGE;
        if (regval != SCG_FIRCCFG_48MHZ)
          {
            return 0;
          }

        /* Fast IRC is trimmed to 48 MHz */

        coreclk = SCG_FIRQ_FREQUENCY0;
        break;

#ifdef CONFIG_ARCH_CHIP_S32K14X
      case 0x6SCG_CSR_SPLL_FIRC:  /* System PLL */
        /* Coreclock = Fxtal * mult / (2 * prediv) */

        regval  = getreg32(S32K1XX_SCG_SPLLCFG);
        prediv  = ((regval & SCG_SPLLCFG_PREDIV_MASK) >> SCG_SPLLCFG_PREDIV_SHIFT) + 1;
        mult    = ((regval & SCG_SPLLCFG_MULT_MASK) >> SCG_SPLLCFG_MULT_SHIFT) + 16;

          coreclk = BOARD_XTAL_FREQUENCY * mult / (2 * prediv);
        break;
#endif

      default:
        return 0;
    }

  return coreclk / divider;
}
