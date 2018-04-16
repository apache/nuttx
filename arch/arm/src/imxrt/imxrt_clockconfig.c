/****************************************************************************
 * arch/arm/src/imxrt/imxrt_clockconfig.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author:  Janne Rosberg <janne@offcode.fi>
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

#include "up_arch.h"
#include <arch/board/board.h>
#include "chip/imxrt_ccm.h"
#include "chip/imxrt_dcdc.h"
#include "imxrt_clockconfig.h"
#include "chip/imxrt105x_memorymap.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_clockconfig
 *
 * Description:
 *   Called to initialize the i.MXRT.  This does whatever setup is needed to
 *   put the SoC in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.
 *
 ****************************************************************************/

void imxrt_clockconfig(void)
{
  /* Don't change the current basic clock configuration if we are running
   * from SDRAM.  In this case, some bootloader logic has already configured
   * clocking and SDRAM.  We are pretty much committed to using things the
   * way that the bootloader has left them.
   */

#ifndef CONFIG_IMXRT_BOOT_SDRAM
  uint32_t reg;

  /* Set clock mux and dividers */

  /* Set PERIPH_CLK2 MUX to OSC */

  reg = getreg32(IMXRT_CCM_CBCMR);
  reg &= ~CCM_CBCMR_PERIPH_CLK2_SEL_MASK;
  reg |= CCM_CBCMR_PERIPH_CLK2_SEL_OSC_CLK;
  putreg32(reg, IMXRT_CCM_CBCMR);

  /* Set PERIPH_CLK MUX to PERIPH_CLK2 */

  reg = getreg32(IMXRT_CCM_CBCDR);
  reg |= CCM_CBCDR_SEMC_PERIPH_CLK_SEL;
  putreg32(reg, IMXRT_CCM_CBCDR);

  /* Wait handshake */

  while ((getreg32(IMXRT_CCM_CDHIPR) & CCM_CDHIPR_PERIPH2_CLK_SEL_BUSY) == 1)
    {
    }

  /* Set Soc VDD */

  reg = getreg32(IMXRT_DCDC_REG3);
  reg &= ~(DCDC_REG3_TRG_MASK);
  reg |= DCDC_REG3_TRG(IMXRT_VDD_SOC);
  putreg32(reg, IMXRT_DCDC_REG3);

  /* Init Arm PLL1 */

  reg = CCM_ANALOG_PLL_ARM_DIV_SELECT(IMXRT_ARM_PLL_SELECT) | CCM_ANALOG_PLL_ARM_ENABLE;
  putreg32(reg, IMXRT_CCM_ANALOG_PLL_ARM);
  while ((getreg32(IMXRT_CCM_ANALOG_PLL_ARM) & CCM_ANALOG_PLL_ARM_LOCK) == 0)
    {
    }

  /* Init Sys PLL2 */

  reg = CCM_ANALOG_PLL_SYS_DIV_SELECT(IMXRT_SYS_PLL_SELECT) | CCM_ANALOG_PLL_SYS_ENABLE;
  putreg32(reg, IMXRT_CCM_ANALOG_PLL_SYS);
  while ((getreg32(IMXRT_CCM_ANALOG_PLL_SYS) & CCM_ANALOG_PLL_SYS_LOCK) == 0)
    {
    }

  /* TODO: other pll configs */

  /* Set Dividers */

  reg = getreg32(IMXRT_CCM_CACRR);
  reg &= ~CCM_CACRR_ARM_PODF_MASK;
  reg |= CCM_CACRR_ARM_PODF(IMXRT_ARM_CLOCK_DIVIDER);
  putreg32(reg, IMXRT_CCM_CACRR);

  reg = getreg32(IMXRT_CCM_CBCDR);
  reg &= ~CCM_CBCDR_AHB_PODF_MASK;
  reg |= CCM_CBCDR_AHB_PODF(IMXRT_AHB_CLOCK_DIVIDER);
  putreg32(reg, IMXRT_CCM_CBCDR);

  reg = getreg32(IMXRT_CCM_CBCDR);
  reg &= ~CCM_CBCDR_IPG_PODF_MASK;
  reg |= CCM_CBCDR_IPG_PODF(IMXRT_IPG_CLOCK_DIVIDER);
  putreg32(reg, IMXRT_CCM_CBCDR);

  /* Set PRE_PERIPH_CLK to PLL1 */

  reg = getreg32(IMXRT_CCM_CBCMR);
  reg &= ~CCM_CBCMR_PRE_PERIPH_CLK_SEL_MASK;
  reg |= CCM_CBCMR_PRE_PERIPH_CLK_SEL_PLL1;
  putreg32(reg, IMXRT_CCM_CBCMR);

  /* Set PERIPH_CLK MUX to PRE_PERIPH_CLK2 */

  reg = getreg32(IMXRT_CCM_CBCDR);
  reg &= ~CCM_CBCDR_SEMC_PERIPH_CLK_SEL;
  putreg32(reg, IMXRT_CCM_CBCDR);

  /* Wait handshake */

  while ((getreg32(IMXRT_CCM_CDHIPR) & CCM_CDHIPR_PERIPH2_CLK_SEL_BUSY) == 1)
  {
  }

  /* Set UART source to PLL3 80M */

  reg = getreg32(IMXRT_CCM_CSCDR1);
  reg &= CCM_CSCDR1_UART_CLK_SEL;
  reg |= CCM_CSCDR1_UART_CLK_SEL_PLL3_80;
  putreg32(reg, IMXRT_CCM_CSCDR1);

  /* Set UART divider to 1 */

  reg = getreg32(IMXRT_CCM_CSCDR1);
  reg &= CCM_CSCDR1_UART_CLK_PODF_MASK;
  reg |= CCM_CSCDR1_UART_CLK_PODF(0);
  putreg32(reg, IMXRT_CCM_CSCDR1);
#endif
}
