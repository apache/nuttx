/****************************************************************************
 * arch/arm/src/stm32f7/stm32_lse.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
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

#include "stm32_rcc.h"
#include "stm32_pwr.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_rcc_enablelse
 *
 * Description:
 *   Enable the External Low-Speed (LSE) oscillator.
 *
 ****************************************************************************/

void stm32_rcc_enablelse(void)
{
  uint32_t regval;

  /* The LSE is in the RTC domain and write access is denied to this domain
   * after reset, you have to enable write access using DBP bit in the PWR CR
   * register before to configuring the LSE.
   */

  stm32_pwr_enablebkp(true);

  /* Enable the External Low-Speed (LSE) oscillator by setting the LSEON bit
   * the RCC BDCR register.
   */

  regval  = getreg32(STM32_RCC_BDCR);
  regval |= RCC_BDCR_LSEON;
  putreg32(regval,STM32_RCC_BDCR);

  /* Wait for the LSE clock to be ready */

  while (((regval = getreg32(STM32_RCC_BDCR)) & RCC_BDCR_LSERDY) == 0);

  /* Disable backup domain access if it was disabled on entry */

  stm32_pwr_enablebkp(false);
}
