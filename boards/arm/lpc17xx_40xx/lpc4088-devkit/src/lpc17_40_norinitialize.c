/****************************************************************************
 * boards/arm/lpc17xx_40xx/lpc4088-devkit/src/lpc17_40_norinitialize.c
 * arch/arm/src/board/lpc17_40_norinitialize.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

#include <debug.h>

#include <arch/board/board.h>

#include "arm_arch.h"
#include "arm_internal.h"

#include "lpc4088-devkit.h"

#if defined(CONFIG_LPC17_40_EMC) && defined(CONFIG_LPC17_40_EXTNOR)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc4088_devkit_nor_initialize
 *
 * Description:
 *   Initialize NOR FLASH
 *
 ****************************************************************************/

void lpc4088_devkit_nor_initialize(void)
{
  uint32_t regval;

  /* Set the memory width and byte lanes */

  regval = getreg32(LPC17_40_EMC_STATICCONFIG0);
  regval &= ~EMC_STATICCONFIG_MW_MASK;
  regbal |= (EMC_STATICCONFIG_MW_16BIT | EMC_STATICCONFIG_PB);
  putreg32(regval, LPC17_40_EMC_STATICCONFIG0);

  /* Configure timing */

  putreg32(2, LPC17_40_EMC_STATICWAITWEN0);
  putreg32(2, LPC17_40_EMC_STATICWAITOEN0);
  putreg32(31, LPC17_40_EMC_STATICWAITRD0);
  putreg32(31, LPC17_40_EMC_STATICWAITPAGE0);
  putreg32(31, LPC17_40_EMC_STATICWAITWR0);
  putreg32(31, LPC17_40_EMC_STATICWAITTURN0);

  up_mdelay(10);
}

#endif /* CONFIG_LPC17_40_EMC && CONFIG_LPC17_40_EXTNOR */
