/************************************************************************************
 * configs/open1788/src/lpc17_nandinitialize.c
 * arch/arm/src/board/lpc17_nandinitialize.c
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <debug.h>

#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"

#include "open1788.h"

#if defined(CONFIG_LPC17_EMC) && defined(CONFIG_LPC17_EXTNAND)

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: open1788_nand_initialize
 *
 * Description:
 *   Initialize NAND FLASH
 *
 ************************************************************************************/

void open1788_nand_initialize(void)
{
  uint32_t regval;

  /* Set the memory width and byte lanes */

  regval = getreg32(LPC17_EMC_STATICCONFIG1);
  regval &= ~EMC_STATICCONFIG_MW_MASK;
  regbal |= (EMC_STATICCONFIG_MW_8BIT | EMC_STATICCONFIG_PB);
  putreg32(regval, LPC17_EMC_STATICCONFIG1);

  /* Configure timing */

  putreg32(2, LPC17_EMC_STATICWAITWEN1);
  putreg32(2, LPC17_EMC_STATICWAITOEN1);
  putreg32(31, LPC17_EMC_STATICWAITRD1);
  putreg32(31, LPC17_EMC_STATICWAITPAGE1);
  putreg32(31, LPC17_EMC_STATICWAITWR1);
  putreg32(31, LPC17_EMC_STATICWAITTURN1);

  /* GPIO P2[21] connects to the Ready/Busy pin of the NAND part.  We need to
   * reconfigure this pin as normal GPIO input.
   */

  lpc17_gpioconfig(GPIO_NAND_RB);
}

#endif /* CONFIG_LPC17_EMC && CONFIG_LPC17_EXTNAND */
