/************************************************************************************
 * arch/arm/src/efm32/efm32_rmu.c
 *
 *   Copyright (C) 2015 Pierre-Noel Bouteville. All rights reserved.
 *   Author: Pierre-Noel Bouteville <pnb990@gmail.com>
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

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "up_arch.h"

#include "chip/efm32_emu.h"
#include "chip/efm32_rmu.h"

#include "emf32_rmu.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/************************************************************************************
 * Private Types
 ************************************************************************************/

/************************************************************************************
 * Private Data
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/* Variable old last reset cause of cpu. */

uint32_t g_efm32_rstcause;

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: efm32_rmu_initialize
 *
 * Description:
 *    Store reset cause into g_efm32_rstcause then clear reset cause register.
 *
 ************************************************************************************/

void efm32_rmu_initialize(void)
{
  uint32_t locked;
  
  g_efm32_rstcause = getreg32(EFM32_RMU_RSTCAUSE);

  /* Now clear reset cause */

  putreg32(RMU_CMD_RCCLR,EFM32_RMU_CMD);

  /* Clear some reset causes not cleared with RMU CMD register 
   * (If EMU registers locked, they must be unlocked first) 
   */

  locked = getreg32(EFM32_EMU_LOCK) & EMU_LOCK_LOCKKEY_LOCKED;
  if (locked)
    {
      /* EMU unlock */

      putreg32(EMU_LOCK_LOCKKEY_LOCK,EMU_LOCK_LOCKKEY_UNLOCK);
    }

  modifyreg32(EFM32_EMU_AUXCTRL,0,EMU_AUXCTRL_HRCCLR);
  modifyreg32(EFM32_EMU_AUXCTRL,EMU_AUXCTRL_HRCCLR,0);

  if (locked)
    {
      /* EMU lock */

      putreg32(EMU_LOCK_LOCKKEY_LOCK,EMU_LOCK_LOCKKEY_LOCK);
    }
}
