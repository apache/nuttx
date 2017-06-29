/****************************************************************************
 * arch/arm/src/tms570/tms570_esm.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Derives from the TI "Project0" sample code which has a compatible 3-
 * clause BSD license:
 *
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
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
 * 3. Neither the name NuttX, Texas Instruments Incorporated, nor the
 *    names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
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

#include <sys/types.h>
#include <stdint.h>
#include <debug.h>

#include <arch/irq.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip/tms570_esm.h"
#include "tms570_esm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  tms570_esm_initialize
 *
 * Description:
 *   Initialize the ESM.
 *
 ****************************************************************************/

int tms570_esm_initialize(void)
{
  uint32_t regval;

  /* Disable error pin channels */

  putreg32(0xffffffff, TMS570_ESM_DEPAPR1);
  putreg32(0xffffffff, TMS570_ESM_IEPCR4);

  /* Disable interrupts */

  putreg32(0xffffffff, TMS570_ESM_IECR1);
  putreg32(0xffffffff, TMS570_ESM_IECR4);

  /* Clear error status flags */

  putreg32(0xffffffff, TMS570_ESM_SR1);
  putreg32(0xffffffff, TMS570_ESM_SR2);
  putreg32(0xffffffff, TMS570_ESM_SSR2);
  putreg32(0xffffffff, TMS570_ESM_SR3);
  putreg32(0xffffffff, TMS570_ESM_SR4);
  putreg32(0xffffffff, TMS570_ESM_SR5);
  putreg32(0xffffffff, TMS570_ESM_SSR5);
  putreg32(0xffffffff, TMS570_ESM_SR6);

  /* Setup LPC preload */

  putreg32(16384 - 1, TMS570_ESM_LTCPR);

  /* Reset error pin */

  regval = getreg32(TMS570_ESM_EPSR);
  if (regval == 0)
    {
      putreg32(5, TMS570_ESM_EKR);
    }
  else
    {
      putreg32(0, TMS570_ESM_EKR);
    }

  /* Clear interrupt level.  All channels set to low level interrupt. */

  putreg32(0xffffffff, TMS570_ESM_ILCR1);
  putreg32(0xffffffff, TMS570_ESM_ILCR4);

  /* Set interrupt level (Writing zero does nothing) */

  putreg32(0, TMS570_ESM_ILSR1);
  putreg32(0, TMS570_ESM_ILSR4);

  /* Enable error pin channels */

  putreg32(0, TMS570_ESM_EEPAPR1);
  putreg32(0, TMS570_ESM_IEPSR4);

  /* Enable interrupts */

  putreg32(0, TMS570_ESM_IESR1);
  putreg32(0, TMS570_ESM_IESR4);
  return OK;
}

/****************************************************************************
 * Name:  tms570_esm_interrupt
 *
 * Description:
 *   ESM interrupt handler
 *
 ****************************************************************************/

int tms570_esm_interrupt(int irq, void *context, FAR void *arg)
{
  /* Save the saved processor context in CURRENT_REGS where it can be accessed
   * for register dumps and possibly context switching.
   */

  CURRENT_REGS = (uint32_t *)context;

  /* Crash -- possibly showing diagnostic debug information. */

  _err("ERROR: ESM Interrupt. PC: %08x\n", CURRENT_REGS[REG_PC]);
  PANIC();
  return OK; /* To keep the compiler happy */
}
