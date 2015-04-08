/****************************************************************************
 * arch/arm/src/sam34/sam4cm_supc.c
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
#include <time.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "chip/sam_supc.h"

#include "sam4cm_supc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Global Functions
 ****************************************************************************/

uint32_t supc_get_slcd_power_mode(void)
{
  return getreg32(SAM_SUPC_MR) & SUPC_MR_LCDMODE_MASK;
}

void supc_set_slcd_power_mode(uint32_t mode)
{
  uint32_t regval = getreg32(SAM_SUPC_MR);
  regval &= ~SUPC_MR_LCDMODE_MASK;
  regval |= SUPC_MR_KEY | mode;
  putreg32(regval, SAM_SUPC_MR);

  if (mode == SUPC_MR_LCDMODE_LCDOFF)
    {
      while(getreg32(SAM_SUPC_SR) & SUPC_SR_LCDS);
    }
  else
    {
      while(!(getreg32(SAM_SUPC_SR) & SUPC_SR_LCDS));
    }
}

void supc_set_slcd_ldo_output(uint32_t vrout)
{
  uint32_t regval = getreg32(SAM_SUPC_MR);
  regval &= ~SUPC_MR_LCDVROUT_MASK;
  regval |=  SUPC_MR_KEY | vrout;
  putreg32(regval, SAM_SUPC_MR);
}
