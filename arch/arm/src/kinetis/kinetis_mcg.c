/****************************************************************************
 * arch/arm/src/kinetis/kinetis_mcg.c
 * arch/arm/src/chip/kinetis_mcg.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#include "kinetis_internal.h"
#include "kinetis_mcg.h"
#include "kinetis_sim.h"
#include "kinetis_fmc.h"

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

#ifndef CONFIG_BOOT_RAMFUNCS
# error "CONFIG_BOOT_RAMFUNCS must be defined for this logic"
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
 
static void __ramfunc__
kinesis_setdividers(uint32_t div1, uint32_t div2, uint32_t div3, uint32_t div4);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kinesis_setdividers
 *
 * Description:
 *  "This routine must be placed in RAM. It is a workaround for errata e2448.
 *   Flash prefetch must be disabled when the flash clock divider is changed.
 *   This cannot be performed while executing out of flash.  There must be a
 *   short delay after the clock dividers are changed before prefetch can be
 *   re-enabled."
 *
 ****************************************************************************/

static void __ramfunc__
kinesis_setdividers(uint32_t div1, uint32_t div2, uint32_t div3, uint32_t div4)
{
  uint32_t regval;
  int i;

  /* Save the current value of the Flash Access Protection Register */

  regval = getreg32(KINETIS_FMC_PFAPR);
  
  /* Set M0PFD through M7PFD to 1 to disable prefetch */

  putreg32(FMC_PFAPR_M7PFD | FMC_PFAPR_M6PFD | FMC_PFAPR_M5PFD |
           FMC_PFAPR_M4PFD | FMC_PFAPR_M3PFD | FMC_PFAPR_M2PFD |
           FMC_PFAPR_M1PFD | FMC_PFAPR_M0PFD,
           KINETIS_FMC_PFAPR);

  /* Set clock dividers to desired value */

  putreg32(SIM_CLKDIV1_OUTDIV1(div1) | SIM_CLKDIV1_OUTDIV2(div2) |
           SIM_CLKDIV1_OUTDIV3(div3) | SIM_CLKDIV1_OUTDIV4(div4),
           KINETIS_SIM_CLKDIV1);

  /* Wait for dividers to change */

  for (i = 0 ; i < div4 ; i++);
  
  /* Re-store saved value of FMC_PFAPR */

  putreg32(regval, KINETIS_FMC_PFAPR);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/************************************************************************************
 * Name: kinetis_clockconfig
 *
 * Description:
 *   Called to initialize the Kinetis chip.  This does whatever setup is needed to
 *   put the  MCU in a usable state.  This includes the initialization of clocking
 *   using the settings in board.h.
 *
 ************************************************************************************/

void kinetis_clockconfig(void)
{
#warning "Missing logic"
}

