/****************************************************************************
 * arch/avr/src/sam34/sam4l_periphclks.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This file is derived from nuttx/arch/avr/src/at32uc3/at32uc3_clkinit.c
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

#include <arch/irq.h>
#include <arch/board/board.h>

#include "up_arch.h"

#include "up_internal.h"
#include "chip/sam4l_pm.h"

#include "sam4l_periphclks.h"

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_modifyperipheral
 *
 * Description:
 *   This is a convenience function that is intended to be used to enable
 *   or disable peripheral module clocking.
 *
 ****************************************************************************/

void sam_modifyperipheral(uintptr_t regaddr, uint32_t clrbits, uint32_t setbits)
{
  irqstate_t flags;
  uint32_t regval;

  /* Make sure that the following operations are atomic */

  flags = irqsave();

  /* Enable/disabling clocking */

  regval  = getreg32(regaddr);
  regval &= ~clrbits;
  regval |= setbits;
  putreg32(PM_UNLOCK_KEY(0xaa) | PM_UNLOCK_ADDR(regaddr - SAM_PM_BASE), SAM_PM_UNLOCK);
  putreg32(regval, regaddr);

  irqrestore(flags);
}

/****************************************************************************
 * Name: sam_pba_modifydivmask
 *
 * Description:
 *   This is a convenience function that is intended to be used to modify
 *   bits in the PBA divided clock (DIVMASK) register.
 *
 ****************************************************************************/

void sam_pba_modifydivmask(uint32_t clrbits, uint32_t setbits)
{
  irqstate_t flags;
  uint32_t regval;

  /* Make sure that the following operations are atomic */

  flags = irqsave();

  /* Modify the PBA DIVMASK */

  regval  = getreg32(SAM_PM_PBADIVMASK);
  regval &= ~clrbits;
  regval |= setbits;
  putreg32(PM_UNLOCK_KEY(0xaa) | PM_UNLOCK_ADDR(SAM_PM_PBADIVMASK_OFFSET), SAM_PM_UNLOCK);
  putreg32(regval, SAM_PM_PBADIVMASK);

  irqrestore(flags);
}

/****************************************************************************
 * Name: sam_pba_enableperipheral
 *
 * Description:
 *   This is a convenience function to enable a peripheral on the APBA
 *   bridge.
 *
 ****************************************************************************/

void sam_pba_enableperipheral(uint32_t bitset)
{
  irqstate_t flags;

  /* The following operations must be atomic */

  flags = irqsave();

  /* Enable the APBA bridge if necessary */

  if (getreg32(SAM_PM_PBAMASK) == 0)
    {
      sam_hsb_enableperipheral(PM_HSBMASK_APBA);
    }

  irqrestore(flags);

  /* Enable the module */

  sam_enableperipheral(SAM_PM_PBAMASK, bitset);
}

/****************************************************************************
 * Name: sam_pba_disableperipheral
 *
 * Description:
 *   This is a convenience function to disable a peripheral on the APBA
 *   bridge.
 *
 ****************************************************************************/

void sam_pba_disableperipheral(uint32_t bitset)
{
  irqstate_t flags;

  /* Disable clocking to the module */

  sam_disableperipheral(SAM_PM_PBAMASK, bitset);

  /* Disable the APBA bridge if possible */

  flags = irqsave();

  if (getreg32(SAM_PM_PBAMASK) == 0)
    {
      sam_hsb_disableperipheral(PM_HSBMASK_APBA);
    }

   /* Disable PBA UART divided clock if none of the UARTS are in use */

  if ((getreg32(SAM_PM_PBAMASK) & PM_PBAMASK_UARTS) == 0)
    {
      sam_pba_disabledivmask(PM_PBADIVMASK_CLK_USART);
    }

   /* Disable PBA TIMER divided clocks if none of the UARTS are in use */

  if ((getreg32(SAM_PM_PBAMASK) & PM_PBAMASK_TIMERS) == 0)
    {
      sam_pba_disabledivmask(PM_PBADIVMASK_TIMER_CLOCKS);
    }

  irqrestore(flags);
}

/****************************************************************************
 * Name: sam_pbb_enableperipheral
 *
 * Description:
 *   This is a convenience function to enable a peripheral on the APBB
 *   bridge.
 *
 ****************************************************************************/

void sam_pbb_enableperipheral(uint32_t bitset)
{
  irqstate_t flags;

  /* The following operations must be atomic */

  flags = irqsave();

  /* Enable the APBB bridge if necessary */

  if (getreg32(SAM_PM_PBBMASK) == 0)
    {
      sam_hsb_enableperipheral(PM_HSBMASK_APBB);
    }

  irqrestore(flags);

  /* Enable the module */

  sam_enableperipheral(SAM_PM_PBBMASK, bitset);
}

/****************************************************************************
 * Name: sam_pbb_disableperipheral
 *
 * Description:
 *   This is a convenience function to disable a peripheral on the APBA
 *   bridge.
 *
 ****************************************************************************/

void sam_pbb_disableperipheral(uint32_t bitset)
{
  irqstate_t flags;

  /* Disable clocking to the peripheral module */

  sam_disableperipheral(SAM_PM_PBBMASK, bitset);

  /* Disable the APBB bridge if possible */

  flags = irqsave();

  if (getreg32(SAM_PM_PBBMASK) == 0)
    {
      sam_hsb_disableperipheral(PM_HSBMASK_APBB);
    }

  irqrestore(flags);
}
