/****************************************************************************
 * arch/arm/src/tiva/cc13xx/cc13x2_aux_sysif.c
 * Driver for the AUX System Interface
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *
 * Technical content derives from a TI aux_sysif.c file that has a
 * compatible BSD license:
 *
 *   Copyright (c) 2015-2017, Texas Instruments Incorporated
 *   All rights reserved.
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

#include <stdint.h>
#include "arm_arch.h"

#include "hardware/tiva_aux_sysif.h"
#include "cc13xx/cc13x2_aux_sysif.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Used in aux_sysif_opmode() to control the change of the operational
 * mode.
 */

static const uint8_t g_opmode_to_order[4] =
{
  1, 2, 0, 3
};

static const uint8_t g_order_to_opmode[4] =
{
  2, 0, 1, 3
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: aux_sysif_opmode
 *
 * Description:
 *
 *   This function controls the change of the AUX operational mode.
 *   The function controls the change of the current operational mode to the
 *   operational mode target by adhering to rules specified by HW.
 *
 * Input Parameters:
 *   - opmode:  AUX operational mode.  One of
 *              AUX_SYSIF_OPMODE_TARGET_PDLP: Power down operational mode
 *                                            with wakeup to low power mode)
 *              AUX_SYSIF_OPMODE_TARGET_PDA:  Power down operational mode
 *                                            with wakeup to active mode
 *              AUX_SYSIF_OPMODE_TARGET_LP:   Low power operational mode)
 *              AUX_SYSIF_OPMODE_TARGET_A:    Active operational mode
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void aux_sysif_opmode(uint32_t opmode)
{
  uint32_t currmode;
  uint32_t currorder;
  uint32_t nextmode;

  do
    {
      currmode = getreg32(TIVA_AUX_SYSIF_OPMODEREQ);
      while (currmode != getreg32(TIVA_AUX_SYSIF_OPMODEACK))
        {
        }

      if (currmode != opmode)
        {
          currorder = g_opmode_to_order[currmode];
          if (currorder < g_opmode_to_order[opmode])
            {
              nextmode = g_order_to_opmode[currorder + 1];
            }
          else
            {
              nextmode = g_order_to_opmode[currorder - 1];
            }

          putreg32(nextmode, TIVA_AUX_SYSIF_OPMODEREQ);
        }
    }
  while (currmode != opmode);
}
