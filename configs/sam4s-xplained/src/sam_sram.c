/************************************************************************************
 * configs/sam4s-xplained/src/sam_sram.c
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

#include "up_arch.h"
#include "sam4s_periphclks.h"
#include "chip/sam_smc.h"
#include "sam4s-xplained.h"

#ifdef CONFIG_SAM34_EXTSRAM0

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define NPINS (3+8+19+1)

/************************************************************************************
 * Private Data
 ************************************************************************************/

static const gpio_pinset_t g_srampins[NPINS] =
{
  GPIO_SMC_NCS0, GPIO_SMC_NRD, GPIO_SMC_NWE,

  GPIO_SMC_D0,   GPIO_SMC_D1,  GPIO_SMC_D2,  GPIO_SMC_D3,
  GPIO_SMC_D4,   GPIO_SMC_D5,  GPIO_SMC_D6,  GPIO_SMC_D7,

  GPIO_SMC_A0,   GPIO_SMC_A1,  GPIO_SMC_A2,  GPIO_SMC_A3,
  GPIO_SMC_A4,   GPIO_SMC_A5,  GPIO_SMC_A6,  GPIO_SMC_A7,
  GPIO_SMC_A8,   GPIO_SMC_A9,  GPIO_SMC_A10, GPIO_SMC_A11,
  GPIO_SMC_A12,  GPIO_SMC_A13, GPIO_SMC_A14, GPIO_SMC_A15,
  GPIO_SMC_A16,  GPIO_SMC_A17, GPIO_SMC_A18,

  GPIO_EBI_NLB
};

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: sam_sram_initialize
 *
 * Description:
 *   Configure and enable SRAM on board the SAM4S Xplained
 *
 ************************************************************************************/

void sam_sram_initialize(void)
{
  int i;

  /* Configure GPIO pins (leaving SRAM in the disabled state) */

  for (i = 0; i < NPINS; i++)
    {
      sam_configgpio(g_srampins[i]);
    }

  /* Enable PMC clock to the SMC */

  sam_smc_enableclk();

  /* Configure SMC setup timing */

  putreg32(SMCCS_SETUP_NWESETUP(1) | SMCCS_SETUP_NCSWRSETUP(1) |
           SMCCS_SETUP_NRDSETUP(1) | SMCCS_SETUP_NCSRDSETUP(1),
           SAM_SMCCS0_SETUP);

  /* Configure the SMC pulse timing */

  putreg32(SMCCS_PULSE_NWEPULSE(6) | SMCCS_PULSE_NCSWRPULSE(6) |
           SMCCS_PULSE_NRDPULSE(6) | SMCCS_PULSE_NCSRDPULSE(6),
           SAM_SMCCS0_PULSE);

  /* Configure the SMC cycle timing */

  putreg32(SMCCS_CYCLE_NWECYCLE(7) | SMCCS_CYCLE_NRDCYCLE(7),
           SAM_SMCCS0_CYCLE);

  /* Configure the SMC mode */

  putreg32(SMCCS_MODE_READMODE | SMCCS_MODE_WRITEMODE, SAM_SMCCS0_MODE);

  /* Enable SRAM access (active low) */

  sam_gpiowrite(GPIO_EBI_NLB, false);
}

#endif /* CONFIG_SAM34_EXTSRAM0 */
