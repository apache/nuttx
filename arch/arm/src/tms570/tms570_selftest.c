/****************************************************************************
 * arch/arm/src/tms570/tms570_selftest.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Most logic in this file was leveraged from TI's Project0 which has a
 * compatible BSD license:
 *
 *   Copyright (c) 2012, Texas Instruments Incorporated
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
 * 3. Neither the name NuttX, Texas Instruments Incorporated, nor the
 *    names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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
#include <stdbool.h>

#include "up_arch.h"

#include "chip/tms570_sys.h"
#include "chip/tms570_pbist.h"
#include "tms570_selftest.h"

#ifdef CONFIG_TMS570_SELFTEST

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pbist_memtest_start
 *
 * Description:
 *   This function performs Memory Built-in Self test using PBIST module.
 *
 * Input Parameters:
 *   rinfol - The OR of each RAM grouping bit.  See the PBIST_RINFOL*
 *     definitions in chip/tms570_pbist.h
 *   algomask - The list of algorithms to be run.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void pbist_memtest_start(uint32_t rinfol, uint32_t algomask)
{
  uint32_t regval;
  volatile int i;

  /* PBIST ROM clock frequency = HCLK frequency /2 */

  regval  = getreg32(TMS570_SYS_MSTGCR);
  regval &= ~SYS_MSTGCR_ROMDIV_MASK;
  regval |= SYS_MSTGCR_ROMDIV_DIV2;
  putreg32(regval, TMS570_SYS_MSTGCR);

  /* Enable PBIST controller */

  putreg32(SYS_MSIENA_RAM, TMS570_SYS_MSIENA);

  /* clear MSTGENA field */

  regval = getreg32(TMS570_SYS_MSTGCR);
  regval &= ~SYS_MSTGCR_MSTGENA_MASK;
  putreg32(regval, TMS570_SYS_MSTGCR);

  /* Enable PBIST self-test */

  regval |= SYS_MSTGCR_MSTGENA_ENABLE;
  putreg32(regval, TMS570_SYS_MSTGCR);

  /* Wait for 32 VBUS clock cycles at least, based on HCLK to VCLK ratio */

  for (i = 0; i < (32 + (32 * 0)); i++);

  /* Enable PBIST clocks and ROM clock */

  regval = (PBIST_PACT_PACT0 | PBIST_PACT_PACT1);
  putreg32(regval, TMS570_PBIST_PACT);

  /* Select all algorithms to be tested */

  putreg32(algomask, TMS570_PBIST_ALGO);

  /* Select RAM groups */

  putreg32(rinfol, TMS570_PBIST_RINFOL);

  /* Select all RAM groups */

  putreg32(0, TMS570_PBIST_RINFOU);

  /* ROM contents will not override RINFOx settings */

  putreg32(0, TMS570_PBIST_OVER);

  /* Algorithm code is loaded from ROM */

  putreg32(PBIST_ROM_BOTH, TMS570_PBIST_ROM);

  /* Start PBIST */

  regval = (PBIST_DLR_DLR2 | PBIST_DLR_DLR4);
  putreg32(PBIST_ROM_BOTH, TMS570_PBIST_DLR);
}

/****************************************************************************
 * Name: pbist_test_complete
 *
 * Description:
 *   Return true if the PBIST test is completed
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   true if the PBIST test is compelte
 *
 ****************************************************************************/

static inline bool pbist_test_complete(void)
{
  return ((getreg32(TMS570_SYS_MSTCGSTAT) & SYS_MSTCGSTAT_MSTDONE) != 0);
}

/****************************************************************************
 * Name: pbist_test_passed
 *
 * Description:
 *   Return true if the PBIST test passed
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   true if the PBIST test passed
 *
 ****************************************************************************/

static inline bool pbist_test_passed(void)
{
  return ((getreg32(TMS570_PBIST_FSRF0) & PBIST_FSRF) == 0 &&
          (getreg32(TMS570_PBIST_FSRF1) & PBIST_FSRF) == 0);
}

/****************************************************************************
 * Name: pbist_stop
 *
 * Description:
 *   This function is called to stop PBIST after test is performed.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void pbist_stop(void)
{
  uint32_t regval;

  /* Disable PBIST clocks and ROM clock */

  putreg32(0, TMS570_PBIST_PACT);

  regval  = getreg32(TMS570_SYS_MSTGCR);
  regval &= ~SYS_MSTGCR_MSTGENA_MASK;
  putreg32(regval, TMS570_SYS_MSTGCR);

  regval |= SYS_MSTGCR_MSTGENA_DISABLE;
  putreg32(regval, TMS570_SYS_MSTGCR);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tms570_memtest_selftest
 *
 * Description:
 *   Run a diagnostic check on the memory self-test controller.
 *
 *   This function chooses a RAM test algorithm and runs it on an on-chip
 *   ROM.  The memory self-test is expected to fail. The function ensures
 *   that the PBIST controller is capable of detecting and indicating a
 *   memory self-test failure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void tms570_memtest_selftest(void)
{
#warning Missing Logic
}

/****************************************************************************
 * Name: tms570_memtest_start
 *
 * Description:
 *   Start the memory test on the selecte set of RAMs.  This test does not
 *   return until the memory test is completed.
 *
 * Input Parameters:
 *   rinfol - The OR of each RAM grouping bit.  See the PBIST_RINFOL*
 *     definitions in chip/tms570_pbist.h
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void tms570_memtest_start(uint32_t rinfol)
{
  pbist_memtest_start(rinfol, PBIST_ALGO_March13N_SP);
}

/****************************************************************************
 * Name: tms570_memtest_complete
 *
 * Description:
 *   Wait for memory self-test to complete and return the result.
 *
 * Returned Value:
 *   Zero (OK) if the test passed; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int tms570_memtest_complete(void)
{
  bool pass;
  /* Wait for the test to complete */

 while (!pbist_test_complete());

 /* Get the test result */

 pass = pbist_test_passed();

 /* Disable PBIST clocks and disable memory self-test mode */

 pbist_stop();

 /* Then return the test result */

 return pass ? OK : ERROR;
}

/****************************************************************************
 * Name: tms570_efc_selftest_start
 *
 * Description:
 *   Run eFuse controller start-up checks and start eFuse controller ECC
 *   self-test.  This includes a check for the eFuse controller error
 *   outputs to  be stuck-at-zero.
 *
 ****************************************************************************/

void tms570_efc_selftest_start(void)
{
#warning Missing Logic
}

/****************************************************************************
 * Name: tms570_efc_selftest_complete
 *
 * Description:
 *   Wait for eFuse controller self-test to complete and return the result.
 *
 * Returned Value:
 *   Zero (OK) if the test passed; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int tms570_efc_selftest_complete(void)
{
#warning Missing Logic
  return OK;
}

/****************************************************************************
 * Name: tms570_cpuecc_selftest
 *
 * Description:
 *   Test the CPU ECC mechanism for RAM accesses.
 *
 *   Cause single-bit and double-bit errors in TCRAM accesses by corrupting
 *   1 or 2 bits in the ECC. Reading from the TCRAM location with a 2-bit
 *   error in the ECC causes a data abort exception. The data abort handler
 *   must include logic written to look for deliberately caused exception and
 *   to return the code execution to the instruction following the one that
 *   caused the abort.
 *
 * Returned Value:
 *   Zero (OK) if the test passed; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

#if 0 /* Needs change to data abort handler */
int tms570_cpuecc_selftest(void)
{
#warning Missing Logic
  return OK;
}
#endif

#endif /* CONFIG_TMS570_SELFTEST */
