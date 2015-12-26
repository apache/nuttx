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

#include "tms570_selftest.h"

#ifdef CONFIG_TMS570_SELFTEST

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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
 * Input Paramters:
 *   rinfol - The OR of each RAM grouping bit.  See the PBIST_RINFOL*
 *     definitions in chip/tms570_pbist.h
 *
 ****************************************************************************/

void tms570_memtest_start(uint32_t rinfol)
{
#warning Missing Logic
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
#warning Missing Logic
  return 0;
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
  return 0;
}

#endif /* CONFIG_TMS570_SELFTEST */
