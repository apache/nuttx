/****************************************************************************
 * arch/arm/src/tms570/tms570_selftest.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_TMS570_TMS570_SELFTEST_H
#define __ARCH_ARM_SRC_TMS570_TMS570_SELFTEST_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#ifdef CONFIG_TMS570_SELFTEST

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
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

void tms570_memtest_selftest(void);

/****************************************************************************
 * Name: tms570_memtest_start
 *
 * Description:
 *   Start the memory test on the selected set of RAMs.
 *
 * Input Parameters:
 *   rinfol - The OR of each RAM grouping bit.  See the PBIST_RINFOL*
 *     definitions in chip/tms570_pbist.h
 *
 ****************************************************************************/

void tms570_memtest_start(uint32_t rinfol);

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

int tms570_memtest_complete(void);

/****************************************************************************
 * Name: tms570_efc_selftest_start
 *
 * Description:
 *   Run eFuse controller start-up checks and start eFuse controller ECC
 *   self-test.  This includes a check for the eFuse controller error
 *   outputs to  be stuck-at-zero.
 *
 ****************************************************************************/

void tms570_efc_selftest_start(void);

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

int tms570_efc_selftest_complete(void);

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

#define tms570_cpuecc_selftest()

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* CONFIG_TMS570_SELFTEST */
#endif /* __ARCH_ARM_SRC_TMS570_TMS570_SELFTEST_H */
