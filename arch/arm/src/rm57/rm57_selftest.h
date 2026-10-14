/****************************************************************************
 * arch/arm/src/rm57/rm57_selftest.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM_SRC_RM57_RM57_SELFTEST_H
#define __ARCH_ARM_SRC_RM57_RM57_SELFTEST_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The start-up diagnostics, in the order they run.  Each one's outcome is
 * a two-bit field of a 32-bit word, so that the outcome of the tests that
 * run before .bss exists can be carried in a register.
 */

#define RM57_ST_ESM_GROUP3        0  /* No ESM group3 error left by reset */
#define RM57_ST_PBIST_ROMINIT     1  /* Errata PBIST#4 / STC#30 workaround */
#define RM57_ST_PBIST_ROM         2  /* PBIST and STC ROMs, triple read */
#define RM57_ST_STC_SELFCHECK     3  /* STC1 signature compare self-check */
#define RM57_ST_STC_CPU           4  /* STC1 Cortex-R5F LBIST */
#define RM57_ST_CCM_CPU           5  /* CCM-R5F CPU output compare */
#define RM57_ST_CCM_VIM           6  /* CCM-R5F VIM output compare */
#define RM57_ST_CCM_INACTIVITY    7  /* CCM-R5F checker CPU inactivity */
#define RM57_ST_CCM_POWERDOMAIN   8  /* CCM-R5F power domain inactivity */
#define RM57_ST_EFUSE             9  /* eFuse controller */
#define RM57_ST_L2RAM_ECC         10 /* L2RAMW SECDED diagnostics */
#define RM57_ST_L2RAM_RAD         11 /* L2RAMW redundant address decode */
#define RM57_ST_NTESTS            12

#define RM57_ST_NOTRUN            0  /* Not built in, or not reached */
#define RM57_ST_PASS              1
#define RM57_ST_SKIPPED           2  /* Deliberately not run this boot */
#define RM57_ST_FAIL              3

#define RM57_ST_RESULT(w, t)      (((w) >> ((t) << 1)) & 3)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct rm57_selftest_s
{
  uint32_t sysesr;     /* SYSESR as found at reset (reset cause flags) */
  uint32_t results;    /* Two bits per test, see RM57_ST_RESULT() */
  uint32_t esm[6];     /* ESM SR1, SR4, SR7, SR2|SSR2, SR3 after the
                        * diagnostics, and SR4 before the late ones */
  bool     debugger;   /* Halting debug was enabled: lockstep is off */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_RM57_SELFTEST

/****************************************************************************
 * Name: rm57_selftest_early
 *
 * Description:
 *   Run the diagnostics that have to precede C data initialization: the
 *   ESM group3 check, the ROM tests and the CPU LBIST.  The LBIST ends in
 *   a CPU reset, so on a normal boot this function is entered up to three
 *   times and only returns from the last one.
 *
 *   Runs before .bss/.data are valid: it must not touch static data.
 *
 * Input Parameters:
 *   sysesr   - SYSESR as read at reset
 *   cpureset - True if this boot follows a CPU-only reset
 *
 * Returned Value:
 *   The result word so far, to be handed to rm57_selftest_late().
 *
 ****************************************************************************/

uint32_t rm57_selftest_early(uint32_t sysesr, bool cpureset);

/****************************************************************************
 * Name: rm57_selftest_late
 *
 * Description:
 *   Run the remaining diagnostics (CCM-R5F, eFuse, L2RAMW) once .bss is
 *   usable, record every result, and clear the reset cause flags.
 *
 ****************************************************************************/

void rm57_selftest_late(uint32_t sysesr, uint32_t results);

/****************************************************************************
 * Name: rm57_selftest_report
 *
 * Description:
 *   Print the reset cause and one line per diagnostic on the low-level
 *   console.  Call after rm57_lowsetup().
 *
 ****************************************************************************/

void rm57_selftest_report(void);

/****************************************************************************
 * Name: rm57_selftest_results
 *
 * Description:
 *   Return the recorded results of this boot's diagnostics.
 *
 ****************************************************************************/

const struct rm57_selftest_s *rm57_selftest_results(void);

/****************************************************************************
 * Name: rm57_errata_pbist4
 *
 * Description:
 *   Workaround for silicon errata PBIST#4 and STC#30 (revision A): bring
 *   the PBIST ROM and the STC ROMs into a readable state after power-on.
 *   Must run before the first PBIST or STC run.  Harmless on revision B.
 *
 * Input Parameters:
 *   info - Receives a detail word if the workaround fails
 *
 * Returned Value:
 *   Zero on success; otherwise 1 (a ROM pass reported an error), 2 (a ROM
 *   pass did not take the expected time) or 3 (the PBIST clocks could not
 *   be stopped), with the ROM index (0-3) in bits 4-7.
 *
 ****************************************************************************/

#ifdef CONFIG_RM57_SELFTEST_PBIST_ROM
int rm57_errata_pbist4(uint32_t *info);
#endif

#endif /* CONFIG_RM57_SELFTEST */
#endif /* __ARCH_ARM_SRC_RM57_RM57_SELFTEST_H */
