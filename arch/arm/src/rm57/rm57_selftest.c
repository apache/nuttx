/****************************************************************************
 * arch/arm/src/rm57/rm57_selftest.c
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

/* Start-up diagnostics for the RM57L843's safety mechanisms: CCM-R5F
 * (lockstep compare), STC1 (CPU logic BIST), PBIST (ROM tests), the eFuse
 * controller and the L2RAMW ECC logic.
 *
 * Written from SPNU562A (chapters 2, 8, 9, 10, 13, 16, 37), the silicon
 * errata SPNZ214C/SPNZ233B and the datasheet SPNS215C.  The sequence in
 * which TI's HALCoGen and SafeTI Diagnostic Library start-up code runs
 * these tests, and the flags each is expected to raise, were used as a
 * behavioural cross-check; no code is taken from the SafeTI library.
 *
 * Everything here runs with IRQ and FIQ masked, before the OS starts.
 * Several tests raise an ESM group2 or group3 error on purpose, which
 * drives the nERROR pin low for one low-time-counter period; each test
 * clears what it raised and releases the pin before returning.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <sys/param.h>

#include <arch/board/board.h>

#include "arm_internal.h"

#include "hardware/rm57_sys.h"
#include "hardware/rm57_esm.h"
#include "hardware/rm57_vim.h"
#include "hardware/rm57_ccm.h"
#include "hardware/rm57_stc.h"
#include "hardware/rm57_pbist.h"
#include "hardware/rm57_efc.h"
#include "hardware/rm57_l2ramw.h"
#include "rm57_lowputc.h"
#include "rm57_selftest.h"

#ifdef CONFIG_RM57_SELFTEST

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Upper bound, in polling loop iterations, on every wait for hardware.
 * The longest legitimate wait is the nERROR low time (LTCPR, a few
 * milliseconds); this is orders of magnitude beyond that and only exists
 * so that a dead peripheral becomes a reported failure instead of a hang.
 */

#define ST_TIMEOUT                4000000

/* DBGDSCR.HDBGen: set by a debugger that has enabled halting debug.  The
 * CCM-R5F switches all its diagnostics off for as long as that lasts
 * (SPNU562A 13.2.5), and a CPU LBIST run under a debugger does not
 * complete, so both are skipped rather than failed.
 */

#define DBGDSCR_HDBGEN            (1 << 14)

/* STCCLK divider: the smallest one that keeps GCLK1 / (n + 1) at or below
 * the 110 MHz limit.
 */

#define ST_STCCLKDIV \
  ((BOARD_PLL_FREQUENCY + STC_MAX_STCCLK - 1) / STC_MAX_STCCLK - 1)

/* DIAG_ECC stimulus for the L2RAMW SECDED tests.  Against an all-zero
 * data vector the correct ECC is 0x0c; these two values differ from it by
 * the syndrome of a single data bit error and by two ECC bits.
 */

#define ST_L2RAM_ECC_1BIT         (L2RAMW_DIAGECC_ZERO ^ 0xce)
#define ST_L2RAM_ECC_2BIT         (L2RAMW_DIAGECC_ZERO ^ 0x03)

/* What a completed LBIST sequence implies about the ROM tests before it */

#ifdef CONFIG_RM57_SELFTEST_PBIST_ROM
#  define ST_ROM_RESULT           RM57_ST_PASS
#else
#  define ST_ROM_RESULT           RM57_ST_NOTRUN
#endif

#define ST_SET(w, t, r)           (((w) & ~(3 << ((t) << 1))) | \
                                   ((uint32_t)(r) << ((t) << 1)))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ccm_diag_s
{
  uintptr_t sr;        /* Status register */
  uintptr_t keyr;      /* Key register */
  uint8_t   channel;   /* ESM group2 channel raised by error forcing */
  uint8_t   test;      /* RM57_ST_CCM_* */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct rm57_selftest_s g_selftest;

static const struct ccm_diag_s g_ccm_diag[] =
{
  { RM57_CCM_SR1, RM57_CCM_KEYR1, ESM_G2_CCM_CPU, RM57_ST_CCM_CPU },
  { RM57_CCM_SR2, RM57_CCM_KEYR2, ESM_G2_CCM_VIM, RM57_ST_CCM_VIM },
  { RM57_CCM_SR3, RM57_CCM_KEYR3, ESM_G2_CCM_INACTIVITY,
    RM57_ST_CCM_INACTIVITY },
  { RM57_CCM_SR4, RM57_CCM_KEYR4, ESM_G2_CCM_PD, RM57_ST_CCM_POWERDOMAIN }
};

static const char * const g_testname[RM57_ST_NTESTS] =
{
  "ESM group3",
  "PBIST ROM init",
  "PBIST ROM",
  "STC self-check",
  "STC CPU LBIST",
  "CCM CPU compare",
  "CCM VIM compare",
  "CCM inactivity",
  "CCM power domain",
  "eFuse",
  "L2RAM ECC",
  "L2RAM addr decode"
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: st_puts / st_puthex
 *
 * Description:
 *   Polled console output.  Usable before the OS and before .bss.
 *
 ****************************************************************************/

static void st_puts(const char *str)
{
  while (*str != '\0')
    {
      if (*str == '\n')
        {
          arm_lowputc('\r');
        }

      arm_lowputc(*str++);
    }
}

static void st_puthex(uint32_t value)
{
  int shift;

  for (shift = 28; shift >= 0; shift -= 4)
    {
      arm_lowputc("0123456789abcdef"[(value >> shift) & 15]);
    }
}

/****************************************************************************
 * Name: st_debugger
 ****************************************************************************/

static bool st_debugger(uint32_t sysesr)
{
  uint32_t dscr;

  __asm__ __volatile__("mrc p14, 0, %0, c0, c1, 0" : "=r"(dscr));

  return (dscr & DBGDSCR_HDBGEN) != 0 || (sysesr & SYS_SYSESR_DBGRST) != 0;
}

/****************************************************************************
 * Name: st_wait
 *
 * Description:
 *   Poll until (reg & mask) == value.  Returns false on timeout.
 *
 ****************************************************************************/

static bool st_wait(uintptr_t regaddr, uint32_t mask, uint32_t value)
{
  uint32_t count;

  for (count = 0; count < ST_TIMEOUT; count++)
    {
      if ((getreg32(regaddr) & mask) == value)
        {
          return true;
        }
    }

  return false;
}

/****************************************************************************
 * Name: st_release_nerror
 *
 * Description:
 *   Release the nERROR pin and wait until it has actually gone inactive.
 *
 *   The pin only releases once the low time counter has expired.  Errata
 *   DEVICE#60: if nERROR was active across a system reset the ESM state
 *   machine no longer knows about it and the 0x5 key does nothing; the
 *   pin then has to be taken through error forcing mode.
 *
 ****************************************************************************/

static bool st_release_nerror(void)
{
  putreg32(ESM_EKR_RELEASE, RM57_ESM_EKR);
  if (st_wait(RM57_ESM_EPSR, ESM_EPSR_EPSF, ESM_EPSR_EPSF))
    {
      return true;
    }

  putreg32(ESM_EKR_FORCE, RM57_ESM_EKR);
  putreg32(ESM_EKR_NORMAL, RM57_ESM_EKR);
  return st_wait(RM57_ESM_EPSR, ESM_EPSR_EPSF, ESM_EPSR_EPSF);
}

/****************************************************************************
 * Name: st_fail
 *
 * Description:
 *   A diagnostic failed.  Report it and, unless the build asks to carry
 *   on, drive nERROR low and stop: the device cannot be trusted.
 *
 ****************************************************************************/

static int st_fail(int test, int step, uint32_t info0, uint32_t info1)
{
  rm57_lowsetup();

  st_puts("\nSELFTEST FAIL: ");
  st_puts(g_testname[test]);
  st_puts(" step ");
  st_puthex(step);
  st_puts(" info ");
  st_puthex(info0);
  arm_lowputc(' ');
  st_puthex(info1);
  st_puts("\n");

#ifdef CONFIG_RM57_SELFTEST_HALT_ON_FAIL
  putreg32(ESM_EKR_FORCE, RM57_ESM_EKR);
  for (; ; )
    {
    }
#endif

  return RM57_ST_FAIL;
}

/****************************************************************************
 * Name: st_result
 *
 * Description:
 *   Fold one test's outcome into the result word.  This is also where
 *   CONFIG_RM57_SELFTEST_FORCE_FAIL substitutes a failure for a pass, to
 *   exercise the failure path on a healthy device.
 *
 ****************************************************************************/

static uint32_t st_result(uint32_t results, int test, int result)
{
#if CONFIG_RM57_SELFTEST_FORCE_FAIL >= 0
  if (test == CONFIG_RM57_SELFTEST_FORCE_FAIL && result == RM57_ST_PASS)
    {
      result = st_fail(test, 0xff, 0, 0);
    }
#endif

  return ST_SET(results, test, result);
}

/****************************************************************************
 * Name: st_esm_group3
 *
 * Description:
 *   A group3 flag left by the reset sequence means the eFuse autoload or
 *   an implicit flash OTP read went wrong.  TI's guidance is that the
 *   device is then not reliable.
 *
 ****************************************************************************/

static int st_esm_group3(void)
{
  uint32_t sr3 = getreg32(RM57_ESM_SR3);

  if (sr3 != 0)
    {
      putreg32(sr3, RM57_ESM_SR3);
      return st_fail(RM57_ST_ESM_GROUP3, 1, sr3, getreg32(RM57_ESM_EPSR));
    }

  return RM57_ST_PASS;
}

/****************************************************************************
 * Name: st_pbist_start / st_pbist_finish
 *
 * Description:
 *   Run one PBIST pass over 'groups' with 'algo' (SPNU562A 9.3.1).
 *
 ****************************************************************************/

#ifdef CONFIG_RM57_SELFTEST_PBIST_ROM
static void st_pbist_start(uint64_t groups, uint32_t algo)
{
  volatile int i;
  uint32_t regval;

  /* ROM clock prescaler, then enable the controller.  Writing the enable
   * key also resets the PBIST state machine, which takes up to 64 VBUS
   * clock cycles at this prescaler.
   */

  regval  = getreg32(RM57_SYS_MSTGCR);
  regval &= ~(SYS_MSTGCR_ROMDIV_MASK | SYS_MSTGCR_MSTGENA_MASK);
  regval |= SYS_MSTGCR_ROMDIV_DIV4 | SYS_MSTGCR_MSTGENA_DISABLE;
  putreg32(regval, RM57_SYS_MSTGCR);

  putreg32(SYS_MSINENA_PBIST, RM57_SYS_MSINENA);

  regval &= ~SYS_MSTGCR_MSTGENA_MASK;
  regval |= SYS_MSTGCR_MSTGENA_ENABLE;
  putreg32(regval, RM57_SYS_MSTGCR);

  for (i = 0; i < 256; i++)
    {
    }

  putreg32(PBIST_PACT_ENABLE, RM57_PBIST_PACT);
  putreg32(algo, RM57_PBIST_ALGO);
  putreg32((uint32_t)groups, RM57_PBIST_RINFOL);
  putreg32((uint32_t)(groups >> 32), RM57_PBIST_RINFOU);
  putreg32(0, RM57_PBIST_OVER);
  putreg32(PBIST_ROM_BOTH, RM57_PBIST_ROM);
  putreg32(PBIST_DLR_START, RM57_PBIST_DLR);
}

static bool st_pbist_finish(uint32_t *info)
{
  bool done;
  uint32_t regval;

  done = st_wait(RM57_SYS_MSTCGSTAT, SYS_MSTCGSTAT_MSTDONE,
                 SYS_MSTCGSTAT_MSTDONE);

  info[0] = getreg32(RM57_PBIST_FSRF0) | getreg32(RM57_PBIST_FSRF1);
  info[1] = getreg32(RM57_PBIST_RAMT);

  putreg32(0, RM57_PBIST_PACT);

  regval  = getreg32(RM57_SYS_MSTGCR);
  regval &= ~SYS_MSTGCR_MSTGENA_MASK;
  regval |= SYS_MSTGCR_MSTGENA_DISABLE;
  putreg32(regval, RM57_SYS_MSTGCR);

  return done && (info[0] & PBIST_FSRF) == 0;
}

/****************************************************************************
 * Name: st_pbist_rominit
 *
 * Description:
 *   Errata PBIST#4 and STC#30: after power-on the PBIST and STC ROMs may
 *   not read correctly until they have been initialized.  Without this a
 *   ROM test can pass without having run and the CPU LBIST can hang.
 *
 ****************************************************************************/

static int st_pbist_rominit(void)
{
  uint32_t info = 0;
  int code;

  code = rm57_errata_pbist4(&info);
  if (code != 0)
    {
      return st_fail(RM57_ST_PBIST_ROMINIT, code, info, 0);
    }

  return RM57_ST_PASS;
}

/****************************************************************************
 * Name: st_pbist_rom
 *
 * Description:
 *   Check the PBIST ROM and the three STC ROMs with both triple-read
 *   algorithms, one ROM and one algorithm at a time so that a failure
 *   identifies its ROM.  The CPU LBIST depends on the STC1 ROMs, so this
 *   runs first.
 *
 ****************************************************************************/

static int st_pbist_rom(void)
{
  uint32_t info[2];
  int group;
  int algo;

#ifdef CONFIG_RM57_SELFTEST_INJECT_PBIST_ALGO
  /* Fault injection: a RAM algorithm cannot pass on a ROM (SPNU562A
   * 9.1.3.3), so the controller has to report a failure.
   */

  st_pbist_start(PBIST_GROUP_PBIST_ROM, PBIST_ALGO_MARCH13N_SP);
  if (!st_pbist_finish(info))
    {
      return st_fail(RM57_ST_PBIST_ROM, 0xf1, info[0], info[1]);
    }

  st_puts("SELFTEST: PBIST fault injection was NOT detected\n");
  return st_fail(RM57_ST_PBIST_ROM, 0xf0, info[0], info[1]);
#endif

  for (group = 1; group <= 4; group++)
    {
      for (algo = 0; algo < 2; algo++)
        {
          st_pbist_start(PBIST_GROUP(group), 1 << algo);
          if (!st_pbist_finish(info))
            {
              return st_fail(RM57_ST_PBIST_ROM, (group << 4) | algo,
                             info[0], info[1]);
            }
        }
    }

  return RM57_ST_PASS;
}
#endif /* CONFIG_RM57_SELFTEST_PBIST_ROM */

/****************************************************************************
 * Name: st_stc_clear
 *
 * Description:
 *   Clear what a CPU LBIST run leaves behind: its status flags, the ESM
 *   group1 "CPU self-test" flag, and - errata DEVICE#48 - the interconnect
 *   global error that a CPU-only reset raises spuriously while GCLK1 runs
 *   faster than the HCLK limit, as it does on this board.
 *
 ****************************************************************************/

#ifdef CONFIG_RM57_SELFTEST_STC
static void st_stc_clear(void)
{
  putreg32(STC_GSTAT_TEST_DONE | STC_GSTAT_TEST_FAIL, RM57_STC1_GSTAT);
  putreg32(STC_FSTAT_CORE1_FAIL | STC_FSTAT_CORE2_FAIL | STC_FSTAT_TO_ERR,
           RM57_STC1_FSTAT);
  putreg32(ESM_CHANBIT(ESM_G1_CPU_SELFTEST), RM57_ESM_SR1(0));
  putreg32(ESM_CHANBIT(ESM_G1_INTERCONNECT), RM57_ESM_SR4(0));
}

/****************************************************************************
 * Name: st_stc_start
 *
 * Description:
 *   Start a CPU LBIST run on both cores in parallel and idle the CPU so
 *   that it can begin (SPNU562A 10.9.1).  Returns only if the
 *   preconditions are not met; otherwise the run ends in a CPU reset.
 *
 ****************************************************************************/

static int st_stc_start(int test, bool selfcheck, uint32_t intervals)
{
  volatile int i;
  uint32_t ssr2;

  /* Nothing may be pending that the run could be confused with, and no
   * interrupt may wake the CPU out of the idle state the STC waits for.
   */

  ssr2 = getreg32(RM57_ESM_SSR2) | getreg32(RM57_ESM_SR2);
  if (ssr2 != 0 || !st_release_nerror())
    {
      return st_fail(test, 1, ssr2, getreg32(RM57_ESM_EPSR));
    }

  putreg32(0xffffffff, RM57_VIM_REQMASKCLR0);
  putreg32(0xffffffff, RM57_VIM_REQMASKCLR1);
  putreg32(0xffffffff, RM57_VIM_REQMASKCLR2);
  putreg32(0xffffffff, RM57_VIM_REQMASKCLR3);

  /* The reset that ends the run is recognized by this flag */

  putreg32(SYS_SYSESR_CPURST_ALL, RM57_SYS_SYSESR);

  putreg32(SYS2_STCCLKDIV_CLKDIV(ST_STCCLKDIV), RM57_SYS2_STCCLKDIV);

  /* Always restart from interval 0 (errata DEVICE#47: interval 1 cannot
   * be run on its own).  The timeout preload reverts to its reset value
   * after every run (errata STC#26), so it is written each time.
   */

  putreg32(STC_GCR0_INTCOUNT(intervals) | STC_GCR0_RS_CNT_RESTART,
           RM57_STC1_GCR0);
  putreg32(selfcheck ? (STC_SCSCR_KEY_ENABLE | STC_SCSCR_FAULT_INS) :
           STC_SCSCR_KEY_DISABLE, RM57_STC1_SCSCR);
#ifdef CONFIG_RM57_SELFTEST_INJECT_STC_TIMEOUT
  /* Fault injection: a timeout of one VBUS clock cycle cannot be met, so
   * the real run has to end in a timeout error.
   */

  putreg32(selfcheck ? STC_TPR_MAX : 1, RM57_STC1_TPR);
#else
  putreg32(STC_TPR_MAX, RM57_STC1_TPR);
#endif

  /* Core select has to be written before, and separately from, the key */

  putreg32(STC_GCR1_CORE_SEL_BOTH | STC_GCR1_ENA_DISABLE, RM57_STC1_GCR1);
  putreg32(STC_GCR1_CORE_SEL_BOTH | STC_GCR1_ENA_ENABLE, RM57_STC1_GCR1);

  for (i = 0; i < 64; i++)
    {
    }

  /* The run starts once the CPU is idle.  Keep idling: only the CPU
   * reset that ends the run leaves this loop.
   */

  for (; ; )
    {
      __asm__ __volatile__("wfi\n\tnop\n\tnop\n\tnop\n\tnop");
    }

  return RM57_ST_FAIL;
}

/****************************************************************************
 * Name: st_stc_resume
 *
 * Description:
 *   Called after a CPU reset: work out which STC run, if any, caused it
 *   and take the next step.
 *
 *   Step 1 is the signature compare self-check (SPNU562A 10.10): one
 *   interval with a stuck-at fault inserted, which must fail on both
 *   cores.  It is recognized by the self-check key, which survives the
 *   CPU reset.  Step 2 is the real run, which must complete without
 *   failure.
 *
 ****************************************************************************/

static uint32_t st_stc_resume(uint32_t results)
{
  uint32_t gstat = getreg32(RM57_STC1_GSTAT);
  uint32_t fstat = getreg32(RM57_STC1_FSTAT);
  uint32_t scscr = getreg32(RM57_STC1_SCSCR);
  int result;

  if ((scscr & STC_SCSCR_KEY_MASK) == STC_SCSCR_KEY_ENABLE)
    {
      putreg32(STC_SCSCR_KEY_DISABLE, RM57_STC1_SCSCR);

      if ((gstat & (STC_GSTAT_TEST_DONE | STC_GSTAT_TEST_FAIL)) !=
          (STC_GSTAT_TEST_DONE | STC_GSTAT_TEST_FAIL) ||
          (fstat & (STC_FSTAT_CORE1_FAIL | STC_FSTAT_CORE2_FAIL)) == 0 ||
          (fstat & (STC_FSTAT_TO_ERR | STC_FSTAT_FSEG_ID_MASK)) != 0)
        {
          result = st_fail(RM57_ST_STC_SELFCHECK, 2, gstat, fstat);
          st_stc_clear();
          results = st_result(results, RM57_ST_STC_SELFCHECK, result);
          return st_result(results, RM57_ST_STC_CPU, RM57_ST_SKIPPED);
        }

      st_stc_clear();

      /* The self-check passed; that is not recorded here because the
       * real run is about to reset the CPU again.  Reaching the branch
       * below is what records it.
       */

      result = st_stc_start(RM57_ST_STC_CPU, false,
                            CONFIG_RM57_SELFTEST_STC_INTERVALS);
      results = st_result(results, RM57_ST_STC_SELFCHECK, RM57_ST_PASS);
      return st_result(results, RM57_ST_STC_CPU, result);
    }

  if ((gstat & STC_GSTAT_TEST_DONE) != 0)
    {
      result = RM57_ST_PASS;
      if ((gstat & STC_GSTAT_TEST_FAIL) != 0)
        {
          result = st_fail(RM57_ST_STC_CPU, 2, gstat, fstat);
        }

      st_stc_clear();

      /* Getting here means everything before the self-check passed too:
       * a failure there would have stopped the sequence.
       */

      results = st_result(results, RM57_ST_PBIST_ROMINIT,
                          ST_ROM_RESULT);
      results = st_result(results, RM57_ST_PBIST_ROM,
                          ST_ROM_RESULT);
      results = st_result(results, RM57_ST_STC_SELFCHECK, RM57_ST_PASS);
      return st_result(results, RM57_ST_STC_CPU, result);
    }

  /* A CPU reset that the STC did not cause (CPURSTCR) */

  results = st_result(results, RM57_ST_STC_SELFCHECK, RM57_ST_SKIPPED);
  return st_result(results, RM57_ST_STC_CPU, RM57_ST_SKIPPED);
}
#endif /* CONFIG_RM57_SELFTEST_STC */

/****************************************************************************
 * Name: st_ccm_clear
 ****************************************************************************/

#ifdef CONFIG_RM57_SELFTEST_CCM
static void st_ccm_clear(const struct ccm_diag_s *diag)
{
  putreg32(CCM_SR_CMPE, diag->sr);
  putreg32(ESM_CHANBIT(diag->channel), RM57_ESM_SR2);
  putreg32(ESM_CHANBIT(diag->channel), RM57_ESM_SSR2);
  putreg32(ESM_CHANBIT(ESM_G1_CCM_SELFTEST), RM57_ESM_SR1(0));
  putreg32(ESM_CHANBIT(ESM_G1_CCM_STATUS), RM57_ESM_SR7(0));

  /* ESM high-level request, VIM channel 0 */

  putreg32(1 << 0, RM57_VIM_INTREQ0);
}

/****************************************************************************
 * Name: st_ccm
 *
 * Description:
 *   Exercise one of the four CCM-R5F diagnostics in its three test modes
 *   (SPNU562A 13.2.1.2 - 13.2.1.4):
 *
 *   1. Self-test: the compare logic checks itself with match and
 *      mismatch patterns.  Must complete without a self-test error.
 *   2. Self-test error forcing: must raise the "CCM-R5F self-test" ESM
 *      flag, proving that path to the ESM.
 *   3. Error forcing: must raise this diagnostic's compare error in ESM
 *      group2, proving that path too (and pulsing nERROR).
 *
 *   The lockstep compare is suspended while a test mode is selected.
 *
 ****************************************************************************/

static int st_ccm(const struct ccm_diag_s *diag)
{
  uint32_t selftest = ESM_CHANBIT(ESM_G1_CCM_SELFTEST);
  uint32_t status;
  uint32_t esm;
  bool done;

  if ((getreg32(diag->keyr) & CCM_KEYR_MKEY_MASK) != CCM_KEYR_LOCKSTEP ||
      !st_release_nerror())
    {
      return st_fail(diag->test, 1, getreg32(diag->keyr),
                     getreg32(RM57_ESM_EPSR));
    }

  st_ccm_clear(diag);

  /* 1. Self-test.  The verdict is taken from the status register, not
   * from the ESM (errata CCMR5#1), and the mode is only changed once the
   * test has run to completion (errata CCMR5#2).
   */

  putreg32(CCM_KEYR_SELFTEST, diag->keyr);
  done   = st_wait(diag->sr, CCM_SR_STC, CCM_SR_STC);
  status = getreg32(diag->sr);
  putreg32(CCM_KEYR_LOCKSTEP, diag->keyr);

  esm = getreg32(RM57_ESM_SR1(0));
  if (!done || (status & CCM_SR_STE) != 0 || (esm & selftest) != 0)
    {
      st_ccm_clear(diag);
      return st_fail(diag->test, 2, status, esm);
    }

  /* 2. Self-test error forcing.  Lasts one cycle, after which the key
   * reads back as lockstep again.
   */

  putreg32(CCM_KEYR_STERRFORCE, diag->keyr);
  done = st_wait(diag->keyr, CCM_KEYR_MKEY_MASK, CCM_KEYR_LOCKSTEP);

  esm = getreg32(RM57_ESM_SR1(0));
  if (!done || (esm & selftest) == 0)
    {
      st_ccm_clear(diag);
      return st_fail(diag->test, 3, getreg32(diag->keyr), esm);
    }

  putreg32(selftest, RM57_ESM_SR1(0));

  /* 3. Error forcing */

  putreg32(CCM_KEYR_ERRFORCE, diag->keyr);
  done = st_wait(diag->keyr, CCM_KEYR_MKEY_MASK, CCM_KEYR_LOCKSTEP);

  esm = getreg32(RM57_ESM_SR2) | getreg32(RM57_ESM_SSR2);
  st_ccm_clear(diag);

  if (!done || (esm & ESM_CHANBIT(diag->channel)) == 0)
    {
      st_release_nerror();
      return st_fail(diag->test, 4, getreg32(diag->keyr), esm);
    }

  if (!st_release_nerror())
    {
      return st_fail(diag->test, 5, getreg32(RM57_ESM_EPSR),
                     getreg32(RM57_ESM_SR2));
    }

  return RM57_ST_PASS;
}
#endif /* CONFIG_RM57_SELFTEST_CCM */

/****************************************************************************
 * Name: st_efuse_stuckzero / st_efuse_ecc / st_efuse
 *
 * Description:
 *   Check the eFuse controller as laid out in SPNU562A Figure 37-1:
 *
 *   - No error from the autoload: prove with the stuck-at-zero test that
 *     the controller could have signalled one, then run the ECC self-test.
 *   - A corrected single-bit error (code 0x15): run the ECC self-test.
 *     Operation is still valid, so this passes, but it is reported.
 *   - Anything else: the autoload result cannot be relied on.
 *
 ****************************************************************************/

#ifdef CONFIG_RM57_SELFTEST_EFUSE
static bool st_efuse_stuckzero(uint32_t *info)
{
#ifdef CONFIG_RM57_SELFTEST_EFUSE_NO_NERROR
  const uint32_t bound = EFC_BOUND_STUCKZERO_B;
  const uint32_t pins  = EFC_PINS_SELFTEST_ERR | EFC_PINS_SINGLEBIT_ERR |
                         EFC_PINS_INSTR_ERR;
  const uint32_t sr3   = 0;
#else
  const uint32_t bound = EFC_BOUND_STUCKZERO_A;
  const uint32_t pins  = EFC_PINS_SELFTEST_ERR | EFC_PINS_SINGLEBIT_ERR |
                         EFC_PINS_INSTR_ERR | EFC_PINS_AUTOLOAD_ERR;
  const uint32_t sr3   = ESM_CHANBIT(ESM_G3_EFUSE_AUTOLOAD);
#endif
  const uint32_t sr4   = ESM_CHANBIT(ESM_G1_EFUSE_SELFTEST);
  bool ok;

  /* Drive the error outputs from the boundary register, check that they
   * arrive, then hand the outputs back to the controller.  Leaving the
   * output enables set would mask every real eFuse error.
   */

  putreg32(bound, RM57_EFC_BOUND);
  info[0] = getreg32(RM57_EFC_PINS);
  putreg32(EFC_BOUND_STUCKZERO_CLEAR, RM57_EFC_BOUND);
  putreg32(0, RM57_EFC_BOUND);

  info[1] = getreg32(RM57_ESM_SR4(0));
  ok = (info[0] & pins) == pins && (info[1] & sr4) == sr4 &&
       (getreg32(RM57_ESM_SR3) & sr3) == sr3;

  putreg32(ESM_CHANBIT(ESM_G1_EFUSE_ERROR) |
           ESM_CHANBIT(ESM_G1_EFUSE_SELFTEST), RM57_ESM_SR4(0));
  putreg32(ESM_CHANBIT(ESM_G3_EFUSE_AUTOLOAD), RM57_ESM_SR3);

  return st_release_nerror() && ok;
}

static bool st_efuse_ecc(uint32_t code, uint32_t *info)
{
  const uint32_t sr4 = ESM_CHANBIT(ESM_G1_EFUSE_ERROR) |
                       ESM_CHANBIT(ESM_G1_EFUSE_SELFTEST);
  bool done;

  putreg32(sr4, RM57_ESM_SR4(0));

  putreg32(EFC_STCY_VALUE, RM57_EFC_STCY);
#ifdef CONFIG_RM57_SELFTEST_INJECT_EFUSE_SIGNATURE
  /* Fault injection: with the wrong signature the self-test must fail */

  putreg32(~EFC_STSIG_VALUE, RM57_EFC_STSIG);
#else
  putreg32(EFC_STSIG_VALUE, RM57_EFC_STSIG);
#endif
  putreg32(EFC_BOUND_SELFTEST_START, RM57_EFC_BOUND);

  done = st_wait(RM57_EFC_PINS, EFC_PINS_SELFTEST_DONE,
                 EFC_PINS_SELFTEST_DONE);

  info[0] = getreg32(RM57_EFC_PINS);
  info[1] = getreg32(RM57_EFC_ERRSTAT);

  /* The error code must be clear, or still show the corrected autoload
   * error that was there before the self-test.
   */

  return done && (info[0] & EFC_PINS_SELFTEST_ERR) == 0 &&
         ((info[1] & EFC_ERRSTAT_CODE_MASK) == EFC_ERRSTAT_CODE_NONE ||
          (info[1] & EFC_ERRSTAT_CODE_MASK) == code) &&
         (getreg32(RM57_ESM_SR4(0)) & sr4) == 0;
}

static int st_efuse(void)
{
  uint32_t info[2];
  uint32_t code;

  code = getreg32(RM57_EFC_ERRSTAT) & EFC_ERRSTAT_CODE_MASK;
  if (code == EFC_ERRSTAT_CODE_NONE)
    {
      if (!st_efuse_stuckzero(info))
        {
          return st_fail(RM57_ST_EFUSE, 1, info[0], info[1]);
        }
    }
  else if (code == EFC_ERRSTAT_CODE_SINGLEBIT)
    {
      /* The corrected error itself set group1 channel 40 */

      putreg32(ESM_CHANBIT(ESM_G1_EFUSE_ERROR), RM57_ESM_SR4(0));
    }
  else
    {
      return st_fail(RM57_ST_EFUSE, 2, code, getreg32(RM57_EFC_PINS));
    }

  if (!st_efuse_ecc(code, info))
    {
      return st_fail(RM57_ST_EFUSE, 3, info[0], info[1]);
    }

  if (code == EFC_ERRSTAT_CODE_SINGLEBIT)
    {
      st_puts("SELFTEST: eFuse autoload corrected a single-bit error\n");
    }

  return RM57_ST_PASS;
}
#endif /* CONFIG_RM57_SELFTEST_EFUSE */

/****************************************************************************
 * Name: st_l2ram_trigger / st_l2ram_ecc / st_l2ram_rad
 *
 * Description:
 *   Exercise the L2RAMW's SECDED blocks and its redundant address decode
 *   comparator with the module's built-in stimulus registers (SPNU562A
 *   8.2.6).  No RAM location is modified.
 *
 *   The functional address decode comparison is suspended while test
 *   mode is enabled, so test mode is always switched off again.
 *
 ****************************************************************************/

#ifdef CONFIG_RM57_SELFTEST_L2RAM
static bool st_l2ram_trigger(uint32_t mode, uint32_t ecc)
{
  bool done;

  /* A trigger is only accepted with every diagnostic flag clear */

  putreg32(L2RAMW_ERR_DIAG_ALL, RM57_L2RAMW_RAMERRSTATUS);

  putreg32(0, RM57_L2RAMW_DIAGDATAH);
  putreg32(0, RM57_L2RAMW_DIAGDATAL);
  putreg32(ecc, RM57_L2RAMW_DIAGECC);

  putreg32(mode | L2RAMW_RAMTEST_ENABLE, RM57_L2RAMW_RAMTEST);
  putreg32(mode | L2RAMW_RAMTEST_ENABLE | L2RAMW_RAMTEST_TRIGGER,
           RM57_L2RAMW_RAMTEST);

  done = st_wait(RM57_L2RAMW_RAMTEST, L2RAMW_RAMTEST_TRIGGER, 0);

  putreg32(L2RAMW_RAMTEST_DISABLE, RM57_L2RAMW_RAMTEST);
  return done;
}

static int st_l2ram_ecc(void)
{
  const uint32_t typeb = ESM_CHANBIT(ESM_G2_L2RAMW_TYPEB);
  uint32_t expect;
  uint32_t status;
  uint32_t esm;
  bool done;
  int step;

  if ((getreg32(RM57_L2RAMW_RAMCTRL) & L2RAMW_RAMCTRL_ECC_DETECT_MASK) ==
      L2RAMW_RAMCTRL_ECC_DETECT_DIS || !st_release_nerror())
    {
      return st_fail(RM57_ST_L2RAM_ECC, 1, getreg32(RM57_L2RAMW_RAMCTRL),
                     getreg32(RM57_ESM_EPSR));
    }

  /* Step 2: a two-bit error must be flagged as uncorrectable by both the
   * read and the write SECDED.  Step 3: a one-bit error must be flagged
   * as a single-bit error by both.  Either reaches the ESM as "L2RAMW
   * uncorrectable error type B".
   */

  for (step = 2; step <= 3; step++)
    {
      expect = (step == 2) ? (L2RAMW_ERR_DRDE | L2RAMW_ERR_DWDE) :
                             (L2RAMW_ERR_DRSE | L2RAMW_ERR_DWSE);

      done   = st_l2ram_trigger(L2RAMW_RAMTEST_MODE_EQ,
                                (step == 2) ? ST_L2RAM_ECC_2BIT :
                                              ST_L2RAM_ECC_1BIT);
      status = getreg32(RM57_L2RAMW_RAMERRSTATUS);
      esm    = getreg32(RM57_ESM_SR2) | getreg32(RM57_ESM_SSR2);

      putreg32(L2RAMW_ERR_DIAG_ALL, RM57_L2RAMW_RAMERRSTATUS);
      putreg32(typeb, RM57_ESM_SR2);
      putreg32(typeb, RM57_ESM_SSR2);
      putreg32(1 << 0, RM57_VIM_INTREQ0);

      if (!done || (status & expect) != expect || (esm & typeb) == 0 ||
          !st_release_nerror())
        {
          st_release_nerror();
          return st_fail(RM57_ST_L2RAM_ECC, step, status, esm);
        }
    }

  return RM57_ST_PASS;
}

static int st_l2ram_rad(void)
{
  const uint32_t errors = L2RAMW_ERR_ADDE | L2RAMW_ERR_ADE;
  uint32_t status;
  bool done;
  int step;

  /* With a clean data/ECC pair, neither feeding the comparator the same
   * vector twice (equality mode) nor feeding it the vector and its
   * inverse (inequality mode) may report an address decode error.
   */

  for (step = 1; step <= 2; step++)
    {
      done   = st_l2ram_trigger((step == 1) ? L2RAMW_RAMTEST_MODE_EQ :
                                              L2RAMW_RAMTEST_MODE_INEQ,
                                L2RAMW_DIAGECC_ZERO);
      status = getreg32(RM57_L2RAMW_RAMERRSTATUS);

      if (!done || (status & errors) != 0)
        {
          putreg32(L2RAMW_ERR_DIAG_ALL, RM57_L2RAMW_RAMERRSTATUS);
          putreg32(ESM_CHANBIT(ESM_G2_L2RAMW_TYPEB), RM57_ESM_SR2);
          putreg32(ESM_CHANBIT(ESM_G2_L2RAMW_TYPEB), RM57_ESM_SSR2);
          st_release_nerror();
          return st_fail(RM57_ST_L2RAM_RAD, step, status,
                         getreg32(RM57_ESM_SR2));
        }
    }

  return RM57_ST_PASS;
}
#endif /* CONFIG_RM57_SELFTEST_L2RAM */

/****************************************************************************
 * Name: st_inject_ccm
 *
 * Description:
 *   Fault injection: make the two CPUs really disagree.  Inverting one of
 *   CPU1's compared output signals ahead of the comparator (SPNU562A
 *   13.2.2) is a genuine lockstep mismatch, detected by the functional
 *   compare path rather than by one of the CCM-R5F's test modes.
 *
 *   The resulting ESM group2 error is deliberately left pending and
 *   nERROR is left low, so that the run-time response can be observed:
 *   as soon as the OS unmasks FIQ, rm57_esm_interrupt() must report
 *   "group 2 channel 2".
 *
 ****************************************************************************/

#ifdef CONFIG_RM57_SELFTEST_INJECT_CCM_MISMATCH
static void st_inject_ccm(void)
{
  uint32_t sr2;
  uint32_t ccmsr;

  putreg32(1, RM57_CCM_POLCNTRL);
  sr2   = getreg32(RM57_ESM_SR2);
  ccmsr = getreg32(RM57_CCM_SR1);
  putreg32(0, RM57_CCM_POLCNTRL);

  st_puts("SELFTEST: injected CPU compare mismatch, ESM SR2 ");
  st_puthex(sr2);
  st_puts(" CCMSR1 ");
  st_puthex(ccmsr);
  st_puts((sr2 & ESM_CHANBIT(ESM_G2_CCM_CPU)) != 0 ?
          " - detected, left pending\n" : " - NOT DETECTED\n");

  if ((sr2 & ESM_CHANBIT(ESM_G2_CCM_CPU)) == 0)
    {
      st_fail(RM57_ST_CCM_CPU, 0xf0, sr2, ccmsr);
    }
}
#endif

/****************************************************************************
 * Name: st_inject_l2ram
 *
 * Description:
 *   Fault injection: raise the L2RAMW two-bit diagnostic error and leave
 *   it pending, with nERROR low, for the run-time ESM handler, which must
 *   report "group 2 channel 7".
 *
 ****************************************************************************/

#ifdef CONFIG_RM57_SELFTEST_INJECT_L2RAM_2BIT
static void st_inject_l2ram(void)
{
  uint32_t sr2;

  st_l2ram_trigger(L2RAMW_RAMTEST_MODE_EQ, ST_L2RAM_ECC_2BIT);
  sr2 = getreg32(RM57_ESM_SR2);

  st_puts("SELFTEST: injected L2RAM 2-bit error, ESM SR2 ");
  st_puthex(sr2);
  st_puts(" RAMERRSTATUS ");
  st_puthex(getreg32(RM57_L2RAMW_RAMERRSTATUS));
  st_puts((sr2 & ESM_CHANBIT(ESM_G2_L2RAMW_TYPEB)) != 0 ?
          " - detected, left pending\n" : " - NOT DETECTED\n");

  if ((sr2 & ESM_CHANBIT(ESM_G2_L2RAMW_TYPEB)) == 0)
    {
      st_fail(RM57_ST_L2RAM_ECC, 0xf0, sr2, 0);
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_selftest_early
 ****************************************************************************/

uint32_t rm57_selftest_early(uint32_t sysesr, bool cpureset)
{
  uint32_t results = 0;
  int result;

  result  = st_esm_group3();
  results = st_result(results, RM57_ST_ESM_GROUP3, result);

#ifdef CONFIG_RM57_SELFTEST_STC
  if (cpureset)
    {
      return st_stc_resume(results);
    }
#endif

  if (cpureset || st_debugger(sysesr) || result != RM57_ST_PASS)
    {
      /* Either this is not the start of a boot sequence, or a debugger
       * is attached, or the device is already suspect: no ROM tests and
       * no LBIST.
       */

      results = st_result(results, RM57_ST_PBIST_ROMINIT, RM57_ST_SKIPPED);
      results = st_result(results, RM57_ST_PBIST_ROM, RM57_ST_SKIPPED);
      results = st_result(results, RM57_ST_STC_SELFCHECK, RM57_ST_SKIPPED);
      return st_result(results, RM57_ST_STC_CPU, RM57_ST_SKIPPED);
    }

#ifdef CONFIG_RM57_SELFTEST_PBIST_ROM
  result  = st_pbist_rominit();
  results = st_result(results, RM57_ST_PBIST_ROMINIT, result);

  if (result == RM57_ST_PASS)
    {
      result  = st_pbist_rom();
      results = st_result(results, RM57_ST_PBIST_ROM, result);
    }
#endif

#ifdef CONFIG_RM57_SELFTEST_STC
  if (result == RM57_ST_PASS)
    {
      /* Does not return unless its preconditions fail */

      result  = st_stc_start(RM57_ST_STC_SELFCHECK, true, 1);
      results = st_result(results, RM57_ST_STC_SELFCHECK, result);
    }
  else
    {
      results = st_result(results, RM57_ST_STC_SELFCHECK, RM57_ST_SKIPPED);
    }

  results = st_result(results, RM57_ST_STC_CPU, RM57_ST_SKIPPED);
#endif

  return results;
}

/****************************************************************************
 * Name: rm57_selftest_late
 ****************************************************************************/

void rm57_selftest_late(uint32_t sysesr, uint32_t results)
{
  bool debugger = st_debugger(sysesr);
  uint32_t esm_before = getreg32(RM57_ESM_SR4(0));
#ifdef CONFIG_RM57_SELFTEST_CCM
  int i;
#endif

#ifdef CONFIG_RM57_SELFTEST_CCM
  for (i = 0; i < nitems(g_ccm_diag); i++)
    {
      results = st_result(results, g_ccm_diag[i].test,
                          debugger ? RM57_ST_SKIPPED :
                                     st_ccm(&g_ccm_diag[i]));
    }
#endif

#ifdef CONFIG_RM57_SELFTEST_EFUSE
  /* The ECC self-test is meant to run once per power-on reset cycle */

  results = st_result(results, RM57_ST_EFUSE,
                      (sysesr & SYS_SYSESR_PORST) != 0 ? st_efuse() :
                                                         RM57_ST_SKIPPED);
#endif

#ifdef CONFIG_RM57_SELFTEST_L2RAM
  results = st_result(results, RM57_ST_L2RAM_ECC, st_l2ram_ecc());
  results = st_result(results, RM57_ST_L2RAM_RAD, st_l2ram_rad());
#endif

  g_selftest.sysesr   = sysesr;
  g_selftest.results  = results;
  g_selftest.debugger = debugger;

  /* Whatever the diagnostics left set in the ESM, they did not expect.
   * Group1 flags do nothing by themselves (no channel is enabled), but
   * they are worth seeing.  SR4 is sampled on both sides of this
   * function to tell the CPU self-test's leftovers from the rest.
   */

  g_selftest.esm[0] = getreg32(RM57_ESM_SR1(0));
  g_selftest.esm[1] = getreg32(RM57_ESM_SR4(0));
  g_selftest.esm[2] = getreg32(RM57_ESM_SR7(0));
  g_selftest.esm[3] = getreg32(RM57_ESM_SR2) | getreg32(RM57_ESM_SSR2);
  g_selftest.esm[4] = getreg32(RM57_ESM_SR3);
  g_selftest.esm[5] = esm_before;

#if defined(CONFIG_RM57_SELFTEST_INJECT_CCM_MISMATCH) || \
    defined(CONFIG_RM57_SELFTEST_INJECT_L2RAM_2BIT)
  /* The injected error is announced on the console, which has not been
   * set up yet on this path.
   */

  rm57_lowsetup();
#endif

#ifdef CONFIG_RM57_SELFTEST_INJECT_CCM_MISMATCH
  if (!debugger)
    {
      st_inject_ccm();
    }
#endif

#ifdef CONFIG_RM57_SELFTEST_INJECT_L2RAM_2BIT
  st_inject_l2ram();
#endif

  /* The reset cause has been captured: clear it so that the next reset
   * starts from a clean slate.
   */

  putreg32(sysesr & SYS_SYSESR_ALL, RM57_SYS_SYSESR);
}

/****************************************************************************
 * Name: rm57_selftest_report
 ****************************************************************************/

void rm57_selftest_report(void)
{
  static const char * const outcome[4] =
  {
    "not run", "PASS", "skipped", "FAIL"
  };

  uint32_t sysesr = g_selftest.sysesr;
  int test;

  st_puts("\nSELFTEST: reset cause ");
  st_puthex(sysesr);
  st_puts((sysesr & SYS_SYSESR_PORST) != 0 ? " power-on" : "");
  st_puts((sysesr & SYS_SYSESR_OSCRST) != 0 ? " osc" : "");
  st_puts((sysesr & SYS_SYSESR_WDRST) != 0 ? " watchdog" : "");
  st_puts((sysesr & SYS_SYSESR_DBGRST) != 0 ? " debugger" : "");
  st_puts((sysesr & SYS_SYSESR_ICSTRST) != 0 ? " interconnect" : "");
  st_puts((sysesr & SYS_SYSESR_CPURST_ALL) != 0 ? " cpu" : "");
  st_puts((sysesr & SYS_SYSESR_SWRST) != 0 ? " software" : "");
  st_puts((sysesr & SYS_SYSESR_EXTRST) != 0 ? " external" : "");
  st_puts(g_selftest.debugger ? " (debugger attached)\n" : "\n");

  for (test = 0; test < RM57_ST_NTESTS; test++)
    {
      st_puts("SELFTEST: ");
      st_puts(g_testname[test]);
      st_puts(": ");
      st_puts(outcome[RM57_ST_RESULT(g_selftest.results, test)]);
      st_puts("\n");
    }

  st_puts("SELFTEST: ESM residue SR1 ");
  st_puthex(g_selftest.esm[0]);
  st_puts(" SR4 ");
  st_puthex(g_selftest.esm[1]);
  st_puts(" (was ");
  st_puthex(g_selftest.esm[5]);
  st_puts(") SR7 ");
  st_puthex(g_selftest.esm[2]);
  st_puts(" SR2 ");
  st_puthex(g_selftest.esm[3]);
  st_puts(" SR3 ");
  st_puthex(g_selftest.esm[4]);
  st_puts("\n");

  rm57_lowflush();
}

/****************************************************************************
 * Name: rm57_selftest_results
 ****************************************************************************/

const struct rm57_selftest_s *rm57_selftest_results(void)
{
  return &g_selftest;
}

#endif /* CONFIG_RM57_SELFTEST */
