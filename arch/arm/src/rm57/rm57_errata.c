/****************************************************************************
 * arch/arm/src/rm57/rm57_errata.c
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: 2009-2018 Texas Instruments Incorporated
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
 * 3. Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
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

/* Port of errata_PBIST_4() from TI's HALCoGen 04.07.01 to NuttX
 * register access and coding style.
 *
 * The routine drives PBIST controller registers and two system registers
 * that the Technical Reference Manual does not document, with values that
 * only TI can explain; TI describes it as a black box.  Every address and
 * value below is therefore kept exactly as in the original and must not
 * be "tidied up" against the TRM.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "arm_internal.h"
#include "sctlr.h"

#include "hardware/rm57_sys.h"
#include "hardware/rm57_pbist.h"
#include "rm57_selftest.h"

#ifdef CONFIG_RM57_SELFTEST_PBIST_ROM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Undocumented system registers that switch the ROMs into, and back out
 * of, the mode in which the PBIST controller can initialize them.
 */

#define ERRATA_ROMINIT_REG0       0xffff0400
#define ERRATA_ROMINIT_REG1       0xffff040c

/* Undocumented PBIST controller registers.  The first group holds the
 * micro-coded algorithm, CMS/RUN/CSR sit in the gap that the TRM leaves
 * between DLR and PACT, and FSRDL0X/FSRDL1X are further fail status words.
 */

#define ERRATA_PBIST_A0           (RM57_PBIST_BASE + 0x0000)
#define ERRATA_PBIST_A1           (RM57_PBIST_BASE + 0x0004)
#define ERRATA_PBIST_A2           (RM57_PBIST_BASE + 0x0008)
#define ERRATA_PBIST_A3           (RM57_PBIST_BASE + 0x000c)
#define ERRATA_PBIST_A4           (RM57_PBIST_BASE + 0x0010)
#define ERRATA_PBIST_L0           (RM57_PBIST_BASE + 0x0040)
#define ERRATA_PBIST_L1           (RM57_PBIST_BASE + 0x0044)
#define ERRATA_PBIST_L2           (RM57_PBIST_BASE + 0x0048)
#define ERRATA_PBIST_L3           (RM57_PBIST_BASE + 0x004c)
#define ERRATA_PBIST_L4           (RM57_PBIST_BASE + 0x0050)
#define ERRATA_PBIST_D0           (RM57_PBIST_BASE + 0x0120)
#define ERRATA_PBIST_D1           (RM57_PBIST_BASE + 0x0124)
#define ERRATA_PBIST_E0           (RM57_PBIST_BASE + 0x0130)
#define ERRATA_PBIST_CA0          (RM57_PBIST_BASE + 0x0140)
#define ERRATA_PBIST_CL0          (RM57_PBIST_BASE + 0x0150)
#define ERRATA_PBIST_CMS          (RM57_PBIST_BASE + 0x0168)
#define ERRATA_PBIST_RUN          (RM57_PBIST_BASE + 0x016c)
#define ERRATA_PBIST_CSR          (RM57_PBIST_BASE + 0x0178)
#define ERRATA_PBIST_FSRDL0X      (RM57_PBIST_BASE + 0x01ac)
#define ERRATA_PBIST_FSRDL1X      (RM57_PBIST_BASE + 0x01b4)
#define ERRATA_PBIST_FSRDL1Y      (RM57_PBIST_BASE + 0x01b8)

/* DLR value for CPU control of the controller without ROM-based testing */

#define ERRATA_PBIST_DLR_CONFIG   0x00000010

/* The algorithm loops 1000 times; one pass that actually ran takes
 * between 20 and 200 ROM clock cycles more than that.  The ROM clock is
 * GCLK1 / 4 here, the same prescaler HALCoGen uses.
 */

#define ERRATA_ROMCLK_DIV         4
#define ERRATA_LOOP_COUNT         1000
#define ERRATA_EXTRA_MIN          20
#define ERRATA_EXTRA_MAX          200

#define ERRATA_TIMEOUT            4000000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct errata_rom_s
{
  uint32_t d0;
  uint32_t d1;
  uint32_t ramt;
  uint32_t csr;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* STC1 ROM 1, STC1 ROM 2, STC2 ROM, PBIST ROM - in HALCoGen's order */

static const struct errata_rom_s g_errata_rom[4] =
{
  { 0xfff0007c, 0x07b3ffff, 0x0e01200c, 1 },
  { 0xa88fa473, 0x00bd719d, 0x0e02200c, 2 },
  { 0xfff0007c, 0x06e3ffff, 0x0f01200c, 1 },
  { 0x00000002, 0x00000000, 0x0101200c, 1 }
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_errata_pbist4
 ****************************************************************************/

int rm57_errata_pbist4(uint32_t *info)
{
  volatile int i;
  uint32_t pmcr;
  uint32_t cycles;
  uint32_t regval;
  uint32_t count;
  int32_t extra;
  int ret = 0;
  int rom;

  /* The run time of each pass is measured with the PMU cycle counter,
   * counting every CPU cycle (no divide-by-64).
   */

  pmcr = cp15_pmu_rdpmcr();
  cp15_pmu_wrpmcr((pmcr | PMCR_E) & ~(PMCR_D | PMCR_C | PMCR_P));
  cp15_pmu_wrcesr(PMCESR_CCES);

  /* ROM init setup, using reserved registers */

  putreg32(0x0000000a, ERRATA_ROMINIT_REG0);
  putreg32(0x0000ee0a, ERRATA_ROMINIT_REG1);

  for (rom = 0; rom < 4 && ret == 0; rom++)
    {
      /* Disable PBIST clocks and ROM clock */

      putreg32(0, RM57_PBIST_PACT);
      if (getreg32(RM57_PBIST_PACT) != 0)
        {
          *info = getreg32(RM57_PBIST_PACT);
          ret   = 3 | (rom << 4);
          break;
        }

      /* PBIST ROM clock = GCLK1 / 4; disable the memory self-test and
       * memory initialization controllers, then enable self-test.
       */

      putreg32(SYS_MSTGCR_ROMDIV_DIV4 | SYS_MSTGCR_MSTGENA_DISABLE,
               RM57_SYS_MSTGCR);
      putreg32(SYS_MINITGCR_DISABLE, RM57_SYS_MINITGCR);
      putreg32(SYS_MSTGCR_ROMDIV_DIV4 | SYS_MSTGCR_MSTGENA_ENABLE,
               RM57_SYS_MSTGCR);

      /* Clear PBIST done, enable the PBIST controller */

      putreg32(SYS_MSTCGSTAT_MSTDONE, RM57_SYS_MSTCGSTAT);
      putreg32(SYS_MSINENA_PBIST, RM57_SYS_MSINENA);

      /* Wait for at least 64 VBUS clock cycles */

      for (i = 0; i < 256; i++)
        {
        }

      /* Enable PBIST clocks and ROM clock; CPU control of PBIST */

      putreg32(PBIST_PACT_ENABLE, RM57_PBIST_PACT);
      putreg32(ERRATA_PBIST_DLR_CONFIG, RM57_PBIST_DLR);

      /* Load the PBIST algorithm that initializes the ROMs */

      putreg32(0x00000001, ERRATA_PBIST_A0);
      putreg32(0x00000025, ERRATA_PBIST_L0);
      putreg32(0x62400001, ERRATA_PBIST_A1);
      putreg32(0x00000004, ERRATA_PBIST_L1);
      putreg32(0x00068003, ERRATA_PBIST_A2);
      putreg32(0x00000000, ERRATA_PBIST_L2);
      putreg32(0x00000004, ERRATA_PBIST_A3);
      putreg32(0x00006860, ERRATA_PBIST_L3);
      putreg32(0x00000000, ERRATA_PBIST_A4);
      putreg32(0x00000001, ERRATA_PBIST_L4);
      putreg32(0x000003e8, ERRATA_PBIST_CA0);
      putreg32(0x00000001, ERRATA_PBIST_CL0);
      putreg32(0x00000000, ERRATA_PBIST_E0);

      /* Select the ROM */

      putreg32(g_errata_rom[rom].d0, ERRATA_PBIST_D0);
      putreg32(g_errata_rom[rom].d1, ERRATA_PBIST_D1);
      putreg32(g_errata_rom[rom].ramt, RM57_PBIST_RAMT);
      putreg32(g_errata_rom[rom].csr, ERRATA_PBIST_CSR);
      putreg32(8, ERRATA_PBIST_CMS);

      /* Zero the cycle counter and run */

      cp15_pmu_wrpmcr(cp15_pmu_rdpmcr() | PMCR_C);
      putreg32(1, ERRATA_PBIST_RUN);

      for (count = 0; count < ERRATA_TIMEOUT; count++)
        {
          if ((getreg32(RM57_SYS_MSTCGSTAT) & SYS_MSTCGSTAT_MSTDONE) != 0)
            {
              break;
            }
        }

      cycles = cp15_pmu_rdccr();
      extra  = (int32_t)(cycles / ERRATA_ROMCLK_DIV) - ERRATA_LOOP_COUNT;

      /* Check the PBIST status results (address, data, ...) */

      regval = getreg32(RM57_PBIST_FSRA0) | getreg32(RM57_PBIST_FSRA1) |
               getreg32(RM57_PBIST_FSRDL0) | getreg32(ERRATA_PBIST_FSRDL0X) |
               getreg32(RM57_PBIST_FSRDL1) | getreg32(ERRATA_PBIST_FSRDL1X) |
               getreg32(ERRATA_PBIST_FSRDL1Y);

      if (regval != 0)
        {
          *info = regval;
          ret   = 1 | (rom << 4);
        }

      /* Check that the algorithm executed in the expected amount of time */

      else if (count >= ERRATA_TIMEOUT || extra <= ERRATA_EXTRA_MIN ||
               extra >= ERRATA_EXTRA_MAX)
        {
          *info = cycles;
          ret   = 2 | (rom << 4);
        }

      /* Disable PBIST clocks and ROM clock, disable PBIST */

      putreg32(0, RM57_PBIST_PACT);

      regval  = getreg32(RM57_SYS_MSTGCR);
      regval &= ~SYS_MSTGCR_MSTGENA_MASK;
      regval |= SYS_MSTGCR_MSTGENA_DISABLE;
      putreg32(regval, RM57_SYS_MSTGCR);
    }

  /* ROM restore default setup (must be completed before continuing) */

  putreg32(0x0000aa0a, ERRATA_ROMINIT_REG1);
  putreg32(0x0000aa05, ERRATA_ROMINIT_REG1);
  putreg32(0x00000005, ERRATA_ROMINIT_REG0);

  cp15_pmu_wrcecr(PMCECR_CCEC);
  cp15_pmu_wrpmcr(pmcr & ~(PMCR_C | PMCR_P));

  return ret;
}

#endif /* CONFIG_RM57_SELFTEST_PBIST_ROM */
