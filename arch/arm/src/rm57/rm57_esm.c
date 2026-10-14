/****************************************************************************
 * arch/arm/src/rm57/rm57_esm.c
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

/* Ported from TI's HALCoGen esmInit(). Runs as early
 * as possible in boot (right after clock configuration) to put the
 * Error Signaling Module into a known, quiet state: all error-pin
 * channels disabled, all interrupts disabled, all latched status
 * flags cleared, the nERROR pin released, and all interrupt levels
 * set to 0 (no channel forces a group2 high-priority response). The
 * reference project enables nothing beyond this default-quiet state.
 *
 * Without this, ESM is left at its power-on-reset configuration for
 * the whole early boot sequence, which on this device family is not
 * necessarily benign.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <inttypes.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>

#include "arm_internal.h"
#include "hardware/rm57_esm.h"
#include "rm57_esm.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_esm_initialize
 ****************************************************************************/

void rm57_esm_initialize(void)
{
  /* Disable all error-pin channels and interrupts */

  putreg32(0xffffffff, RM57_ESM_DEPAPR1);
  putreg32(0xffffffff, RM57_ESM_IEPCR4);
  putreg32(0xffffffff, RM57_ESM_IEPCR7);

  putreg32(0xffffffff, RM57_ESM_IECR1);
  putreg32(0xffffffff, RM57_ESM_IECR4);
  putreg32(0xffffffff, RM57_ESM_IECR7);

  /* Clear all latched error status flags */

  putreg32(0xffffffff, RM57_ESM_SR1(0));
  putreg32(0xffffffff, RM57_ESM_SR1(1));
  putreg32(0xffffffff, RM57_ESM_SSR2);

#ifndef CONFIG_RM57_SELFTEST
  putreg32(0xffffffff, RM57_ESM_SR1(2));
#else
  /* Group3 flags left by the reset sequence (eFuse autoload, implicit
   * flash OTP reads) are evidence: rm57_selftest_early() examines and
   * clears them.
   */
#endif

  putreg32(0xffffffff, RM57_ESM_SR4(0));
  putreg32(0xffffffff, RM57_ESM_SR7(0));

  /* Low-pulse counter preload (matches HALCoGen default) */

  putreg32(16384 - 1, RM57_ESM_LTCPR);

  /* Release the nERROR pin */

  if (getreg32(RM57_ESM_EPSR) == 0)
    {
      putreg32(0x00000005, RM57_ESM_EKR);
    }
  else
    {
      putreg32(0x00000000, RM57_ESM_EKR);
    }

  /* Clear then zero all interrupt levels (no channel forces the
   * group2 high-priority response)
   */

  putreg32(0xffffffff, RM57_ESM_ILCR1);
  putreg32(0xffffffff, RM57_ESM_ILCR4);
  putreg32(0xffffffff, RM57_ESM_ILCR7);

  putreg32(0, RM57_ESM_ILSR1);
  putreg32(0, RM57_ESM_ILSR4);
  putreg32(0, RM57_ESM_ILSR7);

  /* Leave all error-pin channels and interrupts disabled (EEPAPR1 /
   * IEPSR4 / IEPSR7 left at 0) - matches the reference project, which
   * does not arm any ESM-driven response beyond this quiet default.
   */
}

/****************************************************************************
 * Name: rm57_esm_interrupt
 *
 * Description:
 *   Handle the ESM high-level interrupt, which arrives on VIM channel 0 and
 *   is hard-wired to FIQ.
 *
 *   Group2 errors are nonmaskable, so without this handler a latched group2
 *   error leaves the FIQ permanently asserted.  The default arm_vectorfiq
 *   is a bare "subs pc, lr, #4" that acknowledges nothing, so the CPU would
 *   re-enter the vector forever the moment anything unmasked CPSR.F - which
 *   is exactly what a debugger does on resume.
 *
 ****************************************************************************/

#ifdef CONFIG_ARMV7R_DECODEFIQ
int rm57_esm_interrupt(int irq, void *context, void *arg)
{
  unsigned int guard = ESM_NCHANNELS;
  uintptr_t regaddr;
  uint32_t offset;
  int channel;
  int group;

  /* Drain every pending high-level channel.  ESMIOFFHR is a priority
   * encoder that reports one channel per read (SPNU562A Table 16-13), so a
   * single read could strand a second error.  The loop is bounded because a
   * group1 flag we fail to clear below would otherwise keep it alive.
   */

  while ((offset = getreg32(RM57_ESM_IOFFHR) & ESM_IOFF_MASK) !=
         ESM_IOFF_NONE)
    {
      if (guard-- == 0)
        {
          _alert("ESM: giving up, IOFFHR stuck at %02" PRIx32 "\n", offset);
          break;
        }

      /* Decode the banded group/channel encoding, and clear the flag.
       * Reading ESMIOFFHR above already cleared the group2 flag; group1
       * flags are write-1-to-clear in SR1/SR4/SR7.
       */

      if (offset >= ESM_IOFF_GRP2_FIRST && offset <= ESM_IOFF_GRP2_LAST)
        {
          group   = 2;
          channel = offset - ESM_IOFF_GRP2_FIRST;
          regaddr = 0;
        }
      else if (offset <= ESM_IOFF_GRP1_LO_LAST)
        {
          group   = 1;
          channel = offset - ESM_IOFF_GRP1_LO_FIRST;
          regaddr = RM57_ESM_SR1(0);
        }
      else if (offset >= ESM_IOFF_GRP1_MID_FIRST &&
               offset <= ESM_IOFF_GRP1_MID_LAST)
        {
          group   = 1;
          channel = offset - ESM_IOFF_GRP1_MID_FIRST + ESM_CHANS_PER_REG;
          regaddr = RM57_ESM_SR4(0);
        }
      else if (offset >= ESM_IOFF_GRP1_HI_FIRST &&
               offset <= ESM_IOFF_GRP1_HI_LAST)
        {
          group   = 1;
          channel = offset - ESM_IOFF_GRP1_HI_FIRST + 2 * ESM_CHANS_PER_REG;
          regaddr = RM57_ESM_SR7(0);
        }
      else
        {
          /* Reserved encoding (0x61-0x80).  Nothing to clear, and the guard
           * above stops us spinning on it.
           */

          _alert("ESM: reserved IOFFHR value %02" PRIx32 "\n", offset);
          continue;
        }

      if (regaddr != 0)
        {
          putreg32(1 << (channel % ESM_CHANS_PER_REG), regaddr);
        }

      _alert("ESM: group %d channel %d\n", group, channel);
    }

#ifdef CONFIG_RM57_ESM_PANIC
  PANIC();
#endif

  return OK;
}
#endif /* CONFIG_ARMV7R_DECODEFIQ */
