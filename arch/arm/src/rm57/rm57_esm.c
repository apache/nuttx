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

/* Ported from TI's HALCoGen esmInit() (HL_esm.c). Runs as early
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
  putreg32(0xffffffff, RM57_ESM_SR1(2));

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
