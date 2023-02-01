/****************************************************************************
 * arch/arm/src/lc823450/lc823450_mpuinit2.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <sys/param.h>

#include <nuttx/userspace.h>

#include "mpu.h"
#include "arm_internal.h"
#include "lc823450_mpuinit2.h"

#ifdef CONFIG_ARM_MPU

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lc823450_mpuinitialize
 *
 * Description:
 *   Configure the MPU to permit user-space access to only restricted SAM3U
 *   resources.
 *
 ****************************************************************************/

void lc823450_mpuinitialize(void)
{
#ifdef CONFIG_BUILD_PROTECTED
  uintptr_t datastart = MIN(USERSPACE->us_datastart, USERSPACE->us_bssstart);
  uintptr_t dataend   = MAX(USERSPACE->us_dataend,   USERSPACE->us_bssend);

  DEBUGASSERT(USERSPACE->us_textend >= USERSPACE->us_textstart &&
              dataend >= datastart);

  /* Show MPU information */

  mpu_showtype();

  /* Configure user flash and SRAM space */

  mpu_user_flash(USERSPACE->us_textstart,
                 USERSPACE->us_textend - USERSPACE->us_textstart);

  mpu_user_intsram(datastart, dataend - datastart);
#endif

#ifdef CONFIG_BUILD_FLAT
  uint32_t size = _eronly - _stext;

  /* 128KB align */

  size = size & 0x20000;

  /* Protect text area in SRAM as privileged flash */

  mpu_priv_flash((uintptr_t)_stext, size);
#endif

  /* Then enable the MPU */

  mpu_control(true, false, true);
}

/****************************************************************************
 * Name: lc823450_mpu_uheap
 *
 * Description:
 *  Map the user-heap region.
 *
 *  This logic may need an extension to handle external SDRAM).
 *
 ****************************************************************************/

void lc823450_mpu_uheap(uintptr_t start, size_t size)
{
  mpu_user_intsram(start, size);
}

#endif /* CONFIG_ARM_MPU */
