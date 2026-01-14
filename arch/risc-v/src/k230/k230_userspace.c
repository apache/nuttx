/****************************************************************************
 * arch/risc-v/src/k230/k230_userspace.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>

#include <nuttx/queue.h>
#include <nuttx/userspace.h>

#include <arch/board/board_memorymap.h>

#include "k230_userspace.h"
#include "riscv_internal.h"

#ifdef CONFIG_BUILD_PROTECTED

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define UFLASH_F   (PMPCFG_A_NAPOT | PMPCFG_X | PMPCFG_R)
#define USRAM_F    (PMPCFG_A_NAPOT | PMPCFG_W | PMPCFG_R)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: configure_mpu
 *
 * Description:
 *   This function configures the MPU for for kernel- / userspace separation.
 *   It will also grant access to the page table memory for the supervisor.
 *
 ****************************************************************************/

static void configure_mpu(void);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: k230_userspace
 *
 * Description:
 *   For the case of the separate user-/kernel-space build, perform whatever
 *   platform specific initialization of the user memory is required.
 *   Normally this just means initializing the user space .data and .bss
 *   segments.
 *
 ****************************************************************************/

void k230_userspace(void)
{
  uint8_t *src;
  uint8_t *dest;
  uint8_t *end;

  /* Clear all of user-space .bss */

  DEBUGASSERT(USERSPACE->us_bssstart != 0 && USERSPACE->us_bssend != 0 &&
              USERSPACE->us_bssstart <= USERSPACE->us_bssend);

  dest = (uint8_t *)USERSPACE->us_bssstart;
  end  = (uint8_t *)USERSPACE->us_bssend;

  while (dest != end)
    {
      *dest++ = 0;
    }

  /* Initialize all of user-space .data */

  DEBUGASSERT(USERSPACE->us_datasource != 0 &&
              USERSPACE->us_datastart != 0 && USERSPACE->us_dataend != 0 &&
              USERSPACE->us_datastart <= USERSPACE->us_dataend);

  src  = (uint8_t *)USERSPACE->us_datasource;
  dest = (uint8_t *)USERSPACE->us_datastart;
  end  = (uint8_t *)USERSPACE->us_dataend;

  while (dest != end)
    {
      *dest++ = *src++;
    }

  /* Configure MPU / PMP to grant access to the userspace */

  configure_mpu();
}

/****************************************************************************
 * Name: configure_mpu
 *
 * Description:
 *   This function configures the MPU for for kernel- / userspace separation.
 *
 ****************************************************************************/

static void configure_mpu(void)
{
  int ret;
  ret = riscv_append_pmp_region(UFLASH_F, UFLASH_START, UFLASH_SIZE);
  DEBUGASSERT(ret == 0);
  ret = riscv_append_pmp_region(USRAM_F, USRAM_START, USRAM_SIZE);
  DEBUGASSERT(ret == 0);
}

#endif /* CONFIG_BUILD_PROTECTED */
