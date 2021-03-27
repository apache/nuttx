/****************************************************************************
 * arch/risc-v/src/c906/c906_userspace.c
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

#include <nuttx/userspace.h>

#include "riscv_internal.h"
#include "c906_userspace.h"

#ifdef CONFIG_BUILD_PROTECTED

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* TODO: get user space mem layout info from ld script or Configuration ? */

#ifndef CONFIG_NUTTX_USERSPACE_SIZE
#  define CONFIG_NUTTX_USERSPACE_SIZE        (0x00100000)
#endif

#ifndef CONFIG_NUTTX_USERSPACE_RAM_START
#  define CONFIG_NUTTX_USERSPACE_RAM_START   (0x00100000)
#endif

#ifndef CONFIG_NUTTX_USERSPACE_RAM_SIZE
#  define CONFIG_NUTTX_USERSPACE_RAM_SIZE    (0x00100000)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: c906_userspace
 *
 * Description:
 *   For the case of the separate user-/kernel-space build, perform whatever
 *   platform specific initialization of the user memory is required.
 *   Normally this just means initializing the user space .data and .bss
 *   segments.
 *
 ****************************************************************************/

void c906_userspace(void)
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

  /* Configure the PMP to permit user-space access to its ROM and RAM.
   * Now this is done by simply adding the whole memory area to PMP.
   * 1. no access for the 1st 4KB
   * 2. "RX" for the left space until 1MB
   * 3. "RW" for the user RAM area
   * TODO: more accurate memory size control.
   */

  riscv_config_pmp_region(0, PMPCFG_A_NAPOT,
                          0,
                          0x1000);

  riscv_config_pmp_region(1, PMPCFG_A_TOR | PMPCFG_X | PMPCFG_R,
                          0 + CONFIG_NUTTX_USERSPACE_SIZE,
                          0);

  riscv_config_pmp_region(2, PMPCFG_A_NAPOT | PMPCFG_W | PMPCFG_R,
                          CONFIG_NUTTX_USERSPACE_RAM_START,
                          CONFIG_NUTTX_USERSPACE_RAM_SIZE);
}

#endif /* CONFIG_BUILD_PROTECTED */
