/****************************************************************************
 * arch/arm/src/imxrt/imxrt_mpuinit.c
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

#include <nuttx/userspace.h>

#include "mpu.h"
#include "barriers.h"

#include "hardware/imxrt_memorymap.h"

#include "imxrt_mpuinit.h"

#ifdef CONFIG_ARM_MPU

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef MIN
#  define MIN(a,b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#  define MAX(a,b) (((a) > (b)) ? (a) : (b))
#endif

#ifndef CONFIG_ARMV7M_DCACHE
  /*  With Dcache off:
   *  Cacheable (MPU_RASR_C) and Bufferable (MPU_RASR_B) needs to be off
   */
#  undef  MPU_RASR_B
#  define MPU_RASR_B    0
#  define RASR_B_VALUE  0
#  define RASR_C_VALUE  0
#else
#  ifndef CONFIG_ARMV7M_DCACHE_WRITETHROUGH
  /*  With Dcache on:
   *  Cacheable (MPU_RASR_C) and Bufferable (MPU_RASR_B) needs to be on
   */
#  define RASR_B_VALUE  MPU_RASR_B
#  define RASR_C_VALUE  MPU_RASR_C

#  else
  /*  With Dcache in WRITETHROUGH Bufferable (MPU_RASR_B)
   * needs to be off, except for FLASH for alignment leniency
   */
#  define RASR_B_VALUE  0
#  define RASR_C_VALUE  MPU_RASR_C
#  endif
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_mpu_initialize
 *
 * Description:
 *   Configure the MPU to permit user-space access to only restricted i.MXRT
 *   resources.
 *
 ****************************************************************************/

void imxrt_mpu_initialize(void)
{
#ifdef CONFIG_BUILD_PROTECTED
  uintptr_t datastart;
  uintptr_t dataend;
#endif

  /* Show MPU information */

  mpu_showtype();

#ifdef CONFIG_ARMV7M_DCACHE
  /* Memory barrier */

  ARM_DMB();

#ifdef CONFIG_IMXFT_QSPI
  /* Make QSPI memory region strongly ordered */

  mpu_priv_stronglyordered(IMXRT_QSPIMEM_BASE, IMXRT_QSPIMEM_SIZE);

#endif
#endif

#ifdef CONFIG_BUILD_PROTECTED
  /* Configure user flash and SRAM space */

  DEBUGASSERT(USERSPACE->us_textend >= USERSPACE->us_textstart);

  mpu_user_flash(USERSPACE->us_textstart,
                 USERSPACE->us_textend - USERSPACE->us_textstart);

  datastart = MIN(USERSPACE->us_datastart, USERSPACE->us_bssstart);
  dataend   = MAX(USERSPACE->us_dataend,   USERSPACE->us_bssend);

  DEBUGASSERT(dataend >= datastart);

  mpu_user_intsram(datastart, dataend - datastart);
#else
  mpu_configure_region(0xc0000000, 512 * 1024 * 1024,
                       MPU_RASR_TEX_DEV  | /* Device
                                            * Not Cacheable
                                            * Not Bufferable
                                            * Not Shareable      */
                       MPU_RASR_AP_RWRW);  /* P:RW   U:RW
                                            * Instruction access */

  mpu_configure_region(IMXRT_EXTMEM_BASE, 1024 * 1024 * 1024,
                       MPU_RASR_TEX_DEV  | /* Device
                                            * Not Cacheable
                                            * Not Bufferable
                                            * Not Shareable      */
                       MPU_RASR_AP_RWRW);  /* P:RW   U:RW
                                            * Instruction access */

  mpu_configure_region(IMXRT_FLEXCIPHER_BASE, 8 * 1024 * 1024,
                       MPU_RASR_TEX_NOR  | /* Normal             */
                       RASR_C_VALUE      | /* Cacheable          */
                       MPU_RASR_B        | /* Bufferable
                                            * Not Shareable      */
                       MPU_RASR_AP_RORO);  /* P:RO   U:RO
                                            * Instruction access */

  mpu_configure_region(IMXRT_ITCM_BASE,  128 * 1024,
                       MPU_RASR_TEX_NOR  | /* Normal             */
                       RASR_C_VALUE      | /* Cacheable          */
                       RASR_B_VALUE      | /* Bufferable
                                            * Not Shareable      */
                       MPU_RASR_AP_RWRW);  /* P:RW   U:RW
                                            * Instruction access */

  mpu_configure_region(IMXRT_DTCM_BASE,  128 * 1024,
                       MPU_RASR_TEX_NOR  | /* Normal             */
                       RASR_C_VALUE      | /* Cacheable          */
                       RASR_B_VALUE      | /* Bufferable
                                            * Not Shareable      */
                       MPU_RASR_AP_RWRW);  /* P:RW   U:RW
                                            * Instruction access */

  mpu_configure_region(IMXRT_OCRAM2_BASE,  512 * 1024,
                       MPU_RASR_TEX_NOR  | /* Normal             */
                       RASR_C_VALUE      | /* Cacheable          */
                       RASR_B_VALUE      | /* Bufferable
                                            * Not Shareable      */
                       MPU_RASR_AP_RWRW);  /* P:RW   U:RW
                                            * Instruction access */

  mpu_configure_region(IMXRT_OCRAM_BASE,  512 * 1024,
                       MPU_RASR_TEX_NOR  | /* Normal             */
                       RASR_C_VALUE      | /* Cacheable          */
                       RASR_B_VALUE      | /* Bufferable
                                            * Not Shareable      */
                       MPU_RASR_AP_RWRW);  /* P:RW   U:RW
                                            * Instruction access */

  mpu_configure_region(IMXRT_EXTMEM_BASE,  32 * 1024 * 1024,
                       MPU_RASR_TEX_SO   | /* Ordered            */
                       RASR_C_VALUE      | /* Cacheable          */
                       RASR_B_VALUE      | /* Bufferable
                                            * Not Shareable      */
                       MPU_RASR_AP_RWRW);  /* P:RW   U:RW
                                            * Instruction access */

  mpu_configure_region(0x81e00000,  2 * 1024 * 1024,
                       MPU_RASR_TEX_NOR  | /* Normal
                                            * Not Cacheable
                                            * Not Bufferable
                                            * Not Shareable      */
                       MPU_RASR_AP_RWRW);  /* P:RW   U:RW
                                            * Instruction access */

  mpu_control(true, true, true);
  return;
#endif

  /* Then enable the MPU */

  mpu_control(true, false, true);
}

/****************************************************************************
 * Name: imxrt_mpu_uheap
 *
 * Description:
 *  Map the user-heap region.
 *
 *  This logic may need an extension to handle external SDRAM).
 *
 ****************************************************************************/

#ifdef CONFIG_BUILD_PROTECTED
void imxrt_mpu_uheap(uintptr_t start, size_t size)
{
  mpu_user_intsram(start, size);
}
#endif

#endif /* CONFIG_ARM_MPU */
