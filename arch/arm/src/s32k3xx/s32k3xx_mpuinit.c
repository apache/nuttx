/****************************************************************************
 * arch/arm/src/s32k3xx/s32k3xx_mpuinit.c
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
#include "s32k3xx_mpuinit.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef MAX
#  define MAX(a,b) a > b ? a : b
#endif

#ifndef MIN
#  define MIN(a,b) a < b ? a : b
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
 * Private Types
 ****************************************************************************/

extern uint8_t SRAM_BASE_ADDR[];
extern uint8_t SRAM_INIT_END_ADDR[];
extern uint8_t ITCM_BASE_ADDR[];
extern uint8_t ITCM_END_ADDR[];
extern uint8_t DTCM_BASE_ADDR[];
extern uint8_t DTCM_END_ADDR[];
extern uint8_t FLASH_BASE_ADDR[];
extern uint8_t FLASH_END_ADDR[];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k3xx_mpuinitialize
 *
 * Description:
 *   Configure the MPU to permit user-space access to only restricted SAM3U
 *   resources.
 *
 ****************************************************************************/

#if defined(CONFIG_ARCH_USE_MPU)
void s32k3xx_mpuinitialize(void)
{
  uint32_t regval;
  uint32_t region;
#ifdef CONFIG_BUILD_PROTECTED
  uintptr_t datastart;
  uintptr_t dataend;
#endif

  /* Show MPU information */

  mpu_showtype();

#ifdef CONFIG_ARMV7M_DCACHE
  /* Memory barrier */

  ARM_DMB();

#endif

  /* Reset MPU if enabled */

  mpu_reset();

  /* ARM errata 1013783-B Workaround */

  region = mpu_allocregion();
  DEBUGASSERT(region == 0);

  /* Select the region */

  putreg32(region, MPU_RNR);

  /* Select the region base address */

  putreg32(region | MPU_RBAR_VALID, MPU_RBAR);

  /* The configure the region */

  regval = MPU_RASR_ENABLE        | /* Enable region  */
           MPU_RASR_SIZE_LOG2(32) | /* entire memory */
           MPU_RASR_TEX_SO        | /* Strongly ordered */
           MPU_RASR_AP_RWRW       | /* P:RW   U:RW */
           MPU_RASR_XN;             /* Execute-never to prevent instruction fetch */
  putreg32(regval, MPU_RASR);

  mpu_configure_region((uintptr_t)FLASH_BASE_ADDR,
                     FLASH_END_ADDR - FLASH_BASE_ADDR,
                     MPU_RASR_TEX_SO   | /* Strongly ordered    */
                     RASR_C_VALUE      | /* Cacheable          */
                     MPU_RASR_B        | /* Bufferable
                                          * Not Shareable      */
                     MPU_RASR_AP_RORO);  /* P:RO   U:RO
                                          * Instruction access */

  mpu_configure_region((uintptr_t)SRAM_BASE_ADDR,
                     SRAM_INIT_END_ADDR - SRAM_BASE_ADDR,
                     MPU_RASR_TEX_SO   | /* Strongly ordered   */
                     RASR_C_VALUE      | /* Cacheable          */
                     RASR_B_VALUE      | /* Bufferable
                                          * Not Shareable      */
                     MPU_RASR_AP_RWRW);  /* P:RW   U:RW
                                          * Instruction access */

  mpu_configure_region((uintptr_t)ITCM_BASE_ADDR,
                     ITCM_END_ADDR - ITCM_BASE_ADDR,
                     MPU_RASR_TEX_SO   | /* Strongly ordered   */
                     RASR_C_VALUE      | /* Cacheable          */
                     RASR_B_VALUE      | /* Bufferable
                                          * Not Shareable      */
                     MPU_RASR_AP_RWRW);  /* P:RW   U:RW
                                          * Instruction access */

  mpu_configure_region((uintptr_t)DTCM_BASE_ADDR,
                     DTCM_END_ADDR - DTCM_BASE_ADDR,
                     MPU_RASR_TEX_SO   | /* Strongly ordered   */
                     RASR_C_VALUE      | /* Cacheable          */
                     RASR_B_VALUE      | /* Bufferable
                                          * Not Shareable      */
                     MPU_RASR_AP_RWRW);  /* P:RW   U:RW
                                          * Instruction access */

  mpu_configure_region(0x40000000, 3 * 2048 * 1024,
                     MPU_RASR_TEX_DEV  | /* Device
                                          * Not Cacheable
                                          * Not Bufferable
                                          * Not Shareable      */
                     MPU_RASR_AP_RWRW);  /* P:RW   U:RW
                                          * Instruction access */

#ifdef CONFIG_BUILD_PROTECTED
  /* Configure user flash and SRAM space */

  DEBUGASSERT(USERSPACE->us_textend >= USERSPACE->us_textstart);

  mpu_user_flash(USERSPACE->us_textstart,
                 USERSPACE->us_textend - USERSPACE->us_textstart);

  datastart = MIN(USERSPACE->us_datastart, USERSPACE->us_bssstart);
  dataend   = MAX(USERSPACE->us_dataend,   USERSPACE->us_bssend);

  DEBUGASSERT(dataend >= datastart);

  mpu_user_intsram(datastart, dataend - datastart);
#endif

  /* Then enable the MPU */

  mpu_control(true, false, true);
}

#endif

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_ARM_MPU)

/****************************************************************************
 * Name: s32k3xx_mpu_uheap
 *
 * Description:
 *  Map the user-heap region.
 *
 *  This logic may need an extension to handle external SDRAM).
 *
 ****************************************************************************/

void s32k3xx_mpu_uheap(uintptr_t start, size_t size)
{
  /* Configure the user SRAM space
   * Ordered
   * Cacheable
   * Not Bufferable
   * Not Shareable
   * P:RW   U:RW
   * Instruction access
   */

  mpu_configure_region(start, size,
                       MPU_RASR_TEX_SO   |
                       MPU_RASR_C        |
                       MPU_RASR_AP_RWRW
                       );
}

#endif /* CONFIG_BUILD_PROTECTED && CONFIG_ARM_MPU */
