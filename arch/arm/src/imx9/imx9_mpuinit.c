/****************************************************************************
 * arch/arm/src/imx9/imx9_mpuinit.c
 *
 * SPDX-License-Identifier: Apache-2.0
 * SPDX-FileCopyrightText: 2024 NXP
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

#include <arch/barriers.h>

#include "mpu.h"

#include "hardware/imx9_memorymap.h"

#include "imx9_mpuinit.h"
#include "arm_internal.h"

#ifdef CONFIG_ARM_MPU

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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
 * Name: imx9_mpu_initialize
 *
 * Description:
 *   Configure the MPU to permit user-space access to only restricted i.MXRT
 *   resources.
 *
 ****************************************************************************/

void imx9_mpu_initialize(void)
{
  uint32_t regval;
  uint32_t region;
#ifdef CONFIG_BUILD_PROTECTED
  uintptr_t datastart;
  uintptr_t dataend;
#endif

  /* Show MPU information */

  mpu_showtype();

  /* Reset MPU if enabled */

  mpu_reset();

#ifdef CONFIG_ARMV7M_DCACHE
  /* Memory barrier */

  UP_DMB();
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
           MPU_RASR_AP_NONO       | /* P:None U:None              */
           MPU_RASR_XN;             /* Execute-never to prevent instruction fetch */
  putreg32(regval, MPU_RASR);

  mpu_configure_region(IMX9_FLEXSPI1_ALIAS_BASE, 32 * 1024 * 1024,
                       MPU_RASR_AP_RORO  | /* P:R0   U:R0                */
                       MPU_RASR_TEX_NOR  | /* Normal                     */
                       MPU_RASR_C        | /* Cacheable                  */
                       MPU_RASR_B          /* Bufferable                 */
                       );

  mpu_configure_region(IMX9_ITCM_BASE, 512 * 1024,
                       MPU_RASR_AP_RORO  | /* P:R0   U:R0                */
                       MPU_RASR_TEX_NOR  | /* Normal                     */
                       MPU_RASR_C        | /* Cacheable                  */
                       MPU_RASR_B          /* Bufferable                 */
                       );

  mpu_configure_region(IMX9_DTCM_BASE, 512 * 1024,
                       MPU_RASR_AP_RWRW  | /* P:RW   U:RW                */
                       MPU_RASR_TEX_NOR  | /* Normal                     */
                       MPU_RASR_C        | /* Cacheable                  */
                       MPU_RASR_B        | /* Bufferable                 */
                       MPU_RASR_S        | /* Shareable                  */
                       MPU_RASR_XN         /* Execute-never to prevent instruction fetch */
                       );

  mpu_configure_region(0x20400000, 1 * 1024 * 1024,
                       MPU_RASR_AP_RWRW  | /* P:RW   U:RW                */
                       MPU_RASR_TEX_NOR  | /* Normal                     */
                       MPU_RASR_C        | /* Cacheable                  */
                       MPU_RASR_B          /* Bufferable                 */
                       );

  mpu_configure_region(IMX9_FLEXSPI1_BASE, 128 * 1024 * 1024,
                       MPU_RASR_AP_RORO  | /* P:R0   U:R0                */
                       MPU_RASR_TEX_NOR  | /* Strongly Ordered           */
                       MPU_RASR_C        | /* Cacheable                  */
                       MPU_RASR_B          /* Bufferable                 */
                       );

  /* AIPS1 + AIPS2 */

  mpu_configure_region(IMX9_AIPS1_BASE, 16 * 1024 * 1024,
                       MPU_RASR_AP_RWRW  | /* P:RW   U:RW                */
                       MPU_RASR_TEX_DEV    /* Device                     */
                       );

  mpu_configure_region(IMX9_AIPS3_BASE, 8 * 1024 * 1024,
                       MPU_RASR_AP_RWRW  | /* P:RW   U:RW                */
                       MPU_RASR_TEX_DEV    /* Device                     */
                       );

  mpu_configure_region(IMX9_AIPS4_BASE, 8 * 1024 * 1024,
                       MPU_RASR_AP_RWRW  | /* P:RW   U:RW                */
                       MPU_RASR_TEX_DEV    /* Device                     */
                       );

  mpu_configure_region(IMX9_GPIO_MEM_BASE, 320 * 1024,
                       MPU_RASR_AP_RWRW  | /* P:RW   U:RW                */
                       MPU_RASR_TEX_DEV    /* Device                     */
                       );

  mpu_configure_region(IMX9_GPIO1_BASE, 64 * 1024,
                       MPU_RASR_AP_RWRW  | /* P:RW   U:RW                */
                       MPU_RASR_TEX_DEV    /* Device                     */
                       );

  /* For RPMSG VRING and RSC table */

  mpu_configure_region(IMX9_RPMSG_BASE, 4096 * 1024,
                       MPU_RASR_AP_RWRW  | /* P:RW   U:RW                */
                       MPU_RASR_TEX_NOR    /* Normal                     */
                       );

  mpu_configure_region(IMX9_DRAM_XIP_BASE, 0x003ff800,
                       MPU_RASR_AP_RWRW  | /* P:RW   U:RW                */
                       MPU_RASR_TEX_NOR    /* Normal                     */
                       );
#endif

  mpu_control(true, false, true);
}

/****************************************************************************
 * Name: imx9_mpu_uheap
 *
 * Description:
 *  Map the user-heap region.
 *
 *  This logic may need an extension to handle external SDRAM).
 *
 ****************************************************************************/

#ifdef CONFIG_BUILD_PROTECTED
void imx9_mpu_uheap(uintptr_t start, size_t size)
{
  mpu_user_intsram(start, size);
}
#endif

#endif /* CONFIG_ARM_MPU */
