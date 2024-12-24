/****************************************************************************
 * arch/arm/src/imxrt/imxrt_mpuinit.c
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

#include <assert.h>
#include <sys/param.h>

#include <nuttx/userspace.h>
#include <arch/barriers.h>

#include "mpu.h"

#include "hardware/imxrt_memorymap.h"

#include "imxrt_mpuinit.h"
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

  /* Reset MPU if enabled */

  mpu_reset();

#ifdef CONFIG_ARMV7M_DCACHE
  /* Memory barrier */

  UP_DMB();

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
#  if defined(CONFIG_ARCH_FAMILY_IMXRT117x)
  uint32_t regval;
  uint32_t region;

  mpu_reset();
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

#ifdef CONFIG_IMXRT_SEMC
  mpu_configure_region(IMXRT_SEMC0_BASE, 512 * 1024 * 1024,
                       MPU_RASR_AP_RWRW  | /* P:RW   U:RW                */
                       MPU_RASR_TEX_DEV    /* Device
                                            * Not Cacheable
                                            * Not Bufferable
                                            * Not Shareable
                                            * No Subregion disable       */
                       );
#endif

  mpu_configure_region(IMXRT_FLEXSPI2_CIPHER_BASE, 512 * 1024 * 1024,
                       MPU_RASR_AP_RWRW  | /* P:RW   U:RW                */
                       MPU_RASR_TEX_DEV    /* Device
                                            * Not Cacheable
                                            * Not Bufferable
                                            * Not Shareable
                                            * No Subregion disable       */
                       );

  mpu_configure_region(IMXRT_ITCM_BASE, 1 * 1024 * 1024 * 1024,
                       MPU_RASR_AP_RWRW  | /* P:RW   U:RW                */
                       MPU_RASR_TEX_DEV    /* Device
                                            * Not Cacheable
                                            * Not Bufferable
                                            * Not Shareable
                                            * No Subregion disable       */
                       );

  mpu_configure_region(IMXRT_ITCM_BASE, 256 * 1024,
                       MPU_RASR_AP_RORO  | /* P:R0   U:R0                */
                       MPU_RASR_TEX_NOR    /* Normal
                                            * Not Cacheable
                                            * Not Bufferable
                                            * Not Shareable
                                            * No Subregion disable       */
                       );

  mpu_configure_region(IMXRT_DTCM_BASE, 256 * 1024,
                       MPU_RASR_AP_RWRW  | /* P:RW   U:RW                */
                       MPU_RASR_TEX_NOR    /* Normal
                                            * Not Cacheable
                                            * Not Bufferable
                                            * Not Shareable
                                            * No Subregion disable       */
                       );

  mpu_configure_region(IMXRT_OCRAM_M4_BASE, 1 * 1024 * 1024,
                       MPU_RASR_AP_RWRW  | /* P:RW   U:RW                */
                       MPU_RASR_TEX_SO   | /* Strongly Ordered           */
                       RASR_C_VALUE      | /* Cacheable DCACHE ? 0 : 1   */
                       RASR_B_VALUE        /* Bufferable WB    ? 0 : 1
                                            * Not Shareable
                                            * No Subregion disable       */
                       );

  mpu_configure_region(IMXRT_OCRAM_M4_BASE + (1 * 1024 * 1024), 512 * 1024,
                       MPU_RASR_AP_RWRW  | /* P:RW   U:RW                */
                       MPU_RASR_TEX_SO   | /* Strongly Ordered           */
                       RASR_C_VALUE      | /* Cacheable DCACHE ? 0 : 1   */
                       RASR_B_VALUE        /* Bufferable WB    ? 0 : 1
                                            * Not Shareable
                                            * No Subregion disable       */
                       );

  mpu_configure_region(IMXRT_FLEXSPI1_CIPHER_BASE, 16 * 1024 * 1024,
                       MPU_RASR_AP_RORO  | /* P:R0   U:R0                */
                       MPU_RASR_TEX_SO   | /* Strongly Ordered           */
                       MPU_RASR_C        | /* Cacheable                  */
                       MPU_RASR_B          /* Bufferable
                                            * Not Shareable
                                            * No Subregion disable       */
                       );

  mpu_configure_region(IMXRT_AIPS1_BASE, 16 * 1024 * 1024,
                       MPU_RASR_AP_RWRW  | /* P:RW   U:RW                */
                       MPU_RASR_TEX_DEV    /* Device
                                            * Not Cacheable
                                            * Not Bufferable
                                            * Not Shareable
                                            * No Subregion disable       */
                       );

  mpu_configure_region(IMXRT_SIM_DISP_BASE, 2 * 1024 * 1024,
                       MPU_RASR_AP_RWRW  | /* P:RW   U:RW                */
                       MPU_RASR_TEX_DEV    /* Device
                                            * Not Cacheable
                                            * Not Bufferable
                                            * Not Shareable
                                            * No Subregion disable       */
                       );

  mpu_configure_region(IMXRT_SIM_M7_BASE, 1 * 1024 * 1024,
                       MPU_RASR_AP_RWRW  | /* P:RW   U:RW                */
                       MPU_RASR_TEX_DEV    /* Device
                                            * Not Cacheable
                                            * Not Bufferable
                                            * Not Shareable
                                            * No Subregion disable       */
                       );

  mpu_configure_region(IMXRT_GPU2D_BASE, 2 * 1024 * 1024,
                       MPU_RASR_AP_RWRW  | /* P:RW   U:RW                */
                       MPU_RASR_TEX_DEV    /* Device
                                            * Not Cacheable
                                            * Not Bufferable
                                            * Not Shareable
                                            * No Subregion disable       */
                       );

  mpu_configure_region(IMXRT_AIPS_M7_BASE, 1 * 1024 * 1024,
                       MPU_RASR_AP_RWRW  | /* P:RW   U:RW                */
                       MPU_RASR_TEX_DEV    /* Device
                                            * Not Cacheable
                                            * Not Bufferable
                                            * Not Shareable
                                            * No Subregion disable       */
                       );
#  else
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
#endif /* CONFIG_ARCH_FAMILY_IMXRT117x */
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
