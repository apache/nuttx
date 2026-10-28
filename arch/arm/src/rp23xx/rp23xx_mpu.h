/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_mpu.h
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

#ifndef __ARCH_ARM_SRC_RP23XX_RP23XX_MPU_H
#define __ARCH_ARM_SRC_RP23XX_RP23XX_MPU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "arm_internal.h"
#include "mpu.h"
#include "hardware/rp23xx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The SRAM, including the SCRATCH_X and SCRATCH_Y banks */

#define RP23XX_MPU_SRAM_BASE    RP23XX_SRAM_BASE
#define RP23XX_MPU_SRAM_SIZE    (520 * 1024)

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_mpu_share_sram
 *
 * Description:
 *   The Cortex-M33 checks an exclusive access (LDREX/STREX) with the global
 *   exclusive monitor of RP2350 only when the memory is Shareable.  The
 *   default memory map makes the SRAM Non-shareable, so the two cores see
 *   only their own local monitors and an atomic operation, a spinlock among
 *   them, does not exclude the other core.
 *
 *   Map the SRAM as Normal, Non-cacheable, Shareable memory with an MPU
 *   region and keep the default memory map for everything else.  The MPU
 *   is banked per core, so each core must call this before it relies on an
 *   atomic operation.
 *
 ****************************************************************************/

static inline void rp23xx_mpu_share_sram(void)
{
  uintptr_t base  = RP23XX_MPU_SRAM_BASE;
  uintptr_t limit = RP23XX_MPU_SRAM_BASE + RP23XX_MPU_SRAM_SIZE - 1;

  putreg32(0, MPU_CTRL);
  UP_MB();

  /* Attribute index 0: Normal memory, Non-cacheable */

  modifyreg32(MPU_MAIR0, 0xff, MPU_MAIR_NONCACHEABLE);

  putreg32(0, MPU_RNR);
  putreg32((base & MPU_RBAR_BASE_MASK) | MPU_RBAR_SH_OUTER |
           MPU_RBAR_AP_RWRW, MPU_RBAR);
  putreg32((limit & MPU_RLAR_LIMIT_MASK) | (0 << MPU_RLAR_INDX_SHIFT) |
           MPU_RLAR_ENABLE, MPU_RLAR);

  putreg32(MPU_CTRL_ENABLE | MPU_CTRL_PRIVDEFENA, MPU_CTRL);
  UP_MB();
}

#endif /* __ARCH_ARM_SRC_RP23XX_RP23XX_MPU_H */
