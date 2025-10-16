/****************************************************************************
 * arch/arm/src/am67/am67_mpuinit.h
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

#ifndef __ARCH_ARM_SRC_AM67_AM67_MPUINIT_H
#define __ARCH_ARM_SRC_AM67_AM67_MPUINIT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <sys/types.h>

#include "mpu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AM67_NUM_OF_MPU_REGION    (5)

#define AM67_REGISTER_START_ADDR  (0x0)
#define AM67_TCMA_START_ADDR      (0x0)
#define AM67_TCMB_START_ADDR      (0x41010000)
#define AM67_MCU_MSRAM_START_ADDR (0x60000000)
#define AM67_DDR_START_ADDR       (0x80000000)

#define AM67_REGISTER_SIZE        (2ul * 1024 * 1024 * 1024)
#define AM67_TCMA_SIZE            (32ul * 1024)
#define AM67_TCMB_SIZE            (32ul * 1024)
#define AM67_MCU_MSRAM_SIZE       (512ul * 1024)

#define AM67_DDR_SIZE             (2ul * 1024 * 1024 * 1024)

/* REGISTER_REGION
 *   Not Cacheable
 *   Not Bufferable
 *   Shareable
 *   Execute never
 *   P:RW   U:R
 */
#define am67_register_region(base,size) \
  mpu_configure_region(base, size, MPU_RACR_S | \
                                   MPU_RACR_AP_RWRW)

/* TCMA REGION
 *   Bufferable
 *   Cacheable
 *   P:RW   U:R0
 *   Allow user RW access, executable
 */
#define am67_tcma_region(base, size) \
  mpu_configure_region(base, size, MPU_RACR_TEX(1)  | \
                                   MPU_RACR_B       | \
                                   MPU_RACR_AP_RWRW)

/* TCMB REGION
 *   Bufferable
 *   Cacheable
 *   P:RW   U:R0
 *   Allow user RW access, executable
 */
#define am67_tcmb_region(base, size) \
  mpu_configure_region(base, size, MPU_RACR_TEX(1)  | \
                                   MPU_RACR_B       | \
                                   MPU_RACR_C       | \
                                   MPU_RACR_AP_RWRW)

/* TCMB REGION
 *   Bufferable
 *   Cacheable
 *   P:RW   U:R0
 *   Allow user RW access, executable
 */
#define am67_mcu_msram_region(base,size)  \
  mpu_configure_region(base, size, MPU_RACR_TEX(1)  | \
                                   MPU_RACR_C       | \
                                   MPU_RACR_B       | \
                                   MPU_RACR_AP_RWRW)

/* DDR REGION
 *   Shareable
 *   Cacheable
 *   Bufferable
 *   P:RW   U:RW
 */
#define am67_ddr_region(base,size) \
  mpu_configure_region(base, size, MPU_RACR_TEX(1)  | \
                                   MPU_RACR_S       | \
                                   MPU_RACR_C       | \
                                   MPU_RACR_B       | \
                                   MPU_RACR_AP_RWRW)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am67_mpu_disable_br
 *
 * Description:
 *   Disable the MPU background region by clearing bit 17 in the SCTLR
 *   register.
 *
 ****************************************************************************/

static inline void am67_mpu_disable_br(void)
{
  unsigned int sctlr = cp15_rdsctlr();
  sctlr &= ~(1 << 17);  /* Clear bit 17 (disable background region) */
  cp15_wrsctlr(sctlr);
}

/****************************************************************************
 * Name: mpu_set_region_zero
 *
 * Description:
 *   Configure an MPU region with zero base address, size, and attributes
 *   effectively disabling the specified region.
 *
 ****************************************************************************/

static inline void mpu_set_region_zero(uint32_t region_id)
{
  register uint32_t r0 asm("r0") = region_id;
  register uint32_t r1 asm("r1") = 0;
  register uint32_t r2 asm("r2") = 0;
  register uint32_t r3 asm("r3") = 0;

  asm volatile (
    "mcr p15, 0, %0, c6, c2, 0\n\t"
    "mcr p15, 0, %1, c6, c1, 0\n\t"
    "mcr p15, 0, %2, c6, c1, 2\n\t"
    "mcr p15, 0, %3, c6, c1, 4\n\t"
    :
    : "r"(r0), "r"(r1), "r"(r2), "r"(r3)
    : "memory"
  );
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: am67_mpu_reset
 *
 * Description:
 *   Reset all MPU regions by disabling each region.
 *
 ****************************************************************************/

void am67_mpu_reset(void);

/****************************************************************************
 * Name: am67_mpu_init
 *
 * Description:
 *   Initialize the MPU by disabling it, resetting all regions, configuring
 *   specific memory regions, and then re-enabling the MPU.
 *
 ****************************************************************************/

void am67_mpu_init(void);

#endif /* __ARCH_ARM_SRC_AM67_AM67_MPUINIT_H*/
