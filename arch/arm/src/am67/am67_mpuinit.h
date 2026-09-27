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

#define AM67_NUM_OF_MPU_REGION    (7)

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

/* Shared IPC memory (Non-cacheable). The R5F is NOT hardware-coherent
 * with the A53, so the OpenAMP/rptun shared structures in DDR -- dma
 * buffers, the
 * resource table (vdev status incl. DRIVER_OK) and the virtio/rpmsg
 * vrings --
 * must be Non-cacheable, or NuttX reads them from stale cache: it never sees
 * the A53's DRIVER_OK/vring updates and the handshake hangs (no eth0, TX
 * timeout). Two power-of-2, naturally-aligned regions cover 0xA2000000-
 * 0xA223FFFF exactly; NuttX's own RAM (0xA2240000+) stays cacheable. These
 *  are
 * configured AFTER the DDR region so they win the overlap (on the Cortex-R5
 * MPU the highest-numbered matching region takes priority).
 */

#define AM67_IPC_SHM0_START_ADDR  (0xa2000000)  /* dma buffers + resource table */
#define AM67_IPC_SHM0_SIZE        (0x200000)    /* 2 MB   (0xa2000000-0xa21fffff) */
#define AM67_IPC_SHM1_START_ADDR  (0xa2200000)  /* virtio/rpmsg vrings           */
#define AM67_IPC_SHM1_SIZE        (0x40000)     /* 256 KB (0xa2200000-0xa223ffff) */

#define AM67_SCTLR_BG_REGION_EN (1 << 17)

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
 *   Non-shareable (do not add MPU_RACR_S)
 *   Cacheable
 *   Bufferable
 *   P:RW   U:RW
 *
 * NOTE: if DDR is marked Shareable, LDREX/STREX to it take an external data
 * abort on this R5F; Non-shareable uses the core-local monitor instead.
 * Observed with ldrex-based code (e.g. C++ std::atomic).
 */
#define am67_ddr_region(base,size) \
  mpu_configure_region(base, size, MPU_RACR_TEX(1)  | \
                                   MPU_RACR_C       | \
                                   MPU_RACR_B       | \
                                   MPU_RACR_AP_RWRW)

/* SHARED IPC REGION
 *   Normal memory, Outer & Inner Non-cacheable (TEX=0b001, C=0, B=0)
 *   Non-shareable
 *   P:RW   U:RW
 *
 * For the R5F<->A53 OpenAMP shared memory. Non-cacheable means every R5F
 *  access
 * goes straight to DDR, so it stays coherent with the (coherent) A53 with no
 * cache maintenance. Kept Non-shareable so it does not hit the Cortex-R5
 * LDREX-on-Shareable external-monitor abort described on the DDR region.
 */
#define am67_ipc_shm_region(base, size) \
  mpu_configure_region(base, size, MPU_RACR_TEX(1)  | \
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
  sctlr &= ~AM67_SCTLR_BG_REGION_EN;  /* Clear bit 17 (disable background region) */
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
