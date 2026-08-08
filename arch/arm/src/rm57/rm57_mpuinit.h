/****************************************************************************
 * arch/arm/src/rm57/rm57_mpuinit.h
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

#ifndef __ARCH_ARM_SRC_RM57_RM57_MPUINIT_H
#define __ARCH_ARM_SRC_RM57_RM57_MPUINIT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <sys/types.h>

#include "mpu.h"
#include "hardware/rm57l843_memorymap.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RM57_NUM_OF_MPU_REGION    (3)

/* Peripheral register region: covers the 0xf0000000-0xffffffff span,
 * which contains all peripheral bases used by this port (GIO, SCI, VIM,
 * ESM, PCR, RTI, SYS, flash wrapper, pinmux - see
 * hardware/rm57l843_memorymap.h). Not cacheable, not executable.
 */

#define RM57_PERIPH_START_ADDR    (0xf0000000)
#define RM57_PERIPH_SIZE          (0x10000000ul)

/* FLASH REGION
 *   Cacheable, bufferable, executable
 */

#define rm57_flash_region(base, size) \
  mpu_configure_region(base, size, MPU_RACR_TEX(1) | \
                                   MPU_RACR_C       | \
                                   MPU_RACR_B       | \
                                   MPU_RACR_AP_RWRW)

/* SRAM REGION
 *   Cacheable, bufferable, executable
 */

#define rm57_sram_region(base, size) \
  mpu_configure_region(base, size, MPU_RACR_TEX(1) | \
                                   MPU_RACR_C       | \
                                   MPU_RACR_B       | \
                                   MPU_RACR_AP_RWRW)

/* PERIPHERAL REGION
 *   Not cacheable, not bufferable, shareable (device memory), execute
 *   never
 */

#define rm57_periph_region(base, size) \
  mpu_configure_region(base, size, MPU_RACR_S    | \
                                   MPU_RACR_AP_RWRW | \
                                   MPU_RACR_XN)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_mpu_init
 *
 * Description:
 *   Initialize the MPU by disabling it, resetting all regions, configuring
 *   the flash/SRAM/peripheral regions, and then re-enabling the MPU.
 *
 ****************************************************************************/

void rm57_mpu_init(void);

#endif /* __ARCH_ARM_SRC_RM57_RM57_MPUINIT_H */
