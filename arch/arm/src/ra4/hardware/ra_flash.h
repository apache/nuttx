/****************************************************************************
 * arch/arm/src/ra4/hardware/ra_flash.h
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

#ifndef __ARCH_ARM_SRC_RA4M1_HARDWARE_RA4M1_FLASH_H
#define __ARCH_ARM_SRC_RA4M1_HARDWARE_RA4M1_FLASH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/ra4/chip.h>
#include "ra_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define R_FCACHE_FCACHEE_OFFSET           0x0100 /* Flash Cache Enable Register (16-Bits) */
#define R_FCACHE_FCACHEIV_OFFSET          0x0104 /* Flash Cache Invalidate Register (16-Bits) */
#define R_FCACHE_FLWT_OFFSET              0x011c /* Flash Cache FLWT (8-Bits) */

/* Register Addresses *******************************************************/

# define R_FCACHE_FCACHEE                  (R_FCACHE_BASE + R_FCACHE_FCACHEE_OFFSET)
# define R_FCACHE_FCACHEIV                 (R_FCACHE_BASE + R_FCACHE_FCACHEIV_OFFSET)
# define R_FCACHE_FLWT                     (R_FCACHE_BASE + R_FCACHE_FLWT_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Flash Cache Enable Register (16-Bits) */

#define R_FCACHE_FCACHEE_FCACHEEN         (1 <<  0) /* 01: FCACHE Enable */

/* Flash Cache Invalidate Register (16-Bits) */

#define R_FCACHE_FCACHEIV_FCACHEIV        (1 <<  0) /* 01: FCACHE Invalidation */

/* Flash Cache FLWT (8-Bits) */

#define R_FCACHE_FLWT_FLWT                (3 <<  0) /* 01: These bits represent the ratio of the CPU clock period to the Flash memory access time. */
#define R_FCACHE_FLWT_FLWT_MASK           (0x07)

#endif /* __ARCH_ARM_SRC_RA4M1_HARDWARE_RA4M1_FLASH_H */
