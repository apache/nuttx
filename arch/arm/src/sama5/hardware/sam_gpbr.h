/****************************************************************************
 * arch/arm/src/sama5/hardware/sam_gpbr.h
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

#ifndef __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_GPBR_H
#define __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_GPBR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPBR Register Offsets ****************************************************/

#define SAM_SYS_GPBR_OFFSET(n) ((n) << 2) /* General Purpose Backup Register n, 1=0..3 */

#define SAM_SYS_GPBR0_OFFSET   0x0000 /* General Purpose Backup Register 0 */
#define SAM_SYS_GPBR1_OFFSET   0x0004 /* General Purpose Backup Register 0 */
#define SAM_SYS_GPBR2_OFFSET   0x0008 /* General Purpose Backup Register 0 */
#define SAM_SYS_GPBR3_OFFSET   0x000c /* General Purpose Backup Register 0 */

/* GPBR Register Addresses **************************************************/

#define SAM_SYS_GPBR(n)        (SAM_GPBR_VBASE+SAM_SYS_GPBR_OFFSET(n))
#define SAM_SYS_GPBR0          (SAM_GPBR_VBASE+SAM_SYS_GPBR0_OFFSET)
#define SAM_SYS_GPBR1          (SAM_GPBR_VBASE+SAM_SYS_GPBR1_OFFSET)
#define SAM_SYS_GPBR2          (SAM_GPBR_VBASE+SAM_SYS_GPBR2_OFFSET)
#define SAM_SYS_GPBR3          (SAM_GPBR_VBASE+SAM_SYS_GPBR3_OFFSET)

/* GPBR Register Bit Definitions ********************************************/

/* All GPBR registers hold user-defined, 32-bit values */

#endif /* __ARCH_ARM_SRC_SAMA5_HARDWARE_SAM_GPBR_H */
