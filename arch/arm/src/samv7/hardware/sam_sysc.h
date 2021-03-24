/****************************************************************************
 * arch/arm/src/samv7/hardware/sam_sysc.h
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

#ifndef __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_SYSC_H
#define __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_SYSC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SYSC register offsets ****************************************************/

#define SAM_SYSC_WPMR_OFFSET   0x0004 /* Write Protection Mode Register */

/* SYSC register addresses **************************************************/

#define SAM_SYSC_WPMR          (SAM_SYSC_BASE+SAM_SYSC_WPMR_OFFSET)

/* SYSC register bit definitions ********************************************/

/* System Controller Write Protect Mode Register */

#define SYSC_WPMR_WPEN         (1 << 0)  /* Bit 0:  Write Protect Enable */
#define SYSC_WPMR_WPKEY_SHIFT  (8)       /* Bits 8-31: Write Protect KEY */
#define SYSC_WPMR_WPKEY_MASK   (0x00ffffff << SYSC_WPMR_WPKEY_SHIFT)
#  define SYSC_WPMR_WPKEY      (0x00525443 << SYSC_WPMR_WPKEY_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_SYSC_H */
