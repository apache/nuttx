/****************************************************************************
 * arch/arm/src/max326xx/hardware/max326_sir.h
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

#ifndef __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX326_SIR_H
#define __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX326_SIR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/max326_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define MAX326_SIR_STAT_OFFSET        0x0000 /* System Initialization Status Register */
#define MAX326_SIR_ADDRER_OFFSET      0x0004 /* System Initialization Address Error Register */

/* Register Addresses *******************************************************/

#define MAX326_SIR_STAT               (MAX326_SIR_BASE + MAX326_SIR_STAT_OFFSET)
#define MAX326_SIR_ADDRER             (MAX326_SIR_BASE + MAX326_SIR_ADDRER_OFFSET)

/* Register Bit-field Definitions *******************************************/

/* System Initialization Status Register */

#define SIR_STAT_CFGVALID             (1 << 0)  /* Bit 0:  Configuration Valid Flag */
#define SIR_STAT_CFGERR               (1 << 1)  /* Bit 1:  Configuration Error Flag */

/* System Initialization Address Error Register
 * (32-bit Configuration Error Address)
 */

#endif /* __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX326_SIR_H */
