/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/hardware/lpc17_40_memorymap.h
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

#ifndef __ARCH_ARM_SRC_LPC17XX_40XX_HARDWARE_LPC17_40_MEMORYMAP_H
#define __ARCH_ARM_SRC_LPC17XX_40XX_HARDWARE_LPC17_40_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* This file is only a thin shell that includes the correct memory map
 * definitions for the selected LPC17xx/LPC40xx family.
 */

#include <arch/lpc17xx_40xx/chip.h>

#if defined(LPC176x)
#  include "hardware/lpc176x_memorymap.h"
#elif defined(LPC178x_40xx)
#  include "hardware/lpc178x_40xx_memorymap.h"
#else
#  error "Unrecognized LPC17xx/LPC40xx family"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC17XX_40XX_HARDWARE_LPC17_40_MEMORYMAP_H */
