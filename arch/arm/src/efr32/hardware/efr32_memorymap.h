/****************************************************************************
 * arch/arm/src/efr32/hardware/efr32_memorymap.h
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

#ifndef __ARCH_ARM_SRC_EFR32_HARDWARE_EFR32_MEMORYMAP_H
#define __ARCH_ARM_SRC_EFR32_HARDWARE_EFR32_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#if defined(CONFIG_EFR32_EFR32MG24)
#  include "hardware/efr32mg24_memorymap.h"
#else
#  error "Unsupported EFR32 memory map"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Most Series 2 peripherals alias each register block at fixed offsets:
 * a write to the SET alias sets the written bits, a write to the CLR alias
 * clears them and a write to the TGL alias toggles them.
 */

#define EFR32_SET_OFFSET         0x1000
#define EFR32_CLR_OFFSET         0x2000
#define EFR32_TGL_OFFSET         0x3000

#endif /* __ARCH_ARM_SRC_EFR32_HARDWARE_EFR32_MEMORYMAP_H */
