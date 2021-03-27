/****************************************************************************
 * arch/arm/src/kinetis/hardware/kinetis_mpu.h
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

#ifndef __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_MPU_H
#define __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_MPU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/* This file is just a wrapper around MPU header files for the Kinetis family
 * selected by the in chip.h.
 */

#if defined(KINETIS_K20) || defined(KINETIS_K40) || defined(KINETIS_K60)
#  include "hardware/kinetis_k20k40k60mpu.h"
#elif defined(KINETIS_K28) || defined(KINETIS_K64) || defined(KINETIS_K66)
#  include "hardware/kinetis_k28k64k66mpu.h"
#else
#  error "No MPU definitions for this Kinetis part"
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
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_HARDWARE_KINETIS_MPU_H */
