/****************************************************************************
 * arch/arm/include/nrf52/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_NRF52_CHIP_H
#define __ARCH_ARM_INCLUDE_NRF52_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* NVIC priority levels *****************************************************/

/* Each priority field holds a priority value, 0-31. The lower the value,
 * the greater the priority of the corresponding interrupt.
 *
 * The Cortex-M4 core supports 8 programmable interrupt priority levels.
 */

#define NVIC_SYSH_PRIORITY_MIN          0xe0 /* All bits[7:5] set is minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT      0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX          0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP         0x20 /* Steps between priorities */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_INCLUDE_NRF52_CHIP_H */
