/****************************************************************************
 * arch/arm/include/s32k1xx/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_S32K1XX_CHIP_H
#define __ARCH_ARM_INCLUDE_S32K1XX_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* NVIC priority levels *****************************************************/

/* Each priority field holds a priority value. The lower the value, the
 * greater the priority of the corresponding interrupt.
 */

#if defined(CONFIG_ARCH_CORTEXM4)
/* The Cortex-M4F core supports 16 programmable interrupt priority levels. */

#  define NVIC_SYSH_PRIORITY_MIN        0xf0 /* All bits[7:4] set is minimum priority */
#  define NVIC_SYSH_PRIORITY_DEFAULT    0x80 /* Midpoint is the default */
#  define NVIC_SYSH_PRIORITY_MAX        0x00 /* Zero is maximum priority */
#  define NVIC_SYSH_PRIORITY_STEP       0x10 /* Steps between priorities */

#elif defined(CONFIG_ARCH_ARMV6M)
/* The Cortex-M0+ core supports 4 programmable interrupt priority levels. */

#  define NVIC_SYSH_PRIORITY_MIN        0xc0 /* All bits[7:4] set is minimum priority */
#  define NVIC_SYSH_PRIORITY_DEFAULT    0x80 /* Midpoint is the default */
#  define NVIC_SYSH_PRIORITY_MAX        0x00 /* Zero is maximum priority */
#  define NVIC_SYSH_PRIORITY_STEP       0x40 /* Steps between priorities */

#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_INCLUDE_S32K1XX_CHIP_H */
