/****************************************************************************
 * arch/arm/include/xmc4/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_XMC4_CHIP_H
#define __ARCH_ARM_INCLUDE_XMC4_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* Get customizations for each supported chip */

#if defined(CONFIG_ARCH_CHIP_XMC4500)
#  define XMC4_NUSIC          3          /* Three USIC modules: USCI0-2 */
#  undef  XMC4_SCU_GATING                /* No clock gating registers */
#  define XMC4_NECAT          0          /* No EtherCAT support */
#elif defined(CONFIG_ARCH_CHIP_XMC4700)
#  define XMC4_NUSIC          3          /* Three USIC modules: USCI0-2 */
#  define XMC4_SCU_GATING     1          /* Has clock gating registers */
#  define XMC4_NECAT          0          /* No EtherCAT support */
#elif defined(CONFIG_ARCH_CHIP_XMC4800)
#  define XMC4_NUSIC          3          /* Three USIC modules: USCI0-2 */
#  define XMC4_SCU_GATING     1          /* Has clock gating registers */
#  define XMC4_NECAT          1          /* One EtherCAT module */
#else
#  error "Unsupported XMC4xxx chip"
#endif

/* NVIC priority levels *****************************************************/

/* Each priority field holds a priority value.  The lower the value, the
 * greater the priority of the corresponding interrupt.
 * The XMC4500 implements only bits[7:2] of this field, bits[1:0]
 * read as zero and ignore writes.
 */

#define NVIC_SYSH_PRIORITY_MIN     0xfc /* All bits[7:2] set is minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x04 /* Steps between supported priority values */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_INCLUDE_XMC4_CHIP_H */
