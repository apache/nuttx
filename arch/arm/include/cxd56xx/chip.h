/****************************************************************************
 * arch/arm/include/cxd56xx/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_CXD56XX_CHIP_H
#define __ARCH_ARM_INCLUDE_CXD56XX_CHIP_H

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* physical address conversion macro */

#define CXD56_PHYSADDR(a) ((uint32_t)((uint32_t)(a) & 0x9ffffffful))

#define CXD56M4_SYSH_PRIORITY_MIN     0xe0 /* All bits[7:5] set is minimum priority */
#define CXD56M4_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define CXD56M4_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define CXD56M4_SYSH_PRIORITY_STEP    0x20 /* Steps between priorities */

#define NVIC_SYSH_PRIORITY_MIN        CXD56M4_SYSH_PRIORITY_MIN
#define NVIC_SYSH_PRIORITY_DEFAULT    CXD56M4_SYSH_PRIORITY_DEFAULT
#define NVIC_SYSH_PRIORITY_MAX        CXD56M4_SYSH_PRIORITY_MAX
#define NVIC_SYSH_PRIORITY_STEP       CXD56M4_SYSH_PRIORITY_STEP

#endif /* __ARCH_ARM_INCLUDE_CXD56XX_CHIP_H */
