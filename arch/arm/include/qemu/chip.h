/****************************************************************************
 * arch/arm/include/qemu/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_QEMU_CHIP_H
#define __ARCH_ARM_INCLUDE_QEMU_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

#if CONFIG_ARM_GIC_VERSION == 2

#define CONFIG_GICD_BASE           0x8000000
#define CONFIG_GICR_BASE           0x8010000
#define CONFIG_GICM_BASE           0x8020000

#elif CONFIG_ARM_GIC_VERSION == 3 || CONFIG_ARM_GIC_VERSION == 4

#define CONFIG_GICD_BASE           0x8000000
#define CONFIG_GICR_BASE           0x80a0000
#define CONFIG_GICR_OFFSET         0x20000
#else

#error CONFIG_ARM64_GIC_VERSION should be 2, 3 or 4

#endif /* CONFIG_ARM_GIC_VERSION */

#define MPID_TO_CLUSTER_ID(mpid)  ((mpid) & ~0xff)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_INCLUDE_QEMU_CHIP_H */
