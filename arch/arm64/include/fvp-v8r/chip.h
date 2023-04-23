/****************************************************************************
 * arch/arm64/include/fvp-v8r/chip.h
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

#ifndef __ARCH_ARM64_INCLUDE_FVP_V8R_CHIP_H
#define __ARCH_ARM64_INCLUDE_FVP_V8R_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* Number of bytes in @p x kibibytes/mebibytes/gibibytes */

#define KB(x)           ((x) << 10)
#define MB(x)           (KB(x) << 10)
#define GB(x)           (MB(UINT64_C(x)) << 10)

#if defined(CONFIG_ARCH_CHIP_FVP_ARMV8R)

#if CONFIG_ARM_GIC_VERSION == 2

#define CONFIG_GICD_BASE          0xAF000000
#define CONFIG_GICR_BASE          0xAF100000

#elif CONFIG_ARM_GIC_VERSION == 3 || CONFIG_ARM_GIC_VERSION == 4

#define CONFIG_GICD_BASE          0xAF000000
#define CONFIG_GICR_BASE          0xAF100000
#define CONFIG_GICR_OFFSET        0x20000

#else

#error CONFIG_ARM_GIC_VERSION should be 2, 3 or 4

#endif /* CONFIG_ARM_GIC_VERSION */

#define CONFIG_RAMBANK_ADDR      0x00000000
#define CONFIG_RAMBANK_SIZE      MB(128)
#define CONFIG_RAMBANK_END       \
        (CONFIG_RAMBANK_ADDR + CONFIG_RAMBANK_SIZE)

#define CONFIG_DEVICEIO1_BASEADDR  0xAF000000
#define CONFIG_DEVICEIO1_SIZE      MB(128)
#define CONFIG_DEVICEIO1_END       \
        (CONFIG_DEVICEIO1_BASEADDR + CONFIG_DEVICEIO1_SIZE)

#define CONFIG_DEVICEIO2_BASEADDR  0x9C000000
#define CONFIG_DEVICEIO2_SIZE      MB(128)
#define CONFIG_DEVICEIO2_END       \
        (CONFIG_DEVICEIO2_BASEADDR + CONFIG_DEVICEIO2_SIZE)

#define CONFIG_LOAD_BASE          0x00000000

#define MPID_TO_CLUSTER_ID(mpid)  ((mpid) & ~0xff)

#endif

#endif /* __ARCH_ARM64_INCLUDE_FVP_V8R_CHIP_H */
