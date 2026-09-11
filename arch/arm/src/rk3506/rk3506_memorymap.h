/****************************************************************************
 * arch/arm/src/rk3506/rk3506_memorymap.h
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

#ifndef __ARCH_ARM_SRC_RK3506_RK3506_MEMORYMAP_H
#define __ARCH_ARM_SRC_RK3506_RK3506_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <arch/chip/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RK3506 Physical Memory Map (for the NuttX AMP subsystem on CPU2) *********
 *
 * DDR:     0x00000000 - 0x10000000  (256MB window, normal memory)
 *            - NuttX firmware loaded at 0x03e00000 (see amp_linux_mcu.its)
 *            - rpmsg shared memory at 0x03c00000
 * Device:  0xff000000 - 0xfff00000  (peripherals: UART 0xff0e0000,
 *            GIC 0xff581000, etc, device memory)
 */

#define RK3506_DDR_PSECTION      0x00000000  /* 0x00000000-0x10000000 */
#define RK3506_DEVICE_PSECTION   0xff000000  /* 0xff000000-0xfff00000 */

#define RK3506_DDR_VSECTION      RK3506_DDR_PSECTION
#define RK3506_DEVICE_VSECTION   RK3506_DEVICE_PSECTION

/* Sizes of memory regions in bytes. */

#define RK3506_DDR_SECSIZE       (256 * 1024 * 1024)
#define RK3506_DEVICE_SECSIZE    (15 * 1024 * 1024)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

int rk3506_setupmappings(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RK3506_RK3506_MEMORYMAP_H */
