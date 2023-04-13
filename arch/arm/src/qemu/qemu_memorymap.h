/****************************************************************************
 * arch/arm/src/qemu/qemu_memorymap.h
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

#ifndef __ARCH_ARM_SRC_QEMU_QEMU_MEMORYMAP_H
#define __ARCH_ARM_SRC_QEMU_QEMU_MEMORYMAP_H

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

/* Qemu virt Physical Memory Map ********************************************/

#define VIRT_FLASH_PSECTION      0x00000000  /* 0x00000000-0x08000000 */
#define VIRT_IO_PSECTION         0x08000000  /* 0x08000000-0x0f000000 */
#define VIRT_PCIE_PSECTION       0x10000000  /* 0x10000000-0x40000000 */

/* Qemu virt Virtual Memory Map *********************************************/

#define VIRT_FLASH_VSECTION      VIRT_FLASH_PSECTION
#define VIRT_IO_VSECTION         VIRT_IO_PSECTION
#define VIRT_PCIE_VSECTION       VIRT_PCIE_PSECTION

/* Sizes of memory regions in bytes. */

#define VIRT_FLASH_SECSIZE       (128*1024*1024)
#define VIRT_IO_SECSIZE          (112*1024*1024)
#define VIRT_PCIE_SECSIZE        (3*256*1024*1024)

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

int qemu_setupmappings(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_QEMU_QEMU_MEMORYMAP_H */
