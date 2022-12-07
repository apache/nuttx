/****************************************************************************
 * arch/arm64/src/common/arm64_cpu_psci.h
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

#ifndef __ARCH_ARM64_SRC_COMMON_ARM64_CPU_PSCI_H
#define __ARCH_ARM64_SRC_COMMON_ARM64_CPU_PSCI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>
#include <arch/irq.h>
#include <arch/chip/chip.h>
#include <arch/syscall.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PSCI_FN_NATIVE(version, name)   PSCI_##version##_FN64_##name

/* PSCI v0.2 interface */
#define PSCI_0_2_FN_BASE                    0x84000000
#define PSCI_0_2_FN(n)                      (PSCI_0_2_FN_BASE + (n))
#define PSCI_0_2_64BIT                      0x40000000
#define PSCI_0_2_FN64_BASE \
  (PSCI_0_2_FN_BASE + PSCI_0_2_64BIT)
#define PSCI_0_2_FN64(n)                    (PSCI_0_2_FN64_BASE + (n))

#define PSCI_0_2_FN_PSCI_VERSION            PSCI_0_2_FN(0)
#define PSCI_0_2_FN_CPU_SUSPEND             PSCI_0_2_FN(1)
#define PSCI_0_2_FN_CPU_OFF                 PSCI_0_2_FN(2)
#define PSCI_0_2_FN_CPU_ON                  PSCI_0_2_FN(3)
#define PSCI_0_2_FN_AFFINITY_INFO           PSCI_0_2_FN(4)
#define PSCI_0_2_FN_MIGRATE                 PSCI_0_2_FN(5)
#define PSCI_0_2_FN_MIGRATE_INFO_TYPE       PSCI_0_2_FN(6)
#define PSCI_0_2_FN_MIGRATE_INFO_UP_CPU     PSCI_0_2_FN(7)
#define PSCI_0_2_FN_SYSTEM_OFF              PSCI_0_2_FN(8)
#define PSCI_0_2_FN_SYSTEM_RESET            PSCI_0_2_FN(9)

#define PSCI_0_2_FN64_CPU_SUSPEND           PSCI_0_2_FN64(1)
#define PSCI_0_2_FN64_CPU_ON                PSCI_0_2_FN64(3)
#define PSCI_0_2_FN64_AFFINITY_INFO         PSCI_0_2_FN64(4)
#define PSCI_0_2_FN64_MIGRATE               PSCI_0_2_FN64(5)
#define PSCI_0_2_FN64_MIGRATE_INFO_UP_CPU   PSCI_0_2_FN64(7)

/* PSCI return values (inclusive of all PSCI versions) */
#define PSCI_RET_SUCCESS                    0
#define PSCI_RET_NOT_SUPPORTED              -1
#define PSCI_RET_INVALID_PARAMS             -2
#define PSCI_RET_DENIED                     -3
#define PSCI_RET_ALREADY_ON                 -4
#define PSCI_RET_ON_PENDING                 -5
#define PSCI_RET_INTERNAL_FAILURE           -6
#define PSCI_RET_NOT_PRESENT                -7
#define PSCI_RET_DISABLED                   -8
#define PSCI_RET_INVALID_ADDRESS            -9

/* PSCI version decoding (independent of PSCI version) */
#define PSCI_VERSION_MAJOR_SHIFT            16
#define PSCI_VERSION_MINOR_MASK \
  ((1U << PSCI_VERSION_MAJOR_SHIFT) - 1)
#define PSCI_VERSION_MAJOR_MASK             ~PSCI_VERSION_MINOR_MASK

#define PSCI_VERSION_MAJOR(ver) \
  (((ver) & PSCI_VERSION_MAJOR_MASK) >> PSCI_VERSION_MAJOR_SHIFT)
#define PSCI_VERSION_MINOR(ver) \
  ((ver) & PSCI_VERSION_MINOR_MASK)

typedef unsigned long (*psci_fn)(unsigned long, unsigned long, unsigned long,
                                 unsigned long);

struct psci_interface
{
  enum arm64_smccc_conduit conduit;
  psci_fn invoke_psci_fn;
  uint32_t version;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

uint32_t psci_version(void);
int pcsi_cpu_off(void);
int pcsi_cpu_reset(void);
int pcsi_cpu_on(unsigned long cpuid, uintptr_t entry_point);

#endif /* __ARCH_ARM64_SRC_COMMON_ARM64_CPU_PSCI_H */
