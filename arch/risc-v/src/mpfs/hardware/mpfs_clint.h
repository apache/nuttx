/****************************************************************************
 * arch/risc-v/src/mpfs/hardware/mpfs_clint.h
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

#ifndef __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_CLINT_H
#define __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_CLINT_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MPFS_CLINT_MSIP0     (MPFS_CLINT_BASE + 0x0000)
#define MPFS_CLINT_MSIP1     (MPFS_CLINT_BASE + 0x0004)
#define MPFS_CLINT_MSIP2     (MPFS_CLINT_BASE + 0x0008)
#define MPFS_CLINT_MSIP3     (MPFS_CLINT_BASE + 0x000C)
#define MPFS_CLINT_MSIP4     (MPFS_CLINT_BASE + 0x0010)

#define MPFS_CLINT_MTIMECMP0 (MPFS_CLINT_BASE + 0x4000)
#define MPFS_CLINT_MTIMECMP1 (MPFS_CLINT_BASE + 0x4008)
#define MPFS_CLINT_MTIMECMP2 (MPFS_CLINT_BASE + 0x4010)
#define MPFS_CLINT_MTIMECMP3 (MPFS_CLINT_BASE + 0x4018)
#define MPFS_CLINT_MTIMECMP4 (MPFS_CLINT_BASE + 0x4020)

#define MPFS_CLINT_MTIME     (MPFS_CLINT_BASE + 0xbff8)

#endif /* __ARCH_RISCV_SRC_MPFS_HARDWARE_MPFS_CLINT_H */
