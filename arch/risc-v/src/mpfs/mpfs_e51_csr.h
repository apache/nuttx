/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_e51_csr.h
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

#ifndef __ARCH_RISCV_SRC_MPFS_MPFS_E51_CSR_H
#define __ARCH_RISCV_SRC_MPFS_MPFS_E51_CSR_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/*  Microchip PolarFire SoC E51 – Implementation-Specific CSRs
 *  Address range: 0x7C0 – 0x7CF
 *  NOTE: Not part of standard RISC-V spec. Valid only on E51 core
 */

#define CSR_E51_MCACHE_CTL        0x7c0  /* Controls enable/disable of L1 instruction and data caches      */
#define CSR_E51_MCACHE_CFG        0x7c1  /* Read-only: cache configuration (line size, sets, ways)         */
#define CSR_E51_MCACHE_FLUSH      0x7c2  /* Write-only: flush or invalidate instruction/data caches        */
#define CSR_E51_MCACHE_HIT_COUNT  0x7c3  /* Read-only: cache hit counter (performance profiling)           */
#define CSR_E51_MCACHE_MISS_COUNT 0x7c4  /* Read-only: cache miss counter (performance profiling)          */
#define CSR_E51_MCACHE_WAYS       0x7c5  /* Read-only: implemented number of cache ways (I & D)            */
#define CSR_E51_MCACHE_LINE_LOCK  0x7c6  /* Write-only: lock/unlock specific cache lines                   */

#define CSR_E51_MMPU_CFG          0x7c7  /* Memory Protection Unit configuration register                  */
#define CSR_E51_MMPU_BASE         0x7c8  /* MPU region base address register                               */
#define CSR_E51_MMPU_END          0x7c9  /* MPU region end address register                                */
#define CSR_E51_MMPU_CTRL         0x7ca  /* MPU region enable/control and access attributes                */

#define CSR_E51_MIMPLID           0x7cc  /* Implementation ID (Microchip-specific ID register)             */
#define CSR_E51_MPRIV_CFG         0x7cd  /* Privilege configuration and trap routing                       */
#define CSR_E51_MSOFTINT          0x7ce  /* Machine software interrupt pending/clear                       */
#define CSR_E51_MRESET_CAUSE      0x7cf  /* Read-only: indicates cause of last reset (POR, watchdog, etc.) */

/* Common helper bitfields for CSR_MCACHE_CTL */

#define E51_MCACHE_CTL_DCACHE_EN  (1U << 0)  /* Enable data cache            */
#define E51_MCACHE_CTL_ICACHE_EN  (1U << 1)  /* Enable instruction cache     */

/* Common helper bitfields for CSR_MCACHE_FLUSH */

#define E51_MCACHE_FLUSH_DCACHE   (1U << 0)  /* Flush data cache             */
#define E51_MCACHE_FLUSH_ICACHE   (1U << 1)  /* Flush instruction cache      */
#define E51_MCACHE_FLUSH_BOTH     (E51_MCACHE_FLUSH_DCACHE | E51_MCACHE_FLUSH_ICACHE)

#endif /* __ARCH_RISCV_SRC_MPFS_MPFS_E51_CSR_H */
