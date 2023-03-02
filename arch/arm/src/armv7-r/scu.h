/****************************************************************************
 * arch/arm/src/armv7-r/scu.h
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

/* Reference:
 *   Cortex™-R8 MPCore, Revision: r0p3, Technical Reference Manual.
 */

#ifndef __ARCH_ARM_SRC_ARMV7_R_SCU_H
#define __ARCH_ARM_SRC_ARMV7_R_SCU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "mpcore.h"                         /* For MPCORE_SCU_VBASE */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define SCU_CTRL_OFFSET              0x0000           /* SCU Control Register (Implementation defined) */
#define SCU_CONFIG_OFFSET            0x0004           /* SCU Configuration Register (Implementation defined) */
#define SCU_PWRSTATUS_OFFSET         0x0008           /* SCU CPU Power Status Register */
#define SCU_INVALIDATE_OFFSET        0x000c           /* SCU Invalidate All Registers in Secure State */
#define SCU_FILTERSTART_OFFSET       0x0040           /* Filtering Start Address Register Defined by FILTERSTART input */
#define SCU_FILTEREND_OFFSET         0x0044           /* Filtering End Address Register Defined by FILTEREND input */
#define SCU_PFILTERSTART_OFFSET      0x0048           /* Peripherals Filtering Start Address Register */
#define SCU_PFILTEREND_OFFSET        0x004c           /* Peripherals Filtering End Address Register */
#define SCU_SAC_OFFSET               0x0050           /* SCU Access Control (SAC) Register */
#define SCU_ERRBANKFST_OFFSET        0x0060           /* SCU Error Bank First Entry Register */
#define SCU_ERRBANKSND_OFFSET        0x0064           /* SCU Error Bank Second Entry Register */
#define SCU_DEBUGRAM_OFFSET          0x0070           /* SCU Debug Tag RAM Operation Register */
#define SCU_DEBUGRAMDATA_OFFSET      0x0074           /* SCU Debug Tag RAM Data Value Register */
#define SCU_DEBUGRAMECC_OFFSET       0x0078           /* SCU Debug Tag RAM ECC Chunk Register */
#define SCU_ECCERR_OFFSET            0x007c           /* ECC Fatal Error Registe */
#define SCU_FPPFILTERSTART_OFFSET(n) (0x0080 + (n)*8) /* FPP Filtering Start Address Register for core n */
#define SCU_FPPFILTEREND_OFFSET(n)   (0x0084 + (n)*8) /* FPP Filtering End Address Register for core n */

/* Register addresses *******************************************************/

#define SCU_CTRL                     (MPCORE_SCU_VBASE+SCU_CTRL_OFFSET)
#define SCU_CONFIG                   (MPCORE_SCU_VBASE+SCU_CONFIG_OFFSET)
#define SCU_PWRSTATUS                (MPCORE_SCU_VBASE+SCU_PWRSTATUS_OFFSET)
#define SCU_INVALIDATE               (MPCORE_SCU_VBASE+SCU_INVALIDATE_OFFSET)
#define SCU_FILTERSTART              (MPCORE_SCU_VBASE+SCU_FILTERSTART_OFFSET)
#define SCU_FILTEREND                (MPCORE_SCU_VBASE+SCU_FILTEREND_OFFSET)
#define SCU_PFILTERSTART             (MPCORE_SCU_VBASE+SCU_PFILTERSTART_OFFSET)
#define SCU_PFILTEREND               (MPCORE_SCU_VBASE+SCU_PFILTEREND_OFFSET)
#define SCU_SAC                      (MPCORE_SCU_VBASE+SCU_SAC_OFFSET)
#define SCU_ERRBANKFST               (MPCORE_SCU_VBASE+SCU_ERRBANKFST_OFFSET)
#define SCU_ERRBANKSND               (MPCORE_SCU_VBASE+SCU_ERRBANKSND_OFFSET)
#define SCU_DEBUGRAM                 (MPCORE_SCU_VBASE+SCU_DEBUGRAM_OFFSET)
#define SCU_DEBUGRAMDATA             (MPCORE_SCU_VBASE+SCU_DEBUGRAMDATA_OFFSET)
#define SCU_DEBUGRAMECC              (MPCORE_SCU_VBASE+SCU_DEBUGRAMECC_OFFSET)
#define SCU_ECCERR                   (MPCORE_SCU_VBASE+SCU_ECCERR_OFFSET)
#define SCU_FPPFILTERSTART(n)        (MPCORE_SCU_VBASE+SCU_FPPFILTERSTART0_OFFSET(n))
#define SCU_FPPFILTEREND(n)          (MPCORE_SCU_VBASE+SCU_FPPFILTEREND0_OFFSET(n))

/* Register bit-field definitions *******************************************/

/* SCU Control Register (Implementation defined) */

#define SCU_CTRL_ENABLE              (1 << 0)         /* SCU enable */
#define SCU_CTRL_ADDRFILTER          (1 << 1)         /* Address filtering enable */
#define SCU_CTRL_RAMPARITY           (1 << 2)         /* SCU RAMs ECC enable */
#define SCU_CTRL_LINFILL             (1 << 3)         /* SCU speculative linefill enable */
#define SCU_CTRL_STANDBY             (1 << 5)         /* SCU standby enable */
#define SCU_CTRL_ICSTANDBY           (1 << 6)         /* IC standby enable */
#define SCU_CTRL_ECCCHKEN_M0         (1 << 12)        /* ECC check enable on M0 */
#define SCU_CTRL_ECCCHKEN_M1         (1 << 13)        /* ECC check enable on M1 */
#define SCU_CTRL_ECCCHKEN_MP         (1 << 14)        /* ECC check enable on MP */
#define SCU_CTRL_ECCCHKEN_ACP        (1 << 15)        /* ECC check enable on ACP */
#define SCU_CTRL_ECCCHKEN_FPP(n)     (1 << ((n)+16))  /* ECC check enable on FPP for core n */
#define SCU_CTRL_ECCCHKEN_TCM        (1 << 20)        /* ECC check enable on AXI TCM */

/* SCU Configuration Register (Implementation defined) */

#define SCU_CONFIG_NCPUS_SHIFT       0         /* CPU number Number of CPUs present */
#define SCU_CONFIG_NCPUS_MASK        (3 << SCU_CONFIG_NCPUS_SHIFT)
#  define SCU_CONFIG_NCPUS(r)        ((((uint32_t)(r) &  SCU_CONFIG_NCPUS_MASK) >> SCU_CONFIG_NCPUS_SHIFT) + 1)
#define SCU_CONFIG_SMPCPUS_SHIFT     4         /* Processors that are in SMP or AMP mode */
#define SCU_CONFIG_SMPCPUS_MASK      (15 << SCU_CONFIG_SMPCPUS_SHIFT)
#  define SCU_CONFIG_CPU_SMP(n)      (1 << ((n)+4))
#  define SCU_CONFIG_CPU0_SMP        (1 << 4)
#  define SCU_CONFIG_CPU1_SMP        (1 << 5)
#  define SCU_CONFIG_CPU2_SMP        (1 << 6)
#  define SCU_CONFIG_CPU3_SMP        (1 << 7)

#define SCU_CONFIG_CACHE_0KB         0
#define SCU_CONFIG_CACHE_4KB         1
#define SCU_CONFIG_CACHE_8KB         2
#define SCU_CONFIG_CACHE_16KB        3
#define SCU_CONFIG_CACHE_32KB        4
#define SCU_CONFIG_CACHE_64KB        5

#define SCU_CONFIG_CPU0_CACHE_SHIFT  8         /* CPU 0 cache size */
#define SCU_CONFIG_CPU0_CACHE_MASK   (4 << SCU_CONFIG_CPU0_CACHE_SHIFT)
#define SCU_CONFIG_CPU1_CACHE_SHIFT  12        /* CPU 1 cache size */
#define SCU_CONFIG_CPU1_CACHE_MASK   (4 << SCU_CONFIG_CPU1_CACHE_SHIFT)
#define SCU_CONFIG_CPU2_CACHE_SHIFT  16        /* CPU 2 cache size */
#define SCU_CONFIG_CPU2_CACHE_MASK   (4 << SCU_CONFIG_CPU2_CACHE_SHIFT)
#define SCU_CONFIG_CPU3_CACHE_SHIFT  20        /* CPU 3 cache size */
#define SCU_CONFIG_CPU3_CACHE_MASK   (4 << SCU_CONFIG_CPU3_CACHE_SHIFT)

#define SCU_CONFIG_AXI_PORT1_SHIFT   31

/* SCU CPU Power Status Register */

#define SCU_PWRSTATUS_NORMAL         0
#define SCU_PWRSTATUS_DORMANT        2
#define SCU_PWRSTATUS_PWROFF         3

#define SCU_PWRSTATUS_CPU0_SHIFT     0         /* CPU0 status Power status */
#define SCU_PWRSTATUS_CPU0_MASK      (3 << SCU_PWRSTATUS_CPU0_SHIFT)
#define SCU_PWRSTATUS_CPU1_SHIFT     8         /* CPU1 status Power status */
#define SCU_PWRSTATUS_CPU1_MASK      (3 << SCU_PWRSTATUS_CPU1_SHIFT)
#define SCU_PWRSTATUS_CPU2_SHIFT     16        /* CPU2 status Power status */
#define SCU_PWRSTATUS_CPU2_MASK      (3 << SCU_PWRSTATUS_CPU2_SHIFT)
#define SCU_PWRSTATUS_CPU3_SHIFT     24        /* CPU3 status Power status */
#define SCU_PWRSTATUS_CPU3_MASK      (3 << SCU_PWRSTATUS_CPU3_SHIFT)

/* SCU Invalidate All Registers in Secure State */

#define SCU_INVALIDATE_ALL_WAYS      15
#define SCU_INVALIDATE_CPU0_SHIFT    0         /* Ways that must be invalidated for CPU0 */
#define SCU_INVALIDATE_CPU0_MASK     (15 << SCU_INVALIDATE_CPU0_SHIFT)
#define SCU_INVALIDATE_CPU1_SHIFT    4         /* Ways that must be invalidated for CPU1 */
#define SCU_INVALIDATE_CPU1_MASK     (15 << SCU_INVALIDATE_CPU1_SHIFT)
#define SCU_INVALIDATE_CPU2_SHIFT    8         /* Ways that must be invalidated for CPU2 */
#define SCU_INVALIDATE_CPU2_MASK     (15 << SCU_INVALIDATE_CPU2_SHIFT)
#define SCU_INVALIDATE_CPU3_SHIFT    12        /* Ways that must be invalidated for CPU3 */
#define SCU_INVALIDATE_CPU3_MASK     (15 << SCU_INVALIDATE_CPU3_SHIFT)

/* Filtering Start Address Register Defined by FILTERSTART input */

#define SCU_FILTERSTART_SHIFT        20        /* Filtering start address */
#define SCU_FILTERSTART_MASK         (0xfff << SCU_FILTERSTART_SHIFT)

/* Filtering End Address Register Defined by FILTEREND input */

#define SCU_FILTEREND_SHIFT          20        /* Filtering start address */
#define SCU_FILTEREND_MASK           (0xfff << SCU_FILTEREND_SHIFT)

/* LLP Filtering Start Address Register */

#define SCU_LLPFILTERSTART_SHIFT     20
#define SCU_LLPFILTERSTART_MASK      (0xfff << SCU_LLPFILTERSTART_SHIFT)

/* LLP Filtering End Address Register */

#define SCU_LLPFILTEREND_SHIFT       20
#define SCU_LLPFILTEREND_MASK        (0xfff << SCU_LLPFILTEREND_SHIFT)

/* SCU Access Control (SAC) Register */

#define SCU_SAC_CPU(n)               (1 << (n)) /* CPUn may access components */

/* SCU Error Bank First Entry Register */

#define SCU_ERRBANKFST_STATUS_SHIFT  0
#define SCU_ERRBANKFST_STATUS_MASK   (3 << SCU_ERRBANKFST_STATUS_SHIFT)

#define SCU_ERRBANKFST_WAYS_SHIFT(n) (16 + (n)*4)
#define SCU_ERRBANKFST_WAYS_MASK(n)  (0xf << SCU_ERRBANKFST_WAYS_SHIFT(n))

/* SCU Error Bank Second Entry Register */

#define SCU_ERRBANKSND_STATUS_SHIFT  0
#define SCU_ERRBANKSND_STATUS_MASK   (3 << SCU_ERRBANKSND_STATUS_SHIFT)

#define SCU_ERRBANKSND_INDEX_SHIFT   5
#define SCU_ERRBANKSND_INDEX_MASK    (0x1ff << SCU_ERRBANKSND_INDEX_SHIFT)

#define SCU_ERRBANKSND_WAYS_SHIFT    16
#define SCU_ERRBANKSND_WAYS_MASK     (0xffff << SCU_ERRBANKSND_WAYS_SHIFT)

/* SCU Debug Tag RAM Operation Register */

#define SCU_DEBUGRAM_READ            0
#define SCU_DEBUGRAM_WRITE           1

#define SCU_DEBUGRAM_INDEX_SHIFT     5
#define SCU_DEBUGRAM_INDEX_MASK      (0x1ff << SCU_DEBUGRAM_INDEX_SHIFT)

#define SCU_DEBUGRAM_CORE_SHIFT      24
#define SCU_DEBUGRAM_CORE_MASK       (3 << SCU_DEBUGRAM_CORE_SHIFT)

#define SCU_DEBUGRAM_WAY_SHIFT       30
#define SCU_DEBUGRAM_WAY_MASK        (3 << SCU_DEBUGRAM_WAY_SHIFT)

/* SCU Debug Tag RAM Data Value Register */

#define SCU_DEBUGRAMDATA_VALUE_SHIFT 17
#define SCU_DEBUGRAMDATA_VALUE_MASK  (0x1f << SCU_DEBUGRAMDATA_VALUE_SHIFT)

#define SCU_DEBUGRAMDATA_VALID       (1 << 22)

/* SCU Debug Tag RAM ECC Chunk Register */

#define SCU_DEBUGRAMECC_CHUNK_SHIFT  0
#define SCU_DEBUGRAMECC_CHUNK_MASK   (0x3f << SCU_DEBUGRAMECC_CHUNK_SHIFT)

/* ECC Fatal Error Register */

#define SCU_ECCERR_CORE_DETECTED(n)  (n)
#define SCU_ECCERR_DETECTED          (1 << 8)

/* FPP Filtering Start Address Registers 0-3 */

#define SCU_FPPFILTERSTART_SHIFT     20
#define SCU_FPPFILTERSTART_MASK      (0xfff << SCU_FPPFILTERSTART_SHIFT)

/* FPP Filtering End Address Registers 0-3 */

#define SCU_FPPFILTEREND_SHIFT       20
#define SCU_FPPFILTEREND_MASK        (0xfff << SCU_FPPFILTEREND_SHIFT)

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: arm_enable_smp
 *
 * Description:
 *   Enable the SCU and make certain that current CPU is participating in
 *   the SMP cache coherency.
 *
 ****************************************************************************/

void arm_enable_smp(int cpu);

#endif /* __ARCH_ARM_SRC_ARMV7_R_SCU_H */
