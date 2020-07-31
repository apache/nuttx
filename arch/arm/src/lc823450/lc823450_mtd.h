/****************************************************************************
 * arch/arm/src/lc823450/lc823450_mtd.h
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

#ifndef __ARCH_ARM_SRC_LC823450_LC823450_MTD_H
#define __ARCH_ARM_SRC_LC823450_LC823450_MTD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Partition #1: IPL2
 * Partition #2: IPL2 config
 * Partition #3: recovery kernel
 * Partition #4: normal kernel
 * Partition #5: etc
 * Partition #6: rootfs
 * Partition #7: log
 * Partition #8: db
 * Partition #9: cache
 * Partition #10: contents
 */

#define LC823450_NPARTS         10        /* Number of partitions             */
#define LC823450_PART1_START    0         /* Start sector of partition 1      */
#define LC823450_PART1_NBLOCKS  1024      /* Number of sectors of partition 1 */
#define LC823450_PART2_START    (LC823450_PART1_START + LC823450_PART1_NBLOCKS)
#define LC823450_PART2_NBLOCKS  1024
#define LC823450_PART3_START    (LC823450_PART2_START + LC823450_PART2_NBLOCKS)
#define LC823450_PART3_NBLOCKS  1024
#define LC823450_PART4_START    (LC823450_PART3_START + LC823450_PART3_NBLOCKS)
#define LC823450_PART4_NBLOCKS  1024
#define LC823450_PART5_START    (LC823450_PART4_START + LC823450_PART4_NBLOCKS)
#define LC823450_PART5_NBLOCKS  32768
#define LC823450_PART6_START    (LC823450_PART5_START + LC823450_PART5_NBLOCKS)
#define LC823450_PART6_NBLOCKS  131072
#define LC823450_PART7_START    (LC823450_PART6_START + LC823450_PART6_NBLOCKS)
#define LC823450_PART7_NBLOCKS  32768
#define LC823450_PART8_START    (LC823450_PART7_START + LC823450_PART7_NBLOCKS)
#define LC823450_PART8_NBLOCKS  262144
#define LC823450_PART9_START    (LC823450_PART8_START + LC823450_PART8_NBLOCKS)
#define LC823450_PART9_NBLOCKS  139264
#define LC823450_PART10_START   (LC823450_PART9_START + LC823450_PART9_NBLOCKS)
#define LC823450_PART10_NBLOCKS  0      /* 0 means all remaining sectors     */

#if CONFIG_MTD_CP_STARTBLOCK != LC823450_PART10_START
#  error "Start sector of contents partition mismatched"
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int lc823450_mtd_initialize(uint32_t devno);

#if defined(__cplusplus)
}
#endif
#undef EXTERN

#endif /* __ARCH_ARM_SRC_LC823450_LC823450_MTD_H */
