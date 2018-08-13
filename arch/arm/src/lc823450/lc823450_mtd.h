/****************************************************************************
 * arch/arm/src/lc823450/lc823450_mtd.h
 *
 *   Copyright 2014,2015,2017 Sony Video & Sound Products Inc.
 *   Author: Nobutaka Toyoshima <Nobutaka.Toyoshima@jp.sony.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
#  error "Start sector of contents patrition missmatched"
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
 * Public Functions
 ****************************************************************************/

int lc823450_mtd_initialize(uint32_t devno);

#if defined(__cplusplus)
}
#endif
#undef EXTERN

#endif /* __ARCH_ARM_SRC_LC823450_LC823450_MTD_H */
