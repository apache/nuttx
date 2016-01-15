/************************************************************************************
 * arch/arm/src/samv7/chip/sam_tc.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMV7_CHIP_SAM_TC_H
#define __ARCH_ARM_SRC_SAMV7_CHIP_SAM_TC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip/sam_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define SAM_TC_NCHANNELS         3          /* Number of channels per TC peripheral */

/* TC Register Offsets **************************************************************/

#define SAM_TC_CHAN_OFFSET(n)    ((n) << 6) /* Channel n offset */
#define SAM_TC_CCR_OFFSET        0x0000     /* Channel Control Register */
#define SAM_TC_CMR_OFFSET        0x0004     /* Channel Mode Register */
#define SAM_TC_SMMR_OFFSET       0x0008     /* Stepper Motor Mode Register */
#define SAM_TC_RAB_OFFSET        0x000c     /* Register AB */
#define SAM_TC_CV_OFFSET         0x0010     /* Counter Value */
#define SAM_TC_RA_OFFSET         0x0014     /* Register A */
#define SAM_TC_RB_OFFSET         0x0018     /* Register B */
#define SAM_TC_RC_OFFSET         0x001c     /* Register C */
#define SAM_TC_SR_OFFSET         0x0020     /* Status Register */
#define SAM_TC_IER_OFFSET        0x0024     /* Interrupt Enable Register */
#define SAM_TC_IDR_OFFSET        0x0028     /* Interrupt Disable Register */
#define SAM_TC_IMR_OFFSET        0x002c     /* Interrupt Mask Register */
#define SAM_TC_EMR_OFFSET        0x0030     /* Extended Mode Register */

#define SAM_TCn_CCR_OFFSET(n)    (SAM_TC_CHAN_OFFSET(n)+SAM_TC_CCR_OFFSET)
#define SAM_TCn_CMR_OFFSET(n)    (SAM_TC_CHAN_OFFSET(n)+SAM_TC_CMR_OFFSET)
#define SAM_TCn_SMMR_OFFSET(n)   (SAM_TC_CHAN_OFFSET(n)+SAM_TC_SMMR_OFFSET)
#define SAM_TCn_RAB_OFFSET(n)    (SAM_TC_CHAN_OFFSET(n)+SAM_TC_RAB_OFFSET)
#define SAM_TCn_CV_OFFSET(n)     (SAM_TC_CHAN_OFFSET(n)+SAM_TC_CV_OFFSET)
#define SAM_TCn_RA_OFFSET(n)     (SAM_TC_CHAN_OFFSET(n)+SAM_TC_RA_OFFSET)
#define SAM_TCn_RB_OFFSET(n)     (SAM_TC_CHAN_OFFSET(n)+SAM_TC_RB_OFFSET)
#define SAM_TCn_RC_OFFSET(n)     (SAM_TC_CHAN_OFFSET(n)+SAM_TC_RC_OFFSET)
#define SAM_TCn_SR_OFFSET(n)     (SAM_TC_CHAN_OFFSET(n)+SAM_TC_SR_OFFSET)
#define SAM_TCn_IER_OFFSET(n)    (SAM_TC_CHAN_OFFSET(n)+SAM_TC_IER_OFFSET)
#define SAM_TCn_IDR_OFFSET(n)    (SAM_TC_CHAN_OFFSET(n)+SAM_TC_IDR_OFFSET)
#define SAM_TCn_IMR_OFFSET(n)    (SAM_TC_CHAN_OFFSET(n)+SAM_TC_IMR_OFFSET)
#define SAM_TCn_EMR_OFFSET(n)    (SAM_TC_CHAN_OFFSET(n)+SAM_TC_EMR_OFFSET)

#define SAM_TC0_CCR_OFFSET       SAM_TCn_CCR_OFFSET(0)
#define SAM_TC0_CMR_OFFSET       SAM_TCn_CMR_OFFSET(0)
#define SAM_TC0_SMMR_OFFSET      SAM_TCn_SMMR_OFFSET(0)
#define SAM_TC0_RAB_OFFSET       SAM_TCn_RAB_OFFSET(0)
#define SAM_TC0_CV_OFFSET        SAM_TCn_CV_OFFSET(0)
#define SAM_TC0_RA_OFFSET        SAM_TCn_RA_OFFSET(0)
#define SAM_TC0_RB_OFFSET        SAM_TCn_RB_OFFSET(0)
#define SAM_TC0_RC_OFFSET        SAM_TCn_RC_OFFSET(0)
#define SAM_TC0_SR_OFFSET        SAM_TCn_SR_OFFSET(0)
#define SAM_TC0_IER_OFFSET       SAM_TCn_IER_OFFSET(0)
#define SAM_TC0_IDR_OFFSET       SAM_TCn_IDR_OFFSET(0)
#define SAM_TC0_IMR_OFFSET       SAM_TCn_IMR_OFFSET(0)
#define SAM_TC0_EMR_OFFSET       SAM_TCn_EMR_OFFSET(0)

#define SAM_TC1_CCR_OFFSET       SAM_TCn_CCR_OFFSET(1)
#define SAM_TC1_CMR_OFFSET       SAM_TCn_CMR_OFFSET(1)
#define SAM_TC1_SMMR_OFFSET      SAM_TCn_SMMR_OFFSET(1)
#define SAM_TC1_RAB_OFFSET       SAM_TCn_RAB_OFFSET(1)
#define SAM_TC1_CV_OFFSET        SAM_TCn_CV_OFFSET(1)
#define SAM_TC1_RA_OFFSET        SAM_TCn_RA_OFFSET(1)
#define SAM_TC1_RB_OFFSET        SAM_TCn_RB_OFFSET(1)
#define SAM_TC1_RC_OFFSET        SAM_TCn_RC_OFFSET(1)
#define SAM_TC1_SR_OFFSET        SAM_TCn_SR_OFFSET(1)
#define SAM_TC1_IER_OFFSET       SAM_TCn_IER_OFFSET(1)
#define SAM_TC1_IDR_OFFSET       SAM_TCn_IDR_OFFSET(1)
#define SAM_TC1_IMR_OFFSET       SAM_TCn_IMR_OFFSET(1)
#define SAM_TC1_EMR_OFFSET       SAM_TCn_EMR_OFFSET(1)

#define SAM_TC2_CCR_OFFSET       SAM_TCn_CCR_OFFSET(2)
#define SAM_TC2_CMR_OFFSET       SAM_TCn_CMR_OFFSET(2)
#define SAM_TC2_SMMR_OFFSET      SAM_TCn_SMMR_OFFSET(2)
#define SAM_TC2_RAB_OFFSET       SAM_TCn_RAB_OFFSET(2)
#define SAM_TC2_CV_OFFSET        SAM_TCn_CV_OFFSET(2)
#define SAM_TC2_RA_OFFSET        SAM_TCn_RA_OFFSET(2)
#define SAM_TC2_RB_OFFSET        SAM_TCn_RB_OFFSET(2)
#define SAM_TC2_RC_OFFSET        SAM_TCn_RC_OFFSET(2)
#define SAM_TC2_SR_OFFSET        SAM_TCn_SR_OFFSET(2)
#define SAM_TC2_IER_OFFSET       SAM_TCn_IER_OFFSET(2)
#define SAM_TC2_IDR_OFFSET       SAM_TCn_IDR_OFFSET(2)
#define SAM_TC2_IMR_OFFSET       SAM_TCn_IMR_OFFSET(2)
#define SAM_TC2_EMR_OFFSET       SAM_TCn_EMR_OFFSET(2)

#define SAM_TC_BCR_OFFSET        0x00c0     /* Block Control Register */
#define SAM_TC_BMR_OFFSET        0x00c4     /* Block Mode Register */
#define SAM_TC_QIER_OFFSET       0x00c8     /* QDEC Interrupt Enable Register */
#define SAM_TC_QIDR_OFFSET       0x00cc     /* QDEC Interrupt Disable Register */
#define SAM_TC_QIMR_OFFSET       0x00d0     /* QDEC Interrupt Mask Register */
#define SAM_TC_QISR_OFFSET       0x00d4     /* QDEC Interrupt Status Register */
#define SAM_TC_FMR_OFFSET        0x00d8     /* Fault Mode Register */
#define SAM_TC_WPMR_OFFSET       0x00e4     /* Write Protect Mode Register */

/* TC Register Addresses ************************************************************/

#define SAM_TC012_CHAN_BASE(n)   (SAM_TC012_BASE+SAM_TC_CHAN_OFFSET(n))

#define SAM_TC012_CCR(n)         (SAM_TC012_BASE+SAM_TCn_CCR_OFFSET(n))
#define SAM_TC012_CMR(n)         (SAM_TC012_BASE+SAM_TCn_CMR_OFFSET(n))
#define SAM_TC012_SMMR(n)        (SAM_TC012_BASE+SAM_TCn_SMMR_OFFSET(n))
#define SAM_TC012_RAB(n)         (SAM_TC012_BASE+SAM_TCn_RAB_OFFSET(n))
#define SAM_TC012_CV(n)          (SAM_TC012_BASE+SAM_TCn_CV_OFFSET(n))
#define SAM_TC012_RA(n)          (SAM_TC012_BASE+SAM_TCn_RA_OFFSET(n))
#define SAM_TC012_RB(n)          (SAM_TC012_BASE+SAM_TCn_RB(n))
#define SAM_TC012_RC(n)          (SAM_TC012_BASE+SAM_TCn_RC_OFFSET(n))
#define SAM_TC012_SR(n)          (SAM_TC012_BASE+SAM_TCn_SR_OFFSET(n))
#define SAM_TC012_IER(n)         (SAM_TC012_BASE+SAM_TCn_IER_OFFSET(n))
#define SAM_TC012_IDR(n)         (SAM_TC012_BASE+SAM_TCn_IDR_OFFSET(n))
#define SAM_TC012_IMR(n)         (SAM_TC012_BASE+SAM_TCn_IMR_OFFSET(n))
#define SAM_TC012_EMR(n)         (SAM_TC012_BASE+SAM_TCn_EMR_OFFSET(n))

#define SAM_TC0_CCR              SAM_TC012_CCR(0)
#define SAM_TC0_CMR              SAM_TC012_CMR(0)
#define SAM_TC0_SMMR             SAM_TC012_SMMR(0)
#define SAM_TC0_RAB              SAM_TC012_RAB(0)
#define SAM_TC0_CV               SAM_TC012_CV(0)
#define SAM_TC0_RA               SAM_TC012_RA(0)
#define SAM_TC0_RB               SAM_TC012_RB(0)
#define SAM_TC0_RC               SAM_TC012_RC(0)
#define SAM_TC0_SR               SAM_TC012_SR(0)
#define SAM_TC0_IER              SAM_TC012_IER(0)
#define SAM_TC0_IDR              SAM_TC012_IDR(0)
#define SAM_TC0_IMR              SAM_TC012_IMR(0)
#define SAM_TC0_EMR              SAM_TC012_EMR(0)

#define SAM_TC1_CCR              SAM_TC012_CCR(1)
#define SAM_TC1_CMR              SAM_TC012_CMR(1)
#define SAM_TC1_SMMR             SAM_TC012_SMMR(1)
#define SAM_TC1_RAB              SAM_TC012_RAB(1)
#define SAM_TC1_CV               SAM_TC012_CV(1)
#define SAM_TC1_RA               SAM_TC012_RA(1)
#define SAM_TC1_RB               SAM_TC012_RB(1)
#define SAM_TC1_RC               SAM_TC012_RC(1)
#define SAM_TC1_SR               SAM_TC012_SR(1)
#define SAM_TC1_IER              SAM_TC012_IER(1)
#define SAM_TC1_IDR              SAM_TC012_IDR(1)
#define SAM_TC1_IMR              SAM_TC012_IMR(1)
#define SAM_TC1_EMR              SAM_TC012_EMR(1)

#define SAM_TC2_CCR              SAM_TC012_CCR(2)
#define SAM_TC2_CMR              SAM_TC012_CMR(2)
#define SAM_TC2_SMMR             SAM_TC012_SMMR(2)
#define SAM_TC2_RAB              SAM_TC012_RAB(2)
#define SAM_TC2_CV               SAM_TC012_CV(2)
#define SAM_TC2_RA               SAM_TC012_RA(2)
#define SAM_TC2_RB               SAM_TC012_RB(2)
#define SAM_TC2_RC               SAM_TC012_RC(2)
#define SAM_TC2_SR               SAM_TC012_SR(2)
#define SAM_TC2_IER              SAM_TC012_IER(2)
#define SAM_TC2_IDR              SAM_TC012_IDR(2)
#define SAM_TC2_IMR              SAM_TC012_IMR(2)
#define SAM_TC2_EMR              SAM_TC012_EMR(2)

#define SAM_TC012_BCR            (SAM_TC012_BASE+SAM_TC_BCR_OFFSET)
#define SAM_TC012_BMR            (SAM_TC012_BASE+SAM_TC_BMR_OFFSET)
#define SAM_TC012_QIER           (SAM_TC012_BASE+SAM_TC_QIER_OFFSET)
#define SAM_TC012_QIDR           (SAM_TC012_BASE+SAM_TC_QIDR_OFFSET)
#define SAM_TC012_QIMR           (SAM_TC012_BASE+SAM_TC_QIMR_OFFSET)
#define SAM_TC012_QISR           (SAM_TC012_BASE+SAM_TC_QISR_OFFSET)
#define SAM_TC012_FMR            (SAM_TC012_BASE+SAM_TC_FMR_OFFSET)
#define SAM_TC012_WPMR           (SAM_TC012_BASE+SAM_TC_WPMR_OFFSET)

#define SAM_TC345_CHAN_BASE(n)   (SAM_TC345_BASE+SAM_TC_CHAN_OFFSET((n)-3))

#define SAM_TC345_CCR(n)         (SAM_TC345_BASE+SAM_TCn_CCR_OFFSET((n)-3))
#define SAM_TC345_CMR(n)         (SAM_TC345_BASE+SAM_TCn_CMR_OFFSET((n)-3))
#define SAM_TC345_SMMR(n)        (SAM_TC345_BASE+SAM_TCn_SMMR_OFFSET((n)-3))
#define SAM_TC345_RAB(n)         (SAM_TC345_BASE+SAM_TCn_RAB_OFFSET((n)-3))
#define SAM_TC345_CV(n)          (SAM_TC345_BASE+SAM_TCn_CV_OFFSET((n)-3))
#define SAM_TC345_RA(n)          (SAM_TC345_BASE+SAM_TCn_RA_OFFSET((n)-3))
#define SAM_TC345_RB(n)          (SAM_TC345_BASE+SAM_TCn_RB_OFFSET((n)-3))
#define SAM_TC345_RC(n)          (SAM_TC345_BASE+SAM_TCn_RC_OFFSET((n)-3))
#define SAM_TC345_SR(n)          (SAM_TC345_BASE+SAM_TCn_SR_OFFSET((n)-3))
#define SAM_TC345_IER(n)         (SAM_TC345_BASE+SAM_TCn_IER_OFFSET((n)-3))
#define SAM_TC345_IDR(n)         (SAM_TC345_BASE+SAM_TCn_IDR_OFFSET((n)-3))
#define SAM_TC345_IMR(n)         (SAM_TC345_BASE+SAM_TCn_IMR_OFFSET((n)-3))
#define SAM_TC345_EMR(n)         (SAM_TC345_BASE+SAM_TCn_EMR_OFFSET((n)-3))

#define SAM_TC3_CCR              SAM_TC345_CCR(3)
#define SAM_TC3_CMR              SAM_TC345_CMR(3)
#define SAM_TC3_SMMR             SAM_TC345_SMMR(3)
#define SAM_TC3_RAB              SAM_TC345_RAB(3)
#define SAM_TC3_CV               SAM_TC345_CV(3)
#define SAM_TC3_RA               SAM_TC345_RA(3)
#define SAM_TC3_RB               SAM_TC345_RB(3)
#define SAM_TC3_RC               SAM_TC345_RC(3)
#define SAM_TC3_SR               SAM_TC345_SR(3)
#define SAM_TC3_IER              SAM_TC345_IER(3)
#define SAM_TC3_IDR              SAM_TC345_IDR(3)
#define SAM_TC3_IMR              SAM_TC345_IMR(3)
#define SAM_TC3_EMR               SAM_TC345_EMR(3)

#define SAM_TC4_CCR              SAM_TC345_CCR(4)
#define SAM_TC4_CMR              SAM_TC345_CMR(4)
#define SAM_TC4_SMMR             SAM_TC345_SMMR(4)
#define SAM_TC4_RAB              SAM_TC345_RAB(4)
#define SAM_TC4_CV               SAM_TC345_CV(4)
#define SAM_TC4_RA               SAM_TC345_RA(4)
#define SAM_TC4_RB               SAM_TC345_RB(4)
#define SAM_TC4_RC               SAM_TC345_RC(4)
#define SAM_TC4_SR               SAM_TC345_SR(4)
#define SAM_TC4_IER              SAM_TC345_IER(4)
#define SAM_TC4_IDR              SAM_TC345_IDR(4)
#define SAM_TC4_IMR              SAM_TC345_IMR(4)
#define SAM_TC4_EMR              SAM_TC345_EMR(4)

#define SAM_TC5_CCR              SAM_TC345_CCR(5)
#define SAM_TC5_CMR              SAM_TC345_CMR(5)
#define SAM_TC5_SMMR             SAM_TC345_SMMR(5)
#define SAM_TC5_RAB              SAM_TC345_RAB(5)
#define SAM_TC5_CV               SAM_TC345_CV(5)
#define SAM_TC5_RA               SAM_TC345_RA(5)
#define SAM_TC5_RB               SAM_TC345_RB(5)
#define SAM_TC5_RC               SAM_TC345_RC(5)
#define SAM_TC5_SR               SAM_TC345_SR(5)
#define SAM_TC5_IER              SAM_TC345_IER(5)
#define SAM_TC5_IDR              SAM_TC345_IDR(5)
#define SAM_TC5_IMR              SAM_TC345_IMR(5)
#define SAM_TC5_EMR              SAM_345_EMR(5)

#define SAM_TC345_BCR            (SAM_TC345_BASE+SAM_TC_BCR_OFFSET)
#define SAM_TC345_BMR            (SAM_TC345_BASE+SAM_TC_BMR_OFFSET)
#define SAM_TC345_QIER           (SAM_TC345_BASE+SAM_TC_QIER_OFFSET)
#define SAM_TC345_QIDR           (SAM_TC345_BASE+SAM_TC_QIDR_OFFSET)
#define SAM_TC345_QIMR           (SAM_TC345_BASE+SAM_TC_QIMR_OFFSET)
#define SAM_TC345_QISR           (SAM_TC345_BASE+SAM_TC_QISR_OFFSET)
#define SAM_TC345_FMR            (SAM_TC345_BASE+SAM_TC_FMR_OFFSET)
#define SAM_TC345_WPMR           (SAM_TC345_BASE+SAM_TC_WPMR_OFFSET)

#define SAM_TC678_CHAN_BASE(n)   (SAM_TC678_BASE+SAM_TC_CHAN_OFFSET((n)-6))

#define SAM_TC678_CCRn(n)        (SAM_TC678_BASE+SAM_TCn_CCR_OFFSET((n)-6))
#define SAM_TC678_CMR(n)         (SAM_TC678_BASE+SAM_TCn_CMR_OFFSET((n)-6))
#define SAM_TC678_SMMR(n)        (SAM_TC678_BASE+SAM_TCn_SMMR_OFFSET((n)-6))
#define SAM_TC678_RAB(n)         (SAM_TC678_BASE+SAM_TCn_RAB_OFFSET((n)-6))
#define SAM_TC678_CV(n)          (SAM_TC678_BASE+SAM_TCn_CV_OFFSET((n)-6))
#define SAM_TC678_RA(n)          (SAM_TC678_BASE+SAM_TCn_RA_OFFSET((n)-6))
#define SAM_TC678_RB(n)          (SAM_TC678_BASE+SAM_TCn_RB((n)-6))
#define SAM_TC678_RC(n)          (SAM_TC678_BASE+SAM_TCn_RC_OFFSET((n)-6))
#define SAM_TC678_SR(n)          (SAM_TC678_BASE+SAM_TCn_SR_OFFSET((n)-6))
#define SAM_TC678_IER(n)         (SAM_TC678_BASE+SAM_TCn_IER_OFFSET((n)-6))
#define SAM_TC678_IDR(n)         (SAM_TC678_BASE+SAM_TCn_IDR_OFFSET((n)-6))
#define SAM_TC678_IMR(n)         (SAM_TC678_BASE+SAM_TCn_IMR_OFFSET((n)-6))
#define SAM_TC678_EMR(n)         (SAM_TC678_BASE+SAM_TCn_EMR_OFFSET((n)-6))

#define SAM_TC6_CCR              SAM_TC678_CCR(6)
#define SAM_TC6_CMR              SAM_TC678_CMR(6)
#define SAM_TC6_SMMR             SAM_TC678_SMMR(6)
#define SAM_TC6_RAB              SAM_TC678_RAB(6)
#define SAM_TC6_CV               SAM_TC678_CV(6)
#define SAM_TC6_RA               SAM_TC678_RA(6)
#define SAM_TC6_RB               SAM_TC678_RB(6)
#define SAM_TC6_RC               SAM_TC678_RC(6)
#define SAM_TC6_SR               SAM_TC678_SR(6)
#define SAM_TC6_IER              SAM_TC678_IER(6)
#define SAM_TC6_IDR              SAM_TC678_IDR(6)
#define SAM_TC6_IMR              SAM_TC678_IMR(6)
#define SAM_TC6_EMR              SAM_TC678_EMR(6)

#define SAM_TC7_CCR              SAM_TC678_CCR(7)
#define SAM_TC7_CMR              SAM_TC678_CMR(7)
#define SAM_TC7_SMMR             SAM_TC678_SMMR(7)
#define SAM_TC7_RAB              SAM_TC678_RAB(7)
#define SAM_TC7_CV               SAM_TC678_CV(7)
#define SAM_TC7_RA               SAM_TC678_RA(7)
#define SAM_TC7_RB               SAM_TC678_RB(7)
#define SAM_TC7_RC               SAM_TC678_RC(7)
#define SAM_TC7_SR               SAM_TC678_SR(7)
#define SAM_TC7_IER              SAM_TC678_IER(7)
#define SAM_TC7_IDR              SAM_TC678_IDR(7)
#define SAM_TC7_IMR              SAM_TC678_IMR(7)
#define SAM_TC7_EMR              SAM_TC678_EMR(7)

#define SAM_TC8_CCR              SAM_TC678_CCR(8)
#define SAM_TC8_CMR              SAM_TC678_CMR(8)
#define SAM_TC8_SMMR             SAM_TC678_SMMR(8)
#define SAM_TC8_RAB              SAM_TC678_RAB(8)
#define SAM_TC8_CV               SAM_TC678_CV(8)
#define SAM_TC8_RA               SAM_TC678_RA(8)
#define SAM_TC8_RB               SAM_TC678_RB(8)
#define SAM_TC8_RC               SAM_TC678_RC(8)
#define SAM_TC8_SR               SAM_TC678_SR(8)
#define SAM_TC8_IER              SAM_TC678_IER(8)
#define SAM_TC8_IDR              SAM_TC678_IDR(8)
#define SAM_TC8_IMR              SAM_TC678_IMR(8)
#define SAM_TC8_EMR              SAM_TC678_EMR(8)

#define SAM_TC678_BCR            (SAM_TC678_BASE+SAM_TC_BCR_OFFSET)
#define SAM_TC678_BMR            (SAM_TC678_BASE+SAM_TC_BMR_OFFSET)
#define SAM_TC678_QIER           (SAM_TC678_BASE+SAM_TC_QIER_OFFSET)
#define SAM_TC678_QIDR           (SAM_TC678_BASE+SAM_TC_QIDR_OFFSET)
#define SAM_TC678_QIMR           (SAM_TC678_BASE+SAM_TC_QIMR_OFFSET)
#define SAM_TC678_QISR           (SAM_TC678_BASE+SAM_TC_QISR_OFFSET)
#define SAM_TC678_FMR            (SAM_TC678_BASE+SAM_TC_FMR_OFFSET)
#define SAM_TC678_WPMR           (SAM_TC678_BASE+SAM_TC_WPMR_OFFSET)

#define SAM_TC901_CHAN_BASE(n)   (SAM_TC901_BASE+SAM_TC_CHAN_OFFSET((n)-9))

#define SAM_TC901_CCRn(n)        (SAM_TC901_BASE+SAM_TCn_CCR_OFFSET((n)-9))
#define SAM_TC901_CMR(n)         (SAM_TC901_BASE+SAM_TCn_CMR_OFFSET((n)-9))
#define SAM_TC901_SMMR(n)        (SAM_TC901_BASE+SAM_TCn_SMMR_OFFSET((n)-9))
#define SAM_TC901_RAB(n)         (SAM_TC901_BASE+SAM_TCn_RAB_OFFSET((n)-9))
#define SAM_TC901_CV(n)          (SAM_TC901_BASE+SAM_TCn_CV_OFFSET((n)-9))
#define SAM_TC901_RA(n)          (SAM_TC901_BASE+SAM_TCn_RA_OFFSET((n)-9))
#define SAM_TC901_RB(n)          (SAM_TC901_BASE+SAM_TCn_RB((n)-9))
#define SAM_TC901_RC(n)          (SAM_TC901_BASE+SAM_TCn_RC_OFFSET((n)-9))
#define SAM_TC901_SR(n)          (SAM_TC901_BASE+SAM_TCn_SR_OFFSET((n)-9))
#define SAM_TC901_IER(n)         (SAM_TC901_BASE+SAM_TCn_IER_OFFSET((n)-9))
#define SAM_TC901_IDR(n)         (SAM_TC901_BASE+SAM_TCn_IDR_OFFSET((n)-9))
#define SAM_TC901_IMR(n)         (SAM_TC901_BASE+SAM_TCn_IMR_OFFSET((n)-9))
#define SAM_TC901_EMR(n)         (SAM_TC901_BASE+SAM_TCn_EMR_OFFSET((n)-9))

#define SAM_TC9_CCR              SAM_TC901_CCR(9)
#define SAM_TC9_CMR              SAM_TC901_CMR(9)
#define SAM_TC9_SMMR             SAM_TC901_SMMR(9)
#define SAM_TC9_RAB              SAM_TC901_RAB(9)
#define SAM_TC9_CV               SAM_TC901_CV(9)
#define SAM_TC9_RA               SAM_TC901_RA(9)
#define SAM_TC9_RB               SAM_TC901_RB(9)
#define SAM_TC9_RC               SAM_TC901_RC(9)
#define SAM_TC9_SR               SAM_TC901_SR(9)
#define SAM_TC9_IER              SAM_TC901_IER(9)
#define SAM_TC9_IDR              SAM_TC901_IDR(9)
#define SAM_TC9_IMR              SAM_TC901_IMR(9)
#define SAM_TC9_EMR              SAM_TC901_EMR(9)

#define SAM_TC10_CCR             SAM_TC901_CCR(10)
#define SAM_TC10_CMR             SAM_TC901_CMR(10)
#define SAM_TC10_SMMR            SAM_TC901_SMMR(10)
#define SAM_TC10_RAB             SAM_TC901_RAB(10)
#define SAM_TC10_CV              SAM_TC901_CV(10)
#define SAM_TC10_RA              SAM_TC901_RA(10)
#define SAM_TC10_RB              SAM_TC901_RB(10)
#define SAM_TC10_RC              SAM_TC901_RC(10)
#define SAM_TC10_SR              SAM_TC901_SR(10)
#define SAM_TC10_IER             SAM_TC901_IER(10)
#define SAM_TC10_IDR             SAM_TC901_IDR(10)
#define SAM_TC10_IMR             SAM_TC901_IMR(10)
#define SAM_TC10_EMR             SAM_TC901_EMR(10)

#define SAM_TC11_CCR             SAM_TC901_CCR(11)
#define SAM_TC11_CMR             SAM_TC901_CMR(11)
#define SAM_TC11_SMMR            SAM_TC901_SMMR(11)
#define SAM_TC11_RAB             SAM_TC901_RAB(11)
#define SAM_TC11_CV              SAM_TC901_CV(11)
#define SAM_TC11_RA              SAM_TC901_RA(11)
#define SAM_TC11_RB              SAM_TC901_RB(11)
#define SAM_TC11_RC              SAM_TC901_RC(11)
#define SAM_TC11_SR              SAM_TC901_SR(11)
#define SAM_TC11_IER             SAM_TC901_IER(11)
#define SAM_TC11_IDR             SAM_TC901_IDR(11)
#define SAM_TC11_IMR             SAM_TC901_IMR(11)
#define SAM_TC11_EMR             SAM_TC901_EMR(11)

#define SAM_TC901_BCR            (SAM_TC901_BASE+SAM_TC_BCR_OFFSET)
#define SAM_TC901_BMR            (SAM_TC901_BASE+SAM_TC_BMR_OFFSET)
#define SAM_TC901_QIER           (SAM_TC901_BASE+SAM_TC_QIER_OFFSET)
#define SAM_TC901_QIDR           (SAM_TC901_BASE+SAM_TC_QIDR_OFFSET)
#define SAM_TC901_QIMR           (SAM_TC901_BASE+SAM_TC_QIMR_OFFSET)
#define SAM_TC901_QISR           (SAM_TC901_BASE+SAM_TC_QISR_OFFSET)
#define SAM_TC901_FMR            (SAM_TC901_BASE+SAM_TC_FMR_OFFSET)
#define SAM_TC901_WPMR           (SAM_TC901_BASE+SAM_TC_WPMR_OFFSET)

/* TC Register Bit Definitions ******************************************************/

/* Channel Control Register */

#define TC_CCR_CLKEN             (1 << 0)  /* Bit 0:  Counter Clock Enable Command */
#define TC_CCR_CLKDIS            (1 << 1)  /* Bit 1:  Counter Clock Disable Command */
#define TC_CCR_SWTRG             (1 << 2)  /* Bit 2:  Software Trigger Command */

/* Channel Mode Register -- All modes */

#define TC_CMR_TCCLKS_SHIFT      (0)       /* Bits 0-2: Clock Selection */
#define TC_CMR_TCCLKS_MASK       (7 << TC_CMR_TCCLKS_SHIFT)
#  define TC_CMR_TCCLKS(n)       ((uint32_t)(n) << TC_CMR_TCCLKS_SHIFT)
#  define TC_CMR_TCCLKS_TCLK1    (0 << TC_CMR_TCCLKS_SHIFT) /* TIMER_CLOCK1 Clock selected */
#  define TC_CMR_TCCLKS_TCLK2    (1 << TC_CMR_TCCLKS_SHIFT) /* TIMER_CLOCK2 Clock selected */
#  define TC_CMR_TCCLKS_TCLK3    (2 << TC_CMR_TCCLKS_SHIFT) /* TIMER_CLOCK3 Clock selected */
#  define TC_CMR_TCCLKS_TCLK4    (3 << TC_CMR_TCCLKS_SHIFT) /* TIMER_CLOCK4 Clock selected */
#  define TC_CMR_TCCLKS_TCLK5    (4 << TC_CMR_TCCLKS_SHIFT) /* TIMER_CLOCK5 Clock selected */
#  define TC_CMR_TCCLKS_PCK6     TC_CMR_TCCLKS_TCLK1        /* TIMER_CLOCK1 is PCK6 */
#  define TC_CMR_TCCLKS_MCK8     TC_CMR_TCCLKS_TCLK2        /* TIMER_CLOCK2 is MCK/8 */
#  define TC_CMR_TCCLKS_MCK32    TC_CMR_TCCLKS_TCLK3        /* TIMER_CLOCK3 is MCK/32 */
#  define TC_CMR_TCCLKS_MCK128   TC_CMR_TCCLKS_TCLK4        /* TIMER_CLOCK4 is MCK/128 */
#  define TC_CMR_TCCLKS_SLCK     TC_CMR_TCCLKS_TCLK5        /* TIMER_CLOCK5 is SLCK */
#  define TC_CMR_TCCLKS_XC0      (5 << TC_CMR_TCCLKS_SHIFT) /* XC0 Clock selected */
#  define TC_CMR_TCCLKS_XC1      (6 << TC_CMR_TCCLKS_SHIFT) /* XC1 Clock selected */
#  define TC_CMR_TCCLKS_XC2      (7 << TC_CMR_TCCLKS_SHIFT) /* XC2 Clock selected */
#define TC_CMR_CLKI              (1 << 3)  /* Bit 3:  Clock Invert */
#define TC_CMR_BURST_SHIFT       (4)       /* Bits 4-5: Burst Signal Selection */
#define TC_CMR_BURST_MASK        (3 << TC_CMR_BURST_SHIFT)
#  define TC_CMR_BURST_NONE      (0 << TC_CMR_BURST_SHIFT) /* Clock not gated by external signal */
#  define TC_CMR_BURST_XC0       (1 << TC_CMR_BURST_SHIFT) /* XXC0 ANDed with clock */
#  define TC_CMR_BURST_XC1       (2 << TC_CMR_BURST_SHIFT) /* XC1 ANDed with clock */
#  define TC_CMR_BURST_XC2       (3 << TC_CMR_BURST_SHIFT) /* XC2 ANDed with clock */

/* Channel Mode Register -- Capture mode */

#define TC_CMR_LDBSTOP           (1 << 6)  /* Bit 6:  Counter Clock Stopped with RB Loading */
#define TC_CMR_LDBDIS            (1 << 7)  /* Bit 7:  Counter Clock Disable with RB Loading */
#define TC_CMR_ETRGEDG_SHIFT     (8)       /* Bits 8-9: External Trigger Edge Selection */
#define TC_CMR_ETRGEDG_MASK      (3 << TC_CMR_ETRGEDG_SHIFT)
#  define TC_CMR_ETRGEDG_NONE    (0 << TC_CMR_ETRGEDG_SHIFT) /* Clock not gated by external signal */
#  define TC_CMR_ETRGEDG_RISING  (1 << TC_CMR_ETRGEDG_SHIFT) /* Rising edge */
#  define TC_CMR_ETRGEDG_FALLING (2 << TC_CMR_ETRGEDG_SHIFT) /* Falling edge */
#  define TC_CMR_ETRGEDG_BOTH    (3 << TC_CMR_ETRGEDG_SHIFT) /* EDGE Each edge */
#define TC_CMR_ABETRG            (1 << 10) /* Bit 10: TIOA or TIOB External Trigger Selection */
#define TC_CMR_CPCTRG            (1 << 14) /* Bit 14: RC Compare Trigger Enable */
#define TC_CMR_CAPTURE           (0)       /* Bit 15:  0=Capture Mode */
#define TC_CMR_LDRA_SHIFT        (16)      /* Bits 16-17: RA Loading Edge Selection */
#define TC_CMR_LDRA_MASK         (3 << TC_CMR_LDRA_SHIFT)
#  define TC_CMR_LDRA_NONE       (0 << TC_CMR_LDRA_SHIFT) /* None */
#  define TC_CMR_LDRA_RISING     (1 << TC_CMR_LDRA_SHIFT) /* Rising edge of TIOA */
#  define TC_CMR_LDRA_FALLING    (2 << TC_CMR_LDRA_SHIFT) /* Falling edge of TIOA */
#  define TC_CMR_LDRA_BOTH       (3 << TC_CMR_LDRA_SHIFT) /* Each edge of TIOA */
#define TC_CMR_LDRB_SHIFT        (18)      /* Bits 18-19: RB Loading Edge Selection */
#define TC_CMR_LDRB_MASK         (3 << TC_CMR_LDRB_SHIFT)
#  define TC_CMR_LDRB_NONE       (0 << TC_CMR_LDRB_SHIFT) /* None */
#  define TC_CMR_LDRB_RISING     (1 << TC_CMR_LDRB_SHIFT) /* Rising edge of TIOA */
#  define TC_CMR_LDRB_FALLING    (2 << TC_CMR_LDRB_SHIFT) /* Falling edge of TIOA */
#  define TC_CMR_LDRB_BOTH       (3 << TC_CMR_LDRB_SHIFT) /* Each edge of TIOA */
#define TC_CMR_SBSMPLR_SHIFT     (20)      /* Bits 20-22: Loading Edge Subsampling Ratio */
#define TC_CMR_SBSMPLR_MASK      (7 << TC_CMR_SBSMPLR_SHIFT)
#  define TC_CMR_SBSMPLR_ONE     (0 << TC_CMR_SBSMPLR_SHIFT) /* Load on each selected edge */
#  define TC_CMR_SBSMPLR_HALF    (1 << TC_CMR_SBSMPLR_SHIFT) /* Load on every 2 selected edges */
#  define TC_CMR_SBSMPLR_4TH     (2 << TC_CMR_SBSMPLR_SHIFT) /* Load on every 4 selected edges */
#  define TC_CMR_SBSMPLR_8TH     (3 << TC_CMR_SBSMPLR_SHIFT) /* Load on every 8 selected edges */
#  define TC_CMR_SBSMPLR_16TH    (4 << TC_CMR_SBSMPLR_SHIFT) /* Load on every 16 selected edges */

/* Channel Mode Register -- Waveform mode */

#define TC_CMR_CPCSTOP           (1 << 6)  /* Bit 6:  Counter Clock Stopped with RC Compare */
#define TC_CMR_CPCDIS            (1 << 7)  /* Bit 7:  Counter Clock Disable with RC Compare */
#define TC_CMR_EEVTEDG_SHIFT     (8)       /* Bits 8-9: External Event Edge Selection */
#define TC_CMR_EEVTEDG_MASK      (3 << TC_CMR_EEVTEDG_SHIFT)
#  define TC_CMR_EEVTEDG_NONE    (0 << TC_CMR_EEVTEDG_SHIFT) /* None */
#  define TC_CMR_EEVTEDG_RISING  (1 << TC_CMR_EEVTEDG_SHIFT) /* Rising edge */
#  define TC_CMR_EEVTEDG_FALLING (2 << TC_CMR_EEVTEDG_SHIFT) /* Falling edge */
#  define TC_CMR_EEVTEDG_BOTH    (3 << TC_CMR_EEVTEDG_SHIFT) /* Each edge */
#define TC_CMR_EEVT_SHIFT        (10)      /* Bits 10-11: External Event Selection */
#define TC_CMR_EEVT_MASK         (3 << TC_CMR_EEVT_SHIFT)
#  define TC_CMR_EEVT_TIOB       (0 << TC_CMR_EEVT_SHIFT) /* TIOB(1) input */
#  define TC_CMR_EEVT_XC0        (1 << TC_CMR_EEVT_SHIFT) /* XC0 output */
#  define TC_CMR_EEVT_XC1        (2 << TC_CMR_EEVT_SHIFT) /* XC1 output */
#  define TC_CMR_EEVT_XC2        (3 << TC_CMR_EEVT_SHIFT) /* XC2 output */
#define TC_CMR_ENETRG            (1 << 12) /* Bit 12: External Event Trigger Enable */
#define TC_CMR_WAVSEL_SHIFT      (13)      /* Bits 13-14: Waveform Selection */
#define TC_CMR_WAVSEL_MASK       (3 << TC_CMR_WAVSEL_SHIFT)
#  define TC_CMR_WAVSEL_UP       (0 << TC_CMR_WAVSEL_SHIFT) /* UP mode w/o trigger on RC Compare */
#  define TC_CMR_WAVSEL_UPDOWN   (1 << TC_CMR_WAVSEL_SHIFT) /* UPDOWN mode w/o trigger on RC Compare */
#  define TC_CMR_WAVSEL_UPRC     (2 << TC_CMR_WAVSEL_SHIFT) /* UP mode w/ trigger on RC Compare */
#  define TC_CMR_WAVSEL_UPDOWNRC (3 << TC_CMR_WAVSEL_SHIFT) /* UPDOWN w/ with trigger on RC Compare */
#define TC_CMR_WAVE              (1 << 15) /* Bit 15: 1=Waveform Mode */
#define TC_CMR_ACPA_SHIFT        (16)      /* Bits 16-17: RA Compare Effect on TIOA */
#define TC_CMR_ACPA_MASK         (3 << TC_CMR_ACPA_SHIFT)
#  define TC_CMR_ACPA_NONE       (0 << TC_CMR_ACPA_SHIFT) /* None */
#  define TC_CMR_ACPA_SET        (1 << TC_CMR_ACPA_SHIFT) /* Set */
#  define TC_CMR_ACPA_CLEAR      (2 << TC_CMR_ACPA_SHIFT) /* Clear */
#  define TC_CMR_ACPA_TOGGLE     (3 << TC_CMR_ACPA_SHIFT) /* Toggle */
#define TC_CMR_ACPC_SHIFT        (18)      /* Bits 18-19: RC Compare Effect on TIOA */
#define TC_CMR_ACPC_MASK         (3 << TC_CMR_ACPC_SHIFT)
#  define TC_CMR_ACPC_NONE       (0 << TC_CMR_ACPC_SHIFT) /* None */
#  define TC_CMR_ACPC_SET        (1 << TC_CMR_ACPC_SHIFT) /* Set */
#  define TC_CMR_ACPC_CLEAR      (2 << TC_CMR_ACPC_SHIFT) /* Clear */
#  define TC_CMR_ACPC_TOGGLE     (3 << TC_CMR_ACPC_SHIFT) /* Toggle */
#define TC_CMR_AEEVT_SHIFT       (20)      /* Bits 20-21: External Event Effect on TIOA */
#define TC_CMR_AEEVT_MASK        (3 << TC_CMR_AEEVT_SHIFT)
#  define TC_CMR_AEEVT_NONE      (0 << TC_CMR_AEEVT_SHIFT) /* None */
#  define TC_CMR_AEEVT_SET       (1 << TC_CMR_AEEVT_SHIFT) /* Set */
#  define TC_CMR_AEEVT_CLEAR     (2 << TC_CMR_AEEVT_SHIFT) /* Clear */
#  define TC_CMR_AEEVT_TOGGLE    (3 << TC_CMR_AEEVT_SHIFT) /* Toggle */
#define TC_CMR_ASWTRG_SHIFT      (22)      /* Bits 22-23: Software Trigger Effect on TIOA */
#define TC_CMR_ASWTRG_MASK       (3 << TC_CMR_ASWTRG_SHIFT)
#  define TC_CMR_ASWTRG_NONE     (0 << TC_CMR_ASWTRG_SHIFT) /* None */
#  define TC_CMR_ASWTRG_SET      (1 << TC_CMR_ASWTRG_SHIFT) /* Set */
#  define TC_CMR_ASWTRG_CLEAR    (2 << TC_CMR_ASWTRG_SHIFT) /* Clear */
#  define TC_CMR_ASWTRG_TOGGLE   (3 << TC_CMR_ASWTRG_SHIFT) /* Toggle */
#define TC_CMR_BCPB_SHIFT        (24)      /* Bits 24-25: RB Compare Effect on TIOB */
#define TC_CMR_BCPB_MASK         (3 << TC_CMR_BCPB_SHIFT)
#  define TC_CMR_BCPB_NONE       (0 << TC_CMR_BCPB_SHIFT) /* None */
#  define TC_CMR_BCPB_SET        (1 << TC_CMR_BCPB_SHIFT) /* Set */
#  define TC_CMR_BCPB_CLEAR      (2 << TC_CMR_BCPB_SHIFT) /* Clear */
#  define TC_CMR_BCPB_TOGGLE     (3 << TC_CMR_BCPB_SHIFT) /* Toggle */
#define TC_CMR_BCPC_SHIFT        (26)      /* Bits 26-27: RC Compare Effect on TIOB */
#define TC_CMR_BCPC_MASK         (3 << TC_CMR_BCPC_SHIFT)
#  define TC_CMR_BCPC_NONE       (0 << TC_CMR_BCPC_SHIFT) /* None */
#  define TC_CMR_BCPC_SET        (1 << TC_CMR_BCPC_SHIFT) /* Set */
#  define TC_CMR_BCPC_CLEAR      (2 << TC_CMR_BCPC_SHIFT) /* Clear */
#  define TC_CMR_BCPC_TOGGLE     (3 << TC_CMR_BCPC_SHIFT) /* Toggle */
#define TC_CMR_BEEVT_SHIFT       (28)      /* Bits 28-29: External Event Effect on TIOB */
#define TC_CMR_BEEVT_MASK        (3 << TC_CMR_BEEVT_SHIFT)
#  define TC_CMR_BEEVT_NONE      (0 << TC_CMR_BEEVT_SHIFT) /* None */
#  define TC_CMR_BEEVT_SET       (1 << TC_CMR_BEEVT_SHIFT) /* Set */
#  define TC_CMR_BEEVT_CLEAR     (2 << TC_CMR_BEEVT_SHIFT) /* Clear */
#  define TC_CMR_BEEVT_TOGGLE    (3 << TC_CMR_BEEVT_SHIFT) /* Toggle */
#define TC_CMR_BSWTRG_SHIFT      (30)      /* Bits 30-31: Software Trigger Effect on TIOB */
#define TC_CMR_BSWTRG_MASK       (3 << TC_CMR_BSWTRG_SHIFT)
#  define TC_CMR_BSWTRG_NONE     (0 << TC_CMR_BSWTRG_SHIFT) /* None */
#  define TC_CMR_BSWTRG_SET      (1 << TC_CMR_BSWTRG_SHIFT) /* Set */
#  define TC_CMR_BSWTRG_CLEAR    (2 << TC_CMR_BSWTRG_SHIFT) /* Clear */
#  define TC_CMR_BSWTRG_TOGGLE   (3 << TC_CMR_BSWTRG_SHIFT) /* Toggle */

/* Stepper Motor Mode Register */

#define TC_SMMR_GCEN             (1 << 0)  /* Bit 0:  Gray Count Enable */
#define TC_SMMR_DOWN             (1 << 1)  /* Bit 1:  DOWN Count */

/* Register AB (32-bit capture value) */
/* Counter Value (32-bit counter value) */
/* Register A (32-bit value) */
/* Register B (32-bit value) */
/* Register C (32-bit value) */

/* Status Register, Interrupt Enable Register, Interrupt Disable Register, and
 * Interrupt Mask Register
 */

#define TC_INT_COVFS             (1 << 0)  /* Bit 0:  Counter Overflow Status */
#define TC_INT_LOVRS             (1 << 1)  /* Bit 1:  Load Overrun Status */
#define TC_INT_CPAS              (1 << 2)  /* Bit 2:  RA Compare Status */
#define TC_INT_CPBS              (1 << 3)  /* Bit 3:  RB Compare Status */
#define TC_INT_CPCS              (1 << 4)  /* Bit 4:  RC Compare Status */
#define TC_INT_LDRAS             (1 << 5)  /* Bit 5:  RA Loading Status */
#define TC_INT_LDRBS             (1 << 6)  /* Bit 6:  RB Loading Status */
#define TC_INT_ETRGS             (1 << 7)  /* Bit 7:  External Trigger Status */
#define TC_INT_ALL               (0xff)

#define TC_SR_CLKSTA             (1 << 16) /* Bit 16: Clock Enabling Status */
#define TC_SR_MTIOA              (1 << 17) /* Bit 17: TIOA Mirror */
#define TC_SR_MTIOB              (1 << 18) /* Bit 18: TIOB Mirror */

/* Extended Mode Register */

#define TC_EMR_TRIGSRCA_SHIFT    (0)       /* Bits 0-1: Trigger source for input A */
#define TC_EMR_TRIGSRCA_MASK     (3 << TC_EMR_TRIGSRCA_SHIFT)
#  define TC_EMR_TRIGSRCA_TIOA   (0 << TC_EMR_TRIGSRCA_SHIFT) /* Trigger/capture input A driven by TIOAx */
#  define TC_EMR_TRIGSRCA_PWM    (1 << TC_EMR_TRIGSRCA_SHIFT) /* Trigger/capture input A driven by PWMx */
#define TC_EMR_TRIGSRCB_SHIFT    (4)       /* Bits 4-5: Trigger source for input B */
#define TC_EMR_TRIGSRCB_MASK     (3 << TC_EMR_TRIGSRCB_SHIFT)
#  define TC_EMR_TRIGSRCB_TIOB   (0 << TC_EMR_TRIGSRCB_SHIFT) /* Trigger/capture input B driven by TIOBx */
#  define TC_EMR_TRIGSRCB_PWM    (1 << TC_EMR_TRIGSRCB_SHIFT) /* Trigger/capture input B driven PWMx */
#define TC_EMR_NODIVCLK          (1 << 8)  /* Bit 8: No divided clock */

/* Block Control Register */

#define TC_BCR_SYNC              (1 << 0)  /* Bit 0:  Synchro Command */

/* Block Mode Register */

#define TC_BMR_TC0XC0S_SHIFT     (0)       /* Bits 0-1: External Clock Signal 0 Selection */
#define TC_BMR_TC0XC0S_MASK      (3 << TC_BMR_TC0XC0S_SHIFT)
#  define TC_BMR_TC0XC0S_TCLK0   (0 << TC_BMR_TC0XC0S_SHIFT) /* TCLK0 Signal to XC0 */
#  define TC_BMR_TC0XC0S_TIOA1   (2 << TC_BMR_TC0XC0S_SHIFT) /* TIOA1 Signal to XC0 */
#  define TC_BMR_TC0XC0S_TIOA2   (3 << TC_BMR_TC0XC0S_SHIFT) /* TIOA2 Signal to XC0 */
#define TC_BMR_TC1XC1S_SHIFT     (2)       /* Bits 2-3: External Clock Signal 1 Selection */
#define TC_BMR_TC1XC1S_MASK      (3 << TC_BMR_TC1XC1S_SHIFT)
#  define TC_BMR_TC1XC1S_TCLK1   (0 << TC_BMR_TC1XC1S_SHIFT) /* TCLK1 Signal to XC1 */
#  define TC_BMR_TC1XC1S_TIOA0   (2 << TC_BMR_TC1XC1S_SHIFT) /* TIOA0 Signal to XC1 */
#  define TC_BMR_TC1XC1S_TIOA2   (3 << TC_BMR_TC1XC1S_SHIFT) /* TIOA2 Signal to XC1 */
#define TC_BMR_TC2XC2S_SHIFT     (4)       /* Bits 4-5: External Clock Signal 2 Selection */
#define TC_BMR_TC2XC2S_MASK      (3 << TC_BMR_TC2XC2S_SHIFT)
#  define TC_BMR_TC2XC2S_TCLK2   (0 << TC_BMR_TC2XC2S_SHIFT) /* TCLK2 Signal to XC2 */
#  define TC_BMR_TC2XC2S_TIOA0   (2 << TC_BMR_TC2XC2S_SHIFT) /* TIOA0 Signal to XC2 */
#  define TC_BMR_TC2XC2S_TIOA1   (3 << TC_BMR_TC2XC2S_SHIFT) /* TIOA1 Signal to XC2 */
#define TC_BMR_QDEN              (1 << 8)  /* Bit 8:  Quadrature Decoder ENabled */
#define TC_BMR_POSEN             (1 << 9)  /* Bit 9:  POSition ENabled */
#define TC_BMR_SPEEDEN           (1 << 10) /* Bit 10: SPEED ENabled */
#define TC_BMR_QDTRANS           (1 << 11) /* Bit 11: Quadrature Decoding TRANSparent */
#define TC_BMR_EDGPHA            (1 << 12) /* Bit 12: EDGe on PHA count mode */
#define TC_BMR_INVA              (1 << 13) /* Bit 13: INVerted phA */
#define TC_BMR_INVB              (1 << 14) /* Bit 14: INVerted phB */
#define TC_BMR_INVIDX            (1 << 15) /* Bit 15: INVerted InDeX */
#define TC_BMR_SWAP              (1 << 16) /* Bit 16: SWAP PHA and PHB */
#define TC_BMR_IDXPHB            (1 << 17) /* Bit 17: InDeX pin is PHB pin */
#define TC_BMR_MAXFILT_SHIFT     (20)      /* Bits 20-25: MAXimum FILTer */
#define TC_BMR_MAXFILT_MASK      (63 << TC_BMR_MAXFILT_SHIFT)
#  define TC_BMR_MAXFILT(n)      ((uint32_t)(n) << TC_BMR_MAXFILT_SHIFT)

/* QDEC Interrupt Enable Register, QDEC Interrupt Disable Register, QDEC Interrupt Mask Register, and QDEC Interrupt Status Register.
 */

#define TC_QINT_IDX              (1 << 0)  /* Bit 0:  Index */
#define TC_QINT_DIRCHG           (1 << 1)  /* Bit 1:  Direction change */
#define TC_QINT_QERR             (1 << 2)  /* Bit 2:  Quadrature ERRor */

#define TC_QISR_DIRR             (1 << 8)  /* Bit 8: Direction */

/* Fault Mode Register */

#define TC_FMR_ENCF0             (1 << 0)  /* Bit 0:  ENable Compare Fault Channel 0 */
#define TC_FMR_ENCF1             (1 << 1)  /* Bit 1:  ENable Compare Fault Channel 1 */

/* Write Protect Mode Register */

#define TC_WPMR_WPEN             (1 << 0)  /* Bit 0:  Write Protect Enable */
#define TC_WPMR_WPKEY_SHIFT      (8)       /* Bits 8-31: Write Protect KEY */
#define TC_WPMR_WPKEY_MASK       (0xffffff << TC_WPMR_WPKEY_SHIFT)
#  define TC_WPMR_WPKEY          (0x54494d << TC_WPMR_WPKEY_SHIFT) /* "TIM" in ASCII */

#endif /* __ARCH_ARM_SRC_SAMV7_CHIP_SAM_TC_H */
