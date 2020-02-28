/************************************************************************************
 * arch/arm/src/tiva/hardware/tiva_qencoder.h
 *
 *   Copyright (C) 2016 Young Mu. All rights reserved.
 *   Author: Young Mu <young.mu@aliyun.com>
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

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_TIVA_QENCODER_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_TIVA_QENCODER_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define TIVA_QEI_CTL_OFFSET             (0x0)   /* QEI Control */
#define TIVA_QEI_STAT_OFFSET            (0x4)   /* QEI Status */
#define TIVA_QEI_POS_OFFSET             (0x8)   /* QEI Position */
#define TIVA_QEI_MAXPOS_OFFSET          (0xc)   /* QEI Maximum Position */
#define TIVA_QEI_LOAD_OFFSET            (0x10)  /* QEI Timer Load */
#define TIVA_QEI_TIME_OFFSET            (0x14)  /* QEI Timer */
#define TIVA_QEI_COUNT_OFFSET           (0x18)  /* QEI Velocity Counter */
#define TIVA_QEI_SPEED_OFFSET           (0x1c)  /* QEI Velocity */
#define TIVA_QEI_INTEN_OFFSET           (0x20)  /* QEI Interrupt Enable */
#define TIVA_QEI_RIS_OFFSET             (0x24)  /* QEI Raw Interrupt Status */
#define TIVA_QEI_ISC_OFFSET             (0x28)  /* QEI Interrupt Status and Clear */

/* QEI_CTL register */

#define TIVA_QEI_CTL_FILTCNT            (16)    /* (Bit) Input Filter Prescale Count */
#define TIVA_QEI_CTL_FILTEN             (13)    /* (Bit) Enable Input Filter */
#define TIVA_QEI_CTL_STALLEN            (12)    /* (Bit) Stall QEI */
#define TIVA_QEI_CTL_INVI               (11)    /* (Bit) Invert Index Pulse */
#define TIVA_QEI_CTL_INVB               (10)    /* (Bit) Invert PhB */
#define TIVA_QEI_CTL_INVA               (9)     /* (Bit) Invert PhA */

#define TIVA_QEI_CTL_VELDIV             (6)     /* (Bit) Predivide Velocity */
#define VELDIV_1                        (0x0)   /* (Value) Divided by 1 */
#define VELDIV_2                        (0x1)   /* (Value) Divided by 2 */
#define VELDIV_4                        (0x2)   /* (Value) Divided by 4 */
#define VELDIV_8                        (0x3)   /* (Value) Divided by 8 */
#define VELDIV_16                       (0x4)   /* (Value) Divided by 16 */
#define VELDIV_32                       (0x5)   /* (Value) Divided by 32 */
#define VELDIV_64                       (0x6)   /* (Value) Divided by 64 */
#define VELDIV_128                      (0x7)   /* (Value) Divided by 128 */

#define TIVA_QEI_CTL_VELEN              (5)     /* (Bit) Capture Velocity */
#define VELEN_DISABLE                   (0)     /* (value) Disable Velocity Capture */
#define VELEN_ENABLE                    (1)     /* (value) Enable Velocity Capture */

#define TIVA_QEI_CTL_RESMODE            (4)     /* (Bit) Reset Mode */
#define RESMODE_BY_MAXPOS               (0)     /* (Value) Reset by MAXPOS */
#define RESMODE_BY_INDEX_PULSE          (1)     /* (Value) Reset by Index Pulse */

#define TIVA_QEI_CTL_CAPMODE            (3)     /* (Bit) Capture Mode */
#define CAPMODE_ONLY_PHA                (0)     /* (Value) Count PhA both edges */
#define CAPMODE_PHA_AND_PHB             (1)     /* (Value) Count PhA and PhB both edges */

#define TIVA_QEI_CTL_SIGMODE            (2)     /* (Bit) Signal Mode */
#define SIGMODE_QUADRATURE              (0)     /* (Value) PhA and PhB are Quadrature signals */
#define SIGMODE_CLK_AND_DIR             (1)     /* (Value) PhA is CLK, PhB is DIR */

#define TIVA_QEI_CTL_SWAP               (1)     /* (Bit) Swap PhA/PhB Signals */
#define SWAP_NO_SWAP                    (0)     /* (Value) No swapping */
#define SWAP_PHA_PHB                    (1)     /* (Value) Swap PhA and PhB */

#define TIVA_QEI_CTL_ENABLE             (0)     /* (Bit) Enable QEI */
#define QEI_DISABLE                     (0)     /* (Value) Disable QEI */
#define QEI_ENABLE                      (1)     /* (Value) Enable QEI */

/* QEI_STAT register */

#define TIVA_QEI_STAT_DIRECTION         (1)     /* (Bit) Direction of Rotation */
#define DIRECTION_FORWARD               (0)     /* (Value) Forward */
#define DIRECTION_BACKWARD              (1)     /* (Value) Backward */

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_TIVA_QENCODER_H */
