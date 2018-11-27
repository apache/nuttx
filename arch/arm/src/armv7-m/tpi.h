/***********************************************************************************************
 * arch/arm/src/armv7-m/tpi.h
 *
 *   Copyright (c) 2009 - 2013 ARM LIMITED
 *
 *  All rights reserved.
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  - Neither the name of ARM nor the names of its contributors may be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *   Copyright (C) 2014 Pierre-noel Bouteville . All rights reserved.
 *   Author: Pierre-noel Bouteville <pnb990@gmail.com>
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
 ***********************************************************************************************/

#ifndef __ARCH_ARM_SRC_ARMV7_M_TPI_H
#define __ARCH_ARM_SRC_ARMV7_M_TPI_H

/***********************************************************************************************
 * Pre-processor Definitions
 ***********************************************************************************************/

/* Trace Port Interface Register (TPI) Definitions *********************************************/

/* TPI Register Base Address *******************************************************************/

#define TPI_BASE                      (0xe0040000ul)

/* TPI Register Addresses **********************************************************************/

#define TPI_SSPSR                     (TPI_BASE + 0x0000) /* Supported Parallel Port Size Register */
#define TPI_CSPSR                     (TPI_BASE + 0x0004) /* Current Parallel Port Size Register */
#define TPI_ACPR                      (TPI_BASE + 0x0010) /* Asynchronous Clock Prescaler Register */
#define TPI_SPPR                      (TPI_BASE + 0x00f0) /* Selected Pin Protocol Register */
#define TPI_FFSR                      (TPI_BASE + 0x0300) /* Formatter and Flush Status Register */
#define TPI_FFCR                      (TPI_BASE + 0x0304) /* Formatter and Flush Control Register */
#define TPI_FSCR                      (TPI_BASE + 0x0308) /* Formatter Synchronization Counter Register */
#define TPI_TRIGGER                   (TPI_BASE + 0x0ee8) /* TRIGGER */
#define TPI_FIFO0                     (TPI_BASE + 0x0eec) /* Integration ETM Data */
#define TPI_ITATBCTR2                 (TPI_BASE + 0x0ef0) /* ITATBCTR2 */
#define TPI_ITATBCTR0                 (TPI_BASE + 0x0ef8) /* ITATBCTR0 */
#define TPI_FIFO1                     (TPI_BASE + 0x0efc) /* Integration ITM Data */
#define TPI_ITCTRL                    (TPI_BASE + 0x0f00) /* Integration Mode Control */
#define TPI_CLAIMSET                  (TPI_BASE + 0x0fa0) /* Claim tag set */
#define TPI_CLAIMCLR                  (TPI_BASE + 0x0fa4) /* Claim tag clear */
#define TPI_DEVID                     (TPI_BASE + 0x0fc8) /* TPIU_DEVID */
#define TPI_DEVTYPE                   (TPI_BASE + 0x0fcc) /* TPIU_DEVTYPE */

/* TPI Register Bit Field Definitions **********************************************************/

/* TPI ACPR */

#define TPI_ACPR_PRESCALER_SHIFT      0
#define TPI_ACPR_PRESCALER_MASK       (0x1ffful << TPI_ACPR_PRESCALER_SHIFT)

/* TPI SPPR */

#define TPI_SPPR_TXMODE_SHIFT         0
#define TPI_SPPR_TXMODE_MASK          (0x3ul << TPI_SPPR_TXMODE_SHIFT)

/* TPI FFSR */

#define TPI_FFSR_FtNonStop_SHIFT      3
#define TPI_FFSR_FtNonStop_MASK       (0x1ul << TPI_FFSR_FtNonStop_SHIFT)
#define TPI_FFSR_TCPresent_SHIFT      2
#define TPI_FFSR_TCPresent_MASK       (0x1ul << TPI_FFSR_TCPresent_SHIFT)
#define TPI_FFSR_FtStopped_SHIFT      1
#define TPI_FFSR_FtStopped_MASK       (0x1ul << TPI_FFSR_FtStopped_SHIFT)
#define TPI_FFSR_FlInProg_SHIFT       0
#define TPI_FFSR_FlInProg_MASK        (0x1ul << TPI_FFSR_FlInProg_SHIFT)

/* TPI FFCR */

#define TPI_FFCR_TrigIn_SHIFT         8
#define TPI_FFCR_TrigIn_MASK          (0x1ul << TPI_FFCR_TrigIn_SHIFT)
#define TPI_FFCR_EnFCont_SHIFT        1
#define TPI_FFCR_EnFCont_MASK         (0x1ul << TPI_FFCR_EnFCont_SHIFT)

#define TPI_TRIGGER_TRIGGER_SHIFT     0
#define TPI_TRIGGER_TRIGGER_MASK      (0x1ul << TPI_TRIGGER_TRIGGER_SHIFT)

/* TPI FIFO0 */

#define TPI_FIFO0_ITM_ATVALID_SHIFT   29
#define TPI_FIFO0_ITM_ATVALID_MASK    (0x3ul << TPI_FIFO0_ITM_ATVALID_SHIFT)
#define TPI_FIFO0_ITM_bytecount_SHIFT 27
#define TPI_FIFO0_ITM_bytecount_MASK  (0x3ul << TPI_FIFO0_ITM_bytecount_SHIFT)
#define TPI_FIFO0_ETM_ATVALID_SHIFT   26
#define TPI_FIFO0_ETM_ATVALID_MASK    (0x3ul << TPI_FIFO0_ETM_ATVALID_SHIFT)
#define TPI_FIFO0_ETM_bytecount_SHIFT 24
#define TPI_FIFO0_ETM_bytecount_MASK  (0x3ul << TPI_FIFO0_ETM_bytecount_SHIFT)
#define TPI_FIFO0_ETM2_SHIFT          16
#define TPI_FIFO0_ETM2_MASK           (0xfful << TPI_FIFO0_ETM2_SHIFT)
#define TPI_FIFO0_ETM1_SHIFT          8
#define TPI_FIFO0_ETM1_MASK           (0xfful << TPI_FIFO0_ETM1_SHIFT)
#define TPI_FIFO0_ETM0_SHIFT          0
#define TPI_FIFO0_ETM0_MASK           (0xfful << TPI_FIFO0_ETM0_SHIFT)

/* TPI ITATBCTR2 */

#define TPI_ITATBCTR2_ATREADY_SHIFT   0
#define TPI_ITATBCTR2_ATREADY_MASK    (0x1ul << TPI_ITATBCTR2_ATREADY_SHIFT)

/* TPI FIFO1 */

#define TPI_FIFO1_ITM_ATVALID_SHIFT   29
#define TPI_FIFO1_ITM_ATVALID_MASK    (0x3ul << TPI_FIFO1_ITM_ATVALID_SHIFT)
#define TPI_FIFO1_ITM_bytecount_SHIFT 27
#define TPI_FIFO1_ITM_bytecount_MASK  (0x3ul << TPI_FIFO1_ITM_bytecount_SHIFT)
#define TPI_FIFO1_ETM_ATVALID_SHIFT   26
#define TPI_FIFO1_ETM_ATVALID_MASK    (0x3ul << TPI_FIFO1_ETM_ATVALID_SHIFT)
#define TPI_FIFO1_ETM_bytecount_SHIFT 24
#define TPI_FIFO1_ETM_bytecount_MASK  (0x3ul << TPI_FIFO1_ETM_bytecount_SHIFT)
#define TPI_FIFO1_ITM2_SHIFT          16
#define TPI_FIFO1_ITM2_MASK           (0xfful << TPI_FIFO1_ITM2_SHIFT)
#define TPI_FIFO1_ITM1_SHIFT          8
#define TPI_FIFO1_ITM1_MASK           (0xfful << TPI_FIFO1_ITM1_SHIFT)
#define TPI_FIFO1_ITM0_SHIFT          0
#define TPI_FIFO1_ITM0_MASK           (0xfful << TPI_FIFO1_ITM0_SHIFT)

/* TPI ITATBCTR0 */

#define TPI_ITATBCTR0_ATREADY_SHIFT   0
#define TPI_ITATBCTR0_ATREADY_MASK    (0x1ul << TPI_ITATBCTR0_ATREADY_SHIFT)

/* TPI ITCTRL */

#define TPI_ITCTRL_Mode_SHIFT         0
#define TPI_ITCTRL_Mode_MASK          (0x1ul << TPI_ITCTRL_Mode_SHIFT)

/* TPI DEVID */

#define TPI_DEVID_NRZVALID_SHIFT      11
#define TPI_DEVID_NRZVALID_MASK       (0x1ul << TPI_DEVID_NRZVALID_SHIFT)
#define TPI_DEVID_MANCVALID_SHIFT     10
#define TPI_DEVID_MANCVALID_MASK      (0x1ul << TPI_DEVID_MANCVALID_SHIFT)
#define TPI_DEVID_PTINVALID_SHIFT     9
#define TPI_DEVID_PTINVALID_MASK      (0x1ul << TPI_DEVID_PTINVALID_SHIFT)
#define TPI_DEVID_MinBufSz_SHIFT      6
#define TPI_DEVID_MinBufSz_MASK       (0x7ul << TPI_DEVID_MinBufSz_SHIFT)
#define TPI_DEVID_AsynClkIn_SHIFT     5
#define TPI_DEVID_AsynClkIn_MASK      (0x1ul << TPI_DEVID_AsynClkIn_SHIFT)
#define TPI_DEVID_NrTraceInput_SHIFT  0
#define TPI_DEVID_NrTraceInput_MASK   (0x1ful << TPI_DEVID_NrTraceInput_SHIFT)

/* TPI DEVTYPE */

#define TPI_DEVTYPE_SubType_SHIFT     0
#define TPI_DEVTYPE_SubType_MASK      (0xful << TPI_DEVTYPE_SubType_SHIFT)
#define TPI_DEVTYPE_MajorType_SHIFT   4
#define TPI_DEVTYPE_MajorType_MASK    (0xful << TPI_DEVTYPE_MajorType_SHIFT)

#endif /* __ARCH_ARM_SRC_ARMV7_M_TPI_H */
