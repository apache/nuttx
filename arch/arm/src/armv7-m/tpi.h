/***********************************************************************************************
 * arch/arm/src/armv7-m/tpi.h
 *
 *   Copyright (c) 2009 - 2013 ARM LIMITED
 *
 *  All rights reserved.
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  - Neither the name of ARM nor the names of its contributors may be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *  *
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

#define TPI_BASE                    (0xe0040000ul)

/* TPI Register Addresses **********************************************************************/

#define TPI_SSPSR                   (TPI_BASE+0x0000) /* Supported Parallel Port Size Register */
#define TPI_CSPSR                   (TPI_BASE+0x0004) /* Current Parallel Port Size Register */
#define TPI_ACPR                    (TPI_BASE+0x0010) /* Asynchronous Clock Prescaler Register */
#define TPI_SPPR                    (TPI_BASE+0x00f0) /* Selected Pin Protocol Register */
#define TPI_FFSR                    (TPI_BASE+0x0300) /* Formatter and Flush Status Register */
#define TPI_FFCR                    (TPI_BASE+0x0304) /* Formatter and Flush Control Register */
#define TPI_FSCR                    (TPI_BASE+0x0308) /* Formatter Synchronization Counter Register */
#define TPI_TRIGGER                 (TPI_BASE+0x0ee8) /* TRIGGER */
#define TPI_FIFO0                   (TPI_BASE+0x0eec) /* Integration ETM Data */
#define TPI_ITATBCTR2               (TPI_BASE+0x0ef0) /* ITATBCTR2 */
#define TPI_ITATBCTR0               (TPI_BASE+0x0ef8) /* ITATBCTR0 */
#define TPI_FIFO1                   (TPI_BASE+0x0efc) /* Integration ITM Data */
#define TPI_ITCTRL                  (TPI_BASE+0x0f00) /* Integration Mode Control */
#define TPI_CLAIMSET                (TPI_BASE+0x0fa0) /* Claim tag set */
#define TPI_CLAIMCLR                (TPI_BASE+0x0fa4) /* Claim tag clear */
#define TPI_DEVID                   (TPI_BASE+0x0fc8) /* TPIU_DEVID */
#define TPI_DEVTYPE                 (TPI_BASE+0x0fcc) /* TPIU_DEVTYPE */

/* TPI Register Bit Field Definitions **********************************************************/

#define TPI_ACPR_PRESCALER_Pos      0                                      /* TPI ACPR: PRESCALER Position */
#define TPI_ACPR_PRESCALER_Msk      (0x1ffful << TPI_ACPR_PRESCALER_Pos)   /* TPI ACPR: PRESCALER Mask */

#define TPI_SPPR_TXMODE_Pos         0                                      /* TPI SPPR: TXMODE Position */
#define TPI_SPPR_TXMODE_Msk         (0x3ul << TPI_SPPR_TXMODE_Pos)         /* TPI SPPR: TXMODE Mask */

#define TPI_FFSR_FtNonStop_Pos      3                                      /* TPI FFSR: FtNonStop Position */
#define TPI_FFSR_FtNonStop_Msk      (0x1ul << TPI_FFSR_FtNonStop_Pos)      /* TPI FFSR: FtNonStop Mask */
#define TPI_FFSR_TCPresent_Pos      2                                      /* TPI FFSR: TCPresent Position */
#define TPI_FFSR_TCPresent_Msk      (0x1ul << TPI_FFSR_TCPresent_Pos)      /* TPI FFSR: TCPresent Mask */
#define TPI_FFSR_FtStopped_Pos      1                                      /* TPI FFSR: FtStopped Position */
#define TPI_FFSR_FtStopped_Msk      (0x1ul << TPI_FFSR_FtStopped_Pos)      /* TPI FFSR: FtStopped Mask */
#define TPI_FFSR_FlInProg_Pos       0                                      /* TPI FFSR: FlInProg Position */
#define TPI_FFSR_FlInProg_Msk       (0x1ul << TPI_FFSR_FlInProg_Pos)       /* TPI FFSR: FlInProg Mask */

#define TPI_FFCR_TrigIn_Pos         8                                      /* TPI FFCR: TrigIn Position */
#define TPI_FFCR_TrigIn_Msk         (0x1ul << TPI_FFCR_TrigIn_Pos)         /* TPI FFCR: TrigIn Mask */
#define TPI_FFCR_EnFCont_Pos        1                                      /* TPI FFCR: EnFCont Position */
#define TPI_FFCR_EnFCont_Msk        (0x1ul << TPI_FFCR_EnFCont_Pos)        /* TPI FFCR: EnFCont Mask */

#define TPI_TRIGGER_TRIGGER_Pos     0                                      /* TPI TRIGGER: TRIGGER Position */
#define TPI_TRIGGER_TRIGGER_Msk     (0x1ul << TPI_TRIGGER_TRIGGER_Pos)     /* TPI TRIGGER: TRIGGER Mask */

#define TPI_FIFO0_ITM_ATVALID_Pos   29                                     /* TPI FIFO0: ITM_ATVALID Position */
#define TPI_FIFO0_ITM_ATVALID_Msk   (0x3ul << TPI_FIFO0_ITM_ATVALID_Pos)   /* TPI FIFO0: ITM_ATVALID Mask */
#define TPI_FIFO0_ITM_bytecount_Pos 27                                     /* TPI FIFO0: ITM_bytecount Position */
#define TPI_FIFO0_ITM_bytecount_Msk (0x3ul << TPI_FIFO0_ITM_bytecount_Pos) /* TPI FIFO0: ITM_bytecount Mask */
#define TPI_FIFO0_ETM_ATVALID_Pos   26                                     /* TPI FIFO0: ETM_ATVALID Position */
#define TPI_FIFO0_ETM_ATVALID_Msk   (0x3ul << TPI_FIFO0_ETM_ATVALID_Pos)   /* TPI FIFO0: ETM_ATVALID Mask */
#define TPI_FIFO0_ETM_bytecount_Pos 24                                     /* TPI FIFO0: ETM_bytecount Position */
#define TPI_FIFO0_ETM_bytecount_Msk (0x3ul << TPI_FIFO0_ETM_bytecount_Pos) /* TPI FIFO0: ETM_bytecount Mask */
#define TPI_FIFO0_ETM2_Pos          16                                     /* TPI FIFO0: ETM2 Position */
#define TPI_FIFO0_ETM2_Msk          (0xfful << TPI_FIFO0_ETM2_Pos)         /* TPI FIFO0: ETM2 Mask */
#define TPI_FIFO0_ETM1_Pos          8                                      /* TPI FIFO0: ETM1 Position */
#define TPI_FIFO0_ETM1_Msk          (0xfful << TPI_FIFO0_ETM1_Pos)         /* TPI FIFO0: ETM1 Mask */
#define TPI_FIFO0_ETM0_Pos          0                                      /* TPI FIFO0: ETM0 Position */
#define TPI_FIFO0_ETM0_Msk          (0xfful << TPI_FIFO0_ETM0_Pos)         /* TPI FIFO0: ETM0 Mask */

#define TPI_ITATBCTR2_ATREADY_Pos   0                                      /* TPI ITATBCTR2: ATREADY Position */
#define TPI_ITATBCTR2_ATREADY_Msk   (0x1ul << TPI_ITATBCTR2_ATREADY_Pos)   /* TPI ITATBCTR2: ATREADY Mask */

#define TPI_FIFO1_ITM_ATVALID_Pos   29                                     /* TPI FIFO1: ITM_ATVALID Position */
#define TPI_FIFO1_ITM_ATVALID_Msk   (0x3ul << TPI_FIFO1_ITM_ATVALID_Pos)   /* TPI FIFO1: ITM_ATVALID Mask */
#define TPI_FIFO1_ITM_bytecount_Pos 27                                     /* TPI FIFO1: ITM_bytecount Position */
#define TPI_FIFO1_ITM_bytecount_Msk (0x3ul << TPI_FIFO1_ITM_bytecount_Pos) /* TPI FIFO1: ITM_bytecount Mask */
#define TPI_FIFO1_ETM_ATVALID_Pos   26                                     /* TPI FIFO1: ETM_ATVALID Position */
#define TPI_FIFO1_ETM_ATVALID_Msk   (0x3ul << TPI_FIFO1_ETM_ATVALID_Pos)   /* TPI FIFO1: ETM_ATVALID Mask */
#define TPI_FIFO1_ETM_bytecount_Pos 24                                     /* TPI FIFO1: ETM_bytecount Position */
#define TPI_FIFO1_ETM_bytecount_Msk (0x3ul << TPI_FIFO1_ETM_bytecount_Pos) /* TPI FIFO1: ETM_bytecount Mask */
#define TPI_FIFO1_ITM2_Pos          16                                     /* TPI FIFO1: ITM2 Position */
#define TPI_FIFO1_ITM2_Msk          (0xfful << TPI_FIFO1_ITM2_Pos)         /* TPI FIFO1: ITM2 Mask */
#define TPI_FIFO1_ITM1_Pos          8                                      /* TPI FIFO1: ITM1 Position */
#define TPI_FIFO1_ITM1_Msk          (0xfful << TPI_FIFO1_ITM1_Pos)         /* TPI FIFO1: ITM1 Mask */
#define TPI_FIFO1_ITM0_Pos          0                                      /* TPI FIFO1: ITM0 Position */
#define TPI_FIFO1_ITM0_Msk          (0xfful << TPI_FIFO1_ITM0_Pos)         /* TPI FIFO1: ITM0 Mask */

#define TPI_ITATBCTR0_ATREADY_Pos   0                                      /* TPI ITATBCTR0: ATREADY Position */
#define TPI_ITATBCTR0_ATREADY_Msk   (0x1ul << TPI_ITATBCTR0_ATREADY_Pos)   /* TPI ITATBCTR0: ATREADY Mask */

#define TPI_ITCTRL_Mode_Pos         0                                      /* TPI ITCTRL: Mode Position */
#define TPI_ITCTRL_Mode_Msk         (0x1ul << TPI_ITCTRL_Mode_Pos)         /* TPI ITCTRL: Mode Mask */

#define TPI_DEVID_NRZVALID_Pos      11                                     /* TPI DEVID: NRZVALID Position */
#define TPI_DEVID_NRZVALID_Msk      (0x1ul << TPI_DEVID_NRZVALID_Pos)      /* TPI DEVID: NRZVALID Mask */
#define TPI_DEVID_MANCVALID_Pos     10                                     /* TPI DEVID: MANCVALID Position */
#define TPI_DEVID_MANCVALID_Msk     (0x1ul << TPI_DEVID_MANCVALID_Pos)     /* TPI DEVID: MANCVALID Mask */
#define TPI_DEVID_PTINVALID_Pos     9                                      /* TPI DEVID: PTINVALID Position */
#define TPI_DEVID_PTINVALID_Msk     (0x1ul << TPI_DEVID_PTINVALID_Pos)     /* TPI DEVID: PTINVALID Mask */
#define TPI_DEVID_MinBufSz_Pos      6                                      /* TPI DEVID: MinBufSz Position */
#define TPI_DEVID_MinBufSz_Msk      (0x7ul << TPI_DEVID_MinBufSz_Pos)      /* TPI DEVID: MinBufSz Mask */
#define TPI_DEVID_AsynClkIn_Pos     5                                      /* TPI DEVID: AsynClkIn Position */
#define TPI_DEVID_AsynClkIn_Msk     (0x1ul << TPI_DEVID_AsynClkIn_Pos)     /* TPI DEVID: AsynClkIn Mask */
#define TPI_DEVID_NrTraceInput_Pos  0                                      /* TPI DEVID: NrTraceInput Position */
#define TPI_DEVID_NrTraceInput_Msk  (0x1ful << TPI_DEVID_NrTraceInput_Pos) /* TPI DEVID: NrTraceInput Mask */

#define TPI_DEVTYPE_SubType_Pos     0                                      /* TPI DEVTYPE: SubType Position */
#define TPI_DEVTYPE_SubType_Msk     (0xful << TPI_DEVTYPE_SubType_Pos)     /* TPI DEVTYPE: SubType Mask */
#define TPI_DEVTYPE_MajorType_Pos   4                                      /* TPI DEVTYPE: MajorType Position */
#define TPI_DEVTYPE_MajorType_Msk   (0xful << TPI_DEVTYPE_MajorType_Pos)   /* TPI DEVTYPE: MajorType Mask */

#endif /* __ARCH_ARM_SRC_ARMV7_M_TPI_H */
