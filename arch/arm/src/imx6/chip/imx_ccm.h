/************************************************************************************
 * arch/arm/src/imx6/imx_ccm.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Reference:
 *   "i.MX 6Dual/6Quad ApplicationsProcessor Reference Manual," Document Number
 *   IMX6DQRM, Rev. 3, 07/2015, FreeScale.
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

#ifndef __ARCH_ARM_SRC_IMX6_CHIP_IMX_CCM_H
#define __ARCH_ARM_SRC_IMX6_CHIP_IMX_CCM_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <chip/imx_memorymap.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* CCM Register Offsets *************************************************************/

#define IMX_CCM_CCR_OFFSET         0x0000  /* CCM Control Register */
#define IMX_CCM_CCDR_OFFSET        0x0004  /* CCM Control Divider Register */
#define IMX_CCM_CSR_OFFSET         0x0008  /* CCM Status Register */
#define IMX_CCM_CCSR_OFFSET        0x000c  /* CCM Clock Switcher Register */
#define IMX_CCM_CACRR_OFFSET       0x0010  /* CCM Arm Clock Root Register */
#define IMX_CCM_CBCDR_OFFSET       0x0014  /* CCM Bus Clock Divider Register */
#define IMX_CCM_CBCMR_OFFSET       0x0018  /* CCM Bus Clock Multiplexer Register */
#define IMX_CCM_CSCMR1_OFFSET      0x001c  /* CCM Serial Clock Multiplexer Register 1 */
#define IMX_CCM_CSCMR2_OFFSET      0x0020  /* CCM Serial Clock Multiplexer Register 2 */
#define IMX_CCM_CSCDR1_OFFSET      0x0024  /* CCM Serial Clock Divider Register 1 */
#define IMX_CCM_CS1CDR_OFFSET      0x0028  /* CCM SSI1 Clock Divider Register */
#define IMX_CCM_CS2CDR_OFFSET      0x002c  /* CCM SSI2 Clock Divider Register */
#define IMX_CCM_CDCDR_OFFSET       0x0030  /* CCM D1 Clock Divider Register */
#define IMX_CCM_CHSCCDR_OFFSET     0x0034  /* CCM HSC Clock Divider Register */
#define IMX_CCM_CSCDR2_OFFSET      0x0038  /* CCM Serial Clock Divider Register 2 */
#define IMX_CCM_CSCDR3_OFFSET      0x003c  /* CCM Serial Clock Divider Register 3 */
#define IMX_CCM_CWDR_OFFSET        0x0044  /* CCM Wakeup Detector Register */
#define IMX_CCM_CDHIPR_OFFSET      0x0048  /* CCM Divider Handshake In-Process Register */
#define IMX_CCM_CLPCR_OFFSET       0x0054  /* CCM Low Power Control Register */
#define IMX_CCM_CISR_OFFSET        0x0058  /* CCM Interrupt Status Register */
#define IMX_CCM_CIMR_OFFSET        0x005c  /* CCM Interrupt Mask Register */
#define IMX_CCM_CCOSR_OFFSET       0x0060  /* CCM Clock Output Source Register */
#define IMX_CCM_CGPR_OFFSET        0x0064  /* CCM General Purpose Register */
#define IMX_CCM_CCGR0_OFFSET       0x0068  /* CCM Clock Gating Register 0 */
#define IMX_CCM_CCGR1_OFFSET       0x006c  /* CCM Clock Gating Register 1 */
#define IMX_CCM_CCGR2_OFFSET       0x0070  /* CCM Clock Gating Register 2 */
#define IMX_CCM_CCGR3_OFFSET       0x0074  /* CCM Clock Gating Register 3 */
#define IMX_CCM_CCGR4_OFFSET       0x0078  /* CCM Clock Gating Register 4 */
#define IMX_CCM_CCGR5_OFFSET       0x007c  /* CCM Clock Gating Register 5 */
#define IMX_CCM_CCGR6_OFFSET       0x0080  /* CCM Clock Gating Register 6 */
#define IMX_CCM_CMEOR_OFFSET       0x0088  /* CCM Module Enable Overide Register */

/* CCM Register Addresses ***********************************************************/

#define IMX_CCM_CCR                (IMX_CCM_VBASE+IMX_CCM_CCR_OFFSET)
#define IMX_CCM_CCDR               (IMX_CCM_VBASE+IMX_CCM_CCDR_OFFSET)
#define IMX_CCM_CSR                (IMX_CCM_VBASE+IMX_CCM_CSR_OFFSET)
#define IMX_CCM_CCSR               (IMX_CCM_VBASE+IMX_CCM_CCSR_OFFSET)
#define IMX_CCM_CACRR              (IMX_CCM_VBASE+IMX_CCM_CACRR_OFFSET)
#define IMX_CCM_CBCDR              (IMX_CCM_VBASE+IMX_CCM_CBCDR_OFFSET)
#define IMX_CCM_CBCMR              (IMX_CCM_VBASE+IMX_CCM_CBCMR_OFFSET)
#define IMX_CCM_CSCMR1             (IMX_CCM_VBASE+IMX_CCM_CSCMR1_OFFSET)
#define IMX_CCM_CSCMR2             (IMX_CCM_VBASE+IMX_CCM_CSCMR2_OFFSET)
#define IMX_CCM_CSCDR1             (IMX_CCM_VBASE+IMX_CCM_CSCDR1_OFFSET)
#define IMX_CCM_CS1CDR             (IMX_CCM_VBASE+IMX_CCM_CS1CDR_OFFSET)
#define IMX_CCM_CS2CDR             (IMX_CCM_VBASE+IMX_CCM_CS2CDR_OFFSET)
#define IMX_CCM_CDCDR              (IMX_CCM_VBASE+IMX_CCM_CDCDR_OFFSET)
#define IMX_CCM_CHSCCDR            (IMX_CCM_VBASE+IMX_CCM_CHSCCDR_OFFSET)
#define IMX_CCM_CSCDR2             (IMX_CCM_VBASE+IMX_CCM_CSCDR2_OFFSET)
#define IMX_CCM_CSCDR3             (IMX_CCM_VBASE+IMX_CCM_CSCDR3_OFFSET)
#define IMX_CCM_CWDR               (IMX_CCM_VBASE+IMX_CCM_CWDR_OFFSET)
#define IMX_CCM_CDHIPR             (IMX_CCM_VBASE+IMX_CCM_CDHIPR_OFFSET)
#define IMX_CCM_CLPCR              (IMX_CCM_VBASE+IMX_CCM_CLPCR_OFFSET)
#define IMX_CCM_CISR               (IMX_CCM_VBASE+IMX_CCM_CISR_OFFSET)
#define IMX_CCM_CIMR               (IMX_CCM_VBASE+IMX_CCM_CIMR_OFFSET)
#define IMX_CCM_CCOSR              (IMX_CCM_VBASE+IMX_CCM_CCOSR_OFFSET)
#define IMX_CCM_CGPR               (IMX_CCM_VBASE+IMX_CCM_CGPR_OFFSET)
#define IMX_CCM_CCGR0              (IMX_CCM_VBASE+IMX_CCM_CCGR0_OFFSET)
#define IMX_CCM_CCGR1              (IMX_CCM_VBASE+IMX_CCM_CCGR1_OFFSET)
#define IMX_CCM_CCGR2              (IMX_CCM_VBASE+IMX_CCM_CCGR2_OFFSET)
#define IMX_CCM_CCGR3              (IMX_CCM_VBASE+IMX_CCM_CCGR3_OFFSET)
#define IMX_CCM_CCGR4              (IMX_CCM_VBASE+IMX_CCM_CCGR4_OFFSET)
#define IMX_CCM_CCGR5              (IMX_CCM_VBASE+IMX_CCM_CCGR5_OFFSET)
#define IMX_CCM_CCGR6              (IMX_CCM_VBASE+IMX_CCM_CCGR6_OFFSET)
#define IMX_CCM_CMEOR              (IMX_CCM_VBASE+IMX_CCM_CMEOR_OFFSET)

/* CCM Register Bit Definitions *****************************************************/

/* CCM Control Register */
#define CCM_CCR_
/* CCM Control Divider Register */
#define CCM_CCDR_
/* CCM Status Register */
#define CCM_CSR_
/* CCM Clock Switcher Register */
#define CCM_CCSR_
/* CCM Arm Clock Root Register */
#define CCM_CACRR_
/* CCM Bus Clock Divider Register */
#define CCM_CBCDR_
/* CCM Bus Clock Multiplexer Register */
#define CCM_CBCMR_
/* CCM Serial Clock Multiplexer Register 1 */
#define CCM_CSCMR1_
/* CCM Serial Clock Multiplexer Register 2 */
#define CCM_CSCMR2_
/* CCM Serial Clock Divider Register 1 */
#define CCM_CSCDR1_
/* CCM SSI1 Clock Divider Register */
#define CCM_CS1CDR_
/* CCM SSI2 Clock Divider Register */
#define CCM_CS2CDR_
/* CCM D1 Clock Divider Register */
#define CCM_CDCDR_
/* CCM HSC Clock Divider Register */
#define CCM_CHSCCDR_
/* CCM Serial Clock Divider Register 2 */
#define CCM_CSCDR2_
/* CCM Serial Clock Divider Register 3 */
#define CCM_CSCDR3_
/* CCM Wakeup Detector Register */
#define CCM_CWDR_
/* CCM Divider Handshake In-Process Register */
#define CCM_CDHIPR_
/* CCM Low Power Control Register */
#define CCM_CLPCR_
/* CCM Interrupt Status Register */
#define CCM_CISR_
/* CCM Interrupt Mask Register */
#define CCM_CIMR_
/* CCM Clock Output Source Register */
#define CCM_CCOSR_
/* CCM General Purpose Register */
#define CCM_CGPR_
/* CCM Clock Gating Register 0 */
#define CCM_CCGR0_
/* CCM Clock Gating Register 1 */
#define CCM_CCGR1_
/* CCM Clock Gating Register 2 */
#define CCM_CCGR2_
/* CCM Clock Gating Register 3 */
#define CCM_CCGR3_
/* CCM Clock Gating Register 4 */
#define CCM_CCGR4_
/* CCM Clock Gating Register 5 */
#define CCM_CCGR5_
/* CCM Clock Gating Register 6 */
#define CCM_CCGR6_
/* CCM Module Enable Overide Register */
#define CCM_CMEOR_

#endif /* __ARCH_ARM_SRC_IMX6_CHIP_IMX_CCM_H */
