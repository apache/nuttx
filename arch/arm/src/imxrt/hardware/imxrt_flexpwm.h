/************************************************************************************
 * arch/arm/src/imxrt/hardware/imxrt_flexpwm.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_FLEXPWM_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_FLEXPWM_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "hardware/imxrt_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define IMXRT_FLEXPWM_SM0CNT_OFFSET          0x0000  /* Counter Register */
#define IMXRT_FLEXPWM_SM0INIT_OFFSET         0x0002  /* Initial Count Register */
#define IMXRT_FLEXPWM_SM0CTRL2_OFFSET        0x0004  /* Control 2 Register */
#define IMXRT_FLEXPWM_SM0CTRL_OFFSET         0x0006  /* Control Register */
#define IMXRT_FLEXPWM_SM0VAL0_OFFSET         0x000a  /* Value Register 0 */
#define IMXRT_FLEXPWM_SM0FRACVAL1_OFFSET     0x000c  /* Fractional Value Register 1 */
#define IMXRT_FLEXPWM_SM0VAL1_OFFSET         0x000e  /* Value Register 1 */
#define IMXRT_FLEXPWM_SM0FRACVAL2_OFFSET     0x0010  /* Fractional Value Register 2 */
#define IMXRT_FLEXPWM_SM0VAL2_OFFSET         0x0012  /* Value Register 2 */
#define IMXRT_FLEXPWM_SM0FRACVAL3_OFFSET     0x0014  /* Fractional Value Register 3 */
#define IMXRT_FLEXPWM_SM0VAL3_OFFSET         0x0016  /* Value Register 3 */
#define IMXRT_FLEXPWM_SM0FRACVAL4_OFFSET     0x0018  /* Fractional Value Register 4 */
#define IMXRT_FLEXPWM_SM0VAL4_OFFSET         0x001a  /* Value Register 4 */
#define IMXRT_FLEXPWM_SM0FRACVAL5_OFFSET     0x001c  /* Fractional Value Register 5 */
#define IMXRT_FLEXPWM_SM0VAL5_OFFSET         0x001e  /* Value Register 5 */
#define IMXRT_FLEXPWM_SM0FRCTRL_OFFSET       0x0020  /* Fractional Control Register */
#define IMXRT_FLEXPWM_SM0OCTRL_OFFSET        0x0022  /* Output Control Register */
#define IMXRT_FLEXPWM_SM0STS_OFFSET          0x0024  /* Status Register */
#define IMXRT_FLEXPWM_SM0INTEN_OFFSET        0x0026  /* Interrupt Enable Register */
#define IMXRT_FLEXPWM_SM0DMAEN_OFFSET        0x0028  /* DMA Enable Register */
#define IMXRT_FLEXPWM_SM0TCTRL_OFFSET        0x002a  /* Output Trigger Control Register */
#define IMXRT_FLEXPWM_SM0DISMAP0_OFFSET      0x002c  /* Fault Disable Mapping Register 0 */
#define IMXRT_FLEXPWM_SM0DISMAP1_OFFSET      0x002e  /* Fault Disable Mapping Register 1 */
#define IMXRT_FLEXPWM_SM0DTCNT0_OFFSET       0x0030  /* Deadtime Count Register 0 */
#define IMXRT_FLEXPWM_SM0DTCNT1_OFFSET       0x0032  /* Deadtime Count Register 1 */
#define IMXRT_FLEXPWM_SM0CAPTCTRLA_OFFSET    0x0034  /* Capture Control A Register */
#define IMXRT_FLEXPWM_SM0CAPTCOMPA_OFFSET    0x0036  /* Capture Compare A Register */
#define IMXRT_FLEXPWM_SM0CAPTCTRLB_OFFSET    0x0038  /* Capture Control B Register */
#define IMXRT_FLEXPWM_SM0CAPTCOMPB_OFFSET    0x003a  /* Capture Compare B Register */
#define IMXRT_FLEXPWM_SM0CAPTCTRLX_OFFSET    0x003c  /* Capture Control X Register */
#define IMXRT_FLEXPWM_SM0CAPTCOMPX_OFFSET    0x003e  /* Capture Compare X Register */
#define IMXRT_FLEXPWM_SM0CVAL0_OFFSET        0x0040  /* Capture Value 0 Register */
#define IMXRT_FLEXPWM_SM0CVAL0CYC_OFFSET     0x0042  /* Capture Value 0 Cycle Register */
#define IMXRT_FLEXPWM_SM0CVAL1_OFFSET        0x0044  /* Capture Value 1 Register */
#define IMXRT_FLEXPWM_SM0CVAL1CYC_OFFSET     0x0046  /* Capture Value 1 Cycle Register */
#define IMXRT_FLEXPWM_SM0CVAL2_OFFSET        0x0048  /* Capture Value 2 Register */
#define IMXRT_FLEXPWM_SM0CVAL2CYC_OFFSET     0x004a  /* Capture Value 2 Cycle Register */
#define IMXRT_FLEXPWM_SM0CVAL3_OFFSET        0x004c  /* Capture Value 3 Register */
#define IMXRT_FLEXPWM_SM0CVAL3CYC_OFFSET     0x004e  /* Capture Value 3 Cycle Register */
#define IMXRT_FLEXPWM_SM0CVAL4_OFFSET        0x0050  /* Capture Value 4 Register */
#define IMXRT_FLEXPWM_SM0CVAL4CYC_OFFSET     0x0052  /* Capture Value 4 Cycle Register */
#define IMXRT_FLEXPWM_SM0CVAL5_OFFSET        0x0054  /* Capture Value 5 Register */
#define IMXRT_FLEXPWM_SM0CVAL5CYC_OFFSET     0x0056  /* Capture Value 5 Cycle Register */
#define IMXRT_FLEXPWM_SM1CNT_OFFSET          0x0060  /* Counter Register */
#define IMXRT_FLEXPWM_SM1INIT_OFFSET         0x0062  /* Initial Count Register */
#define IMXRT_FLEXPWM_SM1CTRL2_OFFSET        0x0064  /* Control 2 Register */
#define IMXRT_FLEXPWM_SM1CTRL_OFFSET         0x0066  /* Control Register */
#define IMXRT_FLEXPWM_SM1VAL0_OFFSET         0x006a  /* Value Register 0 */
#define IMXRT_FLEXPWM_SM1FRACVAL1_OFFSET     0x006c  /* Fractional Value Register 1 */
#define IMXRT_FLEXPWM_SM1VAL1_OFFSET         0x006e  /* Value Register 1 */
#define IMXRT_FLEXPWM_SM1FRACVAL2_OFFSET     0x0070  /* Fractional Value Register 2 */
#define IMXRT_FLEXPWM_SM1VAL2_OFFSET         0x0072  /* Value Register 2 */
#define IMXRT_FLEXPWM_SM1FRACVAL3_OFFSET     0x0074  /* Fractional Value Register 3 */
#define IMXRT_FLEXPWM_SM1VAL3_OFFSET         0x0076  /* Value Register 3 */
#define IMXRT_FLEXPWM_SM1FRACVAL4_OFFSET     0x0078  /* Fractional Value Register 4 */
#define IMXRT_FLEXPWM_SM1VAL4_OFFSET         0x007a  /* Value Register 4 */
#define IMXRT_FLEXPWM_SM1FRACVAL5_OFFSET     0x007c  /* Fractional Value Register 5 */
#define IMXRT_FLEXPWM_SM1VAL5_OFFSET         0x007e  /* Value Register 5 */
#define IMXRT_FLEXPWM_SM1FRCTRL_OFFSET       0x0080  /* Fractional Control Register */
#define IMXRT_FLEXPWM_SM1OCTRL_OFFSET        0x0082  /* Output Control Register */
#define IMXRT_FLEXPWM_SM1STS_OFFSET          0x0084  /* Status Register */
#define IMXRT_FLEXPWM_SM1INTEN_OFFSET        0x0086  /* Interrupt Enable Register */
#define IMXRT_FLEXPWM_SM1DMAEN_OFFSET        0x0088  /* DMA Enable Register */
#define IMXRT_FLEXPWM_SM1TCTRL_OFFSET        0x008a  /* Output Trigger Control Register */
#define IMXRT_FLEXPWM_SM1DISMAP0_OFFSET      0x008c  /* Fault Disable Mapping Register 0 */
#define IMXRT_FLEXPWM_SM1DISMAP1_OFFSET      0x008e  /* Fault Disable Mapping Register 1 */
#define IMXRT_FLEXPWM_SM1DTCNT0_OFFSET       0x0090  /* Deadtime Count Register 0 */
#define IMXRT_FLEXPWM_SM1DTCNT1_OFFSET       0x0092  /* Deadtime Count Register 1 */
#define IMXRT_FLEXPWM_SM1CAPTCTRLA_OFFSET    0x0094  /* Capture Control A Register */
#define IMXRT_FLEXPWM_SM1CAPTCOMPA_OFFSET    0x0096  /* Capture Compare A Register */
#define IMXRT_FLEXPWM_eFlexPWM_OFFSET        0x000c  /* apter 28 Enhanced Flex Pulse Width Modulator */
#define IMXRT_FLEXPWM_SM1CAPTCTRLB_OFFSET    0x0098  /* Capture Control B Register */
#define IMXRT_FLEXPWM_SM1CAPTCOMPB_OFFSET    0x009a  /* Capture Compare B Register */
#define IMXRT_FLEXPWM_SM1CAPTCTRLX_OFFSET    0x009c  /* Capture Control X Register */
#define IMXRT_FLEXPWM_SM1CAPTCOMPX_OFFSET    0x009e  /* Capture Compare X Register */
#define IMXRT_FLEXPWM_SM1CVAL0_OFFSET        0x00a0  /* Capture Value 0 Register */
#define IMXRT_FLEXPWM_SM1CVAL0CYC_OFFSET     0x00a2  /* Capture Value 0 Cycle Register */
#define IMXRT_FLEXPWM_SM1CVAL1_OFFSET        0x00a4  /* Capture Value 1 Register */
#define IMXRT_FLEXPWM_SM1CVAL1CYC_OFFSET     0x00a6  /* Capture Value 1 Cycle Register */
#define IMXRT_FLEXPWM_SM1CVAL2_OFFSET        0x00a8  /* Capture Value 2 Register */
#define IMXRT_FLEXPWM_SM1CVAL2CYC_OFFSET     0x00aa  /* Capture Value 2 Cycle Register */
#define IMXRT_FLEXPWM_SM1CVAL3_OFFSET        0x00ac  /* Capture Value 3 Register */
#define IMXRT_FLEXPWM_SM1CVAL3CYC_OFFSET     0x00ae  /* Capture Value 3 Cycle Register */
#define IMXRT_FLEXPWM_SM1CVAL4_OFFSET        0x00b0  /* Capture Value 4 Register */
#define IMXRT_FLEXPWM_SM1CVAL4CYC_OFFSET     0x00b2  /* Capture Value 4 Cycle Register */
#define IMXRT_FLEXPWM_SM1CVAL5_OFFSET        0x00b4  /* Capture Value 5 Register */
#define IMXRT_FLEXPWM_SM1CVAL5CYC_OFFSET     0x00b6  /* Capture Value 5 Cycle Register */
#define IMXRT_FLEXPWM_SM2CNT_OFFSET          0x00c0  /* Counter Register */
#define IMXRT_FLEXPWM_SM2INIT_OFFSET         0x00c2  /* Initial Count Register */
#define IMXRT_FLEXPWM_SM2CTRL2_OFFSET        0x00c4  /* Control 2 Register */
#define IMXRT_FLEXPWM_SM2CTRL_OFFSET         0x00c6  /* Control Register */
#define IMXRT_FLEXPWM_SM2VAL0_OFFSET         0x00ca  /* Value Register 0 */
#define IMXRT_FLEXPWM_SM2FRACVAL1_OFFSET     0x00cc  /* Fractional Value Register 1 */
#define IMXRT_FLEXPWM_SM2VAL1_OFFSET         0x00ce  /* Value Register 1 */
#define IMXRT_FLEXPWM_SM2FRACVAL2_OFFSET     0x00d0  /* Fractional Value Register 2 */
#define IMXRT_FLEXPWM_SM2VAL2_OFFSET         0x00d2  /* Value Register 2 */
#define IMXRT_FLEXPWM_SM2FRACVAL3_OFFSET     0x00d4  /* Fractional Value Register 3 */
#define IMXRT_FLEXPWM_SM2VAL3_OFFSET         0x00d6  /* Value Register 3 */
#define IMXRT_FLEXPWM_SM2FRACVAL4_OFFSET     0x00d8  /* Fractional Value Register 4 */
#define IMXRT_FLEXPWM_SM2VAL4_OFFSET         0x00da  /* Value Register 4 */
#define IMXRT_FLEXPWM_SM2FRACVAL5_OFFSET     0x00dc  /* Fractional Value Register 5 */
#define IMXRT_FLEXPWM_SM2VAL5_OFFSET         0x00de  /* Value Register 5 */
#define IMXRT_FLEXPWM_SM2FRCTRL_OFFSET       0x00e0  /* Fractional Control Register */
#define IMXRT_FLEXPWM_SM2OCTRL_OFFSET        0x00e2  /* Output Control Register */
#define IMXRT_FLEXPWM_SM2STS_OFFSET          0x00e4  /* Status Register */
#define IMXRT_FLEXPWM_SM2INTEN_OFFSET        0x00e6  /* Interrupt Enable Register */
#define IMXRT_FLEXPWM_SM2DMAEN_OFFSET        0x00e8  /* DMA Enable Register */
#define IMXRT_FLEXPWM_SM2TCTRL_OFFSET        0x00ea  /* Output Trigger Control Register */
#define IMXRT_FLEXPWM_SM2DISMAP0_OFFSET      0x00ec  /* Fault Disable Mapping Register 0 */
#define IMXRT_FLEXPWM_SM2DISMAP1_OFFSET      0x00ee  /* Fault Disable Mapping Register 1 */
#define IMXRT_FLEXPWM_SM2DTCNT0_OFFSET       0x00f0  /* Deadtime Count Register 0 */
#define IMXRT_FLEXPWM_SM2DTCNT1_OFFSET       0x00f2  /* Deadtime Count Register 1 */
#define IMXRT_FLEXPWM_SM2CAPTCTRLA_OFFSET    0x00f4  /* Capture Control A Register */
#define IMXRT_FLEXPWM_SM2CAPTCOMPA_OFFSET    0x00f6  /* Capture Compare A Register */
#define IMXRT_FLEXPWM_SM2CAPTCTRLB_OFFSET    0x00f8  /* Capture Control B Register */
#define IMXRT_FLEXPWM_SM2CAPTCOMPB_OFFSET    0x00fa  /* Capture Compare B Register */
#define IMXRT_FLEXPWM_SM2CAPTCTRLX_OFFSET    0x00fc  /* Capture Control X Register */
#define IMXRT_FLEXPWM_SM2CAPTCOMPX_OFFSET    0x00fe  /* Capture Compare X Register */
#define IMXRT_FLEXPWM_SM2CVAL0_OFFSET        0x0100  /* Capture Value 0 Register */
#define IMXRT_FLEXPWM_SM2CVAL0CYC_OFFSET     0x0102  /* Capture Value 0 Cycle Register */
#define IMXRT_FLEXPWM_SM2CVAL1_OFFSET        0x0104  /* Capture Value 1 Register */
#define IMXRT_FLEXPWM_SM2CVAL1CYC_OFFSET     0x0106  /* Capture Value 1 Cycle Register */
#define IMXRT_FLEXPWM_SM2CVAL2_OFFSET        0x0108  /* Capture Value 2 Register */
#define IMXRT_FLEXPWM_SM2CVAL2CYC_OFFSET     0x010a  /* Capture Value 2 Cycle Register */
#define IMXRT_FLEXPWM_SM2CVAL3_OFFSET        0x010c  /* Capture Value 3 Register */
#define IMXRT_FLEXPWM_SM2CVAL3CYC_OFFSET     0x010e  /* Capture Value 3 Cycle Register */
#define IMXRT_FLEXPWM_SM2CVAL4_OFFSET        0x0110  /* Capture Value 4 Register */
#define IMXRT_FLEXPWM_SM2CVAL4CYC_OFFSET     0x0112  /* Capture Value 4 Cycle Register */
#define IMXRT_FLEXPWM_SM2CVAL5_OFFSET        0x0114  /* Capture Value 5 Register */
#define IMXRT_FLEXPWM_SM2CVAL5CYC_OFFSET     0x0116  /* Capture Value 5 Cycle Register */
#define IMXRT_FLEXPWM_SM3CNT_OFFSET          0x0120  /* Counter Register */
#define IMXRT_FLEXPWM_SM3INIT_OFFSET         0x0122  /* Initial Count Register */
#define IMXRT_FLEXPWM_SM3CTRL2_OFFSET        0x0124  /* Control 2 Register */
#define IMXRT_FLEXPWM_SM3CTRL_OFFSET         0x0126  /* Control Register */
#define IMXRT_FLEXPWM_SM3VAL0_OFFSET         0x012a  /* Value Register 0 */
#define IMXRT_FLEXPWM_SM3FRACVAL1_OFFSET     0x012c  /* Fractional Value Register 1 */
#define IMXRT_FLEXPWM_SM3VAL1_OFFSET         0x012e  /* Value Register 1 */
#define IMXRT_FLEXPWM_SM3FRACVAL2_OFFSET     0x0130  /* Fractional Value Register 2 */
#define IMXRT_FLEXPWM_SM3VAL2_OFFSET         0x0132  /* Value Register 2 */
#define IMXRT_FLEXPWM_SM3FRACVAL3_OFFSET     0x0134  /* Fractional Value Register 3 */
#define IMXRT_FLEXPWM_SM3VAL3_OFFSET         0x0136  /* Value Register 3 */
#define IMXRT_FLEXPWM_SM3FRACVAL4_OFFSET     0x0138  /* Fractional Value Register 4 */
#define IMXRT_FLEXPWM_SM3VAL4_OFFSET         0x013a  /* Value Register 4 */
#define IMXRT_FLEXPWM_SM3FRACVAL5_OFFSET     0x013c  /* Fractional Value Register 5 */
#define IMXRT_FLEXPWM_SM3VAL5_OFFSET         0x013e  /* Value Register 5 */
#define IMXRT_FLEXPWM_SM3FRCTRL_OFFSET       0x0140  /* Fractional Control Register */
#define IMXRT_FLEXPWM_SM3OCTRL_OFFSET        0x0142  /* Output Control Register */
#define IMXRT_FLEXPWM_SM3STS_OFFSET          0x0144  /* Status Register */
#define IMXRT_FLEXPWM_SM3INTEN_OFFSET        0x0146  /* Interrupt Enable Register */
#define IMXRT_FLEXPWM_SM3DMAEN_OFFSET        0x0148  /* DMA Enable Register */
#define IMXRT_FLEXPWM_SM3TCTRL_OFFSET        0x014a  /* Output Trigger Control Register */
#define IMXRT_FLEXPWM_SM3DISMAP0_OFFSET      0x014c  /* Fault Disable Mapping Register 0 */
#define IMXRT_FLEXPWM_SM3DISMAP1_OFFSET      0x014e  /* Fault Disable Mapping Register 1 */
#define IMXRT_FLEXPWM_SM3DTCNT0_OFFSET       0x0150  /* Deadtime Count Register 0 */
#define IMXRT_FLEXPWM_SM3DTCNT1_OFFSET       0x0152  /* Deadtime Count Register 1 */
#define IMXRT_FLEXPWM_SM3CAPTCTRLA_OFFSET    0x0154  /* Capture Control A Register */
#define IMXRT_FLEXPWM_SM3CAPTCOMPA_OFFSET    0x0156  /* Capture Compare A Register */
#define IMXRT_FLEXPWM_SM3CAPTCTRLB_OFFSET    0x0158  /* Capture Control B Register */
#define IMXRT_FLEXPWM_SM3CAPTCOMPB_OFFSET    0x015a  /* Capture Compare B Register */
#define IMXRT_FLEXPWM_SM3CAPTCTRLX_OFFSET    0x015c  /* Capture Control X Register */
#define IMXRT_FLEXPWM_SM3CAPTCOMPX_OFFSET    0x015e  /* Capture Compare X Register */
#define IMXRT_FLEXPWM_SM3CVAL0_OFFSET        0x0160  /* Capture Value 0 Register */
#define IMXRT_FLEXPWM_SM3CVAL0CYC_OFFSET     0x0162  /* Capture Value 0 Cycle Register */
#define IMXRT_FLEXPWM_SM3CVAL1_OFFSET        0x0164  /* Capture Value 1 Register */
#define IMXRT_FLEXPWM_SM3CVAL1CYC_OFFSET     0x0166  /* Capture Value 1 Cycle Register */
#define IMXRT_FLEXPWM_SM3CVAL2_OFFSET        0x0168  /* Capture Value 2 Register */
#define IMXRT_FLEXPWM_SM3CVAL2CYC_OFFSET     0x016a  /* Capture Value 2 Cycle Register */
#define IMXRT_FLEXPWM_SM3CVAL3_OFFSET        0x016c  /* Capture Value 3 Register */
#define IMXRT_FLEXPWM_SM3CVAL3CYC_OFFSET     0x016e  /* Capture Value 3 Cycle Register */
#define IMXRT_FLEXPWM_SM3CVAL4_OFFSET        0x0170  /* Capture Value 4 Register */
#define IMXRT_FLEXPWM_SM3CVAL4CYC_OFFSET     0x0172  /* Capture Value 4 Cycle Register */
#define IMXRT_FLEXPWM_SM3CVAL5_OFFSET        0x0174  /* Capture Value 5 Register */
#define IMXRT_FLEXPWM_SM3CVAL5CYC_OFFSET     0x0176  /* Capture Value 5 Cycle Register */
#define IMXRT_FLEXPWM_OUTEN_OFFSET           0x0180  /* Output Enable Register */
#define IMXRT_FLEXPWM_MASK_OFFSET            0x0182  /* Mask Register */
#define IMXRT_FLEXPWM_SWCOUT_OFFSET          0x0184  /* Software Controlled Output Register */
#define IMXRT_FLEXPWM_DTSRCSEL_OFFSET        0x0186  /* PWM Source Select Register */
#define IMXRT_FLEXPWM_MCTRL_OFFSET           0x0188  /* Master Control Register */
#define IMXRT_FLEXPWM_MCTRL2_OFFSET          0x018a  /* Master Control 2 Register */
#define IMXRT_FLEXPWM_FCTRL0_OFFSET          0x018c  /* Fault Control Register */
#define IMXRT_FLEXPWM_FSTS0_OFFSET           0x018e  /* Fault Status Register */
#define IMXRT_FLEXPWM_FFILT0_OFFSET          0x0190  /* Fault Filter Register */
#define IMXRT_FLEXPWM_FTST0_OFFSET           0x0192  /* Fault Test Register */
#define IMXRT_FLEXPWM_FCTRL20_OFFSET         0x0194  /* Fault Control 2 Register */

/* Register addresses ***********************************************************************/

/* FLEXPWM1 Register Addresses */

#define IMXRT_FLEXPWM1_SM0CNT                (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM0CNT_OFFSET)        /* FLEXPWM1 Counter Register */
#define IMXRT_FLEXPWM1_SM0INIT               (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM0INIT_OFFSET)       /* FLEXPWM1 Initial Count Register */
#define IMXRT_FLEXPWM1_SM0CTRL2              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM0CTRL2_OFFSET)      /* FLEXPWM1 Control 2 Register */
#define IMXRT_FLEXPWM1_SM0CTRL               (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM0CTRL_OFFSET)       /* FLEXPWM1 Control Register */
#define IMXRT_FLEXPWM1_SM0VAL0               (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM0VAL0_OFFSET)       /* FLEXPWM1 Value Register 0 */
#define IMXRT_FLEXPWM1_SM0FRACVAL1           (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM0FRACVAL1_OFFSET)   /* FLEXPWM1 Fractional Value Register 1 */
#define IMXRT_FLEXPWM1_SM0VAL1               (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM0VAL1_OFFSET)       /* FLEXPWM1 Value Register 1 */
#define IMXRT_FLEXPWM1_SM0FRACVAL2           (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM0FRACVAL2_OFFSET)   /* FLEXPWM1 Fractional Value Register 2 */
#define IMXRT_FLEXPWM1_SM0VAL2               (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM0VAL2_OFFSET)       /* FLEXPWM1 Value Register 2 */
#define IMXRT_FLEXPWM1_SM0FRACVAL3           (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM0FRACVAL3_OFFSET)   /* FLEXPWM1 Fractional Value Register 3 */
#define IMXRT_FLEXPWM1_SM0VAL3               (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM0VAL3_OFFSET)       /* FLEXPWM1 Value Register 3 */
#define IMXRT_FLEXPWM1_SM0FRACVAL4           (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM0FRACVAL4_OFFSET)   /* FLEXPWM1 Fractional Value Register 4 */
#define IMXRT_FLEXPWM1_SM0VAL4               (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM0VAL4_OFFSET)       /* FLEXPWM1 Value Register 4 */
#define IMXRT_FLEXPWM1_SM0FRACVAL5           (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM0FRACVAL5_OFFSET)   /* FLEXPWM1 Fractional Value Register 5 */
#define IMXRT_FLEXPWM1_SM0VAL5               (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM0VAL5_OFFSET)       /* FLEXPWM1 Value Register 5 */
#define IMXRT_FLEXPWM1_SM0FRCTRL             (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM0FRCTRL_OFFSET)     /* FLEXPWM1 Fractional Control Register */
#define IMXRT_FLEXPWM1_SM0OCTRL              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM0OCTRL_OFFSET)      /* FLEXPWM1 Output Control Register */
#define IMXRT_FLEXPWM1_SM0STS                (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM0STS_OFFSET)        /* FLEXPWM1 Status Register */
#define IMXRT_FLEXPWM1_SM0INTEN              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM0INTEN_OFFSET)      /* FLEXPWM1 Interrupt Enable Register */
#define IMXRT_FLEXPWM1_SM0DMAEN              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM0DMAEN_OFFSET)      /* FLEXPWM1 DMA Enable Register */
#define IMXRT_FLEXPWM1_SM0TCTRL              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM0TCTRL_OFFSET)      /* FLEXPWM1 Output Trigger Control Register */
#define IMXRT_FLEXPWM1_SM0DISMAP0            (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM0DISMAP0_OFFSET)    /* FLEXPWM1 Fault Disable Mapping Register 0 */
#define IMXRT_FLEXPWM1_SM0DISMAP1            (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM0DISMAP1_OFFSET)    /* FLEXPWM1 Fault Disable Mapping Register 1 */
#define IMXRT_FLEXPWM1_SM0DTCNT0             (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM0DTCNT0_OFFSET)     /* FLEXPWM1 Deadtime Count Register 0 */
#define IMXRT_FLEXPWM1_SM0DTCNT1             (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM0DTCNT1_OFFSET)     /* FLEXPWM1 Deadtime Count Register 1 */
#define IMXRT_FLEXPWM1_SM0CAPTCTRLA          (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM0CAPTCTRLA_OFFSET)  /* FLEXPWM1 Capture Control A Register */
#define IMXRT_FLEXPWM1_SM0CAPTCOMPA          (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM0CAPTCOMPA_OFFSET)  /* FLEXPWM1 Capture Compare A Register */
#define IMXRT_FLEXPWM1_SM0CAPTCTRLB          (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM0CAPTCTRLB_OFFSET)  /* FLEXPWM1 Capture Control B Register */
#define IMXRT_FLEXPWM1_SM0CAPTCOMPB          (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM0CAPTCOMPB_OFFSET)  /* FLEXPWM1 Capture Compare B Register */
#define IMXRT_FLEXPWM1_SM0CAPTCTRLX          (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM0CAPTCTRLX_OFFSET)  /* FLEXPWM1 Capture Control X Register */
#define IMXRT_FLEXPWM1_SM0CAPTCOMPX          (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM0CAPTCOMPX_OFFSET)  /* FLEXPWM1 Capture Compare X Register */
#define IMXRT_FLEXPWM1_SM0CVAL0              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM0CVAL0_OFFSET)      /* FLEXPWM1 Capture Value 0 Register */
#define IMXRT_FLEXPWM1_SM0CVAL0CYC           (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM0CVAL0CYC_OFFSET)   /* FLEXPWM1 Capture Value 0 Cycle Register */
#define IMXRT_FLEXPWM1_SM0CVAL1              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM0CVAL1_OFFSET)      /* FLEXPWM1 Capture Value 1 Register */
#define IMXRT_FLEXPWM1_SM0CVAL1CYC           (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM0CVAL1CYC_OFFSET)   /* FLEXPWM1 Capture Value 1 Cycle Register */
#define IMXRT_FLEXPWM1_SM0CVAL2              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM0CVAL2_OFFSET)      /* FLEXPWM1 Capture Value 2 Register */
#define IMXRT_FLEXPWM1_SM0CVAL2CYC           (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM0CVAL2CYC_OFFSET)   /* FLEXPWM1 Capture Value 2 Cycle Register */
#define IMXRT_FLEXPWM1_SM0CVAL3              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM0CVAL3_OFFSET)      /* FLEXPWM1 Capture Value 3 Register */
#define IMXRT_FLEXPWM1_SM0CVAL3CYC           (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM0CVAL3CYC_OFFSET)   /* FLEXPWM1 Capture Value 3 Cycle Register */
#define IMXRT_FLEXPWM1_SM0CVAL4              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM0CVAL4_OFFSET)      /* FLEXPWM1 Capture Value 4 Register */
#define IMXRT_FLEXPWM1_SM0CVAL4CYC           (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM0CVAL4CYC_OFFSET)   /* FLEXPWM1 Capture Value 4 Cycle Register */
#define IMXRT_FLEXPWM1_SM0CVAL5              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM0CVAL5_OFFSET)      /* FLEXPWM1 Capture Value 5 Register */
#define IMXRT_FLEXPWM1_SM0CVAL5CYC           (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM0CVAL5CYC_OFFSET)   /* FLEXPWM1 Capture Value 5 Cycle Register */
#define IMXRT_FLEXPWM1_SM1CNT                (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM1CNT_OFFSET)        /* FLEXPWM1 Counter Register */
#define IMXRT_FLEXPWM1_SM1INIT               (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM1INIT_OFFSET)       /* FLEXPWM1 Initial Count Register */
#define IMXRT_FLEXPWM1_SM1CTRL2              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM1CTRL2_OFFSET)      /* FLEXPWM1 Control 2 Register */
#define IMXRT_FLEXPWM1_SM1CTRL               (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM1CTRL_OFFSET)       /* FLEXPWM1 Control Register */
#define IMXRT_FLEXPWM1_SM1VAL0               (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM1VAL0_OFFSET)       /* FLEXPWM1 Value Register 0 */
#define IMXRT_FLEXPWM1_SM1FRACVAL1           (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM1FRACVAL1_OFFSET)   /* FLEXPWM1 Fractional Value Register 1 */
#define IMXRT_FLEXPWM1_SM1VAL1               (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM1VAL1_OFFSET)       /* FLEXPWM1 Value Register 1 */
#define IMXRT_FLEXPWM1_SM1FRACVAL2           (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM1FRACVAL2_OFFSET)   /* FLEXPWM1 Fractional Value Register 2 */
#define IMXRT_FLEXPWM1_SM1VAL2               (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM1VAL2_OFFSET)       /* FLEXPWM1 Value Register 2 */
#define IMXRT_FLEXPWM1_SM1FRACVAL3           (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM1FRACVAL3_OFFSET)   /* FLEXPWM1 Fractional Value Register 3 */
#define IMXRT_FLEXPWM1_SM1VAL3               (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM1VAL3_OFFSET)       /* FLEXPWM1 Value Register 3 */
#define IMXRT_FLEXPWM1_SM1FRACVAL4           (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM1FRACVAL4_OFFSET)   /* FLEXPWM1 Fractional Value Register 4 */
#define IMXRT_FLEXPWM1_SM1VAL4               (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM1VAL4_OFFSET)       /* FLEXPWM1 Value Register 4 */
#define IMXRT_FLEXPWM1_SM1FRACVAL5           (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM1FRACVAL5_OFFSET)   /* FLEXPWM1 Fractional Value Register 5 */
#define IMXRT_FLEXPWM1_SM1VAL5               (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM1VAL5_OFFSET)       /* FLEXPWM1 Value Register 5 */
#define IMXRT_FLEXPWM1_SM1FRCTRL             (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM1FRCTRL_OFFSET)     /* FLEXPWM1 Fractional Control Register */
#define IMXRT_FLEXPWM1_SM1OCTRL              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM1OCTRL_OFFSET)      /* FLEXPWM1 Output Control Register */
#define IMXRT_FLEXPWM1_SM1STS                (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM1STS_OFFSET)        /* FLEXPWM1 Status Register */
#define IMXRT_FLEXPWM1_SM1INTEN              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM1INTEN_OFFSET)      /* FLEXPWM1 Interrupt Enable Register */
#define IMXRT_FLEXPWM1_SM1DMAEN              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM1DMAEN_OFFSET)      /* FLEXPWM1 DMA Enable Register */
#define IMXRT_FLEXPWM1_SM1TCTRL              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM1TCTRL_OFFSET)      /* FLEXPWM1 Output Trigger Control Register */
#define IMXRT_FLEXPWM1_SM1DISMAP0            (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM1DISMAP0_OFFSET)    /* FLEXPWM1 Fault Disable Mapping Register 0 */
#define IMXRT_FLEXPWM1_SM1DISMAP1            (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM1DISMAP1_OFFSET)    /* FLEXPWM1 Fault Disable Mapping Register 1 */
#define IMXRT_FLEXPWM1_SM1DTCNT0             (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM1DTCNT0_OFFSET)     /* FLEXPWM1 Deadtime Count Register 0 */
#define IMXRT_FLEXPWM1_SM1DTCNT1             (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM1DTCNT1_OFFSET)     /* FLEXPWM1 Deadtime Count Register 1 */
#define IMXRT_FLEXPWM1_SM1CAPTCTRLA          (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM1CAPTCTRLA_OFFSET)  /* FLEXPWM1 Capture Control A Register */
#define IMXRT_FLEXPWM1_SM1CAPTCOMPA          (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM1CAPTCOMPA_OFFSET)  /* FLEXPWM1 Capture Compare A Register */
#define IMXRT_FLEXPWM1_eFlexPWM              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_eFlexPWM_OFFSET)      /* FLEXPWM1 apter 28 Enhanced Flex Pulse Width Modulator */
#define IMXRT_FLEXPWM1_SM1CAPTCTRLB          (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM1CAPTCTRLB_OFFSET)  /* FLEXPWM1 Capture Control B Register */
#define IMXRT_FLEXPWM1_SM1CAPTCOMPB          (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM1CAPTCOMPB_OFFSET)  /* FLEXPWM1 Capture Compare B Register */
#define IMXRT_FLEXPWM1_SM1CAPTCTRLX          (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM1CAPTCTRLX_OFFSET)  /* FLEXPWM1 Capture Control X Register */
#define IMXRT_FLEXPWM1_SM1CAPTCOMPX          (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM1CAPTCOMPX_OFFSET)  /* FLEXPWM1 Capture Compare X Register */
#define IMXRT_FLEXPWM1_SM1CVAL0              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM1CVAL0_OFFSET)      /* FLEXPWM1 Capture Value 0 Register */
#define IMXRT_FLEXPWM1_SM1CVAL0CYC           (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM1CVAL0CYC_OFFSET)   /* FLEXPWM1 Capture Value 0 Cycle Register */
#define IMXRT_FLEXPWM1_SM1CVAL1              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM1CVAL1_OFFSET)      /* FLEXPWM1 Capture Value 1 Register */
#define IMXRT_FLEXPWM1_SM1CVAL1CYC           (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM1CVAL1CYC_OFFSET)   /* FLEXPWM1 Capture Value 1 Cycle Register */
#define IMXRT_FLEXPWM1_SM1CVAL2              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM1CVAL2_OFFSET)      /* FLEXPWM1 Capture Value 2 Register */
#define IMXRT_FLEXPWM1_SM1CVAL2CYC           (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM1CVAL2CYC_OFFSET)   /* FLEXPWM1 Capture Value 2 Cycle Register */
#define IMXRT_FLEXPWM1_SM1CVAL3              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM1CVAL3_OFFSET)      /* FLEXPWM1 Capture Value 3 Register */
#define IMXRT_FLEXPWM1_SM1CVAL3CYC           (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM1CVAL3CYC_OFFSET)   /* FLEXPWM1 Capture Value 3 Cycle Register */
#define IMXRT_FLEXPWM1_SM1CVAL4              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM1CVAL4_OFFSET)      /* FLEXPWM1 Capture Value 4 Register */
#define IMXRT_FLEXPWM1_SM1CVAL4CYC           (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM1CVAL4CYC_OFFSET)   /* FLEXPWM1 Capture Value 4 Cycle Register */
#define IMXRT_FLEXPWM1_SM1CVAL5              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM1CVAL5_OFFSET)      /* FLEXPWM1 Capture Value 5 Register */
#define IMXRT_FLEXPWM1_SM1CVAL5CYC           (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM1CVAL5CYC_OFFSET)   /* FLEXPWM1 Capture Value 5 Cycle Register */
#define IMXRT_FLEXPWM1_SM2CNT                (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM2CNT_OFFSET)        /* FLEXPWM1 Counter Register */
#define IMXRT_FLEXPWM1_SM2INIT               (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM2INIT_OFFSET)       /* FLEXPWM1 Initial Count Register */
#define IMXRT_FLEXPWM1_SM2CTRL2              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM2CTRL2_OFFSET)      /* FLEXPWM1 Control 2 Register */
#define IMXRT_FLEXPWM1_SM2CTRL               (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM2CTRL_OFFSET)       /* FLEXPWM1 Control Register */
#define IMXRT_FLEXPWM1_SM2VAL0               (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM2VAL0_OFFSET)       /* FLEXPWM1 Value Register 0 */
#define IMXRT_FLEXPWM1_SM2FRACVAL1           (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM2FRACVAL1_OFFSET)   /* FLEXPWM1 Fractional Value Register 1 */
#define IMXRT_FLEXPWM1_SM2VAL1               (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM2VAL1_OFFSET)       /* FLEXPWM1 Value Register 1 */
#define IMXRT_FLEXPWM1_SM2FRACVAL2           (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM2FRACVAL2_OFFSET)   /* FLEXPWM1 Fractional Value Register 2 */
#define IMXRT_FLEXPWM1_SM2VAL2               (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM2VAL2_OFFSET)       /* FLEXPWM1 Value Register 2 */
#define IMXRT_FLEXPWM1_SM2FRACVAL3           (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM2FRACVAL3_OFFSET)   /* FLEXPWM1 Fractional Value Register 3 */
#define IMXRT_FLEXPWM1_SM2VAL3               (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM2VAL3_OFFSET)       /* FLEXPWM1 Value Register 3 */
#define IMXRT_FLEXPWM1_SM2FRACVAL4           (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM2FRACVAL4_OFFSET)   /* FLEXPWM1 Fractional Value Register 4 */
#define IMXRT_FLEXPWM1_SM2VAL4               (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM2VAL4_OFFSET)       /* FLEXPWM1 Value Register 4 */
#define IMXRT_FLEXPWM1_SM2FRACVAL5           (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM2FRACVAL5_OFFSET)   /* FLEXPWM1 Fractional Value Register 5 */
#define IMXRT_FLEXPWM1_SM2VAL5               (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM2VAL5_OFFSET)       /* FLEXPWM1 Value Register 5 */
#define IMXRT_FLEXPWM1_SM2FRCTRL             (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM2FRCTRL_OFFSET)     /* FLEXPWM1 Fractional Control Register */
#define IMXRT_FLEXPWM1_SM2OCTRL              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM2OCTRL_OFFSET)      /* FLEXPWM1 Output Control Register */
#define IMXRT_FLEXPWM1_SM2STS                (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM2STS_OFFSET)        /* FLEXPWM1 Status Register */
#define IMXRT_FLEXPWM1_SM2INTEN              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM2INTEN_OFFSET)      /* FLEXPWM1 Interrupt Enable Register */
#define IMXRT_FLEXPWM1_SM2DMAEN              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM2DMAEN_OFFSET)      /* FLEXPWM1 DMA Enable Register */
#define IMXRT_FLEXPWM1_SM2TCTRL              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM2TCTRL_OFFSET)      /* FLEXPWM1 Output Trigger Control Register */
#define IMXRT_FLEXPWM1_SM2DISMAP0            (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM2DISMAP0_OFFSET)    /* FLEXPWM1 Fault Disable Mapping Register 0 */
#define IMXRT_FLEXPWM1_SM2DISMAP1            (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM2DISMAP1_OFFSET)    /* FLEXPWM1 Fault Disable Mapping Register 1 */
#define IMXRT_FLEXPWM1_SM2DTCNT0             (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM2DTCNT0_OFFSET)     /* FLEXPWM1 Deadtime Count Register 0 */
#define IMXRT_FLEXPWM1_SM2DTCNT1             (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM2DTCNT1_OFFSET)     /* FLEXPWM1 Deadtime Count Register 1 */
#define IMXRT_FLEXPWM1_SM2CAPTCTRLA          (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM2CAPTCTRLA_OFFSET)  /* FLEXPWM1 Capture Control A Register */
#define IMXRT_FLEXPWM1_SM2CAPTCOMPA          (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM2CAPTCOMPA_OFFSET)  /* FLEXPWM1 Capture Compare A Register */
#define IMXRT_FLEXPWM1_SM2CAPTCTRLB          (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM2CAPTCTRLB_OFFSET)  /* FLEXPWM1 Capture Control B Register */
#define IMXRT_FLEXPWM1_SM2CAPTCOMPB          (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM2CAPTCOMPB_OFFSET)  /* FLEXPWM1 Capture Compare B Register */
#define IMXRT_FLEXPWM1_SM2CAPTCTRLX          (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM2CAPTCTRLX_OFFSET)  /* FLEXPWM1 Capture Control X Register */
#define IMXRT_FLEXPWM1_SM2CAPTCOMPX          (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM2CAPTCOMPX_OFFSET)  /* FLEXPWM1 Capture Compare X Register */
#define IMXRT_FLEXPWM1_SM2CVAL0              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM2CVAL0_OFFSET)      /* FLEXPWM1 Capture Value 0 Register */
#define IMXRT_FLEXPWM1_SM2CVAL0CYC           (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM2CVAL0CYC_OFFSET)   /* FLEXPWM1 Capture Value 0 Cycle Register */
#define IMXRT_FLEXPWM1_SM2CVAL1              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM2CVAL1_OFFSET)      /* FLEXPWM1 Capture Value 1 Register */
#define IMXRT_FLEXPWM1_SM2CVAL1CYC           (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM2CVAL1CYC_OFFSET)   /* FLEXPWM1 Capture Value 1 Cycle Register */
#define IMXRT_FLEXPWM1_SM2CVAL2              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM2CVAL2_OFFSET)      /* FLEXPWM1 Capture Value 2 Register */
#define IMXRT_FLEXPWM1_SM2CVAL2CYC           (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM2CVAL2CYC_OFFSET)   /* FLEXPWM1 Capture Value 2 Cycle Register */
#define IMXRT_FLEXPWM1_SM2CVAL3              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM2CVAL3_OFFSET)      /* FLEXPWM1 Capture Value 3 Register */
#define IMXRT_FLEXPWM1_SM2CVAL3CYC           (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM2CVAL3CYC_OFFSET)   /* FLEXPWM1 Capture Value 3 Cycle Register */
#define IMXRT_FLEXPWM1_SM2CVAL4              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM2CVAL4_OFFSET)      /* FLEXPWM1 Capture Value 4 Register */
#define IMXRT_FLEXPWM1_SM2CVAL4CYC           (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM2CVAL4CYC_OFFSET)   /* FLEXPWM1 Capture Value 4 Cycle Register */
#define IMXRT_FLEXPWM1_SM2CVAL5              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM2CVAL5_OFFSET)      /* FLEXPWM1 Capture Value 5 Register */
#define IMXRT_FLEXPWM1_SM2CVAL5CYC           (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM2CVAL5CYC_OFFSET)   /* FLEXPWM1 Capture Value 5 Cycle Register */
#define IMXRT_FLEXPWM1_SM3CNT                (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM3CNT_OFFSET)        /* FLEXPWM1 Counter Register */
#define IMXRT_FLEXPWM1_SM3INIT               (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM3INIT_OFFSET)       /* FLEXPWM1 Initial Count Register */
#define IMXRT_FLEXPWM1_SM3CTRL2              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM3CTRL2_OFFSET)      /* FLEXPWM1 Control 2 Register */
#define IMXRT_FLEXPWM1_SM3CTRL               (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM3CTRL_OFFSET)       /* FLEXPWM1 Control Register */
#define IMXRT_FLEXPWM1_SM3VAL0               (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM3VAL0_OFFSET)       /* FLEXPWM1 Value Register 0 */
#define IMXRT_FLEXPWM1_SM3FRACVAL1           (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM3FRACVAL1_OFFSET)   /* FLEXPWM1 Fractional Value Register 1 */
#define IMXRT_FLEXPWM1_SM3VAL1               (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM3VAL1_OFFSET)       /* FLEXPWM1 Value Register 1 */
#define IMXRT_FLEXPWM1_SM3FRACVAL2           (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM3FRACVAL2_OFFSET)   /* FLEXPWM1 Fractional Value Register 2 */
#define IMXRT_FLEXPWM1_SM3VAL2               (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM3VAL2_OFFSET)       /* FLEXPWM1 Value Register 2 */
#define IMXRT_FLEXPWM1_SM3FRACVAL3           (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM3FRACVAL3_OFFSET)   /* FLEXPWM1 Fractional Value Register 3 */
#define IMXRT_FLEXPWM1_SM3VAL3               (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM3VAL3_OFFSET)       /* FLEXPWM1 Value Register 3 */
#define IMXRT_FLEXPWM1_SM3FRACVAL4           (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM3FRACVAL4_OFFSET)   /* FLEXPWM1 Fractional Value Register 4 */
#define IMXRT_FLEXPWM1_SM3VAL4               (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM3VAL4_OFFSET)       /* FLEXPWM1 Value Register 4 */
#define IMXRT_FLEXPWM1_SM3FRACVAL5           (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM3FRACVAL5_OFFSET)   /* FLEXPWM1 Fractional Value Register 5 */
#define IMXRT_FLEXPWM1_SM3VAL5               (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM3VAL5_OFFSET)       /* FLEXPWM1 Value Register 5 */
#define IMXRT_FLEXPWM1_SM3FRCTRL             (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM3FRCTRL_OFFSET)     /* FLEXPWM1 Fractional Control Register */
#define IMXRT_FLEXPWM1_SM3OCTRL              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM3OCTRL_OFFSET)      /* FLEXPWM1 Output Control Register */
#define IMXRT_FLEXPWM1_SM3STS                (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM3STS_OFFSET)        /* FLEXPWM1 Status Register */
#define IMXRT_FLEXPWM1_SM3INTEN              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM3INTEN_OFFSET)      /* FLEXPWM1 Interrupt Enable Register */
#define IMXRT_FLEXPWM1_SM3DMAEN              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM3DMAEN_OFFSET)      /* FLEXPWM1 DMA Enable Register */
#define IMXRT_FLEXPWM1_SM3TCTRL              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM3TCTRL_OFFSET)      /* FLEXPWM1 Output Trigger Control Register */
#define IMXRT_FLEXPWM1_SM3DISMAP0            (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM3DISMAP0_OFFSET)    /* FLEXPWM1 Fault Disable Mapping Register 0 */
#define IMXRT_FLEXPWM1_SM3DISMAP1            (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM3DISMAP1_OFFSET)    /* FLEXPWM1 Fault Disable Mapping Register 1 */
#define IMXRT_FLEXPWM1_SM3DTCNT0             (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM3DTCNT0_OFFSET)     /* FLEXPWM1 Deadtime Count Register 0 */
#define IMXRT_FLEXPWM1_SM3DTCNT1             (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM3DTCNT1_OFFSET)     /* FLEXPWM1 Deadtime Count Register 1 */
#define IMXRT_FLEXPWM1_SM3CAPTCTRLA          (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM3CAPTCTRLA_OFFSET)  /* FLEXPWM1 Capture Control A Register */
#define IMXRT_FLEXPWM1_SM3CAPTCOMPA          (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM3CAPTCOMPA_OFFSET)  /* FLEXPWM1 Capture Compare A Register */
#define IMXRT_FLEXPWM1_SM3CAPTCTRLB          (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM3CAPTCTRLB_OFFSET)  /* FLEXPWM1 Capture Control B Register */
#define IMXRT_FLEXPWM1_SM3CAPTCOMPB          (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM3CAPTCOMPB_OFFSET)  /* FLEXPWM1 Capture Compare B Register */
#define IMXRT_FLEXPWM1_SM3CAPTCTRLX          (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM3CAPTCTRLX_OFFSET)  /* FLEXPWM1 Capture Control X Register */
#define IMXRT_FLEXPWM1_SM3CAPTCOMPX          (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM3CAPTCOMPX_OFFSET)  /* FLEXPWM1 Capture Compare X Register */
#define IMXRT_FLEXPWM1_SM3CVAL0              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM3CVAL0_OFFSET)      /* FLEXPWM1 Capture Value 0 Register */
#define IMXRT_FLEXPWM1_SM3CVAL0CYC           (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM3CVAL0CYC_OFFSET)   /* FLEXPWM1 Capture Value 0 Cycle Register */
#define IMXRT_FLEXPWM1_SM3CVAL1              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM3CVAL1_OFFSET)      /* FLEXPWM1 Capture Value 1 Register */
#define IMXRT_FLEXPWM1_SM3CVAL1CYC           (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM3CVAL1CYC_OFFSET)   /* FLEXPWM1 Capture Value 1 Cycle Register */
#define IMXRT_FLEXPWM1_SM3CVAL2              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM3CVAL2_OFFSET)      /* FLEXPWM1 Capture Value 2 Register */
#define IMXRT_FLEXPWM1_SM3CVAL2CYC           (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM3CVAL2CYC_OFFSET)   /* FLEXPWM1 Capture Value 2 Cycle Register */
#define IMXRT_FLEXPWM1_SM3CVAL3              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM3CVAL3_OFFSET)      /* FLEXPWM1 Capture Value 3 Register */
#define IMXRT_FLEXPWM1_SM3CVAL3CYC           (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM3CVAL3CYC_OFFSET)   /* FLEXPWM1 Capture Value 3 Cycle Register */
#define IMXRT_FLEXPWM1_SM3CVAL4              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM3CVAL4_OFFSET)      /* FLEXPWM1 Capture Value 4 Register */
#define IMXRT_FLEXPWM1_SM3CVAL4CYC           (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM3CVAL4CYC_OFFSET)   /* FLEXPWM1 Capture Value 4 Cycle Register */
#define IMXRT_FLEXPWM1_SM3CVAL5              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM3CVAL5_OFFSET)      /* FLEXPWM1 Capture Value 5 Register */
#define IMXRT_FLEXPWM1_SM3CVAL5CYC           (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SM3CVAL5CYC_OFFSET)   /* FLEXPWM1 Capture Value 5 Cycle Register */
#define IMXRT_FLEXPWM1_OUTEN                 (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_OUTEN_OFFSET)         /* FLEXPWM1 Output Enable Register */
#define IMXRT_FLEXPWM1_MASK                  (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_MASK_OFFSET)          /* FLEXPWM1 Mask Register */
#define IMXRT_FLEXPWM1_SWCOUT                (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_SWCOUT_OFFSET)        /* FLEXPWM1 Software Controlled Output Register */
#define IMXRT_FLEXPWM1_DTSRCSEL              (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_DTSRCSEL_OFFSET)      /* FLEXPWM1 PWM Source Select Register */
#define IMXRT_FLEXPWM1_MCTRL                 (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_MCTRL_OFFSET)         /* FLEXPWM1 Master Control Register */
#define IMXRT_FLEXPWM1_MCTRL2                (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_MCTRL2_OFFSET)        /* FLEXPWM1 Master Control 2 Register */
#define IMXRT_FLEXPWM1_FCTRL0                (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_FCTRL0_OFFSET)        /* FLEXPWM1 Fault Control Register */
#define IMXRT_FLEXPWM1_FSTS0                 (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_FSTS0_OFFSET)         /* FLEXPWM1 Fault Status Register */
#define IMXRT_FLEXPWM1_FFILT0                (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_FFILT0_OFFSET)        /* FLEXPWM1 Fault Filter Register */
#define IMXRT_FLEXPWM1_FTST0                 (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_FTST0_OFFSET)         /* FLEXPWM1 Fault Test Register */
#define IMXRT_FLEXPWM1_FCTRL20               (IMXRT_FLEXPWM1_BASE + IMXRT_FLEXPWM_FCTRL20_OFFSET)       /* FLEXPWM1 Fault Control 2 Register */

/* FLEXPWM2 Register Addresses */

#define IMXRT_FLEXPWM2_SM0CNT                (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM0CNT_OFFSET)        /* FLEXPWM2 Counter Register */
#define IMXRT_FLEXPWM2_SM0INIT               (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM0INIT_OFFSET)       /* FLEXPWM2 Initial Count Register */
#define IMXRT_FLEXPWM2_SM0CTRL2              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM0CTRL2_OFFSET)      /* FLEXPWM2 Control 2 Register */
#define IMXRT_FLEXPWM2_SM0CTRL               (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM0CTRL_OFFSET)       /* FLEXPWM2 Control Register */
#define IMXRT_FLEXPWM2_SM0VAL0               (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM0VAL0_OFFSET)       /* FLEXPWM2 Value Register 0 */
#define IMXRT_FLEXPWM2_SM0FRACVAL1           (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM0FRACVAL1_OFFSET)   /* FLEXPWM2 Fractional Value Register 1 */
#define IMXRT_FLEXPWM2_SM0VAL1               (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM0VAL1_OFFSET)       /* FLEXPWM2 Value Register 1 */
#define IMXRT_FLEXPWM2_SM0FRACVAL2           (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM0FRACVAL2_OFFSET)   /* FLEXPWM2 Fractional Value Register 2 */
#define IMXRT_FLEXPWM2_SM0VAL2               (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM0VAL2_OFFSET)       /* FLEXPWM2 Value Register 2 */
#define IMXRT_FLEXPWM2_SM0FRACVAL3           (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM0FRACVAL3_OFFSET)   /* FLEXPWM2 Fractional Value Register 3 */
#define IMXRT_FLEXPWM2_SM0VAL3               (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM0VAL3_OFFSET)       /* FLEXPWM2 Value Register 3 */
#define IMXRT_FLEXPWM2_SM0FRACVAL4           (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM0FRACVAL4_OFFSET)   /* FLEXPWM2 Fractional Value Register 4 */
#define IMXRT_FLEXPWM2_SM0VAL4               (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM0VAL4_OFFSET)       /* FLEXPWM2 Value Register 4 */
#define IMXRT_FLEXPWM2_SM0FRACVAL5           (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM0FRACVAL5_OFFSET)   /* FLEXPWM2 Fractional Value Register 5 */
#define IMXRT_FLEXPWM2_SM0VAL5               (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM0VAL5_OFFSET)       /* FLEXPWM2 Value Register 5 */
#define IMXRT_FLEXPWM2_SM0FRCTRL             (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM0FRCTRL_OFFSET)     /* FLEXPWM2 Fractional Control Register */
#define IMXRT_FLEXPWM2_SM0OCTRL              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM0OCTRL_OFFSET)      /* FLEXPWM2 Output Control Register */
#define IMXRT_FLEXPWM2_SM0STS                (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM0STS_OFFSET)        /* FLEXPWM2 Status Register */
#define IMXRT_FLEXPWM2_SM0INTEN              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM0INTEN_OFFSET)      /* FLEXPWM2 Interrupt Enable Register */
#define IMXRT_FLEXPWM2_SM0DMAEN              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM0DMAEN_OFFSET)      /* FLEXPWM2 DMA Enable Register */
#define IMXRT_FLEXPWM2_SM0TCTRL              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM0TCTRL_OFFSET)      /* FLEXPWM2 Output Trigger Control Register */
#define IMXRT_FLEXPWM2_SM0DISMAP0            (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM0DISMAP0_OFFSET)    /* FLEXPWM2 Fault Disable Mapping Register 0 */
#define IMXRT_FLEXPWM2_SM0DISMAP1            (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM0DISMAP1_OFFSET)    /* FLEXPWM2 Fault Disable Mapping Register 1 */
#define IMXRT_FLEXPWM2_SM0DTCNT0             (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM0DTCNT0_OFFSET)     /* FLEXPWM2 Deadtime Count Register 0 */
#define IMXRT_FLEXPWM2_SM0DTCNT1             (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM0DTCNT1_OFFSET)     /* FLEXPWM2 Deadtime Count Register 1 */
#define IMXRT_FLEXPWM2_SM0CAPTCTRLA          (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM0CAPTCTRLA_OFFSET)  /* FLEXPWM2 Capture Control A Register */
#define IMXRT_FLEXPWM2_SM0CAPTCOMPA          (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM0CAPTCOMPA_OFFSET)  /* FLEXPWM2 Capture Compare A Register */
#define IMXRT_FLEXPWM2_SM0CAPTCTRLB          (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM0CAPTCTRLB_OFFSET)  /* FLEXPWM2 Capture Control B Register */
#define IMXRT_FLEXPWM2_SM0CAPTCOMPB          (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM0CAPTCOMPB_OFFSET)  /* FLEXPWM2 Capture Compare B Register */
#define IMXRT_FLEXPWM2_SM0CAPTCTRLX          (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM0CAPTCTRLX_OFFSET)  /* FLEXPWM2 Capture Control X Register */
#define IMXRT_FLEXPWM2_SM0CAPTCOMPX          (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM0CAPTCOMPX_OFFSET)  /* FLEXPWM2 Capture Compare X Register */
#define IMXRT_FLEXPWM2_SM0CVAL0              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM0CVAL0_OFFSET)      /* FLEXPWM2 Capture Value 0 Register */
#define IMXRT_FLEXPWM2_SM0CVAL0CYC           (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM0CVAL0CYC_OFFSET)   /* FLEXPWM2 Capture Value 0 Cycle Register */
#define IMXRT_FLEXPWM2_SM0CVAL1              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM0CVAL1_OFFSET)      /* FLEXPWM2 Capture Value 1 Register */
#define IMXRT_FLEXPWM2_SM0CVAL1CYC           (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM0CVAL1CYC_OFFSET)   /* FLEXPWM2 Capture Value 1 Cycle Register */
#define IMXRT_FLEXPWM2_SM0CVAL2              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM0CVAL2_OFFSET)      /* FLEXPWM2 Capture Value 2 Register */
#define IMXRT_FLEXPWM2_SM0CVAL2CYC           (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM0CVAL2CYC_OFFSET)   /* FLEXPWM2 Capture Value 2 Cycle Register */
#define IMXRT_FLEXPWM2_SM0CVAL3              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM0CVAL3_OFFSET)      /* FLEXPWM2 Capture Value 3 Register */
#define IMXRT_FLEXPWM2_SM0CVAL3CYC           (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM0CVAL3CYC_OFFSET)   /* FLEXPWM2 Capture Value 3 Cycle Register */
#define IMXRT_FLEXPWM2_SM0CVAL4              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM0CVAL4_OFFSET)      /* FLEXPWM2 Capture Value 4 Register */
#define IMXRT_FLEXPWM2_SM0CVAL4CYC           (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM0CVAL4CYC_OFFSET)   /* FLEXPWM2 Capture Value 4 Cycle Register */
#define IMXRT_FLEXPWM2_SM0CVAL5              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM0CVAL5_OFFSET)      /* FLEXPWM2 Capture Value 5 Register */
#define IMXRT_FLEXPWM2_SM0CVAL5CYC           (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM0CVAL5CYC_OFFSET)   /* FLEXPWM2 Capture Value 5 Cycle Register */
#define IMXRT_FLEXPWM2_SM1CNT                (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM1CNT_OFFSET)        /* FLEXPWM2 Counter Register */
#define IMXRT_FLEXPWM2_SM1INIT               (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM1INIT_OFFSET)       /* FLEXPWM2 Initial Count Register */
#define IMXRT_FLEXPWM2_SM1CTRL2              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM1CTRL2_OFFSET)      /* FLEXPWM2 Control 2 Register */
#define IMXRT_FLEXPWM2_SM1CTRL               (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM1CTRL_OFFSET)       /* FLEXPWM2 Control Register */
#define IMXRT_FLEXPWM2_SM1VAL0               (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM1VAL0_OFFSET)       /* FLEXPWM2 Value Register 0 */
#define IMXRT_FLEXPWM2_SM1FRACVAL1           (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM1FRACVAL1_OFFSET)   /* FLEXPWM2 Fractional Value Register 1 */
#define IMXRT_FLEXPWM2_SM1VAL1               (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM1VAL1_OFFSET)       /* FLEXPWM2 Value Register 1 */
#define IMXRT_FLEXPWM2_SM1FRACVAL2           (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM1FRACVAL2_OFFSET)   /* FLEXPWM2 Fractional Value Register 2 */
#define IMXRT_FLEXPWM2_SM1VAL2               (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM1VAL2_OFFSET)       /* FLEXPWM2 Value Register 2 */
#define IMXRT_FLEXPWM2_SM1FRACVAL3           (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM1FRACVAL3_OFFSET)   /* FLEXPWM2 Fractional Value Register 3 */
#define IMXRT_FLEXPWM2_SM1VAL3               (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM1VAL3_OFFSET)       /* FLEXPWM2 Value Register 3 */
#define IMXRT_FLEXPWM2_SM1FRACVAL4           (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM1FRACVAL4_OFFSET)   /* FLEXPWM2 Fractional Value Register 4 */
#define IMXRT_FLEXPWM2_SM1VAL4               (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM1VAL4_OFFSET)       /* FLEXPWM2 Value Register 4 */
#define IMXRT_FLEXPWM2_SM1FRACVAL5           (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM1FRACVAL5_OFFSET)   /* FLEXPWM2 Fractional Value Register 5 */
#define IMXRT_FLEXPWM2_SM1VAL5               (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM1VAL5_OFFSET)       /* FLEXPWM2 Value Register 5 */
#define IMXRT_FLEXPWM2_SM1FRCTRL             (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM1FRCTRL_OFFSET)     /* FLEXPWM2 Fractional Control Register */
#define IMXRT_FLEXPWM2_SM1OCTRL              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM1OCTRL_OFFSET)      /* FLEXPWM2 Output Control Register */
#define IMXRT_FLEXPWM2_SM1STS                (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM1STS_OFFSET)        /* FLEXPWM2 Status Register */
#define IMXRT_FLEXPWM2_SM1INTEN              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM1INTEN_OFFSET)      /* FLEXPWM2 Interrupt Enable Register */
#define IMXRT_FLEXPWM2_SM1DMAEN              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM1DMAEN_OFFSET)      /* FLEXPWM2 DMA Enable Register */
#define IMXRT_FLEXPWM2_SM1TCTRL              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM1TCTRL_OFFSET)      /* FLEXPWM2 Output Trigger Control Register */
#define IMXRT_FLEXPWM2_SM1DISMAP0            (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM1DISMAP0_OFFSET)    /* FLEXPWM2 Fault Disable Mapping Register 0 */
#define IMXRT_FLEXPWM2_SM1DISMAP1            (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM1DISMAP1_OFFSET)    /* FLEXPWM2 Fault Disable Mapping Register 1 */
#define IMXRT_FLEXPWM2_SM1DTCNT0             (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM1DTCNT0_OFFSET)     /* FLEXPWM2 Deadtime Count Register 0 */
#define IMXRT_FLEXPWM2_SM1DTCNT1             (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM1DTCNT1_OFFSET)     /* FLEXPWM2 Deadtime Count Register 1 */
#define IMXRT_FLEXPWM2_SM1CAPTCTRLA          (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM1CAPTCTRLA_OFFSET)  /* FLEXPWM2 Capture Control A Register */
#define IMXRT_FLEXPWM2_SM1CAPTCOMPA          (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM1CAPTCOMPA_OFFSET)  /* FLEXPWM2 Capture Compare A Register */
#define IMXRT_FLEXPWM2_eFlexPWM              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_eFlexPWM_OFFSET)      /* FLEXPWM2 apter 28 Enhanced Flex Pulse Width Modulator */
#define IMXRT_FLEXPWM2_SM1CAPTCTRLB          (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM1CAPTCTRLB_OFFSET)  /* FLEXPWM2 Capture Control B Register */
#define IMXRT_FLEXPWM2_SM1CAPTCOMPB          (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM1CAPTCOMPB_OFFSET)  /* FLEXPWM2 Capture Compare B Register */
#define IMXRT_FLEXPWM2_SM1CAPTCTRLX          (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM1CAPTCTRLX_OFFSET)  /* FLEXPWM2 Capture Control X Register */
#define IMXRT_FLEXPWM2_SM1CAPTCOMPX          (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM1CAPTCOMPX_OFFSET)  /* FLEXPWM2 Capture Compare X Register */
#define IMXRT_FLEXPWM2_SM1CVAL0              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM1CVAL0_OFFSET)      /* FLEXPWM2 Capture Value 0 Register */
#define IMXRT_FLEXPWM2_SM1CVAL0CYC           (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM1CVAL0CYC_OFFSET)   /* FLEXPWM2 Capture Value 0 Cycle Register */
#define IMXRT_FLEXPWM2_SM1CVAL1              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM1CVAL1_OFFSET)      /* FLEXPWM2 Capture Value 1 Register */
#define IMXRT_FLEXPWM2_SM1CVAL1CYC           (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM1CVAL1CYC_OFFSET)   /* FLEXPWM2 Capture Value 1 Cycle Register */
#define IMXRT_FLEXPWM2_SM1CVAL2              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM1CVAL2_OFFSET)      /* FLEXPWM2 Capture Value 2 Register */
#define IMXRT_FLEXPWM2_SM1CVAL2CYC           (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM1CVAL2CYC_OFFSET)   /* FLEXPWM2 Capture Value 2 Cycle Register */
#define IMXRT_FLEXPWM2_SM1CVAL3              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM1CVAL3_OFFSET)      /* FLEXPWM2 Capture Value 3 Register */
#define IMXRT_FLEXPWM2_SM1CVAL3CYC           (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM1CVAL3CYC_OFFSET)   /* FLEXPWM2 Capture Value 3 Cycle Register */
#define IMXRT_FLEXPWM2_SM1CVAL4              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM1CVAL4_OFFSET)      /* FLEXPWM2 Capture Value 4 Register */
#define IMXRT_FLEXPWM2_SM1CVAL4CYC           (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM1CVAL4CYC_OFFSET)   /* FLEXPWM2 Capture Value 4 Cycle Register */
#define IMXRT_FLEXPWM2_SM1CVAL5              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM1CVAL5_OFFSET)      /* FLEXPWM2 Capture Value 5 Register */
#define IMXRT_FLEXPWM2_SM1CVAL5CYC           (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM1CVAL5CYC_OFFSET)   /* FLEXPWM2 Capture Value 5 Cycle Register */
#define IMXRT_FLEXPWM2_SM2CNT                (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM2CNT_OFFSET)        /* FLEXPWM2 Counter Register */
#define IMXRT_FLEXPWM2_SM2INIT               (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM2INIT_OFFSET)       /* FLEXPWM2 Initial Count Register */
#define IMXRT_FLEXPWM2_SM2CTRL2              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM2CTRL2_OFFSET)      /* FLEXPWM2 Control 2 Register */
#define IMXRT_FLEXPWM2_SM2CTRL               (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM2CTRL_OFFSET)       /* FLEXPWM2 Control Register */
#define IMXRT_FLEXPWM2_SM2VAL0               (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM2VAL0_OFFSET)       /* FLEXPWM2 Value Register 0 */
#define IMXRT_FLEXPWM2_SM2FRACVAL1           (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM2FRACVAL1_OFFSET)   /* FLEXPWM2 Fractional Value Register 1 */
#define IMXRT_FLEXPWM2_SM2VAL1               (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM2VAL1_OFFSET)       /* FLEXPWM2 Value Register 1 */
#define IMXRT_FLEXPWM2_SM2FRACVAL2           (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM2FRACVAL2_OFFSET)   /* FLEXPWM2 Fractional Value Register 2 */
#define IMXRT_FLEXPWM2_SM2VAL2               (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM2VAL2_OFFSET)       /* FLEXPWM2 Value Register 2 */
#define IMXRT_FLEXPWM2_SM2FRACVAL3           (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM2FRACVAL3_OFFSET)   /* FLEXPWM2 Fractional Value Register 3 */
#define IMXRT_FLEXPWM2_SM2VAL3               (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM2VAL3_OFFSET)       /* FLEXPWM2 Value Register 3 */
#define IMXRT_FLEXPWM2_SM2FRACVAL4           (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM2FRACVAL4_OFFSET)   /* FLEXPWM2 Fractional Value Register 4 */
#define IMXRT_FLEXPWM2_SM2VAL4               (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM2VAL4_OFFSET)       /* FLEXPWM2 Value Register 4 */
#define IMXRT_FLEXPWM2_SM2FRACVAL5           (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM2FRACVAL5_OFFSET)   /* FLEXPWM2 Fractional Value Register 5 */
#define IMXRT_FLEXPWM2_SM2VAL5               (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM2VAL5_OFFSET)       /* FLEXPWM2 Value Register 5 */
#define IMXRT_FLEXPWM2_SM2FRCTRL             (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM2FRCTRL_OFFSET)     /* FLEXPWM2 Fractional Control Register */
#define IMXRT_FLEXPWM2_SM2OCTRL              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM2OCTRL_OFFSET)      /* FLEXPWM2 Output Control Register */
#define IMXRT_FLEXPWM2_SM2STS                (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM2STS_OFFSET)        /* FLEXPWM2 Status Register */
#define IMXRT_FLEXPWM2_SM2INTEN              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM2INTEN_OFFSET)      /* FLEXPWM2 Interrupt Enable Register */
#define IMXRT_FLEXPWM2_SM2DMAEN              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM2DMAEN_OFFSET)      /* FLEXPWM2 DMA Enable Register */
#define IMXRT_FLEXPWM2_SM2TCTRL              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM2TCTRL_OFFSET)      /* FLEXPWM2 Output Trigger Control Register */
#define IMXRT_FLEXPWM2_SM2DISMAP0            (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM2DISMAP0_OFFSET)    /* FLEXPWM2 Fault Disable Mapping Register 0 */
#define IMXRT_FLEXPWM2_SM2DISMAP1            (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM2DISMAP1_OFFSET)    /* FLEXPWM2 Fault Disable Mapping Register 1 */
#define IMXRT_FLEXPWM2_SM2DTCNT0             (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM2DTCNT0_OFFSET)     /* FLEXPWM2 Deadtime Count Register 0 */
#define IMXRT_FLEXPWM2_SM2DTCNT1             (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM2DTCNT1_OFFSET)     /* FLEXPWM2 Deadtime Count Register 1 */
#define IMXRT_FLEXPWM2_SM2CAPTCTRLA          (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM2CAPTCTRLA_OFFSET)  /* FLEXPWM2 Capture Control A Register */
#define IMXRT_FLEXPWM2_SM2CAPTCOMPA          (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM2CAPTCOMPA_OFFSET)  /* FLEXPWM2 Capture Compare A Register */
#define IMXRT_FLEXPWM2_SM2CAPTCTRLB          (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM2CAPTCTRLB_OFFSET)  /* FLEXPWM2 Capture Control B Register */
#define IMXRT_FLEXPWM2_SM2CAPTCOMPB          (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM2CAPTCOMPB_OFFSET)  /* FLEXPWM2 Capture Compare B Register */
#define IMXRT_FLEXPWM2_SM2CAPTCTRLX          (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM2CAPTCTRLX_OFFSET)  /* FLEXPWM2 Capture Control X Register */
#define IMXRT_FLEXPWM2_SM2CAPTCOMPX          (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM2CAPTCOMPX_OFFSET)  /* FLEXPWM2 Capture Compare X Register */
#define IMXRT_FLEXPWM2_SM2CVAL0              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM2CVAL0_OFFSET)      /* FLEXPWM2 Capture Value 0 Register */
#define IMXRT_FLEXPWM2_SM2CVAL0CYC           (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM2CVAL0CYC_OFFSET)   /* FLEXPWM2 Capture Value 0 Cycle Register */
#define IMXRT_FLEXPWM2_SM2CVAL1              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM2CVAL1_OFFSET)      /* FLEXPWM2 Capture Value 1 Register */
#define IMXRT_FLEXPWM2_SM2CVAL1CYC           (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM2CVAL1CYC_OFFSET)   /* FLEXPWM2 Capture Value 1 Cycle Register */
#define IMXRT_FLEXPWM2_SM2CVAL2              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM2CVAL2_OFFSET)      /* FLEXPWM2 Capture Value 2 Register */
#define IMXRT_FLEXPWM2_SM2CVAL2CYC           (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM2CVAL2CYC_OFFSET)   /* FLEXPWM2 Capture Value 2 Cycle Register */
#define IMXRT_FLEXPWM2_SM2CVAL3              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM2CVAL3_OFFSET)      /* FLEXPWM2 Capture Value 3 Register */
#define IMXRT_FLEXPWM2_SM2CVAL3CYC           (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM2CVAL3CYC_OFFSET)   /* FLEXPWM2 Capture Value 3 Cycle Register */
#define IMXRT_FLEXPWM2_SM2CVAL4              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM2CVAL4_OFFSET)      /* FLEXPWM2 Capture Value 4 Register */
#define IMXRT_FLEXPWM2_SM2CVAL4CYC           (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM2CVAL4CYC_OFFSET)   /* FLEXPWM2 Capture Value 4 Cycle Register */
#define IMXRT_FLEXPWM2_SM2CVAL5              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM2CVAL5_OFFSET)      /* FLEXPWM2 Capture Value 5 Register */
#define IMXRT_FLEXPWM2_SM2CVAL5CYC           (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM2CVAL5CYC_OFFSET)   /* FLEXPWM2 Capture Value 5 Cycle Register */
#define IMXRT_FLEXPWM2_SM3CNT                (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM3CNT_OFFSET)        /* FLEXPWM2 Counter Register */
#define IMXRT_FLEXPWM2_SM3INIT               (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM3INIT_OFFSET)       /* FLEXPWM2 Initial Count Register */
#define IMXRT_FLEXPWM2_SM3CTRL2              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM3CTRL2_OFFSET)      /* FLEXPWM2 Control 2 Register */
#define IMXRT_FLEXPWM2_SM3CTRL               (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM3CTRL_OFFSET)       /* FLEXPWM2 Control Register */
#define IMXRT_FLEXPWM2_SM3VAL0               (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM3VAL0_OFFSET)       /* FLEXPWM2 Value Register 0 */
#define IMXRT_FLEXPWM2_SM3FRACVAL1           (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM3FRACVAL1_OFFSET)   /* FLEXPWM2 Fractional Value Register 1 */
#define IMXRT_FLEXPWM2_SM3VAL1               (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM3VAL1_OFFSET)       /* FLEXPWM2 Value Register 1 */
#define IMXRT_FLEXPWM2_SM3FRACVAL2           (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM3FRACVAL2_OFFSET)   /* FLEXPWM2 Fractional Value Register 2 */
#define IMXRT_FLEXPWM2_SM3VAL2               (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM3VAL2_OFFSET)       /* FLEXPWM2 Value Register 2 */
#define IMXRT_FLEXPWM2_SM3FRACVAL3           (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM3FRACVAL3_OFFSET)   /* FLEXPWM2 Fractional Value Register 3 */
#define IMXRT_FLEXPWM2_SM3VAL3               (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM3VAL3_OFFSET)       /* FLEXPWM2 Value Register 3 */
#define IMXRT_FLEXPWM2_SM3FRACVAL4           (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM3FRACVAL4_OFFSET)   /* FLEXPWM2 Fractional Value Register 4 */
#define IMXRT_FLEXPWM2_SM3VAL4               (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM3VAL4_OFFSET)       /* FLEXPWM2 Value Register 4 */
#define IMXRT_FLEXPWM2_SM3FRACVAL5           (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM3FRACVAL5_OFFSET)   /* FLEXPWM2 Fractional Value Register 5 */
#define IMXRT_FLEXPWM2_SM3VAL5               (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM3VAL5_OFFSET)       /* FLEXPWM2 Value Register 5 */
#define IMXRT_FLEXPWM2_SM3FRCTRL             (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM3FRCTRL_OFFSET)     /* FLEXPWM2 Fractional Control Register */
#define IMXRT_FLEXPWM2_SM3OCTRL              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM3OCTRL_OFFSET)      /* FLEXPWM2 Output Control Register */
#define IMXRT_FLEXPWM2_SM3STS                (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM3STS_OFFSET)        /* FLEXPWM2 Status Register */
#define IMXRT_FLEXPWM2_SM3INTEN              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM3INTEN_OFFSET)      /* FLEXPWM2 Interrupt Enable Register */
#define IMXRT_FLEXPWM2_SM3DMAEN              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM3DMAEN_OFFSET)      /* FLEXPWM2 DMA Enable Register */
#define IMXRT_FLEXPWM2_SM3TCTRL              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM3TCTRL_OFFSET)      /* FLEXPWM2 Output Trigger Control Register */
#define IMXRT_FLEXPWM2_SM3DISMAP0            (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM3DISMAP0_OFFSET)    /* FLEXPWM2 Fault Disable Mapping Register 0 */
#define IMXRT_FLEXPWM2_SM3DISMAP1            (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM3DISMAP1_OFFSET)    /* FLEXPWM2 Fault Disable Mapping Register 1 */
#define IMXRT_FLEXPWM2_SM3DTCNT0             (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM3DTCNT0_OFFSET)     /* FLEXPWM2 Deadtime Count Register 0 */
#define IMXRT_FLEXPWM2_SM3DTCNT1             (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM3DTCNT1_OFFSET)     /* FLEXPWM2 Deadtime Count Register 1 */
#define IMXRT_FLEXPWM2_SM3CAPTCTRLA          (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM3CAPTCTRLA_OFFSET)  /* FLEXPWM2 Capture Control A Register */
#define IMXRT_FLEXPWM2_SM3CAPTCOMPA          (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM3CAPTCOMPA_OFFSET)  /* FLEXPWM2 Capture Compare A Register */
#define IMXRT_FLEXPWM2_SM3CAPTCTRLB          (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM3CAPTCTRLB_OFFSET)  /* FLEXPWM2 Capture Control B Register */
#define IMXRT_FLEXPWM2_SM3CAPTCOMPB          (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM3CAPTCOMPB_OFFSET)  /* FLEXPWM2 Capture Compare B Register */
#define IMXRT_FLEXPWM2_SM3CAPTCTRLX          (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM3CAPTCTRLX_OFFSET)  /* FLEXPWM2 Capture Control X Register */
#define IMXRT_FLEXPWM2_SM3CAPTCOMPX          (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM3CAPTCOMPX_OFFSET)  /* FLEXPWM2 Capture Compare X Register */
#define IMXRT_FLEXPWM2_SM3CVAL0              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM3CVAL0_OFFSET)      /* FLEXPWM2 Capture Value 0 Register */
#define IMXRT_FLEXPWM2_SM3CVAL0CYC           (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM3CVAL0CYC_OFFSET)   /* FLEXPWM2 Capture Value 0 Cycle Register */
#define IMXRT_FLEXPWM2_SM3CVAL1              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM3CVAL1_OFFSET)      /* FLEXPWM2 Capture Value 1 Register */
#define IMXRT_FLEXPWM2_SM3CVAL1CYC           (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM3CVAL1CYC_OFFSET)   /* FLEXPWM2 Capture Value 1 Cycle Register */
#define IMXRT_FLEXPWM2_SM3CVAL2              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM3CVAL2_OFFSET)      /* FLEXPWM2 Capture Value 2 Register */
#define IMXRT_FLEXPWM2_SM3CVAL2CYC           (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM3CVAL2CYC_OFFSET)   /* FLEXPWM2 Capture Value 2 Cycle Register */
#define IMXRT_FLEXPWM2_SM3CVAL3              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM3CVAL3_OFFSET)      /* FLEXPWM2 Capture Value 3 Register */
#define IMXRT_FLEXPWM2_SM3CVAL3CYC           (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM3CVAL3CYC_OFFSET)   /* FLEXPWM2 Capture Value 3 Cycle Register */
#define IMXRT_FLEXPWM2_SM3CVAL4              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM3CVAL4_OFFSET)      /* FLEXPWM2 Capture Value 4 Register */
#define IMXRT_FLEXPWM2_SM3CVAL4CYC           (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM3CVAL4CYC_OFFSET)   /* FLEXPWM2 Capture Value 4 Cycle Register */
#define IMXRT_FLEXPWM2_SM3CVAL5              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM3CVAL5_OFFSET)      /* FLEXPWM2 Capture Value 5 Register */
#define IMXRT_FLEXPWM2_SM3CVAL5CYC           (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SM3CVAL5CYC_OFFSET)   /* FLEXPWM2 Capture Value 5 Cycle Register */
#define IMXRT_FLEXPWM2_OUTEN                 (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_OUTEN_OFFSET)         /* FLEXPWM2 Output Enable Register */
#define IMXRT_FLEXPWM2_MASK                  (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_MASK_OFFSET)          /* FLEXPWM2 Mask Register */
#define IMXRT_FLEXPWM2_SWCOUT                (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_SWCOUT_OFFSET)        /* FLEXPWM2 Software Controlled Output Register */
#define IMXRT_FLEXPWM2_DTSRCSEL              (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_DTSRCSEL_OFFSET)      /* FLEXPWM2 PWM Source Select Register */
#define IMXRT_FLEXPWM2_MCTRL                 (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_MCTRL_OFFSET)         /* FLEXPWM2 Master Control Register */
#define IMXRT_FLEXPWM2_MCTRL2                (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_MCTRL2_OFFSET)        /* FLEXPWM2 Master Control 2 Register */
#define IMXRT_FLEXPWM2_FCTRL0                (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_FCTRL0_OFFSET)        /* FLEXPWM2 Fault Control Register */
#define IMXRT_FLEXPWM2_FSTS0                 (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_FSTS0_OFFSET)         /* FLEXPWM2 Fault Status Register */
#define IMXRT_FLEXPWM2_FFILT0                (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_FFILT0_OFFSET)        /* FLEXPWM2 Fault Filter Register */
#define IMXRT_FLEXPWM2_FTST0                 (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_FTST0_OFFSET)         /* FLEXPWM2 Fault Test Register */
#define IMXRT_FLEXPWM2_FCTRL20               (IMXRT_FLEXPWM2_BASE + IMXRT_FLEXPWM_FCTRL20_OFFSET)       /* FLEXPWM2 Fault Control 2 Register */

/* FLEXPWM3 Register Addresses */

#define IMXRT_FLEXPWM3_SM0CNT                (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM0CNT_OFFSET)        /* FLEXPWM3 Counter Register */
#define IMXRT_FLEXPWM3_SM0INIT               (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM0INIT_OFFSET)       /* FLEXPWM3 Initial Count Register */
#define IMXRT_FLEXPWM3_SM0CTRL2              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM0CTRL2_OFFSET)      /* FLEXPWM3 Control 2 Register */
#define IMXRT_FLEXPWM3_SM0CTRL               (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM0CTRL_OFFSET)       /* FLEXPWM3 Control Register */
#define IMXRT_FLEXPWM3_SM0VAL0               (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM0VAL0_OFFSET)       /* FLEXPWM3 Value Register 0 */
#define IMXRT_FLEXPWM3_SM0FRACVAL1           (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM0FRACVAL1_OFFSET)   /* FLEXPWM3 Fractional Value Register 1 */
#define IMXRT_FLEXPWM3_SM0VAL1               (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM0VAL1_OFFSET)       /* FLEXPWM3 Value Register 1 */
#define IMXRT_FLEXPWM3_SM0FRACVAL2           (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM0FRACVAL2_OFFSET)   /* FLEXPWM3 Fractional Value Register 2 */
#define IMXRT_FLEXPWM3_SM0VAL2               (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM0VAL2_OFFSET)       /* FLEXPWM3 Value Register 2 */
#define IMXRT_FLEXPWM3_SM0FRACVAL3           (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM0FRACVAL3_OFFSET)   /* FLEXPWM3 Fractional Value Register 3 */
#define IMXRT_FLEXPWM3_SM0VAL3               (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM0VAL3_OFFSET)       /* FLEXPWM3 Value Register 3 */
#define IMXRT_FLEXPWM3_SM0FRACVAL4           (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM0FRACVAL4_OFFSET)   /* FLEXPWM3 Fractional Value Register 4 */
#define IMXRT_FLEXPWM3_SM0VAL4               (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM0VAL4_OFFSET)       /* FLEXPWM3 Value Register 4 */
#define IMXRT_FLEXPWM3_SM0FRACVAL5           (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM0FRACVAL5_OFFSET)   /* FLEXPWM3 Fractional Value Register 5 */
#define IMXRT_FLEXPWM3_SM0VAL5               (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM0VAL5_OFFSET)       /* FLEXPWM3 Value Register 5 */
#define IMXRT_FLEXPWM3_SM0FRCTRL             (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM0FRCTRL_OFFSET)     /* FLEXPWM3 Fractional Control Register */
#define IMXRT_FLEXPWM3_SM0OCTRL              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM0OCTRL_OFFSET)      /* FLEXPWM3 Output Control Register */
#define IMXRT_FLEXPWM3_SM0STS                (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM0STS_OFFSET)        /* FLEXPWM3 Status Register */
#define IMXRT_FLEXPWM3_SM0INTEN              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM0INTEN_OFFSET)      /* FLEXPWM3 Interrupt Enable Register */
#define IMXRT_FLEXPWM3_SM0DMAEN              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM0DMAEN_OFFSET)      /* FLEXPWM3 DMA Enable Register */
#define IMXRT_FLEXPWM3_SM0TCTRL              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM0TCTRL_OFFSET)      /* FLEXPWM3 Output Trigger Control Register */
#define IMXRT_FLEXPWM3_SM0DISMAP0            (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM0DISMAP0_OFFSET)    /* FLEXPWM3 Fault Disable Mapping Register 0 */
#define IMXRT_FLEXPWM3_SM0DISMAP1            (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM0DISMAP1_OFFSET)    /* FLEXPWM3 Fault Disable Mapping Register 1 */
#define IMXRT_FLEXPWM3_SM0DTCNT0             (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM0DTCNT0_OFFSET)     /* FLEXPWM3 Deadtime Count Register 0 */
#define IMXRT_FLEXPWM3_SM0DTCNT1             (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM0DTCNT1_OFFSET)     /* FLEXPWM3 Deadtime Count Register 1 */
#define IMXRT_FLEXPWM3_SM0CAPTCTRLA          (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM0CAPTCTRLA_OFFSET)  /* FLEXPWM3 Capture Control A Register */
#define IMXRT_FLEXPWM3_SM0CAPTCOMPA          (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM0CAPTCOMPA_OFFSET)  /* FLEXPWM3 Capture Compare A Register */
#define IMXRT_FLEXPWM3_SM0CAPTCTRLB          (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM0CAPTCTRLB_OFFSET)  /* FLEXPWM3 Capture Control B Register */
#define IMXRT_FLEXPWM3_SM0CAPTCOMPB          (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM0CAPTCOMPB_OFFSET)  /* FLEXPWM3 Capture Compare B Register */
#define IMXRT_FLEXPWM3_SM0CAPTCTRLX          (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM0CAPTCTRLX_OFFSET)  /* FLEXPWM3 Capture Control X Register */
#define IMXRT_FLEXPWM3_SM0CAPTCOMPX          (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM0CAPTCOMPX_OFFSET)  /* FLEXPWM3 Capture Compare X Register */
#define IMXRT_FLEXPWM3_SM0CVAL0              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM0CVAL0_OFFSET)      /* FLEXPWM3 Capture Value 0 Register */
#define IMXRT_FLEXPWM3_SM0CVAL0CYC           (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM0CVAL0CYC_OFFSET)   /* FLEXPWM3 Capture Value 0 Cycle Register */
#define IMXRT_FLEXPWM3_SM0CVAL1              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM0CVAL1_OFFSET)      /* FLEXPWM3 Capture Value 1 Register */
#define IMXRT_FLEXPWM3_SM0CVAL1CYC           (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM0CVAL1CYC_OFFSET)   /* FLEXPWM3 Capture Value 1 Cycle Register */
#define IMXRT_FLEXPWM3_SM0CVAL2              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM0CVAL2_OFFSET)      /* FLEXPWM3 Capture Value 2 Register */
#define IMXRT_FLEXPWM3_SM0CVAL2CYC           (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM0CVAL2CYC_OFFSET)   /* FLEXPWM3 Capture Value 2 Cycle Register */
#define IMXRT_FLEXPWM3_SM0CVAL3              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM0CVAL3_OFFSET)      /* FLEXPWM3 Capture Value 3 Register */
#define IMXRT_FLEXPWM3_SM0CVAL3CYC           (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM0CVAL3CYC_OFFSET)   /* FLEXPWM3 Capture Value 3 Cycle Register */
#define IMXRT_FLEXPWM3_SM0CVAL4              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM0CVAL4_OFFSET)      /* FLEXPWM3 Capture Value 4 Register */
#define IMXRT_FLEXPWM3_SM0CVAL4CYC           (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM0CVAL4CYC_OFFSET)   /* FLEXPWM3 Capture Value 4 Cycle Register */
#define IMXRT_FLEXPWM3_SM0CVAL5              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM0CVAL5_OFFSET)      /* FLEXPWM3 Capture Value 5 Register */
#define IMXRT_FLEXPWM3_SM0CVAL5CYC           (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM0CVAL5CYC_OFFSET)   /* FLEXPWM3 Capture Value 5 Cycle Register */
#define IMXRT_FLEXPWM3_SM1CNT                (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM1CNT_OFFSET)        /* FLEXPWM3 Counter Register */
#define IMXRT_FLEXPWM3_SM1INIT               (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM1INIT_OFFSET)       /* FLEXPWM3 Initial Count Register */
#define IMXRT_FLEXPWM3_SM1CTRL2              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM1CTRL2_OFFSET)      /* FLEXPWM3 Control 2 Register */
#define IMXRT_FLEXPWM3_SM1CTRL               (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM1CTRL_OFFSET)       /* FLEXPWM3 Control Register */
#define IMXRT_FLEXPWM3_SM1VAL0               (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM1VAL0_OFFSET)       /* FLEXPWM3 Value Register 0 */
#define IMXRT_FLEXPWM3_SM1FRACVAL1           (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM1FRACVAL1_OFFSET)   /* FLEXPWM3 Fractional Value Register 1 */
#define IMXRT_FLEXPWM3_SM1VAL1               (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM1VAL1_OFFSET)       /* FLEXPWM3 Value Register 1 */
#define IMXRT_FLEXPWM3_SM1FRACVAL2           (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM1FRACVAL2_OFFSET)   /* FLEXPWM3 Fractional Value Register 2 */
#define IMXRT_FLEXPWM3_SM1VAL2               (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM1VAL2_OFFSET)       /* FLEXPWM3 Value Register 2 */
#define IMXRT_FLEXPWM3_SM1FRACVAL3           (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM1FRACVAL3_OFFSET)   /* FLEXPWM3 Fractional Value Register 3 */
#define IMXRT_FLEXPWM3_SM1VAL3               (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM1VAL3_OFFSET)       /* FLEXPWM3 Value Register 3 */
#define IMXRT_FLEXPWM3_SM1FRACVAL4           (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM1FRACVAL4_OFFSET)   /* FLEXPWM3 Fractional Value Register 4 */
#define IMXRT_FLEXPWM3_SM1VAL4               (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM1VAL4_OFFSET)       /* FLEXPWM3 Value Register 4 */
#define IMXRT_FLEXPWM3_SM1FRACVAL5           (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM1FRACVAL5_OFFSET)   /* FLEXPWM3 Fractional Value Register 5 */
#define IMXRT_FLEXPWM3_SM1VAL5               (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM1VAL5_OFFSET)       /* FLEXPWM3 Value Register 5 */
#define IMXRT_FLEXPWM3_SM1FRCTRL             (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM1FRCTRL_OFFSET)     /* FLEXPWM3 Fractional Control Register */
#define IMXRT_FLEXPWM3_SM1OCTRL              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM1OCTRL_OFFSET)      /* FLEXPWM3 Output Control Register */
#define IMXRT_FLEXPWM3_SM1STS                (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM1STS_OFFSET)        /* FLEXPWM3 Status Register */
#define IMXRT_FLEXPWM3_SM1INTEN              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM1INTEN_OFFSET)      /* FLEXPWM3 Interrupt Enable Register */
#define IMXRT_FLEXPWM3_SM1DMAEN              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM1DMAEN_OFFSET)      /* FLEXPWM3 DMA Enable Register */
#define IMXRT_FLEXPWM3_SM1TCTRL              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM1TCTRL_OFFSET)      /* FLEXPWM3 Output Trigger Control Register */
#define IMXRT_FLEXPWM3_SM1DISMAP0            (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM1DISMAP0_OFFSET)    /* FLEXPWM3 Fault Disable Mapping Register 0 */
#define IMXRT_FLEXPWM3_SM1DISMAP1            (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM1DISMAP1_OFFSET)    /* FLEXPWM3 Fault Disable Mapping Register 1 */
#define IMXRT_FLEXPWM3_SM1DTCNT0             (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM1DTCNT0_OFFSET)     /* FLEXPWM3 Deadtime Count Register 0 */
#define IMXRT_FLEXPWM3_SM1DTCNT1             (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM1DTCNT1_OFFSET)     /* FLEXPWM3 Deadtime Count Register 1 */
#define IMXRT_FLEXPWM3_SM1CAPTCTRLA          (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM1CAPTCTRLA_OFFSET)  /* FLEXPWM3 Capture Control A Register */
#define IMXRT_FLEXPWM3_SM1CAPTCOMPA          (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM1CAPTCOMPA_OFFSET)  /* FLEXPWM3 Capture Compare A Register */
#define IMXRT_FLEXPWM3_eFlexPWM              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_eFlexPWM_OFFSET)      /* FLEXPWM3 apter 28 Enhanced Flex Pulse Width Modulator */
#define IMXRT_FLEXPWM3_SM1CAPTCTRLB          (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM1CAPTCTRLB_OFFSET)  /* FLEXPWM3 Capture Control B Register */
#define IMXRT_FLEXPWM3_SM1CAPTCOMPB          (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM1CAPTCOMPB_OFFSET)  /* FLEXPWM3 Capture Compare B Register */
#define IMXRT_FLEXPWM3_SM1CAPTCTRLX          (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM1CAPTCTRLX_OFFSET)  /* FLEXPWM3 Capture Control X Register */
#define IMXRT_FLEXPWM3_SM1CAPTCOMPX          (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM1CAPTCOMPX_OFFSET)  /* FLEXPWM3 Capture Compare X Register */
#define IMXRT_FLEXPWM3_SM1CVAL0              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM1CVAL0_OFFSET)      /* FLEXPWM3 Capture Value 0 Register */
#define IMXRT_FLEXPWM3_SM1CVAL0CYC           (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM1CVAL0CYC_OFFSET)   /* FLEXPWM3 Capture Value 0 Cycle Register */
#define IMXRT_FLEXPWM3_SM1CVAL1              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM1CVAL1_OFFSET)      /* FLEXPWM3 Capture Value 1 Register */
#define IMXRT_FLEXPWM3_SM1CVAL1CYC           (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM1CVAL1CYC_OFFSET)   /* FLEXPWM3 Capture Value 1 Cycle Register */
#define IMXRT_FLEXPWM3_SM1CVAL2              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM1CVAL2_OFFSET)      /* FLEXPWM3 Capture Value 2 Register */
#define IMXRT_FLEXPWM3_SM1CVAL2CYC           (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM1CVAL2CYC_OFFSET)   /* FLEXPWM3 Capture Value 2 Cycle Register */
#define IMXRT_FLEXPWM3_SM1CVAL3              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM1CVAL3_OFFSET)      /* FLEXPWM3 Capture Value 3 Register */
#define IMXRT_FLEXPWM3_SM1CVAL3CYC           (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM1CVAL3CYC_OFFSET)   /* FLEXPWM3 Capture Value 3 Cycle Register */
#define IMXRT_FLEXPWM3_SM1CVAL4              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM1CVAL4_OFFSET)      /* FLEXPWM3 Capture Value 4 Register */
#define IMXRT_FLEXPWM3_SM1CVAL4CYC           (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM1CVAL4CYC_OFFSET)   /* FLEXPWM3 Capture Value 4 Cycle Register */
#define IMXRT_FLEXPWM3_SM1CVAL5              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM1CVAL5_OFFSET)      /* FLEXPWM3 Capture Value 5 Register */
#define IMXRT_FLEXPWM3_SM1CVAL5CYC           (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM1CVAL5CYC_OFFSET)   /* FLEXPWM3 Capture Value 5 Cycle Register */
#define IMXRT_FLEXPWM3_SM2CNT                (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM2CNT_OFFSET)        /* FLEXPWM3 Counter Register */
#define IMXRT_FLEXPWM3_SM2INIT               (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM2INIT_OFFSET)       /* FLEXPWM3 Initial Count Register */
#define IMXRT_FLEXPWM3_SM2CTRL2              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM2CTRL2_OFFSET)      /* FLEXPWM3 Control 2 Register */
#define IMXRT_FLEXPWM3_SM2CTRL               (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM2CTRL_OFFSET)       /* FLEXPWM3 Control Register */
#define IMXRT_FLEXPWM3_SM2VAL0               (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM2VAL0_OFFSET)       /* FLEXPWM3 Value Register 0 */
#define IMXRT_FLEXPWM3_SM2FRACVAL1           (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM2FRACVAL1_OFFSET)   /* FLEXPWM3 Fractional Value Register 1 */
#define IMXRT_FLEXPWM3_SM2VAL1               (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM2VAL1_OFFSET)       /* FLEXPWM3 Value Register 1 */
#define IMXRT_FLEXPWM3_SM2FRACVAL2           (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM2FRACVAL2_OFFSET)   /* FLEXPWM3 Fractional Value Register 2 */
#define IMXRT_FLEXPWM3_SM2VAL2               (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM2VAL2_OFFSET)       /* FLEXPWM3 Value Register 2 */
#define IMXRT_FLEXPWM3_SM2FRACVAL3           (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM2FRACVAL3_OFFSET)   /* FLEXPWM3 Fractional Value Register 3 */
#define IMXRT_FLEXPWM3_SM2VAL3               (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM2VAL3_OFFSET)       /* FLEXPWM3 Value Register 3 */
#define IMXRT_FLEXPWM3_SM2FRACVAL4           (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM2FRACVAL4_OFFSET)   /* FLEXPWM3 Fractional Value Register 4 */
#define IMXRT_FLEXPWM3_SM2VAL4               (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM2VAL4_OFFSET)       /* FLEXPWM3 Value Register 4 */
#define IMXRT_FLEXPWM3_SM2FRACVAL5           (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM2FRACVAL5_OFFSET)   /* FLEXPWM3 Fractional Value Register 5 */
#define IMXRT_FLEXPWM3_SM2VAL5               (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM2VAL5_OFFSET)       /* FLEXPWM3 Value Register 5 */
#define IMXRT_FLEXPWM3_SM2FRCTRL             (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM2FRCTRL_OFFSET)     /* FLEXPWM3 Fractional Control Register */
#define IMXRT_FLEXPWM3_SM2OCTRL              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM2OCTRL_OFFSET)      /* FLEXPWM3 Output Control Register */
#define IMXRT_FLEXPWM3_SM2STS                (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM2STS_OFFSET)        /* FLEXPWM3 Status Register */
#define IMXRT_FLEXPWM3_SM2INTEN              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM2INTEN_OFFSET)      /* FLEXPWM3 Interrupt Enable Register */
#define IMXRT_FLEXPWM3_SM2DMAEN              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM2DMAEN_OFFSET)      /* FLEXPWM3 DMA Enable Register */
#define IMXRT_FLEXPWM3_SM2TCTRL              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM2TCTRL_OFFSET)      /* FLEXPWM3 Output Trigger Control Register */
#define IMXRT_FLEXPWM3_SM2DISMAP0            (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM2DISMAP0_OFFSET)    /* FLEXPWM3 Fault Disable Mapping Register 0 */
#define IMXRT_FLEXPWM3_SM2DISMAP1            (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM2DISMAP1_OFFSET)    /* FLEXPWM3 Fault Disable Mapping Register 1 */
#define IMXRT_FLEXPWM3_SM2DTCNT0             (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM2DTCNT0_OFFSET)     /* FLEXPWM3 Deadtime Count Register 0 */
#define IMXRT_FLEXPWM3_SM2DTCNT1             (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM2DTCNT1_OFFSET)     /* FLEXPWM3 Deadtime Count Register 1 */
#define IMXRT_FLEXPWM3_SM2CAPTCTRLA          (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM2CAPTCTRLA_OFFSET)  /* FLEXPWM3 Capture Control A Register */
#define IMXRT_FLEXPWM3_SM2CAPTCOMPA          (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM2CAPTCOMPA_OFFSET)  /* FLEXPWM3 Capture Compare A Register */
#define IMXRT_FLEXPWM3_SM2CAPTCTRLB          (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM2CAPTCTRLB_OFFSET)  /* FLEXPWM3 Capture Control B Register */
#define IMXRT_FLEXPWM3_SM2CAPTCOMPB          (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM2CAPTCOMPB_OFFSET)  /* FLEXPWM3 Capture Compare B Register */
#define IMXRT_FLEXPWM3_SM2CAPTCTRLX          (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM2CAPTCTRLX_OFFSET)  /* FLEXPWM3 Capture Control X Register */
#define IMXRT_FLEXPWM3_SM2CAPTCOMPX          (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM2CAPTCOMPX_OFFSET)  /* FLEXPWM3 Capture Compare X Register */
#define IMXRT_FLEXPWM3_SM2CVAL0              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM2CVAL0_OFFSET)      /* FLEXPWM3 Capture Value 0 Register */
#define IMXRT_FLEXPWM3_SM2CVAL0CYC           (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM2CVAL0CYC_OFFSET)   /* FLEXPWM3 Capture Value 0 Cycle Register */
#define IMXRT_FLEXPWM3_SM2CVAL1              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM2CVAL1_OFFSET)      /* FLEXPWM3 Capture Value 1 Register */
#define IMXRT_FLEXPWM3_SM2CVAL1CYC           (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM2CVAL1CYC_OFFSET)   /* FLEXPWM3 Capture Value 1 Cycle Register */
#define IMXRT_FLEXPWM3_SM2CVAL2              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM2CVAL2_OFFSET)      /* FLEXPWM3 Capture Value 2 Register */
#define IMXRT_FLEXPWM3_SM2CVAL2CYC           (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM2CVAL2CYC_OFFSET)   /* FLEXPWM3 Capture Value 2 Cycle Register */
#define IMXRT_FLEXPWM3_SM2CVAL3              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM2CVAL3_OFFSET)      /* FLEXPWM3 Capture Value 3 Register */
#define IMXRT_FLEXPWM3_SM2CVAL3CYC           (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM2CVAL3CYC_OFFSET)   /* FLEXPWM3 Capture Value 3 Cycle Register */
#define IMXRT_FLEXPWM3_SM2CVAL4              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM2CVAL4_OFFSET)      /* FLEXPWM3 Capture Value 4 Register */
#define IMXRT_FLEXPWM3_SM2CVAL4CYC           (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM2CVAL4CYC_OFFSET)   /* FLEXPWM3 Capture Value 4 Cycle Register */
#define IMXRT_FLEXPWM3_SM2CVAL5              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM2CVAL5_OFFSET)      /* FLEXPWM3 Capture Value 5 Register */
#define IMXRT_FLEXPWM3_SM2CVAL5CYC           (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM2CVAL5CYC_OFFSET)   /* FLEXPWM3 Capture Value 5 Cycle Register */
#define IMXRT_FLEXPWM3_SM3CNT                (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM3CNT_OFFSET)        /* FLEXPWM3 Counter Register */
#define IMXRT_FLEXPWM3_SM3INIT               (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM3INIT_OFFSET)       /* FLEXPWM3 Initial Count Register */
#define IMXRT_FLEXPWM3_SM3CTRL2              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM3CTRL2_OFFSET)      /* FLEXPWM3 Control 2 Register */
#define IMXRT_FLEXPWM3_SM3CTRL               (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM3CTRL_OFFSET)       /* FLEXPWM3 Control Register */
#define IMXRT_FLEXPWM3_SM3VAL0               (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM3VAL0_OFFSET)       /* FLEXPWM3 Value Register 0 */
#define IMXRT_FLEXPWM3_SM3FRACVAL1           (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM3FRACVAL1_OFFSET)   /* FLEXPWM3 Fractional Value Register 1 */
#define IMXRT_FLEXPWM3_SM3VAL1               (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM3VAL1_OFFSET)       /* FLEXPWM3 Value Register 1 */
#define IMXRT_FLEXPWM3_SM3FRACVAL2           (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM3FRACVAL2_OFFSET)   /* FLEXPWM3 Fractional Value Register 2 */
#define IMXRT_FLEXPWM3_SM3VAL2               (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM3VAL2_OFFSET)       /* FLEXPWM3 Value Register 2 */
#define IMXRT_FLEXPWM3_SM3FRACVAL3           (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM3FRACVAL3_OFFSET)   /* FLEXPWM3 Fractional Value Register 3 */
#define IMXRT_FLEXPWM3_SM3VAL3               (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM3VAL3_OFFSET)       /* FLEXPWM3 Value Register 3 */
#define IMXRT_FLEXPWM3_SM3FRACVAL4           (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM3FRACVAL4_OFFSET)   /* FLEXPWM3 Fractional Value Register 4 */
#define IMXRT_FLEXPWM3_SM3VAL4               (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM3VAL4_OFFSET)       /* FLEXPWM3 Value Register 4 */
#define IMXRT_FLEXPWM3_SM3FRACVAL5           (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM3FRACVAL5_OFFSET)   /* FLEXPWM3 Fractional Value Register 5 */
#define IMXRT_FLEXPWM3_SM3VAL5               (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM3VAL5_OFFSET)       /* FLEXPWM3 Value Register 5 */
#define IMXRT_FLEXPWM3_SM3FRCTRL             (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM3FRCTRL_OFFSET)     /* FLEXPWM3 Fractional Control Register */
#define IMXRT_FLEXPWM3_SM3OCTRL              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM3OCTRL_OFFSET)      /* FLEXPWM3 Output Control Register */
#define IMXRT_FLEXPWM3_SM3STS                (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM3STS_OFFSET)        /* FLEXPWM3 Status Register */
#define IMXRT_FLEXPWM3_SM3INTEN              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM3INTEN_OFFSET)      /* FLEXPWM3 Interrupt Enable Register */
#define IMXRT_FLEXPWM3_SM3DMAEN              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM3DMAEN_OFFSET)      /* FLEXPWM3 DMA Enable Register */
#define IMXRT_FLEXPWM3_SM3TCTRL              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM3TCTRL_OFFSET)      /* FLEXPWM3 Output Trigger Control Register */
#define IMXRT_FLEXPWM3_SM3DISMAP0            (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM3DISMAP0_OFFSET)    /* FLEXPWM3 Fault Disable Mapping Register 0 */
#define IMXRT_FLEXPWM3_SM3DISMAP1            (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM3DISMAP1_OFFSET)    /* FLEXPWM3 Fault Disable Mapping Register 1 */
#define IMXRT_FLEXPWM3_SM3DTCNT0             (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM3DTCNT0_OFFSET)     /* FLEXPWM3 Deadtime Count Register 0 */
#define IMXRT_FLEXPWM3_SM3DTCNT1             (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM3DTCNT1_OFFSET)     /* FLEXPWM3 Deadtime Count Register 1 */
#define IMXRT_FLEXPWM3_SM3CAPTCTRLA          (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM3CAPTCTRLA_OFFSET)  /* FLEXPWM3 Capture Control A Register */
#define IMXRT_FLEXPWM3_SM3CAPTCOMPA          (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM3CAPTCOMPA_OFFSET)  /* FLEXPWM3 Capture Compare A Register */
#define IMXRT_FLEXPWM3_SM3CAPTCTRLB          (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM3CAPTCTRLB_OFFSET)  /* FLEXPWM3 Capture Control B Register */
#define IMXRT_FLEXPWM3_SM3CAPTCOMPB          (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM3CAPTCOMPB_OFFSET)  /* FLEXPWM3 Capture Compare B Register */
#define IMXRT_FLEXPWM3_SM3CAPTCTRLX          (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM3CAPTCTRLX_OFFSET)  /* FLEXPWM3 Capture Control X Register */
#define IMXRT_FLEXPWM3_SM3CAPTCOMPX          (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM3CAPTCOMPX_OFFSET)  /* FLEXPWM3 Capture Compare X Register */
#define IMXRT_FLEXPWM3_SM3CVAL0              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM3CVAL0_OFFSET)      /* FLEXPWM3 Capture Value 0 Register */
#define IMXRT_FLEXPWM3_SM3CVAL0CYC           (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM3CVAL0CYC_OFFSET)   /* FLEXPWM3 Capture Value 0 Cycle Register */
#define IMXRT_FLEXPWM3_SM3CVAL1              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM3CVAL1_OFFSET)      /* FLEXPWM3 Capture Value 1 Register */
#define IMXRT_FLEXPWM3_SM3CVAL1CYC           (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM3CVAL1CYC_OFFSET)   /* FLEXPWM3 Capture Value 1 Cycle Register */
#define IMXRT_FLEXPWM3_SM3CVAL2              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM3CVAL2_OFFSET)      /* FLEXPWM3 Capture Value 2 Register */
#define IMXRT_FLEXPWM3_SM3CVAL2CYC           (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM3CVAL2CYC_OFFSET)   /* FLEXPWM3 Capture Value 2 Cycle Register */
#define IMXRT_FLEXPWM3_SM3CVAL3              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM3CVAL3_OFFSET)      /* FLEXPWM3 Capture Value 3 Register */
#define IMXRT_FLEXPWM3_SM3CVAL3CYC           (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM3CVAL3CYC_OFFSET)   /* FLEXPWM3 Capture Value 3 Cycle Register */
#define IMXRT_FLEXPWM3_SM3CVAL4              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM3CVAL4_OFFSET)      /* FLEXPWM3 Capture Value 4 Register */
#define IMXRT_FLEXPWM3_SM3CVAL4CYC           (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM3CVAL4CYC_OFFSET)   /* FLEXPWM3 Capture Value 4 Cycle Register */
#define IMXRT_FLEXPWM3_SM3CVAL5              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM3CVAL5_OFFSET)      /* FLEXPWM3 Capture Value 5 Register */
#define IMXRT_FLEXPWM3_SM3CVAL5CYC           (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SM3CVAL5CYC_OFFSET)   /* FLEXPWM3 Capture Value 5 Cycle Register */
#define IMXRT_FLEXPWM3_OUTEN                 (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_OUTEN_OFFSET)         /* FLEXPWM3 Output Enable Register */
#define IMXRT_FLEXPWM3_MASK                  (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_MASK_OFFSET)          /* FLEXPWM3 Mask Register */
#define IMXRT_FLEXPWM3_SWCOUT                (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_SWCOUT_OFFSET)        /* FLEXPWM3 Software Controlled Output Register */
#define IMXRT_FLEXPWM3_DTSRCSEL              (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_DTSRCSEL_OFFSET)      /* FLEXPWM3 PWM Source Select Register */
#define IMXRT_FLEXPWM3_MCTRL                 (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_MCTRL_OFFSET)         /* FLEXPWM3 Master Control Register */
#define IMXRT_FLEXPWM3_MCTRL2                (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_MCTRL2_OFFSET)        /* FLEXPWM3 Master Control 2 Register */
#define IMXRT_FLEXPWM3_FCTRL0                (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_FCTRL0_OFFSET)        /* FLEXPWM3 Fault Control Register */
#define IMXRT_FLEXPWM3_FSTS0                 (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_FSTS0_OFFSET)         /* FLEXPWM3 Fault Status Register */
#define IMXRT_FLEXPWM3_FFILT0                (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_FFILT0_OFFSET)        /* FLEXPWM3 Fault Filter Register */
#define IMXRT_FLEXPWM3_FTST0                 (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_FTST0_OFFSET)         /* FLEXPWM3 Fault Test Register */
#define IMXRT_FLEXPWM3_FCTRL20               (IMXRT_FLEXPWM3_BASE + IMXRT_FLEXPWM_FCTRL20_OFFSET)       /* FLEXPWM3 Fault Control 2 Register */

/* FLEXPWM4 Register Addresses */

#define IMXRT_FLEXPWM4_SM0CNT                (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM0CNT_OFFSET)        /* FLEXPWM4 Counter Register */
#define IMXRT_FLEXPWM4_SM0INIT               (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM0INIT_OFFSET)       /* FLEXPWM4 Initial Count Register */
#define IMXRT_FLEXPWM4_SM0CTRL2              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM0CTRL2_OFFSET)      /* FLEXPWM4 Control 2 Register */
#define IMXRT_FLEXPWM4_SM0CTRL               (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM0CTRL_OFFSET)       /* FLEXPWM4 Control Register */
#define IMXRT_FLEXPWM4_SM0VAL0               (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM0VAL0_OFFSET)       /* FLEXPWM4 Value Register 0 */
#define IMXRT_FLEXPWM4_SM0FRACVAL1           (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM0FRACVAL1_OFFSET)   /* FLEXPWM4 Fractional Value Register 1 */
#define IMXRT_FLEXPWM4_SM0VAL1               (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM0VAL1_OFFSET)       /* FLEXPWM4 Value Register 1 */
#define IMXRT_FLEXPWM4_SM0FRACVAL2           (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM0FRACVAL2_OFFSET)   /* FLEXPWM4 Fractional Value Register 2 */
#define IMXRT_FLEXPWM4_SM0VAL2               (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM0VAL2_OFFSET)       /* FLEXPWM4 Value Register 2 */
#define IMXRT_FLEXPWM4_SM0FRACVAL3           (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM0FRACVAL3_OFFSET)   /* FLEXPWM4 Fractional Value Register 3 */
#define IMXRT_FLEXPWM4_SM0VAL3               (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM0VAL3_OFFSET)       /* FLEXPWM4 Value Register 3 */
#define IMXRT_FLEXPWM4_SM0FRACVAL4           (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM0FRACVAL4_OFFSET)   /* FLEXPWM4 Fractional Value Register 4 */
#define IMXRT_FLEXPWM4_SM0VAL4               (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM0VAL4_OFFSET)       /* FLEXPWM4 Value Register 4 */
#define IMXRT_FLEXPWM4_SM0FRACVAL5           (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM0FRACVAL5_OFFSET)   /* FLEXPWM4 Fractional Value Register 5 */
#define IMXRT_FLEXPWM4_SM0VAL5               (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM0VAL5_OFFSET)       /* FLEXPWM4 Value Register 5 */
#define IMXRT_FLEXPWM4_SM0FRCTRL             (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM0FRCTRL_OFFSET)     /* FLEXPWM4 Fractional Control Register */
#define IMXRT_FLEXPWM4_SM0OCTRL              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM0OCTRL_OFFSET)      /* FLEXPWM4 Output Control Register */
#define IMXRT_FLEXPWM4_SM0STS                (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM0STS_OFFSET)        /* FLEXPWM4 Status Register */
#define IMXRT_FLEXPWM4_SM0INTEN              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM0INTEN_OFFSET)      /* FLEXPWM4 Interrupt Enable Register */
#define IMXRT_FLEXPWM4_SM0DMAEN              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM0DMAEN_OFFSET)      /* FLEXPWM4 DMA Enable Register */
#define IMXRT_FLEXPWM4_SM0TCTRL              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM0TCTRL_OFFSET)      /* FLEXPWM4 Output Trigger Control Register */
#define IMXRT_FLEXPWM4_SM0DISMAP0            (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM0DISMAP0_OFFSET)    /* FLEXPWM4 Fault Disable Mapping Register 0 */
#define IMXRT_FLEXPWM4_SM0DISMAP1            (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM0DISMAP1_OFFSET)    /* FLEXPWM4 Fault Disable Mapping Register 1 */
#define IMXRT_FLEXPWM4_SM0DTCNT0             (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM0DTCNT0_OFFSET)     /* FLEXPWM4 Deadtime Count Register 0 */
#define IMXRT_FLEXPWM4_SM0DTCNT1             (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM0DTCNT1_OFFSET)     /* FLEXPWM4 Deadtime Count Register 1 */
#define IMXRT_FLEXPWM4_SM0CAPTCTRLA          (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM0CAPTCTRLA_OFFSET)  /* FLEXPWM4 Capture Control A Register */
#define IMXRT_FLEXPWM4_SM0CAPTCOMPA          (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM0CAPTCOMPA_OFFSET)  /* FLEXPWM4 Capture Compare A Register */
#define IMXRT_FLEXPWM4_SM0CAPTCTRLB          (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM0CAPTCTRLB_OFFSET)  /* FLEXPWM4 Capture Control B Register */
#define IMXRT_FLEXPWM4_SM0CAPTCOMPB          (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM0CAPTCOMPB_OFFSET)  /* FLEXPWM4 Capture Compare B Register */
#define IMXRT_FLEXPWM4_SM0CAPTCTRLX          (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM0CAPTCTRLX_OFFSET)  /* FLEXPWM4 Capture Control X Register */
#define IMXRT_FLEXPWM4_SM0CAPTCOMPX          (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM0CAPTCOMPX_OFFSET)  /* FLEXPWM4 Capture Compare X Register */
#define IMXRT_FLEXPWM4_SM0CVAL0              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM0CVAL0_OFFSET)      /* FLEXPWM4 Capture Value 0 Register */
#define IMXRT_FLEXPWM4_SM0CVAL0CYC           (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM0CVAL0CYC_OFFSET)   /* FLEXPWM4 Capture Value 0 Cycle Register */
#define IMXRT_FLEXPWM4_SM0CVAL1              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM0CVAL1_OFFSET)      /* FLEXPWM4 Capture Value 1 Register */
#define IMXRT_FLEXPWM4_SM0CVAL1CYC           (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM0CVAL1CYC_OFFSET)   /* FLEXPWM4 Capture Value 1 Cycle Register */
#define IMXRT_FLEXPWM4_SM0CVAL2              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM0CVAL2_OFFSET)      /* FLEXPWM4 Capture Value 2 Register */
#define IMXRT_FLEXPWM4_SM0CVAL2CYC           (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM0CVAL2CYC_OFFSET)   /* FLEXPWM4 Capture Value 2 Cycle Register */
#define IMXRT_FLEXPWM4_SM0CVAL3              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM0CVAL3_OFFSET)      /* FLEXPWM4 Capture Value 3 Register */
#define IMXRT_FLEXPWM4_SM0CVAL3CYC           (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM0CVAL3CYC_OFFSET)   /* FLEXPWM4 Capture Value 3 Cycle Register */
#define IMXRT_FLEXPWM4_SM0CVAL4              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM0CVAL4_OFFSET)      /* FLEXPWM4 Capture Value 4 Register */
#define IMXRT_FLEXPWM4_SM0CVAL4CYC           (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM0CVAL4CYC_OFFSET)   /* FLEXPWM4 Capture Value 4 Cycle Register */
#define IMXRT_FLEXPWM4_SM0CVAL5              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM0CVAL5_OFFSET)      /* FLEXPWM4 Capture Value 5 Register */
#define IMXRT_FLEXPWM4_SM0CVAL5CYC           (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM0CVAL5CYC_OFFSET)   /* FLEXPWM4 Capture Value 5 Cycle Register */
#define IMXRT_FLEXPWM4_SM1CNT                (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM1CNT_OFFSET)        /* FLEXPWM4 Counter Register */
#define IMXRT_FLEXPWM4_SM1INIT               (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM1INIT_OFFSET)       /* FLEXPWM4 Initial Count Register */
#define IMXRT_FLEXPWM4_SM1CTRL2              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM1CTRL2_OFFSET)      /* FLEXPWM4 Control 2 Register */
#define IMXRT_FLEXPWM4_SM1CTRL               (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM1CTRL_OFFSET)       /* FLEXPWM4 Control Register */
#define IMXRT_FLEXPWM4_SM1VAL0               (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM1VAL0_OFFSET)       /* FLEXPWM4 Value Register 0 */
#define IMXRT_FLEXPWM4_SM1FRACVAL1           (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM1FRACVAL1_OFFSET)   /* FLEXPWM4 Fractional Value Register 1 */
#define IMXRT_FLEXPWM4_SM1VAL1               (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM1VAL1_OFFSET)       /* FLEXPWM4 Value Register 1 */
#define IMXRT_FLEXPWM4_SM1FRACVAL2           (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM1FRACVAL2_OFFSET)   /* FLEXPWM4 Fractional Value Register 2 */
#define IMXRT_FLEXPWM4_SM1VAL2               (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM1VAL2_OFFSET)       /* FLEXPWM4 Value Register 2 */
#define IMXRT_FLEXPWM4_SM1FRACVAL3           (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM1FRACVAL3_OFFSET)   /* FLEXPWM4 Fractional Value Register 3 */
#define IMXRT_FLEXPWM4_SM1VAL3               (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM1VAL3_OFFSET)       /* FLEXPWM4 Value Register 3 */
#define IMXRT_FLEXPWM4_SM1FRACVAL4           (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM1FRACVAL4_OFFSET)   /* FLEXPWM4 Fractional Value Register 4 */
#define IMXRT_FLEXPWM4_SM1VAL4               (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM1VAL4_OFFSET)       /* FLEXPWM4 Value Register 4 */
#define IMXRT_FLEXPWM4_SM1FRACVAL5           (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM1FRACVAL5_OFFSET)   /* FLEXPWM4 Fractional Value Register 5 */
#define IMXRT_FLEXPWM4_SM1VAL5               (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM1VAL5_OFFSET)       /* FLEXPWM4 Value Register 5 */
#define IMXRT_FLEXPWM4_SM1FRCTRL             (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM1FRCTRL_OFFSET)     /* FLEXPWM4 Fractional Control Register */
#define IMXRT_FLEXPWM4_SM1OCTRL              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM1OCTRL_OFFSET)      /* FLEXPWM4 Output Control Register */
#define IMXRT_FLEXPWM4_SM1STS                (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM1STS_OFFSET)        /* FLEXPWM4 Status Register */
#define IMXRT_FLEXPWM4_SM1INTEN              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM1INTEN_OFFSET)      /* FLEXPWM4 Interrupt Enable Register */
#define IMXRT_FLEXPWM4_SM1DMAEN              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM1DMAEN_OFFSET)      /* FLEXPWM4 DMA Enable Register */
#define IMXRT_FLEXPWM4_SM1TCTRL              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM1TCTRL_OFFSET)      /* FLEXPWM4 Output Trigger Control Register */
#define IMXRT_FLEXPWM4_SM1DISMAP0            (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM1DISMAP0_OFFSET)    /* FLEXPWM4 Fault Disable Mapping Register 0 */
#define IMXRT_FLEXPWM4_SM1DISMAP1            (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM1DISMAP1_OFFSET)    /* FLEXPWM4 Fault Disable Mapping Register 1 */
#define IMXRT_FLEXPWM4_SM1DTCNT0             (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM1DTCNT0_OFFSET)     /* FLEXPWM4 Deadtime Count Register 0 */
#define IMXRT_FLEXPWM4_SM1DTCNT1             (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM1DTCNT1_OFFSET)     /* FLEXPWM4 Deadtime Count Register 1 */
#define IMXRT_FLEXPWM4_SM1CAPTCTRLA          (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM1CAPTCTRLA_OFFSET)  /* FLEXPWM4 Capture Control A Register */
#define IMXRT_FLEXPWM4_SM1CAPTCOMPA          (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM1CAPTCOMPA_OFFSET)  /* FLEXPWM4 Capture Compare A Register */
#define IMXRT_FLEXPWM4_eFlexPWM              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_eFlexPWM_OFFSET)      /* FLEXPWM4 apter 28 Enhanced Flex Pulse Width Modulator */
#define IMXRT_FLEXPWM4_SM1CAPTCTRLB          (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM1CAPTCTRLB_OFFSET)  /* FLEXPWM4 Capture Control B Register */
#define IMXRT_FLEXPWM4_SM1CAPTCOMPB          (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM1CAPTCOMPB_OFFSET)  /* FLEXPWM4 Capture Compare B Register */
#define IMXRT_FLEXPWM4_SM1CAPTCTRLX          (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM1CAPTCTRLX_OFFSET)  /* FLEXPWM4 Capture Control X Register */
#define IMXRT_FLEXPWM4_SM1CAPTCOMPX          (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM1CAPTCOMPX_OFFSET)  /* FLEXPWM4 Capture Compare X Register */
#define IMXRT_FLEXPWM4_SM1CVAL0              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM1CVAL0_OFFSET)      /* FLEXPWM4 Capture Value 0 Register */
#define IMXRT_FLEXPWM4_SM1CVAL0CYC           (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM1CVAL0CYC_OFFSET)   /* FLEXPWM4 Capture Value 0 Cycle Register */
#define IMXRT_FLEXPWM4_SM1CVAL1              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM1CVAL1_OFFSET)      /* FLEXPWM4 Capture Value 1 Register */
#define IMXRT_FLEXPWM4_SM1CVAL1CYC           (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM1CVAL1CYC_OFFSET)   /* FLEXPWM4 Capture Value 1 Cycle Register */
#define IMXRT_FLEXPWM4_SM1CVAL2              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM1CVAL2_OFFSET)      /* FLEXPWM4 Capture Value 2 Register */
#define IMXRT_FLEXPWM4_SM1CVAL2CYC           (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM1CVAL2CYC_OFFSET)   /* FLEXPWM4 Capture Value 2 Cycle Register */
#define IMXRT_FLEXPWM4_SM1CVAL3              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM1CVAL3_OFFSET)      /* FLEXPWM4 Capture Value 3 Register */
#define IMXRT_FLEXPWM4_SM1CVAL3CYC           (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM1CVAL3CYC_OFFSET)   /* FLEXPWM4 Capture Value 3 Cycle Register */
#define IMXRT_FLEXPWM4_SM1CVAL4              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM1CVAL4_OFFSET)      /* FLEXPWM4 Capture Value 4 Register */
#define IMXRT_FLEXPWM4_SM1CVAL4CYC           (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM1CVAL4CYC_OFFSET)   /* FLEXPWM4 Capture Value 4 Cycle Register */
#define IMXRT_FLEXPWM4_SM1CVAL5              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM1CVAL5_OFFSET)      /* FLEXPWM4 Capture Value 5 Register */
#define IMXRT_FLEXPWM4_SM1CVAL5CYC           (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM1CVAL5CYC_OFFSET)   /* FLEXPWM4 Capture Value 5 Cycle Register */
#define IMXRT_FLEXPWM4_SM2CNT                (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM2CNT_OFFSET)        /* FLEXPWM4 Counter Register */
#define IMXRT_FLEXPWM4_SM2INIT               (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM2INIT_OFFSET)       /* FLEXPWM4 Initial Count Register */
#define IMXRT_FLEXPWM4_SM2CTRL2              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM2CTRL2_OFFSET)      /* FLEXPWM4 Control 2 Register */
#define IMXRT_FLEXPWM4_SM2CTRL               (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM2CTRL_OFFSET)       /* FLEXPWM4 Control Register */
#define IMXRT_FLEXPWM4_SM2VAL0               (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM2VAL0_OFFSET)       /* FLEXPWM4 Value Register 0 */
#define IMXRT_FLEXPWM4_SM2FRACVAL1           (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM2FRACVAL1_OFFSET)   /* FLEXPWM4 Fractional Value Register 1 */
#define IMXRT_FLEXPWM4_SM2VAL1               (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM2VAL1_OFFSET)       /* FLEXPWM4 Value Register 1 */
#define IMXRT_FLEXPWM4_SM2FRACVAL2           (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM2FRACVAL2_OFFSET)   /* FLEXPWM4 Fractional Value Register 2 */
#define IMXRT_FLEXPWM4_SM2VAL2               (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM2VAL2_OFFSET)       /* FLEXPWM4 Value Register 2 */
#define IMXRT_FLEXPWM4_SM2FRACVAL3           (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM2FRACVAL3_OFFSET)   /* FLEXPWM4 Fractional Value Register 3 */
#define IMXRT_FLEXPWM4_SM2VAL3               (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM2VAL3_OFFSET)       /* FLEXPWM4 Value Register 3 */
#define IMXRT_FLEXPWM4_SM2FRACVAL4           (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM2FRACVAL4_OFFSET)   /* FLEXPWM4 Fractional Value Register 4 */
#define IMXRT_FLEXPWM4_SM2VAL4               (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM2VAL4_OFFSET)       /* FLEXPWM4 Value Register 4 */
#define IMXRT_FLEXPWM4_SM2FRACVAL5           (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM2FRACVAL5_OFFSET)   /* FLEXPWM4 Fractional Value Register 5 */
#define IMXRT_FLEXPWM4_SM2VAL5               (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM2VAL5_OFFSET)       /* FLEXPWM4 Value Register 5 */
#define IMXRT_FLEXPWM4_SM2FRCTRL             (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM2FRCTRL_OFFSET)     /* FLEXPWM4 Fractional Control Register */
#define IMXRT_FLEXPWM4_SM2OCTRL              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM2OCTRL_OFFSET)      /* FLEXPWM4 Output Control Register */
#define IMXRT_FLEXPWM4_SM2STS                (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM2STS_OFFSET)        /* FLEXPWM4 Status Register */
#define IMXRT_FLEXPWM4_SM2INTEN              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM2INTEN_OFFSET)      /* FLEXPWM4 Interrupt Enable Register */
#define IMXRT_FLEXPWM4_SM2DMAEN              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM2DMAEN_OFFSET)      /* FLEXPWM4 DMA Enable Register */
#define IMXRT_FLEXPWM4_SM2TCTRL              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM2TCTRL_OFFSET)      /* FLEXPWM4 Output Trigger Control Register */
#define IMXRT_FLEXPWM4_SM2DISMAP0            (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM2DISMAP0_OFFSET)    /* FLEXPWM4 Fault Disable Mapping Register 0 */
#define IMXRT_FLEXPWM4_SM2DISMAP1            (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM2DISMAP1_OFFSET)    /* FLEXPWM4 Fault Disable Mapping Register 1 */
#define IMXRT_FLEXPWM4_SM2DTCNT0             (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM2DTCNT0_OFFSET)     /* FLEXPWM4 Deadtime Count Register 0 */
#define IMXRT_FLEXPWM4_SM2DTCNT1             (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM2DTCNT1_OFFSET)     /* FLEXPWM4 Deadtime Count Register 1 */
#define IMXRT_FLEXPWM4_SM2CAPTCTRLA          (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM2CAPTCTRLA_OFFSET)  /* FLEXPWM4 Capture Control A Register */
#define IMXRT_FLEXPWM4_SM2CAPTCOMPA          (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM2CAPTCOMPA_OFFSET)  /* FLEXPWM4 Capture Compare A Register */
#define IMXRT_FLEXPWM4_SM2CAPTCTRLB          (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM2CAPTCTRLB_OFFSET)  /* FLEXPWM4 Capture Control B Register */
#define IMXRT_FLEXPWM4_SM2CAPTCOMPB          (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM2CAPTCOMPB_OFFSET)  /* FLEXPWM4 Capture Compare B Register */
#define IMXRT_FLEXPWM4_SM2CAPTCTRLX          (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM2CAPTCTRLX_OFFSET)  /* FLEXPWM4 Capture Control X Register */
#define IMXRT_FLEXPWM4_SM2CAPTCOMPX          (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM2CAPTCOMPX_OFFSET)  /* FLEXPWM4 Capture Compare X Register */
#define IMXRT_FLEXPWM4_SM2CVAL0              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM2CVAL0_OFFSET)      /* FLEXPWM4 Capture Value 0 Register */
#define IMXRT_FLEXPWM4_SM2CVAL0CYC           (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM2CVAL0CYC_OFFSET)   /* FLEXPWM4 Capture Value 0 Cycle Register */
#define IMXRT_FLEXPWM4_SM2CVAL1              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM2CVAL1_OFFSET)      /* FLEXPWM4 Capture Value 1 Register */
#define IMXRT_FLEXPWM4_SM2CVAL1CYC           (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM2CVAL1CYC_OFFSET)   /* FLEXPWM4 Capture Value 1 Cycle Register */
#define IMXRT_FLEXPWM4_SM2CVAL2              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM2CVAL2_OFFSET)      /* FLEXPWM4 Capture Value 2 Register */
#define IMXRT_FLEXPWM4_SM2CVAL2CYC           (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM2CVAL2CYC_OFFSET)   /* FLEXPWM4 Capture Value 2 Cycle Register */
#define IMXRT_FLEXPWM4_SM2CVAL3              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM2CVAL3_OFFSET)      /* FLEXPWM4 Capture Value 3 Register */
#define IMXRT_FLEXPWM4_SM2CVAL3CYC           (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM2CVAL3CYC_OFFSET)   /* FLEXPWM4 Capture Value 3 Cycle Register */
#define IMXRT_FLEXPWM4_SM2CVAL4              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM2CVAL4_OFFSET)      /* FLEXPWM4 Capture Value 4 Register */
#define IMXRT_FLEXPWM4_SM2CVAL4CYC           (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM2CVAL4CYC_OFFSET)   /* FLEXPWM4 Capture Value 4 Cycle Register */
#define IMXRT_FLEXPWM4_SM2CVAL5              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM2CVAL5_OFFSET)      /* FLEXPWM4 Capture Value 5 Register */
#define IMXRT_FLEXPWM4_SM2CVAL5CYC           (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM2CVAL5CYC_OFFSET)   /* FLEXPWM4 Capture Value 5 Cycle Register */
#define IMXRT_FLEXPWM4_SM3CNT                (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM3CNT_OFFSET)        /* FLEXPWM4 Counter Register */
#define IMXRT_FLEXPWM4_SM3INIT               (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM3INIT_OFFSET)       /* FLEXPWM4 Initial Count Register */
#define IMXRT_FLEXPWM4_SM3CTRL2              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM3CTRL2_OFFSET)      /* FLEXPWM4 Control 2 Register */
#define IMXRT_FLEXPWM4_SM3CTRL               (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM3CTRL_OFFSET)       /* FLEXPWM4 Control Register */
#define IMXRT_FLEXPWM4_SM3VAL0               (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM3VAL0_OFFSET)       /* FLEXPWM4 Value Register 0 */
#define IMXRT_FLEXPWM4_SM3FRACVAL1           (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM3FRACVAL1_OFFSET)   /* FLEXPWM4 Fractional Value Register 1 */
#define IMXRT_FLEXPWM4_SM3VAL1               (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM3VAL1_OFFSET)       /* FLEXPWM4 Value Register 1 */
#define IMXRT_FLEXPWM4_SM3FRACVAL2           (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM3FRACVAL2_OFFSET)   /* FLEXPWM4 Fractional Value Register 2 */
#define IMXRT_FLEXPWM4_SM3VAL2               (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM3VAL2_OFFSET)       /* FLEXPWM4 Value Register 2 */
#define IMXRT_FLEXPWM4_SM3FRACVAL3           (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM3FRACVAL3_OFFSET)   /* FLEXPWM4 Fractional Value Register 3 */
#define IMXRT_FLEXPWM4_SM3VAL3               (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM3VAL3_OFFSET)       /* FLEXPWM4 Value Register 3 */
#define IMXRT_FLEXPWM4_SM3FRACVAL4           (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM3FRACVAL4_OFFSET)   /* FLEXPWM4 Fractional Value Register 4 */
#define IMXRT_FLEXPWM4_SM3VAL4               (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM3VAL4_OFFSET)       /* FLEXPWM4 Value Register 4 */
#define IMXRT_FLEXPWM4_SM3FRACVAL5           (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM3FRACVAL5_OFFSET)   /* FLEXPWM4 Fractional Value Register 5 */
#define IMXRT_FLEXPWM4_SM3VAL5               (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM3VAL5_OFFSET)       /* FLEXPWM4 Value Register 5 */
#define IMXRT_FLEXPWM4_SM3FRCTRL             (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM3FRCTRL_OFFSET)     /* FLEXPWM4 Fractional Control Register */
#define IMXRT_FLEXPWM4_SM3OCTRL              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM3OCTRL_OFFSET)      /* FLEXPWM4 Output Control Register */
#define IMXRT_FLEXPWM4_SM3STS                (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM3STS_OFFSET)        /* FLEXPWM4 Status Register */
#define IMXRT_FLEXPWM4_SM3INTEN              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM3INTEN_OFFSET)      /* FLEXPWM4 Interrupt Enable Register */
#define IMXRT_FLEXPWM4_SM3DMAEN              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM3DMAEN_OFFSET)      /* FLEXPWM4 DMA Enable Register */
#define IMXRT_FLEXPWM4_SM3TCTRL              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM3TCTRL_OFFSET)      /* FLEXPWM4 Output Trigger Control Register */
#define IMXRT_FLEXPWM4_SM3DISMAP0            (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM3DISMAP0_OFFSET)    /* FLEXPWM4 Fault Disable Mapping Register 0 */
#define IMXRT_FLEXPWM4_SM3DISMAP1            (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM3DISMAP1_OFFSET)    /* FLEXPWM4 Fault Disable Mapping Register 1 */
#define IMXRT_FLEXPWM4_SM3DTCNT0             (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM3DTCNT0_OFFSET)     /* FLEXPWM4 Deadtime Count Register 0 */
#define IMXRT_FLEXPWM4_SM3DTCNT1             (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM3DTCNT1_OFFSET)     /* FLEXPWM4 Deadtime Count Register 1 */
#define IMXRT_FLEXPWM4_SM3CAPTCTRLA          (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM3CAPTCTRLA_OFFSET)  /* FLEXPWM4 Capture Control A Register */
#define IMXRT_FLEXPWM4_SM3CAPTCOMPA          (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM3CAPTCOMPA_OFFSET)  /* FLEXPWM4 Capture Compare A Register */
#define IMXRT_FLEXPWM4_SM3CAPTCTRLB          (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM3CAPTCTRLB_OFFSET)  /* FLEXPWM4 Capture Control B Register */
#define IMXRT_FLEXPWM4_SM3CAPTCOMPB          (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM3CAPTCOMPB_OFFSET)  /* FLEXPWM4 Capture Compare B Register */
#define IMXRT_FLEXPWM4_SM3CAPTCTRLX          (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM3CAPTCTRLX_OFFSET)  /* FLEXPWM4 Capture Control X Register */
#define IMXRT_FLEXPWM4_SM3CAPTCOMPX          (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM3CAPTCOMPX_OFFSET)  /* FLEXPWM4 Capture Compare X Register */
#define IMXRT_FLEXPWM4_SM3CVAL0              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM3CVAL0_OFFSET)      /* FLEXPWM4 Capture Value 0 Register */
#define IMXRT_FLEXPWM4_SM3CVAL0CYC           (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM3CVAL0CYC_OFFSET)   /* FLEXPWM4 Capture Value 0 Cycle Register */
#define IMXRT_FLEXPWM4_SM3CVAL1              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM3CVAL1_OFFSET)      /* FLEXPWM4 Capture Value 1 Register */
#define IMXRT_FLEXPWM4_SM3CVAL1CYC           (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM3CVAL1CYC_OFFSET)   /* FLEXPWM4 Capture Value 1 Cycle Register */
#define IMXRT_FLEXPWM4_SM3CVAL2              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM3CVAL2_OFFSET)      /* FLEXPWM4 Capture Value 2 Register */
#define IMXRT_FLEXPWM4_SM3CVAL2CYC           (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM3CVAL2CYC_OFFSET)   /* FLEXPWM4 Capture Value 2 Cycle Register */
#define IMXRT_FLEXPWM4_SM3CVAL3              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM3CVAL3_OFFSET)      /* FLEXPWM4 Capture Value 3 Register */
#define IMXRT_FLEXPWM4_SM3CVAL3CYC           (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM3CVAL3CYC_OFFSET)   /* FLEXPWM4 Capture Value 3 Cycle Register */
#define IMXRT_FLEXPWM4_SM3CVAL4              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM3CVAL4_OFFSET)      /* FLEXPWM4 Capture Value 4 Register */
#define IMXRT_FLEXPWM4_SM3CVAL4CYC           (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM3CVAL4CYC_OFFSET)   /* FLEXPWM4 Capture Value 4 Cycle Register */
#define IMXRT_FLEXPWM4_SM3CVAL5              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM3CVAL5_OFFSET)      /* FLEXPWM4 Capture Value 5 Register */
#define IMXRT_FLEXPWM4_SM3CVAL5CYC           (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SM3CVAL5CYC_OFFSET)   /* FLEXPWM4 Capture Value 5 Cycle Register */
#define IMXRT_FLEXPWM4_OUTEN                 (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_OUTEN_OFFSET)         /* FLEXPWM4 Output Enable Register */
#define IMXRT_FLEXPWM4_MASK                  (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_MASK_OFFSET)          /* FLEXPWM4 Mask Register */
#define IMXRT_FLEXPWM4_SWCOUT                (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_SWCOUT_OFFSET)        /* FLEXPWM4 Software Controlled Output Register */
#define IMXRT_FLEXPWM4_DTSRCSEL              (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_DTSRCSEL_OFFSET)      /* FLEXPWM4 PWM Source Select Register */
#define IMXRT_FLEXPWM4_MCTRL                 (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_MCTRL_OFFSET)         /* FLEXPWM4 Master Control Register */
#define IMXRT_FLEXPWM4_MCTRL2                (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_MCTRL2_OFFSET)        /* FLEXPWM4 Master Control 2 Register */
#define IMXRT_FLEXPWM4_FCTRL0                (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_FCTRL0_OFFSET)        /* FLEXPWM4 Fault Control Register */
#define IMXRT_FLEXPWM4_FSTS0                 (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_FSTS0_OFFSET)         /* FLEXPWM4 Fault Status Register */
#define IMXRT_FLEXPWM4_FFILT0                (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_FFILT0_OFFSET)        /* FLEXPWM4 Fault Filter Register */
#define IMXRT_FLEXPWM4_FTST0                 (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_FTST0_OFFSET)         /* FLEXPWM4 Fault Test Register */
#define IMXRT_FLEXPWM4_FCTRL20               (IMXRT_FLEXPWM4_BASE + IMXRT_FLEXPWM_FCTRL20_OFFSET)       /* FLEXPWM4 Fault Control 2 Register */

/* Register Bit Definitions *********************************************************
 *
 *   A single FLEXPWM[n] module (where n is 1..4) has submodules 0 - 3
 * FLEXPWM module Base addresses are spaced every 0x4000 bytes starting at
 * IMXRT_FLEXPWM1_BASE. Therefore each PWM has A0-A3 and B0-B3 outputs.
 * The is 8 outputs per FLEXPWM module.
 *
 *   Each submodule instance is 0x60 bytes.
 *
 *   The address of a register is the sum of a base address and an address offset.
 * The base address is defined as the module IMXRT_FLEXPWMn_BASE (n=1..4), and
 * the address offset is defined at the module level. Each PWM module has a set
 * of registers for each PWM submodule, for the configuration logic, and
 * for each fault channel. While the registers are 16-bits wide, they can be
 * accessed in pairs as 32-bit registers.
 *
 *   Submodule registers are repeated for each PWM submodule. To designate which
 * submodule they are in, register names are prefixed with SM0, SM1, SM2, and SM3
 * Since all these register definitions are identical the defines herein drops the
 * number 0-3 from the prefix.
 *
 *   For example the 'Status Register' appears 4 times per module
 * (IMXRT_FLEXPWMn_BASE) as IMXRT_FLEXPWM_SM0STS_OFFSET, IMXRT_FLEXPWM_SM1STS_OFFSET,
 * IMXRT_FLEXPWM_SM2STS_OFFSET and IMXRT_FLEXPWM_SM3STS_OFFSET. But the bit
 * definitions for the  'Status Register' are defined as SMSTS_xxxxx (with the number
 * dropped.
 *
 *   The base address of submodule 0 is the same as the base address for the PWM
 * module as a whole. The base address of submodule 1 is offset 0x60 from the base
 * address for the PWM module as a whole. This 0x60 offset is based on the number
 * of registers in a submodule.  The base address of submodule 2 is equal to the
 * base address of submodule 1 plus this same 0x60 offset. The pattern repeats for
 * the base address of submodule 3
 * .
 *   The base address of the module configuration registers is equal to the base
 * address of the PWM module as a whole plus an offset of 0x180.
 *
 *   Fault channel registers are repeated for each fault channel. To designate
 * which fault channel they are in, register names are prefixed with F0 and F1. The
 * base address of fault channel 0 is equal to the base address of the PWM module
 * as a whole plus an offset of 0x18C. The base address of fault channel 1 is the
 * base address of fault channel 0 + 4.  This 4 offset is based on the number of
 * registers in a fault channel. Each of the four fields in the fault channel
 * registers corresponds to fault inputs 3-0.
 */

/* Control 2 Register */

#define SMCTRL2_CLK_SEL_SHIFT                (0)        /* Bits: 0-1  Clock Source Select */
#define SMCTRL2_CLK_SEL_MASK                 (3 << SMCTRL2_CLK_SEL_SHIFT)
#  define SMCTRL2_CLK_SEL(n)                 ((uint32_t)(n) << SMCTRL2_CLK_SEL_SHIFT)
#  define SMCTRL2_CLK_SEL_IPG_CLK            (0 << SMCTRL2_CLK_SEL_SHIFT)  /* The IPBus clock is used as the clock for the local prescaler and counter. */
#  define SMCTRL2_CLK_SEL_EXT_CLK            (1 << SMCTRL2_CLK_SEL_SHIFT)  /* EXT_CLK is used as the clock for the local prescaler and counter. */
#  define SMCTRL2_CLK_SEL_AUX_CLK            (2 << SMCTRL2_CLK_SEL_SHIFT)  /* Submodule 0’s clock (AUX_CLK) is used as the source clock for the local prescaler and counter. */
#define SMCTRL2_RELOAD_SEL                   (1 << 2)   /* Bit: 2  Reload Source Select */
#  define SMCTRL2_RELOAD_SEL_LOCAL           (0 << 2)   /* Reload Source is local */
#  define SMCTRL2_RELOAD_SEL_SM0             (1 << 2)   /* Reload Source is submodule 0 */
#define SMCTRL2_FORCE_SEL_SHIFT              (3)        /* Bits: 3-5  This read/write bit determines the source of the FORCE OUTPUT signal for this submodule. */
#define SMCTRL2_FORCE_SEL_MASK               (7 << SMCTRL2_FORCE_SEL_SHIFT)
#  define SMCTRL2_FORCE_SEL(n)               ((uint32_t)(n) << SMCTRL2_FORCE_SEL_SHIFT)
#  define SMCTRL2_FORCE_SEL_CTRL2_FORCE      (0 << SMCTRL2_FORCE_SEL_SHIFT)  /* The local force signal, CTRL2[FORCE], from this submodule is used to force updates. */
#  define SMCTRL2_FORCE_SEL_SM0              (1 << SMCTRL2_FORCE_SEL_SHIFT)  /* The master force signal from submodule 0 is used to force updates. */
#  define SMCTRL2_FORCE_SEL_LOCAL            (2 << SMCTRL2_FORCE_SEL_SHIFT)  /* The local reload signal from this submodule is used to force updates without regard to the state of LDOK. */
#  define SMCTRL2_FORCE_SEL_SM0_LDOK         (3 << SMCTRL2_FORCE_SEL_SHIFT)  /* The master reload signal from submodule0 is used to force updates if LDOK is set.  */
#  define SMCTRL2_FORCE_SEL_LOCAL_SYNC       (4 << SMCTRL2_FORCE_SEL_SHIFT)  /* The local sync signal from this submodule is used to force updates. */
#  define SMCTRL2_FORCE_SEL_SM0_SYNC         (5 << SMCTRL2_FORCE_SEL_SHIFT)  /* The master sync signal from submodule0 is used to force updates. */
#  define SMCTRL2_FORCE_SEL_EXT_FORCE        (6 << SMCTRL2_FORCE_SEL_SHIFT)  /* The external force signal, EXT_FORCE, from outside the PWM module causes updates. */
#  define SMCTRL2_FORCE_SEL_EXT_SYNC         (7 << SMCTRL2_FORCE_SEL_SHIFT)  /* The external sync signal, EXT_SYNC, from outside the PWM module causes updates */
#define SMCTRL2_FORCE                        (1 << 6)   /* Bit: 6  Force Initialization */
#define SMCTRL2_FRCEN                        (1 << 7)   /* Bit: 7  FRCEN */
#define SMCTRL2_INIT_SEL_SHIFT               (8)        /* Bits: 8-9  Initialization Control Select */
#define SMCTRL2_INIT_SEL_MASK                (3 << SMCTRL2_INIT_SEL_SHIFT)
#  define SMCTRL2_INIT_SEL(n)                ((uint32_t)(n) << SMCTRL2_INIT_SEL_SHIFT)
#  define SMCTRL2_INIT_SEL_LOCAL             (0 << SMCTRL2_INIT_SEL_SHIFT)  /* Local sync (PWM_X) causes initialization. */
#  define SMCTRL2_INIT_SEL_SM0               (1 << SMCTRL2_INIT_SEL_SHIFT)  /* Master reload from submodule 0 causes initialization. */
#  define SMCTRL2_INIT_SEL_SM0_SYNC          (2 << SMCTRL2_INIT_SEL_SHIFT)  /* Master sync from submodule 0 causes initialization. */
#  define SMCTRL2_INIT_SEL_EXT_SYNC          (3 << SMCTRL2_INIT_SEL_SHIFT)  /* EXT_SYNC causes initialization. */
#define SMCTRL2_PWMX_INIT                    (1 << 10)  /* Bit: 10 PWM_X Initial Value */
#define SMCTRL2_PWM45_INIT                   (1 << 11)  /* Bit: 11 PWM45 Initial Value */
#define SMCTRL2_PWM23_INIT                   (1 << 12)  /* Bit: 12 PWM23 Initial Value */
#define SMCTRL2_INDEP                        (1 << 13)  /* Bit: 13 Independent or Complementary Pair Operation */
#define SMCTRL2_WAITEN                       (1 << 14)  /* Bit: 14 WAIT Enable */
#define SMCTRL2_DBGEN                        (1 << 15)  /* Bit: 15 Debug Enable */

/* Control Register */

#define SMCTRL_DBLEN                         (1 << 0)   /* Bit: 0  Double Switching Enable */
#define SMCTRL_DBLX                          (1 << 1)   /* Bit: 1  PWMX Double Switching Enable */
#define SMCTRL_LDMOD                         (1 << 2)   /* Bit: 2  Load Mode Select */
#define SMCTRL_SPLIT                         (1 << 3)   /* Bit: 3  Split the DBLPWM signal to PWMA and PWMB */
#define SMCTRL_PRSC_SHIFT                    (4)        /* Bits: 4-6  Prescaler */
#define SMCTRL_PRSC_MASK                     (7 << SMCTRL_PRSC_SHIFT)
#  define SMCTRL_PRSC(n)                     ((uint32_t)(n) << SMCTRL_PRSC_SHIFT)
#  define SMCTRL_PRSC_DIV1                   (0 << SMCTRL_PRSC_SHIFT)  /* PWM clock frequency = fclk */
#  define SMCTRL_PRSC_DIV2                   (1 << SMCTRL_PRSC_SHIFT)  /* PWM clock frequency = fclk/2 */
#  define SMCTRL_PRSC_DIV4                   (2 << SMCTRL_PRSC_SHIFT)  /* PWM clock frequency = fclk/4 */
#  define SMCTRL_PRSC_DIV8                   (3 << SMCTRL_PRSC_SHIFT)  /* PWM clock frequency = fclk/8 */
#  define SMCTRL_PRSC_DIV16                  (4 << SMCTRL_PRSC_SHIFT)  /* PWM clock frequency = fclk/16 */
#  define SMCTRL_PRSC_DIV32                  (5 << SMCTRL_PRSC_SHIFT)  /* PWM clock frequency = fclk/32 */
#  define SMCTRL_PRSC_DIV64                  (6 << SMCTRL_PRSC_SHIFT)  /* PWM clock frequency = fclk/64 */
#  define SMCTRL_PRSC_DIV128                 (7 << SMCTRL_PRSC_SHIFT)  /* PWM clock frequency = fclk/128*/
#define SMCTRL_COMPMODE                      (1 << 7)   /* Bit: 7  Compare Mode */
#define SMCTRL_DT_SHIFT                      (8)        /* Bits: 8-9  Deadtime */
#define SMCTRL_DT_MASK                       (3 << SMCTRL_DT_SHIFT)
#  define SMCTRL_DT(n)                       ((uint32_t)(n) << SMCTRL_DT_SHIFT)
#  define SMCTRL_DT0                         (1 << SMCTRL_DT_SHIFT)  /* Read Only. These read only bits reflect the sampled values of the PWM_X input */
#  define SMCTRL_DT1                         (2 << SMCTRL_DT_SHIFT)  /* Sampling occurs at the end of deadtime 0 for DT[0] and the end of deadtime 1 for DT[1]. */
#define SMCTRL_FULL                          (1 << 10)  /* Bit: 10 Full Cycle Reload */
#define SMCTRL_HALF                          (1 << 11)  /* Bit: 11 Half Cycle Reload */
#define SMCTRL_LDFQ_SHIFT                    (12)       /* Bits: 12-15  Load Frequency */
#define SMCTRL_LDFQ_MASK                     (0xf << SMCTRL_LDFQ_SHIFT)
#  define SMCTRL_LDFQ(n)                     ((uint32_t)(n) << SMCTRL_LDFQ_SHIFT)
#  define SMCTRL_LDFQ_EVERY                  (0 << SMCTRL_LDFQ_SHIFT)   /* Every PWM opportunity */
#  define SMCTRL_LDFQ_EVERY2                 (1 << SMCTRL_LDFQ_SHIFT)   /* Every 2 PWM opportunities */
#  define SMCTRL_LDFQ_EVERY3                 (2 << SMCTRL_LDFQ_SHIFT)   /* Every 3 PWM opportunities */
#  define SMCTRL_LDFQ_EVERY4                 (3 << SMCTRL_LDFQ_SHIFT)   /* Every 4 PWM opportunities */
#  define SMCTRL_LDFQ_EVERY5                 (4 << SMCTRL_LDFQ_SHIFT)   /* Every 5 PWM opportunities */
#  define SMCTRL_LDFQ_EVERY6                 (5 << SMCTRL_LDFQ_SHIFT)   /* Every 6 PWM opportunities */
#  define SMCTRL_LDFQ_EVERY7                 (6 << SMCTRL_LDFQ_SHIFT)   /* Every 7 PWM opportunities */
#  define SMCTRL_LDFQ_EVERY8                 (7 << SMCTRL_LDFQ_SHIFT)   /* Every 8 PWM opportunities */
#  define SMCTRL_LDFQ_EVERY9                 (8 << SMCTRL_LDFQ_SHIFT)   /* Every 9 PWM opportunities */
#  define SMCTRL_LDFQ_EVERY10                (9 << SMCTRL_LDFQ_SHIFT)   /* Every 10 PWM opportunities */
#  define SMCTRL_LDFQ_EVERY11                (10 << SMCTRL_LDFQ_SHIFT)  /* Every 11 PWM opportunities  */
#  define SMCTRL_LDFQ_EVERY12                (11 << SMCTRL_LDFQ_SHIFT)  /* Every 12 PWM opportunities  */
#  define SMCTRL_LDFQ_EVERY13                (12 << SMCTRL_LDFQ_SHIFT)  /* Every 13 PWM opportunities  */
#  define SMCTRL_LDFQ_EVERY14                (13 << SMCTRL_LDFQ_SHIFT)  /* Every 14 PWM opportunities  */
#  define SMCTRL_LDFQ_EVERY15                (14 << SMCTRL_LDFQ_SHIFT)  /* Every 15 PWM opportunities  */
#  define SMCTRL_LDFQ_EVERY16                (0xf << SMCTRL_LDFQ_SHIFT)  /* Every 16 PWM opportunities */

/* Fractional Value Register 1 */

#define SMFRACV_FRACVAL1_SHIFT               (11)       /* Bits: 11-15  Fractional Value 1 Register */
#define SMFRACV_FRACVAL1_MASK                (0x1f << SMFRACV_FRACVAL1_SHIFT)
#  define SMFRACV_FRACVAL1(n)                ((uint32_t)(n) << SMFRACV_FRACVAL1_SHIFT)

/* Fractional Value Register 2 */

#define SMFRACV_FRACVAL2_SHIFT               (11)       /* Bits: 11-15  Fractional Value 2 Register */
#define SMFRACV_FRACVAL2_MASK                (0x1f << SMFRACV_FRACVAL2_SHIFT)
#  define SMFRACV_FRACVAL2(n)                ((uint32_t)(n) << SMFRACV_FRACVAL2_SHIFT)

/* Fractional Value Register 3 */

#define SMFRACV_FRACVAL3_SHIFT               (11)       /* Bits: 11-15  Fractional Value 3 Register */
#define SMFRACV_FRACVAL3_MASK                (0x1f << SMFRACV_FRACVAL3_SHIFT)
#  define SMFRACV_FRACVAL3(n)                ((uint32_t)(n) << SMFRACV_FRACVAL3_SHIFT)

/* Fractional Value Register 4 */

#define SMFRACV_FRACVAL4_SHIFT               (11)       /* Bits: 11-15  Fractional Value 4 Register */
#define SMFRACV_FRACVAL4_MASK                (0x1f << SMFRACV_FRACVAL4_SHIFT)
#  define SMFRACV_FRACVAL4(n)                ((uint32_t)(n) << SMFRACV_FRACVAL4_SHIFT)

/* Fractional Value Register 5 */

#define SMFRACV_FRACVAL5_SHIFT               (11)       /* Bits: 11-15  Fractional Value 5 Register */
#define SMFRACV_FRACVAL5_MASK                (0x1f << SMFRACV_FRACVAL5_SHIFT)
#  define SMFRACV_FRACVAL5(n)                ((uint32_t)(n) << SMFRACV_FRACVAL5_SHIFT)

/* Fractional Control Register */

                                                        /* Bit: 0 Reserved */
#define SMFRCTRL_FRAC1_EN                    (1 << 1)   /* Bit: 1  Fractional Cycle PWM Period Enable */
#define SMFRCTRL_FRAC23_EN                   (1 << 2)   /* Bit: 2  Fractional Cycle Placement Enable for PWM_A */
                                                        /* Bit: 3 Reserved */
#define SMFRCTRL_FRAC45_EN                   (1 << 4)   /* Bit: 4  Fractional Cycle Placement Enable for PWM_B */
                                                        /* Bits: 5-7 Reserved */
#define SMFRCTRL_FRAC_PU                     (1 << 8)   /* Bit: 8  Fractional Delay Circuit Power Up */
                                                        /* Bits: 9-15 Reserved */
#define SMFRCTRL_TEST                        (1 << 15)  /* Bit: 15 Test Status Bit */

/* Output Control Register */

#define SMOCTRL_PWMXFS_SHIFT                 (0)        /* Bits: 0-1  PWM_X Fault State */
#define SMOCTRL_PWMXFS_MASK                  (3 << SMOCTRL_PWMXFS_SHIFT)
#  define SMOCTRL_PWMXFS(n)                  ((uint32_t)(n) << SMOCTRL_PWMXFS_SHIFT)
#  define SMOCTRL_PWMXFS_0                   (0 << SMOCTRL_PWMXFS_SHIFT)  /* Output is forced to logic 0 state prior to consideration of output polarity control. */
#  define SMOCTRL_PWMXFS_1                   (1 << SMOCTRL_PWMXFS_SHIFT)  /* Output is forced to logic 1 state prior to consideration of output polarity control. */
#  define SMOCTRL_PWMXFS_TRISTATE            (2 << SMOCTRL_PWMXFS_SHIFT)  /* Output is tristated. */
#define SMOCTRL_PWMBFS_SHIFT                 (2)        /* Bits: 2-3  PWM_B Fault State */
#define SMOCTRL_PWMBFS_MASK                  (3 << SMOCTRL_PWMBFS_SHIFT)
#  define SMOCTRL_PWMBFS(n)                  ((uint32_t)(n) << SMOCTRL_PWMBFS_SHIFT)
#  define SMOCTRL_PWMBFS_0                   (0 << SMOCTRL_PWMBFS_SHIFT)  /* Output is forced to logic 0 state prior to consideration of output polarity control. */
#  define SMOCTRL_PWMBFS_1                   (1 << SMOCTRL_PWMBFS_SHIFT)  /* Output is forced to logic 1 state prior to consideration of output polarity control. */
#  define SMOCTRL_PWMBFS_TRISTATE            (2 << SMOCTRL_PWMBFS_SHIFT)  /* Output is tristated. */
#define SMOCTRL_PWMAFS_SHIFT                 (4)        /* Bits: 4-5  PWM_A Fault State */
#define SMOCTRL_PWMAFS_MASK                  (3 << SMOCTRL_PWMAFS_SHIFT)
#  define SMOCTRL_PWMAFS(n)                  ((uint32_t)(n) << SMOCTRL_PWMAFS_SHIFT)
#  define SMOCTRL_PWMAFS_0                   (0 << SMOCTRL_PWMAFS_SHIFT)  /* Output is forced to logic 0 state prior to consideration of output polarity control. */
#  define SMOCTRL_PWMAFS_1                   (1 << SMOCTRL_PWMAFS_SHIFT)  /* Output is forced to logic 1 state prior to consideration of output polarity control. */
#  define SMOCTRL_PWMAFS_TRISTATE            (2 << SMOCTRL_PWMAFS_SHIFT)  /* Output is tristated. */
                                                        /* Bits: 6-7 Reserved */
#define SMOCTRL_POLX                         (1 << 8)   /* Bit: 8  PWM_X Output Polarity */
#define SMOCTRL_POLB                         (1 << 9)   /* Bit: 9  PWM_B Output Polarity */
#define SMOCTRL_POLA                         (1 << 10)  /* Bit: 10 PWM_A Output Polarity */
                                                        /* Bits: 11-12 Reserved */
#define SMOCTRL_PWMX_IN                      (1 << 13)  /* Bit: 13 PWM_X Input */
#define SMOCTRL_PWMB_IN                      (1 << 14)  /* Bit: 14 PWM_B Input */
#define SMOCTRL_PWMA_IN                      (1 << 15)  /* Bit: 15 PWM_A Input */

/* Status Register */

#define SMSTS_CMPF_SHIFT                     (0)        /* Bits: 0-5  Compare Flags */
#define SMSTS_CMPF_MASK                      (0x3f << SMSTS_CMPF_SHIFT)
#  define SMSTS_CMPF(n)                      ((uint32_t)(n) << SMSTS_CMPF_SHIFT)
#  define SMSTS_CMPF_VAL0                    (1 << SMSTS_CMPF_SHIFT)   /* A compare event has occurred for VAL0 value */
#  define SMSTS_CMPF_VAL1                    (2 << SMSTS_CMPF_SHIFT)   /* A compare event has occurred for VAL1 value */
#  define SMSTS_CMPF_VAL2                    (4 << SMSTS_CMPF_SHIFT)   /* A compare event has occurred for VAL2 value */
#  define SMSTS_CMPF_VAL3                    (8 << SMSTS_CMPF_SHIFT)   /* A compare event has occurred for VAL3 value */
#  define SMSTS_CMPF_VAL4                    (16 << SMSTS_CMPF_SHIFT)  /* A compare event has occurred for VAL4 value */
#  define SMSTS_CMPF_VAL5                    (32 << SMSTS_CMPF_SHIFT)  /* A compare event has occurred for VAL5 value */
#define SMSTS_CFX0                           (1 << 6)   /* Bit: 6  Capture Flag X0 */
#define SMSTS_CFX1                           (1 << 7)   /* Bit: 7  Capture Flag X1 */
#define SMSTS_CFB0                           (1 << 8)   /* Bit: 8  Capture Flag B0 */
#define SMSTS_CFB1                           (1 << 9)   /* Bit: 9  Capture Flag B1 */
#define SMSTS_CFA0                           (1 << 10)  /* Bit: 10 Capture Flag A0 */
#define SMSTS_CFA1                           (1 << 11)  /* Bit: 11 Capture Flag A1 */
#define SMSTS_RF                             (1 << 12)  /* Bit: 12 Reload Flag */
#define SMSTS_REF                            (1 << 13)  /* Bit: 13 Reload Error Flag */
#define SMSTS_RUF                            (1 << 14)  /* Bit: 14 This read-only flag is set when one of the INIT, VALx,FRACVALx, or CTRL[PRSC] is written */
                                                        /* Bit: 15 Reserved */

/* Interrupt Enable Register */

#define SMINTEN_CMPIE_SHIFT                  (0)        /* Bits: 0-5  Compare Interrupt Enables */
#define SMINTEN_CMPIE_MASK                   (0x3f << SMINTEN_CMPIE_SHIFT)
#  define SMINTEN_CMPIE(n)                   ((uint32_t)(n) << SMINTEN_CMPIE_SHIFT)
#  define SMINTEN_CMPIE_VAL0                 (1 << SMINTEN_CMPIE_SHIFT)  /* A compare event for VAL0 value will generate an interrupt */
#  define SMINTEN_CMPIE_VAL1                 (2 << SMINTEN_CMPIE_SHIFT)  /* A compare event for VAL1 value will generate an interrupt */
#  define SMINTEN_CMPIE_VAL2                 (4 << SMINTEN_CMPIE_SHIFT)  /* A compare event for VAL2 value will generate an interrupt */
#  define SMINTEN_CMPIE_VAL3                 (8 << SMINTEN_CMPIE_SHIFT)  /* A compare event for VAL3 value will generate an interrupt */
#  define SMINTEN_CMPIE_VAL4                 (16 << SMINTEN_CMPIE_SHIFT)  /* A compare event for VAL4 value will generate an interrupt  */
#  define SMINTEN_CMPIE_VAL5                 (32 << SMINTEN_CMPIE_SHIFT)  /* A compare event for VAL5 value will generate an interrupt  */
#define SMINTEN_CX0IE                        (1 << 6)   /* Bit: 6  Capture X 0 Interrupt Enable */
#define SMINTEN_CX1IE                        (1 << 7)   /* Bit: 7  Capture X 1 Interrupt Enable */
#define SMINTEN_CB0IE                        (1 << 8)   /* Bit: 8  Capture B 0 Interrupt Enable */
#define SMINTEN_CB1IE                        (1 << 9)   /* Bit: 9  Capture B 1 Interrupt Enable */
#define SMINTEN_CA0IE                        (1 << 10)  /* Bit: 10 Capture A 0 Interrupt Enable */
#define SMINTEN_CA1IE                        (1 << 11)  /* Bit: 11 Capture A 1 Interrupt Enable */
#define SMINTEN_RIE                          (1 << 12)  /* Bit: 12 Reload Interrupt Enable */
#define SMINTEN_REIE                         (1 << 13)  /* Bit: 13 Reload Error Interrupt Enable */
                                                        /* Bits: 14-15 Reserved */

/* DMA Enable Register */

#define SMDMAEN_CX0DE                        (1 << 0)   /* Bit: 0  Capture X0 FIFO DMA Enable */
#define SMDMAEN_CX1DE                        (1 << 1)   /* Bit: 1  Capture X1 FIFO DMA Enable */
#define SMDMAEN_CB0DE                        (1 << 2)   /* Bit: 2  Capture B0 FIFO DMA Enable */
#define SMDMAEN_CB1DE                        (1 << 3)   /* Bit: 3  Capture B1 FIFO DMA Enable */
#define SMDMAEN_CA0DE                        (1 << 4)   /* Bit: 4  Capture A0 FIFO DMA Enable */
#define SMDMAEN_CA1DE                        (1 << 5)   /* Bit: 5  Capture A1 FIFO DMA Enable */
#define SMDMAEN_CAPTDE_SHIFT                 (6)        /* Bits: 6-7  Capture DMA Enable Source Select */
#define SMDMAEN_CAPTDE_MASK                  (3 << SMDMAEN_CAPTDE_SHIFT)
#  define SMDMAEN_CAPTDE(n)                  ((uint32_t)(n) << SMDMAEN_CAPTDE_SHIFT)
#  define SMDMAEN_CAPTDE_DIS                 (0 << SMDMAEN_CAPTDE_SHIFT)  /* Read DMA requests disabled. */
#  define SMDMAEN_CAPTDE_WMT                 (1 << SMDMAEN_CAPTDE_SHIFT)  /* Exceeding a FIFO watermark sets the DMA read request. */
#  define SMDMAEN_CAPTDE_SYNC                (2 << SMDMAEN_CAPTDE_SHIFT)  /* A local sync (VAL1 matches counter) sets the read DMA request. */
#  define SMDMAEN_CAPTDE_RELOAD              (3 << SMDMAEN_CAPTDE_SHIFT)  /* A local reload (STS[RF] being set) sets the read DMA request. */
#define SMDMAEN_FAND                         (1 << 8)   /* Bit: 8  FIFO Watermark AND Control */
#define SMDMAEN_VALDE                        (1 << 9)   /* Bit: 9  Value Registers DMA Enable */
                                                        /* Bits: 10-15 Reserved */

/* Output Trigger Control Register */

#define SMT_OUT_TRIG_EN_SHIFT                (0)        /* Bits: 0-5  Output Trigger Enables */
#define SMT_OUT_TRIG_EN_MASK                 (0x3f << SMT_OUT_TRIG_EN_SHIFT)
#  define SMT_OUT_TRIG_EN(n)                 ((uint32_t)(n) << SMT_OUT_TRIG_EN_SHIFT)
#  define SMT_OUT_TRIG_EN_VAL0               (1 << SMT_OUT_TRIG_EN_SHIFT)   /* PWM_OUT_TRIGx will set when the counter value matches the VAL0 value */
#  define SMT_OUT_TRIG_EN_VAL1               (2 << SMT_OUT_TRIG_EN_SHIFT)   /* PWM_OUT_TRIGx will set when the counter value matches the VAL1 value */
#  define SMT_OUT_TRIG_EN_VAL2               (4 << SMT_OUT_TRIG_EN_SHIFT)   /* PWM_OUT_TRIGx will set when the counter value matches the VAL2 value */
#  define SMT_OUT_TRIG_EN_VAL3               (8 << SMT_OUT_TRIG_EN_SHIFT)   /* PWM_OUT_TRIGx will set when the counter value matches the VAL3 value */
#  define SMT_OUT_TRIG_EN_VAL4               (16 << SMT_OUT_TRIG_EN_SHIFT)  /* PWM_OUT_TRIGx will set when the counter value matches the VAL4 value */
#  define SMT_OUT_TRIG_EN_VAL5               (32 << SMT_OUT_TRIG_EN_SHIFT)  /* PWM_OUT_TRIGx will set when the counter value matches the VAL5 value */
                                                        /* Bits: 6-11 Reserved */
#define SMT_TRGFRQ                           (1 << 12)  /* Bit: 12 Trigger frequency */
                                                        /* Bit: 13 Reserved */
#define SMT_PWBOT1                           (1 << 14)  /* Bit: 14 Output Trigger 1 Source Select */
#  define SMT_PWBOT1_OUT_TRIG1               (0 << 14)  /* Route the PWM_OUT_TRIG1 signal to PWM_OUT_TRIG1 port */
#  define SMT_PWBOT1_PWMMB                   (1 << 14)  /* Route the PWMB output to the PWM_OUT_TRIG1 port */
#define SMT_PWBOT0                           (1 << 15)  /* Bit: 15 Output Trigger 0 Source Select */
#  define SMT_PWBOT0_OUT_TRIG0               (0 << 15)  /* Route the PWM_OUT_TRIG0 signal to PWM_OUT_TRIG0 port */
#  define SMT_PWBOT0_PWMMB                   (1 << 15)  /* Route the PWMA output to the PWM_OUT_TRIG0 port */

/* Fault Disable Mapping Register 0 */

#define SMD_DIS0A_SHIFT                      (0)        /* Bits: 0-3  PWM_A Fault Disable Mask 0 */
#define SMD_DIS0A_MASK                       (0xf << SMD_DIS0A_SHIFT)
#  define SMD_DIS0A(n)                       ((uint32_t)(n) << SMD_DIS0A_SHIFT)
#  define SMD_DIS0A_FAULT0                   (1 << SMD_DIS0A_SHIFT)  /* FAULT0 inputs of fault channel 0. */
#  define SMD_DIS0A_FAULT1                   (2 << SMD_DIS0A_SHIFT)  /* FAULT1 inputs of fault channel 0. */
#  define SMD_DIS0A_FAULT2                   (4 << SMD_DIS0A_SHIFT)  /* FAULT2 inputs of fault channel 0. */
#  define SMD_DIS0A_FAULT3                   (8 << SMD_DIS0A_SHIFT)  /* FAULT3 inputs of fault channel 0. */
#define SMD_DIS0B_SHIFT                      (4)        /* Bits: 4-7  PWM_B Fault Disable Mask 0 */
#define SMD_DIS0B_MASK                       (0xf << SMD_DIS0B_SHIFT)
#  define SMD_DIS0B(n)                       ((uint32_t)(n) << SMD_DIS0B_SHIFT)
#  define SMD_DIS0B_FAULT0                   (1 << SMD_DIS0B_SHIFT)  /* FAULT0 inputs of fault channel 0. */
#  define SMD_DIS0B_FAULT1                   (2 << SMD_DIS0B_SHIFT)  /* FAULT1 inputs of fault channel 0. */
#  define SMD_DIS0B_FAULT2                   (4 << SMD_DIS0B_SHIFT)  /* FAULT2 inputs of fault channel 0. */
#  define SMD_DIS0B_FAULT3                   (8 << SMD_DIS0B_SHIFT)  /* FAULT3 inputs of fault channel 0. */
#define SMD_DIS0X_SHIFT                      (8)        /* Bits: 8-11  PWM_X Fault Disable Mask 0 */
#define SMD_DIS0X_MASK                       (0xf << SMD_DIS0X_SHIFT)
#  define SMD_DIS0X(n)                       ((uint32_t)(n) << SMD_DIS0X_SHIFT)
#  define SMD_DIS0X_FAULT0                   (1 << SMD_DIS0X_SHIFT)  /* FAULT0 inputs of fault channel 0. */
#  define SMD_DIS0X_FAULT1                   (2 << SMD_DIS0X_SHIFT)  /* FAULT1 inputs of fault channel 0. */
#  define SMD_DIS0X_FAULT2                   (4 << SMD_DIS0X_SHIFT)  /* FAULT2 inputs of fault channel 0. */
#  define SMD_DIS0X_FAULT3                   (8 << SMD_DIS0X_SHIFT)  /* FAULT3 inputs of fault channel 0. */
                                                        /* Bits: 12-15 Reserved */

/* Fault Disable Mapping Register 1 */

#define SMD_DIS1A_SHIFT                      (0)        /* Bits: 0-3  PWM_A Fault Disable Mask 1 */
#define SMD_DIS1A_MASK                       (0xf << SMD_DIS1A_SHIFT)
#  define SMD_DIS1A(n)                       ((uint32_t)(n) << SMD_DIS1A_SHIFT)
#  define SMD_DIS1A_FAULT0                   (1 << SMD_DIS1A_SHIFT)  /* FAULT0 inputs of fault channel 1. */
#  define SMD_DIS1A_FAULT1                   (2 << SMD_DIS1A_SHIFT)  /* FAULT1 inputs of fault channel 1. */
#  define SMD_DIS1A_FAULT2                   (4 << SMD_DIS1A_SHIFT)  /* FAULT2 inputs of fault channel 1. */
#  define SMD_DIS1A_FAULT3                   (8 << SMD_DIS1A_SHIFT)  /* FAULT3 inputs of fault channel 1. */
#define SMD_DIS1B_SHIFT                      (4)        /* Bits: 4-7  PWM_B Fault Disable Mask 1 */
#define SMD_DIS1B_MASK                       (0xf << SMD_DIS1B_SHIFT)
#  define SMD_DIS1B(n)                       ((uint32_t)(n) << SMD_DIS1B_SHIFT)
#  define SMD_DIS1B_FAULT0                   (1 << SMD_DIS1B_SHIFT)  /* FAULT0 inputs of fault channel 1. */
#  define SMD_DIS1B_FAULT1                   (2 << SMD_DIS1B_SHIFT)  /* FAULT1 inputs of fault channel 1. */
#  define SMD_DIS1B_FAULT2                   (4 << SMD_DIS1B_SHIFT)  /* FAULT2 inputs of fault channel 1. */
#  define SMD_DIS1B_FAULT3                   (8 << SMD_DIS1B_SHIFT)  /* FAULT3 inputs of fault channel 1. */
#define SMD_DIS1X_SHIFT                      (8)        /* Bits: 8-11  PWM_X Fault Disable Mask 1 */
#define SMD_DIS1X_MASK                       (0xf << SMD_DIS1X_SHIFT)
#  define SMD_DIS1X(n)                       ((uint32_t)(n) << SMD_DIS1X_SHIFT)
#  define SMD_DIS1X_FAULT0                   (1 << SMD_DIS1X_SHIFT)  /* FAULT0 inputs of fault channel 1. */
#  define SMD_DIS1X_FAULT1                   (2 << SMD_DIS1X_SHIFT)  /* FAULT1 inputs of fault channel 1. */
#  define SMD_DIS1X_FAULT2                   (4 << SMD_DIS1X_SHIFT)  /* FAULT2 inputs of fault channel 1. */
#  define SMD_DIS1X_FAULT3                   (8 << SMD_DIS1X_SHIFT)  /* FAULT3 inputs of fault channel 1. */
                                                        /* Bits: 12-15 Reserved */

/* Capture Control A Register */

#define SMC_ARMA                             (1 << 0)   /* Bit: 0  Arm A */
#define SMC_ONESHOTA                         (1 << 1)   /* Bit: 1  One Shot Mode A */
#define SMC_EDGA0_SHIFT                      (2)        /* Bits: 2-3  Edge A 0 */
#define SMC_EDGA0_MASK                       (3 << SMC_EDGA0_SHIFT)
#  define SMC_EDGA0(n)                       ((uint32_t)(n) << SMC_EDGA0_SHIFT)
#  define SMC_EDGA0_DIS                      (0 << SMC_EDGA0_SHIFT)  /* Disabled */
#  define SMC_EDGA0_FALLING                  (1 << SMC_EDGA0_SHIFT)  /* Capture falling edges */
#  define SMC_EDGA0_RISING                   (2 << SMC_EDGA0_SHIFT)  /* Capture rising edges */
#  define SMC_EDGA0_BOTH                     (3 << SMC_EDGA0_SHIFT)  /* Capture any edge */
#define SMC_EDGA1_SHIFT                      (4)        /* Bits: 4-5  Edge A 1 */
#define SMC_EDGA1_MASK                       (3 << SMC_EDGA1_SHIFT)
#  define SMC_EDGA1(n)                       ((uint32_t)(n) << SMC_EDGA1_SHIFT)
#  define SMC_EDGA1_DIS                      (0 << SMC_EDGA1_SHIFT)  /* Disabled */
#  define SMC_EDGA1_FALLING                  (1 << SMC_EDGA1_SHIFT)  /* Capture falling edges */
#  define SMC_EDGA1_RISING                   (2 << SMC_EDGA1_SHIFT)  /* Capture rising edges */
#  define SMC_EDGA1_BOTH                     (3 << SMC_EDGA1_SHIFT)  /* Capture any edge */
#define SMC_INP_SELA                         (1 << 6)   /* Bit: 6  Input Select A */
#define SMC_EDGCNTA_EN                       (1 << 7)   /* Bit: 7  Edge Counter A Enable */
#define SMC_CFAWM_SHIFT                      (8)        /* Bits: 8-9  Capture A FIFOs Water Mark */
#define SMC_CFAWM_MASK                       (3 << SMC_CFAWM_SHIFT)
#  define SMC_CFAWM(n)                       ((uint32_t)(n) << SMC_CFAWM_SHIFT)
#  define SMC_CFAWM_1                        (0 << SMC_CFAWM_SHIFT)  /* Water mark level of 1 for capture A FIFOs */
#  define SMC_CFAWM_2                        (1 << SMC_CFAWM_SHIFT)  /* Water mark level of 2 for capture A FIFOs */
#  define SMC_CFAWM_3                        (2 << SMC_CFAWM_SHIFT)  /* Water mark level of 3 for capture A FIFOs */
#  define SMC_CFAWM_4                        (3 << SMC_CFAWM_SHIFT)  /* Water mark level of 4 for capture A FIFOs */
#define SMC_CA0CNT_SHIFT                     (10)       /* Bits: 10-12  Capture A0 FIFO Word Count */
#define SMC_CA0CNT_MASK                      (7 << SMC_CA0CNT_SHIFT)
#  define SMC_CA0CNT(n)                      ((uint32_t)(n) << SMC_CA0CNT_SHIFT)
#  define SMC_CA0CNT_00                      (0 << SMC_CA0CNT_SHIFT)  /* 0 words in the Capture A0 FIFO. */
#  define SMC_CA0CNT_01                      (1 << SMC_CA0CNT_SHIFT)  /* 1 word in the Capture A0 FIFO.  */
#  define SMC_CA0CNT_02                      (2 << SMC_CA0CNT_SHIFT)  /* 2 words in the Capture A0 FIFO. */
#  define SMC_CA0CNT_03                      (3 << SMC_CA0CNT_SHIFT)  /* 3 words in the Capture A0 FIFO. */
#  define SMC_CA0CNT_04                      (4 << SMC_CA0CNT_SHIFT)  /* 4 words in the Capture A0 FIFO. */
#  define SMC_CA0CNT_05                      (5 << SMC_CA0CNT_SHIFT)  /* 5 words in the Capture A0 FIFO. */
#  define SMC_CA0CNT_06                      (6 << SMC_CA0CNT_SHIFT)  /* 6 words in the Capture A0 FIFO. */
#  define SMC_CA0CNT_07                      (7 << SMC_CA0CNT_SHIFT)  /* 7 words in the Capture A0 FIFO. */
#define SMC_CA1CNT_SHIFT                     (13)       /* Bits: 13-15  Capture A1 FIFO Word Count */
#define SMC_CA1CNT_MASK                      (7 << SMC_CA1CNT_SHIFT)
#  define SMC_CA1CNT(n)                      ((uint32_t)(n) << SMC_CA1CNT_SHIFT)
#  define SMC_CA1CNT_00                      (0 << SMC_CA1CNT_SHIFT)  /* 1 word in the Capture A1 FIFO. */
#  define SMC_CA1CNT_01                      (1 << SMC_CA1CNT_SHIFT)  /* 2 words in the Capture A1 FIFO. */
#  define SMC_CA1CNT_02                      (2 << SMC_CA1CNT_SHIFT)  /* 3 words in the Capture A1 FIFO. */
#  define SMC_CA1CNT_03                      (3 << SMC_CA1CNT_SHIFT)  /* 4 words in the Capture A1 FIFO. */
#  define SMC_CA1CNT_04                      (4 << SMC_CA1CNT_SHIFT)  /* 5 words in the Capture A1 FIFO. */
#  define SMC_CA1CNT_05                      (5 << SMC_CA1CNT_SHIFT)  /* 6 words in the Capture A1 FIFO. */
#  define SMC_CA1CNT_06                      (6 << SMC_CA1CNT_SHIFT)  /* 7 words in the Capture A1 FIFO. */
#  define SMC_CA1CNT_07                      (7 << SMC_CA1CNT_SHIFT)  /* 8 words in the Capture A1 FIFO. */

/* Capture Compare A Register */

#define SMC_EDGCMPA_SHIFT                    (0)        /* Bits: 0-7  Edge Compare A */
#define SMC_EDGCMPA_MASK                     (0xff << SMC_EDGCMPA_SHIFT)
#  define SMC_EDGCMPA(n)                     ((uint32_t)(n) << SMC_EDGCMPA_SHIFT)
#define SMC_EDGCNTA_SHIFT                    (8)        /* Bits: 8-15  Edge Counter A */
#define SMC_EDGCNTA_MASK                     (0xff << SMC_EDGCNTA_SHIFT)
#  define SMC_EDGCNTA(n)                     ((uint32_t)(n) << SMC_EDGCNTA_SHIFT)

/* Capture Control B Register */

#define SMC_ARMB                             (1 << 0)   /* Bit: 0  Arm B */
#define SMC_ONESHOTB                         (1 << 1)   /* Bit: 1  One Shot Mode B */
#define SMC_EDGB0_SHIFT                      (2)        /* Bits: 2-3  Edge B 0 */
#define SMC_EDGB0_MASK                       (3 << SMC_EDGB0_SHIFT)
#  define SMC_EDGB0(n)                       ((uint32_t)(n) << SMC_EDGB0_SHIFT)
#  define SMC_EDGB0_DIS                      (0 << SMC_EDGB0_SHIFT)  /* Disabled */
#  define SMC_EDGB0_FALLING                  (1 << SMC_EDGB0_SHIFT)  /* Capture falling edges */
#  define SMC_EDGB0_RISING                   (2 << SMC_EDGB0_SHIFT)  /* Capture rising edges */
#  define SMC_EDGB0_BOTH                     (3 << SMC_EDGB0_SHIFT)  /* Capture any edge */
#define SMC_EDGB1_SHIFT                      (4)        /* Bits: 4-5  Edge B 1 */
#define SMC_EDGB1_MASK                       (3 << SMC_EDGB1_SHIFT)
#  define SMC_EDGB1(n)                       ((uint32_t)(n) << SMC_EDGB1_SHIFT)
#  define SMC_EDGB1_DIS                      (0 << SMC_EDGB1_SHIFT)  /* Disabled */
#  define SMC_EDGB1_FALLING                  (1 << SMC_EDGB1_SHIFT)  /* Capture falling edges */
#  define SMC_EDGB1_RISING                   (2 << SMC_EDGB1_SHIFT)  /* Capture rising edges */
#  define SMC_EDGB1_BOTH                     (3 << SMC_EDGB1_SHIFT)  /* Capture any edge */
#define SMC_INP_SELB                         (1 << 6)   /* Bit: 6  Input Select B */
#define SMC_EDGCNTB_EN                       (1 << 7)   /* Bit: 7  Edge Counter B Enable */
#define SMC_CFBWM_SHIFT                      (8)        /* Bits: 8-9  Capture B FIFOs Water Mark */
#define SMC_CFBWM_MASK                       (3 << SMC_CFBWM_SHIFT)
#  define SMC_CFBWM(n)                       ((uint32_t)(n) << SMC_CFBWM_SHIFT)
#  define SMC_CFBWM_0                        (0 << SMC_CFBWM_SHIFT)  /* Water mark level of 1 for capture B FIFOs */
#  define SMC_CFBWM_1                        (1 << SMC_CFBWM_SHIFT)  /* Water mark level of 2 for capture B FIFOs */
#  define SMC_CFBWM_2                        (2 << SMC_CFBWM_SHIFT)  /* Water mark level of 3 for capture B FIFOs */
#  define SMC_CFBWM_3                        (3 << SMC_CFBWM_SHIFT)  /* Water mark level of 4 for capture B FIFOs */
#define SMC_CB0CNT_SHIFT                     (10)       /* Bits: 10-12  Capture B0 FIFO Word Count */
#define SMC_CB0CNT_MASK                      (7 << SMC_CB0CNT_SHIFT)
#  define SMC_CB0CNT(n)                      ((uint32_t)(n) << SMC_CB0CNT_SHIFT)
#  define SMC_CB0CNT_00                      (0 << SMC_CB0CNT_SHIFT)  /* 0 words in the Capture B0 FIFO. */
#  define SMC_CB0CNT_01                      (1 << SMC_CB0CNT_SHIFT)  /* 1 words in the Capture B0 FIFO. */
#  define SMC_CB0CNT_02                      (2 << SMC_CB0CNT_SHIFT)  /* 2 words in the Capture B0 FIFO. */
#  define SMC_CB0CNT_03                      (3 << SMC_CB0CNT_SHIFT)  /* 3 words in the Capture B0 FIFO. */
#  define SMC_CB0CNT_04                      (4 << SMC_CB0CNT_SHIFT)  /* 4 words in the Capture B0 FIFO. */
#  define SMC_CB0CNT_05                      (5 << SMC_CB0CNT_SHIFT)  /* 5 words in the Capture B0 FIFO. */
#  define SMC_CB0CNT_06                      (6 << SMC_CB0CNT_SHIFT)  /* 6 words in the Capture B0 FIFO. */
#  define SMC_CB0CNT_07                      (7 << SMC_CB0CNT_SHIFT)  /* 7 words in the Capture B0 FIFO. */
#define SMC_CB1CNT_SHIFT                     (13)       /* Bits: 13-15  Capture B1 FIFO Word Count */
#define SMC_CB1CNT_MASK                      (7 << SMC_CB1CNT_SHIFT)
#  define SMC_CB1CNT(n)                      ((uint32_t)(n) << SMC_CB1CNT_SHIFT)
#  define SMC_CB1CNT_00                      (0 << SMC_CB1CNT_SHIFT)  /* 0 words in the Capture B1 FIFO. */
#  define SMC_CB1CNT_01                      (1 << SMC_CB1CNT_SHIFT)  /* 1 words in the Capture B1 FIFO. */
#  define SMC_CB1CNT_02                      (2 << SMC_CB1CNT_SHIFT)  /* 2 words in the Capture B1 FIFO. */
#  define SMC_CB1CNT_03                      (3 << SMC_CB1CNT_SHIFT)  /* 3 words in the Capture B1 FIFO. */
#  define SMC_CB1CNT_04                      (4 << SMC_CB1CNT_SHIFT)  /* 4 words in the Capture B1 FIFO. */
#  define SMC_CB1CNT_05                      (5 << SMC_CB1CNT_SHIFT)  /* 5 words in the Capture B1 FIFO. */
#  define SMC_CB1CNT_06                      (6 << SMC_CB1CNT_SHIFT)  /* 6 words in the Capture B1 FIFO. */
#  define SMC_CB1CNT_07                      (7 << SMC_CB1CNT_SHIFT)  /* 7 words in the Capture B1 FIFO. */

/* Capture Compare B Register */

#define SMC_EDGCMPB_SHIFT                    (0)        /* Bits: 0-7  Edge Compare B */
#define SMC_EDGCMPB_MASK                     (0xff << SMC_EDGCMPB_SHIFT)
#  define SMC_EDGCMPB(n)                     ((uint32_t)(n) << SMC_EDGCMPB_SHIFT)
#define SMC_EDGCNTB_SHIFT                    (8)        /* Bits: 8-15  Edge Counter B */
#define SMC_EDGCNTB_MASK                     (0xff << SMC_EDGCNTB_SHIFT)
#  define SMC_EDGCNTB(n)                     ((uint32_t)(n) << SMC_EDGCNTB_SHIFT)

/* Capture Control X Register */

#define SMC_ARMX                             (1 << 0)   /* Bit: 0  Arm X */
#define SMC_ONESHOTX                         (1 << 1)   /* Bit: 1  One Shot Mode Aux */
#define SMC_EDGX0_SHIFT                      (2)        /* Bits: 2-3  Edge X 0 */
#define SMC_EDGX0_MASK                       (3 << SMC_EDGX0_SHIFT)
#  define SMC_EDGX0(n)                       ((uint32_t)(n) << SMC_EDGX0_SHIFT)
#  define SMC_EDGX0_DIS                      (0 << SMC_EDGX0_SHIFT)  /* Disabled */
#  define SMC_EDGX0_FALLING                  (1 << SMC_EDGX0_SHIFT)  /* Capture falling edges */
#  define SMC_EDGX0_RISING                   (2 << SMC_EDGX0_SHIFT)  /* Capture rising edges */
#  define SMC_EDGX0_BOTH                     (3 << SMC_EDGX0_SHIFT)  /* Capture any edge */
#define SMC_EDGX1_SHIFT                      (4)        /* Bits: 4-5  Edge X 1 */
#define SMC_EDGX1_MASK                       (3 << SMC_EDGX1_SHIFT)
#  define SMC_EDGX1(n)                       ((uint32_t)(n) << SMC_EDGX1_SHIFT)
#  define SMC_EDGX1_DIS                      (0 << SMC_EDGX1_SHIFT)  /* Disabled */
#  define SMC_EDGX1_FALLING                  (1 << SMC_EDGX1_SHIFT)  /* Capture falling edges */
#  define SMC_EDGX1_RISING                   (2 << SMC_EDGX1_SHIFT)  /* Capture rising edges */
#  define SMC_EDGX1_BOTH                     (3 << SMC_EDGX1_SHIFT)  /* Capture any edge */
#define SMC_INP_SELX                         (1 << 6)   /* Bit: 6  Input Select X */
#define SMC_EDGCNTX_EN                       (1 << 7)   /* Bit: 7  Edge Counter X Enable */
#define SMC_CFXWM_SHIFT                      (8)        /* Bits: 8-9  Capture X FIFOs Water Mark */
#define SMC_CFXWM_MASK                       (3 << SMC_CFXWM_SHIFT)
#  define SMC_CFXWM(n)                       ((uint32_t)(n) << SMC_CFXWM_SHIFT)
#  define SMC_CFXWM_1                        (0 << SMC_CFXWM_SHIFT)  /* Water mark level of 1 for capture X FIFOs */
#  define SMC_CFXWM_2                        (1 << SMC_CFXWM_SHIFT)  /* Water mark level of 2 for capture X FIFOs */
#  define SMC_CFXWM_3                        (2 << SMC_CFXWM_SHIFT)  /* Water mark level of 3 for capture X FIFOs */
#  define SMC_CFXWM_4                        (3 << SMC_CFXWM_SHIFT)  /* Water mark level of 4 for capture X FIFOs */
#define SMC_CX0CNT_SHIFT                     (10)       /* Bits: 10-12  Capture X0 FIFO Word Count */
#define SMC_CX0CNT_MASK                      (7 << SMC_CX0CNT_SHIFT)
#  define SMC_CX0CNT(n)                      ((uint32_t)(n) << SMC_CX0CNT_SHIFT)
#  define SMC_CX0CNT_00                      (0 << SMC_CX0CNT_SHIFT)  /* 0 words in the Capture X0 FIFO. */
#  define SMC_CX0CNT_01                      (1 << SMC_CX0CNT_SHIFT)  /* 1 word in the Capture X0 FIFO.  */
#  define SMC_CX0CNT_02                      (2 << SMC_CX0CNT_SHIFT)  /* 2 words in the Capture X0 FIFO. */
#  define SMC_CX0CNT_03                      (3 << SMC_CX0CNT_SHIFT)  /* 3 words in the Capture X0 FIFO. */
#  define SMC_CX0CNT_04                      (4 << SMC_CX0CNT_SHIFT)  /* 4 words in the Capture X0 FIFO. */
#  define SMC_CX0CNT_05                      (5 << SMC_CX0CNT_SHIFT)  /* 5 words in the Capture X0 FIFO. */
#  define SMC_CX0CNT_06                      (6 << SMC_CX0CNT_SHIFT)  /* 6 words in the Capture X0 FIFO. */
#  define SMC_CX0CNT_07                      (7 << SMC_CX0CNT_SHIFT)  /* 7 words in the Capture X0 FIFO. */
#define SMC_CX1CNT_SHIFT                     (13)       /* Bits: 13-15  Capture X1 FIFO Word Count */
#define SMC_CX1CNT_MASK                      (7 << SMC_CX1CNT_SHIFT)
#  define SMC_CX1CNT(n)                      ((uint32_t)(n) << SMC_CX1CNT_SHIFT)
#  define SMC_CX1CNT_00                      (0 << SMC_CX1CNT_SHIFT)  /* 0 words in the Capture X1 FIFO. */
#  define SMC_CX1CNT_01                      (1 << SMC_CX1CNT_SHIFT)  /* 1 word in the Capture  X1 FIFO.  */
#  define SMC_CX1CNT_02                      (2 << SMC_CX1CNT_SHIFT)  /* 2 words in the Capture X1 FIFO. */
#  define SMC_CX1CNT_03                      (3 << SMC_CX1CNT_SHIFT)  /* 3 words in the Capture X1 FIFO. */
#  define SMC_CX1CNT_04                      (4 << SMC_CX1CNT_SHIFT)  /* 4 words in the Capture X1 FIFO. */
#  define SMC_CX1CNT_05                      (5 << SMC_CX1CNT_SHIFT)  /* 5 words in the Capture X1 FIFO. */
#  define SMC_CX1CNT_06                      (6 << SMC_CX1CNT_SHIFT)  /* 6 words in the Capture X1 FIFO. */
#  define SMC_CX1CNT_07                      (7 << SMC_CX1CNT_SHIFT)  /* 7 words in the Capture X1 FIFO. */

/* Capture Compare X Register */

#define SMC_EDGCMPX_SHIFT                    (0)        /* Bits: 0-7  Edge Compare X */
#define SMC_EDGCMPX_MASK                     (0xff << SMC_EDGCMPX_SHIFT)
#  define SMC_EDGCMPX(n)                     ((uint32_t)(n) << SMC_EDGCMPX_SHIFT)
#define SMC_EDGCNTX_SHIFT                    (8)        /* Bits: 8-15  Edge Counter X */
#define SMC_EDGCNTX_MASK                     (0xff << SMC_EDGCNTX_SHIFT)
#  define SMC_EDGCNTX(n)                     ((uint32_t)(n) << SMC_EDGCNTX_SHIFT)

/* Capture Value 0 Cycle Register */

#define SMC_CVAL0CYC_SHIFT                   (0)        /* Bits: 0-3  CVAL0CYC */
#define SMC_CVAL0CYC_MASK                    (0xf << SMC_CVAL0CYC_SHIFT)
                                                        /* Bits: 4-15 Reserved */

/* Capture Value 1 Cycle Register */

#define SMC_CVAL1CYC_SHIFT                   (0)        /* Bits: 0-3  CVAL1CYC */
#define SMC_CVAL1CYC_MASK                    (0xf << SMC_CVAL1CYC_SHIFT)
                                                        /* Bits: 4-15 Reserved */

/* Capture Value 2 Cycle Register */

#define SMC_CVAL2CYC_SHIFT                   (0)        /* Bits: 0-3  CVAL2CYC */
#define SMC_CVAL2CYC_MASK                    (0xf << SMC_CVAL2CYC_SHIFT)
                                                        /* Bits: 4-15 Reserved */

/* Capture Value 3 Cycle Register */

#define SMC_CVAL3CYC_SHIFT                   (0)        /* Bits: 0-3  CVAL3CYC */
#define SMC_CVAL3CYC_MASK                    (0xf << SMC_CVAL3CYC_SHIFT)
                                                        /* Bits: 4-15 Reserved */

/* Capture Value 4 Cycle Register */

#define SMC_CVAL4CYC_SHIFT                   (0)        /* Bits: 0-3  CVAL4CYC */
#define SMC_CVAL4CYC_MASK                    (0xf << SMC_CVAL4CYC_SHIFT)
                                                        /* Bits: 4-15 Reserved */

/* Capture Value 5 Cycle Register */

#define SMC_CVAL5CYC_SHIFT                   (0)        /* Bits: 0-3  CVAL5CYC */
#define SMC_CVAL5CYC_MASK                    (0xf << SMC_CVAL5CYC_SHIFT)
                                                        /* Bits: 4-15 Reserved */

/* Output Enable Register */

#define OUTEN_PWMX_EN_SHIFT                  (0)        /* Bits: 0-3  PWM_X Output Enables */
#define OUTEN_PWMX_EN_MASK                   (0xf << OUTEN_PWMX_EN_SHIFT)
#  define OUTEN_PWMX_EN(n)                   ((uint32_t)(n) << OUTEN_PWMX_EN_SHIFT)
#  define OUTEN_PWMX_EN_ALL_DIS              (0 << OUTEN_PWMX_EN_SHIFT)  /* All disabled */
#  define OUTEN_PWMX_EN_SM0                  (1 << OUTEN_PWMX_EN_SHIFT)  /* Enable the PWM_X outputs of submodules 0 */
#  define OUTEN_PWMX_EN_SM1                  (2 << OUTEN_PWMX_EN_SHIFT)  /* Enable the PWM_X outputs of submodules 1 */
#  define OUTEN_PWMX_EN_SM2                  (4 << OUTEN_PWMX_EN_SHIFT)  /* Enable the PWM_X outputs of submodules 2 */
#  define OUTEN_PWMX_EN_SM3                  (8 << OUTEN_PWMX_EN_SHIFT)  /* Enable the PWM_X outputs of submodules 3 */
#  define OUTEN_PWMX_EN_ALL                  (0xf << OUTEN_PWMX_EN_SHIFT)  /* All enabled */
#define OUTEN_PWMB_EN_SHIFT                  (4)        /* Bits: 4-7  PWM_B Output Enables */
#define OUTEN_PWMB_EN_MASK                   (0xf << OUTEN_PWMB_EN_SHIFT)
#  define OUTEN_PWMB_EN(n)                   ((uint32_t)(n) << OUTEN_PWMB_EN_SHIFT)
#  define OUTEN_PWMB_EN_ALL_DIS              (0 << OUTEN_PWMB_EN_SHIFT)  /* All disabled */
#  define OUTEN_PWMB_EN_SM0                  (1 << OUTEN_PWMB_EN_SHIFT)  /* Enable the PWM_B outputs of submodules 0 */
#  define OUTEN_PWMB_EN_SM1                  (2 << OUTEN_PWMB_EN_SHIFT)  /* Enable the PWM_B outputs of submodules 1 */
#  define OUTEN_PWMB_EN_SM2                  (4 << OUTEN_PWMB_EN_SHIFT)  /* Enable the PWM_B outputs of submodules 2 */
#  define OUTEN_PWMB_EN_SM3                  (8 << OUTEN_PWMB_EN_SHIFT)  /* Enable the PWM_B outputs of submodules 3 */
#  define OUTEN_PWMB_EN_ALL                  (0xf << OUTEN_PWMB_EN_SHIFT)  /* All enabled */
#define OUTEN_PWMA_EN_SHIFT                  (8)        /* Bits: 8-11  PWM_A Output Enables */
#define OUTEN_PWMA_EN_MASK                   (0xf << OUTEN_PWMA_EN_SHIFT)
#  define OUTEN_PWMA_EN(n)                   ((uint32_t)(n) << OUTEN_PWMA_EN_SHIFT)
#  define OUTEN_PWMA_EN_ALL_DIS              (0 << OUTEN_PWMA_EN_SHIFT)  /* All disabled */
#  define OUTEN_PWMA_EN_SM0                  (1 << OUTEN_PWMA_EN_SHIFT)  /* Enable the PWM_A outputs of submodules 0 */
#  define OUTEN_PWMA_EN_SM1                  (2 << OUTEN_PWMA_EN_SHIFT)  /* Enable the PWM_A outputs of submodules 1 */
#  define OUTEN_PWMA_EN_SM2                  (4 << OUTEN_PWMA_EN_SHIFT)  /* Enable the PWM_A outputs of submodules 2 */
#  define OUTEN_PWMA_EN_SM3                  (8 << OUTEN_PWMA_EN_SHIFT)  /* Enable the PWM_A outputs of submodules 3 */
#  define OUTEN_PWMA_EN_ALL                  (0xf << OUTEN_PWMA_EN_SHIFT)  /* All enabled */
                                                        /* Bits: 12-15 Reserved */

/* Mask Register */

#define MASK_MASKX_SHIFT                     (0)        /* Bits: 0-3  PWM_X Masks */
#define MASK_MASKX_MASK                      (0xf << MASK_MASKX_SHIFT)
#  define MASK_MASKX(n)                      ((uint32_t)(n) << MASK_MASKX_SHIFT)
#  define MASK_MASKX_SM0                     (1 << MASK_MASKX_SHIFT)  /* Mask the PWM_X outputs of submodules 0 (forces output to 0) */
#  define MASK_MASKX_SM1                     (2 << MASK_MASKX_SHIFT)  /* Mask the PWM_X outputs of submodules 1 (forces output to 0) */
#  define MASK_MASKX_SM2                     (4 << MASK_MASKX_SHIFT)  /* Mask the PWM_X outputs of submodules 2 (forces output to 0) */
#  define MASK_MASKX_SM3                     (8 << MASK_MASKX_SHIFT)  /* Mask the PWM_X outputs of submodules 3 (forces output to 0) */
#define MASK_MASKB_SHIFT                     (4)        /* Bits: 4-7  PWM_B Masks */
#define MASK_MASKB_MASK                      (0xf << MASK_MASKB_SHIFT)
#  define MASK_MASKB(n)                      ((uint32_t)(n) << MASK_MASKB_SHIFT)
#  define MASK_MASKB_SM0                     (1 << MASK_MASKB_SHIFT)  /* Mask the PWM_B outputs of submodules 0 (forces output to 0) */
#  define MASK_MASKB_SM1                     (2 << MASK_MASKB_SHIFT)  /* Mask the PWM_B outputs of submodules 1 (forces output to 0) */
#  define MASK_MASKB_SM2                     (4 << MASK_MASKB_SHIFT)  /* Mask the PWM_B outputs of submodules 2 (forces output to 0) */
#  define MASK_MASKB_SM3                     (8 << MASK_MASKB_SHIFT)  /* Mask the PWM_B outputs of submodules 3 (forces output to 0) */
#define MASK_MASKA_SHIFT                     (8)        /* Bits: 8-11  PWM_A Masks */
#define MASK_MASKA_MASK                      (0xf << MASK_MASKB_SHIFT)
#  define MASK_MASKA(n)                      ((uint32_t)(n) << MASK_MASKA_SHIFT)
#  define MASK_MASKA_SM0                     (1 << MASK_MASKA_SHIFT)  /* Mask the PWM_A outputs of submodules 0 (forces output to 0) */
#  define MASK_MASKA_SM1                     (2 << MASK_MASKA_SHIFT)  /* Mask the PWM_A outputs of submodules 1 (forces output to 0) */
#  define MASK_MASKA_SM2                     (4 << MASK_MASKA_SHIFT)  /* Mask the PWM_A outputs of submodules 2 (forces output to 0) */
#  define MASK_MASKA_SM3                     (8 << MASK_MASKA_SHIFT)  /* Mask the PWM_A outputs of submodules 3 (forces output to 0) */
#define MASK_UPDATE_MASK_SHIFT               (12)       /* Bits: 12-15  Update Mask Bits Immediately */
#define MASK_UPDATE_MASK_MASK                (0xf << MASK_UPDATE_MASK_SHIFT)
#  define MASK_UPDATE_MASK(n)                ((uint32_t)(n) << MASK_UPDATE_MASK_SHIFT)
#  define MASK_UPDATE_MASK_SM0               (1 << MASK_UPDATE_MASK_SHIFT)  /* Immediate update operation on PWM_X output of submodules 0 */
#  define MASK_UPDATE_MASK_SM1               (2 << MASK_UPDATE_MASK_SHIFT)  /* Immediate update operation on PWM_X output of submodules 1 */
#  define MASK_UPDATE_MASK_SM2               (4 << MASK_UPDATE_MASK_SHIFT)  /* Immediate update operation on PWM_X output of submodules 2 */
#  define MASK_UPDATE_MASK_SM3               (8 << MASK_UPDATE_MASK_SHIFT)  /* Immediate update operation on PWM_X output of submodules 3 */

/* Software Controlled Output Register */

#define SWCOUT_SM0OUT45                      (1 << 0)   /* Bit: 0  Submodule 0 Software Controlled Output 45 */
#define SWCOUT_SM0OUT23                      (1 << 1)   /* Bit: 1  Submodule 0 Software Controlled Output 23 */
#define SWCOUT_SM1OUT45                      (1 << 2)   /* Bit: 2  Submodule 1 Software Controlled Output 45 */
#define SWCOUT_SM1OUT23                      (1 << 3)   /* Bit: 3  Submodule 1 Software Controlled Output 23 */
#define SWCOUT_SM2OUT45                      (1 << 4)   /* Bit: 4  Submodule 2 Software Controlled Output 45 */
#define SWCOUT_SM2OUT23                      (1 << 5)   /* Bit: 5  Submodule 2 Software Controlled Output 23 */
#define SWCOUT_SM3OUT45                      (1 << 6)   /* Bit: 6  Submodule 3 Software Controlled Output 45 */
#define SWCOUT_SM3OUT23                      (1 << 7)   /* Bit: 7  Submodule 3 Software Controlled Output 23 */
                                                        /* Bits: 8-15 Reserved */

/* PWM Source Select Register */

/* Register Bit Definitions *********************************************************/

#define DTSRCSEL_SM0SEL45_SHIFT              (0)        /* Bits: 0-1  Submodule 0 PWM45 Control Select */
#define DTSRCSEL_SM0SEL45_MASK               (3 << DTSRCSEL_SM0SEL45_SHIFT)
#  define DTSRCSEL_SM0SEL45(n)               ((uint32_t)(n) << DTSRCSEL_SM0SEL45_SHIFT)
#  define DTSRCSEL_SM0SEL45_NORM             (0 << DTSRCSEL_SM0SEL45_SHIFT)  /* Generated SM0PWM45 signal is used by the deadtime logic. */
#  define DTSRCSEL_SM0SEL45_INVERT           (1 << DTSRCSEL_SM0SEL45_SHIFT)  /* Inverted generated SM0PWM45 signal is used by the deadtime logic. */
#  define DTSRCSEL_SM0SEL45_SWCOUT           (2 << DTSRCSEL_SM0SEL45_SHIFT)  /* SWCOUT[SM0OUT45] is used by the deadtime logic. */
#  define DTSRCSEL_SM0SEL45_PWM0_EXTB        (3 << DTSRCSEL_SM0SEL45_SHIFT)  /* PWM0_EXTB signal is used by the deadtime logic. */
#define DTSRCSEL_SM0SEL23_SHIFT              (2)        /* Bits: 2-3  Submodule 0 PWM23 Control Select */
#define DTSRCSEL_SM0SEL23_MASK               (3 << DTSRCSEL_SM0SEL23_SHIFT)
#  define DTSRCSEL_SM0SEL23(n)               ((uint32_t)(n) << DTSRCSEL_SM0SEL23_SHIFT)
#  define DTSRCSEL_SM0SEL23_NORM             (0 << DTSRCSEL_SM0SEL23_SHIFT)  /* Generated SM0PWM23 signal is used by the deadtime logic. */
#  define DTSRCSEL_SM0SEL23_INVERT           (1 << DTSRCSEL_SM0SEL23_SHIFT)  /* Inverted generated SM0PWM23 signal is used by the deadtime logic. */
#  define DTSRCSEL_SM0SEL23_SWCOUT           (2 << DTSRCSEL_SM0SEL23_SHIFT)  /* SWCOUT[SM0OUT23] is used by the deadtime logic. */
#  define DTSRCSEL_SM0SEL23_PWM0_EXTA        (3 << DTSRCSEL_SM0SEL23_SHIFT)  /* PWM0_EXTA signal is used by the deadtime logic. */
#define DTSRCSEL_SM1SEL45_SHIFT              (4)        /* Bits: 4-5  Submodule 1 PWM45 Control Select */
#define DTSRCSEL_SM1SEL45_MASK               (3 << DTSRCSEL_SM1SEL45_SHIFT)
#  define DTSRCSEL_SM1SEL45(n)               ((uint32_t)(n) << DTSRCSEL_SM1SEL45_SHIFT)
#  define DTSRCSEL_SM1SEL45_NORM             (0 << DTSRCSEL_SM1SEL45_SHIFT)  /* Generated SM1PWM45 signal is used by the deadtime logic. */
#  define DTSRCSEL_SM1SEL45_INVERT           (1 << DTSRCSEL_SM1SEL45_SHIFT)  /* Inverted generated SM1PWM45 signal is used by the deadtime logic. */
#  define DTSRCSEL_SM1SEL45_SWCOUT           (2 << DTSRCSEL_SM1SEL45_SHIFT)  /* SWCOUT[SM1OUT45] is used by the deadtime logic. */
#  define DTSRCSEL_SM1SEL45_PWM1_EXTB        (3 << DTSRCSEL_SM1SEL45_SHIFT)  /* PWM1_EXTB signal is used by the deadtime logic. */
#define DTSRCSEL_SM1SEL23_SHIFT              (6)        /* Bits: 6-7  Submodule 1 PWM23 Control Select */
#define DTSRCSEL_SM1SEL23_MASK               (3 << DTSRCSEL_SM1SEL23_SHIFT)
#  define DTSRCSEL_SM1SEL23(n)               ((uint32_t)(n) << DTSRCSEL_SM1SEL23_SHIFT)
#  define DTSRCSEL_SM1SEL23_NORM             (0 << DTSRCSEL_SM1SEL23_SHIFT)  /* Generated SM1PWM23 signal is used by the deadtime logic. */
#  define DTSRCSEL_SM1SEL23_INVERT           (1 << DTSRCSEL_SM1SEL23_SHIFT)  /* Inverted generated SM1PWM23 signal is used by the deadtime logic. */
#  define DTSRCSEL_SM1SEL23_SWCOUT           (2 << DTSRCSEL_SM1SEL23_SHIFT)  /* SWCOUT[SM1OUT23] is used by the deadtime logic. */
#  define DTSRCSEL_SM1SEL23_PWM1_EXTA        (3 << DTSRCSEL_SM1SEL23_SHIFT)  /* PWM1_EXTA signal is used by the deadtime logic */
#define DTSRCSEL_SM2SEL45_SHIFT              (8)        /* Bits: 8-9  Submodule 2 PWM45 Control Select */
#define DTSRCSEL_SM2SEL45_MASK               (3 << DTSRCSEL_SM2SEL45_SHIFT)
#  define DTSRCSEL_SM2SEL45(n)               ((uint32_t)(n) << DTSRCSEL_SM2SEL45_SHIFT)
#  define DTSRCSEL_SM2SEL45_NORM             (0 << DTSRCSEL_SM2SEL45_SHIFT)  /* Generated SM2PWM45 signal is used by the deadtime logic. */
#  define DTSRCSEL_SM2SEL45_INVERT           (1 << DTSRCSEL_SM2SEL45_SHIFT)  /* Inverted generated SM2PWM45 signal is used by the deadtime logic. */
#  define DTSRCSEL_SM2SEL45_SWCOUT           (2 << DTSRCSEL_SM2SEL45_SHIFT)  /* SWCOUT[SM2OUT45] is used by the deadtime logic. */
#  define DTSRCSEL_SM2SEL45_PWM2_EXTB        (3 << DTSRCSEL_SM2SEL45_SHIFT)  /* PWM2_EXTB signal is used by the deadtime logic. */
#define DTSRCSEL_SM2SEL23_SHIFT              (10)       /* Bits: 10-11  Submodule 2 PWM23 Control Select */
#define DTSRCSEL_SM2SEL23_MASK               (3 << DTSRCSEL_SM2SEL23_SHIFT)
#  define DTSRCSEL_SM2SEL23(n)               ((uint32_t)(n) << DTSRCSEL_SM2SEL23_SHIFT)
#  define DTSRCSEL_SM2SEL23_NORM             (0 << DTSRCSEL_SM2SEL23_SHIFT)  /* Generated SM2PWM23 signal is used by the deadtime logic. */
#  define DTSRCSEL_SM2SEL23_INVERT           (1 << DTSRCSEL_SM2SEL23_SHIFT)  /* Inverted generated SM2PWM23 signal is used by the deadtime logic. */
#  define DTSRCSEL_SM2SEL23_SWCOUT           (2 << DTSRCSEL_SM2SEL23_SHIFT)  /* SWCOUT[SM2OUT23] is used by the deadtime logic. */
#  define DTSRCSEL_SM2SEL23_PWM2_EXTA        (3 << DTSRCSEL_SM2SEL23_SHIFT)  /* PWM2_EXTA signal is used by the deadtime logic */
#define DTSRCSEL_SM3SEL45_SHIFT              (12)       /* Bits: 12-13  Submodule 3 PWM45 Control Select */
#define DTSRCSEL_SM3SEL45_MASK               (3 << DTSRCSEL_SM3SEL45_SHIFT)
#  define DTSRCSEL_SM3SEL45(n)               ((uint32_t)(n) << DTSRCSEL_SM3SEL45_SHIFT)
#  define DTSRCSEL_SM3SEL45_NORM             (0 << DTSRCSEL_SM3SEL45_SHIFT)  /* Generated SM3PWM45 signal is used by the deadtime logic. */
#  define DTSRCSEL_SM3SEL45_INVERT           (1 << DTSRCSEL_SM3SEL45_SHIFT)  /* Inverted generated SM3PWM45 signal is used by the deadtime logic. */
#  define DTSRCSEL_SM3SEL45_SWCOUT           (2 << DTSRCSEL_SM3SEL45_SHIFT)  /* SWCOUT[SM3OUT45] is used by the deadtime logic. */
#  define DTSRCSEL_SM3SEL45_PWM3_EXTB        (3 << DTSRCSEL_SM3SEL45_SHIFT)  /* PWM3_EXTB signal is used by the deadtime logic. */
#define DTSRCSEL_SM3SEL23_SHIFT              (14)       /* Bits: 14-15  Submodule 3 PWM23 Control Select */
#define DTSRCSEL_SM3SEL23_MASK               (3 << DTSRCSEL_SM3SEL23_SHIFT)
#  define DTSRCSEL_SM3SEL23(n)               ((uint32_t)(n) << DTSRCSEL_SM3SEL23_SHIFT)
#  define DTSRCSEL_SM3SEL23_NORM             (0 << DTSRCSEL_SM3SEL23_SHIFT)  /* Generated SM3PWM23 signal is used by the deadtime logic. */
#  define DTSRCSEL_SM3SEL23_INVERT           (1 << DTSRCSEL_SM3SEL23_SHIFT)  /* Inverted generated SM3PWM23 signal is used by the deadtime logic. */
#  define DTSRCSEL_SM3SEL23_SWCOUT           (2 << DTSRCSEL_SM3SEL23_SHIFT)  /* SWCOUT[SM3OUT23] is used by the deadtime logic. */
#  define DTSRCSEL_SM3SEL23_PWM3_EXTA        (3 << DTSRCSEL_SM3SEL23_SHIFT)  /* PWM3_EXTA signal is used by the deadtime logic. */

/* Master Control Register */

#define MCTRL_LDOK_SHIFT                     (0)        /* Bits: 0-3  Load Okay */
#define MCTRL_LDOK_MASK                      (15 << MCTRL_LDOK_SHIFT)
#  define MCTRL_LDOK(n)                      ((uint32_t)(n) << MCTRL_LDOK_SHIFT)
#  define MCTRL_LDOK_SM0                     (1 << MCTRL_LDOK_SHIFT)  /* Load prescaler, modulus, and PWM values of submodule 0 */
#  define MCTRL_LDOK_SM1                     (2 << MCTRL_LDOK_SHIFT)  /* Load prescaler, modulus, and PWM values of submodule 1 */
#  define MCTRL_LDOK_SM2                     (4 << MCTRL_LDOK_SHIFT)  /* Load prescaler, modulus, and PWM values of submodule 2 */
#  define MCTRL_LDOK_SM3                     (8 << MCTRL_LDOK_SHIFT)  /* Load prescaler, modulus, and PWM values of submodule 3 */
#define MCTRL_CLDOK_SHIFT                    (4)        /* Bits: 4-7  Clear Load Okay */
#define MCTRL_CLDOK_MASK                     (15 << MCTRL_CLDOK_SHIFT)
#  define MCTRL_CLDOK(n)                     ((uint32_t)(n) << MCTRL_CLDOK_SHIFT)
#  define MCTRL_CLDOK_SM0                    (1 << MCTRL_CLDOK_SHIFT)  /* Clear Load Okay of submodule 0 */
#  define MCTRL_CLDOK_SM1                    (2 << MCTRL_CLDOK_SHIFT)  /* Clear Load Okay of submodule 1 */
#  define MCTRL_CLDOK_SM2                    (4 << MCTRL_CLDOK_SHIFT)  /* Clear Load Okay of submodule 2 */
#  define MCTRL_CLDOK_SM3                    (8 << MCTRL_CLDOK_SHIFT)  /* Clear Load Okay of submodule 3 */
#define MCTRL_RUN_SHIFT                      (8)        /* Bits: 8-11  Run */
#define MCTRL_RUN_MASK                       (15 << MCTRL_RUN_SHIFT)
#  define MCTRL_RUN(n)                       ((uint32_t)(n) << MCTRL_RUN_SHIFT)
#  define MCTRL_RUN_SM0                      (1 << MCTRL_RUN_SHIFT)  /* Enable PWM generator of submodules 0 */
#  define MCTRL_RUN_SM1                      (2 << MCTRL_RUN_SHIFT)  /* Enable PWM generator of submodules 1 */
#  define MCTRL_RUN_SM2                      (4 << MCTRL_RUN_SHIFT)  /* Enable PWM generator of submodules 2 */
#  define MCTRL_RUN_SM3                      (8 << MCTRL_RUN_SHIFT)  /* Enable PWM generator of submodules 3 */
#define MCTRL_IPOL_SHIFT                     (12)       /* Bits: 12-15  Current Polarity */
#define MCTRL_IPOL_MASK                      (15 << MCTRL_IPOL_SHIFT)
#  define MCTRL_IPOL(n)                      ((uint32_t)(n) << MCTRL_IPOL_SHIFT)
#  define MCTRL_IPOL_SM0                     (1 << MCTRL_IPOL_SHIFT)  /* PWM45 is used to generate complementary PWM pair in submodule 0. */
#  define MCTRL_IPOL_SM1                     (2 << MCTRL_IPOL_SHIFT)  /* PWM45 is used to generate complementary PWM pair in submodule 1. */
#  define MCTRL_IPOL_SM2                     (4 << MCTRL_IPOL_SHIFT)  /* PWM45 is used to generate complementary PWM pair in submodule 2. */
#  define MCTRL_IPOL_SM3                     (8 << MCTRL_IPOL_SHIFT)  /* PWM45 is used to generate complementary PWM pair in submodule 3. */

/* Master Control 2 Register */

/* Register Bit Definitions *********************************************************/

#define MCTRL2_MONPLL_SHIFT                  (0)        /* Bits: 0-1  Monitor PLL State */
#define MCTRL2_MONPLL_MASK                   (3 << MCTRL2_MONPLL_SHIFT)
#  define MCTRL2_MONPLL(n)                   ((uint32_t)(n) << MCTRL2_MONPLL_SHIFT)
#  define MCTRL2_MONPLL_NOT_LOCKED           (0 << MCTRL2_MONPLL_SHIFT)  /* Not locked. Do not monitor PLL operation. */
#  define MCTRL2_MONPLL_NOT_LOCKED_NONITOR   (1 << MCTRL2_MONPLL_SHIFT)  /* Not locked. Monitor PLL operation. */
#  define MCTRL2_MONPLL_LOCKED               (2 << MCTRL2_MONPLL_SHIFT)  /* Locked. Do not monitor PLL operation. */
#  define MCTRL2_MONPLL_LOCKED_NONITOR       (3 << MCTRL2_MONPLL_SHIFT)  /* Locked. Monitor PLL operation */

/* Fault Control Register */

#define FCTRL_FIE_SHIFT                      (0)        /* Bits: 0-3  Fault Interrupt Enables */
#define FCTRL_FIE_MASK                       (15 << FCTRL_FIE_SHIFT)
#  define FCTRL_FIE(n)                       ((uint32_t)(n) << FCTRL_FIE_SHIFT)
#  define FCTRL_FIE_ALL_DIS                  (0 << FCTRL_FIE_SHIFT)  /* All Normal mode. PWM outputs disabled by this fault are not enabled */
#  define FCTRL_FIE_FAULT0                   (1 << FCTRL_FIE_SHIFT)  /* FAULT0 Safe mode. PWM outputs disabled by this fault */
#  define FCTRL_FIE_FAULT1                   (2 << FCTRL_FIE_SHIFT)  /* FAULT1 Safe mode. PWM outputs disabled by this fault */
#  define FCTRL_FIE_FAULT2                   (4 << FCTRL_FIE_SHIFT)  /* FAULT2 Safe mode. PWM outputs disabled by this fault */
#  define FCTRL_FIE_FAULT3                   (8 << FCTRL_FIE_SHIFT)  /* FAULT3 Safe mode. PWM outputs disabled by this fault */
#define FCTRL0_FSAFE_SHIFT                   (4)        /* Bits: 4-7  Fault Safety Mode */
#define FCTRL0_FSAFE_MASK                    (15 << FCTRL0_FSAFE_SHIFT)
#  define FCTRL0_FSAFE(n)                    ((uint32_t)(n) << FCTRL0_FSAFE_SHIFT)
#  define FCTRL0_FSAFE_ALL_DIS               (0 << FCTRL0_FSAFE_SHIFT)  /* All FAULT CPU interrupt requests disabled */
#  define FCTRL0_FSAFE_FAULT0                (1 << FCTRL0_FSAFE_SHIFT)  /* FAULT0 CPU interrupt requests enabled */
#  define FCTRL0_FSAFE_FAULT1                (2 << FCTRL0_FSAFE_SHIFT)  /* FAULT1 CPU interrupt requests enabled */
#  define FCTRL0_FSAFE_FAULT2                (4 << FCTRL0_FSAFE_SHIFT)  /* FAULT2 CPU interrupt requests enabled */
#  define FCTRL0_FSAFE_FAULT3                (8 << FCTRL0_FSAFE_SHIFT)  /* FAULT3 CPU interrupt requests enabled */
#define FCTRL0_FAUTO_SHIFT                   (8)        /* Bits: 8-11  Automatic Fault Clearing */
#define FCTRL0_FAUTO_MASK                    (15 << FCTRL0_FAUTO_SHIFT)
#  define FCTRL0_FAUTO(n)                    ((uint32_t)(n) << FCTRL0_FAUTO_SHIFT)
#  define FCTRL0_FAUTO_ALL_MANUAL            (0 << FCTRL0_FAUTO_SHIFT)  /* All Manual fault clearing */
#  define FCTRL0_FAUTO_FAULT0                (1 << FCTRL0_FAUTO_SHIFT)  /* FAULT0 Automatic fault clearing. */
#  define FCTRL0_FAUTO_FAULT1                (2 << FCTRL0_FAUTO_SHIFT)  /* FAULT1 Automatic fault clearing. */
#  define FCTRL0_FAUTO_FAULT2                (4 << FCTRL0_FAUTO_SHIFT)  /* FAULT2 Automatic fault clearing. */
#  define FCTRL0_FAUTO_FAULT3                (8 << FCTRL0_FAUTO_SHIFT)  /* FAULT3 Automatic fault clearing. */
#define FCTRL0_FLVL_SHIFT                    (12)       /* Bits: 12-15  Fault Level */
#define FCTRL0_FLVL_MASK                     (15 << FCTRL0_FLVL_SHIFT)
#  define FCTRL0_FLVL(n)                     ((uint32_t)(n) << FCTRL0_FLVL_SHIFT)
#  define FCTRL0_FLVL_ALL_FAULT_LOW          (0 << FCTRL0_FLVL_SHIFT)  /* A logic 0 on the ALL fault input indicates a fault condition */
#  define FCTRL0_FLVL_FAULT0                 (1 << FCTRL0_FLVL_SHIFT)  /* A logic 1 on the FAULT0 input indicates a fault condition */
#  define FCTRL0_FLVL_FAULT1                 (2 << FCTRL0_FLVL_SHIFT)  /* A logic 1 on the FAULT1 input indicates a fault condition */
#  define FCTRL0_FLVL_FAULT2                 (4 << FCTRL0_FLVL_SHIFT)  /* A logic 1 on the FAULT2 input indicates a fault condition */
#  define FCTRL0_FLVL_FAULT3                 (8 << FCTRL0_FLVL_SHIFT)  /* A logic 1 on the FAULT3 input indicates a fault condition */

/* Fault Status Register */

#define FSTS_FFLAG_SHIFT                     (0)        /* Bits: 0-3  Fault Flags */
#define FSTS_FFLAG_MASK                      (15 << FSTS_FFLAG_SHIFT)
#  define FSTS_FFLAG(n)                      ((uint32_t)(n) << FSTS_FFLAG_SHIFT)
#  define FSTS_FFLAG_ALL_NO_FAULT            (0 << FSTS_FFLAG_SHIFT)  /* No fault on the ALL of the FAULT pins */
#  define FSTS_FFLAG_FAULT0                  (1 << FSTS_FFLAG_SHIFT)  /* Fault on the FAULT0 pin */
#  define FSTS_FFLAG_FAULT1                  (2 << FSTS_FFLAG_SHIFT)  /* Fault on the FAULT1 pin */
#  define FSTS_FFLAG_FAULT2                  (4 << FSTS_FFLAG_SHIFT)  /* Fault on the FAULT2 pin */
#  define FSTS_FFLAG_FAULT3                  (8 << FSTS_FFLAG_SHIFT)  /* Fault on the FAULT3 pin */
#define FSTS_FFULL_SHIFT                     (4)        /* Bits: 4-7  Full Cycle */
#define FSTS_FFULL_MASK                      (15 << FSTS_FFULL_SHIFT)
#  define FSTS_FFULL(n)                      ((uint32_t)(n) << FSTS_FFULL_SHIFT)
#  define FSTS_FFULL_ALL_NOT_REENABLED       (0 << FSTS_FFULL_SHIFT)  /* All PWM outputs are not re-enabled at the start of a full cycle */
#  define FSTS_FFULL_SM0                     (1 << FSTS_FFULL_SHIFT)  /* SM0 PWM output is re-enabled at the start of a full cycle */
#  define FSTS_FFULL_SM1                     (2 << FSTS_FFULL_SHIFT)  /* SM1 PWM output is re-enabled at the start of a full cycle */
#  define FSTS_FFULL_SM2                     (4 << FSTS_FFULL_SHIFT)  /* SM2 PWM output is re-enabled at the start of a full cycle */
#  define FSTS_FFULL_SM3                     (8 << FSTS_FFULL_SHIFT)  /* SM3 PWM output is re-enabled at the start of a full cycle */
#define FSTS_FFPIN_SHIFT                     (8)        /* Bits: 8-11  Filtered Fault Pins */
#define FSTS_FFPIN_MASK                      (15 << FSTS_FFPIN_SHIFT)
#  define FSTS_FFPIN(n)                      ((uint32_t)(n) << FSTS_FFPIN_SHIFT)
#  define FSTS_FFPIN_ALL_NO_FAULT            (0 << FSTS_FFPIN_SHIFT)  /* No Faults as current state of the filtered FAULT pins */
#  define FSTS_FFPIN_FAULT0                  (1 << FSTS_FFPIN_SHIFT)  /* This read-only bit reflect the current state of the filtered FAULT0 pin */
#  define FSTS_FFPIN_FAULT1                  (2 << FSTS_FFPIN_SHIFT)  /* This read-only bit reflect the current state of the filtered FAULT1 pin */
#  define FSTS_FFPIN_FAULT2                  (4 << FSTS_FFPIN_SHIFT)  /* This read-only bit reflect the current state of the filtered FAULT2 pin */
#  define FSTS_FFPIN_FAULT3                  (8 << FSTS_FFPIN_SHIFT)  /* This read-only bit reflect the current state of the filtered FAULT3 pin */
#define FSTS_FHALF_SHIFT                     (12)       /* Bits: 12-15  Half Cycle Fault Recovery */
#define FSTS_FHALF_MASK                      (15 << FSTS_FHALF_SHIFT)
#  define FSTS_FHALF(n)                      ((uint32_t)(n) << FSTS_FHALF_SHIFT)
#  define FSTS_FHALF_ALL_NOT_REENABLED       (0 << FSTS_FHALF_SHIFT)  /* All PWM outputs are not re-enabled at the start of a half cycle */
#  define FSTS_FHALF_SM0                     (1 << FSTS_FHALF_SHIFT)  /* SM0 PWM output is re-enabled at the start of a half cycle (as defined by VAL0).*/
#  define FSTS_FHALF_SM1                     (2 << FSTS_FHALF_SHIFT)  /* SM1 PWM output is re-enabled at the start of a half cycle (as defined by VAL0).*/
#  define FSTS_FHALF_SM2                     (4 << FSTS_FHALF_SHIFT)  /* SM2 PWM output is re-enabled at the start of a half cycle (as defined by VAL0).*/
#  define FSTS_FHALF_SM3                     (8 << FSTS_FHALF_SHIFT)  /* SM3 PWM output is re-enabled at the start of a half cycle (as defined by VAL0).*/

/* Fault Filter Register */

#define FFILT_FILT_PER_SHIFT                 (0)        /* Bits: 0-7  Fault Filter Period */
#define FFILT_FILT_PER_MASK                  (0xff << FFILT_FILT_PER_SHIFT)
#  define FFILT_FILT_PER(n)                  ((uint32_t)(n) << FFILT_FILT_PER_SHIFT)
#define FFILT_FILT_CNT_SHIFT                 (8)        /* Bits: 8-10  Fault Filter Count */
#define FFILT_FILT_CNT_MASK                  (7 << FFILT_FILT_CNT_SHIFT)
#  define FFILT_FILT_CNT(n)                  ((uint32_t)(n) << FFILT_FILT_CNT_SHIFT)
#  define FFILT_FILT_CNT_3                   (0 << FFILT_FILT_CNT_SHIFT)  /* */
#  define FFILT_FILT_CNT_4                   (1 << FFILT_FILT_CNT_SHIFT)  /* */
#  define FFILT_FILT_CNT_5                   (2 << FFILT_FILT_CNT_SHIFT)  /* */
#  define FFILT_FILT_CNT_6                   (3 << FFILT_FILT_CNT_SHIFT)  /* */
#  define FFILT_FILT_CNT_7                   (4 << FFILT_FILT_CNT_SHIFT)  /* */
#  define FFILT_FILT_CNT_8                   (5 << FFILT_FILT_CNT_SHIFT)  /* */
#  define FFILT_FILT_CNT_9                   (6 << FFILT_FILT_CNT_SHIFT)  /* */
#  define FFILT_FILT_CNT_10                  (7 << FFILT_FILT_CNT_SHIFT)  /* */
                                                        /* Bits: 11-14 Reserved */
#define FFILT_GSTR                           (1 << 15)  /* Bit: 15 Fault Glitch Stretch Enable */

/* Fault Test Register */

#define FTST0_FTEST                          (1 << 0)   /* Bit: 0  Fault Test */
                                                        /* Bits: 1-15 Reserved */

/* Fault Control 2 Register */

#define FCTRL20_NOCOMB_SHIFT                 (0)        /* Bits: 0-3  No Combinational Path From Fault Input To PWM Output */
#define FCTRL20_NOCOMB_MASK                  (0xf << FCTRL20_NOCOMB_SHIFT)
#  define FCTRL20_NOCOMB(n)                  ((uint32_t)(n) << FCTRL20_NOCOMB_SHIFT)
#  define FCTRL20_NOCOMB_ALL_ENABLED         (0 << FCTRL20_NOCOMB_SHIFT)  /* All combinational link from the fault inputs to the PWM outputs are enabled */
#  define FCTRL20_NOCOMB_FAULT0              (1 << FCTRL20_NOCOMB_SHIFT)  /*  Disable direct combinational path from the FAULT0 input to the PWM output */
#  define FCTRL20_NOCOMB_FAULT1              (2 << FCTRL20_NOCOMB_SHIFT)  /*  Disable direct combinational path from the FAULT1 input to the PWM output */
#  define FCTRL20_NOCOMB_FAULT2              (4 << FCTRL20_NOCOMB_SHIFT)  /*  Disable direct combinational path from the FAULT2 input to the PWM output */
#  define FCTRL20_NOCOMB_FAULT3              (8 << FCTRL20_NOCOMB_SHIFT)  /*  Disable direct combinational path from the FAULT3 input to the PWM output */

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_IMXRT_FLEXPWM_H */
