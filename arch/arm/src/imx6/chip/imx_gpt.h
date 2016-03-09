/************************************************************************************
 * arch/arm/src/imx6/imx_gpt.h
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

#ifndef __ARCH_ARM_SRC_IMX6_CHIP_IMX_GPT_H
#define __ARCH_ARM_SRC_IMX6_CHIP_IMX_GPT_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <chip/imx_memorymap.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* GPT Register Offsets ************************************************************/

#define IMX_GPT_CR_OFFSET          0x0000 /* GPT Control Register */
#define IMX_GPT_PR_OFFSET          0x0004 /* GPT Prescaler Register */
#define IMX_GPT_SR_OFFSET          0x0008 /* GPT Status Register */
#define IMX_GPT_IR_OFFSET          0x000c /* GPT Interrupt Register */
#define IMX_GPT_OCR1_OFFSET        0x0010 /* GPT Output Compare Register 1 */
#define IMX_GPT_OCR2_OFFSET        0x0014 /* GPT Output Compare Register 2 */
#define IMX_GPT_OCR3_OFFSET        0x0018 /* GPT Output Compare Register 3 */
#define IMX_GPT_ICR1_OFFSET        0x001c /* GPT Input Capture Register 1 */
#define IMX_GPT_ICR2_OFFSET        0x0020 /* GPT Input Capture Register 2 */
#define IMX_GPT_CNT_OFFSET         0x0024 /* GPT Counter Register */

/* GPT Register Addresses **********************************************************/

#define IMX_GPT_CR                 (IMX_GPT_VBASE+IMX_GPT_CR_OFFSET)
#define IMX_GPT_PR                 (IMX_GPT_VBASE+IMX_GPT_PR_OFFSET)
#define IMX_GPT_SR                 (IMX_GPT_VBASE+IMX_GPT_SR_OFFSET)
#define IMX_GPT_IR                 (IMX_GPT_VBASE+IMX_GPT_IR_OFFSET)
#define IMX_GPT_OCR1               (IMX_GPT_VBASE+IMX_GPT_OCR1_OFFSET)
#define IMX_GPT_OCR2               (IMX_GPT_VBASE+IMX_GPT_OCR2_OFFSET)
#define IMX_GPT_OCR3               (IMX_GPT_VBASE+IMX_GPT_OCR3_OFFSET)
#define IMX_GPT_ICR1               (IMX_GPT_VBASE+IMX_GPT_ICR1_OFFSET)
#define IMX_GPT_ICR2               (IMX_GPT_VBASE+IMX_GPT_ICR2_OFFSET)
#define IMX_GPT_CNT                (IMX_GPT_VBASE+IMX_GPT_CNT_OFFSET)

/* GPT Register Bit Definitions ****************************************************/

/* GPT Control Register */
#define GPT_CR_
/* GPT Prescaler Register */
#define GPT_PR_
/* GPT Status Register */
#define GPT_SR_
/* GPT Interrupt Register */
#define GPT_IR_
/* GPT Output Compare Register 1 */
#define GPT_OCR1_
/* GPT Output Compare Register 2 */
#define GPT_OCR2_
/* GPT Output Compare Register 3 */
#define GPT_OCR3_
/* GPT Input Capture Register 1 */
#define GPT_ICR1_
/* GPT Input Capture Register 2 */
#define GPT_ICR2_
/* GPT Counter Register */
#define GPT_CNT_

#endif /* __ARCH_ARM_SRC_IMX6_CHIP_IMX_GPT_H */
