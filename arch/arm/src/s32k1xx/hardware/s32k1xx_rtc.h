/****************************************************************************************************
 * arch/arm/src/s32k1xx/chip/s32k1xx_sim.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_RTC_H
#define __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_RTC_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include <hardware/s32k1xx_memorymap.h>

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* RTC Register Offsets *****************************************************************************/

#define S32K1XX_RTC_TSR_OFFSET                  0x0000  /* Time Seconds register */
#define S32K1XX_RTC_TPR_OFFSET                  0x0004  /* Time Prescaler Register */
#define S32K1XX_RTC_TAR_OFFSET                  0x0008  /* Time Alarm Register */
#define S32K1XX_RTC_TCR_OFFSET                  0x000C  /* Time Compensation Register */
#define S32K1XX_RTC_CR_OFFSET                   0x0010  /* Control Register */
#define S32K1XX_RTC_SR_OFFSET                   0x0014  /* Status Register */
#define S32K1XX_RTC_LR_OFFSET                   0x0018  /* Lock Register */
#define S32K1XX_RTC_IER_OFFSET                  0x001C  /* Interrupt Enable Register */

/* RTC Register Addresses ***************************************************************************/

#define S32K1XX_RTC_TSR                         (S32K1XX_RTC_BASE + S32K1XX_RTC_TSR_OFFSET)
#define S32K1XX_RTC_TPR                         (S32K1XX_RTC_BASE + S32K1XX_RTC_TPR_OFFSET)
#define S32K1XX_RTC_TAR                         (S32K1XX_RTC_BASE + S32K1XX_RTC_TAR_OFFSET)
#define S32K1XX_RTC_TCR                         (S32K1XX_RTC_BASE + S32K1XX_RTC_TCR_OFFSET)
#define S32K1XX_RTC_CR                          (S32K1XX_RTC_BASE + S32K1XX_RTC_CR_OFFSET)
#define S32K1XX_RTC_SR                          (S32K1XX_RTC_BASE + S32K1XX_RTC_SR_OFFSET)
#define S32K1XX_RTC_LR                          (S32K1XX_RTC_BASE + S32K1XX_RTC_LR_OFFSET)
#define S32K1XX_RTC_IER                         (S32K1XX_RTC_BASE + S32K1XX_RTC_IER_OFFSET)

/* RTC Register Bitfield Definitions ****************************************************************/

/* TSR Bit Fields */
#define RTC_TSR_TSR_MASK                         0xFFFFFFFFu
#define RTC_TSR_TSR_SHIFT                        0u
#define RTC_TSR_TSR_WIDTH                        32u
#define RTC_TSR_TSR(x)                           (((uint32_t)(((uint32_t)(x))<<RTC_TSR_TSR_SHIFT))&RTC_TSR_TSR_MASK)

/* CR Bit Fields */
#define RTC_CR_SWR_MASK                          0x1u
#define RTC_CR_SWR_SHIFT                         0u
#define RTC_CR_SWR_WIDTH                         1u
#define RTC_CR_SWR(x)                            (((uint32_t)(((uint32_t)(x))<<RTC_CR_SWR_SHIFT))&RTC_CR_SWR_MASK)
#define RTC_CR_SUP_MASK                          0x4u
#define RTC_CR_SUP_SHIFT                         2u
#define RTC_CR_SUP_WIDTH                         1u
#define RTC_CR_SUP(x)                            (((uint32_t)(((uint32_t)(x))<<RTC_CR_SUP_SHIFT))&RTC_CR_SUP_MASK)
#define RTC_CR_UM_MASK                           0x8u
#define RTC_CR_UM_SHIFT                          3u
#define RTC_CR_UM_WIDTH                          1u
#define RTC_CR_UM(x)                             (((uint32_t)(((uint32_t)(x))<<RTC_CR_UM_SHIFT))&RTC_CR_UM_MASK)
#define RTC_CR_CPS_MASK                          0x20u
#define RTC_CR_CPS_SHIFT                         5u
#define RTC_CR_CPS_WIDTH                         1u
#define RTC_CR_CPS(x)                            (((uint32_t)(((uint32_t)(x))<<RTC_CR_CPS_SHIFT))&RTC_CR_CPS_MASK)
#define RTC_CR_LPOS_MASK                         0x80u
#define RTC_CR_LPOS_SHIFT                        7u
#define RTC_CR_LPOS_WIDTH                        1u
#define RTC_CR_LPOS(x)                           (((uint32_t)(((uint32_t)(x))<<RTC_CR_LPOS_SHIFT))&RTC_CR_LPOS_MASK)
#define RTC_CR_CPE_MASK                          0x1000000u
#define RTC_CR_CPE_SHIFT                         24u
#define RTC_CR_CPE_WIDTH                         1u
#define RTC_CR_CPE(x)                            (((uint32_t)(((uint32_t)(x))<<RTC_CR_CPE_SHIFT))&RTC_CR_CPE_MASK)


/* SR Bit Fields */
#define RTC_SR_TIF_MASK                          0x1u
#define RTC_SR_TIF_SHIFT                         0u
#define RTC_SR_TIF_WIDTH                         1u
#define RTC_SR_TIF(x)                            (((uint32_t)(((uint32_t)(x))<<RTC_SR_TIF_SHIFT))&RTC_SR_TIF_MASK)
#define RTC_SR_TOF_MASK                          0x2u
#define RTC_SR_TOF_SHIFT                         1u
#define RTC_SR_TOF_WIDTH                         1u
#define RTC_SR_TOF(x)                            (((uint32_t)(((uint32_t)(x))<<RTC_SR_TOF_SHIFT))&RTC_SR_TOF_MASK)
#define RTC_SR_TAF_MASK                          0x4u
#define RTC_SR_TAF_SHIFT                         2u
#define RTC_SR_TAF_WIDTH                         1u
#define RTC_SR_TAF(x)                            (((uint32_t)(((uint32_t)(x))<<RTC_SR_TAF_SHIFT))&RTC_SR_TAF_MASK)
#define RTC_SR_TCE_MASK                          0x10u
#define RTC_SR_TCE_SHIFT                         4u
#define RTC_SR_TCE_WIDTH                         1u
#define RTC_SR_TCE(x)                            (((uint32_t)(((uint32_t)(x))<<RTC_SR_TCE_SHIFT))&RTC_SR_TCE_MASK)


#endif /* __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_RTC_H */
