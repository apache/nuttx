/****************************************************************************************************
 * arch/arm/src/s32k1xx/chip/s32k1xx_rtc.h
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

#define RTC_TSR_SHIFT                           (0)
#define RTC_TSR_MASK                            (0xffffffff << RTC_TSR_SHIFT)

/* CR Bit Fields */

#define RTC_CR_SWR                              (1 << 0)
#define RTC_CR_SUP                              (1 << 2)
#define RTC_CR_UM                               (1 << 3)
#define RTC_CR_CPS                              (1 << 5)
#define RTC_CR_LPOS                             (1 << 7)
#define RTC_CR_CPE                              (1 << 24)

/* SR Bit Fields */

#define RTC_SR_TIF                              (1 << 0)
#define RTC_SR_TOF                              (1 << 1)
#define RTC_SR_TAF                              (1 << 2)
#define RTC_SR_TCE                              (1 << 4)
#endif /* __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_RTC_H */
