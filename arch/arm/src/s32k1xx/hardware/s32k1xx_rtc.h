/****************************************************************************
 * arch/arm/src/s32k1xx/hardware/s32k1xx_rtc.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_RTC_H
#define __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_RTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <hardware/s32k1xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RTC Register Offsets *****************************************************/

#define S32K1XX_RTC_TSR_OFFSET                  0x0000  /* Time Seconds register */
#define S32K1XX_RTC_TPR_OFFSET                  0x0004  /* Time Prescaler Register */
#define S32K1XX_RTC_TAR_OFFSET                  0x0008  /* Time Alarm Register */
#define S32K1XX_RTC_TCR_OFFSET                  0x000C  /* Time Compensation Register */
#define S32K1XX_RTC_CR_OFFSET                   0x0010  /* Control Register */
#define S32K1XX_RTC_SR_OFFSET                   0x0014  /* Status Register */
#define S32K1XX_RTC_LR_OFFSET                   0x0018  /* Lock Register */
#define S32K1XX_RTC_IER_OFFSET                  0x001C  /* Interrupt Enable Register */

/* RTC Register Addresses ***************************************************/

#define S32K1XX_RTC_TSR                         (S32K1XX_RTC_BASE + S32K1XX_RTC_TSR_OFFSET)
#define S32K1XX_RTC_TPR                         (S32K1XX_RTC_BASE + S32K1XX_RTC_TPR_OFFSET)
#define S32K1XX_RTC_TAR                         (S32K1XX_RTC_BASE + S32K1XX_RTC_TAR_OFFSET)
#define S32K1XX_RTC_TCR                         (S32K1XX_RTC_BASE + S32K1XX_RTC_TCR_OFFSET)
#define S32K1XX_RTC_CR                          (S32K1XX_RTC_BASE + S32K1XX_RTC_CR_OFFSET)
#define S32K1XX_RTC_SR                          (S32K1XX_RTC_BASE + S32K1XX_RTC_SR_OFFSET)
#define S32K1XX_RTC_LR                          (S32K1XX_RTC_BASE + S32K1XX_RTC_LR_OFFSET)
#define S32K1XX_RTC_IER                         (S32K1XX_RTC_BASE + S32K1XX_RTC_IER_OFFSET)

/* RTC Register Bitfield Definitions ****************************************/

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
