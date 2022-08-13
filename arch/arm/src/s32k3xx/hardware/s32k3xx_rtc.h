/****************************************************************************
 * arch/arm/src/s32k3xx/hardware/s32k3xx_rtc.h
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

/* Copyright 2022 NXP */

#ifndef __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_RTC_H
#define __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_RTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <hardware/s32k3xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RTC Register Offsets *****************************************************/

#define S32K3XX_RTC_RTCSUPV_OFFSET (0x00) /* RTC Supervisor Control Register (RTCSUPV) */
#define S32K3XX_RTC_RTCC_OFFSET    (0x04) /* RTC Control Register (RTCC) */
#define S32K3XX_RTC_RTCS_OFFSET    (0x08) /* RTC Status Register (RTCS) */
#define S32K3XX_RTC_RTCCNT_OFFSET  (0x0c) /* RTC Counter Register (RTCCNT) */
#define S32K3XX_RTC_APIVAL_OFFSET  (0x10) /* API Compare Value Register (APIVAL) */
#define S32K3XX_RTC_RTCVAL_OFFSET  (0x14) /* RTC Compare Value Register (RTCVAL) */

/* RTC Register Addresses ***************************************************/

#define S32K3XX_RTC_RTCSUPV        (S32K3XX_RTC_BASE + S32K3XX_RTC_RTCSUPV_OFFSET)
#define S32K3XX_RTC_RTCC           (S32K3XX_RTC_BASE + S32K3XX_RTC_RTCC_OFFSET)
#define S32K3XX_RTC_RTCS           (S32K3XX_RTC_BASE + S32K3XX_RTC_RTCS_OFFSET)
#define S32K3XX_RTC_RTCCNT         (S32K3XX_RTC_BASE + S32K3XX_RTC_RTCCNT_OFFSET)
#define S32K3XX_RTC_APIVAL         (S32K3XX_RTC_BASE + S32K3XX_RTC_APIVAL_OFFSET)
#define S32K3XX_RTC_RTCVAL         (S32K3XX_RTC_BASE + S32K3XX_RTC_RTCVAL_OFFSET)

/* RTC Register Bitfield Definitions ****************************************/

/* RTC Supervisor Control Register (RTCSUPV) */

                                             /* Bits 0-30: Reserved */
#define RTC_RTCSUPV_SUPV           (1 << 31) /* Bit 31: RTC Supervisor Bit (SUPV) */

/* RTC Control Register (RTCC) */

#define RTC_RTCC_TRIG_EN           (1 << 0)  /* Bit 0: Trigger enable for Analog Comparator (TRIG_EN) */
                                             /* Bit 1-9: Reserved */
#define RTC_RTCC_DIV32EN           (1 << 10) /* Bit 10: Divide by 32 enable (DIV32EN) */
#define RTC_RTCC_DIV512EN          (1 << 11) /* Bit 11: Divide by 512 enable (DIV512EN) */
#define RTC_RTCC_CLKSEL_SHIFT      (12)      /* Bits 12-13: Clock select (CLKSEL) */
#define RTC_RTCC_CLKSEL_MASK       (0x03 << RTC_RTCC_CLKSEL_SHIFT)
#define RTC_RTCC_CLKSEL0           (0x00 << RTC_RTCC_CLKSEL_SHIFT) /* Clock source 0 */
#define RTC_RTCC_CLKSEL1           (0x01 << RTC_RTCC_CLKSEL_SHIFT) /* Clock source 1 */
#define RTC_RTCC_CLKSEL2           (0x02 << RTC_RTCC_CLKSEL_SHIFT) /* Clock source 2 */
#define RTC_RTCC_CLKSEL3           (0x03 << RTC_RTCC_CLKSEL_SHIFT) /* Clock source 3 */

#define RTC_RTCC_APIIE             (1 << 14) /* Bit 14: API Interrupt Enable (APIIE) */
#define RTC_RTCC_APIEN             (1 << 15) /* Bit 15: Autonomous Periodic Interrupt Enable (APIEN) */
                                             /* Bits 16-27: Reserved */
#define RTC_RTCC_ROVREN            (1 << 28) /* Bit 28: Counter Roll Over Wakeup/Interrupt Enable (ROVREN) */
#define RTC_RTCC_FRZEN             (1 << 29) /* Bit 29: Freeze Enable Bit (FRZEN) */
#define RTC_RTCC_RTCIE             (1 << 30) /* Bit 30: RTC Interrupt Enable (RTCIE) */
#define RTC_RTCC_CNTEN             (1 << 31) /* Bit 31: Counter Enable (CNTEN) */

/* RTC Status Register (RTCS) */

                                             /* Bits 0-9: Reserved */
#define RTC_RTCS_ROVRF             (1 << 10) /* Bit 10: Counter Roll Over Interrupt Flag (ROVRF) */
                                             /* Bits 11-12: Reserved */
#define RTC_RTCS_APIF              (1 << 13) /* Bit 13: API Interrupt Flag (APIF) */
                                             /* Bits 14-16: Reserved */
#define RTC_RTCS_INV_API           (1 << 17) /* Bit 17: Invalid APIVAL write (INV_API) */
#define RTC_RTCS_INV_RTC           (1 << 18) /* Bit 18: Invalid RTC write (INV_RTC) */
                                             /* Bits 19-28: Reserved */
#define RTC_RTCS_RTCF              (1 << 29) /* Bit 29: RTC Interrupt Flag (RTCF) */
                                             /* Bits 30-31: Reserved */

/* RTC Counter Register (RTCCNT) */

#define RTC_RTCCNT_SHIFT           (0)       /* Bits 0-31: RTC Counter Value (RTCCNT) */
#define RTC_RTCCNT_MASK            (0xffffffff << RTC_RTCCNT_SHIFT)

/* API Compare Value Register (APIVAL) */

#define RTC_APIVAL_SHIFT           (0)       /* Bits 0-31: API Compare Value (APIVAL) */
#define RTC_APIVAL_MASK            (0xffffffff << RTC_APIVAL_SHIFT)

/* RTC Compare Value Register (RTCVAL) */

#define RTC_RTCVAL_SHIFT           (0)       /* Bits 0-31: RTC Compare Value (RTCVAL) */
#define RTC_RTCVAL_MASK            (0xffffffff << RTC_RTCVAL_SHIFT)

#endif /* __ARCH_ARM_SRC_S32K3XX_HARDWARE_S32K3XX_RTC_H */
