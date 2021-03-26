/****************************************************************************
 * arch/arm/src/s32k1xx/hardware/s32k1xx_cmu.h
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

#ifndef __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_CMU_H
#define __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_CMU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <hardware/s32k1xx_memorymap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CMU Register Offsets *****************************************************/

#define S32K1XX_CMU_GCR_OFFSET   0x0000  /* Global Configuration Register */
#define S32K1XX_CMU_RCCR_OFFSET  0x0004  /* Reference Count Configuration Register */
#define S32K1XX_CMU_HTCR_OFFSET  0x0008  /* High Threshold Configuration Register */
#define S32K1XX_CMU_LTCR_OFFSET  0x000c  /* Low Threshold Configuration Register */
#define S32K1XX_CMU_SR_OFFSET    0x0010  /* Status Register */
#define S32K1XX_CMU_IER_OFFSET   0x0014  /* Interrupt Enable Register */

/* CMU Register Addresses ***************************************************/

#define S32K1XX_CMU_GCR          (S32K1XX_CMU_BASE + S32K1XX_CMU_GCR_OFFSET)
#define S32K1XX_CMU_RCCR         (S32K1XX_CMU_BASE + S32K1XX_CMU_RCCR_OFFSET)
#define S32K1XX_CMU_HTCR         (S32K1XX_CMU_BASE + S32K1XX_CMU_HTCR_OFFSET)
#define S32K1XX_CMU_LTCR         (S32K1XX_CMU_BASE + S32K1XX_CMU_LTCR_OFFSET)
#define S32K1XX_CMU_SR           (S32K1XX_CMU_BASE + S32K1XX_CMU_SR_OFFSET)
#define S32K1XX_CMU_IER          (S32K1XX_CMU_BASE + S32K1XX_CMU_IER_OFFSET)

/* CMU Register Bitfield Definitions ****************************************/

/* Global Configuration Register */

#define CMU_GCR_FCE              (1 << 0)  /* Bit 0:  Frequency Check Enable */

/* Reference Count Configuration Register */

#define CMU_RCCR_REFCNT_SHIFT    (0)       /* Bits 0-15:  Reference clock count */
#define CMU_RCCR_REFCNT_MASK     (0xffff << CMU_RCCR_REFCNT_SHIFT)
#  define CMU_RCCR_REFCNT(n)     ((uint32_t)(n) << CMU_RCCR_REFCNT_SHIFT)

/* High Threshold Configuration Register */

#define CMU_HTCR_HFREF_SHIFT     (0)       /* Bits 0-23:  High frequency reference threshold */
#define CMU_HTCR_HFREF_MASK      (0xffffff << CMU_HTCR_HFREF_SHIFT)
#  define CMU_HTCR_HFREF(n)      ((uint32_t)(n) << CMU_HTCR_HFREF_SHIFT)

/* Low Threshold Configuration Register */

#define CMU_LTCR_LFREF_SHIFT     (0)       /* Bits 0-23:  Low Frequency Reference Threshold. */
#define CMU_LTCR_LFREF_MASK      (0xffffff << CMU_LTCR_LFREF_SHIFT)
#  define CMU_LTCR_LFREF(n)      ((uint32_t)(n) << CMU_LTCR_LFREF_SHIFT)

/* Status Register */

#define CMU_SR_FLL               (1 << 0)  /* Bit 0:  Frequency < low frequency reference threshold */
#define CMU_SR_FHH               (1 << 1)  /* Bit 1:  Frequency > high frequency reference threshold */
#define CMU_SR_STATE_SHIFT       (2)       /* Bits 2-3:  Module state */
#define CMU_SR_STATE_MASK        (3 << CMU_SR_STATE_SHIFT)
#  define CMU_SR_STATE_INIT      (1 << CMU_SR_STATE_SHIFT)  /* Initialization state */
#  define CMU_SR_STATE_INITWAIT  (2 << CMU_SR_STATE_SHIFT)  /* Initialization wait state */
#  define CMU_SR_STATE_FREQCHECK (3 << CMU_SR_STATE_SHIFT)  /* Frequency check state */

#define CMU_SR_RS                (1 << 4)  /* Bit 4: Run Status */

/* Interrupt Enable Register */

#define CMU_IER_FLLIE            (1 << 0)  /* Bit 0:  Frequency < low frequency reference threshold,
                                            * synchronous */
#define CMU_IER_FHHIE            (1 << 1)  /* Bit 1:  Frequency > high frequency reference threshold,
                                            * synchronous */
#define CMU_IER_FLLAIE           (1 << 2)  /* Bit 2:  Frequency < low frequency reference threshold,
                                            * asynchronous */
#define CMU_IER_FHHAIE           (1 << 3)  /* Bit 3:  Frequency > high frequency reference threshold,
                                            * asynchronous */

#endif /* __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_CMU_H */
