/************************************************************************************
 * arch/arm/src/s32k1xx/chip/s32k1xx_cmu.h
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_CMU_H
#define __ARCH_ARM_SRC_S32K1XX_HARDWARE_S32K1XX_CMU_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <hardware/s32k1xx_memorymap.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* CMU Register Offsets *************************************************************/

#define S32K1XX_CMU_GCR_OFFSET   0x0000  /* Global Configuration Register */
#define S32K1XX_CMU_RCCR_OFFSET  0x0004  /* Reference Count Configuration Register */
#define S32K1XX_CMU_HTCR_OFFSET  0x0008  /* High Threshold Configuration Register */
#define S32K1XX_CMU_LTCR_OFFSET  0x000c  /* Low Threshold Configuration Register */
#define S32K1XX_CMU_SR_OFFSET    0x0010  /* Status Register */
#define S32K1XX_CMU_IER_OFFSET   0x0014  /* Interrupt Enable Register */

/* CMU Register Addresses ***********************************************************/

#define S32K1XX_CMU_GCR          (S32K1XX_CMU_BASE + S32K1XX_CMU_GCR_OFFSET)
#define S32K1XX_CMU_RCCR         (S32K1XX_CMU_BASE + S32K1XX_CMU_RCCR_OFFSET)
#define S32K1XX_CMU_HTCR         (S32K1XX_CMU_BASE + S32K1XX_CMU_HTCR_OFFSET)
#define S32K1XX_CMU_LTCR         (S32K1XX_CMU_BASE + S32K1XX_CMU_LTCR_OFFSET)
#define S32K1XX_CMU_SR           (S32K1XX_CMU_BASE + S32K1XX_CMU_SR_OFFSET)
#define S32K1XX_CMU_IER          (S32K1XX_CMU_BASE + S32K1XX_CMU_IER_OFFSET)

/* CMU Register Bitfield Definitions ************************************************/

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
