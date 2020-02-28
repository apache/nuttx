/********************************************************************************************
 * arch/arm/src/samd2l2/hardware/samd_eic.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Matt Thompson <matt@extent3d.com>
 *
 * References:
 *   "Microchip SAMD21 datasheet"
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
 ********************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAMD_EIC_H
#define __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAMD_EIC_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#ifdef CONFIG_ARCH_FAMILY_SAMD21

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/
/* EIC register offsets *********************************************************************/

#define SAM_EIC_CTRLA_OFFSET         0x0000  /* Control A register */
#define SAM_EIC_STATUS_OFFSET        0x0001  /* Status register */
#define SAM_EIC_NMICTRL_OFFSET       0x0002  /* Non-maskable interrupt control register */
#define SAM_EIC_NMIFLAG_OFFSET       0x0003  /* Non-maskable interrupt flag register */
#define SAM_EIC_EVCTRL_OFFSET        0x0004  /* Event control register */
#define SAM_EIC_INTENCLR_OFFSET      0x0008  /* Interrupt enable clear register */
#define SAM_EIC_INTENSET_OFFSET      0x000c  /* Interrupt enable set register */
#define SAM_EIC_INTFLAG_OFFSET       0x0010  /* Interrupt flag and status clear register */
#define SAM_EIC_WAKEUP_OFFSET        0x0014  /* Wakeup register */
#define SAM_EIC_CONFIG0_OFFSET       0x0018  /* Configuration 0 register */
#define SAM_EIC_CONFIG1_OFFSET       0x001c  /* Configuration 1 register */
#define SAM_EIC_CONFIG2_OFFSET       0x0020  /* Configuration 2 register */

/* EIC register addresses *******************************************************************/

#define SAM_EIC_CTRLA                (SAM_EIC_BASE+SAM_EIC_CTRLA_OFFSET)
#define SAM_EIC_STATUS               (SAM_EIC_BASE+SAM_EIC_STATUS_OFFSET)
#define SAM_EIC_NMICTRL              (SAM_EIC_BASE+SAM_EIC_NMICTRL_OFFSET)
#define SAM_EIC_NMIFLAG              (SAM_EIC_BASE+SAM_EIC_NMIFLAG_OFFSET)
#define SAM_EIC_EVCTRL               (SAM_EIC_BASE+SAM_EIC_EVCTRL_OFFSET)
#define SAM_EIC_INTENCLR             (SAM_EIC_BASE+SAM_EIC_INTENCLR_OFFSET)
#define SAM_EIC_INTENSET             (SAM_EIC_BASE+SAM_EIC_INTENSET_OFFSET)
#define SAM_EIC_INTFLAG              (SAM_EIC_BASE+SAM_EIC_INTFLAG_OFFSET)
#define SAM_EIC_WAKEUP               (SAM_EIC_BASE+SAM_EIC_WAKEUP_OFFSET)
#define SAM_EIC_CONFIG0              (SAM_EIC_BASE+SAM_EIC_CONFIG0_OFFSET)
#define SAM_EIC_CONFIG1              (SAM_EIC_BASE+SAM_EIC_CONFIG1_OFFSET)
#define SAM_EIC_CONFIG2              (SAM_EIC_BASE+SAM_EIC_CONFIG2_OFFSET)

/* EIC register bit definitions *************************************************************/

/* Control A register */

#define EIC_CTRLA_SWRST              (1 << 0)  /* Bit 0:  Software reset */
#define EIC_CTRLA_ENABLE             (1 << 1)  /* Bit 1:  Enable */

/* Status register */

#define EIC_STATUS_SYNCBUSY          (1 << 7)  /* Bit 7:  Synchronization busy */

/* Non-maskable interrupt control register */

#define EIC_NMICTRL_NMISENSE_SHIFT  (0)       /* Bits 0-2: Non-maskable interrupt sense */
#define EIC_NMICTRL_NMISENSE_MASK   (7 << EIC_NVMICTRL_NMISENSE_SHIFT)
#  define EIC_NMICTRL_NMISENSE_NONE (0 << EIC_NVMICTRL_NMISENSE_SHIFT) /* No detection */
#  define EIC_NMICTRL_NMISENSE_RISE (1 << EIC_NVMICTRL_NMISENSE_SHIFT) /* Rising edge detection */
#  define EIC_NMICTRL_NMISENSE_FALL (2 << EIC_NVMICTRL_NMISENSE_SHIFT) /* Falling edge detection */
#  define EIC_NMICTRL_NMISENSE_BOTH (3 << EIC_NVMICTRL_NMISENSE_SHIFT) /* Both edge detection */
#  define EIC_NMICTRL_NMISENSE_HIGH (4 << EIC_NVMICTRL_NMISENSE_SHIFT) /* High level detection */
#  define EIC_NMICTRL_NMISENSE_LOW  (5 << EIC_NVMICTRL_NMISENSE_SHIFT) /* Low level detection */
#define EIC_NMICTRL_NMIFLTEN        (1 << 3)  /* Bit 3: Non-maskable interrupt filter enable */

/* Non-maskable interrupt flas status and clear register */

#define EIC_NMIFLAG_NMI              (1 << 0)  /* Non-maskable interrupt */

/* Event control, Interrupt enable clear, interrupt enable set register, interrupt flag
 * status and clear, and external interrupt wakeup registers.
 */

#define EIC_EXTINT_SHIFT             (0)       /* Bits 0-15: External interrupt n */
#define EIC_EXTINT_MASK              (0x3ffff << EIC_EXTINT_SHIFT)
#  define EIC_EXTINT(n)              (1 << (n))
#  define EIC_EXTINT_0               (1 << 0)  /* Bit 0: External interrupt 0 */
#  define EIC_EXTINT_1               (1 << 1)  /* Bit 1: External interrupt 1 */
#  define EIC_EXTINT_2               (1 << 2)  /* Bit 2: External interrupt 2 */
#  define EIC_EXTINT_3               (1 << 3)  /* Bit 3: External interrupt 3 */
#  define EIC_EXTINT_4               (1 << 4)  /* Bit 4: External interrupt 4 */
#  define EIC_EXTINT_5               (1 << 5)  /* Bit 5: External interrupt 5 */
#  define EIC_EXTINT_6               (1 << 6)  /* Bit 6: External interrupt 6 */
#  define EIC_EXTINT_7               (1 << 7)  /* Bit 7: External interrupt 7 */
#  define EIC_EXTINT_8               (1 << 8)  /* Bit 8: External interrupt 8 */
#  define EIC_EXTINT_9               (1 << 9)  /* Bit 9: External interrupt 9 */
#  define EIC_EXTINT_10              (1 << 10) /* Bit 10: External interrupt 10 */
#  define EIC_EXTINT_11              (1 << 11) /* Bit 11: External interrupt 11 */
#  define EIC_EXTINT_12              (1 << 12) /* Bit 12: External interrupt 12 */
#  define EIC_EXTINT_13              (1 << 13) /* Bit 13: External interrupt 13 */
#  define EIC_EXTINT_14              (1 << 14) /* Bit 14: External interrupt 14 */
#  define EIC_EXTINT_15              (1 << 15) /* Bit 15: External interrupt 15 */
#  define EIC_EXTINT_16              (1 << 16) /* Bit 16: External interrupt 16 */
#  define EIC_EXTINT_17              (1 << 17) /* Bit 17: External interrupt 17 */

#define EIC_EXTINT_ALL               EIC_EXTINT_MASK

/* Configuration 0 register */

#define EIC_CONFIG0_FILTEN(n)        (0x8 << ((n) << 2))               /* Filter n enable, n=0-7 */
#define EIC_CONFIG0_SENSE_SHIFT(n)   ((n) << 2)                        /* Filter n input sense, n=0-7 */
#define EIC_CONFIG0_SENSE_MASK(n)    (7 << EIC_CONFIG0_SENSE_SHIFT(n))
#  define EIC_CONFIG0_SENSE_NONE(n)  (0 << EIC_CONFIG0_SENSE_SHIFT(n)) /* No detection */
#  define EIC_CONFIG0_SENSE_RISE(n)  (1 << EIC_CONFIG0_SENSE_SHIFT(n)) /* Rising edge detection */
#  define EIC_CONFIG0_SENSE_FALL(n)  (2 << EIC_CONFIG0_SENSE_SHIFT(n)) /* Falling edge detection */
#  define EIC_CONFIG0_SENSE_BOTH(n)  (3 << EIC_CONFIG0_SENSE_SHIFT(n)) /* Both edge detection */
#  define EIC_CONFIG0_SENSE_HIGH(n)  (4 << EIC_CONFIG0_SENSE_SHIFT(n)) /* High level detection */
#  define EIC_CONFIG0_SENSE_LOW(n)   (5 << EIC_CONFIG0_SENSE_SHIFT(n)) /* Low level detection */

/* Configuration 1 register */

#define EIC_CONFIG1_FILTEN(n)        (0x8 << (((n) - 8) << 2))         /* Filter n enable, n=8-15 */
#define EIC_CONFIG1_SENSE_SHIFT(n)   (((n) - 8) << 2)                  /* Filter n input sense, n=8-17 */
#define EIC_CONFIG1_SENSE_MASK(n)    (7 << EIC_CONFIG1_SENSE_SHIFT(n))
#  define EIC_CONFIG1_SENSE_NONE(n)  (0 << EIC_CONFIG1_SENSE_SHIFT(n)) /* No detection */
#  define EIC_CONFIG1_SENSE_RISE(n)  (1 << EIC_CONFIG1_SENSE_SHIFT(n)) /* Rising edge detection */
#  define EIC_CONFIG1_SENSE_FALL(n)  (2 << EIC_CONFIG1_SENSE_SHIFT(n)) /* Falling edge detection */
#  define EIC_CONFIG1_SENSE_BOTH(n)  (3 << EIC_CONFIG1_SENSE_SHIFT(n)) /* Both edge detection */
#  define EIC_CONFIG1_SENSE_HIGH(n)  (4 << EIC_CONFIG1_SENSE_SHIFT(n)) /* High level detection */
#  define EIC_CONFIG1_SENSE_LOW(n)   (5 << EIC_CONFIG1_SENSE_SHIFT(n)) /* Low level detection */

/* Configuration 2 register */

#define EIC_CONFIG2_FILTEN(n)        (0x8 << (((n) - 16) << 2))        /* Filter n enable, n=16-23 */
#define EIC_CONFIG2_SENSE_SHIFT(n)   (((n) - 16) << 2)                 /* Filter n input sense, n=16-23 */
#define EIC_CONFIG2_SENSE_MASK(n)    (7 << EIC_CONFIG2_SENSE_SHIFT(n))
#  define EIC_CONFIG2_SENSE_NONE(n)  (0 << EIC_CONFIG2_SENSE_SHIFT(n)) /* No detection */
#  define EIC_CONFIG2_SENSE_RISE(n)  (1 << EIC_CONFIG2_SENSE_SHIFT(n)) /* Rising edge detection */
#  define EIC_CONFIG2_SENSE_FALL(n)  (2 << EIC_CONFIG2_SENSE_SHIFT(n)) /* Falling edge detection */
#  define EIC_CONFIG2_SENSE_BOTH(n)  (3 << EIC_CONFIG2_SENSE_SHIFT(n)) /* Both edge detection */
#  define EIC_CONFIG2_SENSE_HIGH(n)  (4 << EIC_CONFIG2_SENSE_SHIFT(n)) /* High level detection */
#  define EIC_CONFIG2_SENSE_LOW(n)   (5 << EIC_CONFIG2_SENSE_SHIFT(n)) /* Low level detection */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* CONFIG_ARCH_FAMILY_SAMD21 */
#endif /* __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAMD_EIC_H */
