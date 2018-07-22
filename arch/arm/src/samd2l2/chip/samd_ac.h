/********************************************************************************************
 * arch/arm/src/samd2l2/chip/samd_ac.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Matt Thompson <matt@extent3d.com>
 *
 * References:
 *   "Atmel SAM L21E / SAM L21G / SAM L21J Smart ARM-Based Microcontroller
 *   Datasheet", Atmel-42385C-SAML21_Datasheet_Preliminary-03/20/15
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

#ifndef __ARCH_ARM_SRC_SAMD2L2_CHIP_SAMD_AC_H
#define __ARCH_ARM_SRC_SAMD2L2_CHIP_SAMD_AC_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#ifdef CONFIG_ARCH_FAMILY_SAMD21

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/
/* AC register offsets **********************************************************************/

#define SAM_AC_CTRLA_OFFSET          0x0000 /* Control A Register */
#define SAM_AC_CTRLB_OFFSET          0x0001 /* Control B Register */
#define SAM_AC_EVCTRL_OFFSET         0x0002 /* Event Control Register */
#define SAM_AC_INTENCLR_OFFSET       0x0004 /* Interrupt Enable Clear Register */
#define SAM_AC_INTENSET_OFFSET       0x0005 /* Interrupt Enable Set Register */
#define SAM_AC_INTFLAG_OFFSET        0x0006 /* Interrupt Flag Status and Clear Register */
#define SAM_AC_STATUSA_OFFSET        0x0008 /* Status A Register */
#define SAM_AC_STATUSB_OFFSET        0x0009 /* Status B Register */
#define SAM_AC_STATUSC_OFFSET        0x000A /* Status C Register */
#define SAM_AC_WINCTRL_OFFSET        0x000C /* Window Control Register */
#define SAM_AC_COMPCTRL0_OFFSET      0x0010 /* Comparator 0 Control Register */
#define SAM_AC_COMPCTRL1_OFFSET      0x0014 /* Comparator 1 Control Register */
#define SAM_AC_SCALER0_OFFSET        0x0020 /* Scaler 0 Register */
#define SAM_AC_SCALER1_OFFSET        0x0021 /* Scaler 1 Register */

/* AC register addresses *******************************************************************/

#define SAM_AC_CTRLA                 (SAM_AC_BASE+SAM_AC_CTRLA_OFFSET)
#define SAM_AC_CTRLB                 (SAM_AC_BASE+SAM_AC_CTRLB_OFFSET)
#define SAM_AC_EVCTRL                (SAM_AC_BASE+SAM_AC_EVCTRL_OFFSET)
#define SAM_AC_INTENCLR              (SAM_AC_BASE+SAM_AC_INTENCLR_OFFSET)
#define SAM_AC_INTENSET              (SAM_AC_BASE+SAM_AC_INTENSET_OFFSET)
#define SAM_AC_INTFLAG               (SAM_AC_BASE+SAM_AC_INTFLAG_OFFSET)
#define SAM_AC_STATUSA               (SAM_AC_BASE+SAM_AC_STATUSA_OFFSET)
#define SAM_AC_STATUSB               (SAM_AC_BASE+SAM_AC_STATUSB_OFFSET)
#define SAM_AC_STATUSC               (SAM_AC_BASE+SAM_AC_STATUSC_OFFSET)
#define SAM_AC_WINCTRL               (SAM_AC_BASE+SAM_AC_WINCTRL_OFFSET)
#define SAM_AC_COMPCTRL0             (SAM_AC_BASE+SAM_AC_COMPCTRL0_OFFSET)
#define SAM_AC_COMPCTRL1             (SAM_AC_BASE+SAM_AC_COMPCTRL1_OFFSET)
#define SAM_AC_SCALER0               (SAM_AC_BASE+SAM_AC_SCALER0_OFFSET)
#define SAM_AC_SCALER1               (SAM_AC_BASE+SAM_AC_SCALER1_OFFSET)

/* AC register bit definitions ************************************************************/

/* Control A Register */

#define AC_CTRLA_SWRTS               (1 << 0)  /* Bit 0:  Software reset */
#define AC_CTRLA_ENABLE              (1 << 1)  /* Bit 1:  Enable AC */
#define AC_CTRLA_RUNSTDBY            (1 << 2)  /* Bit 2:  Run in standby */
#define AC_CTRLA_LPMUX               (1 << 7)  /* Bit 7:  Low-Power Mux */

/* Control B Register */

#define AC_CTRLB_START0              (1 << 0)  /* Bit 0:  Comparator 0 start */
#define AC_CTRLB_START1              (1 << 1)  /* Bit 1:  Comparator 1 start */

/* Event Control Register */

#define AC_EVCTRL_COMPEO0            (1 << 0)  /* Bit 0:  Comparator 0 Event Output enable */
#define AC_EVCTRL_COMPEO1            (1 << 1)  /* Bit 1:  Comparator 1 Event Output enable */
#define AC_EVCTRL_WINEO0             (1 << 4)  /* Bit 4:  Window 0 Event Output enable */
#define AC_EVCTRL_COMPEI0            (1 << 8)  /* Bit 8:  Comparator 0 Event Input enable */
#define AC_EVCTRL_COMPEI1            (1 << 9)  /* Bit 9:  Comparator 1 Event Input enable */

/* Common bit definitions for Interrupt Enable Clear Register, Interrupt Enable Set
 * Register, and Interrupt Flag Status and Clear Register
 */

#define AC_INT_COMP0                 (1 << 0)  /* Bit 0:  Comparator 0 */
#define AC_INT_COMP1                 (1 << 1)  /* Bit 1:  Comparator 1 */
#define AC_INT_WIN0                  (1 << 4)  /* Bit 4:  Window 0 */
#define AC_INT_ALL                   0x13

/* Status A Register */

#define AC_STATUSA_STATE0            (1 << 0)  /* Bit 0:  State 0 - Output state of comparator 0 */
#define AC_STATUSA_STATE1            (1 << 1)  /* Bit 1:  State 1 - Output state of comparator 1 */
#define AC_STATUSA_WSTATE_SHIFT      (4)
#define AC_STATUSA_WSTATE_MASK       (3 << AC_STATUSA_WSTATE_SHIFT)
#  define AC_STATUSA_WSTATE_ABOVE    (0 << AC_STATUSA_WSTATE_SHIFT)
#  define AC_STATUSA_WSTATE_INSIDE   (1 << AC_STATUSA_WSTATE_SHIFT)
#  define AC_STATUSA_WSTATE_BELOW    (2 << AC_STATUSA_WSTATE_SHIFT)

/* Status B Register */

#define AC_STATUSB_READY0            (1 << 0)  /* Bit 0:  Ready 0 - Comparator 0 ready status */
#define AC_STATUSB_READY1            (1 << 1)  /* Bit 1:  Ready 1 - Comparator 1 ready status */
#define AC_STATUSB_SYNCBUSY          (1 << 7)  /* Bit 7:  Synchronoziation ready */

/* Status C Register */

/* Window Control Register */

#define AC_WINCTRL_WEN0              (1 << 0)  /* Bit 0:  Window enable (both comparators) */
#define AC_WINCTRL_WINTSEL_SHIFT     (1)
#define AC_WINCTRL_WINTSEL_MASK      (3 << AC_WINCTRL_WINTSEL_SHIFT)
#  define AC_WINCTRL_WINTSEL_ABOVE   (0 << AC_WINCTRL_WINTSEL_SHIFT)
#  define AC_WINCTRL_WINTSEL_INSIDE  (1 << AC_WINCTRL_WINTSEL_SHIFT)
#  define AC_WINCTRL_WINTSEL_BELOW   (2 << AC_WINCTRL_WINTSEL_SHIFT)
#  define AC_WINCTRL_WINTSEL_OUTSIDE (3 << AC_WINCTRL_WINTSEL_SHIFT)

/* Comparator Control Registers */

#define AC_COMPCTRL_ENABLE           (1 << 0)  /* Bit 0:  Enable Comparator */
#define AC_COMPCTRL_SINGLE           (1 << 1)  /* Bit 1:  Single Shot Mode */
#define AC_COMPCTRL_SPEED_SHIFT      (2)
#define AC_COMPCTRL_SPEED_MASK       (3 << AC_COMPCTRL_SPEED_SHIFT)
#  define AC_COMPCTRL_SPEED_LOW      (0 << AC_COMPCTRL_SPEED_SHIFT)
#  define AC_COMPCTRL_SPEED_HIGH     (1 << AC_COMPCTRL_SPEED_SHIFT)
#define AC_COMPCTRL_INTSEL_SHIFT     (5)
#define AC_COMPCTRL_INTSEL_MASK      (3 << AC_COMPCTRL_INTSEL_SHIFT)
#  define AC_COMPCTRL_INTSEL_TOGGLE  (0 << AC_COMPCTRL_INTSEL_SHIFT)
#  define AC_COMPCTRL_INTSEL_RISING  (1 << AC_COMPCTRL_INTSEL_SHIFT)
#  define AC_COMPCTRL_INTSEL_FALLING (2 << AC_COMPCTRL_INTSEL_SHIFT)
#  define AC_COMPCTRL_INTSEL_EOC     (3 << AC_COMPCTRL_INTSEL_SHIFT)
#define AC_COMPCTRL_MUXNEG_SHIFT     (8)
#define AC_COMPCTRL_MUXNEG_MASK      (7 << AC_COMPCTRL_MUXNEG_SHIFT)
#  define AC_COMPCTRL_MUXNEG_PIN0    (0 << AC_COMPCTRL_MUXNEG_SHIFT)
#  define AC_COMPCTRL_MUXNEG_PIN1    (1 << AC_COMPCTRL_MUXNEG_SHIFT)
#  define AC_COMPCTRL_MUXNEG_PIN2    (2 << AC_COMPCTRL_MUXNEG_SHIFT)
#  define AC_COMPCTRL_MUXNEG_PIN3    (3 << AC_COMPCTRL_MUXNEG_SHIFT)
#  define AC_COMPCTRL_MUXNEG_GND     (4 << AC_COMPCTRL_MUXNEG_SHIFT)
#  define AC_COMPCTRL_MUXNEG_VSCALE  (5 << AC_COMPCTRL_MUXNEG_SHIFT)
#  define AC_COMPCTRL_MUXNEG_BANDGAP (6 << AC_COMPCTRL_MUXNEG_SHIFT)
#  define AC_COMPCTRL_MUXNEG_DAC     (7 << AC_COMPCTRL_MUXNEG_SHIFT)
#define AC_COMPCTRL_MUXPOS_SHIFT     (12)
#define AC_COMPCTRL_MUXPOS_MASK      (3 << AC_COMPCTRL_MUXPOS_SHIFT)
#  define AC_COMPCTRL_MUXPOS_PIN0    (0 << AC_COMPCTRL_MUXPOS_SHIFT)
#  define AC_COMPCTRL_MUXPOS_PIN1    (1 << AC_COMPCTRL_MUXPOS_SHIFT)
#  define AC_COMPCTRL_MUXPOS_PIN2    (2 << AC_COMPCTRL_MUXPOS_SHIFT)
#  define AC_COMPCTRL_MUXPOS_PIN3    (3 << AC_COMPCTRL_MUXPOS_SHIFT)
#define AC_COMPCTRL_SWAP             (1 << 13)  /* Bit 13:  Swap Inputs and Invert */
#define AC_COMPCTRL_OUT_SHIFT        (16)
#define AC_COMPCTRL_OUT_MASK         (3 << AC_COMPCTRL_OUT_SHIFT)
#  define AC_COMPCTRL_OUT_OFF        (0 << AC_COMPCTRL_OUT_SHIFT)
#  define AC_COMPCTRL_OUT_ASYNC      (1 << AC_COMPCTRL_OUT_SHIFT)
#  define AC_COMPCTRL_OUT_SYNC       (2 << AC_COMPCTRL_OUT_SHIFT)
#define AC_COMPCTRL_HYST             (1 << 19)  /* Bit 19:  Hysteresis Enable */
#define AC_COMPCTRL_FLEN_SHIFT       (24)
#define AC_COMPCTRL_FLEN_MASK        (7 << AC_COMPCTRL_FLEN_SHIFT)
#  define AC_COMPCTRL_FLEN_OFF       (0 << AC_COMPCTRL_FLEN_SHIFT)
#  define AC_COMPCTRL_FLEN_MAJ3      (1 << AC_COMPCTRL_FLEN_SHIFT)
#  define AC_COMPCTRL_FLEN_MAJ5      (2 << AC_COMPCTRL_FLEN_SHIFT)

/* Scaler Registers */

#define AC_COMPCTRL_SCALER_MASK      (0x3f)

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
#endif /* __ARCH_ARM_SRC_SAMD2L2_CHIP_SAMD_AC_H */
