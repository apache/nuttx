/********************************************************************************************
 * arch/arm/src/samd2l2/hardware/saml_dac.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAML_DAC_H
#define __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAML_DAC_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#ifdef CONFIG_ARCH_FAMILY_SAML21

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/
/* DAC register offsets ********************************************************************/

#define SAM_DAC_CTRLA_OFFSET       0x0000 /* Control A Register */
#define SAM_DAC_CTRLB_OFFSET       0x0001 /* Control B Register */
#define SAM_DAC_EVCTRL_OFFSET      0x0002 /* Event Control Register */
#define SAM_DAC_INTENCLR_OFFSET    0x0004 /* Interrupt Enable Clear Register */
#define SAM_DAC_INTENSET_OFFSET    0x0005 /* Interrupt Enable Set Register */
#define SAM_DAC_INTFLAG_OFFSET     0x0006 /* Interrupt Flag Status and Clear Register */
#define SAM_DAC_STATUS_OFFSET      0x0007 /* Status Register */
#define SAM_DAC_SYNCBUSY_OFFSET    0x0008 /* Synchronization Busy Register */
#define SAM_DAC_DACCTRL0_OFFSET    0x000c /* DAC0 Control Register */
#define SAM_DAC_DACCTRL1_OFFSET    0x000e /* DAC1 Control Register */
#define SAM_DAC_DATA0_OFFSET       0x0010 /* Data DAC0 Register */
#define SAM_DAC_DATA1_OFFSET       0x0012 /* Data DAC1 Register */
#define SAM_DAC_DATABUF0_OFFSET    0x0014 /* Data Buffer DAC0 Register */
#define SAM_DAC_DATABUF1_OFFSET    0x0016 /* Data Buffer DAC1 Register */
#define SAM_DAC_DBCTRL_OFFSET      0x0017 /* Debug Control Register */

/* DAC register addresses ******************************************************************/

#define SAM_DAC_CTRLA              (SAM_DAC_BASE+SAM_DAC_CTRLA_OFFSET)
#define SAM_DAC_CTRLB              (SAM_DAC_BASE+SAM_DAC_CTRLB_OFFSET)
#define SAM_DAC_EVCTRL             (SAM_DAC_BASE+SAM_DAC_EVCTRL_OFFSET)
#define SAM_DAC_INTENCLR           (SAM_DAC_BASE+SAM_DAC_INTENCLR_OFFSET)
#define SAM_DAC_INTENSET           (SAM_DAC_BASE+SAM_DAC_INTENSET_OFFSET)
#define SAM_DAC_INTFLAG            (SAM_DAC_BASE+SAM_DAC_INTFLAG_OFFSET)
#define SAM_DAC_STATUS             (SAM_DAC_BASE+SAM_DAC_STATUS_OFFSET)
#define SAM_DAC_SYNCBUSY           (SAM_DAC_BASE+SAM_DAC_SYNCBUSY_OFFSET)
#define SAM_DAC_DACCTRL0           (SAM_DAC_BASE+SAM_DAC_DACCTRL0_OFFSET)
#define SAM_DAC_DACCTRL1           (SAM_DAC_BASE+SAM_DAC_DACCTRL1_OFFSET)
#define SAM_DAC_DATA0              (SAM_DAC_BASE+SAM_DAC_DATA0_OFFSET)
#define SAM_DAC_DATA1              (SAM_DAC_BASE+SAM_DAC_DATA1_OFFSET)
#define SAM_DAC_DATABUF0           (SAM_DAC_BASE+SAM_DAC_DATABUF0_OFFSET)
#define SAM_DAC_DATABUF1           (SAM_DAC_BASE+SAM_DAC_DATABUF1_OFFSET)
#define SAM_DAC_DBCTRL              (SAM_DAC_BASE+SAM_DAC_DBCTRL_OFFSET)

/* DAC register bit definitions ************************************************************/

/* Control A Register */

#define DAC_CTRLA_SWRTS            (1 << 0)  /* Bit 0:  Software reset */
#define DAC_CTRLA_ENABLE           (1 << 1)  /* Bit 1:  Enable DAC controller */

/* Control B Register */

#define DAC_CTRLB_DIFF             (1 << 0)  /* Bit 0:  Differential mode enable */
#define DAC_CTRLB_REFSEL_SHIFT     (1)       /* Bit 1-2: Reference selection */
#define DAC_CTRLB_REFSEL_MASK      (3 << DAC_CTRLB_REFSEL_SHIFT)
#  define DAC_CTRLB_REFSEL_VREFAU  (0 << DAC_CTRLB_REFSEL_SHIFT) /* Unbuffered external voltage reference */
#  define DAC_CTRLB_REFSEL_VDDANA  (1 << DAC_CTRLB_REFSEL_SHIFT) /* Voltage supply */
#  define DAC_CTRLB_REFSEL_VREFAB  (2 << DAC_CTRLB_REFSEL_SHIFT) /* Buffered external voltage reference */
#  define DAC_CTRLB_REFSEL_INTREF  (3 << DAC_CTRLB_REFSEL_SHIFT) /* Internal bandgap reference */

/* Event Control Register */

#define DAC_EVCTRL_STARTEI0        (1 << 0)  /* Bit 0:  Start conversion event input DAC0 */
#define DAC_EVCTRL_STARTEI1        (1 << 1)  /* Bit 1:  Start conversion event input DAC1 */
#define DAC_EVCTRL_EMTPYEO0        (1 << 2)  /* Bit 2:  Data buffer empty event output DAC0 */
#define DAC_EVCTRL_EMTPYEO1        (1 << 3)  /* Bit 3:  Data buffer empty event output DAC1 */
#define DAC_EVCTRL_INVEI0          (1 << 4)  /* Bit 4:  Enable inversion of DAC0 input event */
#define DAC_EVCTRL_INVEI1          (1 << 5)  /* Bit 5:  Enable inversion of DAC1 input event */

/* Common bit definitions for Interrupt Enable Clear Register, Interrupt Enable Set
 * Register, and Interrupt Flag Status and Clear Register
 */

#define DAC_INT_UNDERRUN0          (1 << 0)  /* Bit 0:  Underrun interrupt for DAC2 */
#define DAC_INT_UNDERRUN1          (1 << 1)  /* Bit 1:  Underrun interrupt for DAC1 */
#define DAC_INT_EMPTY0             (1 << 2)  /* Bit 2:  Data buffer 0 empty interrupt */
#define DAC_INT_EMPTY1             (1 << 3)  /* Bit 3:  Data buffer 1 empty interrupt */
#define DAC_INT_ALL                0x0f

/* Status Register */

#define DAC_STATUS_READY0          (1 << 0)  /* Bit 0:  DAC0 startup ready */
#define DAC_STATUS_READY1          (1 << 1)  /* Bit 1:  DAC1 startup ready */
#define DAC_STATUS_EOC0            (1 << 2)  /* Bit 2:  DAC0 end of conversion */
#define DAC_STATUS_EOC1            (1 << 3)  /* Bit 3:  DAC1 end of conversion */

/* Synchronization Busy Register */

#define DAC_SYNCBUSY_SWRST         (1 << 0)  /* Bit 0:  Software reset */
#define DAC_SYNCBUSY_ENABLE        (1 << 1)  /* Bit 1:  DAC enable status */
#define DAC_SYNCBUSY_DATA0         (1 << 2)  /* Bit 2:  Data DAC0 */
#define DAC_SYNCBUSY_DATA1         (1 << 3)  /* Bit 3:  Data DAC1 */
#define DAC_SYNCBUSY_DATABUF0      (1 << 4)  /* Bit 4:  Data buffer DAC0 */
#define DAC_SYNCBUSY_DATABUF1      (1 << 5)  /* Bit 5:  Data buffer DAC1 */

/* DAC0/1 Control Register */

#define DAC_DACCTRL_LEFTADJ        (1 << 0)  /* Bit 0:  Left adjusted data */
#define DAC_DACCTRL_ENABLE         (1 << 1)  /* Bit 1:  Enable DAC */
#define DAC_DACCTRL_CCTRL_SHIFT    (2)       /* Bit 2-3: Current control */
#define DAC_DACCTRL_CCTRL_MASK     (3 << DAC_DACCTRL_CCTRL_SHIFT)
#  define DAC_DACCTRL_CCTRL_CC100K (0 << DAC_DACCTRL_CCTRL_SHIFT) /* GCLK_DAC <= 1.2MHz */
#  define DAC_DACCTRL_CCTRL_CC1M   (1 << DAC_DACCTRL_CCTRL_SHIFT) /* 1.2MHz < GCLK_DAC <= 6MHz */
#  define DAC_DACCTRL_CCTRL_CC2M   (2 << DAC_DACCTRL_CCTRL_SHIFT) /* 6MHz < GCLK_DAC <= 12MHz */
#define DAC_DACCTRL_RUNSTDBY       (1 << 6)  /* Bit 6:  Run in standby */
#define DAC_DACCTRL_DITHER         (1 << 7)  /* Bit 7:  Dithering mode */
#define DAC_DACCTRL_REFRESH_SHIFT  (8)       /* Bit 8-11: Refresh period */
#define DAC_DACCTRL_REFRESH_MASK   (15 << DAC_DACCTRL_REFRESH_SHIFT)
#  define DAC_DACCTRL_REFRESH(n)   ((uin16_t)(n) << DAC_DACCTRL_REFRESH_SHIFT)

/* Data DAC0/1 Register (16-bit data) */
/* Data Buffer DAC0/1 Register (16-bit data) */

/* Debug Control Register */

#define DAC_DBCTRL_DBGRUN          (1 << 0)  /* Bit 0:  Debug run */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* CONFIG_ARCH_FAMILY_SAML21 */
#endif /* __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAML_DAC_H */
