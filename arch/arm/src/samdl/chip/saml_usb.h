/********************************************************************************************
 * arch/arm/src/samdl/chip/saml_usb.h
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

#ifndef __ARCH_ARM_SRC_SAMDL_CHIP_SAML_USB_H
#define __ARCH_ARM_SRC_SAMDL_CHIP_SAML_USB_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#ifdef CONFIG_ARCH_FAMILY_SAML21

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/
/* USB register offsets ********************************************************************/

/* Common USB Device/Host Register Offsets */

#define SAM_USB_CTRLA_OFFSET       0x0000 /* Control A Register */
#define SAM_USB_SYNCBUSY_OFFSET    0x0002 /* Synchronization Busy Register */
#define SAM_USB_QOSCTRL_OFFSET     0x0003 /* QOS Control Register */
#define SAM_USB_FSMSTATUS_OFFSET   0x000d /* Finite State Machine Register */
#define SAM_USB_DESCADD_OFFSET     0x0024 /* Descriptor Address Register */
#define SAM_USB_PADCAL_OFFSET      0x0028 /* Pad Calibration Register */

/* USB Device Register Offsets */

/* USB Host Register Offsets */

/* USB register addresses ******************************************************************/

/* Common USB Device/Host Register Addresses */

#define SAM_USB_CTRLA              (SAM_USB_BASE+SAM_USB_CTRLA_OFFSET)
#define SAM_USB_SYNCBUSY           (SAM_USB_BASE+SAM_USB_SYNCBUSY_OFFSET)
#define SAM_USB_QOSCTRL            (SAM_USB_BASE+SAM_USB_QOSCTRL_OFFSET)
#define SAM_USB_FSMSTATUS          (SAM_USB_BASE+SAM_USB_FSMSTATUS_OFFSET)
#define SAM_USB_DESCADD            (SAM_USB_BASE+SAM_USB_DESCADD_OFFSET)
#define SAM_USB_PADCAL             (SAM_USB_BASE+SAM_USB_PADCAL_OFFSET)

/* USB Device Register Addresses */

/* USB Host Register Addresses */

/* USB register bit definitions ************************************************************/

/* Common USB Device/Host Register Offsets */

/* Control A Register */

#define USB_CTRLA_SWRST            (1 << 0)  /* Bit 0:  Software reset */
#define USB_CTRLA_ENABLE           (1 << 1)  /* Bit 1:  Enable */
#define USB_CTRLA_RUNSTBY          (1 << 2)  /* Bit 2:  Run in standby */
#define USB_CTRLA_MODE             (1 << 7)  /* Bit 7:  Operating mode */
#  define USB_CTRLA_MODE_DEVICE    (0)       /*         0 = USB device mode */
#   define USB_CTRLA_MODE          (1 << 7)  /*         1 = USB hose mode */

/* Synchronization Busy Register */

#define USB_SYNCBUSY_SWRST         (1 << 0)  /* Bit 0:  Software reset status */
#define USB_SYNCBUSY_ENABLE        (1 << 1)  /* Bit 1:  Enable status */

/* QOS Control Register */

#define USB_QOSCTRL_CQOS_SHIFT     (0)       /* Bit 0-1: Data quality of service */
#define USB_QOSCTRL_CQOS_MASK      (3 << USB_QOSCTRL_CQOS_SHIFT)
#  define USB_QOSCTRL_CQOS(n)      ((uint8_t)(n) << USB_QOSCTRL_CQOS_SHIFT)
#define USB_QOSCTRL_DQOS_SHIFT     (2)       /* Bit 2-3: Configuration qualilty of service */
#define USB_QOSCTRL_DQOS_MASK      (3 << USB_QOSCTRL_DQOS_SHIFT)
#  define USB_QOSCTRL_DQOS(n)      ((uint8_t)(n) << USB_QOSCTRL_DQOS_SHIFT)

/* Finite State Machine Register */

#define USB_FSMSTATUS_SHIFT        (0)       /* Bits 0-6:  Finite state machine status */
#define USB_FSMSTATUS_MASK         (0x7f  << USB_FSMSTATUS_SHIFT)
#  define USB_FSMSTATUS_OFF        (0x01  << USB_FSMSTATUS_SHIFT) /* Powered-off, disconnect, disabled state */
#  define USB_FSMSTATUS_ON         (0x02  << USB_FSMSTATUS_SHIFT) /* Idle and Active states */
#  define USB_FSMSTATUS_SUSPEND    (0x04  << USB_FSMSTATUS_SHIFT)
#  define USB_FSMSTATUS_SLEEP      (0x08  << USB_FSMSTATUS_SHIFT)
#  define USB_FSMSTATUS_DNRESUME   (0x10  << USB_FSMSTATUS_SHIFT) /* Down stream resume */
#  define USB_FSMSTATUS_UPRESUME   (0x20  << USB_FSMSTATUS_SHIFT) /* Up stream resume */
#  define USB_FSMSTATUS_RESET      (0x40  << USB_FSMSTATUS_SHIFT) /* USB lines reset */

/* Descriptor Address Register (32-bit address) */

/* Pad Calibration Register */

#define USB_PADCAL_TRANSP_SHIFT    (0)       /* Bit 0-4: Trimmable output driver impedance P */
#define USB_PADCAL_TRANSP_MASK     (31 << USB_PADCAL_TRANSP_SHIFT)
#  define USB_PADCAL_TRANSP(n)     ((uint16_t)(n) << USB_PADCAL_TRANSP_SHIFT)
#define USB_PADCAL_TRANSN_SHIFT    (6)       /* Bit 7-9: Trimmable output driver impedance N */
#define USB_PADCAL_TRANSN_MASK     (31 << USB_PADCAL_TRANSN_SHIFT)
#  define USB_PADCAL_TRANSN(n)     ((uint16_t)(n) << USB_PADCAL_TRANSN_SHIFT)
#define USB_PADCAL_TRIM_SHIFT      (12)      /* Bit 12-14: Trim bits for DP/DM */
#define USB_PADCAL_TRIM_MASK       (7 << USB_PADCAL_TRIM_SHIFT)
#  define USB_PADCAL_TRIM(n)       ((uint16_t)(n) << USB_PADCAL_TRIM_SHIFT)

/* USB Device Register Offsets */

/* USB Host Register Offsets */

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
#endif /* __ARCH_ARM_SRC_SAMDL_CHIP_SAML_USB_H */
