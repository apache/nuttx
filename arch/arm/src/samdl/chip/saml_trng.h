/********************************************************************************************
 * arch/arm/src/samdl/chip/saml_trng.h
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

#ifndef __ARCH_ARM_SRC_SAMDL_CHIP_SAML_TRNG_H
#define __ARCH_ARM_SRC_SAMDL_CHIP_SAML_TRNG_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#ifdef CONFIG_ARCH_FAMILY_SAML21

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/
/* TRNG register offsets *********************************************************************/

#define SAM_TRNG_CTRLA_OFFSET       0x0000  /* Control A register */
#define SAM_TRNG_EVCTRL_OFFSET      0x0004  /* Event control register */
#define SAM_TRNG_INTENCLR_OFFSET    0x0008  /* Interrupt enable clear register */
#define SAM_TRNG_INTENSET_OFFSET    0x0009  /* Interrupt enable set register */
#define SAM_TRNG_INTFLAG_OFFSET     0x000a  /* Interrupt flag and status clear register */
#define SAM_TRNG_DATA_OFFSET        0x0020  /* Output data register */

/* TRNG register addresses *******************************************************************/

#define SAM_TRNG_CTRLA              (SAM_TRNG_BASE+SAM_TRNG_CTRLA_OFFSET)
#define SAM_TRNG_EVCTRL             (SAM_TRNG_BASE+SAM_TRNG_EVCTRL_OFFSET)
#define SAM_TRNG_INTENCLR           (SAM_TRNG_BASE+SAM_TRNG_INTENCLR_OFFSET)
#define SAM_TRNG_INTENSET           (SAM_TRNG_BASE+SAM_TRNG_INTENSET_OFFSET)
#define SAM_TRNG_INTFLAG            (SAM_TRNG_BASE+SAM_TRNG_INTFLAG_OFFSET)
#define SAM_TRNG_DATA               (SAM_TRNG_BASE+SAM_TRNG_DATA_OFFSET)

/* TRNG register bit definitions *************************************************************/

/* Control register */

#define TRNG_CTRLA_ENABLE            (1 << 1)  /* Bit 1:  Enable */
#define TRNG_CTRLA_WEN               (1 << 6)  /* Bit 6:  Run in standby */

/* Event control register, Interrupt enable clear, interrupt enable set register, interrupt
 * flag status registers.
 */

#define TRNG_EVCTRL_DATARDY          (1 << 0)  /* Bit 0:  Data ready */

/* Data register (32-bit data) */

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
#endif /* __ARCH_ARM_SRC_SAMDL_CHIP_SAML_TRNG_H */
