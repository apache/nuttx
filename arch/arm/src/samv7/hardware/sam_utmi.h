/************************************************************************************************************
 * arch/arm/src/samv7/hardware/sam_utmi.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   SAMV7D3 Series Data Sheet
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
 ************************************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_UTMI_H
#define __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_UTMI_H

/************************************************************************************************************
 * Included Files
 ************************************************************************************************************/

#include <nuttx/config.h>
#include <arch/samv7/chip.h>

#include "hardware/sam_memorymap.h"

/************************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************************/
/* Register offsets *****************************************************************************************/

#define SAM_UTMI_OHCIICR_OFFSET            0x0010            /* OHCI Interrupt Configuration Register */
#define SAM_UTMI_CKTRIM_OFFSET             0x0030            /* UTMI Clock Trimming Register */

/* Register addresses ***************************************************************************************/

#define SAM_UTMI_OHCIICR                   (SAM_UTMI_BASE+SAM_UTMI_OHCIICR_OFFSET)
#define SAM_UTMI_CKTRIM                    (SAM_UTMI_BASE+SAM_UTMI_CKTRIM_OFFSET)

/* Register bit-field definitions ***************************************************************************/

/* OHCI Interrupt Configuration Register */

#define UTMI_OHCIICR_RES0                  (1 << 0)          /* Bit 0:  USB PORT0 Reset */
#define UTMI_OHCIICR_ARIE                  (1 << 4)          /* Bit 4:  OHCI Asynchronous Resume Interrupt Enable */
#define UTMI_OHCIICR_APPSTART              (0 << 5)          /* Bit 5:  Reserved, must be zero */
#define UTMI_OHCIICR_UDPPUDIS              (1 << 23)         /* Bit 23: USB Device Pull-up Disable */

/* UTMI Clock Trimming Register */

#define UTMI_CKTRIM_FREQ_SHIFT             (0)               /* Bits 0-1: UTMI Reference Clock Frequency */
#define UTMI_CKTRIM_FREQ_MASK              (3 << UTMI_CKTRIM_FREQ_SHIFT)
#  define UTMI_CKTRIM_FREQ_XTAL12          (0 << UTMI_CKTRIM_FREQ_SHIFT) /* 12 MHz reference clock */
#  define UTMI_CKTRIM_FREQ_XTAL16          (1 << UTMI_CKTRIM_FREQ_SHIFT) /* 16 MHz reference clock */

/************************************************************************************************************
 * Public Types
 ************************************************************************************************************/

/************************************************************************************************************
 * Public Data
 ************************************************************************************************************/

/************************************************************************************************************
 * Public Functions
 ************************************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAMV7_HARDWARE_SAM_UTMI_H */
