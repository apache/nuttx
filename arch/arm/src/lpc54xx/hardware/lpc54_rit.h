/************************************************************************************
 * arch/arm/src/lpc54xx/lpc54_rit.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_RIT_H
#define __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_RIT_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "hardware/lpc54_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

#define LPC54_RIT_COMPVAL_OFFSET  0x0000  /* LS 48-bit Compare register */
#define LPC54_RIT_MASK_OFFSET     0x0004  /* LS 48-bit Mask register */
#define LPC54_RIT_CTRL_OFFSET     0x0008  /* Control register */
#define LPC54_RIT_COUNTER_OFFSET  0x000c  /* LS 48-bit counter */
#define LPC54_RIT_COMPVALH_OFFSET 0x0010  /* MS 48-bit Compare register */
#define LPC54_RIT_MASKH_OFFSET    0x0014  /* MS 48-bit Mask register */
#define LPC54_RIT_COUNTERH_OFFSET 0x001c  /* MS 48-bit counter */

/* Register addresses ***************************************************************/

#define LPC54_RIT_COMPVAL         (LPC54_RIT_BASE+LPC54_RIT_COMPVAL_OFFSET)
#define LPC54_RIT_MASK            (LPC54_RIT_BASE+LPC54_RIT_MASK_OFFSET)
#define LPC54_RIT_CTRL            (LPC54_RIT_BASE+LPC54_RIT_CTRL_OFFSET)
#define LPC54_RIT_COUNTER         (LPC54_RIT_BASE+LPC54_RIT_COUNTER_OFFSET)
#define LPC54_RIT_COMPVALH        (LPC54_RIT_BASE+LPC54_RIT_COMPVALH_OFFSET)
#define LPC54_RIT_MASKH           (LPC54_RIT_BASE+LPC54_RIT_MASKH_OFFSET)
#define LPC54_RIT_COUNTERH        (LPC54_RIT_BASE+LPC54_RIT_COUNTERH_OFFSET)

/* Register bit definitions *********************************************************/
/* LS Compare register (Bits 0-31: value compared to the counter) */
/* MS Compare register (Bits 21-47: value compared to the counter) */

/* LS Mask register (Bits 0-31: 32-bit mask value) */
/* MS Mask register (Bits 32-47: 16-bit mask value) */

/* Control register */

#define RIT_CTRL_INT             (1 << 0)  /* Bit 0: Interrupt flag */
#define RIT_CTRL_ENCLR           (1 << 1)  /* Bit 1: Timer enable clear */
#define RIT_CTRL_ENBR            (1 << 2)  /* Bit 2: Timer enable for debug */
#define RIT_CTRL_EN              (1 << 3)  /* Bit 3: Timer enable */
                                           /* Bits 4-31: Reserved */
/* LS 48-bit counter (Bits 0-31: 48-bit up counter) */
/* MS 48-bit counter (Bits 32-47: 48-bit up counter) */

#endif /* __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_RIT_H */
