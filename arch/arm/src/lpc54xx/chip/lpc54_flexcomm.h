/************************************************************************************
 * arch/arm/src/lpc54xx/chip/lpc54_flexcomm.h
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

#ifndef __ARCH_ARM_SRC_LPC54XX_CHIP_LPC54_FLEXCOMM_H
#define __ARCH_ARM_SRC_LPC54XX_CHIP_LPC54_FLEXCOMM_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip/lpc54_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

#define LPC54_FLEXCOMM_PSELID_OFFSET   0x0ff8  /* Peripheral Select /Flexcomm Interface ID */
#define LPC54_FLEXCOMM_PID_OFFSET      0x0ffc  /* Peripheral identification register */

/* Register addresses ***************************************************************/

#define LPC54_FLEXCOMM0_PSELID         (LPC54_FLEXCOMM0_BASE+LPC54_FLEXCOMM_PSELID_OFFSET)
#define LPC54_FLEXCOMM0_PID            (LPC54_FLEXCOMM0_BASE+LPC54_FLEXCOMM_PID_OFFSET)

#define LPC54_FLEXCOMM1_PSELID         (LPC54_FLEXCOMM1_BASE+LPC54_FLEXCOMM_PSELID_OFFSET)
#define LPC54_FLEXCOMM1_PID            (LPC54_FLEXCOMM1_BASE+LPC54_FLEXCOMM_PID_OFFSET)

#define LPC54_FLEXCOMM2_PSELID         (LPC54_FLEXCOMM2_BASE+LPC54_FLEXCOMM_PSELID_OFFSET)
#define LPC54_FLEXCOMM2_PID            (LPC54_FLEXCOMM2_BASE+LPC54_FLEXCOMM_PID_OFFSET)

#define LPC54_FLEXCOMM3_PSELID         (LPC54_FLEXCOMM3_BASE+LPC54_FLEXCOMM_PSELID_OFFSET)
#define LPC54_FLEXCOMM3_PID            (LPC54_FLEXCOMM3_BASE+LPC54_FLEXCOMM_PID_OFFSET)

#define LPC54_FLEXCOMM4_PSELID         (LPC54_FLEXCOMM4_BASE+LPC54_FLEXCOMM_PSELID_OFFSET)
#define LPC54_FLEXCOMM4_PID            (LPC54_FLEXCOMM4_BASE+LPC54_FLEXCOMM_PID_OFFSET)

#define LPC54_FLEXCOMM5_PSELID         (LPC54_FLEXCOMM5_BASE+LPC54_FLEXCOMM_PSELID_OFFSET)
#define LPC54_FLEXCOMM5_PID            (LPC54_FLEXCOMM5_BASE+LPC54_FLEXCOMM_PID_OFFSET)

#define LPC54_FLEXCOMM6_PSELID         (LPC54_FLEXCOMM6_BASE+LPC54_FLEXCOMM_PSELID_OFFSET)
#define LPC54_FLEXCOMM6_PID            (LPC54_FLEXCOMM6_BASE+LPC54_FLEXCOMM_PID_OFFSET)

#define LPC54_FLEXCOMM7_PSELID         (LPC54_FLEXCOMM7_BASE+LPC54_FLEXCOMM_PSELID_OFFSET)
#define LPC54_FLEXCOMM7_PID            (LPC54_FLEXCOMM7_BASE+LPC54_FLEXCOMM_PID_OFFSET)

#define LPC54_FLEXCOMM8_PSELID         (LPC54_FLEXCOMM8_BASE+LPC54_FLEXCOMM_PSELID_OFFSET)
#define LPC54_FLEXCOMM8_PID            (LPC54_FLEXCOMM8_BASE+LPC54_FLEXCOMM_PID_OFFSET)

#define LPC54_FLEXCOMM9_PSELID         (LPC54_FLEXCOMM9_BASE+LPC54_FLEXCOMM_PSELID_OFFSET)
#define LPC54_FLEXCOMM9_PID            (LPC54_FLEXCOMM9_BASE+LPC54_FLEXCOMM_PID_OFFSET)

/* Register bit definitions *********************************************************/

/* Peripheral Select /Flexcomm Interface ID */

#define FLEXCOMM_PSELID_PERSEL_SHIFT   (0)       /* Bits 0-2: Peripheral Select */
#define FLEXCOMM_PSELID_PERSEL_MASK    (7 << FLEXCOMM_PSELID_PERSEL_SHIFT)
#  define FLEXCOMM_PSELID_PERSEL_NONE  (0 << FLEXCOMM_PSELID_PERSEL_SHIFT) /* No peripheral selected */
#  define FLEXCOMM_PSELID_PERSEL_USART (1 << FLEXCOMM_PSELID_PERSEL_SHIFT) /* USART function selected */
#  define FLEXCOMM_PSELID_PERSEL_SPI   (2 << FLEXCOMM_PSELID_PERSEL_SHIFT) /* SPI function selected */
#  define FLEXCOMM_PSELID_PERSEL_I2C   (3 << FLEXCOMM_PSELID_PERSEL_SHIFT) /* I2C function selected */
#  define FLEXCOMM_PSELID_PERSEL_I2STX (4 << FLEXCOMM_PSELID_PERSEL_SHIFT) /* I2S transmit function */
#  define FLEXCOMM_PSELID_PERSEL_I2SRX (5 << FLEXCOMM_PSELID_PERSEL_SHIFT) /* I2S receive function */
#define FLEXCOMM_PSELID_LOCK           (1 << 3)  /* Bit 3:  Lock the peripheral select */
#define FLEXCOMM_PSELID_USARTPRESENT   (1 << 4)  /* Bit 4:  USART present indicator */
#define FLEXCOMM_PSELID_SPIPRESENT     (1 << 5)  /* Bit 5:  SPI present indicator */
#define FLEXCOMM_PSELID_I2CPRESENT     (1 << 6)  /* Bit 6:  I2C present indicator */
#define FLEXCOMM_PSELID_I2SPRESENT     (1 << 7)  /* Bit 7:  I2S present indicator */
#define FLEXCOMM_PSELID_ID_SHIFT       (12)      /* Bits 12-31: Flexcomm Interface ID */
#define FLEXCOMM_PSELID_ID_MASK        (0xfffff << FLEXCOMM_PSELID_ID_SHIFT)

/* Peripheral identification register */

#define FLEXCOMM_PID_MINOR_SHIFT       (8)       /* Bits 8-11:  Minor revision number */
#define FLEXCOMM_PID_MINOR_MASK        (15 << FLEXCOMM_PID_MINOR_SHIFT)
#define FLEXCOMM_PID_MAJOR_SHIFT       (12)      /* Bits 12-15: Major revision number */
#define FLEXCOMM_PID_MAJOR_MASK        (15 << FLEXCOMM_PID_MAJOR_SHIFT)
#define FLEXCOMM_PID_ID_SHIFT          (16)      /* Bits 15-31: Module ID for selected function */
#define FLEXCOMM_PID_ID_MASK           (0xffff << FLEXCOMM_PID_ID_SHIFT)

#endif /* __ARCH_ARM_SRC_LPC54XX_CHIP_LPC54_FLEXCOMM_H */
