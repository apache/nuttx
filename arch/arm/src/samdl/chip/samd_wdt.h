/********************************************************************************************
 * arch/arm/src/samdl/chip/samd_wdt.h
 *
 *   Copyright (C) 2014-2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   "Atmel SAM D20J / SAM D20G / SAM D20E ARM-Based Microcontroller
 *   Datasheet", 42129J–SAM–12/2013
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

#ifndef __ARCH_ARM_SRC_SAMDL_CHIP_SAM_WDT_H
#define __ARCH_ARM_SRC_SAMDL_CHIP_SAM_WDT_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#ifdef CONFIG_ARCH_FAMILY_SAMD20

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/
/* WDT register offsets *********************************************************************/

#define SAM_WDT_CTRL_OFFSET        0x0000  /* Control register */
#define SAM_WDT_CONFIG_OFFSET      0x0001  /* Configuration register */
#define SAM_WDT_EWCTRL_OFFSET      0x0002  /* Early warning interrupt control register */
#define SAM_WDT_INTENCLR_OFFSET    0x0004  /* Interrupt enable clear register */
#define SAM_WDT_INTENSET_OFFSET    0x0005  /* Interrupt enable set register */
#define SAM_WDT_INTFLAG_OFFSET     0x0006  /* Interrupt flag and status clear register */
#define SAM_WDT_STATUS_OFFSET      0x0007  /* Status register */
#define SAM_WDT_CLEAR_OFFSET       0x0008  /* Clear register */

/* WDT register addresses *******************************************************************/

#define SAM_WDT_CTRL               (SAM_WDT_BASE+SAM_WDT_CTRL_OFFSET)
#define SAM_WDT_CONFIG             (SAM_WDT_BASE+SAM_WDT_CONFIG_OFFSET)
#define SAM_WDT_EWCTRL             (SAM_WDT_BASE+SAM_WDT_EWCTRL_OFFSET)
#define SAM_WDT_INTENCLR           (SAM_WDT_BASE+SAM_WDT_INTENCLR_OFFSET)
#define SAM_WDT_INTENSET           (SAM_WDT_BASE+SAM_WDT_INTENSET_OFFSET)
#define SAM_WDT_INTFLAG            (SAM_WDT_BASE+SAM_WDT_INTFLAG_OFFSET)
#define SAM_WDT_STATUS             (SAM_WDT_BASE+SAM_WDT_STATUS_OFFSET)
#define SAM_WDT_CLEAR              (SAM_WDT_BASE+SAM_WDT_CLEAR_OFFSET)

/* WDT register bit definitions *************************************************************/

/* Control register */

#define WDT_CTRL_ENABLE            (1 << 1)  /* Bit 1:  Enable */
#define WDT_CTRL_WEN               (1 << 2)  /* Bit 2:  Watchdog Timer Window Mode Enable */
#define WDT_CTRL_ALWAYSON          (1 << 7)  /* Bit 7:  Always-On */

/* Configuration register */

#define WDT_CONFIG_PER_SHIFT       (0)       /* Bits 0–3: Time-Out Period */
#define WDT_CONFIG_PER_MASK        (15 << WDT_CONFIG_PER_SHIFT)
#  define WDT_CONFIG_PER_8         (0 << WDT_CONFIG_PER_SHIFT) /* 8 clock cycles */
#  define WDT_CONFIG_PER_16        (1 << WDT_CONFIG_PER_SHIFT) /* 16 clock cycles */
#  define WDT_CONFIG_PER_32        (2 << WDT_CONFIG_PER_SHIFT) /* 32 clock cycles */
#  define WDT_CONFIG_PER_64        (3 << WDT_CONFIG_PER_SHIFT) /* 64 clock cycles */
#  define WDT_CONFIG_PER_128       (4 << WDT_CONFIG_PER_SHIFT) /* 128 clock cycles */
#  define WDT_CONFIG_PER_256       (5 << WDT_CONFIG_PER_SHIFT) /* 256 clocks cycles */
#  define WDT_CONFIG_PER_512       (6 << WDT_CONFIG_PER_SHIFT) /* 512 clocks cycles */
#  define WDT_CONFIG_PER_1K        (7 << WDT_CONFIG_PER_SHIFT) /* 1024 clock cycles */
#  define WDT_CONFIG_PER_2K        (8 << WDT_CONFIG_PER_SHIFT) /* 2048 clock cycles */
#  define WDT_CONFIG_PER_4K        (9 << WDT_CONFIG_PER_SHIFT) /* 4096 clock cycles */
#  define WDT_CONFIG_PER_8k        (10 << WDT_CONFIG_PER_SHIFT) /* 8192 clock cycles */
#  define WDT_CONFIG_PER_16K       (11 << WDT_CONFIG_PER_SHIFT) /* 16384 clock cycles */
#define WDT_CONFIG_WINDOW_SHIFT    (4)       /* Bits 4-7: Window Mode Time-Out Period */
#define WDT_CONFIG_WINDOW_MASK     (15 << WDT_CONFIG_WINDOW_SHIFT)
#  define WDT_CONFIG_WINDOW_8      (0 << WDT_CONFIG_WINDOW_SHIFT) /* 8 clock cycles */
#  define WDT_CONFIG_WINDOW_16     (1 << WDT_CONFIG_WINDOW_SHIFT) /* 16 clock cycles */
#  define WDT_CONFIG_WINDOW_32     (2 << WDT_CONFIG_WINDOW_SHIFT) /* 32 clock cycles */
#  define WDT_CONFIG_WINDOW_64     (3 << WDT_CONFIG_WINDOW_SHIFT) /* 64 clock cycles */
#  define WDT_CONFIG_WINDOW_128    (4 << WDT_CONFIG_WINDOW_SHIFT) /* 128 clock cycles */
#  define WDT_CONFIG_WINDOW_256    (5 << WDT_CONFIG_WINDOW_SHIFT) /* 256 clocks cycles */
#  define WDT_CONFIG_WINDOW_512    (6 << WDT_CONFIG_WINDOW_SHIFT) /* 512 clocks cycles */
#  define WDT_CONFIG_WINDOW_1K     (7 << WDT_CONFIG_WINDOW_SHIFT) /* 1024 clock cycles */
#  define WDT_CONFIG_WINDOW_2K     (8 << WDT_CONFIG_WINDOW_SHIFT) /* 2048 clock cycles */
#  define WDT_CONFIG_WINDOW_4K     (9 << WDT_CONFIG_WINDOW_SHIFT) /* 4096 clock cycles */
#  define WDT_CONFIG_WINDOW_8k     (10 << WDT_CONFIG_WINDOW_SHIFT) /* 8192 clock cycles */
#  define WDT_CONFIG_WINDOW_16K    (11 << WDT_CONFIG_WINDOW_SHIFT) /* 16384 clock cycles */

/* Early warning interrupt control register */

#define WDT_EWCTRL_EWOFFSET_SHIFT  (0)       /* Bits 0-3: Early warning interrupt time offset */
#define WDT_EWCTRL_EWOFFSET_MASK   (15 << WDT_EWCTRL_EWOFFSET_SHIFT)
#  define WDT_EWCTRL_EWOFFSET_8    (0 << WDT_EWCTRL_EWOFFSET_SHIFT) /* 8 clock cycles */
#  define WDT_EWCTRL_EWOFFSET_16   (1 << WDT_EWCTRL_EWOFFSET_SHIFT) /* 16 clock cycles */
#  define WDT_EWCTRL_EWOFFSET_32   (2 << WDT_EWCTRL_EWOFFSET_SHIFT) /* 32 clock cycles */
#  define WDT_EWCTRL_EWOFFSET_64   (3 << WDT_EWCTRL_EWOFFSET_SHIFT) /* 64 clock cycles */
#  define WDT_EWCTRL_EWOFFSET_128  (4 << WDT_EWCTRL_EWOFFSET_SHIFT) /* 128 clock cycles */
#  define WDT_EWCTRL_EWOFFSET_256  (5 << WDT_EWCTRL_EWOFFSET_SHIFT) /* 256 clocks cycles */
#  define WDT_EWCTRL_EWOFFSET_512  (6 << WDT_EWCTRL_EWOFFSET_SHIFT) /* 512 clocks cycles */
#  define WDT_EWCTRL_EWOFFSET_1K   (7 << WDT_EWCTRL_EWOFFSET_SHIFT) /* 1024 clock cycles */
#  define WDT_EWCTRL_EWOFFSET_2K   (8 << WDT_EWCTRL_EWOFFSET_SHIFT) /* 2048 clock cycles */
#  define WDT_EWCTRL_EWOFFSET_4K   (9 << WDT_EWCTRL_EWOFFSET_SHIFT) /* 4096 clock cycles */
#  define WDT_EWCTRL_EWOFFSET_8k   (10 << WDT_EWCTRL_EWOFFSET_SHIFT) /* 8192 clock cycles */
#  define WDT_EWCTRL_EWOFFSET_16K  (11 << WDT_EWCTRL_EWOFFSET_SHIFT) /* 16384 clock cycles */

/* Interrupt enable clear, interrupt enable set register, interrupt flag status and clear registers */

#define WDT_INT_EW                 (1 << 0)  /* Bit 0:  Early warning interrupt */
#define WDT_INT_All                (0x01)

/* Status register */

#define WDT_STATUS_SYNCBUSY        (1 << 7)  /* Bit 7:  Synchronization Busy */

/* Clear register */

#define WDT_CLEAR_CLEAR_SHIFT      (0)       /* Bits 0-7: Watchdog clear */
#define WDT_CLEAR_CLEAR_MASK       (0xff << WDT_CLEAR_CLEAR_SHIFT)
#  define WDT_CLEAR_CLEAR          (0xa5 << WDT_CLEAR_CLEAR_SHIFT)

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

/********************************************************************************************
 * Public Functions
 ********************************************************************************************/

#endif /* CONFIG_ARCH_FAMILY_SAMD20 */
#endif /* __ARCH_ARM_SRC_SAMDL_CHIP_SAM_WDT_H */
