/****************************************************************************
 * arch/arm/src/samd2l2/hardware/saml_wdt.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/* References:
 *   "Atmel SAM L21E / SAM L21G / SAM L21J Smart ARM-Based Microcontroller
 *   Datasheet", Atmel-42385C-SAML21_Datasheet_Preliminary-03/20/15
 */

#ifndef __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAML_WDT_H
#define __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAML_WDT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

#ifdef CONFIG_ARCH_FAMILY_SAML21

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* WDT register offsets *****************************************************/

#define SAM_WDT_CTRLA_OFFSET       0x0000  /* Control A register */
#define SAM_WDT_CONFIG_OFFSET      0x0001  /* Configuration register */
#define SAM_WDT_EWCTRL_OFFSET      0x0002  /* Early warning interrupt control register */
#define SAM_WDT_INTENCLR_OFFSET    0x0004  /* Interrupt enable clear register */
#define SAM_WDT_INTENSET_OFFSET    0x0005  /* Interrupt enable set register */
#define SAM_WDT_INTFLAG_OFFSET     0x0006  /* Interrupt flag and status clear register */
#define SAM_WDT_SYNCBUSY_OFFSET    0x0008  /* Synchronization busy register */
#define SAM_WDT_CLEAR_OFFSET       0x000c  /* Clear register */

/* WDT register addresses ***************************************************/

#define SAM_WDT_CTRLA              (SAM_WDT_BASE+SAM_WDT_CTRLA_OFFSET)
#define SAM_WDT_CONFIG             (SAM_WDT_BASE+SAM_WDT_CONFIG_OFFSET)
#define SAM_WDT_EWCTRL             (SAM_WDT_BASE+SAM_WDT_EWCTRL_OFFSET)
#define SAM_WDT_INTENCLR           (SAM_WDT_BASE+SAM_WDT_INTENCLR_OFFSET)
#define SAM_WDT_INTENSET           (SAM_WDT_BASE+SAM_WDT_INTENSET_OFFSET)
#define SAM_WDT_INTFLAG            (SAM_WDT_BASE+SAM_WDT_INTFLAG_OFFSET)
#define SAM_WDT_SYNCBUSY           (SAM_WDT_BASE+SAM_WDT_SYNCBUSY_OFFSET)
#define SAM_WDT_CLEAR              (SAM_WDT_BASE+SAM_WDT_CLEAR_OFFSET)

/* WDT register bit definitions *********************************************/

/* Control register */

#define WDT_CTRLA_ENABLE            (1 << 1)  /* Bit 1:  Enable */
#define WDT_CTRLA_WEN               (1 << 2)  /* Bit 2:  Watchdog Timer Window Mode Enable */
#define WDT_CTRLA_ALWAYSON          (1 << 7)  /* Bit 7:  Always-On */

/* Configuration register */

#define WDT_CONFIG_PER_SHIFT       (0)       /* Bits 0-3: Time-Out Period */
#define WDT_CONFIG_PER_MASK        (15 << WDT_CONFIG_PER_SHIFT)
#  define WDT_CONFIG_PER_8         (0 << WDT_CONFIG_PER_SHIFT)  /* 8 clock cycles */
#  define WDT_CONFIG_PER_16        (1 << WDT_CONFIG_PER_SHIFT)  /* 16 clock cycles */
#  define WDT_CONFIG_PER_32        (2 << WDT_CONFIG_PER_SHIFT)  /* 32 clock cycles */
#  define WDT_CONFIG_PER_64        (3 << WDT_CONFIG_PER_SHIFT)  /* 64 clock cycles */
#  define WDT_CONFIG_PER_128       (4 << WDT_CONFIG_PER_SHIFT)  /* 128 clock cycles */
#  define WDT_CONFIG_PER_256       (5 << WDT_CONFIG_PER_SHIFT)  /* 256 clocks cycles */
#  define WDT_CONFIG_PER_512       (6 << WDT_CONFIG_PER_SHIFT)  /* 512 clocks cycles */
#  define WDT_CONFIG_PER_1K        (7 << WDT_CONFIG_PER_SHIFT)  /* 1024 clock cycles */
#  define WDT_CONFIG_PER_2K        (8 << WDT_CONFIG_PER_SHIFT)  /* 2048 clock cycles */
#  define WDT_CONFIG_PER_4K        (9 << WDT_CONFIG_PER_SHIFT)  /* 4096 clock cycles */
#  define WDT_CONFIG_PER_8k        (10 << WDT_CONFIG_PER_SHIFT) /* 8192 clock cycles */
#  define WDT_CONFIG_PER_16K       (11 << WDT_CONFIG_PER_SHIFT) /* 16384 clock cycles */

#define WDT_CONFIG_WINDOW_SHIFT    (4)       /* Bits 4-7: Window Mode Time-Out Period */
#define WDT_CONFIG_WINDOW_MASK     (15 << WDT_CONFIG_WINDOW_SHIFT)
#  define WDT_CONFIG_WINDOW_8      (0 << WDT_CONFIG_WINDOW_SHIFT)  /* 8 clock cycles */
#  define WDT_CONFIG_WINDOW_16     (1 << WDT_CONFIG_WINDOW_SHIFT)  /* 16 clock cycles */
#  define WDT_CONFIG_WINDOW_32     (2 << WDT_CONFIG_WINDOW_SHIFT)  /* 32 clock cycles */
#  define WDT_CONFIG_WINDOW_64     (3 << WDT_CONFIG_WINDOW_SHIFT)  /* 64 clock cycles */
#  define WDT_CONFIG_WINDOW_128    (4 << WDT_CONFIG_WINDOW_SHIFT)  /* 128 clock cycles */
#  define WDT_CONFIG_WINDOW_256    (5 << WDT_CONFIG_WINDOW_SHIFT)  /* 256 clocks cycles */
#  define WDT_CONFIG_WINDOW_512    (6 << WDT_CONFIG_WINDOW_SHIFT)  /* 512 clocks cycles */
#  define WDT_CONFIG_WINDOW_1K     (7 << WDT_CONFIG_WINDOW_SHIFT)  /* 1024 clock cycles */
#  define WDT_CONFIG_WINDOW_2K     (8 << WDT_CONFIG_WINDOW_SHIFT)  /* 2048 clock cycles */
#  define WDT_CONFIG_WINDOW_4K     (9 << WDT_CONFIG_WINDOW_SHIFT)  /* 4096 clock cycles */
#  define WDT_CONFIG_WINDOW_8k     (10 << WDT_CONFIG_WINDOW_SHIFT) /* 8192 clock cycles */
#  define WDT_CONFIG_WINDOW_16K    (11 << WDT_CONFIG_WINDOW_SHIFT) /* 16384 clock cycles */

/* Early warning interrupt control register */

#define WDT_EWCTRL_EWOFFSET_SHIFT  (0)       /* Bits 0-3: Early warning interrupt time offset */
#define WDT_EWCTRL_EWOFFSET_MASK   (15 << WDT_EWCTRL_EWOFFSET_SHIFT)
#  define WDT_EWCTRL_EWOFFSET_8    (0 << WDT_EWCTRL_EWOFFSET_SHIFT)  /* 8 clock cycles */
#  define WDT_EWCTRL_EWOFFSET_16   (1 << WDT_EWCTRL_EWOFFSET_SHIFT)  /* 16 clock cycles */
#  define WDT_EWCTRL_EWOFFSET_32   (2 << WDT_EWCTRL_EWOFFSET_SHIFT)  /* 32 clock cycles */
#  define WDT_EWCTRL_EWOFFSET_64   (3 << WDT_EWCTRL_EWOFFSET_SHIFT)  /* 64 clock cycles */
#  define WDT_EWCTRL_EWOFFSET_128  (4 << WDT_EWCTRL_EWOFFSET_SHIFT)  /* 128 clock cycles */
#  define WDT_EWCTRL_EWOFFSET_256  (5 << WDT_EWCTRL_EWOFFSET_SHIFT)  /* 256 clocks cycles */
#  define WDT_EWCTRL_EWOFFSET_512  (6 << WDT_EWCTRL_EWOFFSET_SHIFT)  /* 512 clocks cycles */
#  define WDT_EWCTRL_EWOFFSET_1K   (7 << WDT_EWCTRL_EWOFFSET_SHIFT)  /* 1024 clock cycles */
#  define WDT_EWCTRL_EWOFFSET_2K   (8 << WDT_EWCTRL_EWOFFSET_SHIFT)  /* 2048 clock cycles */
#  define WDT_EWCTRL_EWOFFSET_4K   (9 << WDT_EWCTRL_EWOFFSET_SHIFT)  /* 4096 clock cycles */
#  define WDT_EWCTRL_EWOFFSET_8k   (10 << WDT_EWCTRL_EWOFFSET_SHIFT) /* 8192 clock cycles */
#  define WDT_EWCTRL_EWOFFSET_16K  (11 << WDT_EWCTRL_EWOFFSET_SHIFT) /* 16384 clock cycles */

/* Interrupt enable clear, interrupt enable set register, interrupt
 * flag status and clear registers
 */

#define WDT_INT_EW                 (1 << 0)  /* Bit 0:  Early warning interrupt */
#define WDT_INT_All                (0x01)

/* Synchronization busy register */

#define WDT_SYNCBUSY_ENABLE        (1 << 1)  /* Bit 1:  Enable syncrhonization busy */
#define WDT_SYNCBUSY_WEN           (1 << 2)  /* Bit 2:  Window enable synchronization busy */
#define WDT_SYNCBUSY_ALWAYSON      (1 << 3)  /* Bit 3:  Always-on synchronization busy */
#define WDT_SYNCBUSY_CLEAR         (1 << 4)  /* Bit 4:  Clear syncrhonization busy */

/* Clear register */

#define WDT_CLEAR_CLEAR_SHIFT      (0)       /* Bits 0-7: Watchdog clear */
#define WDT_CLEAR_CLEAR_MASK       (0xff << WDT_CLEAR_CLEAR_SHIFT)
#  define WDT_CLEAR_CLEAR          (0xa5 << WDT_CLEAR_CLEAR_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* CONFIG_ARCH_FAMILY_SAML21 */
#endif /* __ARCH_ARM_SRC_SAMD2L2_HARDWARE_SAML_WDT_H */
