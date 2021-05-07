/****************************************************************************
 * arch/arm/src/sam34/hardware/sam4l_wdt.h
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

#ifndef __ARCH_ARM_SRC_SAM34_HARDWARE_SAM4L_WDT_H
#define __ARCH_ARM_SRC_SAM34_HARDWARE_SAM4L_WDT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/sam_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* WDT register offsets *****************************************************/

#define SAM_WDT_CTRL_OFFSET        0x0000 /* Control Register */
#define SAM_WDT_CLR_OFFSET         0x0004 /* Clear Register */
#define SAM_WDT_SR_OFFSET          0x0008 /* Status Register */
#define SAM_WDT_IER_OFFSET         0x000c /* Interrupt Enable Register */
#define SAM_WDT_IDR_OFFSET         0x0010 /* Interrupt Disable Register */
#define SAM_WDT_IMR_OFFSET         0x0014 /* Interrupt Mask Register */
#define SAM_WDT_ISR_OFFSET         0x0018 /* Interrupt Status Register */
#define SAM_WDT_ICR_OFFSET         0x001c /* Interrupt Clear Register */
#define SAM_WDT_VERSION_OFFSET     0x03fc /* Version Register */

/* WDT register addresses ***************************************************/

#define SAM_WDT_CTRL               (SAM_WDT_BASE+SAM_WDT_CTRL_OFFSET)
#define SAM_WDT_CLR                (SAM_WDT_BASE+SAM_WDT_CLR_OFFSET)
#define SAM_WDT_SR                 (SAM_WDT_BASE+SAM_WDT_SR_OFFSET)
#define SAM_WDT_IER                (SAM_WDT_BASE+SAM_WDT_IER_OFFSET)
#define SAM_WDT_IDR                (SAM_WDT_BASE+SAM_WDT_IDR_OFFSET)
#define SAM_WDT_IMR                (SAM_WDT_BASE+SAM_WDT_IMR_OFFSET)
#define SAM_WDT_ISR                (SAM_WDT_BASE+SAM_WDT_ISR_OFFSET)
#define SAM_WDT_ICR                (SAM_WDT_BASE+SAM_WDT_ICR_OFFSET)
#define SAM_WDT_VERSION            (SAM_WDT_BASE+SAM_WDT_VERSION_OFFSET)

/* WDT register bit definitions *********************************************/

/* Control Register */

#define WDT_CTRL_EN                (1 << 0)  /* Bit 0:  WDT Enable */
#define WDT_CTRL_DAR               (1 << 1)  /* Bit 1:  WDT Disable After Reset */
#define WDT_CTRL_MODE              (1 << 2)  /* Bit 2:  WDT Mode */
#define WDT_CTRL_SFV               (1 << 3)  /* Bit 3:  WDT Control Register Store Final Value */
#define WDT_CTRL_IM                (1 << 4)  /* Bit 4:  Interrupt Mode */
#define WDT_CTRL_FCD               (1 << 7)  /* Bit 7:  Flash Calibration Done */
#define WDT_CTRL_PSEL_SHIFT        (8)       /* Bits 8-12: Time Out Prescale Select */
#define WDT_CTRL_PSEL_MASK         (31 << WDT_CTRL_PSEL_SHIFT)
#define WDT_CTRL_CEN               (1 << 16) /* Bit 16: Clock Enable */
#define WDT_CTRL_CSSEL             (1 << 17) /* Bit 17: Clock Source Select */
#define WDT_CTRL_TBAN_SHIFT        (18)      /* Bits 18-22: Time Ban Prescale Select */
#define WDT_CTRL_TBAN_MASK         (31 << WDT_CTRL_TBAN_SHIFT)
#define WDT_CTRL_KEY_SHIFT         (24)      /* Bits 24-31: Key */
#define WDT_CTRL_KEY_MASK          (0xff << WDT_CTRL_KEY_SHIFT)
#  define WDT_CTRL_KEY_FIRST       (0x55 << WDT_CTRL_KEY_SHIFT)
#  define WDT_CTRL_KEY_SECOND      (0xaa << WDT_CTRL_KEY_SHIFT)

/* Clear Register */

#define WDT_CLR_WDTCLR             (1 << 0) /* Bit 0: Watchdog Clear */
#define WDT_CLR_KEY_SHIFT          (24)     /* Bits 24-31: Key */
#define WDT_CLR_KEY_MASK           (0xff << WDT_CLR_KEY_SHIFT)
#  define WDT_CLR_KEY_FIRST        (0x55 << WDT_CLR_KEY_SHIFT)
#  define WDT_CLR_KEY_SECOND       (0xaa << WDT_CLR_KEY_SHIFT)

/* Status Register */

#define WDT_SR_WINDOW              (1 << 0) /* Bit 0:  Within Window */
#define WDT_SR_CLEARED             (1 << 1) /* Bit 1:  WDT Counter Cleared */

/* Interrupt Enable Register */

/* Interrupt Disable Register */

/* Interrupt Mask Register */

/* Interrupt Status Register */

/* Interrupt Clear Register */

#define WDT_WINT                   (1 << 2) /* Bit 2: WINT */

/* Version Register */

#define WDT_VERSION_SHIFT          (0)        /* Bits 0-11: Version Number */
#define WDT_VERSION_MASK           (0xfff << WDT_VERSION_VERSION_SHIFT)
#define WDT_VARIANT_SHIFT          (16)       /* Bits 16-19: Variant Number */
#define WDT_VARIANT_MASK           (15 << WDT_VARIANT_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM34_HARDWARE_SAM4L_WDT_H */
