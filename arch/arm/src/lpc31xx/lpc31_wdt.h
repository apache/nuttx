/****************************************************************************
 * arch/arm/src/lpc31xx/lpc31_wdt.h
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

#ifndef __ARCH_ARM_SRC_LPC31XX_LPC31_WDT_H
#define __ARCH_ARM_SRC_LPC31XX_LPC31_WDT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "lpc31_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* WDT register base address offset into the APB0 domain ********************/

#define LPC31_WDT_VBASE                (LPC31_APB0_VADDR+LPC31_APB0_WDT_OFFSET)
#define LPC31_WDT_PBASE                (LPC31_APB0_PADDR+LPC31_APB0_WDT_OFFSET)

/* WDT register offsets (with respect to the WDT base) **********************/

#define LPC31_WDT_IR_OFFSET            0x000 /* Interrupt Register */
#define LPC31_WDT_TCR_OFFSET           0x004 /* Timer Control Register */
#define LPC31_WDT_TC_OFFSET            0x008 /* Timer Counter */
#define LPC31_WDT_PR_OFFSET            0x00c /* Timer Prescale Register */
#define LPC31_WDT_PC_OFFSET            0x010 /* Prescale Counter */
#define LPC31_WDT_MCR_OFFSET           0x014 /* Match Control Register */
#define LPC31_WDT_MR0_OFFSET           0x018 /* Match Register 0 */
#define LPC31_WDT_MR1_OFFSET           0x01c /* Match Register 1 */
                                             /* 0x020-0x038: Reserved */
#define LPC31_WDT_EMR_OFFSET           0x03c /* External Match Register */

/* WDT register (virtual) addresses *****************************************/

#define LPC31_WDT_IR                   (LPC31_WDT_VBASE+LPC31_WDT_IR_OFFSET)
#define LPC31_WDT_TCR                  (LPC31_WDT_VBASE+LPC31_WDT_TCR_OFFSET)
#define LPC31_WDT_TC                   (LPC31_WDT_VBASE+LPC31_WDT_TC_OFFSET)
#define LPC31_WDT_PR                   (LPC31_WDT_VBASE+LPC31_WDT_PR_OFFSET)
#define LPC31_WDT_PC                   (LPC31_WDT_VBASE+LPC31_WDT_PC_OFFSET)
#define LPC31_WDT_MCR                  (LPC31_WDT_VBASE+LPC31_WDT_MCR_OFFSET)
#define LPC31_WDT_MR0                  (LPC31_WDT_VBASE+LPC31_WDT_MR0_OFFSET)
#define LPC31_WDT_MR1                  (LPC31_WDT_VBASE+LPC31_WDT_MR1_OFFSET)
#define LPC31_WDT_EMR                  (LPC31_WDT_VBASE+LPC31_WDT_EMR_OFFSET)

/* WDT register bit definitions *********************************************/

/* Interrupt Register (IR), address 0x13002400 */

#define WDT_IR_INTRM1                    (1 << 1)  /* Bit 1:  MR1 and TC match interrupt */
#define WDT_IR_INTRM0                    (1 << 0)  /* Bit 0:  MR0 and TC match interrupt */

/* Timer Control Register (TCR), address 0x13002404 */

#define WDT_TCR_RESET                    (1 << 1)  /* Bit 1:  Reset on the next WDOG_PCLK */
#define WDT_TCR_ENABLE                   (1 << 0)  /* Bit 0:  Enable */

/* Match Control Register (MCR), address 0x1300 2414 */

#define WDT_MCR_MR1STOP                  (1 << 5)  /* Bit 5:  Stop counting when MR1=TC */
#define WDT_MCR_MR1RESET                 (1 << 4)  /* Bit 4:  Reset TC if MR1=TC */
#define WDT_MCR_MR1INT                   (1 << 3)  /* Bit 3:  System reset when MR1=TC */
#define WDT_MCR_MR0STOP                  (1 << 2)  /* Bit 2:  Stop counting when MR0=TC */
#define WDT_MCR_MR0RESET                 (1 << 1)  /* Bit 1:  Reset TC if MR0=TC */
#define WDT_MCR_MR0INT                   (1 << 0)  /* Bit 0:  System reset when MR0=TC */

/* External Match Registers (EMR), address 0x1300 243c */

#define WDT_EMR_EXTMATCHCTRL1_SHIFT      (6)       /* Bits 6-7: Controls EXTMATCH1 when MR1=TC */
#define WDT_EMR_EXTMATCHCTRL1_MASK       (3 << WDT_EMR_EXTMATCHCTRL1_SHIFT)
#  define WDT_EMR_EXTMATCHCTRL1_NOTHING  (0 << WDT_EMR_EXTMATCHCTRL1_SHIFT) /* Do Nothing */
#  define WDT_EMR_EXTMATCHCTRL1_SETLOW   (1 << WDT_EMR_EXTMATCHCTRL1_SHIFT) /* Set LOW */
#  define WDT_EMR_EXTMATCHCTRL1_SETHIGH  (2 << WDT_EMR_EXTMATCHCTRL1_SHIFT) /* Set HIGH */
#  define WDT_EMR_EXTMATCHCTRL1_TOGGLE   (3 << WDT_EMR_EXTMATCHCTRL1_SHIFT) /* Toggle */

#define WDT_EMR_EXTMATCHCTRL0_SHIFT      (4)       /* Bits 4-5: Controls EXTMATCH0 when MR0=TC */
#define WDT_EMR_EXTMATCHCTRL0_MASK       (3 << WDT_EMR_EXTMATCHCTRL0_SHIFT)
#  define WDT_EMR_EXTMATCHCTRL0_NOTHING  (0 << WDT_EMR_EXTMATCHCTRL0_SHIFT) /* Do Nothing */
#  define WDT_EMR_EXTMATCHCTRL0_SETLOW   (1 << WDT_EMR_EXTMATCHCTRL0_SHIFT) /* Set LOW */
#  define WDT_EMR_EXTMATCHCTRL0_SETHIGH  (2 << WDT_EMR_EXTMATCHCTRL0_SHIFT) /* Set HIGH */
#  define WDT_EMR_EXTMATCHCTRL0_TOGGLE   (3 << WDT_EMR_EXTMATCHCTRL0_SHIFT) /* Toggle */

#define WDT_EMR_EXTMATCH1                (1 << 1)  /* Bit 1:  EXTMATCHCTRL1 controls behavior */
#define WDT_EMR_EXTMATCH0                (1 << 0)  /* Bit 0:  EXTMATCHCTRL1 controls behavior */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC31XX_LPC31_WDT_H */
