/****************************************************************************
 * arch/arm/src/lpc31xx/lpc31_pwm.h
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

#ifndef __ARCH_ARM_SRC_LPC31XX_LPC31_PWM_H
#define __ARCH_ARM_SRC_LPC31XX_LPC31_PWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "lpc31_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PWM register base address offset into the APB1 domain ********************/

#define LPC31_PWM_VBASE                (LPC31_APB1_VADDR+LPC31_APB1_PWM_OFFSET)
#define LPC31_PWM_PBASE                (LPC31_APB1_PADDR+LPC31_APB1_PWM_OFFSET)

/* PWM register offsets (with respect to the PWM base) **********************/

#define LPC31_PWM_TMR_OFFSET           0x000 /* Timer Register */
#define LPC31_PWM_CNTL_OFFSET          0x004 /* Control Register */

/* PWM register (virtual) addresses *****************************************/

#define LPC31_PWM_TMR                 (LPC31_PWM_VBASE+LPC31_PWM_TMR_OFFSET)
#define LPC31_PWM_CNTL                (LPC31_PWM_VBASE+LPC31_PWM_CNTL_OFFSET)

/* PWM register bit definitions *********************************************/

/* Timer register TMR, address 0x13009000 */

#define PWM_TMR_SHIFT                   (0)    /* Bits 0-11: Timer used for PWM and PDM */
#define PWM_TMR_MASK                    (0xfff << PWM_TMR_SHIFT)

/* Control register CNTL, address 0x13009004 */

#define PWM_CNTL_PDM                    (1 << 7)  /* Bit 7:  PDM Select PDM mode */
#define PWM_CNTL_LOOP                   (1 << 6)  /* Bit 6:  Output inverted with top 4 TMR bits */
#define PWM_CNTL_HI                     (1 << 4)  /* Bit 4:  PWM output forced high */
                                                  /* Bits 2-3: Reserved */
#define PWM_CNTL_CLK_SHIFT              (0)       /* Bits 0-1: Configure PWM_CLK for output pulses */
#define PWM_CNTL_CLK_MASK               (3 << PWM_CNTL_CLK_SHIFT)
#  define PWM_CNTL_CLKDIV1              (0 << PWM_CNTL_CLK_SHIFT) /* PWM_CLK */
#  define PWM_CNTL_CLKDIV2              (1 << PWM_CNTL_CLK_SHIFT) /* PWM_CLK/2 */
#  define PWM_CNTL_CLKDIV4              (2 << PWM_CNTL_CLK_SHIFT) /* PWM_CLK/4 */
#  define PWM_CNTL_CLKDIV8              (3 << PWM_CNTL_CLK_SHIFT) /* PWM_CLK/8 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC31XX_LPC31_PWM_H */
