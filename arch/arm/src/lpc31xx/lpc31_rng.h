/****************************************************************************
 * arch/arm/src/lpc31xx/lpc31_rng.h
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

#ifndef __ARCH_ARM_SRC_LPC31XX_LPC31_RNG_H
#define __ARCH_ARM_SRC_LPC31XX_LPC31_RNG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "lpc31_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RNG register base address offset into the APB0 domain ********************/

#define LPC31_RNG_VBASE                (LPC31_APB0_VADDR+LPC31_APB0_RNG_OFFSET)
#define LPC31_RNG_PBASE                (LPC31_APB0_PADDR+LPC31_APB0_RNG_OFFSET)

/* RNG register offsets (with respect to the RNG base) **********************/

#define LPC31_RNG_RAND_OFFSET          0x0000 /* Random number */
#define LPC31_RNG_PWRDWN_OFFSET        0x0ff4 /* Power-down mode */

/* RNG register (virtual) addresses *****************************************/

#define LPC31_RNG_RAND                 (LPC31_RNG_VBASE+LPC31_RNG_RAND_OFFSET)
#define LPC31_RNG_PWRDWN               (LPC31_RNG_VBASE+LPC31_RNG_PWRDWN_OFFSET)

/* RNG register bit definitions *********************************************/

/* POWERDOWN, address 0x13006ff4 */

#define RNG_PWRDWN_PWRDWN                (1 << 2)  /* Block all accesses to standard registers */
#define RNG_PWRDWN_FORCERST              (1 << 1)  /* With SOFTRST forces immediate RNG reset */
#define RNG_PWRDWN_SOFTRST               (1 << 0)  /* Software RNG reset (delayed) */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC31XX_LPC31_RNG_H */
