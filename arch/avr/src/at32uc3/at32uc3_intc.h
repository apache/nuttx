/****************************************************************************
 * arch/avr/src/at32uc3/at32uc3_intc.h
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

#ifndef __ARCH_AVR_SRC_AT32UC3_AT32UC3_INTC_H
#define __ARCH_AVR_SRC_AT32UC3_AT32UC3_INTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define AVR32_INTC_IPR_OFFSET(n)  ((n) << 2)           /* Interrupt priority registers */
#define AVR32_INTC_IRR_OFFSET(n)  (0x100 + ((n) << 2)) /* Interrupt request registers */
#define AVR32_INTC_ICR_OFFSET(n)  (0x20c - ((n) << 2)) /* Interrupt cause registers */

/* Register Addresses *******************************************************/

#define AVR32_INTC_IPR(n)         (AVR32_INTC_BASE+AVR32_INTC_IPR_OFFSET(n))
#define AVR32_INTC_IRR(n)         (AVR32_INTC_BASE+AVR32_INTC_IRR_OFFSET(n))
#define AVR32_INTC_ICR(n)         (AVR32_INTC_BASE+AVR32_INTC_ICR_OFFSET(n))

/* Register Bit-field Definitions *******************************************/

/* Interrupt priority register bit-field definitions */

#define INTC_IPR_AUTOVECTOR_SHIFT (0)       /* Bits 0-13: Autovector address */
#define INTC_IPR_AUTOVECTOR_MASK  (0x3fff << INTC_IPR_AUTOVECTOR_SHIFT)
#define INTC_IPR_INTLEVEL_SHIFT   (30)      /* Bits 30-31: Interrupt Level */
#define INTC_IPR_INTLEVEL_MASK    (3 << INTC_IPR_INTLEVEL_SHIFT)
#  define INTC_IPR_INTLEVEL_INT0  (0 << INTC_IPR_INTLEVEL_SHIFT) /* Lowest priority */
#  define INTC_IPR_INTLEVEL_INT1  (1 << INTC_IPR_INTLEVEL_SHIFT)
#  define INTC_IPR_INTLEVEL_INT2  (2 << INTC_IPR_INTLEVEL_SHIFT)
#  define INTC_IPR_INTLEVEL_INT3  (3 << INTC_IPR_INTLEVEL_SHIFT) /* Highest priority */

/* Interrupt request register bit-field definitions */

#define INTC_IRR_REG(n)           (AVR32_INTC_IPR((n) >> 5))
#define INTC_IRR_SHIFT(n)         (1 << ((n) & 0x1f))
#define INTC_IRR_MASK(n)          (1 << NTC_IRR_SHIFT(n))

/* Interrupt cause register bit-field definitions */

#define INTC_ICR_CAUSE_SHIFT      (0)       /* Bits 0-5: Interrupt Group Causing Interrupt of Priority n */
#define INTC_ICR_CAUSE_MASK       (0x3f << INTC_ICR_CAUSE_SHIFT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_AVR_SRC_AT32UC3_AT32UC3_INTC_H */
