/****************************************************************************
 * arch/hc/src/m9s12/m9s12_int.h
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

#ifndef __ARCH_HC_SRC_M9S12_M9S12_INT_H
#define __ARCH_HC_SRC_M9S12_M9S12_INT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

/* Offsets relative to CORE1 */

#define HCS12_INT_ITCR_OFFSET          0x0015 /* Interrupt Test Control Register */
#define HCS12_INT_ITEST_OFFSET         0x0016 /* Interrupt Test Registers */

/* Offsets relative to CORE2 */

#define HCS12_INT_HPRIO_OFFSET         0x0003 /* Highest Priority Interrupt */

/* Register Addresses *******************************************************/

#define HCS12_INT_ITCR                 (HCS12_REG_BASE+HCS12_CORE1_BASE+HCS12_INT_ITCR_OFFSET)
#define HCS12_INT_ITEST                (HCS12_REG_BASE+HCS12_CORE1_BASE+HCS12_INT_ITEST_OFFSET)
#define HCS12_INT_HPRIO                (HCS12_REG_BASE+HCS12_CORE2_BASE+HCS12_INT_HPRIO_OFFSET)

/* Register Bit-Field Definitions *******************************************/

/* Interrupt Test Control Register Bit-Field Definitions */

#define INT_ITCR_ADR_SHIFT             (0)       /* Bits 0-3: Test register select */
#define INT_ITCR_ADR_MASK              (15 << INT_ITCR_ADR_SHIFT)
#define INT_ITCR_WRTINT                (1 << 4)  /* Bit 4:  Write to the Interrupt Test Registers */

/* Interrupt Test Registers Bit-Field Definitions */

#define INT_ITEST_INT(n)               (1 << ((n) >> 1))
#define INT_ITEST_INT0                 (1 << 0)  /* Bit 0: Test vector 0xffx0 */
#define INT_ITEST_INT2                 (1 << 1)  /* Bit 1: Test vector 0xffx2 */
#define INT_ITEST_INT4                 (1 << 2)  /* Bit 2: Test vector 0xffx4 */
#define INT_ITEST_INT6                 (1 << 3)  /* Bit 3: Test vector 0xffx6 */
#define INT_ITEST_INT8                 (1 << 4)  /* Bit 4: Test vector 0xffx8 */
#define INT_ITEST_INTA                 (1 << 5)  /* Bit 5: Test vector 0xffxa */
#define INT_ITEST_INTC                 (1 << 6)  /* Bit 6: Test vector 0xffxc */
#define INT_ITEST_INTE                 (1 << 7)  /* Bit 7: Test vector 0xffxe */

/* Highest Priority Interrupt Bit-Field Definitions */

/* Holds the least of the highest priority interrupt vector address */

#define INT_HPRIO_MASK                 (0xfe)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_HC_SRC_M9S12_M9S12_INT_H */
