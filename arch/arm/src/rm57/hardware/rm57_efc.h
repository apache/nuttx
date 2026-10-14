/****************************************************************************
 * arch/arm/src/rm57/hardware/rm57_efc.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

/* eFuse Controller (EFC), SPNU562A chapter 37.
 *
 * The eFuses are loaded and ECC-checked by hardware during power-on reset;
 * software only inspects the outcome and exercises the error paths.
 * Offsets follow TRM Table 37-2 and match HALCoGen's HL_reg_efc.h.
 */

#ifndef __ARCH_ARM_SRC_RM57_HARDWARE_RM57_EFC_H
#define __ARCH_ARM_SRC_RM57_HARDWARE_RM57_EFC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/rm57l843_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define RM57_EFC_BOUND_OFFSET       0x001c /* Boundary Control Register */
#define RM57_EFC_PINS_OFFSET        0x002c /* Pins Register */
#define RM57_EFC_ERRSTAT_OFFSET     0x003c /* Error Status Register */
#define RM57_EFC_STCY_OFFSET        0x0048 /* Self Test Cycles Register */
#define RM57_EFC_STSIG_OFFSET       0x004c /* Self Test Signature Register */

/* Register Addresses *******************************************************/

#define RM57_EFC_BOUND              (RM57_EFC_BASE + RM57_EFC_BOUND_OFFSET)
#define RM57_EFC_PINS               (RM57_EFC_BASE + RM57_EFC_PINS_OFFSET)
#define RM57_EFC_ERRSTAT            (RM57_EFC_BASE + RM57_EFC_ERRSTAT_OFFSET)
#define RM57_EFC_STCY               (RM57_EFC_BASE + RM57_EFC_STCY_OFFSET)
#define RM57_EFC_STSIG              (RM57_EFC_BASE + RM57_EFC_STSIG_OFFSET)

/* Register Bit-Field Definitions *******************************************/

/* Boundary Control Register */

/* Bits 0-3: Input enable key */
#define EFC_BOUND_INPUT_ENABLE      (0x0f << 0)

/* Bit 13: Start ECC self-test */
#define EFC_BOUND_SELFTEST_ENABLE   (1 << 13)

/* Bit 14: Drive autoload error */
#define EFC_BOUND_AUTOLOAD_OE       (1 << 14)

/* Bit 15: Drive instruction error */
#define EFC_BOUND_INSTR_OE          (1 << 15)

/* Bit 16: Drive single-bit error */
#define EFC_BOUND_SINGLEBIT_OE      (1 << 16)

/* Bit 17: Drive self-test error */
#define EFC_BOUND_SELFTEST_OE       (1 << 17)

/* Bit 18: ESM group3 channel 1 */
#define EFC_BOUND_AUTOLOAD_ERR      (1 << 18)
#define EFC_BOUND_INSTR_ERR         (1 << 19) /* Bit 19: Not routed to ESM */

/* Bit 20: ESM group1 channel 40 */
#define EFC_BOUND_SINGLEBIT_ERR     (1 << 20)

/* Bit 21: ESM group1 channel 41 */
#define EFC_BOUND_SELFTEST_ERR      (1 << 21)

#define EFC_BOUND_OE_ALL            (0x0f << 14)

/* Values prescribed by TRM 37.3.2.4 / 37.3.2.5 */

/* All four errors, nERROR low */
#define EFC_BOUND_STUCKZERO_A       (0x003fc000)

/* Without the autoload error */
#define EFC_BOUND_STUCKZERO_B       (0x003bc000)

/* Drive all four low again */
#define EFC_BOUND_STUCKZERO_CLEAR   (0x003c0000)
#define EFC_BOUND_SELFTEST_START    (0x0000200f)

/* Pins Register */

/* Bit 10: Uncorrectable autoload */
#define EFC_PINS_AUTOLOAD_ERR       (1 << 10)
#define EFC_PINS_INSTR_ERR          (1 << 11) /* Bit 11: Instruction error */

/* Bit 12: Corrected single bit */
#define EFC_PINS_SINGLEBIT_ERR      (1 << 12)

/* Bit 14: ECC self-test failed */
#define EFC_PINS_SELFTEST_ERR       (1 << 14)
#define EFC_PINS_SELFTEST_DONE      (1 << 15) /* Bit 15: ECC self-test done */

/* Error Status Register */

#define EFC_ERRSTAT_CODE_MASK       (0x1f << 0) /* Bits 0-4: Error code */
#  define EFC_ERRSTAT_CODE_NONE     (0x00)

/* Uncorrectable autoload error */
#  define EFC_ERRSTAT_CODE_MULTIBIT (0x05)
#  define EFC_ERRSTAT_CODE_SINGLEBIT (0x15)   /* Corrected single-bit error */
#define EFC_ERRSTAT_INSTR_DONE      (1 << 5)  /* Bit 5: Instruction done */

/* ECC self-test parameters (TRM 37.3.2.5) */

#define EFC_STCY_VALUE              (0x00000258)
#define EFC_STSIG_VALUE             (0x5362f97f)

#endif /* __ARCH_ARM_SRC_RM57_HARDWARE_RM57_EFC_H */
